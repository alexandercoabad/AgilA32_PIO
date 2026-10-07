"""can_frame.py -- CAN 2.0A/B frame helpers for pio/can_tx.pio and pio/can_rx.pio.

Everything the PIO cannot do (it has no XOR): CRC-15, bit stuffing and destuffing, frame field parsing, packing the
stuffed bit stream into TX FIFO words, decoding the raw bit capture of the RX state machine, and decoding the result
words of the TX state machine.  Pure Python, no simulator; shared by the cocotb tests, the mutation sweep and the
firmware builder.

Bits are Python lists of 0/1 in transmission order; 0 = dominant, 1 = recessive.
"""

CRC15_POLY = 0x4599                     # x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1

TAIL_BITS = 12                          # the PIO appends: ACK slot + ACK delimiter + 7 EOF + 3 intermission
ABORT_TAG = 0xFFFFFF                    # upper 24 bits of an abort word / of an RX tag word

# forced-instruction images used when the host restarts a state machine (see pio/can_tx.pio, pio/can_rx.pio)
SET_PINS_1 = 0xE001                     # set pins, 1      (TXD recessive before the PIO owns the pin)
MOV_ISR_NULL = 0xA0C3                   # mov isr, null
MOV_X_NOT_NULL = 0xA02B                 # mov x, ~null


def crc15(bits):
    """CRC-15/CAN over a bit list (SOF .. last data bit), bit-serial like the shift register in the standard."""
    crc = 0
    for b in bits:
        top = ((crc >> 14) & 1) ^ (b & 1)
        crc = (crc << 1) & 0x7FFF
        if top:
            crc ^= CRC15_POLY
    return crc


def _field(value, n):
    return [(value >> (n - 1 - i)) & 1 for i in range(n)]


def frame_bits(ident, data=b"", rtr=False, ext=False, dlc=None):
    """SOF .. CRC sequence of a frame (not stuffed, no CRC delimiter).  ident: 11 bit, or 29 bit when ext.
    A remote frame (rtr) has no data field; its DLC defaults to 0."""
    data = bytes(data)
    if rtr:
        assert not data
    else:
        assert len(data) <= 8
    if dlc is None:
        dlc = len(data)
    assert 0 <= dlc <= 15
    if not rtr:
        assert dlc == len(data) or (dlc > 8 and len(data) == 8)
    bits = [0]                                                    # SOF
    if ext:
        assert 0 <= ident < (1 << 29)
        bits += _field(ident >> 18, 11) + [1, 1]                  # base ID, SRR, IDE
        bits += _field(ident & 0x3FFFF, 18) + [1 if rtr else 0, 0, 0]   # ID ext, RTR, r1, r0
    else:
        assert 0 <= ident < (1 << 11)
        bits += _field(ident, 11) + [1 if rtr else 0, 0, 0]       # ID, RTR, IDE, r0
    bits += _field(dlc, 4)
    for byte in data:
        bits += _field(byte, 8)
    return bits + _field(crc15(bits), 15)


def stuff(bits):
    """Insert a complementary bit after every five equal bits (the inserted bit counts for the next run)."""
    out, run, last = [], 0, None
    for b in bits:
        out.append(b)
        run = run + 1 if b == last else 1
        last = b
        if run == 5:
            out.append(1 - b)
            last, run = 1 - b, 1
    return out


def destuff(bits):
    """Inverse of stuff().  Returns (data_bits, error_index or None); error_index is the index in `bits` of the
    first bit that breaks the rule (a sixth equal bit)."""
    out, run, last, i = [], 0, None, 0
    while i < len(bits):
        b = bits[i]
        out.append(b)
        run = run + 1 if b == last else 1
        last = b
        if run == 5:
            i += 1
            if i >= len(bits):
                break
            if bits[i] == b:
                return out, i
            last, run = bits[i], 1
        i += 1
    return out, None


def tx_stream(ident, data=b"", rtr=False, ext=False, dlc=None):
    """The strict part the TX state machine sends: stuffed SOF..CRC plus the CRC delimiter."""
    return stuff(frame_bits(ident, data, rtr, ext, dlc)) + [1]


def pack_words(bits):
    """Bits (first sent first) -> 32 bit words, LSB first; the last word is padded with recessive (1) bits."""
    words = []
    for k in range(0, len(bits), 32):
        w = 0
        for j, b in enumerate(bits[k:k + 32]):
            w |= (b & 1) << j
        for j in range(len(bits[k:k + 32]), 32):
            w |= 1 << j
        words.append(w)
    return words


def tx_command(bits):
    """TX FIFO words for pio/can_tx.pio: [n-1, data words...]."""
    assert 1 <= len(bits) <= 256
    return [len(bits) - 1] + pack_words(bits)


def tx_frame_command(ident, data=b"", rtr=False, ext=False, dlc=None):
    return tx_command(tx_stream(ident, data, rtr, ext, dlc))


def error_frame_command():
    """Six dominant bits (an active error flag) + the 12 recessive bits the PIO appends."""
    return tx_command([0] * 6)


def wire_bits(strict_bits, acked=True, tail_ones=True):
    """What the bus carries for a TX command: the strict bits, then ACK slot (0 when acknowledged) and 11 ones."""
    return list(strict_bits) + [0 if acked else 1] + [1] * 11


# ----------------------------------------------------------------------------- TX results
def tx_result(word):
    """Decode the word the TX state machine pushes.  Returns a dict:
        completed: ack (bool, ACK slot was dominant), tail_ok (the other 11 bits all read recessive), bits (12 samples)
        aborted:   remaining x (bits left after the bad one); index of the bad bit = n - 1 - x (pass n)"""
    if (word >> 8) == ABORT_TAG:
        return {"aborted": True, "remaining": 0xFF - (word & 0xFF)}
    assert word & 0xFFFFF == 0, hex(word)
    bits = [(word >> (20 + i)) & 1 for i in range(12)]
    return {"aborted": False, "ack": bits[0] == 0, "tail_ok": all(bits[1:]), "bits": bits}


def abort_index(word, n):
    """Index (0 = SOF) of the strict bit at which the TX state machine saw a mismatch."""
    r = tx_result(word)
    assert r["aborted"], hex(word)
    return n - 1 - r["remaining"]


# ----------------------------------------------------------------------------- frame parser
def parse_frame(raw, through_crc=False):
    """Parse a raw (stuffed) bit stream that starts at SOF.  `raw` may run on past the end of the frame.
    through_crc=True stops after the CRC field (and the stuff bit that may follow it): no delimiter / ACK / EOF needed
    (raw_len is then the stuffed length SOF..CRC); used by receivers that must act at the CRC delimiter.
    Returns a dict with: ident, ext, rtr, dlc, data, crc, crc_ok, stuff_error, form_error, ack, eof_ok, raw_len
    (stuffed length SOF..CRC delimiter inclusive), error (None or a short string)."""
    res = dict(ident=None, ext=False, rtr=False, dlc=None, data=b"", crc=None, crc_ok=False, stuff_error=False,
               form_error=False, ack=False, eof_ok=False, raw_len=0, error=None)
    d, run, last, i = [], 0, None, 0
    total = None
    # --- destuff while the field lengths are discovered
    while True:
        if total is not None and len(d) >= total:
            break
        if i >= len(raw):
            res["error"] = "truncated"
            return res
        b = raw[i]
        i += 1
        d.append(b)
        run = run + 1 if b == last else 1
        last = b
        if run == 5:
            # next raw bit must be a stuff bit; it is consumed even after the last CRC bit
            if i >= len(raw):
                res["error"] = "truncated"
                return res
            if raw[i] == b:
                res["stuff_error"], res["error"] = True, "stuff error at raw bit %d" % i
                res["raw_len"] = i + 1
                return res
            last, run = raw[i], 1
            i += 1
        if total is None and len(d) >= 14:
            ide = d[13]
            hdr = 39 if ide else 19
            if len(d) >= hdr:
                dlc = sum(bit << (3 - k) for k, bit in enumerate(d[hdr - 4:hdr]))
                rtr = d[32] if ide else d[12]
                nbytes = 0 if rtr else min(dlc, 8)
                total = hdr + 8 * nbytes + 15
    hdr = 39 if d[13] else 19
    res["ext"] = bool(d[13])
    if res["ext"]:
        res["ident"] = (sum(bit << (10 - k) for k, bit in enumerate(d[1:12])) << 18) | \
                       sum(bit << (17 - k) for k, bit in enumerate(d[14:32]))
        res["rtr"] = bool(d[32])
    else:
        res["ident"] = sum(bit << (10 - k) for k, bit in enumerate(d[1:12]))
        res["rtr"] = bool(d[12])
    res["dlc"] = sum(bit << (3 - k) for k, bit in enumerate(d[hdr - 4:hdr]))
    nbytes = 0 if res["rtr"] else min(res["dlc"], 8)
    res["data"] = bytes(sum(bit << (7 - k) for k, bit in enumerate(d[hdr + 8 * j:hdr + 8 * j + 8]))
                        for j in range(nbytes))
    res["crc"] = sum(bit << (14 - k) for k, bit in enumerate(d[total - 15:total]))
    res["crc_ok"] = crc15(d[:total - 15]) == res["crc"]
    if through_crc:
        res["raw_len"] = i
        return res
    tail = raw[i:]
    if len(tail) < 2:
        res["error"] = "truncated"
        res["raw_len"] = i
        return res
    res["raw_len"] = i + 1
    if tail[0] != 1:
        res["form_error"], res["error"] = True, "CRC delimiter dominant"
    res["ack"] = tail[1] == 0
    rest = tail[2:]
    res["eof_ok"] = len(rest) >= 8 and all(rest[:8])          # ACK delimiter + 7 EOF
    if not res["crc_ok"] and res["error"] is None:
        res["error"] = "CRC mismatch"
    return res


# ----------------------------------------------------------------------------- RX capture
def capture_bits(words):
    """Turn the words of ONE capture (samples..., partial, tag) into the sample list; raises ValueError when the
    words do not fit together."""
    if len(words) < 2 or (words[-1] >> 8) != ABORT_TAG:
        raise ValueError("last word is not a capture tag: %s" % [hex(w) for w in words])
    s = (0xFF - (words[-1] & 0xFF)) + 1           # S samples, (S-1) = ~tag & 0xFF
    full, m = divmod(s, 32)
    if len(words) != full + 2:
        raise ValueError("S=%d needs %d words before the tag, got %d" % (s, full + 1, len(words) - 1))
    bits = []
    for w in words[:full]:
        bits += [(w >> k) & 1 for k in range(32)]
    part = words[full]
    bits += [(part >> (32 - m + k)) & 1 for k in range(m)]
    return bits


def split_captures(words):
    """Split a flat list of RX FIFO words into captures (each ends with its tag word)."""
    out, cur = [], []
    for w in words:
        cur.append(w)
        if (w >> 8) == ABORT_TAG:
            out.append(cur)
            cur = []
    return out, cur


def decode_capture(words):
    """Decode one capture (list of words ending with the tag) into parse_frame()'s dict plus 'samples'."""
    bits = capture_bits(words)
    res = parse_frame(bits)
    res["samples"] = bits
    return res


def capture_words(wire):
    """Reference model of pio/can_rx.pio: the words (samples..., partial, tag) the RX state machine pushes for a
    bus bit stream that starts at SOF (wire[0] must be 0) and continues with recessive bits for at least 11 bits after
    the end of the frame.  Used by the tests to compare the hardware word for word."""
    assert wire and wire[0] == 0
    words, isr, cnt, y, k = [], 0, 0, 0, 0
    for s in wire:
        isr = (isr >> 1) | (s << 31)
        cnt += 1
        if cnt == 32:
            words.append(isr)
            isr, cnt = 0, 0
        if s == 0:
            y = 10
            k += 1
        elif y:
            y -= 1
            k += 1
        else:
            break
    else:
        raise ValueError("the bit stream never ends with 11 recessive bits")
    words.append(isr)                                  # the partial word (0 when it is empty)
    words.append((0xFFFFFFFF - k) & 0xFFFFFFFF)
    return words
