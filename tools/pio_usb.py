#!/usr/bin/env python3
"""pio_usb.py -- packet builders / decoders for pio/usb_ls.pio (low-speed USB host engine).

TX side: build the *logical* bit stream (SYNC + PID + fields + CRC, bit-stuffed) and pack it into
TX FIFO words: word 0 = nbits-1, then the bits LSB first, 32 per word. The PIO program does NRZI
and the EOP.
RX side: the PIO pushes the NRZI-decoded (but still stuffed) bits, first bit = MSB of word 0; the
last word is partial and right-aligned. The packet length is recovered here from SYNC, the PID
check and the CRC.
"""

# PID nibbles
OUT, IN, SOF, SETUP = 0x1, 0x9, 0x5, 0xD
DATA0, DATA1 = 0x3, 0xB
ACK, NAK, STALL = 0x2, 0xA, 0xE
NAMES = {OUT: "OUT", IN: "IN", SOF: "SOF", SETUP: "SETUP", DATA0: "DATA0", DATA1: "DATA1",
         ACK: "ACK", NAK: "NAK", STALL: "STALL"}

SYNC_BITS = [0, 0, 0, 0, 0, 0, 0, 1]          # KJKJKJKK after NRZI, sent LSB first (0x80)


def pid_byte(pid):
    return (pid & 0xF) | ((~pid & 0xF) << 4)


def crc5(value, nbits=11):
    crc = 0x1F
    for i in range(nbits):
        b = (value >> i) & 1
        crc = (crc >> 1) ^ 0x14 if (crc ^ b) & 1 else crc >> 1
    return crc ^ 0x1F


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        for i in range(8):
            b = (byte >> i) & 1
            crc = (crc >> 1) ^ 0xA001 if (crc ^ b) & 1 else crc >> 1
    return crc ^ 0xFFFF


def _bits(value, n):
    return [(value >> i) & 1 for i in range(n)]


def stuff(bits):
    out, ones = [], 0
    for b in bits:
        out.append(b)
        ones = ones + 1 if b else 0
        if ones == 6:
            out.append(0)
            ones = 0
    return out


def unstuff(bits):
    out, ones, i = [], 0, 0
    while i < len(bits):
        b = bits[i]
        out.append(b)
        ones = ones + 1 if b else 0
        if ones == 6:
            i += 1                                  # the next bit is a stuffed 0
            if i < len(bits) and bits[i] != 0:
                raise ValueError("bit-stuff error")
            ones = 0
        i += 1
    return out


def packet_bits(pid, fields=()):
    """Stuffed logical bits of a packet: SYNC + PID byte + `fields` (list of bits)."""
    return stuff(SYNC_BITS + _bits(pid_byte(pid), 8) + list(fields))


def token_bits(pid, addr, endp):
    v = (addr & 0x7F) | ((endp & 0xF) << 7)
    return packet_bits(pid, _bits(v, 11) + _bits(crc5(v), 5))


def data_bits(pid, payload):
    f = []
    for b in payload:
        f += _bits(b, 8)
    return packet_bits(pid, f + _bits(crc16(payload), 16))


def handshake_bits(pid):
    return packet_bits(pid)


def tx_words(bits):
    """TX FIFO words for one packet (count word first)."""
    words = [len(bits) - 1]
    for i in range(0, len(bits), 32):
        w = 0
        for k, b in enumerate(bits[i:i + 32]):
            w |= b << k
        words.append(w)
    return words


def parse(bits_stuffed):
    """Decode stuffed logical bits (SYNC first). Returns dict(pid, name, payload, ok, raw)."""
    bits = unstuff(bits_stuffed)
    if bits[:8] != SYNC_BITS:
        raise ValueError("bad SYNC %s" % bits[:8])
    body = bits[8:]
    if len(body) < 8:
        raise ValueError("short packet")
    byte = sum(b << i for i, b in enumerate(body[:8]))
    pid, chk = byte & 0xF, byte >> 4
    if chk != (~pid & 0xF):
        raise ValueError("bad PID check")
    rest = body[8:]
    res = dict(pid=pid, name=NAMES.get(pid, hex(pid)), payload=None, ok=True, raw=bits)
    if pid in (ACK, NAK, STALL):
        res["ok"] = not rest
    elif pid in (OUT, IN, SOF, SETUP):
        if len(rest) != 16:
            res["ok"] = False
        else:
            v = sum(b << i for i, b in enumerate(rest[:11]))
            res["addr"], res["endp"], res["frame"] = v & 0x7F, v >> 7, v
            res["ok"] = crc5(v) == sum(b << i for i, b in enumerate(rest[11:]))
    elif pid in (DATA0, DATA1):
        if len(rest) < 16 or (len(rest) - 16) % 8:
            res["ok"] = False
        else:
            n = (len(rest) - 16) // 8
            payload = [sum(rest[8 * j + i] << i for i in range(8)) for j in range(n)]
            crc = sum(b << i for i, b in enumerate(rest[8 * n:]))
            res["payload"] = payload
            res["ok"] = crc16(payload) == crc
    return res


def _try_decode(words):
    full = []
    for w in words[:-1]:
        full += [(w >> (31 - i)) & 1 for i in range(32)]
    last = words[-1]
    err = None
    for k in range(32, 0, -1):                      # how many bits of the last word are real
        st = full + [(last >> (k - 1 - i)) & 1 for i in range(k)]
        try:
            r = parse(st)
            if r["ok"]:
                return r
        except ValueError as e:
            err = e
    raise ValueError("no valid packet in RX data (%s)" % err)


def decode_rx(words):
    """Decode the words that the RX side of usb_ls.pio pushed into a packet dict (see parse()).

    Every word but the last is full; the last (pushed at SE0) holds only its low k bits, and k is
    found by trying each value until SYNC, PID check, structure and CRC all agree. If the stream
    ended exactly on a word boundary the closing push adds an all-zero word, which is dropped on
    the second attempt.
    """
    words = list(words)
    try:
        return _try_decode(words)
    except ValueError:
        if len(words) > 1 and words[-1] == 0:
            return _try_decode(words[:-1])
        raise
