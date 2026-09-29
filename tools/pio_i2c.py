#!/usr/bin/env python3
"""pio_i2c.py -- FIFO word builders for pio/i2c.pio (the unmodified Pico SDK I2C program).

The SDK program assumes an INVERTED pad output-enable (pindir = 1 releases the line).
AgilA32 drives the pad enable straight from PINDIR (pindir = 1 pulls the line low), so every
pindir bit is complemented here (data bits, the ACK slot, the start/stop `set pindirs`
instructions) and pio/i2c.pio has its side-set values swapped to match.

Each word is a 16-bit record, written to the TX FIFO replicated into both halfwords (this is
what a halfword store does on the RP2040; the SM autopulls 16 bits at a time):
    [15:10] n  (n > 0: the next n+1 words are PIO instructions)     [9] final (NAK-ignore)
    [8:1]   data                                                    [0] NAK / ACK-slot
Pin mapping: SDA = base pin, SCL = base + 1 (side-set), in/out/set/jmp pin = SDA.
"""
from pioasm import assemble

_HDR = ".program s\n.side_set 1 opt pindirs\n"


def _set(sda, scl):
    """`set pindirs` + side-set for the wanted LINE levels (1 = released/high, 0 = pulled low)."""
    src = _HDR + "set pindirs, %d side %d [7]" % (0 if sda else 1, 0 if scl else 1)
    return assemble(src).instrs[0]


def _esc(n):
    return n << 10


def start():
    return [_esc(2), _set(1, 1), _set(0, 1), _set(0, 0)]


def repeated_start():
    return [_esc(3), _set(1, 0), _set(1, 1), _set(0, 1), _set(0, 0)]


def stop():
    return [_esc(2), _set(0, 0), _set(0, 1), _set(1, 1)]


def write_byte(b):
    """Send a byte and release SDA for the slave's ACK (a NAK stalls the SM and raises IRQ 0)."""
    return [((~b & 0xFF) << 1)]


def read_byte(last):
    """Clock in a byte; ACK it unless `last`, in which case NAK it and tell the SM that the
    resulting high SDA is expected (FINAL = 1)."""
    return [(1 << 9) if last else 1]


def fifo_word(w16):
    return ((w16 & 0xFFFF) << 16) | (w16 & 0xFFFF)


def write_transaction(addr7, data):
    w = start() + write_byte((addr7 << 1) | 0)
    for b in data:
        w += write_byte(b)
    return w + stop()


def read_transaction(addr7, n):
    w = start() + write_byte((addr7 << 1) | 1)
    for i in range(n):
        w += read_byte(i == n - 1)
    return w + stop()
