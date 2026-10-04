"""pio_swd.py -- SWD (ARM Serial Wire Debug) packet helpers for pio/swd.pio.

Pure functions and constants shared by the cocotb tests and the firmware builders: request byte, parity, command
words for the PIO program, DP / AP register addresses and the CTRL/STAT bits.  See pio/swd.pio for the protocol split.
"""

JTAG_TO_SWD = 0xE79E                  # sent LSB first, between two >= 50-ones line resets

ACK_OK, ACK_WAIT, ACK_FAULT = 0b001, 0b010, 0b100      # as read LSB first from the wire

# DP register addresses (A[3:2] << 2); the register behind an address depends on RnW
DP_DPIDR = 0x0                         # read
DP_ABORT = 0x0                         # write
DP_CTRLSTAT = 0x4
DP_SELECT = 0x8
DP_RDBUFF = 0xC                        # read

# MEM-AP registers (bank << 4 | A[3:2] << 2)
AP_CSW = 0x00
AP_TAR = 0x04
AP_DRW = 0x0C
AP_IDR = 0xFC

CSYSPWRUPACK, CSYSPWRUPREQ = 1 << 31, 1 << 30
CDBGPWRUPACK, CDBGPWRUPREQ = 1 << 29, 1 << 28
STICKYERR, WDATAERR = 1 << 5, 1 << 7


def parity(x):
    """Even parity bit of a word: 1 when the number of ones is odd."""
    return bin(x & 0xFFFFFFFF).count("1") & 1


def request_byte(apndp, rnw, addr):
    """The 8-bit request packet, as a value whose bit 0 is sent first:
    start(1) APnDP RnW A2 A3 parity stop(0) park(1);  parity covers APnDP, RnW, A2, A3."""
    assert addr in (0x0, 0x4, 0x8, 0xC)
    a2, a3 = (addr >> 2) & 1, (addr >> 3) & 1
    p = (apndp ^ rnw ^ a2 ^ a3) & 1
    return 1 | (apndp << 1) | (rnw << 2) | (a2 << 3) | (a3 << 4) | (p << 5) | (1 << 7)


def cmd(n, read):
    """Header word for pio/swd.pio: n clocks (1..32), read = release SWDIO and sample, else drive the next word."""
    assert 1 <= n <= 32
    return (n - 1) | (1 << 5 if read else 0)


def select_word(apsel=0, apbank=0, dpbank=0):
    return ((apsel & 0xFF) << 24) | ((apbank & 0xF) << 4) | (dpbank & 0xF)
