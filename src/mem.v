// mem.v -- byte-addressable memory for AgilA32
//
// Address map (8-bit address space, 256 bytes total):
//   0x00 - 0xAF : BOOT ROM (176 bytes / 44 instructions) -- combinational,
//                   fixed logic at synthesis time, always present with or
//                   without any Pmod/host attached. Holds the self-test +
//                   demo/listen loop + bootloader -- see
//                   tools/build_boot_rom.py and "Reprogrammability" below.
//                   Safe to rely on at power-up on real silicon since it
//                   is NOT flip-flop state.
//   0xB0 - 0xDF : RAM (48 bytes) -- flip-flops, undefined at power-on on
//                   real silicon. Split into two sub-ranges:
//                     0xB0-0xB3 : always on-chip scratch, never affected
//                                 by FLASH_MODE. The boot ROM doesn't use
//                                 these itself; free for whatever gets
//                                 bootloaded (e.g. its own stack).
//                     0xB4-0xDF : the bootloader's load target (44 bytes).
//                                 The CPU's own fetch/data window once a
//                                 program is running there -- same bytes
//                                 serve both roles, no Harvard split
//                                 needed. When FLASH_MODE (0xF8) has been
//                                 set, reads of *this* sub-range are
//                                 transparently sourced from external
//                                 flash (CS0) instead -- see below. Writes
//                                 to it while FLASH_MODE is set are
//                                 dropped (real NOR flash can't be
//                                 written with a plain 0x02 command the
//                                 way PSRAM can).
//   0xE0 - 0xEF : RAM (16 bytes) -- external PSRAM on the Tiny Tapeout
//                   QSPI Pmod, via qspi_shared_engine. Backed by chip
//                   "RAM A" (CS1) by default, or chip "RAM B" (CS2)
//                   instead once PSRAM_BANK (0xF1) is set -- only one
//                   chip is reachable at a time through this one
//                   16-byte window, same idea as FLASH_MODE picking
//                   which chip backs the 0xB4-0xDF window. Reads/writes
//                   here take multiple clock cycles (the core stalls on
//                   `ready` until the SPI transaction completes)
//                   instead of the single-cycle response everything
//                   else on this bus gets. This is also what the boot
//                   ROM's own power-on self-test probes (write 0xA5,
//                   read back, compare) to light uo_out[7] when no Pmod
//                   is attached -- the self-test runs before any
//                   program has touched PSRAM_BANK, so it always probes
//                   RAM A. Unaffected by FLASH_MODE -- always PSRAM.
//   0xF0        : LED_OUT   (memory-mapped, write-only, drives uo_out)
//   0xF1        : PSRAM_BANK (memory-mapped, read/write, resets to 0) --
//                   bit 0 selects which PSRAM chip the 0xE0-0xEF window
//                   above is backed by: 0 = RAM A (CS1, the reset
//                   default -- matches every previous revision's
//                   behavior), 1 = RAM B (CS2). RAM B uses the exact
//                   same 0x03 READ / 0x02 WRITE command protocol as RAM
//                   A; on the stock Tiny Tapeout QSPI Pmod, CS2 is
//                   already wired to a second, populated PSRAM chip, so
//                   no board modification is needed to reach it (unlike
//                   AgilA8's CS2, which is a generic-purpose SPI
//                   front-end and needs a trace cut on the Pmod before
//                   it can reach anything other than that same chip).
//                   Bits [7:1] are unused/reserved, read as 0.
//   0xF4        : SW_IN     (memory-mapped, read-only, reflects ui_in --
//                   also the bootloader's DATA/CLOCK/START input)
//   0xF2        : TIMER_LO  (memory-mapped, read-only) -- low byte of a
//                   free-running 16-bit counter, ported from AgilA8's
//                   a8_peripherals.v (same bit layout/behavior).
//   0xF3        : TIMER_HI  (memory-mapped, read-only) -- high byte of
//                   the same counter.
//   0xF5        : TIMER_CTRL (memory-mapped, read/write, resets to 0) --
//                   bit0 = enable (counts up once per clock while set),
//                   bit1 = write-1-to-reset the counter to 0 (write-only,
//                   always reads back 0). Bits [7:2] unused, read as 0.
//   0xF6        : TIMER_FLAG (memory-mapped, read/write, resets to 0) --
//                   bit0 = overflow (sticky, set when the counter wraps
//                   0xFFFF -> 0x0000); any write to this address clears
//                   it, regardless of the written value (matches
//                   AgilA8's TIMER_FLAG). Bits [7:1] unused, read as 0.
//   0xF7        : PWM_DUTY  (memory-mapped, read/write, resets to 0) --
//                   8-bit duty cycle out of a free-running 256-cycle
//                   period; 0xFF is special-cased always-on, same as
//                   AgilA8's PWM_DUTY.
//   0xF9        : PWM_CTRL  (memory-mapped, read/write, resets to 0) --
//                   bit0 = enable. Bits [7:1] unused, read as 0.
//   0xFA        : PIN_MUX   (memory-mapped, read/write, resets to 0) --
//                   bits[1:0] select what drives uo_out[7]: 2'b00 =
//                   LED_OUT[7] (the reset default -- matches every
//                   revision before PWM existed), 2'b01 = the PWM
//                   waveform, 2'b10 = the core's `halted` status
//                   (high for as long as the core is parked after an
//                   EBREAK -- see rv32i_core.v), ported from AgilA8's
//                   GPIO_DIR[7] halted-status mux. 2'b11 is reserved
//                   and currently falls back to LED_OUT[7]. uo_out
//                   [6:0] always shows LED_OUT[6:0] regardless. Bits
//                   [7:2] unused, read as 0. Existing code that only
//                   ever wrote 0 or 1 here keeps working unchanged --
//                   those values still mean LED and PWM respectively.
//   0xF8        : FLASH_MODE (memory-mapped, write-only, write-any-value-
//                   to-set -- see "Reprogrammability" below)
//   0xFB        : QSPI_CTRL  (memory-mapped, read/write, resets to 2'd3) --
//                   bits[1:0] select the SCK clock divider for
//                   qspi_shared_engine's flash/PSRAM transactions: 2'd0 =
//                   sys_clk/2 (fastest -- this engine's original, only-
//                   ever speed before this register existed), 2'd1 =
//                   sys_clk/8, 2'd2 = sys_clk/32, 2'd3 = sys_clk/128
//                   (slowest, the reset default), ported from AgilA8's
//                   SPI_CTRL clock-divider field. Sampled by the engine
//                   once per transaction at accept time, so a runtime
//                   write never corrupts a transfer already in flight --
//                   see qspi_shared_engine.v's header for the full
//                   rationale. Bits [7:2] unused, read as 0. The boot
//                   ROM never touches this register, same as FLASH_MODE/
//                   FLASH_PAGE/PSRAM_BANK -- it's left at the slow, safe
//                   reset default for whatever gets bootloaded (or runs
//                   from flash) to speed up once a real device's timing
//                   is confirmed safe.
//   0xFC        : FLASH_PAGE (memory-mapped, read/write, resets to 0) --
//                   selects which WINDOW_BYTES-sized slice of the flash
//                   chip's own larger address space the LOAD_BASE window
//                   currently maps to. See "Bank-switched flash
//                   execution" further down for what this is for and why
//                   it needs care to use correctly -- don't write it from
//                   hand-assembled code without reading that section.
//   0xFD        : SPI_DATA   (memory-mapped, read/write) -- generic SPI
//                   peripheral, ported from AgilA8's spi_ctrl.v. Writing
//                   a byte here clocks it out MOSI-first (CS2, no cmd/
//                   addr framing at all -- byte-only, not word/half:
//                   see "Generic SPI peripheral" below) and simultaneously
//                   captures whatever MISO returns during those same 8
//                   clocks; reading it back (no write) returns that
//                   captured byte immediately, without re-triggering any
//                   hardware transfer -- exactly AgilA8's spi_ctrl "plain
//                   read returns the last transfer's byte" behavior.
//                   Resets to 0x00 (nothing has been clocked in yet).
//
// Word accesses (LW/SW) must be 4-byte aligned. Byte/half accesses
// (LB/LH/SB/SH) are supported at any address within a region, including
// a half-word that straddles a word boundary within on-chip RAM or ROM.
//
// ---------------------------------------------------------------------
// Reprogrammability
// ---------------------------------------------------------------------
// 0x00-0xAF used to hold a fixed demo *application* program, baked in as
// synthesized combinational logic -- permanent the instant the chip was
// taped out. It now holds a fixed boot ROM instead (see
// tools/build_boot_rom.py), following the same pattern AgilA8's
// boot_rom/shared_ram split uses, adapted to this core's single unified
// 8-bit address space (PC and mem_addr are the same bus here, so a
// loaded program is immediately both writable AND fetchable at the same
// address -- no separate IMEM/DMEM aliasing trick needed the way
// AgilA8's shared_ram requires).
//
// On every reset the boot ROM runs first, from 0x00. It self-tests the
// external PSRAM window (write/read/compare, result latched into
// uo_out[7]), then enters an indefinite demo/listen loop -- blinking a
// counter into uo_out[3:0] while polling ui_in[2] (START) every
// iteration, forever, not just in a bounded post-reset window. Once
// START is seen, it bit-bangs a length-prefixed program over ui_in[0:2]
// (DATA/CLOCK/START -- see build_boot_rom.py's docstring for the exact
// wire protocol) into the 0xB4-0xDF RAM window using plain SB
// instructions, then jumps to 0xB4 to run it -- no special hardware
// write port, just software issuing ordinary stores.
//
// The boot ROM itself never touches FLASH_MODE. It's there for a
// bootloaded program to opt into: writing FLASH_MODE (0xF8, write-any-
// value-to-set) makes the *same* 0xB4-0xDF window resolve to external
// flash (CS0) instead of on-chip RAM from then on, so a chip with a
// flashed QSPI Pmod attached can be set up (once, by something you
// bootload) to boot straight from flash on subsequent power-cycles
// without the host re-pushing anything over the wire. Reflashing that
// chip afterward is a normal SPI flash write, not a new tapeout.
//
// This is a single-cycle combinational read / synchronous write memory
// for the on-chip regions; the external windows (0xB4-0xDF in
// FLASH_MODE, and 0xE0-0xEF always) hand off to qspi_shared_engine and
// stall the core on `ready` for as many cycles as the SPI transaction
// needs.

`default_nettype none

module mem #(
    parameter ROM_BYTES      = 176,
    parameter RAM_BASE       = 8'hB0,
    parameter RAM_BYTES      = 48,
    parameter LOAD_BASE      = 8'hB4,   // start of the FLASH_MODE-redirectable sub-range
    parameter EXT_PSRAM_BASE = 8'hE0,
    parameter EXT_PSRAM_BYTES= 16
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  addr,       // byte address
    input  wire [31:0] wdata,
    input  wire [1:0]  size,       // 0=byte, 1=half, 2=word
    input  wire        we,
    input  wire        valid,      // held high by the core for the whole access
    output wire        ready,      // 1 whenever no external transaction is
                                   // in flight -- on-chip accesses always
                                   // see this high immediately (same 1-cycle
                                   // timing as before this port existed)
    output reg  [31:0] rdata,

    input  wire [7:0]  gpio_in,    // ui_in, mapped at 0xF4 -- also the
                                   // bootloader's bit-bang input
    output reg  [7:0]  gpio_out,   // uo_out, mapped at 0xF0

    // Tiny Tapeout QSPI Pmod pins (single-line mode). CS0/flash backs
    // the FLASH_MODE-redirected 0xB4-0xDF window; CS1/psram (RAM A)
    // backs the 0xE0-0xEF window by default (and is what the boot
    // ROM's self-test probes); CS2/psram (RAM B) backs that SAME
    // 0xE0-0xEF window instead, whenever PSRAM_BANK (0xF1) is set --
    // see PSRAM_BANK below.
    output wire        qspi_cs0,   // flash CS
    output wire        qspi_cs1,   // psram "RAM A" CS
    output wire        qspi_cs2,   // psram "RAM B" CS
    output wire        qspi_sck,
    output wire        qspi_mosi,
    input  wire        qspi_miso,

    // Timer/PWM peripherals, ported from AgilA8's a8_peripherals.v --
    // see header above for the register map. pwm_out/pin_mux_out are
    // muxed onto uo_out[7] by the top level (along with the core's
    // `halted` signal, wired directly core-to-top), gated by PIN_MUX
    // (0xFA).
    output wire        pwm_out,
    output wire [1:0]  pin_mux_out
);

    // ---------------------------------------------------------------
    // Boot ROM: fixed self-test + demo/listen loop + bootloader, one
    // 32-bit WORD per case arm (addr[7:2]-indexed), little-endian.
    // Generated by tools/build_boot_rom.py -- regenerate that and
    // re-copy its output here if the boot ROM routine changes; don't
    // hand-edit the case statement.
    //
    // Indexed by word, not byte: a case-statement-in-a-function like
    // this is the standard portable way to describe fixed combinational
    // ROM for ASIC synthesis (a `reg` array with an `initial` isn't
    // reliably synthesizable as permanent logic the way it is for FPGA
    // BRAM), but each instantiation of the case costs real area, and an
    // earlier version instantiated it 4 TIMES per access -- once per
    // byte lane, via `rom_byte(addr)`, `rom_byte(addr+1)`, etc, each
    // with its own adder and its own full ~176-entry decode. Reading
    // one 32-bit word per lookup instead of one byte cuts the case to
    // 1/4 the arms (176 bytes -> 44 words) *and* cuts the number of
    // instantiations from 4 down to 2 (below), which is roughly an
    // order-of-magnitude reduction in the comparator/mux logic this
    // ROM synthesizes to -- confirmed with a generic yosys synth run.
    // ---------------------------------------------------------------
    function [31:0] rom_word_at;
        input [5:0] widx;
        begin
            case (widx)
`include "boot_rom_body.vh"
                default: rom_word_at = 32'h0; // unused ROM space, never fetched
            endcase
        end
    endfunction

    // Byte/half accesses can straddle a word boundary at any address
    // (per the header: "Byte/half accesses are supported at any
    // address within a region"), so fetch the word containing `addr`
    // plus the next word, and byte-select the needed 4 bytes out of
    // that 64-bit pair with a dynamic part-select -- equivalent to the
    // old per-byte-call result for every alignment, but built from just
    // two word lookups instead of four byte lookups.
    wire [31:0] rom_w0   = rom_word_at(addr[7:2]);
    wire [31:0] rom_w1   = rom_word_at(addr[7:2] + 6'd1);
    wire [63:0] rom_pair = {rom_w1, rom_w0};
    wire [31:0] rom_word = rom_pair[(addr[1:0] * 8) +: 32];

    // ---------------------------------------------------------------
    // RAM: RAM_BYTES bytes (default 48) at RAM_BASE (default 0xB0),
    // flip-flop backed. 0xB0-0xB3 is plain scratch; LOAD_BASE-and-up
    // (0xB4-0xDF) is the bootloader's load target AND, once running,
    // the CPU's own fetch/data window -- unless FLASH_MODE has
    // redirected reads of that sub-range to external flash (below).
    //
    // Stored as RAM_BYTES/4 32-bit WORDS, not RAM_BYTES individual
    // bytes -- an earlier version used a flat `reg [7:0] ram [0:47]`
    // byte array and read/wrote it through 4 independently-addressed
    // lanes (ram_addr, +1, +2, +3) every access, each a full dynamic
    // 48-entry array reference. Unlike the ROM fix above (a read-only
    // *constant* lookup, where logic optimization can collapse a lot
    // of the redundancy across lanes), a RAM read/write mux selects
    // among *variable* register values, which doesn't compress the
    // same way -- isolating just this module's old read+write logic
    // in a generic yosys synth measured ~3.3k cells (2.3k of them
    // muxes) for what's logically a 48-byte register file, bigger
    // than the ROM's own footprint. Storing words and reading/writing
    // through the same two-word dynamic part-select trick as the ROM
    // (below) cuts the array depth 4x (48 -> 12) and the number of
    // per-access dynamic array references from 4 down to 1 or 2.
    //
    // Word writes (SW) are handled as a single aligned word store, per
    // the header's documented "word accesses must be 4-byte aligned"
    // contract. Byte/half writes (SB/SH) are still supported at any
    // address, including a half-word that straddles a word boundary
    // (ram_byte_off==3) -- split explicitly into the two words it
    // touches rather than relying on 4 independent byte lanes.
    // ---------------------------------------------------------------
    localparam NWORDS = RAM_BYTES / 4;

    // `mem2reg` tells Yosys up front to treat this as individual
    // flip-flops rather than attempting memory inference first and
    // then falling back -- avoids a "Replacing memory \ram_words with
    // list of registers" lint warning for what was always going to
    // end up as plain DFFs anyway at this size (11 words).
    (* mem2reg *) reg [31:0] ram_words [0:NWORDS-1];
    integer i;

    // `ifndef SYNTHESIS` (which Yosys and other synthesis tools define
    // automatically) is the portable, standards-compliant replacement
    // for the old `synthesis translate_off/on` pragma pair -- same
    // "simulation-only" effect, no lint warning about it.
`ifndef SYNTHESIS
    initial for (i = 0; i < NWORDS; i = i + 1) ram_words[i] = 32'h0;
`endif

    // NWORDS is 12, so a 4-bit index is exactly what's needed (covers
    // 0-15); ram_addr is always < RAM_BYTES(48) whenever these are
    // actually used (gated by in_ram/in_ram_range downstream), so
    // this never actually indexes past entry 11 in practice -- narrows
    // away a WIDTHTRUNC warning (array[11:0] only needs a 4-bit index)
    // along with the UNUSEDSIGNAL warning a wider index's unused top
    // bits would otherwise produce.
    wire [3:0]  ram_widx0    = ram_addr[5:2];
    wire [3:0]  ram_widx1    = ram_addr[5:2] + 4'd1;
    wire [1:0]  ram_byte_off = ram_addr[1:0];
    wire [63:0] ram_pair     = {ram_words[ram_widx1], ram_words[ram_widx0]};

    // ---------------------------------------------------------------
    // FLASH_MODE: write-any-value-to-set, no readback, sticky until
    // reset. Never touched by the boot ROM itself -- opt-in for
    // whatever gets bootloaded (see header).
    //
    // FLASH_PAGE: 8-bit page number, write-any-value-SETS-it (unlike
    // FLASH_MODE, the actual written value matters here), readable,
    // resets to 0, sticky until changed or reset. Selects which
    // WINDOW_BYTES-sized slice of the flash chip's own (much larger)
    // address space the fixed LOAD_BASE-sized chip-address window
    // currently maps to -- see "Bank-switched flash execution" below
    // for why this exists and how a program actually uses it. Reset
    // value 0 means an un-paged program (one that never touches
    // FLASH_PAGE) sees exactly the same page-0-only behavior as
    // before this register existed -- fully backward compatible.
    // ---------------------------------------------------------------
    reg       flash_mode;
    reg [7:0] flash_page;

    // PSRAM_BANK: plain read/write register, resets to 0 (RAM A) -- see
    // header. Only bit 0 is meaningful; kept as a full byte register
    // (rather than a bare 1-bit reg) so the read path can return it the
    // same way every other byte-wide register here does.
    reg       psram_bank;

    // QSPI_CTRL: plain read/write register, resets to 2'd3 (the
    // slowest of the four settings, sys_clk/128) -- see "Variable SPI
    // clock divider" below. Only bits[1:0] are meaningful.
    reg [1:0] qspi_div_sel;

    // SPI_DATA's persistent last-received-byte latch -- see "Generic
    // SPI peripheral" below for why this exists (a plain read must
    // return this without re-triggering any hardware transfer, ported
    // from AgilA8's spi_ctrl.v/a8_peripherals.v spi_last_rx). Latched
    // straight from ext_rdata whenever a SPI_DATA *write* transaction
    // (the only kind that actually reaches the engine -- see
    // in_spi_write below) completes.
    reg [7:0] spi_last_rx;

    // ---------------------------------------------------------------
    // Timer + PWM, ported from AgilA8's a8_peripherals.v (same bit
    // layout/behavior, see header above for the register map). Both
    // are plain free-running counters with no interaction with the
    // ext-memory/`ready` machinery -- they're read/written the same
    // single-cycle way FLASH_MODE/FLASH_PAGE/PSRAM_BANK already are.
    // ---------------------------------------------------------------
    reg [15:0] timer_cnt;
    reg        timer_enable;
    reg        timer_overflow;

    reg [7:0]  pwm_counter;
    reg [7:0]  pwm_duty;
    reg        pwm_enable;
    reg [1:0]  pin_mux;    // 00=LED_OUT[7], 01=pwm_out, 10=halted, 11=reserved(LED)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timer_cnt      <= 16'h0000;
            timer_enable   <= 1'b0;
            timer_overflow <= 1'b0;
        end else begin
            if (valid && we && addr == 8'hF5) begin
                timer_enable <= wdata[0];
                if (wdata[1])
                    timer_cnt <= 16'h0000;
            end else if (timer_enable) begin
                timer_cnt <= timer_cnt + 16'd1;
                if (timer_cnt == 16'hFFFF)
                    timer_overflow <= 1'b1;
            end

            if (valid && we && addr == 8'hF6)
                timer_overflow <= 1'b0;   // write-any-value-to-clear
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_counter <= 8'h00;
            pwm_duty    <= 8'h00;
            pwm_enable  <= 1'b0;
            pin_mux     <= 2'b00;
        end else begin
            pwm_counter <= pwm_counter + 8'd1;

            if (valid && we && addr == 8'hF7)
                pwm_duty <= wdata[7:0];
            if (valid && we && addr == 8'hF9)
                pwm_enable <= wdata[0];
            if (valid && we && addr == 8'hFA)
                pin_mux <= wdata[1:0];
        end
    end

    assign pwm_out = pwm_enable &&
                      ((pwm_duty == 8'hFF) || (pwm_counter < pwm_duty));
    assign pin_mux_out = pin_mux;

    wire in_rom        = (addr < ROM_BYTES);
    wire in_ram_range  = (addr >= RAM_BASE) && (addr < (RAM_BASE + RAM_BYTES));
    wire in_load_range = (addr >= LOAD_BASE) && (addr < (RAM_BASE + RAM_BYTES));
    wire in_ext_flash  = flash_mode && in_load_range;
    wire in_ram        = in_ram_range && !in_ext_flash;
    // Only the low 6 bits of (addr - RAM_BASE) are ever consumed
    // below (ram_widx0/1 need bits[5:2], ram_byte_off needs bits[1:0]).
    // Compute the full 8-bit subtraction first, then take an explicit
    // 6-bit slice of it -- the explicit slice (vs. an implicit
    // width-mismatched assignment) avoids a WIDTHTRUNC warning here
    // while the narrower final wire avoids an UNUSEDSIGNAL warning on
    // bits nothing reads. The 8-bit intermediate itself is
    // intentionally wider than what's consumed (correct wraparound
    // for out-of-range addr needs a full mod-256 subtract, not
    // mod-64), so its own top 2 bits are deliberately unused --
    // suppressed explicitly rather than narrowing the subtraction
    // itself and getting the wrong wraparound.
    /* verilator lint_off UNUSEDSIGNAL */
    wire [7:0] ram_addr_full = addr - RAM_BASE;
    /* verilator lint_on UNUSEDSIGNAL */
    wire [5:0] ram_addr = ram_addr_full[5:0];

    // ---------------------------------------------------------------
    // External windows via qspi_shared_engine: the LOAD_BASE sub-range
    // when FLASH_MODE is set (CS0/flash), the PSRAM window (CS1/psram)
    // always, and SPI_DATA writes (CS2, generic peripheral -- see
    // "Generic SPI peripheral" below). Only one is ever selected for a
    // given access, and the CPU only ever has one access outstanding
    // at a time (mem_valid is never asserted for two different
    // addresses in the same cycle), so a single shared request bus is
    // safe -- same reasoning qspi_shared_engine's own header
    // documents.
    // ---------------------------------------------------------------
    wire in_ext_psram = (addr >= EXT_PSRAM_BASE) && (addr < (EXT_PSRAM_BASE + EXT_PSRAM_BYTES));
    // Only a WRITE to SPI_DATA reaches the engine -- a plain read
    // returns spi_last_rx directly (see the read path below), no
    // hardware transfer, matching AgilA8's spi_ctrl.v exactly. `we`
    // here is `valid`-independent by design (same pattern in_ext_flash
    // already uses above); in_ext itself is only ever consulted where
    // `valid` also gates it (ext_req_valid, ready), same as always.
    wire in_spi_write = (addr == 8'hFD) && we;
    wire in_ext       = in_ext_flash || in_ext_psram || in_spi_write;

    // 0=generic SPI(CS2) 1=flash(CS0) 2=psram RAM A(CS1) 3=psram RAM
    // B(CS2) -- PSRAM_BANK picks between the last two; flash and
    // SPI_DATA always win their own slots since in_ext_flash,
    // in_spi_write, and in_ext_psram are mutually exclusive by address
    // range/decode (see in_ram/in_ext_flash above and in_spi_write's
    // own addr==0xFD check).
    wire [1:0]  req_dev  = in_ext_flash ? 2'd1 : in_spi_write ? 2'd0 : (psram_bank ? 2'd3 : 2'd2);

    // ---------------------------------------------------------------
    // Bank-switched flash execution
    // ---------------------------------------------------------------
    // The chip-address window that FLASH_MODE redirects is fixed --
    // LOAD_BASE through (RAM_BASE+RAM_BYTES-1), WINDOW_BYTES wide --
    // and always will be; that's baked into the 8-bit unified address
    // bus and isn't changing. But the flash CHIP behind it has its own
    // much larger 24-bit address space, and FLASH_PAGE lets a running
    // program pick which WINDOW_BYTES-sized slice of *that* currently
    // shows up at the fixed chip-address window: ext_addr becomes
    // flash_page*WINDOW_BYTES + (addr-LOAD_BASE) instead of just
    // (addr-LOAD_BASE). WINDOW_BYTES*256 pages = up to ~11KB of flash
    // reachable this way with an 8-bit page register, comfortably past
    // what a single 44-byte window could ever run on its own.
    //
    // This does NOT make flash a flat linear address space the CPU can
    // just walk through, though -- switching pages only changes what's
    // fetched from the SAME fixed chip addresses going forward; PC
    // still physically increments through LOAD_BASE..(RAM_BASE+
    // RAM_BYTES-1) and then off the end into the PSRAM window
    // regardless of FLASH_PAGE. A page-switch has to end with an
    // explicit jump back to LOAD_BASE, not just fall through -- and
    // that jump runs into the exact same same-cycle-redirect hazard
    // documented in tools/build_flash_handoff_stub.py's "Why flash
    // byte 0 is dead": FLASH_PAGE takes effect for the very next
    // fetch, which is wherever in the (now new) page the jump
    // instruction itself happens to be sitting -- not fetched from
    // where the programmer wrote it in the OLD page. tools/
    // asm_pineapple.py's PagedAsm handles this by reserving the
    // window's last word as an identical "JAL LOAD_BASE" in every
    // single page, so it doesn't matter which page's copy of that word
    // actually ends up executing after a switch -- see that class's
    // own docstring for the full page layout. Don't hand-roll paged
    // flash programs without it.
    localparam WINDOW_BYTES = (RAM_BASE + RAM_BYTES) - LOAD_BASE;

    // ---------------------------------------------------------------
    // Generic SPI peripheral
    // ---------------------------------------------------------------
    // Ported from AgilA8's spi_ctrl.v/qspi_shared_engine.v CS2 owner --
    // a raw 8-bit SPI transfer with no command/address framing at all,
    // for talking to whatever non-flash/PSRAM SPI device a board
    // happens to have wired to CS2 (an ADC, another MCU, an LCD
    // controller not already covered by a dedicated driver, etc).
    // SPI_DATA (0xFD) is the whole interface: an SB write clocks that
    // byte out MOSI-first and captures MISO's response in the same 8
    // clocks into spi_last_rx; an LBU read of the same address returns
    // spi_last_rx immediately, without clocking anything -- so a
    // multi-byte exchange with a real device is a sequence of SB-then-
    // LBU pairs, not one write followed by however many free reads the
    // device might want to offer.
    //
    // Shares CS2 with PSRAM "RAM B" (see PSRAM_BANK above) -- NOT a
    // separate pin. The stock Tiny Tapeout QSPI Pmod wires CS2 straight
    // to a second, populated PSRAM chip; reaching a *different* device
    // on that pin needs the same board-level trace cut AgilA8's own
    // CS2 has always required (there's no spare CS-capable pin on the
    // board otherwise -- see qspi_shared_engine.v's header for the
    // full reasoning). A program should only ever use one of
    // PSRAM_BANK=1 or SPI_DATA, never both, depending on what's
    // actually populated on its particular board -- this RTL doesn't
    // arbitrate between them, the same way it never has for AgilA8's
    // CS2 either.
    //

    wire [23:0] flash_page_base = flash_page * WINDOW_BYTES; // constant multiply, synthesizes as a couple of adders
    // Generic SPI (in_spi_write): req_addr is unused by the engine for
    // req_dev==2'd0 (no cmd/addr framing at all -- see
    // qspi_shared_engine.v's ST_IDLE), so any well-defined value is
    // fine here; 24'h0 keeps this an honest "don't-care, not garbage".
    wire [23:0] ext_addr = in_ext_flash
        ? (flash_page_base + {16'h0, (addr - LOAD_BASE)})
        : in_spi_write
            ? 24'h0
            : {16'h0, (addr - EXT_PSRAM_BASE)};

    // Flash is read-only from this port -- real NOR flash needs an
    // erase/program sequence a plain 0x02 command can't provide, so
    // writes to the FLASH_MODE-redirected sub-range are dropped (see
    // the write path below), and the request to the engine is never
    // issued as a write for that case either.
    wire        ext_req_we    = we && !in_ext_flash;
    wire        ext_req_valid = valid && in_ext;
    wire [31:0] ext_rdata;
    wire        ext_ready;

    // 1 whenever no genuine external transaction is outstanding this
    // cycle -- preserves the original single-cycle response for every
    // on-chip address, and only ever actually waits on ext_ready when
    // this access is both `valid` and inside an external window.
    assign ready = ext_req_valid ? ext_ready : 1'b1;

    qspi_shared_engine u_qspi (
        .clk       (clk),
        .rst_n     (rst_n),
        .req_valid (ext_req_valid),
        .req_we    (ext_req_we),
        .req_dev   (req_dev),
        .req_addr  (ext_addr),
        .req_wdata (wdata),
        .req_size  (size),
        .req_div_sel (qspi_div_sel),
        .req_rdata (ext_rdata),
        .req_ready (ext_ready),
        .pin_cs0   (qspi_cs0),
        .pin_cs1   (qspi_cs1),
        .pin_cs2   (qspi_cs2),
        .pin_sck   (qspi_sck),
        .pin_mosi  (qspi_mosi),
        .pin_miso  (qspi_miso)
    );

    // ---------------------------------------------------------------
    // Read path
    // ---------------------------------------------------------------
    always @(*) begin
        if (in_rom) begin
            rdata = rom_word;
        end else if (in_ram) begin
            rdata = ram_pair[(ram_byte_off * 8) +: 32];
        end else if (in_ext) begin
            rdata = ext_rdata; // only meaningful once `ready` has pulsed -- see header
        end else if (addr == 8'hF0) begin
            rdata = {24'b0, gpio_out};
        end else if (addr == 8'hF1) begin
            rdata = {31'b0, psram_bank};
        end else if (addr == 8'hF2) begin
            rdata = {24'b0, timer_cnt[7:0]};
        end else if (addr == 8'hF3) begin
            rdata = {24'b0, timer_cnt[15:8]};
        end else if (addr == 8'hF4) begin
            rdata = {24'b0, gpio_in};
        end else if (addr == 8'hF5) begin
            rdata = {30'b0, 1'b0, timer_enable};
        end else if (addr == 8'hF6) begin
            rdata = {31'b0, timer_overflow};
        end else if (addr == 8'hF7) begin
            rdata = {24'b0, pwm_duty};
        end else if (addr == 8'hF9) begin
            rdata = {31'b0, pwm_enable};
        end else if (addr == 8'hFA) begin
            rdata = {30'b0, pin_mux};
        end else if (addr == 8'hFB) begin
            rdata = {30'b0, qspi_div_sel};
        end else if (addr == 8'hFC) begin
            rdata = {24'b0, flash_page};
        end else if (addr == 8'hFD) begin
            // Plain read (we=0, so in_spi_write/in_ext are both false
            // for this access -- the `in_ext` branch above never fires
            // for a SPI_DATA read): return the last byte a SPI_DATA
            // WRITE actually clocked in, no hardware transfer, same
            // single-cycle response every other register on this bus
            // gets. See spi_last_rx's declaration and latch for why.
            rdata = {24'b0, spi_last_rx};
        end else begin
            rdata = 32'h0;
        end
    end

    // ---------------------------------------------------------------
    // Write path (synchronous)
    // ---------------------------------------------------------------
    // spi_last_rx used to be updated by a second, sibling top-level
    // `if (rst_n && ext_ready && in_spi_write) ...` statement placed
    // AFTER this block's main `if (!rst_n) ... else if (we) ...`
    // chain, rather than nested inside it. That's valid Verilog and
    // simulates correctly (confirmed against the standalone
    // testbenches before this fix), but it made this whole process
    // unsynthesizable: Yosys's proc_dff pass errored with "Multiple
    // edge sensitive events found for this signal", reported against
    // `gpio_out` specifically (the first signal driven in this
    // process, in source order) even though the actual structural
    // ambiguity came from spi_last_rx's second, disconnected top-level
    // `if`. Reproduced and confirmed fixed with
    // `yosys -p "read_verilog -sv mem.v; hierarchy -top mem; proc"`.
    // Nesting spi_last_rx's update inside the SAME `else` branch as
    // everything else (as a sibling `if`, not a second top-level one)
    // gives Yosys a single unified sync tree for the whole process,
    // which resolves it -- and also means `rst_n &&` in the guard is
    // now redundant (the `else` branch already only runs when
    // rst_n==1) and has been dropped.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gpio_out   <= 8'h00;
            flash_mode <= 1'b0;
            flash_page <= 8'h00;
            psram_bank <= 1'b0;   // reset default = RAM A, matches every
                                   // pre-CS2 revision's behavior
            qspi_div_sel <= 2'd3; // reset default = slowest (sys_clk/128),
                                   // matches AgilA8's own reset-safe default
            spi_last_rx  <= 8'h00;
        end else begin
            if (we) begin
                if (in_ram_range && !in_ext_flash) begin
                    case (size)
                        2'd0: begin // SB -- always fits in one word, no crossing possible
                            ram_words[ram_widx0][(ram_byte_off * 8) +: 8] <= wdata[7:0];
                        end
                        2'd1: begin // SH -- may straddle a word boundary
                            if (ram_byte_off == 2'd3) begin
                                ram_words[ram_widx0][31:24] <= wdata[7:0];
                                ram_words[ram_widx1][7:0]   <= wdata[15:8];
                            end else begin
                                ram_words[ram_widx0][(ram_byte_off * 8) +: 16] <= wdata[15:0];
                            end
                        end
                        default: begin // SW -- documented to be 4-byte aligned
                            ram_words[ram_widx0] <= wdata;
                        end
                    endcase
                end else if (addr == 8'hF0) begin
                    gpio_out <= wdata[7:0];
                end else if (addr == 8'hF1) begin
                    psram_bank <= wdata[0];   // 0=RAM A(CS1) 1=RAM B(CS2)
                end else if (addr == 8'hF8) begin
                    flash_mode <= 1'b1;   // write-any-value-to-set, sticky until reset
                end else if (addr == 8'hFB) begin
                    qspi_div_sel <= wdata[1:0];
                end else if (addr == 8'hFC) begin
                    flash_page <= wdata[7:0];   // the written value IS the new page, unlike FLASH_MODE
                end
                // 0xFD (SPI_DATA) is deliberately NOT handled in this
                // if/else-if chain -- unlike every other register here,
                // a SPI_DATA write doesn't complete this same cycle; it
                // goes through in_spi_write/qspi_shared_engine instead
                // (see the instantiation above) and takes multiple cycles,
                // same as any other external transaction. See spi_last_rx's
                // latch just below.
            end

            // spi_last_rx: updated independently of the `we`-gated chain
            // above, because it fires on ext_ready, which pulses several
            // cycles AFTER the store that requested it, not on the same
            // edge `we` was sampled. `in_spi_write` is still correctly
            // asserted on that later cycle -- addr/we/valid are all held
            // constant by the core's own wait-state protocol (see
            // rv32i_core.v's ST_MEM_WAIT) for the whole transaction, only
            // clearing the SAME cycle `ready` (here, `ext_ready`) pulses,
            // not before -- see qspi_shared_engine.v's ST_POST comment
            // for the matching reasoning on the engine's own side of
            // this race.
            if (ext_ready && in_spi_write)
                spi_last_rx <= ext_rdata[7:0];
        end
    end

endmodule
