// qspi_shared_engine.v -- minimal single-line SPI master shared between
// external flash (CS0), PSRAM "RAM A" (CS1), PSRAM "RAM B" (CS2), and a
// generic SPI peripheral (also CS2) on the Tiny Tapeout QSPI Pmod.
//
// RAM B uses the exact same 0x03/0x02 command protocol as RAM A -- on
// the stock QSPI Pmod board, CS2 is already wired directly to a second,
// populated PSRAM chip (see mole99/qspi-pmod), so unlike AgilA8's CS2
// (a generic-purpose SPI front-end that needs a board trace cut before
// it can reach anything other than that chip), no board modification
// is needed here at all -- RAM B is reachable out of the box, with the
// same read/write command set RAM A already uses. mem.v picks which of
// RAM A/RAM B backs its one external PSRAM window via req_dev.
//
// GENERIC SPI PERIPHERAL, ALSO ON CS2: ported from AgilA8's third SPI
// front-end (its own spi_ctrl.v / qspi_shared_engine.v CS2 owner) --
// req_dev==2'd0 (previously unused/reserved), a raw 8-bit byte shifted
// out MOSI-first with no command/address framing at all, unlike flash/
// PSRAM/RAM B's fixed 0x02/0x03 protocol. This is the SAME physical
// CS2 pin RAM B uses, not a separate one -- there is no spare CS-
// capable pin on the stock QSPI Pmod (confirmed against the board's
// own schematic docs: uio[4]/uio[5] (SD2/SD3) are the flash chip's
// WP/HOLD lines, not general-purpose pins; the only way to free a CS
// line is the board's own documented per-chip trace-cut, same
// mechanism AgilA8's CS2 already depends on). So CS2 here is
// mutually exclusive between RAM B and this generic peripheral at the
// BOARD level, decided by which physical device is actually wired
// there (the stock RAM B chip, unmodified; or an external device,
// after cutting RAM B's trace) -- not something this RTL arbitrates,
// exactly like AgilA8's own CS2 was always board-modification-gated.
// A program should only ever use PSRAM_BANK=1 (RAM B) or the generic
// peripheral (SPI_DATA, mem.v's 0xFD), never both, depending on what's
// actually populated on its particular board.
//
// Deliberately uses only plain single-line SPI (standard 0x03 READ /
// 0x02 WRITE commands, 24-bit address), NOT flash's continuous-read
// mode or PSRAM's QPI mode -- both need an extra mode-byte/setup
// sequence that's easy to get subtly wrong without real hardware to
// verify against. Same reasoning AgilA8's shared engine uses.
//
// One requester at a time only: req_valid must be held high for the
// whole transaction and mem.v must guarantee only one device is ever
// selected (see mem.v's req_dev decode) -- there's no arbitration
// logic here because the CPU's own FSM only ever has one outstanding
// access.
//
// SCK is generated at clk/2 (500 kHz at a 1 MHz system clock, well
// inside both chips' timing budget), SPI mode 0: MOSI changes on the
// SCK falling edge (held stable through the whole low half), MISO is
// sampled on the SCK rising edge.
//
// *** THIS HAS NOT BEEN VALIDATED AGAINST A REAL FLASH/PSRAM        ***
// *** BEHAVIORAL MODEL YET. Simulate against one (see AgilA8's      ***
// *** test/ directory for the kind of testbench this needs) before  ***
// *** trusting this for tapeout.                                    ***
//
// Variable SPI clock divider, ported from AgilA8's SPI_CTRL clock-
// divider field (sys_clk/2, /8, /32, /128) -- in AgilA8 that field only
// ever gated its own standalone generic SPI peripheral (CS2), never
// flash/PSRAM, which always ran at a fixed HALF_PERIOD_CYCLES=1
// regardless of it. AgilA32 unifies this instead: mem.v's QSPI_CTRL
// register (0xFB) gates every front-end through this ONE shared
// engine -- flash, PSRAM, and now the generic SPI peripheral too --
// rather than giving the generic peripheral a second, separate divider
// field the way AgilA8's own CTRL register did (redundant now that
// this engine's timing is already unified across every owner; see
// mem.v's SPI_DATA register comment for the resulting one-register,
// not two-register, design). QSPI_CTRL resets to 2'd3 (the slowest
// setting, sys_clk/128), matching AgilA8's own reset-safe default, and
// is left there by the boot ROM -- exactly like FLASH_MODE, the boot
// ROM itself never touches QSPI_CTRL either; it's there for whatever
// gets bootloaded (or runs from flash) to speed up once a real
// device's timing is confirmed safe at the slow, safe default.
// `req_div_sel` at 2'd0 reproduces the exact fixed-fast timing this
// engine always ran at before this register existed (bit-for-bit
// identical cycle count per transaction, confirmed in
// test/tb_qspi_clkdiv.v), so nothing downstream needs to change to
// keep running exactly as fast as before -- it just isn't the reset
// default anymore.
//
// Byte-order convention: bytes are transferred in ADDRESS order (the
// byte at req_addr goes out first, then req_addr+1, etc.), matching
// how a real flash/PSRAM auto-increments its own address on
// consecutive clocked bytes after the address phase. Combined with
// this project's little-endian convention (wdata[7:0]/rdata[7:0] is
// always the byte at the lowest address), the first byte transferred
// is wdata[7:0] on a write and becomes rdata[7:0] on a read. Because
// the command+address phase is a fixed 32 bits but the data phase is
// variable width (8/16/32 bits), the data bytes must sit at the TOP
// of the data field (immediately after the address bits), not at a
// fixed low position -- see build_preload() below, this is the exact
// bug this version fixes relative to an earlier draft. (Doesn't apply
// to the generic SPI peripheral -- that front-end has no command/
// address phase at all, just a single raw byte.)

`default_nettype none

module qspi_shared_engine (
    input  wire        clk,
    input  wire        rst_n,

    // request interface (mem.v side)
    input  wire        req_valid,   // held high for the whole transaction
    input  wire        req_we,      // 0 = read, 1 = write
    input  wire [1:0]  req_dev,     // 2'd0 = generic SPI peripheral (CS2, raw
                                     // byte, no cmd/addr framing), 2'd1 =
                                     // flash (CS0), 2'd2 = psram RAM A (CS1),
                                     // 2'd3 = psram RAM B (CS2)
    input  wire [23:0] req_addr,    // byte address within the selected device
    input  wire [31:0] req_wdata,
    input  wire [1:0]  req_size,    // 0=byte 1=half 2=word (same as mem.v's `size`)
    input  wire [1:0]  req_div_sel, // clock divider -- see "Variable SPI clock
                                     // divider" below. Sampled once per
                                     // transaction (ST_IDLE, at accept time),
                                     // so a runtime change never corrupts a
                                     // transfer already in flight.
    output reg  [31:0] req_rdata,
    output reg         req_ready,   // pulses high for exactly 1 cycle when done

    // Pmod pins (subset used in single-line mode)
    output reg         pin_cs0,     // flash CS, active low
    output reg         pin_cs1,     // psram "RAM A" CS, active low
    output reg         pin_cs2,     // psram "RAM B" CS, active low
    output reg         pin_sck,
    output reg         pin_mosi,    // SD0, engine-driven
    input  wire        pin_miso     // SD1, engine-sampled
);

    localparam [7:0] CMD_READ  = 8'h03;
    localparam [7:0] CMD_WRITE = 8'h02;

    // Builds the full 64-bit shift preload: {CMD(8), ADDR(24), DATA(32)}.
    // Only the top (32 + nbytes*8) bits of this ever actually get
    // shifted out, so the data field is packed against the top of its
    // 32-bit region (right after the address), not against the bottom.
    // NOT used for the generic SPI peripheral (req_dev==2'd0) -- that
    // front-end has no cmd/addr phase at all, see ST_IDLE below, which
    // left-justifies the raw byte into sreg's top 8 bits directly
    // instead of calling this function.
    // Declared `automatic` deliberately: Yosys's proc_dff pass turns a
    // plain (non-automatic) function's locals into module-scope static
    // regs, and calling such a function only on ONE side of a ternary
    // inside a clocked non-blocking assignment (see ST_IDLE's `sreg <=
    // (req_dev==2'd0) ? ... : build_preload(...)` below, added by the
    // generic-SPI-peripheral change) makes it ambiguous, per call site,
    // whether that static storage should behave as a register or as
    // plain combinational logic -- Yosys reports this as "Multiple edge
    // sensitive events found for this signal" on the function's `addr`
    // input specifically (reproduced with `yosys -p "... proc"` against
    // this file). `automatic` gives the function's locals per-call
    // (stack-like) storage instead of shared static storage, which
    // removes the ambiguity Yosys was tripping on. half_period_for()
    // below doesn't need this -- it's always called unconditionally (no
    // ternary), never from a branch, which apparently sidesteps the
    // issue -- but it's marked `automatic` too for consistency, so neither
    // function's synthesizability depends on how/where it happens to be
    // called from in the future.
    function automatic [63:0] build_preload;
        input        we;
        input [1:0]  size;
        input [23:0] addr;
        input [31:0] wd;
        reg [31:0] data_field;
        begin
            if (!we) begin
                data_field = 32'h0; // don't-care during a read's data phase
            end else begin
                case (size)
                    2'd0:    data_field = {wd[7:0], 24'h0};
                    2'd1:    data_field = {wd[7:0], wd[15:8], 16'h0};
                    default: data_field = {wd[7:0], wd[15:8], wd[23:16], wd[31:24]};
                endcase
            end
            build_preload = {(we ? CMD_WRITE : CMD_READ), addr, data_field};
        end
    endfunction

    // total bits in this transaction: 8 (cmd) + 24 (addr) + data bits
    reg  [6:0] nbits_total;
    reg  [6:0] bits_done;

    // half_period_for(): req_div_sel -> half-SCK-period length in clk
    // cycles. Same 1/4/16/64 encoding AgilA8's own spi_half_period
    // uses (see that module's qspi_shared_engine.v) -- full SCK period
    // is 2x this, so 1/4/16/64 here means SCK = clk/2, clk/8, clk/32,
    // clk/128, exactly the four settings named in the porting roadmap.
    function automatic [7:0] half_period_for;
        input [1:0] sel;
        begin
            case (sel)
                2'd0:    half_period_for = 8'd1;  // fastest -- this
                                                    // engine's original,
                                                    // only-ever speed
                                                    // before this register
                2'd1:    half_period_for = 8'd4;
                2'd2:    half_period_for = 8'd16;
                default: half_period_for = 8'd64; // slowest, reset default
            endcase
        end
    endfunction

    // Latched once per transaction (ST_IDLE, at accept time) into
    // half_period_r -- see req_div_sel's port comment above for why:
    // a runtime QSPI_CTRL write mid-transfer must not change the
    // timing of a transfer already in flight. div_cnt counts clk
    // cycles within the current half-phase; a half-phase (LO or HI)
    // ends when div_cnt reaches half_period_r-1.
    reg [7:0] half_period_r;
    reg [7:0] div_cnt;

    // One big shift register: MSB shifted out on pin_mosi, new bit
    // from pin_miso shifted in at the bottom every bit. After the
    // whole transaction, the low `data_bits` bits hold whatever was
    // clocked in during the data phase (cmd/addr bits have been
    // shifted fully out the top by then).
    reg [63:0] sreg;
    reg [5:0]  data_bits; // nbytes*8 for this transfer

    localparam ST_IDLE     = 3'd0;
    localparam ST_SHIFT_LO = 3'd1; // SCK low half of a bit period
    localparam ST_SHIFT_HI = 3'd2; // SCK high half (sample + shift)
    localparam ST_DONE     = 3'd3;
    localparam ST_POST     = 3'd4; // one-cycle buffer after DONE, before IDLE
                                    // re-checks req_valid -- without this,
                                    // req_ready becomes visible on the exact
                                    // same edge the engine re-enters IDLE, so
                                    // a consumer that hasn't dropped req_valid
                                    // by then causes an immediate spurious
                                    // re-trigger with stale address/data.
                                    // Caught by test/tb_mem_ext.v.

    // Computed unconditionally, combinationally, every cycle -- calling
    // build_preload() itself must never be conditional (e.g. inside a
    // ternary feeding a non-blocking assignment), even after marking it
    // `automatic` above: Yosys's proc_dff pass still errors identically
    // ("Multiple edge sensitive events found for this signal" on the
    // function's own `addr` local), reproduced and confirmed with
    // `yosys -p "... proc"` even with `automatic` in place. Only the
    // SELECTION between this wire and the generic-SPI raw byte may be
    // conditional (see ST_IDLE below) -- exactly mirroring how
    // half_period_for() already gets called unconditionally every
    // ST_IDLE pass and has never hit this error.
    wire [63:0] preload_value = build_preload(req_we, req_size, req_addr, req_wdata);

    reg [2:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            pin_cs0   <= 1'b1;
            pin_cs1   <= 1'b1;
            pin_cs2   <= 1'b1;
            pin_sck   <= 1'b0;
            pin_mosi  <= 1'b1;
            req_ready <= 1'b0;
            req_rdata <= 32'h0;
            sreg      <= 64'h0;
            half_period_r <= 8'd1;
            div_cnt       <= 8'd0;
        end else begin
            req_ready <= 1'b0; // default: single-cycle pulse only

            case (state)
                ST_IDLE: begin
                    pin_sck <= 1'b0;
                    if (req_valid) begin
                        pin_cs0 <= (req_dev == 2'd1) ? 1'b0 : 1'b1;
                        pin_cs1 <= (req_dev == 2'd2) ? 1'b0 : 1'b1;
                        // RAM B and the generic SPI peripheral share the
                        // same physical CS2 pin (see header) -- either
                        // req_dev asserts it, mutually exclusive by which
                        // device is actually wired there, not by this mux.
                        pin_cs2 <= (req_dev == 2'd3 || req_dev == 2'd0) ? 1'b0 : 1'b1;

                        // Generic SPI (req_dev==2'd0): always exactly one
                        // raw byte, no cmd/addr framing, req_size ignored
                        // (mem.v documents SPI_DATA as byte-only -- see
                        // that register's comment). Every other req_dev:
                        // unchanged 32-bit cmd+addr phase plus req_size-
                        // wide data phase.
                        data_bits   <= (req_dev == 2'd0) ? 6'd8 :
                                       (req_size == 2'd0) ? 6'd8  :
                                       (req_size == 2'd1) ? 6'd16 : 6'd32;
                        nbits_total <= (req_dev == 2'd0) ? 7'd8 : 7'd32 +
                                       ((req_size == 2'd0) ? 7'd8  :
                                        (req_size == 2'd1) ? 7'd16 : 7'd32);
                        bits_done   <= 7'd0;

                        // Generic SPI: left-justify the raw byte into
                        // sreg's top 8 bits directly (same left-justify-
                        // the-payload technique the header describes for
                        // build_preload's data field, just with no cmd/
                        // addr phase in front of it at all) -- nbits_total
                        // = 8 means only these top 8 bits ever get shifted
                        // out, whatever ends up in the rest of sreg is
                        // don't-care.
                        sreg  <= (req_dev == 2'd0) ? {req_wdata[7:0], 56'h0}
                                                    : preload_value;
                        half_period_r <= half_period_for(req_div_sel);
                        div_cnt       <= 8'd0;
                        state <= ST_SHIFT_LO;
                        // pin_mosi is intentionally NOT set here -- ST_SHIFT_LO
                        // (below) sets it from sreg[63] on its very first pass
                        // too, using the preload just written above, so the
                        // first bit gets exactly the same settle time before
                        // its rising edge as every subsequent bit.
                    end
                end

                // Each of ST_SHIFT_LO/ST_SHIFT_HI now lasts half_period_r
                // clk cycles (latched above), not always exactly 1 -- div_cnt
                // counts up through the phase and the phase-ending action
                // (set MOSI / sample MISO+advance) fires on its LAST cycle,
                // not its first, so the signal has already been held stable
                // for the full half-period by the time the opposite edge
                // arrives -- same margin reasoning as the original single-
                // cycle version's own comments, just generalized to more
                // than 1 cycle per phase. At half_period_r=1 (req_div_sel=0)
                // div_cnt's first (and only) value is already its last,
                // so both phases collapse back to exactly 1 cycle each --
                // bit-for-bit the same timing as this engine had before this
                // register existed.
                ST_SHIFT_LO: begin
                    pin_sck  <= 1'b0;
                    if (div_cnt == 8'd0)
                        // Set up MOSI here (SCK low half) so it's stable for a
                        // full half-period before the next rising edge -- doing
                        // this in ST_SHIFT_HI instead (as an earlier version of
                        // this file did) changes MOSI on the SAME edge SCK rises,
                        // a same-edge race that shifts every bit one position
                        // early. Caught by test/tb_qspi_engine.v's bitstream check.
                        pin_mosi <= sreg[63];
                    if (div_cnt == half_period_r - 8'd1) begin
                        div_cnt <= 8'd0;
                        state   <= ST_SHIFT_HI;
                    end else begin
                        div_cnt <= div_cnt + 8'd1;
                    end
                end

                ST_SHIFT_HI: begin
                    pin_sck <= 1'b1;
                    if (div_cnt == half_period_r - 8'd1) begin
                        sreg    <= {sreg[62:0], pin_miso};
                        div_cnt <= 8'd0;
                        if (bits_done + 7'd1 == nbits_total) begin
                            state <= ST_DONE;
                        end else begin
                            bits_done <= bits_done + 7'd1;
                            state     <= ST_SHIFT_LO;
                        end
                    end else begin
                        div_cnt <= div_cnt + 8'd1;
                    end
                end

                ST_DONE: begin
                    pin_cs0 <= 1'b1;
                    pin_cs1 <= 1'b1;
                    pin_cs2 <= 1'b1;
                    pin_sck <= 1'b0;
                    // Low `data_bits` bits of sreg = data phase content, in
                    // address order (first byte transferred ends up most
                    // significant within this field). Reverse byte order
                    // here to land back in rdata[7:0]-is-lowest-address form.
                    case (data_bits)
                        6'd8:    req_rdata <= {24'h0, sreg[7:0]};
                        6'd16:   req_rdata <= {16'h0, sreg[7:0], sreg[15:8]};
                        default: req_rdata <= {sreg[7:0], sreg[15:8], sreg[23:16], sreg[31:24]};
                    endcase
                    req_ready <= 1'b1;
                    state     <= ST_POST;
                end

                ST_POST: begin
                    // req_ready has now been visible for a full cycle;
                    // safe to check req_valid again starting next cycle.
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
