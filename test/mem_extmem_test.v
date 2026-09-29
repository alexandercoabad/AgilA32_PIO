// mem_extmem_test.v -- standalone test-memory used only by
// tb_core_ext.v. NOT mem.v -- mem.v now has a fixed boot ROM (self-test
// + listen loop + bootloader) that isn't a convenient vehicle for a
// short, deterministic "CPU drives a real load/store at the external
// window" test, so this keeps the OLD simple-ROM-with-a-swapped-in-test-
// program shape mem.v used to have, just updated to match mem.v's
// current external-window address (0xE0-0xEF -- see mem.v's header for
// why the PSRAM window moved there when the boot ROM grew to make room
// for the bootloader's 0xB0-0xDF RAM window).
//
// Address map here (this test module only):
//   0x00-0x7F : ROM (test program below)
//   0x80-0xAF : RAM (unused by the test program, present for parity)
//   0xE0-0xEF : external PSRAM window (CS1), via qspi_shared_engine
//   0xF0      : LED_OUT
//   0xF4      : SW_IN

`default_nettype none

module mem_test_ext #(
    parameter ROM_BYTES = 128,
    parameter RAM_BYTES = 48
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

    input  wire [7:0]  gpio_in,    // ui_in, mapped at 0xF4
    output reg  [7:0]  gpio_out,   // uo_out, mapped at 0xF0

    // Tiny Tapeout QSPI Pmod pins (single-line mode), backing the
    // 0xB0-0xEF window below via external PSRAM ("RAM A" / CS1)
    output wire        qspi_cs0,   // flash CS -- reserved, unused this round
    output wire        qspi_cs1,   // psram CS
    output wire        qspi_sck,
    output wire        qspi_mosi,
    input  wire        qspi_miso
);

    // ---------------------------------------------------------------
    // ROM: your program goes here, one byte per line, little-endian.
    // See tools/asm_encode.py for the encoder used to generate this
    // table (and to check any instructions you hand-encode yourself).
    //
    // Default contents below are a tiny demo program, verified against
    // test/tb_top.v:
    //   x1 = 0
    // loop:
    //   x1 = x1 + 1
    //   x2 = x1 & 0xF
    //   store x2 -> LED_OUT (0xF0)      // masked value, wraps 0..15
    //   jump loop
    // ---------------------------------------------------------------
    function [7:0] rom_byte;
        input [7:0] a;
        begin
            case (a)
                // TEST PROGRAM (not the production demo -- see
                // test/tb_core_ext.v) exercising the external
                // 0xE0-0xEF window through real CPU load/store
                // instructions instead of driving the bus directly:
                //
                //   addi x1, x0, 0xE0   ; x1 = external RAM address
                //   addi x2, x0, 0xA5   ; x2 = test value
                //   sw   x2, 0(x1)      ; mem[0xE0..0xE3] = 0xA5 (external write)
                //   lw   x3, 0(x1)      ; x3 = mem[0xE0] (external read)
                //   addi x4, x0, 0xF0   ; x4 = LED_OUT address
                //   sw   x3, 0(x4)      ; LED_OUT = x3 -- uo_out should read 0xA5
                // loop:
                //   jal  x0, loop       ; spin forever
                //
                // Encodings independently verified with a standalone
                // decoder (see conversation/dev notes) before use here.
                8'h00: rom_byte = 8'h93; // addi x1,x0,0xE0  -> 0x0E000093
                8'h01: rom_byte = 8'h00;
                8'h02: rom_byte = 8'h00;
                8'h03: rom_byte = 8'h0e;
                8'h04: rom_byte = 8'h13; // addi x2,x0,0xA5  -> 0x0A500113
                8'h05: rom_byte = 8'h01;
                8'h06: rom_byte = 8'h50;
                8'h07: rom_byte = 8'h0a;
                8'h08: rom_byte = 8'h23; // sw   x2,0(x1)    -> 0x0020A023
                8'h09: rom_byte = 8'ha0;
                8'h0a: rom_byte = 8'h20;
                8'h0b: rom_byte = 8'h00;
                8'h0c: rom_byte = 8'h83; // lw   x3,0(x1)    -> 0x0000A183
                8'h0d: rom_byte = 8'ha1;
                8'h0e: rom_byte = 8'h00;
                8'h0f: rom_byte = 8'h00;
                8'h10: rom_byte = 8'h13; // addi x4,x0,0xF0  -> 0x0F000213
                8'h11: rom_byte = 8'h02;
                8'h12: rom_byte = 8'h00;
                8'h13: rom_byte = 8'h0f;
                8'h14: rom_byte = 8'h23; // sw   x3,0(x4)    -> 0x00322023
                8'h15: rom_byte = 8'h20;
                8'h16: rom_byte = 8'h32;
                8'h17: rom_byte = 8'h00;
                // loop: jal x0, loop    -> 0x0000006F
                8'h18: rom_byte = 8'h6f;
                8'h19: rom_byte = 8'h00;
                8'h1a: rom_byte = 8'h00;
                8'h1b: rom_byte = 8'h00;
                default: rom_byte = 8'h00; // unused ROM space, never fetched by this test program
            endcase
        end
    endfunction

    wire [31:0] rom_word = {rom_byte(addr+8'd3), rom_byte(addr+8'd2), rom_byte(addr+8'd1), rom_byte(addr)};

    // ---------------------------------------------------------------
    // RAM: RAM_BYTES bytes (default 48), flip-flop backed
    // ---------------------------------------------------------------
    reg [7:0] ram [0:RAM_BYTES-1];
    integer i;

    // synthesis translate_off
    initial for (i = 0; i < RAM_BYTES; i = i + 1) ram[i] = 8'h00;
    // synthesis translate_on

    wire in_rom = (addr < ROM_BYTES);
    wire in_ram = (addr >= 8'h80) && (addr < (8'h80 + RAM_BYTES));
    wire [7:0] ram_addr = addr - 8'h80;

    // ---------------------------------------------------------------
    // External PSRAM window (0xE0-0xEF, 16 bytes) via qspi_shared_engine
    // -- matches mem.v's current EXT_PSRAM_BASE/EXT_PSRAM_BYTES.
    // ---------------------------------------------------------------
    wire in_ext = (addr >= 8'hE0) && (addr < 8'hF0);
    wire [23:0] ext_addr = {16'h0, (addr - 8'hE0)};

    wire        ext_req_valid = valid && in_ext;
    wire [31:0] ext_rdata;
    wire        ext_ready;

    // 1 whenever no genuine external transaction is outstanding this
    // cycle -- preserves the original single-cycle response for every
    // on-chip address, and only ever actually waits on ext_ready when
    // this access is both `valid` and inside the external window.
    assign ready = ext_req_valid ? ext_ready : 1'b1;

    qspi_shared_engine u_qspi (
        .clk       (clk),
        .rst_n     (rst_n),
        .req_valid (ext_req_valid),
        .req_we    (we),
        .req_dev   (2'd2),          // fixed: this window only ever targets PSRAM (CS1)
        .req_addr  (ext_addr),
        .req_wdata (wdata),
        .req_size  (size),
        .req_div_sel (2'd0),        // fixed fastest -- this test memory has
                                     // no QSPI_CTRL register of its own;
                                     // 2'd0 reproduces the engine's original
                                     // fixed-speed behavior so tb_core_ext.v's
                                     // cycle-count expectations are unaffected
        .req_rdata (ext_rdata),
        .req_ready (ext_ready),
        .pin_cs0   (qspi_cs0),
        .pin_cs1   (qspi_cs1),
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
            rdata = {ram[ram_addr[5:0] + 6'd3], ram[ram_addr[5:0] + 6'd2], ram[ram_addr[5:0] + 6'd1], ram[ram_addr[5:0]]};
        end else if (in_ext) begin
            rdata = ext_rdata; // only meaningful once `ready` has pulsed -- see header
        end else if (addr == 8'hF0) begin
            rdata = {24'b0, gpio_out};
        end else if (addr == 8'hF4) begin
            rdata = {24'b0, gpio_in};
        end else begin
            rdata = 32'h0;
        end
    end

    // ---------------------------------------------------------------
    // Write path (synchronous)
    // ---------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gpio_out <= 8'h00;
        end else if (we) begin
            if (in_ram) begin
                case (size)
                    2'd0: ram[ram_addr[5:0]] <= wdata[7:0];
                    2'd1: begin
                        ram[ram_addr[5:0]]        <= wdata[7:0];
                        ram[ram_addr[5:0] + 6'd1] <= wdata[15:8];
                    end
                    default: begin
                        ram[ram_addr[5:0]]        <= wdata[7:0];
                        ram[ram_addr[5:0] + 6'd1] <= wdata[15:8];
                        ram[ram_addr[5:0] + 6'd2] <= wdata[23:16];
                        ram[ram_addr[5:0] + 6'd3] <= wdata[31:24];
                    end
                endcase
            end else if (addr == 8'hF0) begin
                gpio_out <= wdata[7:0];
            end
        end
    end

endmodule
