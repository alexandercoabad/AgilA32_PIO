`timescale 1ns/1ps

// tb_flash_handoff.v -- end-to-end check of the FLASH_MODE handoff
// path: bootload flash_handoff_stub.bin over the normal DATA/CLOCK/
// START protocol (same one test/tb_check.v exercises for RAM
// bootloads), let it set FLASH_MODE, and confirm the CPU actually
// continues executing from external flash (CS0) afterward, landing on
// the right flash byte -- not just that FLASH_MODE got set.
//
// "The right flash byte" is flash offset 4, not offset 0 -- see
// tools/build_flash_handoff_stub.py's header for why. Short version:
// FLASH_MODE's redirect takes effect for the very next fetch, which is
// PC+4 (LOAD_BASE+4) after the 1-instruction stub -- flash's own byte
// 0 is never fetched by anything, and both build_flash_canary.py and
// build_st7789_flash_image.py reserve it as an explicit NOP rather
// than a real instruction that happens to be unreachable. A 2-
// instruction stub (write FLASH_MODE, then jump to flash byte 0) looks
// like the more natural fix but doesn't work: by the time its own
// second instruction would be fetched, FLASH_MODE is already set, so
// that fetch *also* redirects to flash instead of retrieving the jump
// instruction actually sitting in RAM where the bootloader put it.
// Confirmed both ways in simulation while developing this test.
//
// Both CS0 (flash) and CS1 (psram) slave models are attached at once
// here, sharing the single physical SPI bus the way real QSPI Pmod
// hardware would -- spi_ram_model.v itself has no per-instance CS
// awareness beyond driving miso=0 while deselected, so two instances
// wired directly together would contend on miso whenever the selected
// one drives a 1. A small testbench-level mux (below) picks whichever
// model's cs_n is currently low, exactly the way a real shared MISO
// line only ever has one live driver at a time. PSRAM (CS1) has to be
// attached too even though this test doesn't care about it -- the boot
// ROM's power-on self-test probes it unconditionally before the demo
// loop it needs to reach even starts; self-test failure doesn't block
// anything downstream, but the model still needs to be there to answer
// the self-test's own read so simulation time isn't spent chasing a
// misleading uo_out[7].

module tb_flash_handoff;
    reg clk = 0;
    reg rst_n;
    reg [7:0] ui_in;
    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    always #5 clk = ~clk;

    wire cs0  = uio_out[0]; // flash
    wire cs1  = uio_out[6]; // psram
    wire sck  = uio_out[3];
    wire mosi = uio_out[1];
    wire miso_flash, miso_psram;

    // Only one of cs0/cs1 is ever low at a time (mem.v only ever
    // issues a request to one external device per access) -- pick
    // whichever slave is actually selected; default 0 matches both
    // models' own "deselected" output.
    wire miso_bus = (!cs0) ? miso_flash : (!cs1) ? miso_psram : 1'b0;
    wire [7:0] uio_in = {5'b0, miso_bus, 2'b0};

    tt_um_agila32 dut (
        .ui_in   (ui_in),
        .uo_out  (uo_out),
        .uio_in  (uio_in),
        .uio_out (uio_out),
        .uio_oe  (uio_oe),
        .ena     (1'b1),
        .clk     (clk),
        .rst_n   (rst_n)
    );

    spi_ram_model u_flash (
        .cs_n (cs0),
        .sck  (sck),
        .mosi (mosi),
        .miso (miso_flash)
    );

    spi_ram_model u_psram (
        .cs_n (cs1),
        .sck  (sck),
        .mosi (mosi),
        .miso (miso_psram)
    );

    // Preload the flash model with the tiny canary program (see
    // tools/build_flash_canary.py): byte 0 is a dead NOP, bytes 4-15
    // are the actual test -- corresponds to chip addresses LOAD_BASE
    // (0xB4, dead/never fetched) through LOAD_BASE+15 once FLASH_MODE
    // is set. Direct array preload (not a real SPI write sequence) is
    // fine here: this is standing in for "flash programmed by some
    // means outside this bootloader", same as the real chip's own
    // flash would already be pre-flashed before the handoff stub ever
    // runs.
    reg [7:0] canary [0:15];
    integer ci;
    initial begin
        $readmemh("flash_canary.hex", canary);
        for (ci = 0; ci < 16; ci = ci + 1) u_flash.mem[ci] = canary[ci];
    end

    task automatic reset_dut;
        begin
            ui_in = 0; rst_n = 0;
            repeat (10) @(posedge clk);
            rst_n = 1;
            // QSPI_CTRL resets to the slow default (2'd3) on every rst_n
            // deassertion; this testbench's fixed cycle budgets predate
            // that register and assume the engine's original fixed-fast
            // timing, so deposit req_div_sel=0 right after each reset --
            // standing in for "hardware whose real SPI timing has already
            // been confirmed safe" (see CHANGES_feature3.md).
            dut.u_mem.qspi_div_sel = 2'd0;
        end
    endtask

    task automatic send_bit(input b);
        begin
            ui_in[0] = b; ui_in[1] = 0;
            repeat (200) @(posedge clk);
            ui_in[1] = 1;
            repeat (200) @(posedge clk);
            ui_in[1] = 0;
            repeat (200) @(posedge clk);
        end
    endtask

    task automatic send_byte(input [7:0] b);
        integer bi;
        begin
            for (bi = 7; bi >= 0; bi = bi - 1) send_bit(b[bi]);
        end
    endtask

    // Sized to and hardcoded for the current 1-instruction (4-byte)
    // flash_handoff_stub.bin -- if tools/build_flash_handoff_stub.py's
    // output size ever changes, update STUB_LEN (and the array bound
    // below) to match; its own $display print reports the byte count
    // to check against.
    reg [7:0] stub_bytes [0:3];
    localparam integer STUB_LEN = 4;

    initial begin
        $dumpfile("tb_flash_handoff.vcd");
        $dumpvars(0, tb_flash_handoff);

        $readmemh("flash_handoff_stub.hex", stub_bytes);

        reset_dut();
        repeat (3000) @(posedge clk); // past self-test + a few demo-loop iterations

        ui_in[2] = 1; // START
        repeat (20) @(posedge clk);
        send_byte(STUB_LEN);
        for (ci = 0; ci < STUB_LEN; ci = ci + 1) send_byte(stub_bytes[ci]);

        // Give the stub time to run (SW FLASH_MODE) and then the
        // canary's 3 real instructions (its byte-0 NOP is dead, never
        // fetched -- see header). Every canary fetch comes from
        // external flash over the single-line QSPI bus -- a 32-bit
        // instruction fetch is a 64-bit SPI transaction (32-bit
        // cmd+addr, 32-bit data), 2 clock cycles per bit, so ~130
        // cycles just for the SPI shift *per instruction*, on top of
        // the usual decode/exec/mem/wb overhead. 3000 cycles is
        // comfortable margin for the whole sequence.
        repeat (3000) @(posedge clk);

        if (uo_out === 8'h2A)
            $display("PASS tb_flash_handoff: uo_out=0x%02x (flash handoff landed on the right byte)", uo_out);
        else
            $display("FAIL tb_flash_handoff: expected uo_out=0x2a, got uo_out=0x%02x", uo_out);

        $finish;
    end

    initial begin
        #500000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
