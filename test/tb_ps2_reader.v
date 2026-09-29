`timescale 1ns/1ps

// tb_ps2_reader.v -- end-to-end check of the PS/2 keyboard reader built
// by tools/build_ps2_reader.py: bootload the flash-handoff stub, let
// it redirect to the PS/2 reader flash image, then drive ui_in[3]
// (CLOCK) / ui_in[4] (DATA) with a Verilog task that bit-bangs actual
// 11-bit PS/2 frames -- start bit, 8 data bits LSB-first, odd parity,
// stop bit -- the same way a real keyboard would, and confirm
// GPIO_OUT (uo_out) ends up holding the exact scancode byte sent.
//
// Sent as several DISTINCT scancodes back-to-back (not just one),
// specifically to prove the reader's state machine re-arms itself
// correctly after RESULT_PAGE loops back to START_LOW rather than
// only ever working for the very first frame.

module tb_ps2_reader;
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

    // Preload the flash model with the PS/2 reader image.
    reg [7:0] image [0:1055]; // ps2_reader_flash_image.hex is 24 pages * 44 bytes
    integer ci;
    initial begin
        $readmemh("ps2_reader_flash_image.hex", image);
        for (ci = 0; ci < 1056; ci = ci + 1) u_flash.mem[ci] = image[ci];
    end

    task automatic reset_dut;
        begin
            ui_in = 8'h00;
            // PS/2 lines idle high, same as a real keyboard between frames.
            ui_in[3] = 1'b1; // CLOCK
            ui_in[4] = 1'b1; // DATA
            rst_n = 0;
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

    // --- Bootload-over-GPIO helper (ui_in[0]=DATA,[1]=CLOCK,[2]=START),
    // same protocol/timing as tb_st7789_driver.v's send_bit/send_byte. ---
    task automatic boot_send_bit(input b);
        begin
            ui_in[0] = b; ui_in[1] = 0;
            repeat (200) @(posedge clk);
            ui_in[1] = 1;
            repeat (200) @(posedge clk);
            ui_in[1] = 0;
            repeat (200) @(posedge clk);
        end
    endtask

    task automatic boot_send_byte(input [7:0] b);
        integer bi;
        begin
            for (bi = 7; bi >= 0; bi = bi - 1) boot_send_bit(b[bi]);
        end
    endtask

    reg [7:0] stub_bytes [0:3];
    localparam integer STUB_LEN = 4;

    // --- PS/2 device model: bit-bangs ui_in[3]/ui_in[4] like a real
    // keyboard. HOLD is generously long relative to this program's own
    // poll-loop period (each poll iteration is a handful of
    // FLASH-fetched instructions -- much slower than an on-chip fetch,
    // since every instruction is a full single-line SPI word read --
    // so a short pulse here could be missed if too tight; there's no
    // such risk at this margin). ---
    localparam integer PS2_HOLD = 4000;
    localparam integer PS2_SETTLE = 50;

    task automatic ps2_send_bit(input b);
        begin
            ui_in[4] = b;                 // DATA settles while CLOCK is still high
            repeat (PS2_SETTLE) @(posedge clk);
            ui_in[3] = 1'b0;               // CLOCK low -- reader samples DATA here
            repeat (PS2_HOLD) @(posedge clk);
            ui_in[3] = 1'b1;               // CLOCK high -- reader re-arms for next bit
            repeat (PS2_HOLD) @(posedge clk);
        end
    endtask

    task automatic ps2_send_frame(input [7:0] scancode);
        reg parity;
        integer bi;
        begin
            parity = ~(^scancode); // odd parity: total set bits (data+parity) is odd
            ps2_send_bit(1'b0);            // start bit
            for (bi = 0; bi < 8; bi = bi + 1) ps2_send_bit(scancode[bi]); // LSB first
            ps2_send_bit(parity);
            ps2_send_bit(1'b1);            // stop bit
            ui_in[4] = 1'b1;                // release DATA idle
        end
    endtask

    integer errors;
    reg [7:0] got;

    task automatic check_scancode(input [7:0] expected, input [383:0] label);
        begin
            ps2_send_frame(expected);
            repeat (30000) @(posedge clk); // let RESULT_PAGE run and settle
            got = uo_out;
            if (got !== expected) begin
                errors = errors + 1;
                $display("FAIL: %0s: GPIO_OUT=0x%02x, expected 0x%02x", label, got, expected);
            end else begin
                $display("PASS: %0s: GPIO_OUT=0x%02x as expected", label, got);
            end
        end
    endtask

    initial begin
        $dumpfile("tb_ps2_reader.vcd");
        $dumpvars(0, tb_ps2_reader);

        $readmemh("flash_handoff_stub.hex", stub_bytes);

        errors = 0;
        reset_dut();
        repeat (3000) @(posedge clk); // past self-test + a few demo-loop iterations

        ui_in[2] = 1; // START
        repeat (20) @(posedge clk);
        boot_send_byte(STUB_LEN);
        for (ci = 0; ci < STUB_LEN; ci = ci + 1) boot_send_byte(stub_bytes[ci]);

        // Give the handoff a moment to land and the reader to reach
        // its START_LOW wait state before the first frame arrives.
        repeat (20000) @(posedge clk);

        check_scancode(8'h1C, "scancode 0x1C ('A' make code)");
        check_scancode(8'h00, "scancode 0x00 (all-zero data byte)");
        check_scancode(8'hFF, "scancode 0xFF (all-one data byte)");
        check_scancode(8'hF0, "scancode 0xF0 (break-code prefix, raw byte)");
        check_scancode(8'h5A, "scancode 0x5A (alternating bits)");

        if (errors == 0)
            $display("PASS tb_ps2_reader: every scancode reflected correctly on GPIO_OUT");
        else
            $display("FAIL tb_ps2_reader: %0d mismatch(es)", errors);

        $finish;
    end

    initial begin
        #20000000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
