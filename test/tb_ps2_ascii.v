`timescale 1ns/1ps

// tb_ps2_ascii.v -- end-to-end check of tools/build_ps2_ascii.py
// (Step 2: scancode -> ASCII), reusing tb_ps2_reader.v's bootload and
// PS/2-frame-bit-banging tasks unchanged. Where this differs from
// Step 1's test: it checks GPIO_OUT against the TRANSLATED ASCII
// value, not the raw scancode, and specifically exercises the framing
// cases Step 1 never had to care about -- a full make+break pair (one
// key press, one release) confirming exactly one GPIO_OUT write
// happens and the break itself produces no further write; an
// extended make+break pair (0xE0-prefixed) confirming BOTH bytes are
// fully consumed with no emit at all; and an unmapped key confirming
// it resolves to 0x00 rather than garbage or the raw scancode.

module tb_ps2_ascii;
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

    // Preload the flash model with the Step 2 image: 256 pages * 44
    // bytes = 11264 bytes.
    reg [7:0] image [0:11263];
    integer ci;
    initial begin
        $readmemh("ps2_ascii_flash_image.hex", image);
        for (ci = 0; ci < 11264; ci = ci + 1) u_flash.mem[ci] = image[ci];
    end

    task automatic reset_dut;
        begin
            ui_in = 8'h00;
            ui_in[3] = 1'b1; // CLOCK idle high
            ui_in[4] = 1'b1; // DATA idle high
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

    localparam integer PS2_HOLD = 4000;
    localparam integer PS2_SETTLE = 50;

    task automatic ps2_send_bit(input b);
        begin
            ui_in[4] = b;
            repeat (PS2_SETTLE) @(posedge clk);
            ui_in[3] = 1'b0;
            repeat (PS2_HOLD) @(posedge clk);
            ui_in[3] = 1'b1;
            repeat (PS2_HOLD) @(posedge clk);
        end
    endtask

    task automatic ps2_send_frame(input [7:0] scancode);
        reg parity;
        integer bi;
        begin
            parity = ~(^scancode);
            ps2_send_bit(1'b0);            // start bit
            for (bi = 0; bi < 8; bi = bi + 1) ps2_send_bit(scancode[bi]);
            ps2_send_bit(parity);
            ps2_send_bit(1'b1);            // stop bit
            ui_in[4] = 1'b1;
        end
    endtask

    // Time for one full frame to be received AND processed through
    // the (up to 5-page) framing-check chain before we sample
    // GPIO_OUT -- generous margin, same reasoning tb_ps2_reader.v uses.
    localparam integer SETTLE = 40000;

    integer errors;
    reg [7:0] got;

    task automatic expect_emit(input [7:0] scancode, input [7:0] expected,
                                input [383:0] label);
        begin
            ps2_send_frame(scancode);
            repeat (SETTLE) @(posedge clk);
            got = uo_out;
            if (got !== expected) begin
                errors = errors + 1;
                $display("FAIL: %0s: GPIO_OUT=0x%02x, expected 0x%02x", label, got, expected);
            end else begin
                $display("PASS: %0s: GPIO_OUT=0x%02x as expected", label, got);
            end
        end
    endtask

    // Sends a frame and checks GPIO_OUT is UNCHANGED from what it was
    // (i.e. this frame produced no emit at all -- break codes,
    // prefix bytes, and extended sequences all fall in this bucket).
    task automatic expect_no_emit(input [7:0] scancode, input [383:0] label);
        reg [7:0] prev_val;
        begin
            prev_val = uo_out;
            ps2_send_frame(scancode);
            repeat (SETTLE) @(posedge clk);
            got = uo_out;
            if (got !== prev_val) begin
                errors = errors + 1;
                $display("FAIL: %0s: GPIO_OUT changed to 0x%02x (expected to stay 0x%02x, no emit)",
                          label, got, prev_val);
            end else begin
                $display("PASS: %0s: GPIO_OUT stayed 0x%02x (no emit) as expected", label, got);
            end
        end
    endtask

    initial begin
        $dumpfile("tb_ps2_ascii.vcd");
        $dumpvars(0, tb_ps2_ascii);

        $readmemh("flash_handoff_stub.hex", stub_bytes);

        errors = 0;
        reset_dut();
        repeat (3000) @(posedge clk);

        ui_in[2] = 1; // START
        repeat (20) @(posedge clk);
        boot_send_byte(STUB_LEN);
        for (ci = 0; ci < STUB_LEN; ci = ci + 1) boot_send_byte(stub_bytes[ci]);

        repeat (20000) @(posedge clk);

        // --- Plain make codes translate to the right ASCII ---
        expect_emit(8'h1C, 8'h61, "scancode 0x1C ('a' make code) -> 'a' (0x61)");
        expect_emit(8'h32, 8'h62, "scancode 0x32 ('b' make code) -> 'b' (0x62)");
        expect_emit(8'h45, 8'h30, "scancode 0x45 ('0' make code) -> '0' (0x30)");
        expect_emit(8'h29, 8'h20, "scancode 0x29 (space make code) -> ' ' (0x20)");
        expect_emit(8'h5A, 8'h0D, "scancode 0x5A (enter make code) -> CR (0x0D)");

        // --- Unmapped key (e.g. F1 = 0x05) -> 0x00 ---
        expect_emit(8'h05, 8'h00, "scancode 0x05 (F1, unmapped) -> 0x00");

        // --- Make + break of the SAME key: exactly one emit (the
        // make), the break itself produces no further write. ---
        expect_emit(8'h1C, 8'h61, "make 0x1C ('a') before break test -> 'a'");
        expect_no_emit(8'hF0, "break prefix 0xF0 -> no emit");
        expect_no_emit(8'h1C, "break code's key byte (0x1C again) -> no emit");

        // --- Confirm the reader is still healthy after a break
        // sequence: a following plain key still translates normally. ---
        expect_emit(8'h1B, 8'h73, "scancode 0x1B ('s') after break sequence -> 's' (0x73)");

        // --- Extended make + break (e.g. right-ctrl: E0 14 / E0 F0 14):
        // no ASCII mapping, both frames of both sequences produce no
        // emit at all. ---
        expect_no_emit(8'hE0, "extended prefix 0xE0 -> no emit");
        expect_no_emit(8'h14, "extended make code 0x14 -> no emit (no ASCII mapping)");
        expect_no_emit(8'hE0, "extended prefix 0xE0 (break) -> no emit");
        expect_no_emit(8'hF0, "break prefix 0xF0 (within extended break) -> no emit");
        expect_no_emit(8'h14, "extended break code's key byte -> no emit");

        // --- Reader still healthy after an extended sequence. ---
        expect_emit(8'h24, 8'h65, "scancode 0x24 ('e') after extended sequence -> 'e' (0x65)");

        // --- A few more plain keys back-to-back, to confirm the
        // state machine keeps re-arming correctly, not just twice. ---
        expect_emit(8'h44, 8'h6F, "scancode 0x44 ('o') -> 'o' (0x6F)");
        expect_emit(8'h4B, 8'h6C, "scancode 0x4B ('l') -> 'l' (0x6C)");

        // --- FLASH_PAGE bounds check: a plain make code whose
        // TABLE_START+scancode would exceed 255 (mem.v's FLASH_PAGE
        // is only 8 bits) must NOT wrap into some other page -- an
        // earlier version of this design wrapped scancode 0xFF onto
        // EMIT_PAGE's own page number and blindly re-emitted whatever
        // stale ASCII value was left in x8 from the last real
        // translation. Prime x8 with a real translation first so a
        // recurrence of that bug would be visible as the STALE value
        // reappearing instead of 0x00. ---
        expect_emit(8'h44, 8'h6F, "prime x8 with 'o' (0x6F) via scancode 0x44");
        expect_emit(8'hFF, 8'h00, "scancode 0xFF (past MAX_SAFE_SCANCODE) -> 0x00, not stale 'o'");

        if (errors == 0)
            $display("PASS tb_ps2_ascii: every case translated/suppressed correctly");
        else
            $display("FAIL tb_ps2_ascii: %0d mismatch(es)", errors);

        $finish;
    end

    initial begin
        #40000000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
