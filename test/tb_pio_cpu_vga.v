`timescale 1ns/1ps

// tb_pio_cpu_vga.v -- END-TO-END: real RV32I core + real PIO block in the real top level (tt_um_agila32).
//
// Flow: bootload the flash-handoff stub -> core jumps into external flash -> the image built by
// tools/build_pio_vga.py (assembles pio/vga_frame.pio + pio/vga_line.pio) programs both state machines,
// pushes the palette, hands uo_out[7:0] to the PIO, starts both machines with ONE write and executes EBREAK.
// The PIO then draws 640x480 colour bars on the Tiny VGA Pmod pins with the CPU parked.
//
// The testbench is a VGA monitor on uo_out ({HS,B0,G0,R0,VS,B1,G1,R1}); one PIO tick = one clock:
//   (a) the pad is idle (HS=1, VS=1, black) at the PIN_OWN hand-over: no glitch;
//   (b) over two complete frames: every HSYNC period is 800 clocks and its low pulse 96; VSYNC low is exactly
//       2 lines; the frame is 420000 clocks with 525 HSYNC pulses;
//   (c) exactly 480 lines per frame carry a picture, each: 7 coloured bars of exactly 80 clocks in the palette
//       order white yellow cyan green magenta red blue, then black (the 8th bar), starting at the same clock
//       on every line; every other line is completely black; the unused LSB colour pins stay low;
//   (d) the core was halted (EBREAK) long before the first frame ended: the PIO runs alone.

module tb_pio_cpu_vga;
    reg clk = 0;
    reg rst_n;
    reg [7:0] ui_in;
    wire [7:0] uo_out, uio_out, uio_oe;
    always #5 clk = ~clk;

    wire cs0 = uio_out[0], cs1 = uio_out[6], sck = uio_out[3], mosi = uio_out[1];
    wire miso_flash, miso_psram;
    wire miso_bus = (!cs0) ? miso_flash : (!cs1) ? miso_psram : 1'b0;
    wire [7:0] uio_in = {5'b0, miso_bus, 2'b0};

    tt_um_agila32 dut (.ui_in(ui_in), .uo_out(uo_out), .uio_in(uio_in),
                       .uio_out(uio_out), .uio_oe(uio_oe), .ena(1'b1),
                       .clk(clk), .rst_n(rst_n));

    spi_ram_model u_flash (.cs_n(cs0), .sck(sck), .mosi(mosi), .miso(miso_flash));
    spi_ram_model u_psram (.cs_n(cs1), .sck(sck), .mosi(mosi), .miso(miso_psram));

    reg [7:0] image [0:2047];
    integer ci;
    reg [8*64-1:0] imgname;
    initial begin
        imgname = "pio_vga_flash_image.hex";
        if ($value$plusargs("img=%s", imgname)) ;          // +img=... selects a negative-test image
        $readmemh(imgname, image);
        for (ci = 0; ci < 2048; ci = ci + 1) u_flash.mem[ci] = image[ci];
    end

    task automatic boot_send_bit(input b);
        begin
            ui_in[0] = b; ui_in[1] = 0; repeat (200) @(posedge clk);
            ui_in[1] = 1;               repeat (200) @(posedge clk);
            ui_in[1] = 0;               repeat (200) @(posedge clk);
        end
    endtask
    task automatic boot_send_byte(input [7:0] b);
        integer bi;
        begin for (bi = 7; bi >= 0; bi = bi - 1) boot_send_bit(b[bi]); end
    endtask

    reg [7:0] stub_bytes [0:3];
    localparam integer STUB_LEN = 4;

    // ------------------------------------------------------------------ VGA monitor on uo_out
    localparam integer LINE = 800;
    integer clkcnt;
    reg [7:0] pad_q;
    reg       mon;                                  // set once PIO owns the pins
    integer hs_fall, vs_fall, hs_seen, vs_seen, started;
    integer n_period_bad, n_low_bad, n_vslow_bad, n_frame_bad, n_lines_bad, hs_since_vs, frames_done;
    integer halt_clk, first_vs_clk, second_vs_clk, glitch;
    // per-line picture tracking
    integer run_len, bars_seen, bad_runs, first_nz, S_ref, n_S_bad, n_pic, n_pic_bad, line_pic_bad;
    reg [2:0] cur_c;
    reg [2:0] pal [0:6];
    integer line_idx;

    initial begin
        clkcnt = 0; pad_q = 8'h88; mon = 0; hs_fall = 0; vs_fall = 0; hs_seen = 0; vs_seen = 0; started = 0;
        n_period_bad = 0; n_low_bad = 0; n_vslow_bad = 0; n_frame_bad = 0; n_lines_bad = 0; hs_since_vs = 0;
        frames_done = 0; halt_clk = -1; first_vs_clk = -1; second_vs_clk = -1; glitch = 0;
        run_len = 0; bars_seen = 0; bad_runs = 0; first_nz = -1; S_ref = -1; n_S_bad = 0; n_pic = 0;
        n_pic_bad = 0; line_pic_bad = 0; cur_c = 0; line_idx = 0;
        pal[0] = 7; pal[1] = 3; pal[2] = 6; pal[3] = 2; pal[4] = 5; pal[5] = 1; pal[6] = 4;
    end

    task end_line;
        begin
            if (started && hs_seen) begin
                if (bars_seen != 0) begin
                    n_pic = n_pic + 1;
                    if (bars_seen != 7 || bad_runs != 0) n_pic_bad = n_pic_bad + 1;
                    if (S_ref < 0) S_ref = first_nz;
                    if (first_nz != S_ref) n_S_bad = n_S_bad + 1;
                end
            end
            bars_seen = 0; bad_runs = 0; first_nz = -1; run_len = 0; cur_c = 0;
        end
    endtask

    always @(posedge clk) begin
        clkcnt = clkcnt + 1;
        if (dut.halted && halt_clk < 0) halt_clk = clkcnt;
        if (!mon && dut.u_pio.own_q[7:0] === 8'hFF) begin
            mon = 1;
            if (uo_out !== 8'h88) glitch = glitch + 1;          // HS=1 VS=1 black at the hand-over
        end
        if (mon) begin
            // ---- HSYNC
            if (pad_q[7] && !uo_out[7]) begin
                end_line;
                if (hs_seen && clkcnt - hs_fall != LINE) n_period_bad = n_period_bad + 1;
                hs_fall = clkcnt; hs_seen = 1;
                if (started) hs_since_vs = hs_since_vs + 1;
            end
            if (!pad_q[7] && uo_out[7] && hs_seen && clkcnt - hs_fall != 96) n_low_bad = n_low_bad + 1;
            // ---- VSYNC
            if (pad_q[3] && !uo_out[3]) begin
                if (vs_seen) begin
                    frames_done = frames_done + 1;
                    if (clkcnt - vs_fall != LINE * 525) n_frame_bad = n_frame_bad + 1;
                    if (hs_since_vs != 525) n_lines_bad = n_lines_bad + 1;
                end
                if (first_vs_clk < 0) first_vs_clk = clkcnt; else if (second_vs_clk < 0) second_vs_clk = clkcnt;
                vs_fall = clkcnt; vs_seen = 1; started = 1; hs_since_vs = 0;
            end
            if (!pad_q[3] && uo_out[3] && vs_seen && clkcnt - vs_fall != 2 * LINE) n_vslow_bad = n_vslow_bad + 1;
            // ---- colour runs (only meaningful once a frame has started)
            if (uo_out[6:4] !== 3'b000) bad_runs = bad_runs + 1;
            if (uo_out[2:0] !== cur_c) begin
                if (cur_c != 0) begin                           // a coloured bar just ended
                    if (run_len != 80 || cur_c !== pal[bars_seen - 1]) bad_runs = bad_runs + 1;
                end
                if (uo_out[2:0] != 0) begin
                    bars_seen = bars_seen + 1;
                    if (first_nz < 0) first_nz = clkcnt - hs_fall;
                    if (bars_seen > 7 || uo_out[2:0] !== pal[bars_seen - 1]) bad_runs = bad_runs + 1;
                end
                cur_c = uo_out[2:0]; run_len = 0;
            end
            run_len = run_len + 1;
        end
        pad_q = uo_out;
    end

    // ------------------------------------------------------------------ run + checks
    integer errors, i;
    initial begin
        $readmemh("flash_handoff_stub.hex", stub_bytes);
        errors = 0;
        ui_in = 8'h00;
        rst_n = 0; repeat (10) @(posedge clk); rst_n = 1;
        dut.u_mem.qspi_div_sel = 2'd0;
        repeat (3000) @(posedge clk);

        ui_in[2] = 1;
        repeat (20) @(posedge clk);
        boot_send_byte(STUB_LEN);
        for (ci = 0; ci < STUB_LEN; ci = ci + 1) boot_send_byte(stub_bytes[ci]);

        i = 0;
        while (frames_done < 3 && i < 4000000) begin @(posedge clk); i = i + 1; end
        repeat (10) @(posedge clk);

        if (frames_done < 3) begin errors = errors + 1; $display("FAIL: only %0d complete frames measured", frames_done); end
        if (glitch != 0) begin errors = errors + 1; $display("FAIL: pads not idle (HS=1 VS=1 black) at the PIN_OWN hand-over"); end
        else $display("PASS: pads idle (HS=1, VS=1, black) at the PIN_OWN hand-over");
        if (n_period_bad != 0) begin errors = errors + 1; $display("FAIL: %0d HSYNC period(s) != 800 clocks", n_period_bad); end
        else $display("PASS: every HSYNC period is exactly 800 clocks");
        if (n_low_bad != 0) begin errors = errors + 1; $display("FAIL: %0d HSYNC low pulse(s) != 96 clocks", n_low_bad); end
        else $display("PASS: every HSYNC low pulse is exactly 96 clocks");
        if (n_vslow_bad != 0) begin errors = errors + 1; $display("FAIL: VSYNC low != 2 lines"); end
        else $display("PASS: every VSYNC low pulse is exactly 2 lines (1600 clocks)");
        if (n_frame_bad != 0 || n_lines_bad != 0) begin errors = errors + 1; $display("FAIL: frame period / line count (bad=%0d / %0d)", n_frame_bad, n_lines_bad); end
        else $display("PASS: every frame is exactly 420000 clocks = 525 lines");
        if (n_pic < 2 * 480) begin errors = errors + 1; $display("FAIL: only %0d picture lines", n_pic); end
        if (n_pic_bad != 0) begin errors = errors + 1; $display("FAIL: %0d picture line(s) are not 7 bars of 80 clocks in palette order", n_pic_bad); end
        else $display("PASS: %0d picture lines, each 7 coloured bars of exactly 80 clocks (white yellow cyan green magenta red blue) + black", n_pic);
        if (n_pic % 480 != 0) begin errors = errors + 1; $display("FAIL: picture-line count %0d is not a whole number of 480-line frames", n_pic); end
        if (n_S_bad != 0) begin errors = errors + 1; $display("FAIL: the picture does not start at the same clock on every line"); end
        else $display("PASS: picture starts at clock %0d of every line", S_ref);
        if (halt_clk < 0 || halt_clk >= first_vs_clk + LINE * 525) begin errors = errors + 1; $display("FAIL: core not halted before the first frame ended (halt at %0d)", halt_clk); end
        else $display("PASS: core halted (EBREAK) at clock %0d, %0d clocks before the first frame ended; PIO runs alone", halt_clk, first_vs_clk + LINE * 525 - halt_clk);

        if (errors == 0) $display("PASS tb_pio_cpu_vga: CPU started two PIO state machines and halted; PIO drew %0d frames of 640x480 colour bars", frames_done);
        else             $display("FAIL tb_pio_cpu_vga: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #400000000;
        $display("TIMEOUT (frames_done=%0d)", frames_done);
        $finish;
    end
endmodule
