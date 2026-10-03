// tb_pio_vga.v -- 640x480 @ 60 Hz VGA from the PIO block alone (pio.v + pio_sm.v + pio_fifo.v):
// SM1 = vga_line.pio (HSYNC + line IRQ), SM0 = vga_frame.pio (vertical timing + colour bars).
//
// One PIO tick = one pixel clock, so the monitor below counts in clocks:
//   line 800 = HSYNC low 96 + back porch + 640 visible + front porch; frame 525 lines
//   = 480 visible + 10 front porch + 2 VSYNC + 33 back porch.
//
// A VGA monitor model watches the Tiny VGA PMOD pins (uo_out[7:0] = {HS,B0,G0,R0,VS,B1,G1,R1}) and
// checks, over several complete frames:
//   * HSYNC period is 800 clocks on EVERY line and its low pulse is 96
//   * VSYNC low pulse is exactly 2 lines (1600 clocks), frame period exactly 525*800 = 420000
//     clocks and exactly 525 HSYNC pulses per frame
//   * 480 visible lines, each: black until the picture starts, then 8 bars of EXACTLY 80 clocks
//     with the palette colours, then black; the picture starts at the same clock on every line
//     and leaves >= 40 clocks of back porch and >= 8 of front porch
//   * every non-visible line is completely black; the unused LSB colour pins stay low
//   * a palette written during the vertical blank takes effect on the next frame (and not earlier)
//
// Hex inputs (made by the Makefile):  /tmp/pio_vga_frame.hex (origin 0, 23 words)
//                                     /tmp/pio_vga_line.hex  (origin 23, 9 words)

`timescale 1ns/1ps
`default_nettype none

module tb_pio_vga;
    reg         clk, rst_n;
    reg         valid, we;
    reg  [7:0]  addr;
    reg  [31:0] wdata;
    wire [31:0] rdata;
    wire        sel;
    wire [9:0]  pin_out, pin_dir, pin_own;
    wire [9:0]  pins_raw = 10'h0;

    `include "tb_pio_common.vh"

    pio #(.N_SM(2), .FIFO_LOG2(2)) dut (
        .clk(clk), .rst_n(rst_n),
        .valid(valid), .we(we), .addr(addr), .wdata(wdata),
        .rdata(rdata), .sel(sel),
        .pins_raw(pins_raw),
        .pin_out(pin_out), .pin_dir(pin_dir), .pin_own(pin_own));

    // ------------------------------------------------------------------ programs
    reg [15:0] frame_prog [0:22];
    reg [15:0] line_prog  [0:8];
    integer i;

    task load_prog;
        input integer base; input integer n; input integer which;
        integer j;
        begin
            bus_write(8'hFF, {24'h0, 1'b1, 7'h20 + base[6:0]});
            for (j = 0; j < n; j = j + 1)
                bus_write(8'hFE, {16'h0, which ? line_prog[j] : frame_prog[j]});
            bus_write(8'hFF, 32'h0);
        end
    endtask

    // palette word: bar k in bits [31-3k : 29-3k]
    function [31:0] pal_word;
        input [2:0] c0, c1, c2, c3, c4, c5, c6, c7;
        begin
            pal_word = {c0, c1, c2, c3, c4, c5, c6, c7, 8'h00};
        end
    endfunction

    task set_palette;               // what the CPU does: push, force `pull`, force `mov isr, osr`
        input [31:0] w;
        begin
            pio_wr(smreg(0, SM_TXF), w);
            sm_exec(0, 16'h80A0);   // pull block
            sm_exec(0, 16'hA0C7);   // mov isr, osr
        end
    endtask

    // ------------------------------------------------------------------ VGA monitor
    localparam [31:0] LINE = 800;
    localparam [31:0] FRAME_LINES = 525;

    reg [2:0]  pal [0:7];           // palette the monitor expects for the line being analysed
    reg [7:0]  lb  [0:799];         // pins sampled for the line in progress (index = clocks since HSYNC fell)
    integer    cyc;
    reg        hs_q, vs_q, started;
    integer    hs_fall, vs_fall, hs_seen, vs_seen;
    integer    line_rel, hs_since_vs;
    integer    frames_done;
    integer    n_period_bad, n_low_bad, n_vslow_bad, n_frame_bad, n_lines_bad;
    integer    n_vis, n_vis_bad, n_blank_bad, n_S_bad, S_ref, n_porch_bad;
    integer    cur_pal_gen;         // incremented by the TB when it changes the palette
    integer    pal_frame_gen;       // generation expected for the lines being analysed
    reg [2:0]  palA [0:7];
    reg [2:0]  palB [0:7];
    reg        use_palB;
    integer    first_bad_msg;
    reg        mon_on;

    function [2:0] expect_color;
        input integer k;
        begin
            expect_color = use_palB ? palB[k] : palA[k];
        end
    endfunction

    task analyze;                   // line `rel` (rel 36..515 are visible) just ended
        input integer rel;
        integer idx, k, j, S, bad;
        begin
            if (rel >= 36 && rel <= 515) begin
                n_vis = n_vis + 1;
                S = -1;
                for (idx = 0; idx < 800 && S < 0; idx = idx + 1)
                    if (lb[idx][2:0] !== 3'b000) S = idx;
                bad = 0;
                if (S < 0) bad = 1;
                else begin
                    if (S_ref < 0) S_ref = S;
                    if (S != S_ref) n_S_bad = n_S_bad + 1;
                    if (S < 96 + 40 || S + 640 > 800 - 8) n_porch_bad = n_porch_bad + 1;
                    for (idx = 0; idx < S; idx = idx + 1)
                        if (lb[idx][2:0] !== 3'b000) bad = bad + 1;
                    for (k = 0; k < 8; k = k + 1)
                        for (j = 0; j < 80; j = j + 1)
                            if (lb[S + 80*k + j][2:0] !== expect_color(k)) bad = bad + 1;
                    for (idx = S + 640; idx < 800; idx = idx + 1)
                        if (lb[idx][2:0] !== 3'b000) bad = bad + 1;
                end
                for (idx = 0; idx < 800; idx = idx + 1)
                    if (lb[idx][6:4] !== 3'b000) bad = bad + 1;
                if (bad != 0) begin
                    n_vis_bad = n_vis_bad + 1;
                    if (first_bad_msg < 3) begin
                        first_bad_msg = first_bad_msg + 1;
                        $display("      visible line rel %0d: %0d bad samples (picture starts at %0d)", rel, bad, S);
                    end
                end
            end else begin
                bad = 0;
                for (idx = 0; idx < 800; idx = idx + 1)
                    if (lb[idx][2:0] !== 3'b000 || lb[idx][6:4] !== 3'b000) bad = bad + 1;
                if (bad != 0) begin
                    n_blank_bad = n_blank_bad + 1;
                    if (first_bad_msg < 3) begin
                        first_bad_msg = first_bad_msg + 1;
                        $display("      blank line rel %0d: %0d non-black samples", rel, bad);
                    end
                end
            end
        end
    endtask

    reg hs, vs;
    integer off;
    initial begin
        cyc = 0; hs_q = 1; vs_q = 1; started = 0; hs_seen = 0; vs_seen = 0;
        line_rel = 0; hs_since_vs = 0; frames_done = 0;
        n_period_bad = 0; n_low_bad = 0; n_vslow_bad = 0; n_frame_bad = 0; n_lines_bad = 0;
        n_vis = 0; n_vis_bad = 0; n_blank_bad = 0; n_S_bad = 0; S_ref = -1; n_porch_bad = 0;
        use_palB = 0; first_bad_msg = 0; mon_on = 0; hs_fall = 0; vs_fall = 0;
    end

    always @(posedge clk) if (mon_on) begin
        #1;
        cyc = cyc + 1;
        hs = pin_out[7]; vs = pin_out[3];
        // ---- HSYNC edges
        if (hs_q && !hs) begin                               // falling
            if (started && hs_seen) begin
                analyze(line_rel);
                if (cyc - hs_fall != LINE) n_period_bad = n_period_bad + 1;
            end
            hs_fall = cyc; hs_seen = 1;
            if (started) begin line_rel = line_rel + 1; hs_since_vs = hs_since_vs + 1; end
        end
        if (!hs_q && hs && hs_seen) begin                    // rising
            if (cyc - hs_fall != 96) n_low_bad = n_low_bad + 1;
        end
        // ---- VSYNC edges
        if (vs_q && !vs) begin                               // falling
            if (vs_seen) begin
                frames_done = frames_done + 1;
                if (cyc - vs_fall != LINE * FRAME_LINES) n_frame_bad = n_frame_bad + 1;
                if (hs_since_vs != FRAME_LINES) n_lines_bad = n_lines_bad + 1;
            end
            vs_fall = cyc; vs_seen = 1; started = 1; line_rel = 0; hs_since_vs = 0;
        end
        if (!vs_q && vs && vs_seen) begin
            if (cyc - vs_fall != 2 * LINE) n_vslow_bad = n_vslow_bad + 1;
        end
        // ---- sample the pins for the line in progress
        off = cyc - hs_fall;
        if (hs_seen && off >= 0 && off < 800) lb[off] = pin_out[7:0];
        hs_q = hs; vs_q = vs;
    end

    // ------------------------------------------------------------------ main
    integer guard;
    initial begin
        valid = 0; we = 0; addr = 0; wdata = 0;
        rst_n = 0;
        $readmemh("/tmp/pio_vga_frame.hex", frame_prog);
        $readmemh("/tmp/pio_vga_line.hex",  line_prog);
        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

        // colour bars: white yellow cyan green magenta red blue black (c = R + 2G + 4B)
        palA[0]=3'd7; palA[1]=3'd3; palA[2]=3'd6; palA[3]=3'd2; palA[4]=3'd5; palA[5]=3'd1; palA[6]=3'd4; palA[7]=3'd0;
        // second palette: shuffled, still ends with black
        palB[0]=3'd4; palB[1]=3'd1; palB[2]=3'd5; palB[3]=3'd2; palB[4]=3'd6; palB[5]=3'd3; palB[6]=3'd7; palB[7]=3'd0;

        load_prog(0, 23, 0);
        load_prog(23, 9, 1);

        // SM0: OUT {R1,G1,B1} = pins 0..2, SET {R1,G1,B1,VS} = pins 0..3; shift left, pull threshold 24
        pio_wr(smreg(0, SM_PINCTRL), pinctrl(4'd0, 4'd3, 4'd0, 3'd4, 4'd0, 3'd0, 4'd0));
        pio_wr(smreg(0, SM_EXEC),    execctrl(5'd0, 5'd22, 1'b0, 1'b0, 4'd0));
        pio_wr(smreg(0, SM_SHIFT),   shiftctrl(1'b0, 1'b0, 1'b1, 1'b0, 5'd0, 5'd24));
        pio_wr(smreg(0, SM_CLKDIV),  clkdiv_reg(16'd1, 8'd0));
        // SM1: SET {HS} = pin 7
        pio_wr(smreg(1, SM_PINCTRL), pinctrl(4'd0, 4'd0, 4'd7, 3'd1, 4'd0, 3'd0, 4'd0));
        pio_wr(smreg(1, SM_EXEC),    execctrl(5'd23, 5'd31, 1'b0, 1'b0, 4'd0));
        pio_wr(smreg(1, SM_SHIFT),   shiftctrl(1'b0, 1'b0, 1'b1, 1'b1, 5'd0, 5'd0));
        pio_wr(smreg(1, SM_CLKDIV),  clkdiv_reg(16'd1, 8'd0));
        pio_wr(R_PIN_OWN, 32'h0000_00FF);

        set_palette(pal_word(palA[0], palA[1], palA[2], palA[3], palA[4], palA[5], palA[6], palA[7]));
        sm_exec(0, 16'hE008);                 // set pins, 8 : black, VSYNC high (idle)
        sm_exec(1, 16'hE001);                 // set pins, 1 : HSYNC high (idle)
        sm_exec(1, 16'h0000 | 16'd23);        // jmp 23 : SM1 starts at its program
        pio_rd(R_PINS_OUT);
        check(rd_val[7] === 1'b1 && rd_val[3] === 1'b1 && rd_val[2:0] === 3'b000, "idle pins: HS=1 VS=1 colour=black");

        mon_on = 1;
        pio_wr(R_CTRL, 32'h0000_0003);        // ONE write enables both state machines: they start together

        // first VSYNC fall, then watch frame 1 with palette A
        guard = 0;
        while (!started && guard < 800000) begin @(posedge clk); guard = guard + 1; end
        check(started, "VSYNC never fell: no frame was generated");
        // wait until the front porch of this frame (rel 520), then switch palette during the blank
        guard = 0;
        while (line_rel < 520 && guard < 800000) begin @(posedge clk); guard = guard + 1; end
        check(line_rel >= 520, "frame did not reach its front porch");
        set_palette(pal_word(palB[0], palB[1], palB[2], palB[3], palB[4], palB[5], palB[6], palB[7]));
        // lines of the NEXT frame use palette B; the 36 invisible lines in between are black either way
        guard = 0;
        while (line_rel >= 520 && guard < 100000) begin @(posedge clk); guard = guard + 1; end
        use_palB = 1;
        // let two more complete frames run
        guard = 0;
        while (frames_done < 3 && guard < 2000000) begin @(posedge clk); guard = guard + 1; end
        check(frames_done >= 3, "fewer than 3 complete frames measured");
        repeat (10) @(posedge clk);
        mon_on = 0;

        // ---------------- results
        check(n_period_bad == 0, "HSYNC period != 800 clocks on some line");
        check(n_low_bad == 0,    "HSYNC low pulse != 96 clocks on some line");
        check(n_vslow_bad == 0,  "VSYNC low pulse != 2 lines (1600 clocks)");
        check(n_frame_bad == 0,  "frame period != 420000 clocks");
        check(n_lines_bad == 0,  "a frame did not have exactly 525 HSYNC pulses");
        check(n_S_bad == 0,      "the picture does not start at the same clock on every line");
        check(n_porch_bad == 0,  "back porch < 40 or front porch < 8 clocks");
        check(n_vis_bad == 0,    "a visible line has wrong bar colours / widths");
        check(n_blank_bad == 0,  "a non-visible line is not black");
        check(n_vis >= 3 * 480 - 480, "too few visible lines were analysed");
        $display("measured: %0d frames, %0d visible lines analysed, picture starts at clock %0d of the line",
                 frames_done, n_vis, S_ref);
        $display("          HSYNC period bad=%0d low bad=%0d | VSYNC low bad=%0d frame bad=%0d lines bad=%0d",
                 n_period_bad, n_low_bad, n_vslow_bad, n_frame_bad, n_lines_bad);
        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("FAIL: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #400000000;
        $display("TIMEOUT (frames_done=%0d line_rel=%0d)", frames_done, line_rel);
        $finish;
    end
endmodule
