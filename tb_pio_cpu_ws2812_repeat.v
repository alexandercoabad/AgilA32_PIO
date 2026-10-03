`timescale 1ns/1ps

// tb_pio_cpu_ws2812_repeat.v -- END-TO-END: real RV32I core + real PIO block in the real top level
// (tt_um_agila32), no shortcuts on the bus.
//
// Flow: bootload the flash-handoff stub over the GPIO bootloader -> core jumps into external flash -> the image built
// by tools/build_pio_ws2812_repeat.py (assembles pio/ws2812_repeat.pio) sets GPIO_OUT[0] low, programs SM0, queues TWO
// colour runs (4 FIFO words), enables SM0, queues a third run (2 words) and executes EBREAK. 60 pixels (30 green,
// 20 red, 10 blue) come from SIX FIFO words; SM0 keeps driving the strip's data line on uo_out[0] with the CPU parked.
//
// The testbench is a WS2812 strip on uo_out[0]: it decodes every bit from the width of its HIGH pulse and checks,
// counting clocks (24 MHz, PIO tick = CLKDIV 3 = 3 clocks):
//   (a) 60 pixels x 24 bits decode to the three runs the firmware queued, in ONE frame (never latched in between);
//   (b) every HIGH pulse is exactly 9 clocks (0) or 21 clocks (1);
//   (c) every LOW pulse is exactly what ws2812_repeat.pio documents: inside a pixel 21 / 9 clocks (after a 0 / 1),
//       between pixels of a run 24 / 12, between two runs 36 / 27 (so never longer than 1.5 us);
//   (d) no pulse at the PIN_OWN hand-over; the line was low >= 50 us before the first bit;
//   (e) the core was halted (EBREAK) long before the last bit: the PIO finished the frame alone;
//   (f) the line idles low afterwards and nothing else toggles it.

module tb_pio_cpu_ws2812_repeat;
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

    reg [7:0] image [0:1023];
    integer ci;
    reg [8*64-1:0] imgname;
    initial begin
        imgname = "pio_ws2812_repeat_flash_image.hex";
        if ($value$plusargs("img=%s", imgname)) ;          // +img=... selects a negative-test image
        $readmemh(imgname, image);
        for (ci = 0; ci < 1024; ci = ci + 1) u_flash.mem[ci] = image[ci];
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

    // ------------------------------------------------------------------ expected data
    localparam integer NRUN = 3;
    localparam integer NPIX = 60;
    localparam integer NBITS = NPIX * 24;
    localparam integer TICK = 3;                    // CLKDIV 3
    localparam integer T0H = 3 * TICK;              // 9 clocks  = 375 ns
    localparam integer T1H = 7 * TICK;              // 21 clocks = 875 ns
    localparam integer RESET_CLK = 1200;            // 50 us at 24 MHz
    integer run_n  [0:NRUN-1];                      // keep in sync with RUNS in tools/build_pio_ws2812_repeat.py
    reg [23:0] run_px [0:NRUN-1];

    // ------------------------------------------------------------------ WS2812 strip on uo_out[0]
    reg  armed;
    integer clkcnt;
    reg  lvl, lvl_q, pad_q;
    integer hi_cnt, lo_cnt, since_armed;
    integer nbits, bad_width, bad_low, glitch_at_own, lead_idle, max_low, last_fall_clk, halt_clk, rises, falls;
    reg  gotb [0:NBITS-1];
    reg  prev_bit;
    integer pj, pidx, cum, r, want_lo, first_bad;
    reg  [23:0] val;

    // expected LOW time (clocks) after bit j whose value is `pb`
    function integer exp_low;
        input integer j; input pb;
        integer b, p, c, k, last_of_run;
        begin
            b = j % 24; p = j / 24; last_of_run = 0; c = 0;
            for (k = 0; k < NRUN; k = k + 1) begin c = c + run_n[k]; if (p == c - 1) last_of_run = 1; end
            if (b != 23)         exp_low = pb ? 3 * TICK : 7 * TICK;
            else if (last_of_run) exp_low = pb ? 9 * TICK : 12 * TICK;
            else                  exp_low = pb ? 4 * TICK : 8 * TICK;
        end
    endfunction

    initial begin
        armed = 0; clkcnt = 0; lvl_q = 0; pad_q = 0; hi_cnt = 0; lo_cnt = 0; since_armed = 0;
        nbits = 0; bad_width = 0; bad_low = 0; glitch_at_own = 0; lead_idle = -1; max_low = 0;
        last_fall_clk = 0; halt_clk = -1; rises = 0; falls = 0; first_bad = 0; prev_bit = 0;
    end

    always @(posedge clk) begin
        clkcnt = clkcnt + 1;
        lvl = uo_out[0];
        if (dut.halted && halt_clk < 0) halt_clk = clkcnt;
        if (!armed && dut.u_pio.own_q[0] === 1'b1) begin
            armed = 1; since_armed = 0;
            if (lvl !== 1'b0 || pad_q !== 1'b0) glitch_at_own = glitch_at_own + 1;   // pad must be low before AND after
        end
        if (armed) begin
            since_armed = since_armed + 1;
            if (lvl && !lvl_q) begin                                        // rising edge
                rises = rises + 1;
                if (nbits == 0) lead_idle = since_armed;
                else begin
                    if (lo_cnt > max_low) max_low = lo_cnt;
                    if (lo_cnt != exp_low(nbits - 1, prev_bit)) begin
                        bad_low = bad_low + 1;
                        if (first_bad < 3) begin
                            first_bad = first_bad + 1;
                            $display("      bit %0d (value %0d): low %0d clocks, expected %0d", nbits - 1, prev_bit, lo_cnt, exp_low(nbits - 1, prev_bit));
                        end
                    end
                end
                hi_cnt = 0;
            end
            if (lvl) hi_cnt = hi_cnt + 1;
            if (!lvl && lvl_q) begin                                        // falling edge: one bit complete
                falls = falls + 1;
                if (hi_cnt != T0H && hi_cnt != T1H) bad_width = bad_width + 1;
                if (nbits < NBITS) gotb[nbits] = (hi_cnt >= 15);
                prev_bit = (hi_cnt >= 15);
                nbits = nbits + 1;
                last_fall_clk = clkcnt;
                lo_cnt = 0;
            end
            if (!lvl) lo_cnt = lo_cnt + 1;
        end
        lvl_q = lvl;
        pad_q = uo_out[0];
    end

    // ------------------------------------------------------------------ run + checks
    integer errors, i, p;
    initial begin
        run_n[0] = 30; run_px[0] = 24'hFF0000;
        run_n[1] = 20; run_px[1] = 24'h00FF00;
        run_n[2] = 10; run_px[2] = 24'h0000FF;
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
        while (nbits < NBITS && i < 4000000) begin @(posedge clk); i = i + 1; end
        repeat (2 * RESET_CLK) @(posedge clk);                       // idle past the reset time

        if (nbits != NBITS) begin
            errors = errors + 1; $display("FAIL: decoded %0d bits, expected %0d", nbits, NBITS);
        end
        pidx = 0;
        for (r = 0; r < NRUN; r = r + 1) begin
            pj = 0;
            for (p = 0; p < run_n[r]; p = p + 1) begin
                val = 0;
                for (i = 0; i < 24; i = i + 1) val = (val << 1) | gotb[pidx*24 + i];
                if (val !== run_px[r]) begin
                    pj = pj + 1;
                    if (pj <= 2) begin errors = errors + 1; $display("FAIL: run %0d pixel %0d (stream pixel %0d) = GRB %06x, expected %06x", r, p, pidx, val, run_px[r]); end
                end
                pidx = pidx + 1;
            end
            if (pj == 0) $display("PASS: run %0d: %0d pixels of GRB %06x", r, run_n[r], run_px[r]);
        end
        if (bad_width != 0) begin errors = errors + 1; $display("FAIL: %0d pulse(s) not exactly %0d / %0d clocks high", bad_width, T0H, T1H); end
        else $display("PASS: every 0-bit is %0d clocks (375 ns) high, every 1-bit %0d clocks (875 ns)", T0H, T1H);
        if (bad_low != 0) begin errors = errors + 1; $display("FAIL: %0d LOW pulse(s) differ from the documented widths", bad_low); end
        else $display("PASS: every LOW pulse as documented (inside pixel 9/21, pixel border 12/24, run border 27/36 clocks); longest %0d clocks (%0d ns)", max_low, max_low * 125 / 3);
        if (rises != NBITS || falls != NBITS) begin errors = errors + 1; $display("FAIL: %0d rising / %0d falling edges, expected %0d each (stray pulses or a latch in between?)", rises, falls, NBITS); end
        else $display("PASS: exactly %0d pulses on uo_out[0], one frame, nothing else", NBITS);
        if (glitch_at_own != 0) begin errors = errors + 1; $display("FAIL: pad was high around the PIN_OWN hand-over"); end
        else $display("PASS: pad low before and after the PIN_OWN hand-over (no glitch)");
        if (lead_idle < RESET_CLK) begin errors = errors + 1; $display("FAIL: line low for only %0d clocks before the first bit, need >= %0d (50 us)", lead_idle, RESET_CLK); end
        else $display("PASS: line low for %0d clocks (%0d us) before the first bit", lead_idle, lead_idle / 24);
        if (halt_clk < 0 || halt_clk + NBITS * 30 / 2 >= last_fall_clk) begin errors = errors + 1; $display("FAIL: core halted at clock %0d, last bit ended at %0d (core must be parked for most of the frame)", halt_clk, last_fall_clk); end
        else $display("PASS: core halted (EBREAK) %0d clocks before the last bit ended; 60 pixels came from 6 FIFO words and the PIO finished alone", last_fall_clk - halt_clk);
        if (uo_out[0] !== 1'b0) begin errors = errors + 1; $display("FAIL: line not idle low after the frame"); end
        else $display("PASS: line idles low after the frame");

        if (errors == 0) $display("PASS tb_pio_cpu_ws2812_repeat: CPU queued 3 colour runs (6 words), halted, PIO drove 60 WS2812 pixels on uo_out[0]");
        else             $display("FAIL tb_pio_cpu_ws2812_repeat: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #200000000;
        $display("TIMEOUT (decoded %0d bits)", nbits);
        $finish;
    end
endmodule
