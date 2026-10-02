`timescale 1ns/1ps

// tb_pio_cpu_ws2812.v -- END-TO-END: real RV32I core + real PIO block in the real top level
// (tt_um_agila32), no shortcuts on the bus.
//
// Flow: bootload the flash-handoff stub over the GPIO bootloader -> core jumps into external flash ->
// the flash image built by tools/build_pio_ws2812.py (assembles pio/ws2812.pio) sets GPIO_OUT[0] low,
// programs SM0, preloads four pixels into the TX FIFO, enables SM0, queues a fifth pixel and executes
// EBREAK -> SM0 keeps driving the strip's data line on uo_out[0] with the CPU parked.
//
// The testbench is a WS2812 strip on uo_out[0]: it decodes every bit from the width of its HIGH pulse
// (like the real chip) and checks, counting clocks (24 MHz by convention, PIO tick = CLKDIV 3 = 3 clocks):
//   (a) 5 pixels x 24 bits decode to the GRB values the firmware queued;
//   (b) every 0-bit is exactly 9 clocks high (375 ns) and every 1-bit 21 clocks (875 ns), every bit period
//       exactly 30 clocks (1.25 us) wherever the stream is contiguous -- inside the datasheet windows;
//   (c) no pulse at the PIN_OWN hand-over: the pad is already low when the PIO takes it and stays low;
//   (d) the line was low for at least the 50 us reset time before the first bit (a strip that saw boot
//       garbage has latched);
//   (e) the longest gap between two bits of the frame stays below the 50 us reset time (the strip must
//       not latch mid-frame: the CPU feeds one word per ~3000 clocks but a pixel is only 720 clocks);
//   (f) the core was halted (EBREAK) while the last pixel was still on the wire;
//   (g) the line idles low afterwards and nothing else toggles it.

module tb_pio_cpu_ws2812;
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
        imgname = "pio_ws2812_flash_image.hex";
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
    localparam integer NPIX = 5;
    localparam integer NBITS = NPIX * 24;
    localparam integer TICK = 3;                    // CLKDIV 3
    localparam integer BIT_CLK = 10 * TICK;         // 30 clocks = 1.25 us
    localparam integer T0H = 3 * TICK;              // 9 clocks  = 375 ns
    localparam integer T1H = 7 * TICK;              // 21 clocks = 875 ns
    localparam integer RESET_CLK = 1200;            // 50 us at 24 MHz
    reg [23:0] exp_px [0:NPIX-1];                   // GRB, keep in sync with tools/build_pio_ws2812.py

    // ------------------------------------------------------------------ WS2812 strip on uo_out[0]
    reg  armed, measuring;
    integer clkcnt;
    reg  lvl, lvl_q, pad_q;
    integer hi_cnt, lo_cnt, since_armed;
    integer nbits, bad_width, bad_period, glitch_at_own, lead_idle, max_gap, last_fall_clk, halt_clk, rises, falls;
    reg  [NBITS-1:0] got;                           // bit i of the stream at got[NBITS-1-i]
    integer first_rise_clk;

    initial begin
        armed = 0; measuring = 0; clkcnt = 0; lvl_q = 0; pad_q = 0; hi_cnt = 0; lo_cnt = 0; since_armed = 0;
        nbits = 0; bad_width = 0; bad_period = 0; glitch_at_own = 0; lead_idle = -1; max_gap = 0;
        last_fall_clk = 0; halt_clk = -1; rises = 0; falls = 0; first_rise_clk = -1; got = 0;
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
                if (nbits == 0) begin lead_idle = since_armed; first_rise_clk = clkcnt; end
                else begin
                    if (lo_cnt > max_gap) max_gap = lo_cnt;
                    if (lo_cnt + hi_cnt == BIT_CLK) ;                       // contiguous: exact period
                    else if (lo_cnt + hi_cnt < BIT_CLK) bad_period = bad_period + 1;
                end
                hi_cnt = 0;
            end
            if (lvl) hi_cnt = hi_cnt + 1;
            if (!lvl && lvl_q) begin                                        // falling edge: one bit complete
                falls = falls + 1;
                if (hi_cnt != T0H && hi_cnt != T1H) bad_width = bad_width + 1;
                if (nbits < NBITS) got[NBITS-1-nbits] = (hi_cnt >= 15);
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
        exp_px[0] = 24'h00FF00; exp_px[1] = 24'hFF0000; exp_px[2] = 24'h0000FF;
        exp_px[3] = 24'hFFFFFF; exp_px[4] = 24'h12AB7E;
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
        for (p = 0; p < NPIX; p = p + 1) begin
            if (got[NBITS-1-24*p -: 24] !== exp_px[p]) begin
                errors = errors + 1;
                $display("FAIL: pixel %0d = GRB %06x, expected %06x", p, got[NBITS-1-24*p -: 24], exp_px[p]);
            end else
                $display("PASS: pixel %0d = GRB %06x", p, got[NBITS-1-24*p -: 24]);
        end
        if (bad_width != 0) begin errors = errors + 1; $display("FAIL: %0d pulse(s) not exactly %0d / %0d clocks high", bad_width, T0H, T1H); end
        else $display("PASS: every 0-bit is %0d clocks (375 ns) high, every 1-bit %0d clocks (875 ns)", T0H, T1H);
        if (bad_period != 0) begin errors = errors + 1; $display("FAIL: %0d bit period(s) shorter than %0d clocks", bad_period, BIT_CLK); end
        if (rises != NBITS || falls != NBITS) begin errors = errors + 1; $display("FAIL: %0d rising / %0d falling edges, expected %0d each (stray pulses?)", rises, falls, NBITS); end
        else $display("PASS: exactly %0d pulses on uo_out[0], nothing else", NBITS);
        if (glitch_at_own != 0) begin errors = errors + 1; $display("FAIL: pad was high around the PIN_OWN hand-over"); end
        else $display("PASS: pad low before and after the PIN_OWN hand-over (no glitch)");
        if (lead_idle < RESET_CLK) begin errors = errors + 1; $display("FAIL: line low for only %0d clocks before the first bit, need >= %0d (50 us)", lead_idle, RESET_CLK); end
        else $display("PASS: line low for %0d clocks (%0d us) before the first bit (reset time 50 us)", lead_idle, lead_idle / 24);
        if (max_gap >= RESET_CLK) begin errors = errors + 1; $display("FAIL: gap of %0d clocks inside the frame >= reset time %0d: strip would latch mid-frame", max_gap, RESET_CLK); end
        else $display("PASS: longest gap inside the frame is %0d clocks (%0d us) < 50 us reset time", max_gap, max_gap / 24);
        if (halt_clk < 0 || halt_clk >= last_fall_clk) begin errors = errors + 1; $display("FAIL: core halted at clock %0d, last bit ended at %0d (core must be halted before the stream ends)", halt_clk, last_fall_clk); end
        else $display("PASS: core halted (EBREAK) %0d clocks before the last bit ended; PIO finished alone", last_fall_clk - halt_clk);
        if (uo_out[0] !== 1'b0) begin errors = errors + 1; $display("FAIL: line not idle low after the frame"); end
        else $display("PASS: line idles low after the frame");

        if (errors == 0) $display("PASS tb_pio_cpu_ws2812: CPU programmed PIO, halted, PIO drove 5 WS2812 pixels on uo_out[0]");
        else             $display("FAIL tb_pio_cpu_ws2812: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #200000000;
        $display("TIMEOUT (decoded %0d bits)", nbits);
        $finish;
    end
endmodule
