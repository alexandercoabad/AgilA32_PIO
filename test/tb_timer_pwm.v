// tb_timer_pwm.v -- exercises mem.v's new Timer/PWM peripheral
// registers (TIMER_LO/HI 0xF2/0xF3, TIMER_CTRL 0xF5, TIMER_FLAG 0xF6,
// PWM_DUTY 0xF7, PWM_CTRL 0xF9, PIN_MUX 0xFA), ported from AgilA8's
// a8_peripherals.v. Drives the addr/wdata/size/we/valid bus exactly
// the way rv32i_core's FSM does (hold `valid` high until `ready`),
// mirroring tb_mem_ext.v's style.
//
// Part 1 drives the `mem` module directly (bus-level register
// behavior: counting, reset, overflow, duty cycle, enables). Part 2
// instantiates the real top level, tt_um_agila32, and confirms the
// PIN_MUX (0xFA) wiring actually reaches uo_out[7] the way
// tt_um_agila32.v's `assign uo_out = {pin_mux ? pwm_out : ...}` line
// promises -- forcing the peripheral's own internal regs directly
// (same idea as driving the bus in Part 1, just skipping the need for
// a full bootloaded program to reach these write-only-from-software
// registers) since a full CPU program isn't needed to check a purely
// combinational mux. Both parts share one clock and run strictly
// sequentially in a single initial block, so $finish only fires once
// both have actually completed.

`timescale 1ns/1ps
`default_nettype none

module tb_timer_pwm;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    // ---------------- Part 1 DUT: mem module directly ----------------
    reg  [7:0]  addr = 0;
    reg  [31:0] wdata = 0;
    reg  [1:0]  size = 2'd0;
    reg         we = 0;
    reg         valid = 0;
    wire        ready;
    wire [31:0] rdata;
    wire        pwm_out;
    wire [1:0]  pin_mux_out;

    integer errors = 0;
    integer errors2 = 0;

    mem dut (
        .clk(clk), .rst_n(rst_n),
        .addr(addr), .wdata(wdata), .size(size), .we(we),
        .valid(valid), .ready(ready), .rdata(rdata),
        .gpio_in(8'h00), .gpio_out(),
        .qspi_cs0(), .qspi_cs1(), .qspi_cs2(), .qspi_sck(),
        .qspi_mosi(), .qspi_miso(1'b0),
        .pwm_out(pwm_out), .pin_mux_out(pin_mux_out)
    );

    // ---------------- Part 2 DUT: the real top level ----------------
    reg  [7:0] ui_in2 = 8'h00;
    reg  [7:0] uio_in2 = 8'h00;
    wire [7:0] uo_out2;
    wire [7:0] uio_out2;
    wire [7:0] uio_oe2;

    tt_um_agila32 top_dut (
        .ui_in   (ui_in2),
        .uo_out  (uo_out2),
        .uio_in  (uio_in2),
        .uio_out (uio_out2),
        .uio_oe  (uio_oe2),
        .ena     (1'b1),
        .clk     (clk),
        .rst_n   (rst_n)
    );

    // Same access task as tb_mem_ext.v -- byte-sized register writes
    // (size=0) are what real firmware issues for these single-byte
    // control registers.
    task do_access(input [7:0] a, input [31:0] wd, input [1:0] sz, input do_we);
        begin
            addr  = a;
            wdata = wd;
            size  = sz;
            we    = do_we;
            valid = 1'b1;
            @(posedge clk);
            while (!ready) @(posedge clk);
            valid <= 1'b0;
            we    <= 1'b0;
            #1;
        end
    endtask

    integer k;
    integer high_count;
    reg     led7_before;

    initial begin
        rst_n = 0;
        #20 rst_n = 1;
        #10;

        // =============================================================
        // Part 1: direct bus-level test of the peripheral registers
        // =============================================================

        // -----------------------------------------------------------
        // Test 1: TIMER_LO/HI (0xF2/0xF3) reset to 0 and stay there
        // while TIMER_CTRL's enable bit is 0.
        // -----------------------------------------------------------
        do_access(8'hF2, 32'h0, 2'd0, 1'b0);
        if (rdata[7:0] !== 8'h00) begin
            errors = errors + 1;
            $display("FAIL test1: TIMER_LO read %h at reset, expected 00", rdata[7:0]);
        end else begin
            $display("PASS test1: TIMER_LO resets to 0 and doesn't run while disabled");
        end

        // -----------------------------------------------------------
        // Test 2: enable the timer (TIMER_CTRL bit0) and confirm it
        // actually counts up over a known number of cycles.
        // -----------------------------------------------------------
        do_access(8'hF5, 32'h1, 2'd0, 1'b1); // TIMER_CTRL <= enable
        for (k = 0; k < 100; k = k + 1) @(posedge clk);
        do_access(8'hF2, 32'h0, 2'd0, 1'b0);
        // The enable write itself consumes a cycle before counting
        // starts, so allow the readback a couple of cycles of slack
        // either side rather than an exact match.
        if (rdata[7:0] < 8'd95 || rdata[7:0] > 8'd103) begin
            errors = errors + 1;
            $display("FAIL test2: TIMER_LO read %0d after ~100 cycles, expected ~100", rdata[7:0]);
        end else begin
            $display("PASS test2: timer counts up while enabled (read %0d after ~100 cycles)", rdata[7:0]);
        end

        // -----------------------------------------------------------
        // Test 3: TIMER_CTRL bit1 (write-1-to-reset) zeroes the
        // counter, and disabling (bit0=0) freezes it.
        // -----------------------------------------------------------
        do_access(8'hF5, 32'h2, 2'd0, 1'b1); // reset pulse, enable left low
        do_access(8'hF2, 32'h0, 2'd0, 1'b0);
        if (rdata[7:0] !== 8'h00) begin
            errors = errors + 1;
            $display("FAIL test3: TIMER_LO read %h after reset pulse, expected 00", rdata[7:0]);
        end else begin
            $display("PASS test3: TIMER_CTRL bit1 resets the counter to 0");
        end
        for (k = 0; k < 50; k = k + 1) @(posedge clk);
        do_access(8'hF2, 32'h0, 2'd0, 1'b0);
        if (rdata[7:0] !== 8'h00) begin
            errors = errors + 1;
            $display("FAIL test3b: TIMER_LO read %0d after 50 idle cycles, expected 0 (disabled)", rdata[7:0]);
        end else begin
            $display("PASS test3b: timer stays frozen while TIMER_CTRL enable bit is 0");
        end

        // -----------------------------------------------------------
        // Test 4: TIMER_FLAG (0xF6) overflow bit sets when the 16-bit
        // counter wraps 0xFFFF -> 0x0000, and any write clears it.
        // Rather than free-running 65536 cycles from 0, deposit the
        // internal counter (a one-shot testbench nudge, not a
        // continuous force) to just below the wrap point.
        // -----------------------------------------------------------
        do_access(8'hF5, 32'h1, 2'd0, 1'b1); // re-enable
        dut.timer_cnt = 16'hFFFE; // deposit, one-shot
        for (k = 0; k < 4; k = k + 1) @(posedge clk);
        do_access(8'hF6, 32'h0, 2'd0, 1'b0);
        if (rdata[0] !== 1'b1) begin
            errors = errors + 1;
            $display("FAIL test4: TIMER_FLAG read %0d after wraparound, expected overflow=1", rdata[0]);
        end else begin
            $display("PASS test4: TIMER_FLAG overflow bit sets on 0xFFFF->0x0000 wraparound");
        end
        do_access(8'hF6, 32'h0, 2'd0, 1'b1); // any write clears it
        do_access(8'hF6, 32'h0, 2'd0, 1'b0);
        if (rdata[0] !== 1'b0) begin
            errors = errors + 1;
            $display("FAIL test4b: TIMER_FLAG read %0d after clearing write, expected 0", rdata[0]);
        end else begin
            $display("PASS test4b: writing TIMER_FLAG clears the overflow bit");
        end
        do_access(8'hF5, 32'h0, 2'd0, 1'b1); // disable again before PWM tests

        // -----------------------------------------------------------
        // Test 5: PWM_DUTY=0xFF (always-on special case) with PWM_CTRL
        // enabled should hold pwm_out high continuously.
        // -----------------------------------------------------------
        do_access(8'hF7, 32'hFF, 2'd0, 1'b1); // PWM_DUTY <= 0xFF
        do_access(8'hF9, 32'h1,  2'd0, 1'b1); // PWM_CTRL <= enable
        high_count = 0;
        for (k = 0; k < 256; k = k + 1) begin
            @(posedge clk);
            if (pwm_out) high_count = high_count + 1;
        end
        if (high_count != 256) begin
            errors = errors + 1;
            $display("FAIL test5: pwm_out high for %0d/256 cycles at DUTY=0xFF, expected 256 (always-on)", high_count);
        end else begin
            $display("PASS test5: PWM_DUTY=0xFF holds pwm_out continuously high");
        end

        // -----------------------------------------------------------
        // Test 6: PWM_DUTY=0x00 should hold pwm_out continuously low.
        // -----------------------------------------------------------
        do_access(8'hF7, 32'h00, 2'd0, 1'b1);
        high_count = 0;
        for (k = 0; k < 256; k = k + 1) begin
            @(posedge clk);
            if (pwm_out) high_count = high_count + 1;
        end
        if (high_count != 0) begin
            errors = errors + 1;
            $display("FAIL test6: pwm_out high for %0d/256 cycles at DUTY=0x00, expected 0 (always-off)", high_count);
        end else begin
            $display("PASS test6: PWM_DUTY=0x00 holds pwm_out continuously low");
        end

        // -----------------------------------------------------------
        // Test 7: mid-range PWM_DUTY (0x80) should be high for
        // approximately duty/256 of a full free-running period.
        // -----------------------------------------------------------
        do_access(8'hF7, 32'h80, 2'd0, 1'b1);
        high_count = 0;
        for (k = 0; k < 256; k = k + 1) begin
            @(posedge clk);
            if (pwm_out) high_count = high_count + 1;
        end
        if (high_count != 128) begin
            errors = errors + 1;
            $display("FAIL test7: pwm_out high for %0d/256 cycles at DUTY=0x80, expected 128", high_count);
        end else begin
            $display("PASS test7: PWM_DUTY=0x80 gives a ~50%% duty cycle (128/256 high)");
        end

        // -----------------------------------------------------------
        // Test 8: PWM_CTRL disable forces pwm_out low regardless of
        // duty.
        // -----------------------------------------------------------
        do_access(8'hF9, 32'h0, 2'd0, 1'b1); // PWM_CTRL <= disable
        high_count = 0;
        for (k = 0; k < 256; k = k + 1) begin
            @(posedge clk);
            if (pwm_out) high_count = high_count + 1;
        end
        if (high_count != 0) begin
            errors = errors + 1;
            $display("FAIL test8: pwm_out high for %0d/256 cycles with PWM_CTRL disabled, expected 0", high_count);
        end else begin
            $display("PASS test8: PWM_CTRL enable=0 forces pwm_out low regardless of duty");
        end

        $display("PART1: %0d error(s)", errors);

        // =============================================================
        // Part 2: top-level PIN_MUX wiring, confirmed against the real
        // tt_um_agila32 top module (uo_out[7] = pin_mux ? pwm_out :
        // led_out[7]).
        // =============================================================

        // Sanity: PIN_MUX defaults to 2'b00 at reset, so uo_out[7]
        // follows LED_OUT[7] (0, since the boot ROM hasn't driven LEDs
        // yet) regardless of whatever pwm_out happens to read.
        if (top_dut.pin_mux !== 2'b00) begin
            errors2 = errors2 + 1;
            $display("FAIL test9: PIN_MUX read %0d at reset, expected 00 (LED_OUT selected)", top_dut.pin_mux);
        end else begin
            $display("PASS test9: PIN_MUX defaults to 00 (uo_out[7] = LED_OUT[7]) at reset");
        end

        // Directly deposit into the peripheral's own internal PWM
        // regs (u_mem is the `mem` instance name in tt_um_agila32.v)
        // to drive a known, steady pwm_out without needing a full
        // bootloaded program -- then flip PIN_MUX the same way and
        // confirm uo_out[7] follows it. top_dut's own boot ROM has
        // been running the whole time on the shared clk/rst_n, so
        // LED_OUT[7] may already hold the self-test flag by now --
        // capture its live value as the "still on LED" expectation
        // rather than assuming it's still 0.
        led7_before = top_dut.led_out[7];
        top_dut.u_mem.pwm_duty   = 8'hFF;
        top_dut.u_mem.pwm_enable = 1'b1;
        @(posedge clk); #1;
        if (uo_out2[7] !== led7_before) begin
            errors2 = errors2 + 1;
            $display("FAIL test10: uo_out[7]=%0d with PIN_MUX=00 and pwm_out=1, expected %0d (LED_OUT[7] still selected)", uo_out2[7], led7_before);
        end else begin
            $display("PASS test10: uo_out[7] ignores pwm_out while PIN_MUX=00 (still shows LED_OUT[7]=%0d)", led7_before);
        end

        top_dut.u_mem.pin_mux = 2'b01;
        @(posedge clk); #1;
        if (uo_out2[7] !== 1'b1) begin
            errors2 = errors2 + 1;
            $display("FAIL test11: uo_out[7]=%0d with PIN_MUX=01 and pwm_out=1, expected 1", uo_out2[7]);
        end else begin
            $display("PASS test11: uo_out[7] follows pwm_out once PIN_MUX=01");
        end

        // -----------------------------------------------------------
        // Test 12: PIN_MUX=2'b10 selects the core's `halted` status
        // instead. The core hasn't executed EBREAK here, so halted
        // should read 0 and uo_out[7] should follow it (not pwm_out,
        // even though pwm_out is still driven high from test 11).
        // -----------------------------------------------------------
        top_dut.u_mem.pin_mux = 2'b10;
        @(posedge clk); #1;
        if (top_dut.halted !== 1'b0) begin
            errors2 = errors2 + 1;
            $display("FAIL test12: top_dut.halted=%0d before any EBREAK, expected 0", top_dut.halted);
        end else if (uo_out2[7] !== 1'b0) begin
            errors2 = errors2 + 1;
            $display("FAIL test12: uo_out[7]=%0d with PIN_MUX=10 and halted=0, expected 0 (ignoring pwm_out=1)", uo_out2[7]);
        end else begin
            $display("PASS test12: uo_out[7] follows halted (=0) once PIN_MUX=10, ignoring pwm_out");
        end

        // -----------------------------------------------------------
        // Test 13: reserved PIN_MUX=2'b11 falls back to LED_OUT[7],
        // same as 2'b00.
        // -----------------------------------------------------------
        top_dut.u_mem.pin_mux = 2'b11;
        @(posedge clk); #1;
        if (uo_out2[7] !== top_dut.led_out[7]) begin
            errors2 = errors2 + 1;
            $display("FAIL test13: uo_out[7]=%0d with reserved PIN_MUX=11, expected LED_OUT[7]=%0d fallback", uo_out2[7], top_dut.led_out[7]);
        end else begin
            $display("PASS test13: reserved PIN_MUX=11 falls back to LED_OUT[7]");
        end

        $display("PART2: %0d error(s)", errors2);

        if (errors == 0 && errors2 == 0)
            $display("ALL TESTS PASSED");
        else
            $display("%0d TEST(S) FAILED", errors + errors2);

        $finish;
    end

endmodule
