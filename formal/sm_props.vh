// sm_props.vh -- formal properties for src/pio_sm.v (one PIO state machine).  NOT part of the design: formal/run.sh inserts this
// file before the last `endmodule` of a COPY (build/pio_sm_formal.v); src/pio_sm.v is never modified.
// Every input is free in every cycle: any instruction word, any configuration, any FIFO / pin / IRQ state, reset may fall at
// any time.  Only the first cycle is constrained to be in reset.  With -D CLKDIV the clock-divider timing is also checked, under
// the stated assumption that CLKDIV does not change and has an integer part >= 1.
//
//   S1  FIFO handshake: never pops an empty TX FIFO, never pushes into a full RX FIFO (the FIFO would silently drop data)
//   S2  the shift counters never exceed 32 bits
//   S3  quiet when not ticking: while the SM is disabled (and nothing is forced), or between two divided-clock ticks, or in a
//       delay slot, it drives no pin / direction, pops / pushes nothing and raises / clears no IRQ flag
//   S4  pin values only appear where the mask says the pin is written
//   S5  a stalled instruction (WAIT, blocking PUSH / PULL, IRQ WAIT ...) keeps the PC; a host-forced instruction that does not
//       jump does not move the PC
//   S6  delay slots last exactly the programmed number of ticks: after an instruction with delay d (fetch, EXEC or forced)
//       completes, the next d ticks execute nothing and the following tick executes the next instruction
//   S7  restart clears OSR/ISR counters, ISR, delay, phase and the EXEC state
//   S8  (CLKDIV) the divided clock ticks exactly every N clocks for an integer divider N (and the accumulator stays below the divisor)
    reg init = 1'b1;
    always @(posedge clk) init <= 1'b0;
    always @* if (init) assume(!rst_n);

    // ---------------------------------------------------------------- S1
    always @* if (!init) begin
        assert(!(tx_pop  && tx_empty));
        assert(!(rx_push && rx_full));
    end
    // ---------------------------------------------------------------- S2
    always @* if (!init) begin
        assert(osr_cnt <= 6'd32);
        assert(isr_cnt <= 6'd32);
    end
    // ---------------------------------------------------------------- S3
    wire quiet = !(o_mask != 0 || d_mask != 0 || tx_pop || rx_push || irq_set != 0 || irq_clr != 0 || force_done);
    always @* if (!init) begin
        if (!run)       assert(quiet);                 // run is false while disabled (unless forced) and between ticks
        if (!en && !force_valid) assert(!run);
        if (use_delay)  assert(quiet);                 // delay slot
    end
    // ---------------------------------------------------------------- S4
    always @* if (!init) begin
        assert((o_val & ~o_mask) == 16'h0);
        assert((d_val & ~d_mask) == 16'h0);
    end

    // ---------------------------------------------------------------- history for S5 - S7
    reg past_valid = 1'b0;
    reg [4:0] pc_q; reg run_q, done_q, use_force_q, use_delay_q, restart_q, rst_ok_q, jump_q, exec_now_q;
    reg [4:0] dly_q;
    always @(posedge clk) begin
        past_valid <= 1'b1;
        pc_q <= pc; run_q <= run; done_q <= done; use_force_q <= use_force; use_delay_q <= use_delay;
        restart_q <= restart; rst_ok_q <= rst_n; jump_q <= jump; exec_now_q <= exec_now; dly_q <= dly_val;
    end
    wire normal_prev = past_valid && !init && rst_n && rst_ok_q && !restart_q;      // the previous cycle was an ordinary cycle

    // ---------------------------------------------------------------- S5
    always @* if (normal_prev) begin
        if (run_q && !use_delay_q && !done_q) assert(pc == pc_q);                    // stalled: PC holds
        if (run_q && !use_delay_q && use_force_q && !jump_q) assert(pc == pc_q);     // forced non-jump: PC holds
        if (!run_q) assert(pc == pc_q);                                              // no tick: nothing moves
        if (run_q && use_delay_q) assert(pc == pc_q);                                // delay slot: PC holds
    end
    // ---------------------------------------------------------------- S6
    // `expect` counts the idle ticks still owed after the last completed instruction.
    reg [4:0] owed = 5'd0;
    always @(posedge clk) begin
        if (!rst_n || restart) owed <= 5'd0;
        else if (run && !use_delay && done) owed <= dly_val;       // an instruction completed on this tick: its delay starts
        else if (run && use_delay) owed <= owed - 5'd1;
    end
    always @* if (!init && rst_n && !force_valid) begin
        assert(delay == owed);                                       // the RTL delay counter is exactly the delay the program asked for
        if (run && owed != 0) assert(use_delay);                     // idle while owed
        if (run && owed == 0) assert(!use_delay);                    // then execute
    end
    // ---------------------------------------------------------------- S7
    always @* if (!init && past_valid && rst_n && rst_ok_q && restart_q) begin
        assert(osr_cnt == 6'd32);
        assert(isr_cnt == 6'd0);
        assert(isr == 32'h0);
        assert(delay == 5'd0);
        assert(!phase);
        assert(!exec_valid);
    end

`ifdef CLKDIV
    // ---------------------------------------------------------------- S8
    wire [15:0] n_int = clkdiv[23:8];
    reg [23:0] clkdiv_q; reg have_q = 1'b0;
    always @(posedge clk) begin clkdiv_q <= clkdiv; have_q <= 1'b1; end
    always @* if (!init) begin
        assume(clkdiv[23:8] != 16'h0 || clkdiv == 24'h0);           // legal divider: integer part >= 1 (or 0 = 65536.0)
        if (have_q) assume(clkdiv == clkdiv_q);                      // the divider does not change
`ifdef CLKDIV_SMALL
        assume(clkdiv[23:11] == 13'h0);                              // proof limited to dividers below 8.0 (the solver cannot reach the full 24-bit range)
`endif
    end
`ifdef CLKDIV_ACC
    always @* if (!init) assert(acc < div_eff);                      // accumulator stays below the divisor
`endif
`ifdef CLKDIV_TICK
    // frac == 0: tick exactly every N clocks.  `since` counts clocks since the divider was (re)started or last ticked.
    reg [24:0] since = 25'd0;
    always @(posedge clk) begin
        if (!rst_n || !en || clkdiv_restart) since <= 25'd0;
        else if (div_tick) since <= 25'd0;
        else since <= since + 25'd1;
    end
    always @* if (!init && rst_n && clkdiv[7:0] == 8'h0) begin
        assert(acc == {since[16:0], 8'h0});                          // accumulator is 256 * clocks since the last tick
        assert(div_tick == (since + 25'd1 == (div_eff >> 8)));       // tick on the N-th clock, and only then
    end
`endif
`endif

    // ---------------------------------------------------------------- reachability (cover tasks)
    always @* if (!init) begin
        cover(run && !use_delay && done && o_mask != 0);
        cover(tx_pop);
        cover(rx_push);
        cover(use_delay);
        cover(run && !done);
    end
