// fifo_props.vh -- formal properties for src/pio_fifo.v.  NOT part of the design: formal/run.sh inserts this file just
// before the last `endmodule` of a COPY of pio_fifo.v (build/pio_fifo_formal.v), so the properties can see wp / rp / mem.
// src/pio_fifo.v itself is never modified.  Inputs are left completely free every cycle (push, pop, clear, wdata, and the
// reset may fall at any time); only the first cycle is constrained to be in reset.
//
//   P1  cnt <= DEPTH; full == (cnt == DEPTH); empty == (cnt == 0); never both
//   P2  cnt next = cnt + do_push - do_pop (clear or reset -> 0); a push into a full FIFO and a pop of an empty FIFO
//       change nothing
//   P3  the pointers agree with the cnt (wp - rp == cnt mod DEPTH; cnt 0 or DEPTH -> wp == rp)
//   P4  data integrity: a word pushed is the word popped, in order, never altered, never duplicated (one free
//       symbolic word `magic` is followed through the queue and must reach the head with its value)
//   P5  clear and reset empty the queue
    // ---- environment: reset low in the first cycle, free afterwards
    reg init = 1'b1;
    always @(posedge clk) init <= 1'b0;
    always @* if (init) assume(!rst_n);

    
    

    // ---- P1
    always @* if (!init) begin
        assert(cnt <= DEPTH);
        assert(full  == (cnt == DEPTH));
        assert(empty == (cnt == 0));
        assert(!(full && empty));
        assert(level == cnt);
    end

    // ---- P2 / P5 (relate this cycle's inputs to the next cycle's cnt)
    reg past_valid = 1'b0;
    reg [DEPTH_LOG2:0] level_q; reg clear_q, rst_ok_q, dpush_q, dpop_q;
    always @(posedge clk) begin
        past_valid <= 1'b1;
        level_q <= cnt; clear_q <= clear; rst_ok_q <= rst_n; dpush_q <= do_push; dpop_q <= do_pop;
    end
    always @* if (past_valid && !init && rst_n && rst_ok_q && !clear_q)      // previous cycle was a normal cycle (and no reset now)
        assert(cnt == level_q + dpush_q - dpop_q);
    always @* if (past_valid && !init && rst_n && rst_ok_q && clear_q)       // previous cycle cleared
        assert(cnt == 0);
    always @* if (!init && !rst_n) assert(cnt == 0);              // asynchronous reset empties the queue

    // ---- P3 (internal pointers; also the induction strengthening)
    always @* if (!init) begin
        assert(wp - rp == cnt[DEPTH_LOG2-1:0]);
        if (cnt == 0 || cnt == DEPTH) assert(wp == rp);
    end

    // ---- P4: follow one symbolic word through the queue
    (* anyconst *) reg [31:0] magic;
    reg tracking = 1'b0;
    reg [DEPTH_LOG2:0] ahead;                   // entries in front of `magic`
    always @(posedge clk) begin
        if (!rst_n || clear) tracking <= 1'b0;
        else begin
            if (tracking && do_pop) begin
                if (ahead == 0) tracking <= 1'b0;
                else ahead <= ahead - 1'b1;
            end
            if (!tracking && do_push && wdata == magic) begin
                tracking <= 1'b1;
                ahead <= cnt - (do_pop ? 1'b1 : 1'b0);
            end
        end
    end
    always @* if (!init && rst_n && tracking) begin
        assert(ahead < cnt);                                        // it is still in the queue
        assert(mem[(rp + ahead[DEPTH_LOG2-1:0]) & (DEPTH - 1)] == magic);   // stored unchanged at the right slot
        if (ahead == 0) assert(rdata == magic);                       // reaches the head intact
    end

    // ---- cover: the interesting situations are reachable
    always @* if (!init) begin
        cover(full);
        cover(empty && !init && past_valid && dpush_q);
        cover(tracking && ahead == 2);
        cover(tracking && ahead == 0 && do_pop);
    end
