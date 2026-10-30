// Formal check of src/pio_fifo.v (unmodified): the 4-deep circular-buffer FIFO is equivalent to a simple
// shift-register reference queue for EVERY input sequence (push / pop / clear / reset), including push while full
// and pop while empty (both must be ignored), simultaneous push+pop, and pointer wrap-around.
`default_nettype none
module pio_fifo_formal (input wire clk);
    localparam LOG2 = 2, DEPTH = 4;
    (* anyseq *) wire        rst_n_in;
    (* anyseq *) wire        clear;
    (* anyseq *) wire        push;
    (* anyseq *) wire [31:0] wdata;
    (* anyseq *) wire        pop;

    // reset only at the start (the design uses an asynchronous active-low reset)
    reg init = 1'b1;
    always @(posedge clk) init <= 1'b0;
    wire rst_n = ~init;

    wire [31:0] rdata;
    wire        full, empty;
    wire [LOG2:0] level;
    pio_fifo #(.DEPTH_LOG2(LOG2)) dut (.clk(clk), .rst_n(rst_n), .clear(clear), .push(push), .wdata(wdata),
                                         .pop(pop), .rdata(rdata), .full(full), .empty(empty), .level(level));

    // ---- reference model: shift-register queue, head at q0 ----
    reg [31:0] q0, q1, q2, q3;
    reg [2:0]  cnt;
    wire do_push = push && cnt != DEPTH;
    wire do_pop  = pop  && cnt != 0;
    wire [2:0] cnt_after_pop = cnt - (do_pop ? 3'd1 : 3'd0);
    reg [31:0] n0, n1, n2, n3;
    always @(*) begin
        {n0, n1, n2, n3} = do_pop ? {q1, q2, q3, 32'h0} : {q0, q1, q2, q3};
        if (do_push) case (cnt_after_pop[1:0])
            2'd0: n0 = wdata;
            2'd1: n1 = wdata;
            2'd2: n2 = wdata;
            2'd3: n3 = wdata;
        endcase
    end
    always @(posedge clk) begin
        if (init) cnt <= 3'd0;
        else if (clear) cnt <= 3'd0;
        else begin
            cnt <= cnt_after_pop + (do_push ? 3'd1 : 3'd0);
            {q0, q1, q2, q3} <= {n0, n1, n2, n3};
        end
    end

    reg past_valid = 1'b0;
    always @(posedge clk) past_valid <= 1'b1;

    // ---- properties ----
    always @(*) if (!init) begin
        assert (level == cnt);                         // fill level is exact (and so never above DEPTH)
        assert (full  == (cnt == DEPTH));
        assert (empty == (cnt == 0));
        assert (level <= DEPTH);
        if (cnt != 0) assert (rdata == q0);           // the head is the oldest word, in order, bit-exact
    end
    always @(posedge clk) if (past_valid && !init) begin
        if ($past(clear) && !$past(init)) assert (empty && level == 0);   // clear empties
    end
    always @(posedge clk) if (init) assert (empty && level == 0);          // reset state
endmodule
