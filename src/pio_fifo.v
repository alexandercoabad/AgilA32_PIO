// pio_fifo.v -- small synchronous FIFO used for each PIO state machine's
// TX (host -> SM) and RX (SM -> host) queues.
//
// Depth is DEPTH words (power of two). `rdata` is the combinational head of
// the queue (valid whenever !empty), so a consumer can look at the head in
// the same cycle it pops it. Push while full and pop while empty are both
// silently ignored -- callers are expected to check full/empty first (the
// state machine stalls on them; the host bus interface checks them too).
// A simultaneous push and pop is legal at any fill level except full+push
// (which is dropped, since `full` gates the push).
//
// `clear` empties the queue (used by the host's FIFO_CLEAR control bit).

`default_nettype none

module pio_fifo #(
    parameter DEPTH_LOG2 = 2            // 2 -> 4 entries
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clear,
    input  wire        push,
    input  wire [31:0] wdata,
    input  wire        pop,
    output wire [31:0] rdata,
    output wire        full,
    output wire        empty,
    output wire [DEPTH_LOG2:0] level
);

    localparam DEPTH = 1 << DEPTH_LOG2;

    (* mem2reg *) reg [31:0] mem [0:DEPTH-1];
    reg [DEPTH_LOG2-1:0] wp, rp;
    reg [DEPTH_LOG2:0]   cnt;

`ifndef SYNTHESIS
    integer k;
    initial for (k = 0; k < DEPTH; k = k + 1) mem[k] = 32'h0;
`endif

    assign full  = (cnt == DEPTH[DEPTH_LOG2:0]);
    assign empty = (cnt == 0);
    assign level = cnt;
    assign rdata = mem[rp];

    wire do_push = push && !full;
    wire do_pop  = pop  && !empty;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wp  <= {DEPTH_LOG2{1'b0}};
            rp  <= {DEPTH_LOG2{1'b0}};
            cnt <= {(DEPTH_LOG2+1){1'b0}};
        end else if (clear) begin
            wp  <= {DEPTH_LOG2{1'b0}};
            rp  <= {DEPTH_LOG2{1'b0}};
            cnt <= {(DEPTH_LOG2+1){1'b0}};
        end else begin
            if (do_push) wp <= wp + 1'b1;
            if (do_pop)  rp <= rp + 1'b1;
            case ({do_push, do_pop})
                2'b10:   cnt <= cnt + 1'b1;
                2'b01:   cnt <= cnt - 1'b1;
                default: cnt <= cnt;
            endcase
        end
    end

    // Data storage has no reset (contents are don't-care while empty) --
    // smaller flops than the reset-able ones used for control state.
    always @(posedge clk) begin
        if (do_push) mem[wp] <= wdata;
    end

endmodule
