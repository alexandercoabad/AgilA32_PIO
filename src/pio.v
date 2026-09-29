// pio.v -- programmable I/O block: N state machines sharing one instruction
// memory, plus the host register interface, FIFOs, pin muxing and IRQ flags.
//
// The RV32I host reaches all of this through TWO memory-mapped bytes (the
// CPU's 8-bit address space is otherwise full):
//
//   0xFF  PIO_IDX   (R/W)  bits[6:0] = register index, bit 7 = auto-increment
//   0xFE  PIO_DATA  (R/W)  32-bit access to the register selected by PIO_IDX
//
// Use LW/SW (or SH/LH for 16-bit instruction words) on 0xFE. The core has no
// alignment trap and mem.v-style decoding is by exact address, so a word
// access at 0xFE works even though it is not 4-byte aligned. With the
// auto-increment bit set, every PIO_DATA access steps PIO_IDX by one, so a
// whole program can be streamed into instruction memory with back-to-back
// stores and no index writes in between.
//
// Register index map (PIO_IDX[6:0]):
//   0x00 CTRL      [3:0] SM_ENABLE   [7:4] SM_RESTART (W1, self-clearing)
//                  [11:8] CLKDIV_RESTART (W1)   [15:12] FIFO_CLEAR (W1)
//   0x01 IRQ       [7:0] flags; write 1s to clear
//   0x02 IRQ_FORCE [7:0] write 1s to set flags
//   0x03 FSTAT     RXFULL[3:0] RXEMPTY[11:8] TXFULL[19:16] TXEMPTY[27:24]
//   0x04 PIN_OWN   [9:0] 1 = pin is driven by PIO instead of the CPU/LED logic
//   0x05 SYNC_BYP  [9:0] 1 = bypass the 2-flop input synchroniser for that pin
//   0x06 PINS_IN   [9:0] (read) input pins as the state machines see them
//   0x07 PINS_OUT  (read) [9:0] output values, [25:16] output enables
//   0x08 INFO      (read) [3:0] N_SM  [15:8] IMEM_DEPTH  [23:16] FIFO_DEPTH  [31:24] version
//   0x20-0x3F      instruction memory word 0..31 (16-bit)
//   0x40 + 0x10*n  state machine n registers:
//        +0 CLKDIV    [31:16] int, [15:8] frac (RP2040 layout)
//        +1 EXECCTRL  RP2040 layout: STATUS_N[3:0] STATUS_SEL[4] WRAP_BOTTOM[11:7]
//                     WRAP_TOP[16:12] JMP_PIN[27:24] SIDE_PINDIR[29] SIDE_EN[30]
//                     [31] (read) = a host-forced instruction is still pending
//        +2 SHIFTCTRL AUTOPUSH[16] AUTOPULL[17] IN_SHIFTDIR[18] OUT_SHIFTDIR[19]
//                     PUSH_THRESH[24:20] PULL_THRESH[29:25]
//        +3 PINCTRL   OUT_BASE[3:0] SET_BASE[8:5] SIDESET_BASE[13:10]
//                     IN_BASE[18:15] OUT_COUNT[23:20] SET_COUNT[28:26]
//                     SIDESET_COUNT[31:29]
//        +4 INSTR     (write) execute this 16-bit instruction now
//        +5 ADDR      (read) current program counter
//        +6 TXF       (write) push a word into the TX FIFO
//        +7 RXF       (read) pop a word from the RX FIFO (0 if empty)
//        +8 FLEVEL    (read) TX level[3:0], RX level[7:4]
//
// PIO pin space (4-bit pin numbers, only 0-9 are connected):
//   pin 0-7 : OUT -> uo_out[n] (push-pull, direction ignored)   IN <- ui_in[n]
//   pin 8-9 : true bidirectional pad uio[4], uio[5] (open-drain capable: the
//             PINDIR bit is the real output enable)               IN <- uio_in
// A pin is only driven by PIO once its PIN_OWN bit is set.
//
// Bus-timing notes (see rv32i_core.v): a store presents valid&&we for exactly
// one cycle, so writes take effect on that edge. A load presents valid for one
// cycle but the core samples rdata one cycle *later* (write-back), so the
// side effects of a read (RXF pop, auto-increment) are deliberately delayed
// one cycle (`rd_pend`) so rdata is still the pre-pop value when sampled.

`default_nettype none

module pio #(
    parameter N_SM       = 2,         // 1..4 state machines
    parameter FIFO_LOG2  = 2          // FIFO depth = 2**FIFO_LOG2 words
) (
    input  wire        clk,
    input  wire        rst_n,

    // CPU bus (tapped directly off the core)
    input  wire        valid,
    input  wire        we,
    input  wire [7:0]  addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output wire        sel,           // addr is 0xFE or 0xFF -> use `rdata`

    // pads
    input  wire [9:0]  pins_raw,      // {uio_in[5:4], ui_in[7:0]}
    output wire [9:0]  pin_out,
    output wire [9:0]  pin_dir,
    output wire [9:0]  pin_own
);

    localparam IMEM_DEPTH = 32;
    localparam FDEPTH     = (1 << FIFO_LOG2);
    localparam [7:0] INFO_NSM  = N_SM;
    localparam [7:0] INFO_FD   = FDEPTH;
    localparam [7:0] INFO_IMEM = IMEM_DEPTH;

    // ------------------------------------------------------------------
    // Bus decode
    // ------------------------------------------------------------------
    wire sel_data = (addr == 8'hFE);
    wire sel_idx  = (addr == 8'hFF);
    assign sel    = sel_data | sel_idx;

    wire wr_data = valid &&  we && sel_data;
    wire wr_idx  = valid &&  we && sel_idx;
    wire rd_data = valid && !we && sel_data;

    reg [6:0] idx;
    reg       autoinc;
    reg       rd_pend;

    wire idx_glob = (idx[6:5] == 2'b00);
    wire idx_imem = (idx[6:5] == 2'b01);
    wire idx_sm   = idx[6];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            idx     <= 7'h00;
            autoinc <= 1'b0;
            rd_pend <= 1'b0;
        end else begin
            rd_pend <= rd_data;
            if (wr_idx) begin
                idx     <= wdata[6:0];
                autoinc <= wdata[7];
            end else if (autoinc && (wr_data || rd_pend)) begin
                idx <= idx + 7'd1;
            end
        end
    end

    // ------------------------------------------------------------------
    // Instruction memory (shared). No reset: contents are host-loaded.
    // ------------------------------------------------------------------
    (* mem2reg *) reg [15:0] imem [0:IMEM_DEPTH-1];
`ifndef SYNTHESIS
    integer ii;
    initial for (ii = 0; ii < IMEM_DEPTH; ii = ii + 1) imem[ii] = 16'h0000; // jmp 0
`endif
    always @(posedge clk) begin
        if (wr_data && idx_imem) imem[idx[4:0]] <= wdata[15:0];
    end

    // ------------------------------------------------------------------
    // Global registers
    // ------------------------------------------------------------------
    reg [3:0] sm_en;
    reg [3:0] restart_q, clkdiv_restart_q, fifo_clear_q;
    // Bits [3:2] are only consumed when N_SM > 2 (default N_SM = 2).
    wire _unused_hi_ctrl = &{1'b0, restart_q[3:2], clkdiv_restart_q[3:2], fifo_clear_q[3:2]};
    reg [9:0] own_q, bypass_q;
    reg [7:0] irq_q;

    wire wr_glob = wr_data && idx_glob;
    wire g_ctrl  = wr_glob && (idx[3:0] == 4'h0);
    wire g_irq   = wr_glob && (idx[3:0] == 4'h1);
    wire g_irqf  = wr_glob && (idx[3:0] == 4'h2);
    wire g_own   = wr_glob && (idx[3:0] == 4'h4);
    wire g_byp   = wr_glob && (idx[3:0] == 4'h5);

    wire [7:0] irq_set_all, irq_clr_all;   // OR of the SMs' requests (below)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sm_en            <= 4'h0;
            restart_q        <= 4'h0;
            clkdiv_restart_q <= 4'h0;
            fifo_clear_q     <= 4'h0;
            own_q            <= 10'h0;
            bypass_q         <= 10'h0;
            irq_q            <= 8'h0;
        end else begin
            // one-cycle pulses
            restart_q        <= g_ctrl ? wdata[7:4]   : 4'h0;
            clkdiv_restart_q <= g_ctrl ? wdata[11:8]  : 4'h0;
            fifo_clear_q     <= g_ctrl ? wdata[15:12] : 4'h0;
            if (g_ctrl) sm_en <= wdata[3:0];
            if (g_own)  own_q <= wdata[9:0];
            if (g_byp)  bypass_q <= wdata[9:0];
            irq_q <= (irq_q & ~(irq_clr_all | (g_irq ? wdata[7:0] : 8'h0)))
                     | irq_set_all | (g_irqf ? wdata[7:0] : 8'h0);
        end
    end

    // ------------------------------------------------------------------
    // Input synchronisers (2 flops per pin, per-pin bypass)
    // ------------------------------------------------------------------
    reg [9:0] pin_out_q, pin_dir_q;   // pad output value / output-enable state
    reg [9:0] sync1, sync2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync1 <= 10'h0;
            sync2 <= 10'h0;
        end else begin
            sync1 <= pins_raw;
            sync2 <= sync1;
        end
    end
    wire [9:0]  pins_eff = (pins_raw & bypass_q) | (sync2 & ~bypass_q);
    wire [15:0] pins_in  = {6'b0, pins_eff};

    // ------------------------------------------------------------------
    // State machines
    // ------------------------------------------------------------------
    wire [N_SM*16-1:0] o_mask_v, o_val_v, d_mask_v, d_val_v;
    wire [N_SM*8-1:0]  irq_set_v, irq_clr_v;
    wire [N_SM*32-1:0] sm_rdata_v;
    wire [N_SM-1:0]    tx_empty_v, tx_full_v, rx_empty_v, rx_full_v;

    genvar g;
    generate
        for (g = 0; g < N_SM; g = g + 1) begin : sm_gen
            // ---- per-SM configuration registers ----
            reg [23:0] clkdiv;
            reg [3:0]  status_n;
            reg        status_sel;
            reg [4:0]  wrap_bot, wrap_top;
            reg [3:0]  jmp_pin;
            reg        side_pindir, side_en;
            reg        autopush, autopull, in_shiftdir, out_shiftdir;
            reg [4:0]  push_thresh, pull_thresh;
            reg [3:0]  out_base, set_base, side_base, in_base, out_count;
            reg [2:0]  set_count, side_count;
            reg        force_valid;
            reg [15:0] force_instr;

            wire mine   = idx_sm && (idx[5:4] == g);
            wire wr_sm  = wr_data && mine;
            wire [3:0] r = idx[3:0];

            wire force_done;

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    clkdiv       <= 24'h000100;          // 1.0
                    status_n     <= 4'd0;
                    status_sel   <= 1'b0;
                    wrap_bot     <= 5'd0;
                    wrap_top     <= 5'd31;
                    jmp_pin      <= 4'd0;
                    side_pindir  <= 1'b0;
                    side_en      <= 1'b0;
                    autopush     <= 1'b0;
                    autopull     <= 1'b0;
                    in_shiftdir  <= 1'b1;
                    out_shiftdir <= 1'b1;
                    push_thresh  <= 5'd0;
                    pull_thresh  <= 5'd0;
                    out_base     <= 4'd0;
                    set_base     <= 4'd0;
                    side_base    <= 4'd0;
                    in_base      <= 4'd0;
                    out_count    <= 4'd0;
                    set_count    <= 3'd5;
                    side_count   <= 3'd0;
                    force_valid  <= 1'b0;
                    force_instr  <= 16'h0;
                end else begin
                    if (force_done) force_valid <= 1'b0;
                    if (wr_sm) begin
                        case (r)
                            4'h0: clkdiv <= wdata[31:8];
                            4'h1: begin
                                status_n    <= wdata[3:0];
                                status_sel  <= wdata[4];
                                wrap_bot    <= wdata[11:7];
                                wrap_top    <= wdata[16:12];
                                jmp_pin     <= wdata[27:24];
                                side_pindir <= wdata[29];
                                side_en     <= wdata[30];
                            end
                            4'h2: begin
                                autopush     <= wdata[16];
                                autopull     <= wdata[17];
                                in_shiftdir  <= wdata[18];
                                out_shiftdir <= wdata[19];
                                push_thresh  <= wdata[24:20];
                                pull_thresh  <= wdata[29:25];
                            end
                            4'h3: begin
                                out_base   <= wdata[3:0];
                                set_base   <= wdata[8:5];
                                side_base  <= wdata[13:10];
                                in_base    <= wdata[18:15];
                                out_count  <= wdata[23:20];
                                set_count  <= wdata[28:26];
                                side_count <= wdata[31:29];
                            end
                            4'h4: begin
                                force_valid <= 1'b1;      // wins over force_done
                                force_instr <= wdata[15:0];
                            end
                            default: ;
                        endcase
                    end
                end
            end

            // ---- FIFOs ----
            wire        tx_pop, rx_push;
            wire [31:0] rx_wdata, tx_rdata, rx_rdata;
            wire [FIFO_LOG2:0] tx_level, rx_level;
            wire        tx_empty, tx_full, rx_empty, rx_full;
            wire [2:0]  tx_level3 = tx_level;   // FIFO_LOG2 <= 2
            wire [2:0]  rx_level3 = rx_level;

            wire tx_host_push = wr_sm && (r == 4'h6);
            wire rx_host_pop  = rd_pend && mine && (r == 4'h7);

            pio_fifo #(.DEPTH_LOG2(FIFO_LOG2)) u_txf (
                .clk(clk), .rst_n(rst_n), .clear(fifo_clear_q[g]),
                .push(tx_host_push), .wdata(wdata),
                .pop(tx_pop), .rdata(tx_rdata),
                .full(tx_full), .empty(tx_empty), .level(tx_level));

            pio_fifo #(.DEPTH_LOG2(FIFO_LOG2)) u_rxf (
                .clk(clk), .rst_n(rst_n), .clear(fifo_clear_q[g]),
                .push(rx_push), .wdata(rx_wdata),
                .pop(rx_host_pop), .rdata(rx_rdata),
                .full(rx_full), .empty(rx_empty), .level(rx_level));

            assign tx_empty_v[g] = tx_empty;
            assign tx_full_v[g]  = tx_full;
            assign rx_empty_v[g] = rx_empty;
            assign rx_full_v[g]  = rx_full;

            // ---- the state machine ----
            wire [4:0]  pc;
            wire [15:0] imem_rd = imem[pc];

            pio_sm #(.SM_ID(g)) u_sm (
                .clk(clk), .rst_n(rst_n),
                .en(sm_en[g]), .clkdiv(clkdiv),
                .wrap_bot(wrap_bot), .wrap_top(wrap_top),
                .status_sel(status_sel), .status_n(status_n),
                .jmp_pin(jmp_pin), .side_pindir(side_pindir), .side_en(side_en),
                .autopush(autopush), .autopull(autopull),
                .in_shiftdir(in_shiftdir), .out_shiftdir(out_shiftdir),
                .push_thresh(push_thresh), .pull_thresh(pull_thresh),
                .out_base(out_base), .set_base(set_base),
                .side_base(side_base), .in_base(in_base),
                .out_count(out_count), .set_count(set_count),
                .side_count(side_count),
                .restart(restart_q[g]), .clkdiv_restart(clkdiv_restart_q[g]),
                .pc_o(pc), .imem_rdata(imem_rd),
                .force_valid(force_valid), .force_instr(force_instr),
                .force_done(force_done),
                .tx_empty(tx_empty), .tx_data(tx_rdata), .tx_pop(tx_pop),
                .rx_full(rx_full), .rx_push(rx_push), .rx_data(rx_wdata),
                .tx_level(tx_level3),
                .rx_level(rx_level3),
                .pins_in(pins_in),
                .o_mask(o_mask_v[g*16 +: 16]), .o_val(o_val_v[g*16 +: 16]),
                .d_mask(d_mask_v[g*16 +: 16]), .d_val(d_val_v[g*16 +: 16]),
                .irq_flags(irq_q),
                .irq_set(irq_set_v[g*8 +: 8]), .irq_clr(irq_clr_v[g*8 +: 8])
            );

            // ---- register read-back ----
            reg [31:0] rd;
            always @(*) begin
                rd = 32'h0;
                case (r)
                    4'h0: rd[31:8] = clkdiv;
                    4'h1: begin
                        rd[3:0]   = status_n;
                        rd[4]     = status_sel;
                        rd[11:7]  = wrap_bot;
                        rd[16:12] = wrap_top;
                        rd[27:24] = jmp_pin;
                        rd[29]    = side_pindir;
                        rd[30]    = side_en;
                        rd[31]    = force_valid;
                    end
                    4'h2: begin
                        rd[16]    = autopush;
                        rd[17]    = autopull;
                        rd[18]    = in_shiftdir;
                        rd[19]    = out_shiftdir;
                        rd[24:20] = push_thresh;
                        rd[29:25] = pull_thresh;
                    end
                    4'h3: begin
                        rd[3:0]   = out_base;
                        rd[8:5]   = set_base;
                        rd[13:10] = side_base;
                        rd[18:15] = in_base;
                        rd[23:20] = out_count;
                        rd[28:26] = set_count;
                        rd[31:29] = side_count;
                    end
                    4'h5: rd[4:0] = pc;
                    4'h7: rd = rx_empty ? 32'h0 : rx_rdata;
                    4'h8: begin
                        rd[2:0] = tx_level3;
                        rd[6:4] = rx_level3;
                    end
                    default: ;
                endcase
            end
            assign sm_rdata_v[g*32 +: 32] = rd;
        end
    endgenerate

    // ------------------------------------------------------------------
    // IRQ flag aggregation (OR of all state machines)
    // ------------------------------------------------------------------
    reg [7:0] irq_set_r, irq_clr_r;
    integer a;
    always @(*) begin
        irq_set_r = 8'h0;
        irq_clr_r = 8'h0;
        for (a = 0; a < N_SM; a = a + 1) begin
            irq_set_r = irq_set_r | irq_set_v[a*8 +: 8];
            irq_clr_r = irq_clr_r | irq_clr_v[a*8 +: 8];
        end
    end
    assign irq_set_all = irq_set_r;
    assign irq_clr_all = irq_clr_r;

    // ------------------------------------------------------------------
    // Pin merge: state machines apply in index order, so the highest-
    // numbered SM wins when two write the same pin in the same tick.
    // ------------------------------------------------------------------
    reg [15:0] pin_out_n, pin_dir_n;
    integer b;
    always @(*) begin
        pin_out_n = {6'b0, pin_out_q};
        pin_dir_n = {6'b0, pin_dir_q};
        for (b = 0; b < N_SM; b = b + 1) begin
            pin_out_n = (pin_out_n & ~o_mask_v[b*16 +: 16]) |
                        (o_val_v[b*16 +: 16] & o_mask_v[b*16 +: 16]);
            pin_dir_n = (pin_dir_n & ~d_mask_v[b*16 +: 16]) |
                        (d_val_v[b*16 +: 16] & d_mask_v[b*16 +: 16]);
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pin_out_q <= 10'h0;
            pin_dir_q <= 10'h0;
        end else begin
            pin_out_q <= pin_out_n[9:0];
            pin_dir_q <= pin_dir_n[9:0];
        end
    end

    assign pin_out = pin_out_q;
    assign pin_dir = pin_dir_q;
    assign pin_own = own_q;

    // ------------------------------------------------------------------
    // Global read-back + top-level read mux
    // ------------------------------------------------------------------
    reg [31:0] g_rd;
    always @(*) begin
        case (idx[3:0])
            4'h0: g_rd = {16'h0, 4'h0, 4'h0, 4'h0, sm_en};
            4'h1: g_rd = {24'h0, irq_q};
            4'h3: g_rd = {4'h0, pad4(tx_empty_v), 4'h0, pad4(tx_full_v),
                          4'h0, pad4(rx_empty_v), 4'h0, pad4(rx_full_v)};
            4'h4: g_rd = {22'h0, own_q};
            4'h5: g_rd = {22'h0, bypass_q};
            4'h6: g_rd = {22'h0, pins_eff};
            4'h7: g_rd = {6'h0, pin_dir_q, 6'h0, pin_out_q};
            4'h8: g_rd = {8'h01, INFO_FD, INFO_IMEM, INFO_NSM};
            default: g_rd = 32'h0;
        endcase
    end

    function [3:0] pad4;
        input [N_SM-1:0] v;
        begin
            pad4 = 4'h0;
            pad4[N_SM-1:0] = v;
        end
    endfunction

    reg [31:0] sm_rd_sel;
    integer c;
    always @(*) begin
        sm_rd_sel = 32'h0;
        for (c = 0; c < N_SM; c = c + 1)
            if (idx[5:4] == c[1:0]) sm_rd_sel = sm_rdata_v[c*32 +: 32];
    end

    always @(*) begin
        if (sel_idx)          rdata = {24'h0, autoinc, idx};
        else if (idx_sm)      rdata = sm_rd_sel;
        else if (idx_imem)    rdata = {16'h0, imem[idx[4:0]]};
        else                  rdata = g_rd;
    end

endmodule
