// pio_sm.v -- one PIO state machine (RP2040-PIO-compatible instruction set)
//
// A tiny programmable I/O engine: 9 instructions (JMP WAIT IN OUT PUSH/PULL
// MOV IRQ SET), a 5-bit program counter over a shared instruction memory, two
// scratch registers (X, Y), input/output shift registers (ISR/OSR) with
// autopush/autopull, per-instruction delay + side-set, and a fractional
// clock divider. It executes one instruction per divided-clock tick, with
// cycle-exact, deterministic timing -- which is the whole point: firmware
// running on the host CPU can *emulate* a UART/SPI/I2C/... peripheral by
// loading a short PIO program, with the bit timing done in hardware.
//
// Encoding and behaviour follow the RP2040 datasheet (section 3.4/3.5), so
// programs assembled by pioasm for the RP2040 run unmodified when they use
// pins 0-9 (see docs/info.md, "PIO", for the pin mapping and the short list
// of deliberate deviations). Instruction summary (16 bits):
//
//   [15:13] opcode   [12:8] delay / side-set   [7:0] operands
//   000 JMP   cond[7:5] addr[4:0]     cond: always,!X,X--,!Y,Y--,X!=Y,PIN,!OSRE
//   001 WAIT  pol[7] src[6:5] idx[4:0]  src: GPIO, PIN, IRQ
//   010 IN    src[7:5] count[4:0]       src: PINS,X,Y,NULL,ISR,OSR
//   011 OUT   dst[7:5] count[4:0]       dst: PINS,X,Y,NULL,PINDIRS,PC,ISR,EXEC
//   100 PUSH/PULL  [7]=pull [6]=iffull/ifempty [5]=block
//   101 MOV   dst[7:5] op[4:3] src[2:0]
//   110 IRQ   clr[6] wait[5] idx[4:0]
//   111 SET   dst[7:5] data[4:0]        dst: PINS,X,Y,PINDIRS
//
// Timing rules implemented here (all mirrored bit-for-bit by tools/pio_model.py):
//  * An instruction that completes on tick T occupies tick T; its `delay`
//    field then holds the SM idle for that many further ticks.
//  * An instruction that stalls (WAIT, blocking PUSH/PULL, autopull/autopush,
//    IRQ WAIT) re-executes every tick, with no delay counting, until it can
//    complete. Its side-set is applied on every one of those ticks.
//  * Side-set overrides an OUT/SET/MOV pin write to the same pin in the same
//    tick.
//  * Instructions forced by the host (INSTR register), and those produced by
//    OUT EXEC / MOV EXEC, run in place of the fetch and do not advance PC
//    (a JMP/OUT PC/MOV PC in them still sets it). Host-forced instructions
//    run even while the SM is disabled.
//
// The design is a single-cycle combinational "execute" block (`always @(*)`)
// that computes the next value of everything the current instruction can
// touch, plus a small sequential block that commits it on a tick.

`default_nettype none

module pio_sm #(
    parameter [1:0] SM_ID = 2'd0
) (
    input  wire        clk,
    input  wire        rst_n,

    // ---- configuration (registers live in pio.v) ----
    input  wire        en,
    input  wire [23:0] clkdiv,        // {int[15:0], frac[7:0]}; 0 means 65536.0
    input  wire [4:0]  wrap_bot,
    input  wire [4:0]  wrap_top,
    input  wire        status_sel,    // 0: TX level < N, 1: RX level < N
    input  wire [3:0]  status_n,
    input  wire [3:0]  jmp_pin,
    input  wire        side_pindir,
    input  wire        side_en,       // side-set is optional (enable bit)
    input  wire        autopush,
    input  wire        autopull,
    input  wire        in_shiftdir,   // 1 = shift right
    input  wire        out_shiftdir,  // 1 = shift right
    input  wire [4:0]  push_thresh,   // 0 means 32
    input  wire [4:0]  pull_thresh,   // 0 means 32
    input  wire [3:0]  out_base,
    input  wire [3:0]  set_base,
    input  wire [3:0]  side_base,
    input  wire [3:0]  in_base,
    input  wire [3:0]  out_count,
    input  wire [2:0]  set_count,
    input  wire [2:0]  side_count,    // includes the optional-enable bit
    input  wire        restart,       // one-cycle pulse
    input  wire        clkdiv_restart,// one-cycle pulse

    // ---- instruction memory (shared, read combinationally by pio.v) ----
    output wire [4:0]  pc_o,
    input  wire [15:0] imem_rdata,

    // ---- host-forced instruction ----
    input  wire        force_valid,
    input  wire [15:0] force_instr,
    output wire        force_done,    // one-cycle pulse when it completes

    // ---- FIFOs ----
    input  wire        tx_empty,
    input  wire [31:0] tx_data,
    output wire        tx_pop,
    input  wire        rx_full,
    output wire        rx_push,
    output wire [31:0] rx_data,
    input  wire [2:0]  tx_level,
    input  wire [2:0]  rx_level,

    // ---- pins (16-bit "PIO pin space"; pins >= 10 are unconnected) ----
    input  wire [15:0] pins_in,
    output wire [15:0] o_mask,        // pins whose output value is written
    output wire [15:0] o_val,
    output wire [15:0] d_mask,        // pins whose direction is written
    output wire [15:0] d_val,

    // ---- IRQ flags ----
    input  wire [7:0]  irq_flags,
    output wire [7:0]  irq_set,
    output wire [7:0]  irq_clr
);

    // ------------------------------------------------------------------
    // Helper functions
    // ------------------------------------------------------------------
    function [15:0] rotl16;
        input [15:0] v;
        input [3:0]  n;
        reg   [31:0] t;
        begin
            t = {v, v} << n;
            rotl16 = t[31:16];
        end
    endfunction

    function [15:0] rotr16;
        input [15:0] v;
        input [3:0]  n;
        reg   [31:0] t;
        begin
            t = {v, v} >> n;
            rotr16 = t[15:0];
        end
    endfunction

    function [15:0] cmask16;            // (1<<n)-1 for n in 0..15
        input [3:0] n;
        begin
            cmask16 = (16'd1 << n) - 16'd1;
        end
    endfunction

    function [31:0] cmask32;            // low n bits set, n in 1..32
        input [5:0] n;
        begin
            cmask32 = (n >= 6'd32) ? 32'hFFFF_FFFF : ((32'd1 << n) - 32'd1);
        end
    endfunction

    function [31:0] bitrev32;
        input [31:0] v;
        integer b;
        begin
            for (b = 0; b < 32; b = b + 1) bitrev32[b] = v[31-b];
        end
    endfunction

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    reg [4:0]  pc;
    reg [31:0] x, y, osr, isr;
    reg [5:0]  osr_cnt;        // bits shifted OUT of osr so far (32 = empty)
    reg [5:0]  isr_cnt;        // bits shifted INTO isr so far
    reg [4:0]  delay;
    reg        phase;          // mid-instruction stall state (IN autopush / IRQ wait)
    reg        exec_valid;
    reg [15:0] exec_instr;
    reg [24:0] acc;            // fractional clock-divider accumulator

    assign pc_o = pc;

    // ------------------------------------------------------------------
    // Clock divider: tick every `div` clocks on average (16.8 fixed point).
    // ------------------------------------------------------------------
    wire [24:0] div_eff = (clkdiv == 24'd0) ? 25'h100_0000 : {1'b0, clkdiv};
    wire [24:0] acc_n   = acc + 25'd256;
    wire        div_tick = (acc_n >= div_eff);

    // A forced instruction runs at the next tick when enabled, or on every
    // clock when the SM is disabled. Normal flow needs `en` and a tick.
    wire run = force_valid ? (en ? div_tick : 1'b1) : (en & div_tick);

    // ------------------------------------------------------------------
    // Instruction selection
    // ------------------------------------------------------------------
    wire use_force = force_valid;
    wire use_delay = !use_force && (delay != 5'd0);
    wire use_exec  = !use_force && !use_delay && exec_valid;
    wire src_fetch = !use_force && !use_delay && !exec_valid;
    wire [15:0] instr = use_force ? force_instr :
                        (use_exec ? exec_instr : imem_rdata);

    wire [2:0] op   = instr[15:13];
    wire [4:0] dlyf = instr[12:8];
    wire [7:0] arg  = instr[7:0];

    // ------------------------------------------------------------------
    // Delay / side-set field decode
    // ------------------------------------------------------------------
    wire [2:0] sc       = (side_count > 3'd5) ? 3'd5 : side_count;
    wire [2:0] dly_bits = 3'd5 - sc;
    wire [4:0] dly_mask = (5'd1 << dly_bits) - 5'd1;
    wire [4:0] dly_val  = dlyf & dly_mask;
    wire [4:0] side_fld = dlyf >> dly_bits;                 // sc bits wide
    wire [2:0] val_bits = (side_en && sc != 3'd0) ? (sc - 3'd1) : sc;
    wire       side_on  = (sc != 3'd0) && (!side_en || side_fld[val_bits]);
    wire [4:0] side_dat = side_fld & ((5'd1 << val_bits) - 5'd1);
    wire [15:0] s_mask  = rotl16(cmask16({1'b0, val_bits}), side_base);
    wire [15:0] s_val   = rotl16({11'b0, side_dat}, side_base) & s_mask;

    // ------------------------------------------------------------------
    // Threshold / pin-window helpers
    // ------------------------------------------------------------------
    wire [5:0] pull_thr = (pull_thresh == 5'd0) ? 6'd32 : {1'b0, pull_thresh};
    wire [5:0] push_thr = (push_thresh == 5'd0) ? 6'd32 : {1'b0, push_thresh};

    wire [15:0] outm = rotl16(cmask16(out_count), out_base);
    wire [2:0]  setc = (set_count > 3'd5) ? 3'd5 : set_count;
    wire [15:0] setm = rotl16(cmask16({1'b0, setc}), set_base);

    wire [15:0] pins_rot = rotr16(pins_in, in_base);
    wire [31:0] pins32   = {pins_rot, pins_rot};

    wire        status_hit = status_sel ? ({1'b0, rx_level} < status_n)
                                        : ({1'b0, tx_level} < status_n);
    wire [31:0] status_v   = status_hit ? 32'hFFFF_FFFF : 32'h0;

    wire [4:0]  pc_inc = (pc == wrap_top) ? wrap_bot : (pc + 5'd1);

    // relative IRQ index: bit 4 set adds the SM id to the low two bits (mod 4)
    wire [2:0]  irq_idx = {arg[2], arg[4] ? (arg[1:0] + SM_ID) : arg[1:0]};

    // ------------------------------------------------------------------
    // Execute: next-state for the current instruction
    // ------------------------------------------------------------------
    reg        stall;
    reg        jump;
    reg [4:0]  jump_pc;
    reg [31:0] n_x, n_y, n_osr, n_isr;
    reg [5:0]  n_osr_cnt, n_isr_cnt;
    reg        n_phase;
    reg        set_exec;
    reg [15:0] set_exec_instr;
    reg        c_tx_pop, c_rx_push;
    reg [31:0] c_rx_data;
    reg [7:0]  c_irq_set, c_irq_clr;
    reg [15:0] i_o_mask, i_o_val, i_d_mask, i_d_val;

    // temporaries
    reg [5:0]  cnt;
    reg [31:0] srcv, dat, mv, osr_use;
    reg [5:0]  osr_cnt_use;
    reg        refilled, take, met, pin_b;
    reg [6:0]  sum;

    always @(*) begin
        stall = 1'b0;
        jump  = 1'b0;
        jump_pc = 5'd0;
        n_x = x;  n_y = y;  n_osr = osr;  n_isr = isr;
        n_osr_cnt = osr_cnt;  n_isr_cnt = isr_cnt;
        n_phase = phase;      // only IN / IRQ own the stall phase
        set_exec = 1'b0;  set_exec_instr = 16'h0;
        c_tx_pop = 1'b0;  c_rx_push = 1'b0;  c_rx_data = isr;
        c_irq_set = 8'h0; c_irq_clr = 8'h0;
        i_o_mask = 16'h0; i_o_val = 16'h0; i_d_mask = 16'h0; i_d_val = 16'h0;
        cnt = 6'd0; srcv = 32'h0; dat = 32'h0; mv = 32'h0; osr_use = osr;
        osr_cnt_use = osr_cnt; refilled = 1'b0; take = 1'b0; met = 1'b0;
        pin_b = 1'b0; sum = 7'd0;

        case (op)
        // ---------------------------------------------------------- JMP
        3'b000: begin
            case (arg[7:5])
                3'd0: take = 1'b1;
                3'd1: take = (x == 32'd0);
                3'd2: begin take = (x != 32'd0); n_x = x - 32'd1; end
                3'd3: take = (y == 32'd0);
                3'd4: begin take = (y != 32'd0); n_y = y - 32'd1; end
                3'd5: take = (x != y);
                3'd6: take = pins_in[jmp_pin];
                default: take = (osr_cnt < pull_thr);            // !OSRE
            endcase
            if (take) begin jump = 1'b1; jump_pc = arg[4:0]; end
        end

        // --------------------------------------------------------- WAIT
        3'b001: begin
            case (arg[6:5])
                2'd0: begin                                       // GPIO
                    pin_b = arg[4] ? 1'b0 : pins_in[arg[3:0]];
                    met   = (pin_b == arg[7]);
                end
                2'd1: begin                                       // PIN
                    pin_b = pins_in[in_base + arg[3:0]];
                    met   = (pin_b == arg[7]);
                end
                2'd2: begin                                       // IRQ
                    met = (irq_flags[irq_idx] == arg[7]);
                    if (met && arg[7]) c_irq_clr[irq_idx] = 1'b1;
                end
                default: met = 1'b1;                              // reserved
            endcase
            stall = !met;
        end

        // ----------------------------------------------------------- IN
        3'b010: begin
            n_phase = 1'b0;
            cnt = (arg[4:0] == 5'd0) ? 6'd32 : {1'b0, arg[4:0]};
            case (arg[7:5])
                3'd0:    srcv = pins32;
                3'd1:    srcv = x;
                3'd2:    srcv = y;
                3'd6:    srcv = isr;
                3'd7:    srcv = osr;
                default: srcv = 32'h0;
            endcase
            if (phase) begin
                // Shift already done on the first tick; just wait for RX space.
                if (rx_full) begin
                    stall = 1'b1; n_phase = 1'b1;
                end else begin
                    c_rx_push = 1'b1; c_rx_data = isr;
                    n_isr = 32'h0; n_isr_cnt = 6'd0;
                end
            end else begin
                dat = srcv & cmask32(cnt);
                n_isr = in_shiftdir ? ((isr >> cnt) | (dat << (6'd32 - cnt)))
                                    : ((isr << cnt) | dat);
                sum = {1'b0, isr_cnt} + {1'b0, cnt};
                n_isr_cnt = (sum > 7'd32) ? 6'd32 : sum[5:0];
                if (autopush && n_isr_cnt >= push_thr) begin
                    if (rx_full) begin
                        stall = 1'b1; n_phase = 1'b1;
                    end else begin
                        c_rx_push = 1'b1; c_rx_data = n_isr;
                        n_isr = 32'h0; n_isr_cnt = 6'd0;
                    end
                end
            end
        end

        // ---------------------------------------------------------- OUT
        3'b011: begin
            cnt = (arg[4:0] == 5'd0) ? 6'd32 : {1'b0, arg[4:0]};
            if (autopull && osr_cnt >= pull_thr) begin
                if (tx_empty) stall = 1'b1;
                else begin
                    osr_use = tx_data; osr_cnt_use = 6'd0;
                    c_tx_pop = 1'b1; refilled = 1'b1;
                end
            end
            if (!stall) begin
                dat = out_shiftdir ? (osr_use & cmask32(cnt))
                                   : (osr_use >> (6'd32 - cnt));
                n_osr = out_shiftdir ? (osr_use >> cnt) : (osr_use << cnt);
                sum = {1'b0, osr_cnt_use} + {1'b0, cnt};
                n_osr_cnt = (sum > 7'd32) ? 6'd32 : sum[5:0];
                if (autopull && !refilled && n_osr_cnt >= pull_thr && !tx_empty) begin
                    n_osr = tx_data; n_osr_cnt = 6'd0; c_tx_pop = 1'b1;
                end
                case (arg[7:5])
                    3'd0: begin
                        i_o_mask = outm;
                        i_o_val  = rotl16(dat[15:0], out_base) & outm;
                    end
                    3'd1: n_x = dat;
                    3'd2: n_y = dat;
                    3'd4: begin
                        i_d_mask = outm;
                        i_d_val  = rotl16(dat[15:0], out_base) & outm;
                    end
                    3'd5: begin jump = 1'b1; jump_pc = dat[4:0]; end
                    3'd6: begin n_isr = dat; n_isr_cnt = cnt; end
                    3'd7: begin set_exec = 1'b1; set_exec_instr = dat[15:0]; end
                    default: ;                                    // NULL
                endcase
            end
        end

        // ------------------------------------------------- PUSH / PULL
        3'b100: begin
            if (!arg[7]) begin                                    // PUSH
                if (arg[6] && isr_cnt < push_thr) begin
                    // IfFull and not yet full: no-op
                end else if (rx_full) begin
                    if (arg[5]) stall = 1'b1;                     // block
                    else begin n_isr = 32'h0; n_isr_cnt = 6'd0; end // drop
                end else begin
                    c_rx_push = 1'b1; c_rx_data = isr;
                    n_isr = 32'h0; n_isr_cnt = 6'd0;
                end
            end else begin                                        // PULL
                if (arg[6] && osr_cnt < pull_thr) begin
                    // IfEmpty and not yet empty: no-op
                end else if (tx_empty) begin
                    if (arg[5]) stall = 1'b1;                     // block
                    else begin n_osr = x; n_osr_cnt = 6'd0; end   // noblock: OSR <- X
                end else begin
                    c_tx_pop = 1'b1; n_osr = tx_data; n_osr_cnt = 6'd0;
                end
            end
        end

        // ---------------------------------------------------------- MOV
        3'b101: begin
            case (arg[2:0])
                3'd0:    srcv = pins32;
                3'd1:    srcv = x;
                3'd2:    srcv = y;
                3'd5:    srcv = status_v;
                3'd6:    srcv = isr;
                3'd7:    srcv = osr;
                default: srcv = 32'h0;                            // NULL / reserved
            endcase
            case (arg[4:3])
                2'd1:    mv = ~srcv;
                2'd2:    mv = bitrev32(srcv);
                default: mv = srcv;
            endcase
            case (arg[7:5])
                3'd0: begin
                    i_o_mask = outm;
                    i_o_val  = rotl16(mv[15:0], out_base) & outm;
                end
                3'd1: n_x = mv;
                3'd2: n_y = mv;
                3'd4: begin set_exec = 1'b1; set_exec_instr = mv[15:0]; end
                3'd5: begin jump = 1'b1; jump_pc = mv[4:0]; end
                3'd6: begin n_isr = mv; n_isr_cnt = 6'd0; end
                3'd7: begin n_osr = mv; n_osr_cnt = 6'd0; end
                default: ;
            endcase
        end

        // ---------------------------------------------------------- IRQ
        3'b110: begin
            n_phase = 1'b0;
            if (phase) begin
                if (irq_flags[irq_idx]) begin stall = 1'b1; n_phase = 1'b1; end
            end else if (arg[6]) begin
                c_irq_clr[irq_idx] = 1'b1;                        // clear
            end else begin
                c_irq_set[irq_idx] = 1'b1;                        // set
                if (arg[5]) begin stall = 1'b1; n_phase = 1'b1; end // wait
            end
        end

        // ---------------------------------------------------------- SET
        default: begin
            case (arg[7:5])
                3'd0: begin
                    i_o_mask = setm;
                    i_o_val  = rotl16({11'b0, arg[4:0]}, set_base) & setm;
                end
                3'd1: n_x = {27'b0, arg[4:0]};
                3'd2: n_y = {27'b0, arg[4:0]};
                3'd4: begin
                    i_d_mask = setm;
                    i_d_val  = rotl16({11'b0, arg[4:0]}, set_base) & setm;
                end
                default: ;
            endcase
        end
        endcase
    end

    wire done = !stall;
    // Only meaningful when an instruction (not a delay slot) executes.
    wire exec_now = run && !use_delay;

    // ------------------------------------------------------------------
    // Pin / FIFO / IRQ side effects (gated to the tick that executes)
    // ------------------------------------------------------------------
    wire s_o_en = side_on && !side_pindir;
    wire s_d_en = side_on &&  side_pindir;

    // Instruction writes only happen on completion; side-set on every
    // executing tick (including stalled ones), and it wins on overlap.
    wire wr_ok = exec_now && done;
    wire [15:0] io_m = wr_ok ? i_o_mask : 16'h0;
    wire [15:0] id_m = wr_ok ? i_d_mask : 16'h0;
    wire [15:0] so_m = (exec_now && s_o_en) ? s_mask : 16'h0;
    wire [15:0] sd_m = (exec_now && s_d_en) ? s_mask : 16'h0;

    assign o_mask = io_m | so_m;
    assign o_val  = (i_o_val & io_m & ~so_m) | (s_val & so_m);
    assign d_mask = id_m | sd_m;
    assign d_val  = (i_d_val & id_m & ~sd_m) | (s_val & sd_m);

    assign tx_pop   = wr_ok && c_tx_pop;
    assign rx_push  = wr_ok && c_rx_push;
    assign rx_data  = c_rx_data;
    // IRQ WAIT raises its flag on the *first* tick even though it then
    // stalls, so IRQ flags are gated by exec_now, not by completion. (No
    // stalling path ever asserts c_irq_clr.)
    assign irq_set  = exec_now ? c_irq_set : 8'h0;
    assign irq_clr  = exec_now ? c_irq_clr : 8'h0;
    assign force_done = exec_now && use_force && done;

    // ------------------------------------------------------------------
    // Sequential commit
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc <= 5'd0;
            x <= 32'h0; y <= 32'h0; osr <= 32'h0; isr <= 32'h0;
            osr_cnt <= 6'd32; isr_cnt <= 6'd0;
            delay <= 5'd0; phase <= 1'b0;
            exec_valid <= 1'b0; exec_instr <= 16'h0;
            acc <= 25'd0;
        end else begin
            // clock divider
            if (!en || clkdiv_restart)
                acc <= 25'd0;
            else
                acc <= div_tick ? (acc_n - div_eff) : acc_n;

            if (restart) begin
                osr_cnt <= 6'd32; isr <= 32'h0; isr_cnt <= 6'd0;
                delay <= 5'd0; phase <= 1'b0; exec_valid <= 1'b0;
            end else if (run) begin
                if (use_delay) begin
                    delay <= delay - 5'd1;
                end else begin
                    x <= n_x; y <= n_y; osr <= n_osr; isr <= n_isr;
                    osr_cnt <= n_osr_cnt; isr_cnt <= n_isr_cnt;
                    phase <= n_phase;
                    if (set_exec) begin
                        exec_valid <= 1'b1; exec_instr <= set_exec_instr;
                    end else if (done && use_exec) begin
                        exec_valid <= 1'b0;
                    end
                    if (done) begin
                        delay <= dly_val;
                        if (jump)           pc <= jump_pc;
                        else if (src_fetch) pc <= pc_inc;
                    end
                end
            end
        end
    end

endmodule
