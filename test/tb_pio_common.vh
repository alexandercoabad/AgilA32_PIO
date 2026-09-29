// tb_pio_common.vh -- shared helpers for the standalone PIO testbenches.
//
// `include this inside a testbench module AFTER declaring:
//     reg clk; reg rst_n; reg valid, we; reg [7:0] addr; reg [31:0] wdata;
//     wire [31:0] rdata;
// It provides a free-running clock, a self-checking `check` task, and bus
// tasks that reproduce the RV32I core's REAL bus timing (see rv32i_core.v):
//   * a store presents valid&&we for exactly one clock,
//   * a load presents valid for one clock and the core samples rdata ONE
//     CLOCK LATER (write-back) -- so pio.v defers RXF pops to that cycle.
// Mirroring that here means these tests exercise the same handshake the CPU
// does, not an idealised bus.

integer errors;
initial errors = 0;

initial clk = 1'b0;
always #5 clk = ~clk;

task check;
    input cond;
    input [8*72-1:0] msg;
    begin
        if (!cond) begin
            errors = errors + 1;
            $display("FAIL: %0s  (t=%0t)", msg, $time);
        end
    end
endtask

task bus_write;
    input [7:0]  a;
    input [31:0] d;
    begin
        @(posedge clk); #1;
        valid = 1'b1; we = 1'b1; addr = a; wdata = d;
        @(posedge clk); #1;
        valid = 1'b0; we = 1'b0; addr = 8'h00; wdata = 32'h0;
    end
endtask

reg [31:0] rd_val;
task bus_read;                     // result in rd_val
    input [7:0] a;
    begin
        @(posedge clk); #1;
        valid = 1'b1; we = 1'b0; addr = a;
        @(posedge clk); #1;
        valid = 1'b0;              // 'write-back' cycle: rdata still valid
        #3 rd_val = rdata;
        @(posedge clk);
        #1 addr = 8'h00;
    end
endtask

// indirect register access: PIO_IDX (0xFF) then PIO_DATA (0xFE)
task pio_wr;
    input [6:0]  idx;
    input [31:0] d;
    begin
        bus_write(8'hFF, {25'h0, idx});
        bus_write(8'hFE, d);
    end
endtask

task pio_rd;                       // result in rd_val
    input [6:0] idx;
    begin
        bus_write(8'hFF, {25'h0, idx});
        bus_read(8'hFE);
    end
endtask

// register-index helpers (see pio.v's map)
function [6:0] smreg;
    input integer sm;
    input integer r;
    begin
        smreg = 7'h40 + (sm * 16) + r;
    end
endfunction

localparam R_CTRL     = 7'h00;
localparam R_IRQ      = 7'h01;
localparam R_IRQ_FORCE= 7'h02;
localparam R_FSTAT    = 7'h03;
localparam R_PIN_OWN  = 7'h04;
localparam R_SYNC_BYP = 7'h05;
localparam R_PINS_IN  = 7'h06;
localparam R_PINS_OUT = 7'h07;
localparam R_INFO     = 7'h08;
localparam R_IMEM     = 7'h20;

localparam SM_CLKDIV  = 0;
localparam SM_EXEC    = 1;
localparam SM_SHIFT   = 2;
localparam SM_PINCTRL = 3;
localparam SM_INSTR   = 4;
localparam SM_ADDR    = 5;
localparam SM_TXF     = 6;
localparam SM_RXF     = 7;
localparam SM_FLEVEL  = 8;

// force-execute one instruction on state machine `sm` and wait for it to finish
task sm_exec;
    input integer sm;
    input [15:0] ins;
    integer guard;
    begin
        pio_wr(smreg(sm, SM_INSTR), {16'h0, ins});
        guard = 0;
        pio_rd(smreg(sm, SM_EXEC));
        while (rd_val[31] && guard < 100) begin
            pio_rd(smreg(sm, SM_EXEC));
            guard = guard + 1;
        end
    end
endtask

// RP2040-layout register image builders
function [31:0] pinctrl;
    input [3:0] out_base; input [3:0] out_count;
    input [3:0] set_base; input [2:0] set_count;
    input [3:0] side_base; input [2:0] side_count;
    input [3:0] in_base;
    begin
        pinctrl = {side_count, set_count, 2'b00, out_count, 1'b0, in_base,
                   1'b0, side_base, 1'b0, set_base, 1'b0, out_base};
        // NB: field positions: OUT_BASE[3:0] SET_BASE[8:5] SIDE_BASE[13:10]
        //     IN_BASE[18:15] OUT_COUNT[23:20] SET_COUNT[28:26] SIDE_COUNT[31:29]
        pinctrl = 32'h0;
        pinctrl[3:0]   = out_base;
        pinctrl[8:5]   = set_base;
        pinctrl[13:10] = side_base;
        pinctrl[18:15] = in_base;
        pinctrl[23:20] = out_count;
        pinctrl[28:26] = set_count;
        pinctrl[31:29] = side_count;
    end
endfunction

function [31:0] execctrl;
    input [4:0] wrap_bot; input [4:0] wrap_top;
    input side_en; input side_pindir; input [3:0] jmp_pin;
    begin
        execctrl = 32'h0;
        execctrl[11:7]  = wrap_bot;
        execctrl[16:12] = wrap_top;
        execctrl[27:24] = jmp_pin;
        execctrl[29]    = side_pindir;
        execctrl[30]    = side_en;
    end
endfunction

function [31:0] shiftctrl;
    input autopush; input autopull; input in_right; input out_right;
    input [4:0] push_thr; input [4:0] pull_thr;
    begin
        shiftctrl = 32'h0;
        shiftctrl[16]    = autopush;
        shiftctrl[17]    = autopull;
        shiftctrl[18]    = in_right;
        shiftctrl[19]    = out_right;
        shiftctrl[24:20] = push_thr;
        shiftctrl[29:25] = pull_thr;
    end
endfunction

function [31:0] clkdiv_reg;        // integer + 8-bit fraction
    input [15:0] int_part; input [7:0] frac;
    begin
        clkdiv_reg = {int_part, frac, 8'h00};
    end
endfunction
