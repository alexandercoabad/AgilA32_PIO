`timescale 1ns/1ps

// tb_alu_test.v -- runs tools/build_alu_test.py's program (every
// newly-added asm_pineapple.py instruction) through the real
// rv32i_core against alu_test_mem.v, then checks every register it
// sets against hand-computed expected values via direct register-file
// access. Values are chosen (see build_alu_test.py's own header) so
// signed/unsigned and logical/arithmetic variants of the SAME opcode
// pair disagree on the test input -- a wrapper encoding the wrong
// funct3/funct7 is expected to fail loudly here, not coincidentally
// pass.

module tb_alu_test;
    reg clk = 0;
    reg rst_n;
    always #5 clk = ~clk;

    wire [7:0]  mem_addr;
    wire [31:0] mem_wdata;
    wire [1:0]  mem_size;
    wire        mem_we;
    wire        mem_valid;
    wire        mem_ready;
    wire [31:0] mem_rdata;

    rv32i_core dut (
        .clk(clk), .rst_n(rst_n),
        .mem_addr(mem_addr), .mem_wdata(mem_wdata), .mem_size(mem_size),
        .mem_we(mem_we), .mem_valid(mem_valid), .mem_ready(mem_ready),
        .mem_rdata(mem_rdata)
    );

    alu_test_mem u_mem (
        .clk(clk), .rst_n(rst_n),
        .addr(mem_addr), .wdata(mem_wdata), .size(mem_size), .we(mem_we),
        .valid(mem_valid), .ready(mem_ready), .rdata(mem_rdata)
    );

    integer failures;

    task automatic check(input [255:0] name, input [31:0] got, input [31:0] expected);
        begin
            if (got !== expected) begin
                $display("FAIL %0s: got 0x%08x, expected 0x%08x", name, got, expected);
                failures = failures + 1;
            end else begin
                $display("PASS %0s: 0x%08x", name, got);
            end
        end
    endtask

    initial begin
        failures = 0;
        rst_n = 0;
        repeat (5) @(posedge clk);
        rst_n = 1;

        // 44 instructions * 7 cycles/instruction (this core's fixed
        // multi-cycle FSM cost, all on-chip here so no extra wait
        // states) plus margin -- comfortably reaches the SPIN loop.
        repeat (44 * 7 + 200) @(posedge clk);

        check("SUB   x3",  dut.regs[3],  32'd7);
        check("SLL   x4",  dut.regs[4],  32'd16);
        check("SLT   x5",  dut.regs[5],  32'd1);
        check("SLTU  x6",  dut.regs[6],  32'd0);
        check("SRL   x7",  dut.regs[7],  32'h08000000);
        check("SRA   x8",  dut.regs[8],  32'hF8000000);
        check("SLTI  x9",  dut.regs[9],  32'd1);
        check("SLTIU x10", dut.regs[10], 32'd0);
        check("XORI  x11", dut.regs[11], 32'h000000F0);
        check("SRAI  x12", dut.regs[12], 32'hF8000000);
        check("LH    x13", dut.regs[13], 32'hFFFFFFFF);
        check("LHU   x14", dut.regs[14], 32'h0000FFFF);
        check("LB    x15", dut.regs[15], 32'hFFFFFFFF);
        check("LBU   x16", dut.regs[16], 32'h000000FF);
        check("BGE   x17", dut.regs[17], 32'd111);
        check("BLTU  x18", dut.regs[18], 32'd222);
        check("BGEU  x19", dut.regs[19], 32'd333);
        check("AUIPC x20", dut.regs[20], 32'h000000A8);

        if (failures == 0)
            $display("ALL TESTS PASSED (18/18 instruction checks)");
        else
            $display("%0d CHECK(S) FAILED", failures);

        $finish;
    end

    initial begin
        #10000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
