// tb_ebreak_halt.v -- exercises the EBREAK-halts-the-core behavior
// added to rv32i_core.v (ST_HALTED / `halted` output), ported from
// AgilA8's a8_core.v S_HALTED/`halted` pattern, plus its PIN_MUX
// (0xFA=2'b10) wiring at the top level.
//
// Part 1 drives rv32i_core directly against a tiny hand-built
// combinational instruction memory (mirroring tb_alu_test.v's style):
// a short program runs a couple of ADDIs, hits EBREAK, and the test
// confirms `halted` goes high exactly then, PC/regs freeze, and
// `halted` stays high indefinitely (not just a one-cycle pulse) until
// rst_n deasserts.
//
// Part 2 instantiates the real top level, tt_um_agila32, bootloads the
// same program over the GPIO bootloader protocol (matching tb_check.v/
// tb_flash_handoff.v's style), and confirms uo_out[7] actually shows
// the halted status once PIN_MUX (0xFA) is set to 2'b10.

`timescale 1ns/1ps
`default_nettype none

module tb_ebreak_halt;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    integer errors = 0;

    // =================================================================
    // Part 1: rv32i_core directly, against a hand-built instruction
    // memory containing: ADDI x1,x0,5 ; ADDI x2,x0,7 ; EBREAK ; ADDI x1,x0,99
    // (the last instruction must NEVER execute -- that's the whole point)
    // =================================================================
    reg  [7:0]  mem_addr;
    reg  [31:0] mem_wdata;
    reg  [1:0]  mem_size;
    reg         mem_we;
    reg         mem_valid;
    reg         mem_ready;
    reg  [31:0] mem_rdata;
    wire        halted;

    rv32i_core core1 (
        .clk       (clk),
        .rst_n     (rst_n),
        .mem_addr  (mem_addr),
        .mem_wdata (mem_wdata),
        .mem_size  (mem_size),
        .mem_we    (mem_we),
        .mem_valid (mem_valid),
        .mem_ready (mem_ready),
        .mem_rdata (mem_rdata),
        .halted    (halted)
    );

    // Tiny combinational "ROM": word-indexed by mem_addr[7:2], 4 words.
    // ADDI x1,x0,5   = imm(5)<<20 | rs1(0)<<15 | funct3(000)<<12 | rd(1)<<7 | OP_IMM(0010011)
    // ADDI x2,x0,7   = imm(7)<<20 | ...rd(2)...
    // EBREAK         = imm(1)<<20 | rs1(0) | funct3(000) | rd(0) | OP_SYSTEM(1110011)
    // ADDI x1,x0,99  = must never be fetched/executed
    localparam [31:0] I_ADDI_X1_5  = (32'd5  << 20) | (32'd0 << 15) | (3'b000 << 12) | (32'd1 << 7) | 7'b0010011;
    localparam [31:0] I_ADDI_X2_7  = (32'd7  << 20) | (32'd0 << 15) | (3'b000 << 12) | (32'd2 << 7) | 7'b0010011;
    localparam [31:0] I_EBREAK     = (32'd1  << 20) | (32'd0 << 15) | (3'b000 << 12) | (32'd0 << 7) | 7'b1110011;
    localparam [31:0] I_ADDI_X1_99 = (32'd99 << 20) | (32'd0 << 15) | (3'b000 << 12) | (32'd1 << 7) | 7'b0010011;

    always @(*) begin
        mem_ready = 1'b1; // on-chip-style: ready immediately, every cycle
        case (mem_addr[7:2])
            6'd0: mem_rdata = I_ADDI_X1_5;
            6'd1: mem_rdata = I_ADDI_X2_7;
            6'd2: mem_rdata = I_EBREAK;
            6'd3: mem_rdata = I_ADDI_X1_99;
            default: mem_rdata = 32'h0; // NOP-shaped (ADDI x0,x0,0) if ever reached
        endcase
    end

    integer k;

    initial begin
        rst_n = 0;
        #20 rst_n = 1;

        // top_dut (Part 2, declared below) shares this same rst_n with
        // Part 1's bare core -- deposit its QSPI_CTRL fast (2'd0) here,
        // right after the very first reset, so top_dut's boot-ROM self-
        // test resolves quickly during Part 1 (well before
        // bootload_and_check is ever called) instead of at the new slow
        // reset default, which this test isn't exercising. Same
        // technique this file already uses below for PIN_MUX.
        top_dut.u_mem.qspi_div_sel = 2'd0;

        // halted must be low immediately out of reset, before the core
        // has had any chance to fetch anything.
        if (halted !== 1'b0) begin
            errors = errors + 1;
            $display("FAIL test1: halted=%0d immediately after reset, expected 0", halted);
        end else begin
            $display("PASS test1: halted is 0 immediately after reset");
        end

        // Run long enough for both ADDIs, the EBREAK, and (if the bug
        // were present) the trailing ADDI x1,x0,99 to all have had time
        // to execute at 7 cycles/instruction -- give it a generous
        // margin (40 instruction-widths of cycles).
        for (k = 0; k < 40 * 7; k = k + 1) @(posedge clk);

        if (halted !== 1'b1) begin
            errors = errors + 1;
            $display("FAIL test2: halted=%0d after EBREAK should have executed, expected 1", halted);
        end else begin
            $display("PASS test2: halted goes high after EBREAK executes");
        end

        if (core1.regs[1] !== 32'd5) begin
            errors = errors + 1;
            $display("FAIL test3: x1=%0d after halt, expected 5 (the trailing ADDI x1,x0,99 must never execute)", core1.regs[1]);
        end else begin
            $display("PASS test3: x1 stayed at 5 -- the post-EBREAK instruction never executed");
        end

        if (core1.regs[2] !== 32'd7) begin
            errors = errors + 1;
            $display("FAIL test4: x2=%0d after halt, expected 7", core1.regs[2]);
        end else begin
            $display("PASS test4: x2 holds 7 as expected (second ADDI did run before EBREAK)");
        end

        // Confirm halted isn't a one-cycle pulse -- it should still be
        // high a long time later, with no more mem_valid activity.
        for (k = 0; k < 50; k = k + 1) @(posedge clk);
        if (halted !== 1'b1) begin
            errors = errors + 1;
            $display("FAIL test5: halted dropped back to 0 after extra idle cycles, expected it to stay 1");
        end else if (mem_valid !== 1'b0) begin
            errors = errors + 1;
            $display("FAIL test5b: mem_valid=%0d while halted, expected 0 (no further bus activity)", mem_valid);
        end else begin
            $display("PASS test5: halted stays high indefinitely, no further memory activity, until reset");
        end

        // Reset should bring the core back out of ST_HALTED.
        rst_n = 0;
        #20;
        if (halted !== 1'b0) begin
            errors = errors + 1;
            $display("FAIL test6: halted=%0d after rst_n asserted, expected 0", halted);
        end else begin
            $display("PASS test6: rst_n clears halted (core leaves ST_HALTED on reset)");
        end
        rst_n = 1;
        // Second reset -- re-deposit QSPI_CTRL fast for top_dut (its
        // own reset block reverts qspi_div_sel to the slow default on
        // every rst_n deassertion, same as any other reset register).
        top_dut.u_mem.qspi_div_sel = 2'd0;

        $display("PART1: %0d error(s)", errors);

        // =============================================================
        // Part 2: bootload the same program into the real top level
        // and confirm PIN_MUX=2'b10 actually surfaces `halted` on
        // uo_out[7].
        // =============================================================
        bootload_and_check;

        if (errors == 0)
            $display("ALL TESTS PASSED");
        else
            $display("%0d TEST(S) FAILED", errors);

        $finish;
    end

    // -----------------------------------------------------------------
    // Part 2 DUT: the real top level, bootloaded with the same 4-word
    // program via the GPIO bit-bang protocol (DATA=ui_in[0], CLOCK=
    // ui_in[1], START=ui_in[2]) -- same wire protocol tb_check.v/
    // tb_flash_handoff.v already exercise, so this doesn't reinvent it.
    // -----------------------------------------------------------------
    reg  [7:0] ui_in2  = 8'h00;
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

    // Timing/protocol here is copied verbatim from tb_check.v's own
    // bootload task (200 cycles/phase, length byte first, little-endian
    // bytes per instruction word) rather than re-derived -- that's the
    // validated reference for this wire protocol, not something to
    // re-guess here.
    task send_bit(input b);
        begin
            ui_in2[0] = b; ui_in2[1] = 0;
            repeat (200) @(posedge clk);
            ui_in2[1] = 1;
            repeat (200) @(posedge clk);
            ui_in2[1] = 0;
            repeat (200) @(posedge clk);
        end
    endtask

    task send_byte(input [7:0] b);
        integer bi;
        begin
            for (bi = 7; bi >= 0; bi = bi - 1) send_bit(b[bi]);
        end
    endtask

    // Sends one instruction word as 4 little-endian bytes (LSB first),
    // matching tb_check.v scenario 3's byte order.
    task send_word(input [31:0] w);
        begin
            send_byte(w[7:0]);
            send_byte(w[15:8]);
            send_byte(w[23:16]);
            send_byte(w[31:24]);
        end
    endtask

    task bootload_and_check;
        integer m;
        begin
            repeat (200) @(posedge clk);
            ui_in2[2] = 1'b1; // START
            repeat (20) @(posedge clk);

            send_byte(8'd16); // length: 4 instructions * 4 bytes = 16
            send_word(I_ADDI_X1_5);
            send_word(I_ADDI_X2_7);
            send_word(I_EBREAK);
            send_word(I_ADDI_X1_99);

            // Give the bootloaded program time to load and run (EBREAK
            // should land well within this margin).
            for (m = 0; m < 30000; m = m + 1) @(posedge clk);

            if (top_dut.halted !== 1'b1) begin
                errors = errors + 1;
                $display("FAIL test7: top_dut.halted=%0d after bootloading+running the EBREAK program, expected 1", top_dut.halted);
            end else begin
                $display("PASS test7: bootloaded program hits EBREAK and halts the real top-level core");
            end

            // PIN_MUX defaults to 00 (LED) -- uo_out[7] should NOT be
            // following halted yet even though halted=1.
            if (top_dut.pin_mux !== 2'b00) begin
                errors = errors + 1;
                $display("FAIL test8: PIN_MUX=%0d before being set, expected 00 (default)", top_dut.pin_mux);
            end else begin
                $display("PASS test8: PIN_MUX still defaults to 00 post-bootload (uo_out[7] shows LED, not halted, yet)");
            end

            // Force PIN_MUX=2'b10 directly on the peripheral register
            // (same technique tb_timer_pwm.v uses) and confirm uo_out[7]
            // follows halted.
            top_dut.u_mem.pin_mux = 2'b10;
            @(posedge clk); #1;
            if (uo_out2[7] !== 1'b1) begin
                errors = errors + 1;
                $display("FAIL test9: uo_out[7]=%0d with PIN_MUX=10 and a real halted core, expected 1", uo_out2[7]);
            end else begin
                $display("PASS test9: uo_out[7] shows the real bootloaded-and-halted core's status via PIN_MUX=10");
            end
        end
    endtask

endmodule
