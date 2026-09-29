// tb_mem_ext.v -- exercises mem.v's external PSRAM window (0xB0-0xEF)
// through the REAL qspi_shared_engine and a behavioral SPI RAM model,
// driving the addr/wdata/size/we/valid bus exactly the way the core's
// FSM does (hold `valid` high until `ready`). This is the integration
// test that tb_qspi_engine.v (engine-only) and test.py (on-chip-only,
// via the real core) don't individually cover.

`timescale 1ns/1ps
`default_nettype none

module tb_mem_ext;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    reg  [7:0]  addr = 0;
    reg  [31:0] wdata = 0;
    reg  [1:0]  size = 0;
    reg         we = 0;
    reg         valid = 0;
    wire        ready;
    wire [31:0] rdata;

    wire qspi_cs0, qspi_cs1, qspi_cs2, qspi_sck, qspi_mosi, qspi_miso_a, qspi_miso_b;

    // Two independent behavioral PSRAM models, one per chip select --
    // MISO is only driven by whichever model currently has its own
    // cs_n asserted low (spi_ram_model tri-states miso otherwise, same
    // as a real chip would), so ORing/muxing isn't needed: whichever
    // one is deselected simply doesn't drive the shared line.
    wire qspi_miso = qspi_cs1 == 1'b0 ? qspi_miso_a :
                      qspi_cs2 == 1'b0 ? qspi_miso_b : 1'bz;

    integer errors = 0;
    integer wait_cycles;

    mem dut (
        .clk(clk), .rst_n(rst_n),
        .addr(addr), .wdata(wdata), .size(size), .we(we),
        .valid(valid), .ready(ready), .rdata(rdata),
        .gpio_in(8'h00), .gpio_out(),
        .qspi_cs0(qspi_cs0), .qspi_cs1(qspi_cs1), .qspi_cs2(qspi_cs2), .qspi_sck(qspi_sck),
        .qspi_mosi(qspi_mosi), .qspi_miso(qspi_miso)
    );

    spi_ram_model ram_a (
        .cs_n(qspi_cs1),
        .sck(qspi_sck),
        .mosi(qspi_mosi),
        .miso(qspi_miso_a)
    );

    spi_ram_model ram_b (
        .cs_n(qspi_cs2),
        .sck(qspi_sck),
        .mosi(qspi_mosi),
        .miso(qspi_miso_b)
    );

    // Drives one access exactly the way rv32i_core's FSM does: set the
    // bus, assert valid, count cycles until ready, then drop valid.
    task do_access(input [7:0] a, input [31:0] wd, input [1:0] sz, input do_we);
        begin
            addr  = a;
            wdata = wd;
            size  = sz;
            we    = do_we;
            valid = 1'b1;
            wait_cycles = 0;
            @(posedge clk);
            while (!ready) begin
                @(posedge clk);
                wait_cycles = wait_cycles + 1;
            end
            valid <= 1'b0;
            we    <= 1'b0;
            #1;
        end
    endtask

    initial begin
        rst_n = 0;
        #20 rst_n = 1;
        #10;

        // -----------------------------------------------------------
        // Test 1: on-chip ROM read (addr 0) should be ready with ZERO
        // wait cycles -- confirms the external engine being present at
        // all doesn't slow down/break on-chip accesses.
        // -----------------------------------------------------------
        do_access(8'h00, 32'h0, 2'd2, 1'b0);
        if (wait_cycles != 0) begin
            errors = errors + 1;
            $display("FAIL test1: on-chip access took %0d wait cycles, expected 0", wait_cycles);
        end else begin
            $display("PASS test1: on-chip access has zero added latency (%0d wait cycles)", wait_cycles);
        end

        // -----------------------------------------------------------
        // Test 2: word WRITE to the external window (addr 0xE0, the
        // first word of the new 0xE0-0xEF PSRAM window -- moved here
        // when the address map was corrected: ROM/RAM/PSRAM had been
        // overlapping, see mem.v's header comment), through the real
        // engine + behavioral SPI RAM. Should take many cycles
        // (genuine multi-cycle SPI transaction), not 0.
        // -----------------------------------------------------------
        do_access(8'hE0, 32'hCAFEBABE, 2'd2, 1'b1);
        if (wait_cycles < 10) begin
            errors = errors + 1;
            $display("FAIL test2: external write only took %0d wait cycles, expected a real multi-cycle SPI transaction", wait_cycles);
        end else begin
            $display("PASS test2: external write took %0d wait cycles (genuine SPI transaction, not instant)", wait_cycles);
        end

        // -----------------------------------------------------------
        // Test 3: word READ back from the same address. Confirms the
        // write actually landed in the model's memory and the read
        // path (engine + mem.v's in_ext mux) reconstructs it correctly.
        // -----------------------------------------------------------
        do_access(8'hE0, 32'h0, 2'd2, 1'b0);
        if (rdata !== 32'hCAFEBABE) begin
            errors = errors + 1;
            $display("FAIL test3: read back %h, expected CAFEBABE", rdata);
        end else begin
            $display("PASS test3: read back CAFEBABE, write-then-read round trip through mem.v works");
        end

        // -----------------------------------------------------------
        // Test 4: byte-sized write+read at a different external
        // address (0xE4, still within the 0xE0-0xEF window, distinct
        // from test 2/3's word at 0xE0-0xE3), to check size=byte isn't
        // broken by the word-sized tests above.
        // -----------------------------------------------------------
        do_access(8'hE4, 32'h000000A5, 2'd0, 1'b1);
        do_access(8'hE4, 32'h0, 2'd0, 1'b0);
        if (rdata[7:0] !== 8'hA5) begin
            errors = errors + 1;
            $display("FAIL test4: byte read back %h, expected A5", rdata[7:0]);
        end else begin
            $display("PASS test4: byte-sized write/read at a second address works");
        end

        // -----------------------------------------------------------
        // Test 5: on-chip RAM (addr 0xB4, within the new 0xB0-0xDF RAM
        // range) still works correctly after all this -- confirms
        // in_ram/in_ext decode doesn't overlap or interfere with each
        // other.
        // -----------------------------------------------------------
        do_access(8'hB4, 32'h11223344, 2'd2, 1'b1);
        do_access(8'hB4, 32'h0, 2'd2, 1'b0);
        if (rdata !== 32'h11223344) begin
            errors = errors + 1;
            $display("FAIL test5: on-chip RAM read back %h, expected 11223344", rdata);
        end else begin
            $display("PASS test5: on-chip RAM still works correctly alongside the external window");
        end

        // -----------------------------------------------------------
        // Test 6: PSRAM_BANK (0xF1) resets to 0 (RAM A) -- confirms
        // the default matches every pre-CS2 revision's behavior, i.e.
        // 0xE0-0xEF still reaches RAM A (test2/3's value) without a
        // program ever having to touch PSRAM_BANK.
        // -----------------------------------------------------------
        do_access(8'hF1, 32'h0, 2'd0, 1'b0);
        if (rdata[0] !== 1'b0) begin
            errors = errors + 1;
            $display("FAIL test6: PSRAM_BANK read back %0d, expected 0 (RAM A) on reset", rdata[0]);
        end else begin
            $display("PASS test6: PSRAM_BANK resets to 0 (RAM A)");
        end

        // -----------------------------------------------------------
        // Test 7: switch PSRAM_BANK to 1 (RAM B) and write/read-back
        // through the SAME 0xE0-0xEF window -- confirms CS2 reaches a
        // genuinely separate chip (ram_b), not the same memory RAM A
        // (ram_a) already has CAFEBABE/A5 sitting in from tests 2-4.
        // -----------------------------------------------------------
        do_access(8'hF1, 32'h1, 2'd0, 1'b1);
        do_access(8'hE0, 32'h0, 2'd2, 1'b0); // RAM B is freshly zeroed, unlike RAM A
        if (rdata !== 32'h0) begin
            errors = errors + 1;
            $display("FAIL test7a: RAM B pre-write read %h, expected 0 (separate chip from RAM A)", rdata);
        end else begin
            $display("PASS test7a: RAM B is a genuinely separate chip from RAM A (reads back 0, not CAFEBABE)");
        end

        do_access(8'hE0, 32'h13572468, 2'd2, 1'b1);
        do_access(8'hE0, 32'h0, 2'd2, 1'b0);
        if (rdata !== 32'h13572468) begin
            errors = errors + 1;
            $display("FAIL test7b: RAM B read back %h, expected 13572468", rdata);
        end else begin
            $display("PASS test7b: RAM B write-then-read round trip through mem.v works");
        end

        // -----------------------------------------------------------
        // Test 8: switch PSRAM_BANK back to 0 (RAM A) and confirm RAM
        // A's own earlier data (test2/3's CAFEBABE) is still intact --
        // i.e. banking doesn't disturb either chip's actual contents.
        // -----------------------------------------------------------
        do_access(8'hF1, 32'h0, 2'd0, 1'b1);
        do_access(8'hE0, 32'h0, 2'd2, 1'b0);
        if (rdata !== 32'hCAFEBABE) begin
            errors = errors + 1;
            $display("FAIL test8: RAM A read back %h after switching banks, expected CAFEBABE still intact", rdata);
        end else begin
            $display("PASS test8: switching PSRAM_BANK back to RAM A preserves its earlier contents");
        end

        #20;
        if (errors == 0)
            $display("ALL TESTS PASSED");
        else
            $display("%0d TEST(S) FAILED", errors);
        $finish;
    end

endmodule
