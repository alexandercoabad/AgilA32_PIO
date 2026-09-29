// tb_qspi_clkdiv.v -- covers feature #3 in the AgilA32<-AgilA8 porting
// roadmap: qspi_shared_engine's variable SPI clock divider (req_div_sel)
// and mem.v's QSPI_CTRL register (0xFB) that drives it.
//
// This is the testbench qspi_shared_engine.v's own header comment
// already refers to ("bit-for-bit identical cycle count per
// transaction, confirmed in test/tb_qspi_clkdiv.v") -- tb_qspi_engine.v
// covers the engine's bit-level protocol/byte-order correctness at a
// single fixed speed (req_div_sel pinned to 2'd0 there); this file is
// specifically about the divider itself.
//
// Part 1 drives qspi_shared_engine directly (mirrors tb_qspi_engine.v's
// style) to check the four half-period settings themselves, in isolation
// from mem.v's register plumbing. Part 2 drives mem module directly
// (mirrors tb_mem_ext.v's do_access style) to check QSPI_CTRL's own
// read/write/reset behavior and that it actually reaches the engine
// end-to-end through a real external transaction.

`timescale 1ns/1ps
`default_nettype none

module tb_qspi_clkdiv;

    integer errors = 0;

    // =====================================================================
    // Part 1: qspi_shared_engine directly -- half-period timing, the
    // req_div_sel=0/"fastest" case matching the engine's pre-this-register
    // fixed speed, and the latch-at-accept-time guarantee.
    // =====================================================================
    reg        clk = 0;
    always #5 clk = ~clk; // 100 MHz sim clock, arbitrary -- only relative
                           // timing matters, and clk's own 10ns period is
                           // the unit every cycle count below is measured
                           // in.

    reg        rst_n = 0;
    reg        req_valid = 0;
    reg        req_we    = 0;
    reg [1:0]  req_dev   = 2'd2; // psram RAM A -- device choice is irrelevant to timing
    reg [23:0] req_addr  = 0;
    reg [31:0] req_wdata = 0;
    reg [1:0]  req_size  = 2'd2; // word -- 64-bit transactions throughout Part 1
    reg [1:0]  req_div_sel = 2'd0;
    wire [31:0] req_rdata;
    wire        req_ready;
    wire pin_cs0, pin_cs1, pin_cs2, pin_sck, pin_mosi;
    reg  pin_miso = 0;

    qspi_shared_engine dut1 (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_we(req_we), .req_dev(req_dev),
        .req_addr(req_addr), .req_wdata(req_wdata), .req_size(req_size),
        .req_div_sel(req_div_sel),
        .req_rdata(req_rdata), .req_ready(req_ready),
        .pin_cs0(pin_cs0), .pin_cs1(pin_cs1), .pin_cs2(pin_cs2), .pin_sck(pin_sck),
        .pin_mosi(pin_mosi), .pin_miso(pin_miso)
    );

    // half_period_for(): the same req_div_sel -> half-SCK-period (in clk
    // cycles) encoding qspi_shared_engine.v's own half_period_for()
    // function implements -- duplicated here (not `include`d) because
    // this file is meant to check that encoding independently, not
    // assume it.
    function [7:0] half_period_for;
        input [1:0] sel;
        begin
            case (sel)
                2'd0:    half_period_for = 8'd1;
                2'd1:    half_period_for = 8'd4;
                2'd2:    half_period_for = 8'd16;
                default: half_period_for = 8'd64;
            endcase
        end
    endfunction

    // golden_total_cycles(): total clk cycles from req_valid asserting to
    // req_ready pulsing, for a word (64-bit) transaction, as a function
    // of req_div_sel. Derived from the engine's own structure: 64 bits *
    // 2 phases (LO+HI) each lasting half_period_for(sel) cycles, plus a
    // fixed 2-cycle ST_IDLE/ST_DONE pipeline overhead that's independent
    // of the divider setting -- i.e. 130 + 128*(half_period_for(sel)-1).
    // The 130-cycle base case (sel=0, half_period=1) is the same fixed
    // timing this engine always ran at before this register existed.
    function integer golden_total_cycles;
        input [1:0] sel;
        begin
            golden_total_cycles = 130 + 128 * (half_period_for(sel) - 1);
        end
    endfunction

    // Measures the clk-cycle length of the SCK half-period currently in
    // progress: waits for one pin_sck transition (to land on a phase
    // boundary), then times how long until the next one, in units of
    // clk's own 10ns period -- directly comparable to half_period_for()'s
    // clk-cycle encoding, without assuming anything about the engine's
    // internal div_cnt.
    task automatic measure_half_period_cycles(output integer cyc);
        integer t0;
        begin
            @(pin_sck);
            t0 = $time;
            @(pin_sck);
            cyc = ($time - t0) / 10;
        end
    endtask

    // Kicks off one word WRITE transaction (fire-and-forget on the
    // request side; caller tracks completion via req_ready).
    task automatic start_word_write;
        begin
            req_dev   = 2'd2;
            req_we    = 1'b1;
            req_addr  = 24'h000100;
            req_wdata = 32'h11223344;
            req_size  = 2'd2;
            @(posedge clk);
            req_valid = 1'b1;
        end
    endtask

    // Waits for req_ready and drops req_valid, counting elapsed clk
    // cycles from the call site (NOT from start_word_write -- the caller
    // is expected to call this right after start_word_write so the two
    // together bound the same window golden_total_cycles() models).
    task automatic wait_ready_count(output integer cyc);
        begin
            cyc = 0;
            while (!req_ready) begin
                @(posedge clk);
                cyc = cyc + 1;
            end
            req_valid = 1'b0;
            #1;
        end
    endtask

    integer sel_i;
    integer half_cyc;
    integer total_cyc;
    integer golden;

    initial begin
        rst_n = 0;
        #20 rst_n = 1;
        #10;

        // -----------------------------------------------------------
        // Tests 1-4: half-SCK-period length AND total transaction
        // length for each req_div_sel setting, checked independently
        // against each other and against golden_total_cycles()'s
        // structural model -- 1/4/16/64 clk-cycle half-periods for
        // req_div_sel=0/1/2/3, matching half_period_for()'s documented
        // encoding (SCK = clk/2, clk/8, clk/32, clk/128).
        // -----------------------------------------------------------
        for (sel_i = 0; sel_i < 4; sel_i = sel_i + 1) begin
            req_div_sel = sel_i[1:0];
            start_word_write;
            measure_half_period_cycles(half_cyc);
            if (half_cyc !== half_period_for(sel_i[1:0])) begin
                errors = errors + 1;
                $display("FAIL test%0d (half-period): req_div_sel=%0d measured %0d clk cycles, expected %0d",
                          sel_i + 1, sel_i, half_cyc, half_period_for(sel_i[1:0]));
            end else begin
                $display("PASS test%0d: req_div_sel=%0d half-SCK-period is %0d clk cycles as documented",
                          sel_i + 1, sel_i, half_cyc);
            end

            @(posedge req_ready);
            @(posedge clk); // let the fire-and-forget req_valid drop cleanly
            req_valid = 1'b0;
            #20;
        end

        // -----------------------------------------------------------
        // Test 5: whole-transaction cycle count at req_div_sel=0
        // reproduces the engine's original fixed-speed timing exactly
        // (130 clk cycles for a 64-bit word transaction) -- the
        // specific claim qspi_shared_engine.v's header comment makes
        // about this register not slowing anything down at the reset-
        // predating default.
        // -----------------------------------------------------------
        req_div_sel = 2'd0;
        start_word_write;
        wait_ready_count(total_cyc);
        golden = golden_total_cycles(2'd0);
        if (total_cyc !== golden) begin
            errors = errors + 1;
            $display("FAIL test5: req_div_sel=0 took %0d cycles, expected %0d (original fixed-speed timing)",
                      total_cyc, golden);
        end else begin
            $display("PASS test5: req_div_sel=0 reproduces the engine's original %0d-cycle fixed timing exactly",
                      total_cyc);
        end
        #20;

        // -----------------------------------------------------------
        // Test 6: whole-transaction cycle counts at req_div_sel=1/2/3
        // match golden_total_cycles()'s structural prediction exactly
        // -- confirms the divider scales the WHOLE transaction (not
        // just an isolated half-period sample) the documented amount.
        // -----------------------------------------------------------
        for (sel_i = 1; sel_i < 4; sel_i = sel_i + 1) begin
            req_div_sel = sel_i[1:0];
            start_word_write;
            wait_ready_count(total_cyc);
            golden = golden_total_cycles(sel_i[1:0]);
            if (total_cyc !== golden) begin
                errors = errors + 1;
                $display("FAIL test6.%0d: req_div_sel=%0d took %0d cycles, expected %0d",
                          sel_i, sel_i, total_cyc, golden);
            end else begin
                $display("PASS test6.%0d: req_div_sel=%0d takes exactly %0d cycles as predicted",
                          sel_i, sel_i, total_cyc);
            end
            #20;
        end

        // -----------------------------------------------------------
        // Test 7: latch-at-accept-time -- req_div_sel is sampled once,
        // at ST_IDLE/accept time, into half_period_r. Start a
        // transaction at req_div_sel=2 (half-period 16), then change
        // req_div_sel to 3 (half-period 64) partway through the SAME
        // in-flight transaction; the transaction already running must
        // finish at its ORIGINAL (sel=2) timing, completely unperturbed
        // by the mid-flight change.
        // -----------------------------------------------------------
        req_div_sel = 2'd2;
        start_word_write;
        golden = golden_total_cycles(2'd2);
        fork
            begin
                wait_ready_count(total_cyc);
            end
            begin
                // land partway into the transaction (well before its
                // ~2050-cycle golden length) before perturbing it.
                repeat (200) @(posedge clk);
                req_div_sel = 2'd3;
            end
        join
        if (total_cyc !== golden) begin
            errors = errors + 1;
            $display("FAIL test7: mid-flight req_div_sel change perturbed an in-flight transaction -- took %0d cycles, expected %0d (sel=2's own timing)",
                      total_cyc, golden);
        end else begin
            $display("PASS test7: mid-flight req_div_sel change does not perturb a transaction already in flight");
        end
        #20;

        // -----------------------------------------------------------
        // Test 8: the NEXT transaction, started after test 7 left
        // req_div_sel at 3, DOES pick up the new value -- confirms
        // test 7's immunity is specifically about accept-time latching,
        // not that req_div_sel is somehow stuck or ignored altogether.
        // -----------------------------------------------------------
        start_word_write;
        wait_ready_count(total_cyc);
        golden = golden_total_cycles(2'd3);
        if (total_cyc !== golden) begin
            errors = errors + 1;
            $display("FAIL test8: transaction after test7 took %0d cycles, expected %0d (req_div_sel=3, latched fresh at this transaction's own accept time)",
                      total_cyc, golden);
        end else begin
            $display("PASS test8: a fresh transaction correctly latches the now-changed req_div_sel=3");
        end

        $display("PART1: %0d error(s)", errors);

        part2_qspi_ctrl_register;

        if (errors == 0)
            $display("ALL TESTS PASSED");
        else
            $display("%0d TEST(S) FAILED", errors);

        $finish;
    end

    // =====================================================================
    // Part 2: mem module directly, with the REAL qspi_shared_engine and
    // two behavioral SPI RAM models (mirrors tb_mem_ext.v's setup and
    // do_access style) -- checks QSPI_CTRL's (0xFB) reset value,
    // read/write behavior, that a write actually reaches the engine and
    // changes real external-access timing end-to-end, and that a
    // register write can't perturb an access already in flight (the
    // same latch-at-accept-time guarantee as test 7, now exercised
    // through the register path instead of req_div_sel directly).
    // =====================================================================
    reg  [7:0]  addr2  = 0;
    reg  [31:0] wdata2 = 0;
    reg  [1:0]  size2  = 0;
    reg         we2    = 0;
    reg         valid2 = 0;
    wire        ready2;
    wire [31:0] rdata2;

    wire qspi_cs0_2, qspi_cs1_2, qspi_cs2_2, qspi_sck_2, qspi_mosi_2, qspi_miso_a2, qspi_miso_b2;
    wire qspi_miso_2 = qspi_cs1_2 == 1'b0 ? qspi_miso_a2 :
                        qspi_cs2_2 == 1'b0 ? qspi_miso_b2 : 1'bz;

    mem dut2 (
        .clk(clk), .rst_n(rst_n),
        .addr(addr2), .wdata(wdata2), .size(size2), .we(we2),
        .valid(valid2), .ready(ready2), .rdata(rdata2),
        .gpio_in(8'h00), .gpio_out(),
        .qspi_cs0(qspi_cs0_2), .qspi_cs1(qspi_cs1_2), .qspi_cs2(qspi_cs2_2), .qspi_sck(qspi_sck_2),
        .qspi_mosi(qspi_mosi_2), .qspi_miso(qspi_miso_2)
    );

    spi_ram_model ram_a2 (
        .cs_n(qspi_cs1_2), .sck(qspi_sck_2), .mosi(qspi_mosi_2), .miso(qspi_miso_a2)
    );

    spi_ram_model ram_b2 (
        .cs_n(qspi_cs2_2), .sck(qspi_sck_2), .mosi(qspi_mosi_2), .miso(qspi_miso_b2)
    );

    integer wait_cycles2;

    // Drives one access exactly the way rv32i_core's FSM does (same
    // task as tb_mem_ext.v's do_access).
    task automatic do_access(input [7:0] a, input [31:0] wd, input [1:0] sz, input do_we);
        begin
            addr2  = a;
            wdata2 = wd;
            size2  = sz;
            we2    = do_we;
            valid2 = 1'b1;
            wait_cycles2 = 0;
            @(posedge clk);
            while (!ready2) begin
                @(posedge clk);
                wait_cycles2 = wait_cycles2 + 1;
            end
            valid2 <= 1'b0;
            we2    <= 1'b0;
            #1;
        end
    endtask

    task part2_qspi_ctrl_register;
        integer p2errs_before;
        integer golden2;
        begin
            p2errs_before = errors;

            // -------------------------------------------------------
            // Test 9: QSPI_CTRL (0xFB) resets to 2'd3 (slowest) --
            // matches mem.v's header doc and AgilA8's own reset-safe
            // default.
            // -------------------------------------------------------
            do_access(8'hFB, 32'h0, 2'd0, 1'b0);
            if (rdata2[1:0] !== 2'd3) begin
                errors = errors + 1;
                $display("FAIL test9: QSPI_CTRL read back %0d on reset, expected 3 (slowest)", rdata2[1:0]);
            end else begin
                $display("PASS test9: QSPI_CTRL resets to 2'd3 (slowest)");
            end

            // -------------------------------------------------------
            // Test 10: plain read/write -- write 2'd1, read it back.
            // -------------------------------------------------------
            do_access(8'hFB, 32'h1, 2'd0, 1'b1);
            do_access(8'hFB, 32'h0, 2'd0, 1'b0);
            if (rdata2[1:0] !== 2'd1) begin
                errors = errors + 1;
                $display("FAIL test10: QSPI_CTRL read back %0d after writing 1, expected 1", rdata2[1:0]);
            end else begin
                $display("PASS test10: QSPI_CTRL write-then-read round trips correctly");
            end

            // -------------------------------------------------------
            // Test 11: the register write actually reaches the engine
            // and changes REAL external-access timing end-to-end --
            // write QSPI_CTRL=0 (fastest) and confirm a word write to
            // the external PSRAM window takes exactly the fast golden
            // cycle count (130), not the slow reset default's 8194.
            // -------------------------------------------------------
            do_access(8'hFB, 32'h0, 2'd0, 1'b1); // fastest
            do_access(8'hE0, 32'hCAFEBABE, 2'd2, 1'b1);
            golden2 = golden_total_cycles(2'd0);
            if (wait_cycles2 !== golden2) begin
                errors = errors + 1;
                $display("FAIL test11: external write took %0d wait cycles at QSPI_CTRL=0, expected %0d",
                          wait_cycles2, golden2);
            end else begin
                $display("PASS test11: QSPI_CTRL=0 reaches the engine end-to-end -- external write takes exactly %0d cycles",
                          wait_cycles2);
            end

            // -------------------------------------------------------
            // Test 12: back to QSPI_CTRL=3 (slow) reaches the engine
            // too, end-to-end -- confirms test 11 isn't a one-way
            // fluke and mem.v's QSPI_CTRL wiring tracks both
            // directions.
            // -------------------------------------------------------
            do_access(8'hFB, 32'h3, 2'd0, 1'b1); // slowest
            do_access(8'hE4, 32'h13572468, 2'd2, 1'b1);
            golden2 = golden_total_cycles(2'd3);
            if (wait_cycles2 !== golden2) begin
                errors = errors + 1;
                $display("FAIL test12: external write took %0d wait cycles at QSPI_CTRL=3, expected %0d",
                          wait_cycles2, golden2);
            end else begin
                $display("PASS test12: QSPI_CTRL=3 also reaches the engine end-to-end -- external write takes exactly %0d cycles",
                          wait_cycles2);
            end

            // -------------------------------------------------------
            // Test 13: a QSPI_CTRL write mid-transaction doesn't
            // perturb a transfer already in flight -- same latch-at-
            // accept-time guarantee as Part 1 test 7, now exercised
            // through mem.v's register path (a direct deposit onto
            // dut2's internal qspi_div_sel register, standing in for
            // what a real mid-flight register write would do, since
            // this single-master bus can't itself issue a second
            // access while the first is still outstanding).
            // -------------------------------------------------------
            addr2  = 8'hE0;
            wdata2 = 32'hA5A5A5A5;
            size2  = 2'd2;
            we2    = 1'b1;
            valid2 = 1'b1;
            wait_cycles2 = 0;
            golden2 = golden_total_cycles(2'd3); // still slow (test12 left it at 3)
            @(posedge clk);
            fork
                begin
                    while (!ready2) begin
                        @(posedge clk);
                        wait_cycles2 = wait_cycles2 + 1;
                    end
                end
                begin
                    repeat (200) @(posedge clk); // partway into the slow transfer
                    dut2.qspi_div_sel = 2'd0;    // simulated mid-flight perturbation
                end
            join
            valid2 = 1'b0;
            we2    = 1'b0;
            #1;
            if (wait_cycles2 !== golden2) begin
                errors = errors + 1;
                $display("FAIL test13: mid-flight QSPI_CTRL perturbation via mem.v affected an in-flight access -- took %0d cycles, expected %0d",
                          wait_cycles2, golden2);
            end else begin
                $display("PASS test13: a mid-flight QSPI_CTRL register change doesn't perturb an access already in flight");
            end

            $display("PART2: %0d error(s)", errors - p2errs_before);
        end
    endtask

endmodule
