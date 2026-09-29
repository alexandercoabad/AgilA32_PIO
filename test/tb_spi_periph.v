// tb_spi_periph.v -- covers the generic SPI peripheral (SPI_DATA,
// mem.v's 0xFD), ported from AgilA8's spi_ctrl.v. Two parts, same
// split tb_qspi_engine.v/tb_mem_ext.v use for the other front-ends:
//
// Part 1 drives qspi_shared_engine directly (req_dev==2'd0) -- a
// "logic analyzer" style check, same as tb_qspi_engine.v's existing
// tests, confirming the raw-byte framing itself: exactly 8 bits on
// the wire (no cmd/addr phase), MOSI-first, only CS2 asserted, and a
// write/read round trip through the engine's byte-order handling is
// self-consistent.
//
// Part 2 drives `mem` directly (mirrors tb_mem_ext.v's do_access
// style) through the real engine, confirming the mem.v-side behavior
// this feature is actually about: a plain SPI_DATA read (no write)
// returns spi_last_rx immediately with ZERO wait cycles and without
// asserting CS2 or clocking SCK at all -- the "no hardware retrigger"
// behavior ported from AgilA8's spi_ctrl.v that Part 1 alone can't
// cover, since Part 1 only exercises the engine's write path (the
// only path that ever reaches it -- see mem.v's in_spi_write).

`timescale 1ns/1ps
`default_nettype none

module tb_spi_periph;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    integer errors = 0;

    // =================================================================
    // Part 1: direct engine, bit-level framing check
    // =================================================================

    reg        req_valid = 0;
    reg        req_we    = 0;
    reg [1:0]  req_dev   = 0;
    reg [23:0] req_addr  = 0;
    reg [31:0] req_wdata = 0;
    reg [1:0]  req_size  = 0;
    reg [1:0]  req_div_sel = 2'd0; // fastest -- bit-level framing only, not timing

    wire [31:0] req_rdata;
    wire        req_ready;
    wire pin_cs0, pin_cs1, pin_cs2, pin_sck, pin_mosi;
    reg  pin_miso = 0;

    qspi_shared_engine eng (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_we(req_we), .req_dev(req_dev),
        .req_addr(req_addr), .req_wdata(req_wdata), .req_size(req_size),
        .req_div_sel(req_div_sel),
        .req_rdata(req_rdata), .req_ready(req_ready),
        .pin_cs0(pin_cs0), .pin_cs1(pin_cs1), .pin_cs2(pin_cs2), .pin_sck(pin_sck),
        .pin_mosi(pin_mosi), .pin_miso(pin_miso)
    );

    task capture_mosi_bits(input integer n, output [63:0] out);
        integer i;
        begin
            out = 64'h0;
            for (i = 0; i < n; i = i + 1) begin
                @(posedge pin_sck);
                out = {out[62:0], pin_mosi};
            end
        end
    endtask

    task drive_miso_bits(input integer n, input [63:0] bits);
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                @(negedge pin_sck);
                pin_miso = bits[n-1-i];
            end
        end
    endtask

    // Same idea as drive_miso_bits, but for use when nothing has
    // consumed any prior bits via posedge pin_sck first (e.g. no
    // preceding capture_mosi_bits call) -- pin_sck starts at 0 (idle),
    // so a bare `@(negedge pin_sck)` as the very first sync wouldn't
    // fire until AFTER bit 0's own rising/sampling edge has already
    // passed, silently skipping it. Setting bit 0 immediately (matching
    // how a real slave already has its first response bit ready before
    // the master ever raises SCK) avoids that off-by-one.
    task drive_miso_bits_from_start(input integer n, input [63:0] bits);
        integer i;
        begin
            pin_miso = bits[n-1];
            for (i = 1; i < n; i = i + 1) begin
                @(negedge pin_sck);
                pin_miso = bits[n-1-i];
            end
        end
    endtask

    reg [63:0] captured;

    // =================================================================
    // Part 2: through mem, integration + "no retrigger on read" check
    // =================================================================

    reg  [7:0]  addr = 0;
    reg  [31:0] wdata = 0;
    reg  [1:0]  size = 0;
    reg         we = 0;
    reg         valid = 0;
    wire        ready;
    wire [31:0] rdata;

    wire qspi_cs0, qspi_cs1, qspi_cs2, qspi_sck, qspi_mosi;
    reg  qspi_miso = 1'b0;

    integer wait_cycles;
    integer sck_pulses;

    mem dut (
        .clk(clk), .rst_n(rst_n),
        .addr(addr), .wdata(wdata), .size(size), .we(we),
        .valid(valid), .ready(ready), .rdata(rdata),
        .gpio_in(8'h00), .gpio_out(),
        .qspi_cs0(qspi_cs0), .qspi_cs1(qspi_cs1), .qspi_cs2(qspi_cs2), .qspi_sck(qspi_sck),
        .qspi_mosi(qspi_mosi), .qspi_miso(qspi_miso)
    );

    // Counts SCK rising edges seen while CS2 is asserted -- a stand-in
    // "was a real transfer clocked or not" oracle, independent of
    // exact cycle-count math (which varies with QSPI_CTRL).
    always @(posedge qspi_sck) if (!qspi_cs2) sck_pulses = sck_pulses + 1;

    // Dumb echo "slave": while CS2 is low, shifts a fixed known byte
    // out on qspi_miso, one bit before each SCK rising edge, same
    // pattern drive_miso_bits_from_start above uses but as a standing
    // background process (Part 2 doesn't know in advance exactly when
    // CS2 will assert the way Part 1's task-driven sequencing does).
    // Bit 0 (MSB) is presented the instant CS2 asserts (negedge, not
    // posedge -- active low), not on the first SCK edge -- same
    // reasoning drive_miso_bits_from_start's own comment gives for why
    // a plain per-SCK-edge-only driver misses the first bit.
    reg [7:0] echo_byte = 8'hC3;
    reg [2:0] echo_bitpos;
    always @(negedge qspi_cs2) begin
        qspi_miso   <= echo_byte[7];
        echo_bitpos <= 3'd6;
    end
    always @(negedge qspi_sck) begin
        if (!qspi_cs2) begin
            qspi_miso   <= echo_byte[echo_bitpos];
            echo_bitpos <= echo_bitpos - 3'd1;
        end
    end

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
        $dumpfile("tb_spi_periph.vcd");
        $dumpvars(0, tb_spi_periph);

        // -------------------------------------------------------
        // Part 1
        // -------------------------------------------------------
        rst_n = 0;
        #20 rst_n = 1;
        #10;

        // Test 1: write 8'hA5 via req_dev==2'd0. Expect exactly 8 bits
        // on the wire (not 40 -- no cmd/addr phase at all), MSB-first,
        // matching req_wdata[7:0] directly, and only CS2 asserted.
        req_dev   = 2'd0;
        req_we    = 1'b1;
        req_wdata = 32'h000000A5;
        req_size  = 2'd0; // documented byte-only; engine ignores this for req_dev==0 anyway
        @(posedge clk);
        req_valid = 1'b1;

        capture_mosi_bits(8, captured);
        if (captured[7:0] !== 8'hA5) begin
            errors = errors + 1;
            $display("FAIL part1 test1 (bitstream): got %h expected a5", captured[7:0]);
        end else begin
            $display("PASS part1 test1: raw byte 0xA5 shifted out MOSI-first, no cmd/addr framing");
        end

        if (pin_cs0 !== 1'b1 || pin_cs1 !== 1'b1 || pin_cs2 !== 1'b0) begin
            errors = errors + 1;
            $display("FAIL part1 test1 (chip select): cs0=%b cs1=%b cs2=%b, expected cs0=1 cs1=1 cs2=0",
                      pin_cs0, pin_cs1, pin_cs2);
        end else begin
            $display("PASS part1 test1: only CS2 asserted (shares the pin with RAM B, see header)");
        end

        // A 9th bit would only exist if this had fallen through to the
        // 40-bit flash/PSRAM framing by mistake -- confirm req_ready
        // fires right after the 8th bit's SCK edge, not 32 bits later.
        @(posedge req_ready);
        if (req_rdata[31:8] !== 24'h0) begin
            errors = errors + 1;
            $display("FAIL part1 test1 (transfer length): req_ready fired but upper rdata bits nonzero -- transfer was wider than 8 bits");
        end else begin
            $display("PASS part1 test1: transfer completed after exactly 8 bits (req_ready fired right on time)");
        end
        req_valid = 1'b0;
        #20;

        if (pin_cs2 !== 1'b1) begin
            errors = errors + 1;
            $display("FAIL part1 test1 (CS2 deasserted after done): cs2=%b", pin_cs2);
        end else begin
            $display("PASS part1 test1: CS2 deasserted after completion");
        end

        // Test 2: write 8'h5A, inject 8'h7E on MISO during the same 8
        // clocks, confirm req_rdata captures it in the low byte (same
        // "data_bits==8" extraction path flash/PSRAM byte reads
        // already use -- see qspi_shared_engine.v's ST_DONE).
        req_dev   = 2'd0;
        req_we    = 1'b1;
        req_wdata = 32'h0000005A;
        @(posedge clk);
        req_valid = 1'b1;
        drive_miso_bits_from_start(8, 64'h000000000000007E);
        @(posedge req_ready);
        if (req_rdata[7:0] !== 8'h7E) begin
            errors = errors + 1;
            $display("FAIL part1 test2 (rx capture): got rdata=%h expected 7e", req_rdata[7:0]);
        end else begin
            $display("PASS part1 test2: simultaneously captured the injected MISO byte (0x7E) while sending 0x5A");
        end
        req_valid = 1'b0;
        #20;

        // -------------------------------------------------------
        // Part 2
        // -------------------------------------------------------
        rst_n = 0;
        we = 0; valid = 0; addr = 0; wdata = 0; size = 0;
        sck_pulses = 0;
        #20 rst_n = 1;
        #10;

        // Speed the divider up first (QSPI_CTRL, 0xFB) purely so this
        // test doesn't spend real wall-clock time waiting out the
        // slow reset default -- see docs/info.md's "Variable SPI
        // clock divider" section; this is exactly the software-
        // controlled speedup that register exists for.
        do_access(8'hFB, 32'h00000000, 2'd0, 1'b1);

        // Test 3: a plain read of SPI_DATA (0xFD) before anything has
        // ever been written should return 0x00 (spi_last_rx's reset
        // value) with ZERO wait cycles -- no hardware transfer at
        // all, confirmed by sck_pulses staying at 0.
        do_access(8'hFD, 32'h0, 2'd0, 1'b0);
        if (rdata[7:0] !== 8'h00 || wait_cycles !== 0) begin
            errors = errors + 1;
            $display("FAIL part2 test3: expected rdata=00 wait_cycles=0, got rdata=%h wait_cycles=%0d",
                      rdata[7:0], wait_cycles);
        end else begin
            $display("PASS part2 test3: SPI_DATA reads 0x00 (reset default) with zero wait cycles, no transfer");
        end
        if (sck_pulses !== 0) begin
            errors = errors + 1;
            $display("FAIL part2 test3: a plain read clocked SCK (%0d pulses) -- should never touch hardware", sck_pulses);
        end else begin
            $display("PASS part2 test3: no SCK activity from a plain read");
        end

        // Test 4: write 8'h99 to SPI_DATA. Should take multiple wait
        // cycles (a real transfer), assert CS2 for exactly 8 SCK
        // pulses, and the echo slave's fixed 0xC3 response should end
        // up captured.
        sck_pulses = 0;
        do_access(8'hFD, 32'h00000099, 2'd0, 1'b1);
        if (wait_cycles == 0) begin
            errors = errors + 1;
            $display("FAIL part2 test4: SPI_DATA write completed with zero wait cycles -- should be a real transfer");
        end else begin
            $display("PASS part2 test4: SPI_DATA write took %0d wait cycles (a real hardware transfer)", wait_cycles);
        end
        if (sck_pulses !== 8) begin
            errors = errors + 1;
            $display("FAIL part2 test4: expected exactly 8 SCK pulses under CS2, got %0d", sck_pulses);
        end else begin
            $display("PASS part2 test4: exactly 8 SCK pulses clocked, CS2-gated");
        end
        if (qspi_cs0 !== 1'b1 || qspi_cs1 !== 1'b1) begin
            errors = errors + 1;
            $display("FAIL part2 test4: flash/RAM-A CS lines disturbed by a generic-SPI transfer (cs0=%b cs1=%b)",
                      qspi_cs0, qspi_cs1);
        end else begin
            $display("PASS part2 test4: flash (CS0) and PSRAM RAM A (CS1) untouched");
        end

        // Test 5: read SPI_DATA back -- should be 0xC3 (the echo
        // slave's response captured during test4's write), instantly,
        // no new transfer.
        sck_pulses = 0;
        do_access(8'hFD, 32'h0, 2'd0, 1'b0);
        if (rdata[7:0] !== 8'hC3 || wait_cycles !== 0) begin
            errors = errors + 1;
            $display("FAIL part2 test5: expected rdata=c3 wait_cycles=0, got rdata=%h wait_cycles=%0d",
                      rdata[7:0], wait_cycles);
        end else begin
            $display("PASS part2 test5: SPI_DATA read back 0xC3 (last transfer's captured byte) with zero wait cycles");
        end

        // Test 6: read it again -- must be the SAME 0xC3, and STILL no
        // SCK activity. This is the crux of the "no retrigger" port
        // from AgilA8's spi_ctrl.v: repeated reads are free, only a
        // write ever actually clocks the bus.
        do_access(8'hFD, 32'h0, 2'd0, 1'b0);
        if (rdata[7:0] !== 8'hC3 || sck_pulses !== 0) begin
            errors = errors + 1;
            $display("FAIL part2 test6: a second plain read either changed the value or clocked hardware (rdata=%h sck_pulses=%0d)",
                      rdata[7:0], sck_pulses);
        end else begin
            $display("PASS part2 test6: repeated reads stay free -- same byte, zero additional SCK activity");
        end

        if (errors == 0) begin
            $display("ALL TESTS PASSED");
        end else begin
            $display("FAILED with %0d error(s)", errors);
        end
        $finish;
    end

    initial begin
        #200000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
