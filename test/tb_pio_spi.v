// tb_pio_spi.v -- standalone test of the PIO block as an SPI master.
//
// Runs the SDK-style two-instruction `spi_cpha0` program (pio/spi_master.pio)
// against a behavioural mode-0 slave, with autopull/autopush at 8 bits.
//   MOSI = pin 2, SCK = pin 3 (side-set), MISO = pin 4 (IN_BASE).
//
// Checks: every MOSI byte the slave sees, every MISO byte the master
// receives, SCK period is exactly 4 PIO clocks with no gap between
// back-to-back bytes (autopull refill costs no cycles), SCK idles low when
// the TX FIFO runs dry (side-set applies to a stalled instruction), and --
// on purpose -- that the 2-flop input synchroniser makes MISO sampling stale
// at this SCK rate unless SYNC_BYP is set for the MISO pin, exactly as the
// RP2040 SDK's PIO SPI driver has to work around.
//
// Hex input: /tmp/pio_spi.hex  (origin 0, 2 words)

`timescale 1ns/1ps
`default_nettype none

module tb_pio_spi;
    reg         clk, rst_n;
    reg         valid, we;
    reg  [7:0]  addr;
    reg  [31:0] wdata;
    wire [31:0] rdata;
    wire        sel;
    wire [9:0]  pin_out, pin_dir, pin_own;

    reg         miso;
    wire        mosi = pin_out[2];
    wire        sck  = pin_out[3];
    wire [9:0]  pins_raw = {5'b0, miso, sck, mosi, 2'b00};

    `include "tb_pio_common.vh"

    pio #(.N_SM(2), .FIFO_LOG2(2)) dut (
        .clk(clk), .rst_n(rst_n),
        .valid(valid), .we(we), .addr(addr), .wdata(wdata),
        .rdata(rdata), .sel(sel),
        .pins_raw(pins_raw),
        .pin_out(pin_out), .pin_dir(pin_dir), .pin_own(pin_own));

    // ------------------------------------------------------------------
    // Behavioural SPI slave, mode 0, MSB first, no CS (always selected).
    // Drives MISO 2 ns after the falling SCK edge (a realistic pad delay),
    // samples MOSI on the rising edge.
    // ------------------------------------------------------------------
    reg [7:0] resp [0:7];           // bytes the slave will answer with
    reg [7:0] sl_rx [0:15];         // bytes the slave received
    integer   sl_nrx, sl_bits, sl_ridx;
    reg [7:0] sl_sh;

    time      last_rise;
    integer   bad_period, n_rise;
    integer   period_hist_ok, period_hist_seen;

    always @(posedge sck) begin
        sl_sh = {sl_sh[6:0], mosi};
        sl_bits = sl_bits + 1;
        if (sl_bits == 8) begin
            sl_rx[sl_nrx] = sl_sh;
            sl_nrx = sl_nrx + 1;
            sl_bits = 0;
            sl_ridx = sl_ridx + 1;
        end
        // period check between consecutive rising edges
        if (n_rise > 0) begin
            period_hist_seen = period_hist_seen + 1;
            if ($time - last_rise == 40) period_hist_ok = period_hist_ok + 1;
        end
        last_rise = $time;
        n_rise = n_rise + 1;
    end
    always @(negedge sck) begin
        #2 miso = resp[sl_ridx][7 - sl_bits];
    end

    // ------------------------------------------------------------------
    reg [15:0] prog [0:1];
    integer i;

    task slave_reset;
        begin
            sl_nrx = 0; sl_bits = 0; sl_ridx = 0; sl_sh = 8'h00;
            n_rise = 0; period_hist_ok = 0; period_hist_seen = 0;
            miso = resp[0][7];
        end
    endtask

    task rx_word;               // pop SM0 RXF into rd_val once available
        integer g;
        begin
            g = 0;
            pio_rd(R_FSTAT);
            while (rd_val[8] && g < 500) begin
                repeat (8) @(posedge clk);
                pio_rd(R_FSTAT);
                g = g + 1;
            end
            check(g < 500, "SPI RX FIFO stayed empty");
            pio_rd(smreg(0, SM_RXF));
        end
    endtask

    task config_spi;
        input bypass_miso;
        begin
            pio_wr(R_CTRL, 32'h0000_0000);                       // stop
            pio_wr(R_CTRL, 32'h0000_F0F0);                       // restart + clear FIFOs
            pio_wr(R_SYNC_BYP, bypass_miso ? 32'h10 : 32'h0);    // pin 4
            // OUT_BASE=2 (MOSI, 1 pin), SET_BASE=2 (2 pins), SIDESET_BASE=3 (1 bit), IN_BASE=4
            pio_wr(smreg(0, SM_PINCTRL), pinctrl(4'd2, 4'd1, 4'd2, 3'd2, 4'd3, 3'd1, 4'd4));
            pio_wr(smreg(0, SM_EXEC),    execctrl(5'd0, 5'd1, 1'b0, 1'b0, 4'd0));
            // autopush+autopull, both shift LEFT (MSB first), thresholds 8
            pio_wr(smreg(0, SM_SHIFT),   shiftctrl(1'b1, 1'b1, 1'b0, 1'b0, 5'd8, 5'd8));
            pio_wr(smreg(0, SM_CLKDIV),  clkdiv_reg(16'd1, 8'd0));
            pio_wr(R_PIN_OWN, 32'h0000_000C);                    // pins 2,3
            sm_exec(0, 16'hE083);                                // set pindirs, 3
            sm_exec(0, 16'hE000);                                // set pins, 0
            sm_exec(0, 16'h0000);                                // jmp 0
        end
    endtask

    task run_burst;             // send 3 bytes back-to-back, expect them + slave's replies
        input [7:0] b0; input [7:0] b1; input [7:0] b2;
        input integer expect_good_miso;
        reg [31:0] r0, r1, r2;
        begin
            slave_reset;
            pio_wr(smreg(0, SM_TXF), {b0, 24'h0});
            pio_wr(smreg(0, SM_TXF), {b1, 24'h0});
            pio_wr(smreg(0, SM_TXF), {b2, 24'h0});
            pio_wr(R_CTRL, 32'h0000_0001);                       // enable SM0
            rx_word; r0 = rd_val;
            rx_word; r1 = rd_val;
            rx_word; r2 = rd_val;
            repeat (30) @(posedge clk);
            check(sl_nrx == 3, "slave should have received exactly 3 bytes");
            check(sl_rx[0] === b0, "MOSI byte 0 mismatch at slave");
            check(sl_rx[1] === b1, "MOSI byte 1 mismatch at slave");
            check(sl_rx[2] === b2, "MOSI byte 2 mismatch at slave");
            if (expect_good_miso) begin
                check(r0 === {24'h0, resp[0]}, "MISO byte 0 mismatch at master");
                check(r1 === {24'h0, resp[1]}, "MISO byte 1 mismatch at master");
                check(r2 === {24'h0, resp[2]}, "MISO byte 2 mismatch at master");
            end
            pio_wr(R_CTRL, 32'h0000_0000);                       // stop the SM
            // stash for the caller
            rd_val = {r2[7:0], r1[7:0], r0[7:0], 8'h00};
        end
    endtask

    reg [31:0] got;
    initial begin
        valid = 0; we = 0; addr = 0; wdata = 0;
        resp[0] = 8'hC3; resp[1] = 8'h3C; resp[2] = 8'hA5; resp[3] = 8'h5A;
        resp[4] = 8'hFF; resp[5] = 8'h00; resp[6] = 8'h81; resp[7] = 8'h7E;
        rst_n = 0;
        $readmemh("/tmp/pio_spi.hex", prog);
        slave_reset;
        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

        // load the 2-instruction program
        bus_write(8'hFF, {24'h0, 1'b1, 7'h20});
        bus_write(8'hFE, {16'h0, prog[0]});
        bus_write(8'hFE, {16'h0, prog[1]});
        bus_write(8'hFF, 32'h0);

        // ---- with the MISO synchroniser bypassed (the SDK's configuration) ----
        config_spi(1'b1);
        pio_rd(R_SYNC_BYP);
        check(rd_val[4] == 1'b1, "SYNC_BYP bit 4 should read back set");
        run_burst(8'hA5, 8'h3C, 8'hFF, 1);
        check(period_hist_seen == 23 && period_hist_ok == 23,
              "SCK period must be exactly 4 clocks with no gap between bytes");
        if (!(period_hist_seen == 23 && period_hist_ok == 23))
            $display("      SCK rising-edge intervals: %0d of %0d were 40 ns", period_hist_ok, period_hist_seen);
        check(sck === 1'b0, "SCK must idle low after the SM stops on a stalled OUT");

        // second burst, different data, includes 0x00 (all-zero MOSI must still clock 8 times)
        config_spi(1'b1);
        run_burst(8'h00, 8'h81, 8'h5A, 1);

        // ---- idle: SM enabled with an empty TX FIFO -> SCK stays low, MOSI/SCK quiet ----
        config_spi(1'b1);
        slave_reset;
        pio_wr(R_CTRL, 32'h0000_0001);
        repeat (100) @(posedge clk);
        check(n_rise == 0 && sck === 1'b0, "no SCK activity while TX FIFO is empty");
        pio_wr(smreg(0, SM_TXF), 32'h9900_0000);
        repeat (60) @(posedge clk);
        check(n_rise == 8, "exactly 8 SCK pulses for one byte");
        check(sl_rx[0] === 8'h99, "single-byte transfer after idle");
        pio_wr(R_CTRL, 32'h0000_0000);

        // ---- WITHOUT bypass: the 2-flop synchroniser delays MISO by 2 clocks, so at a
        //      4-clock SCK period the master samples the *previous* bit -> stale data.
        //      MOSI direction is unaffected. This documents the SDK's reason for SYNC_BYP.
        config_spi(1'b0);
        run_burst(8'hA5, 8'h3C, 8'hFF, 0);
        check(sl_rx[0] === 8'hA5 && sl_rx[1] === 8'h3C && sl_rx[2] === 8'hFF,
              "MOSI path must be correct regardless of SYNC_BYP");
        // bit-0 comes right (settled long before the first edge); later bits are shifted by one
        got = {24'h0, rd_val[15:8]};              // r0 lives in rd_val[15:8]
        check(got !== 32'hC3, "without SYNC_BYP MISO must NOT read back cleanly at this SCK rate");
        // the stale result is exactly the byte shifted right by one with bit7 duplicated:
        // sampling lags one bit -> {b7, b7, b6, b5, b4, b3, b2, b1} = 0xC3=1100_0011 -> 1110_0001 = 0xE1
        check(got === 32'hE1, "stale-MISO result should be the 1-bit-lagged byte 0xE1");

        repeat (20) @(posedge clk);
        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("FAIL: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #100_000_000;
        $display("FAIL: watchdog timeout");
        $finish;
    end
endmodule
