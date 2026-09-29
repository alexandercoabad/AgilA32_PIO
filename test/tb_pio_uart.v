// tb_pio_uart.v -- standalone test of the PIO block (pio.v + pio_sm.v +
// pio_fifo.v) running the *unmodified* Raspberry Pi SDK uart_tx / uart_rx
// programs, assembled by tools/pioasm.py.
//
// Covers: INFO/FSTAT/FLEVEL registers, instruction-memory read-back with
// auto-increment, FIFO depth + overflow behaviour, forced instructions,
// SM0 = UART TX with a cycle-exact waveform check (8 PIO clocks per bit),
// SM1 = UART RX looped back from SM0 through the input synchroniser,
// the framing-error IRQ (irq 4 rel -> flag 4+SM_ID), and the fractional
// clock divider (clkdiv = 2.5).
//
// Hex inputs (made by the Makefile, one word per line, relocated to the
// origin the test loads them at):
//     /tmp/pio_uart_tx.hex   (origin 0,  4 words)
//     /tmp/pio_uart_rx.hex   (origin 4,  9 words)

`timescale 1ns/1ps
`default_nettype none

module tb_pio_uart;
    reg         clk, rst_n;
    reg         valid, we;
    reg  [7:0]  addr;
    reg  [31:0] wdata;
    wire [31:0] rdata;
    wire        sel;

    reg  [9:0]  ext_in;          // testbench-driven part of the pin space
    wire [9:0]  pin_out, pin_dir, pin_own;

    // Loopback: pin 1 (RX) sees pin 0 (TX) unless the TB forces it.
    reg         force_rx;        // 1 -> TB drives pin 1 from force_rx_val
    reg         force_rx_val;
    wire        rx_line = force_rx ? force_rx_val : pin_out[0];
    wire [9:0]  pins_raw = {ext_in[9:2], rx_line, pin_out[0]};

    `include "tb_pio_common.vh"

    pio #(.N_SM(2), .FIFO_LOG2(2)) dut (
        .clk(clk), .rst_n(rst_n),
        .valid(valid), .we(we), .addr(addr), .wdata(wdata),
        .rdata(rdata), .sel(sel),
        .pins_raw(pins_raw),
        .pin_out(pin_out), .pin_dir(pin_dir), .pin_own(pin_own));

    // -------------------------------------------------------------
    reg [15:0] tx_prog [0:3];
    reg [15:0] rx_prog [0:8];
    integer i, k, t;

    task load_words;            // stream `n` words from `mem` at imem `base`
        input integer base;
        input integer n;
        input integer which;    // 0 = tx_prog, 1 = rx_prog
        integer j;
        begin
            bus_write(8'hFF, {24'h0, 1'b1, 7'h20 + base[6:0]});   // auto-increment on
            for (j = 0; j < n; j = j + 1)
                bus_write(8'hFE, {16'h0, which ? rx_prog[j] : tx_prog[j]});
            bus_write(8'hFF, 32'h0);                               // auto-increment off
        end
    endtask

    // Capture pin_out[0] once per clock for `n` clocks starting at the first falling edge.
    reg [255:0] cap;
    integer     cap_n;
    task capture_frame;
        input integer n;
        integer guard, j;
        begin
            guard = 0;
            @(posedge clk); #1;
            while (pin_out[0] !== 1'b0 && guard < 20000) begin @(posedge clk); #1; guard = guard + 1; end
            check(guard < 20000, "TX start bit never appeared");
            for (j = 0; j < n; j = j + 1) begin
                cap[j] = pin_out[0];
                @(posedge clk); #1;
            end
            cap_n = n;
        end
    endtask

    // Expected 8N1 frame at `spb` clocks per bit -> compare with cap[]
    task check_frame;
        input [7:0] data;
        input integer spb;
        integer bitn, c, bad;
        reg exp;
        begin
            bad = 0;
            for (c = 0; c < 10 * spb; c = c + 1) begin
                bitn = c / spb;
                if (bitn == 0)       exp = 1'b0;
                else if (bitn == 9)  exp = 1'b1;
                else                 exp = data[bitn - 1];
                if (cap[c] !== exp) bad = bad + 1;
            end
            check(bad == 0, "UART TX waveform differs from ideal 8N1 frame");
            if (bad != 0) $display("      (%0d mismatching clock samples, data=%02h)", bad, data);
        end
    endtask

    task rx_expect;             // pop RXF of SM1 and compare
        input [7:0] data;
        integer g;
        begin
            g = 0;
            pio_rd(smreg(1, SM_ADDR));    // dummy read keeps bus busy a bit
            pio_rd(7'h03);
            while (rd_val[9] && g < 400) begin   // RXEMPTY[1] (bit 8+1)
                repeat (10) @(posedge clk);
                pio_rd(7'h03);
                g = g + 1;
            end
            check(g < 400, "RX FIFO stayed empty (byte never received)");
            pio_rd(smreg(1, SM_RXF));
            check(rd_val === {data, 24'h0}, "RX byte mismatch");
            if (rd_val !== {data, 24'h0})
                $display("      got %08h expected %08h", rd_val, {data, 24'h0});
        end
    endtask

    // -------------------------------------------------------------
    initial begin
        valid = 0; we = 0; addr = 0; wdata = 0;
        ext_in = 10'h0; force_rx = 0; force_rx_val = 1;
        rst_n = 0;
        $readmemh("/tmp/pio_uart_tx.hex", tx_prog);
        $readmemh("/tmp/pio_uart_rx.hex", rx_prog);
        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);

        // ---------------- INFO / reset state ----------------
        pio_rd(R_INFO);
        check(rd_val[3:0]   == 4'd2,  "INFO.N_SM should be 2");
        check(rd_val[15:8]  == 8'd32, "INFO.IMEM_DEPTH should be 32");
        check(rd_val[23:16] == 8'd4,  "INFO.FIFO_DEPTH should be 4");
        check(rd_val[31:24] == 8'd1,  "INFO.version should be 1");
        pio_rd(R_FSTAT);
        check(rd_val[11:8]  == 4'b0011, "both RX FIFOs should be empty at reset");
        check(rd_val[27:24] == 4'b0011, "both TX FIFOs should be empty at reset");
        check(rd_val[3:0] == 0 && rd_val[19:16] == 0, "no FIFO should be full at reset");
        pio_rd(R_PIN_OWN);
        check(rd_val == 0, "PIN_OWN resets to 0");

        // ---------------- program memory with auto-increment ----------------
        load_words(0, 4, 0);
        load_words(4, 9, 1);
        for (i = 0; i < 4; i = i + 1) begin
            pio_rd(R_IMEM + i);
            check(rd_val === {16'h0, tx_prog[i]}, "TX program read-back mismatch");
        end
        for (i = 0; i < 9; i = i + 1) begin
            pio_rd(R_IMEM + 4 + i);
            check(rd_val === {16'h0, rx_prog[i]}, "RX program read-back mismatch");
        end
        // auto-increment on READ too: idx must step after each PIO_DATA read
        bus_write(8'hFF, {24'h0, 1'b1, 7'h20});
        bus_read(8'hFE); check(rd_val[15:0] === tx_prog[0], "autoinc read 0");
        bus_read(8'hFE); check(rd_val[15:0] === tx_prog[1], "autoinc read 1");
        bus_read(8'hFE); check(rd_val[15:0] === tx_prog[2], "autoinc read 2");
        bus_write(8'hFF, 32'h0);

        // ---------------- SM0 = UART TX ----------------
        // OUT/SET/SIDESET base = pin 0; side-set = 2 bits (1 data + opt enable)
        pio_wr(smreg(0, SM_PINCTRL), pinctrl(4'd0, 4'd1, 4'd0, 3'd1, 4'd0, 3'd2, 4'd0));
        pio_wr(smreg(0, SM_EXEC),    execctrl(5'd0, 5'd3, 1'b1, 1'b0, 4'd0));
        pio_wr(smreg(0, SM_SHIFT),   shiftctrl(1'b0, 1'b0, 1'b1, 1'b1, 5'd0, 5'd0));
        pio_wr(smreg(0, SM_CLKDIV),  clkdiv_reg(16'd1, 8'd0));
        pio_wr(R_PIN_OWN, 32'h0000_0001);
        sm_exec(0, 16'hE001);                     // set pins, 1   (line idle high)
        sm_exec(0, 16'hE081);                     // set pindirs, 1
        pio_rd(R_PINS_OUT);
        check(rd_val[0] === 1'b1 && rd_val[16] === 1'b1, "TX pin should be driven high by forced SET");
        check(pin_own[0] === 1'b1 && pin_dir[0] === 1'b1, "pin_own/pin_dir export");

        // ---------------- SM1 = UART RX ----------------
        pio_wr(smreg(1, SM_PINCTRL), pinctrl(4'd0, 4'd0, 4'd0, 3'd0, 4'd0, 3'd0, 4'd1)); // IN_BASE = 1
        pio_wr(smreg(1, SM_EXEC),    execctrl(5'd4, 5'd12, 1'b0, 1'b0, 4'd1));           // wrap 4..12, JMP_PIN = 1
        pio_wr(smreg(1, SM_SHIFT),   shiftctrl(1'b0, 1'b0, 1'b1, 1'b1, 5'd0, 5'd0));
        pio_wr(smreg(1, SM_CLKDIV),  clkdiv_reg(16'd1, 8'd0));
        sm_exec(1, 16'h0004);                     // jmp 4  (start of RX program)

        // ---------------- FIFO depth / overflow (SMs still disabled) ----------------
        for (i = 0; i < 6; i = i + 1)
            pio_wr(smreg(0, SM_TXF), 32'h1000 + i);
        pio_rd(R_FSTAT);
        check(rd_val[16] == 1'b1, "TXFULL[0] should be set after 4+ pushes");
        pio_rd(smreg(0, SM_FLEVEL));
        check(rd_val[2:0] == 3'd4, "TX level should saturate at 4 (extra pushes dropped)");
        pio_wr(R_CTRL, 32'h0000_F000);            // FIFO_CLEAR all
        pio_rd(R_FSTAT);
        check(rd_val[27:24] == 4'b0011 && rd_val[19:16] == 4'b0, "FIFO_CLEAR should empty TX FIFOs");
        pio_rd(smreg(0, SM_RXF));
        check(rd_val == 32'h0, "reading an empty RXF returns 0");

        // ---------------- enable both, exact waveform of one frame ----------------
        pio_wr(R_CTRL, 32'h0000_0003);            // SM_ENABLE = 11
        repeat (40) @(posedge clk);
        pio_wr(smreg(0, SM_TXF), 32'h0000_0055);
        capture_frame(10 * 8 + 8);
        check_frame(8'h55, 8);
        rx_expect(8'h55);

        // ---------------- several bytes back to back, loopback ----------------
        pio_wr(smreg(0, SM_TXF), 32'h0000_00A3);
        pio_wr(smreg(0, SM_TXF), 32'h0000_0000);
        pio_wr(smreg(0, SM_TXF), 32'h0000_00FF);
        pio_wr(smreg(0, SM_TXF), 32'h0000_0081);
        rx_expect(8'hA3);
        rx_expect(8'h00);
        rx_expect(8'hFF);
        rx_expect(8'h81);
        pio_rd(R_IRQ);
        check(rd_val[7:0] == 8'h00, "no framing-error IRQ expected on clean traffic");

        // ---------------- framing error: line held low (a break) ----------------
        force_rx = 1; force_rx_val = 0;
        repeat (200) @(posedge clk);
        pio_rd(R_IRQ);
        check(rd_val[5] == 1'b1, "break must raise IRQ flag 5 (irq 4 rel on SM1)");
        pio_rd(R_FSTAT);
        check(rd_val[9] == 1'b1, "framing error must not push a word");
        force_rx_val = 1;                        // release the line
        repeat (40) @(posedge clk);
        pio_wr(R_IRQ, 32'h20);                   // W1C
        pio_rd(R_IRQ);
        check(rd_val[7:0] == 8'h00, "IRQ flag should clear on W1C");
        force_rx = 0;
        // recovers: a new byte still goes through
        pio_wr(smreg(0, SM_TXF), 32'h0000_005A);
        rx_expect(8'h5A);

        // ---------------- fractional clock divider: 2.5 -> 20 clks/bit on average ----------------
        pio_wr(smreg(0, SM_CLKDIV), clkdiv_reg(16'd2, 8'h80));
        pio_wr(R_CTRL, 32'h0000_0F03);           // keep enabled + restart the clock dividers
        repeat (60) @(posedge clk);
        pio_wr(smreg(0, SM_TXF), 32'h0000_0096);
        begin : frac
            integer start_t, end_t;
            pio_wr(smreg(0, SM_TXF), 32'h0000_0096);
            pio_wr(smreg(0, SM_TXF), 32'h0000_0001);   // queue byte 2 so the frames abut
            @(negedge pin_out[0]);                     // start bit of byte 1
            start_t = $time / 10;
            // 0x96 = 1001_0110, MSB (bit 7) = 1 -> line is high from t+160 to t+200 (2.5 div)
            repeat (190) @(posedge clk);
            @(negedge pin_out[0]);                     // start bit of byte 2
            end_t = $time / 10;
            // 10 bits x 8 ticks = 80 ticks; at 2.5 clocks/tick that is exactly 200 clocks.
            check(end_t - start_t >= 198 && end_t - start_t <= 202,
                  "clkdiv 2.5: frame period should be ~200 clocks");
            if (!(end_t - start_t >= 198 && end_t - start_t <= 202))
                $display("      measured %0d clocks", end_t - start_t);
        end

        // ---------------- done ----------------
        repeat (50) @(posedge clk);
        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("FAIL: %0d error(s)", errors);
        $finish;
    end

    // global watchdog
    initial begin
        #200_000_000;
        $display("FAIL: watchdog timeout");
        $finish;
    end
endmodule
