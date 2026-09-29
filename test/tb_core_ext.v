// tb_core_ext.v -- the test tb_mem_ext.v and tb_qspi_engine.v don't
// cover: the real rv32i_core, running real RV32I load/store
// instructions (not directly-driven bus signals), against the real
// qspi_shared_engine and a behavioral SPI RAM, through mem_test_ext.v
// (a copy of mem.v with a small test program in place of the
// production demo -- see mem_test_ext.v's rom_byte function for the
// program and its hand-verified encoding).
//
// Test program: store 0xA5 to external RAM (0xB4), load it back, and
// write the loaded value to the LED register (0xF0). PASS means
// uo_out reads 0xA5 before the loop instruction spins forever --
// which only happens if the address decode, the wait-state handshake
// in rv32i_core's FSM, the QSPI engine, and the byte-order handling
// all work together correctly, driven by the real CPU rather than a
// testbench pretending to be one.

`timescale 1ns/1ps
`default_nettype none

module tb_core_ext;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    wire [7:0]  mem_addr;
    wire [31:0] mem_wdata;
    wire [1:0]  mem_size;
    wire        mem_we;
    wire        mem_valid;
    wire        mem_ready;
    wire [31:0] mem_rdata;

    wire qspi_cs0, qspi_cs1, qspi_sck, qspi_mosi, qspi_miso;

    rv32i_core u_core (
        .clk       (clk),
        .rst_n     (rst_n),
        .mem_addr  (mem_addr),
        .mem_wdata (mem_wdata),
        .mem_size  (mem_size),
        .mem_we    (mem_we),
        .mem_valid (mem_valid),
        .mem_ready (mem_ready),
        .mem_rdata (mem_rdata)
    );

    mem_test_ext u_mem (
        .clk      (clk),
        .rst_n    (rst_n),
        .addr     (mem_addr),
        .wdata    (mem_wdata),
        .size     (mem_size),
        .we       (mem_we),
        .valid    (mem_valid),
        .ready    (mem_ready),
        .rdata    (mem_rdata),
        .gpio_in  (8'h00),
        .gpio_out (led_out),
        .qspi_cs0 (qspi_cs0),
        .qspi_cs1 (qspi_cs1),
        .qspi_sck (qspi_sck),
        .qspi_mosi(qspi_mosi),
        .qspi_miso(qspi_miso)
    );
    wire [7:0] led_out;

    spi_ram_model ram (
        .cs_n(qspi_cs1),
        .sck (qspi_sck),
        .mosi(qspi_mosi),
        .miso(qspi_miso)
    );

    integer cycles;
    integer errors = 0;

    initial begin
        rst_n = 0;
        #20 rst_n = 1;

        // Generous budget: 6 instructions, 2 of which are genuine
        // multi-hundred-cycle external SPI transactions (write + read
        // at 0xB4), plus margin. tb_qspi_engine.v measured ~130 wait
        // cycles for one word transaction standalone, so budget well
        // above 2x that for both external accesses plus the on-chip
        // instructions around them.
        for (cycles = 0; cycles < 2000; cycles = cycles + 1) begin
            @(posedge clk);
            if (led_out == 8'hA5) begin
                $display("PASS: uo_out reached 0xA5 after %0d cycles -- external write-then-read round trip through the real CPU works", cycles);
                cycles = 2000; // break
            end
        end

        if (led_out !== 8'hA5) begin
            errors = errors + 1;
            $display("FAIL: uo_out never reached 0xA5 within the cycle budget (final value: %h)", led_out);
        end

        #20;
        if (errors == 0)
            $display("ALL TESTS PASSED");
        else
            $display("%0d TEST(S) FAILED", errors);
        $finish;
    end

endmodule
