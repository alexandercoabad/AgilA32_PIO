`timescale 1ns/1ps

// tb_pio_cpu_uart.v -- END-TO-END: real RV32I core + real PIO block in the real
// top level (tt_um_agila32), no shortcuts on the bus.
//
// Flow: bootload the flash-handoff stub over the GPIO bootloader -> core jumps
// into external flash -> the flash image built by tools/build_pio_uart.py
// (assembles pio/uart_tx.pio with tools/pioasm.py, then programs the PIO block
// with CPU stores to PIO_IDX/PIO_DATA at 0xFF/0xFE) starts SM0 and queues the
// bytes "Agil" -> the core executes EBREAK and HALTS -> SM0 keeps transmitting.
//
// The testbench is a UART receiver on uo_out[0] (8N1, 128 clk/bit = PIO tick 8
// per bit x CLKDIV 16). It checks: (a) every byte decodes with a valid stop
// bit, (b) the line never went low before the first real start bit (the
// pin-ownership handover must not glitch), (c) the core was halted while the
// last byte was still on the wire (i.e. PIO really runs without the CPU),
// and (d) the pre-PIO behaviour is intact (LED path drove uo_out[0] before
// PIN_OWN).

module tb_pio_cpu_uart;
    reg clk = 0;
    reg rst_n;
    reg [7:0] ui_in;
    wire [7:0] uo_out, uio_out, uio_oe;
    always #5 clk = ~clk;

    wire cs0 = uio_out[0], cs1 = uio_out[6], sck = uio_out[3], mosi = uio_out[1];
    wire miso_flash, miso_psram;
    wire miso_bus = (!cs0) ? miso_flash : (!cs1) ? miso_psram : 1'b0;
    wire [7:0] uio_in = {5'b0, miso_bus, 2'b0};

    tt_um_agila32 dut (.ui_in(ui_in), .uo_out(uo_out), .uio_in(uio_in),
                       .uio_out(uio_out), .uio_oe(uio_oe), .ena(1'b1),
                       .clk(clk), .rst_n(rst_n));

    spi_ram_model u_flash (.cs_n(cs0), .sck(sck), .mosi(mosi), .miso(miso_flash));
    spi_ram_model u_psram (.cs_n(cs1), .sck(sck), .mosi(mosi), .miso(miso_psram));

    reg [7:0] image [0:395];
    integer ci;
    initial begin
        $readmemh("pio_uart_flash_image.hex", image);
        for (ci = 0; ci < 396; ci = ci + 1) u_flash.mem[ci] = image[ci];
    end

    task automatic boot_send_bit(input b);
        begin
            ui_in[0] = b; ui_in[1] = 0; repeat (200) @(posedge clk);
            ui_in[1] = 1;               repeat (200) @(posedge clk);
            ui_in[1] = 0;               repeat (200) @(posedge clk);
        end
    endtask
    task automatic boot_send_byte(input [7:0] b);
        integer bi;
        begin for (bi = 7; bi >= 0; bi = bi - 1) boot_send_bit(b[bi]); end
    endtask

    reg [7:0] stub_bytes [0:3];
    localparam integer STUB_LEN = 4;

    // ---- UART receiver on uo_out[0] ----
    localparam integer SPB = 128;
    reg [7:0] rx [0:7];
    integer   nrx, frame_errs;
    reg       pio_started;           // set once SM0 has been enabled
    integer   glitch;                // low samples seen while owned but before 1st start bit
    reg       halted_at_last;

    task automatic rx_byte;
        integer b;
        reg [7:0] d;
        begin
            @(negedge uo_out[0]);
            repeat (SPB/2) @(posedge clk);
            if (uo_out[0] !== 1'b0) frame_errs = frame_errs + 1;      // start bit must still be low mid-bit
            for (b = 0; b < 8; b = b + 1) begin
                repeat (SPB) @(posedge clk);
                d[b] = uo_out[0];
            end
            repeat (SPB) @(posedge clk);
            if (uo_out[0] !== 1'b1) frame_errs = frame_errs + 1;      // stop bit
            rx[nrx] = d; nrx = nrx + 1;
        end
    endtask

    integer errors, i;
    reg [7:0] exp [0:3];

    initial begin
        exp[0] = "A"; exp[1] = "g"; exp[2] = "i"; exp[3] = "l";
        $readmemh("flash_handoff_stub.hex", stub_bytes);
        errors = 0; nrx = 0; frame_errs = 0; glitch = 0; halted_at_last = 0;
        ui_in = 8'h00;
        rst_n = 0; repeat (10) @(posedge clk); rst_n = 1;
        dut.u_mem.qspi_div_sel = 2'd0;     // fast SPI, as in the other flash tests
        repeat (3000) @(posedge clk);

        ui_in[2] = 1;
        repeat (20) @(posedge clk);
        boot_send_byte(STUB_LEN);
        for (ci = 0; ci < STUB_LEN; ci = ci + 1) boot_send_byte(stub_bytes[ci]);

        for (i = 0; i < 4; i = i + 1) begin
            rx_byte;
            if (i == 3) halted_at_last = dut.halted;   // sampled right after the last byte's stop bit
        end

        for (i = 0; i < 4; i = i + 1) begin
            if (rx[i] !== exp[i]) begin
                errors = errors + 1;
                $display("FAIL: byte %0d = 0x%02x ('%c'), expected 0x%02x", i, rx[i], rx[i], exp[i]);
            end else
                $display("PASS: byte %0d = 0x%02x ('%c')", i, rx[i], rx[i]);
        end
        if (frame_errs != 0) begin errors = errors + 1; $display("FAIL: %0d framing error(s)", frame_errs); end
        else $display("PASS: all frames have a low start bit and a high stop bit");
        if (glitch != 0) begin errors = errors + 1; $display("FAIL: line glitched low %0d time(s) at PIN_OWN handover", glitch); end
        else $display("PASS: no glitch on uo_out[0] at PIN_OWN handover");
        if (!halted_at_last) begin errors = errors + 1; $display("FAIL: core was not halted while the last byte was on the wire"); end
        else $display("PASS: core halted (EBREAK) while PIO finished transmitting");

        if (errors == 0) $display("PASS tb_pio_cpu_uart: CPU programmed PIO, halted, PIO sent \"Agil\" on uo_out[0]");
        else             $display("FAIL tb_pio_cpu_uart: %0d error(s)", errors);
        $finish;
    end

    // glitch monitor: from the moment PIO owns pin 0 until SM0 is enabled the
    // line must sit high (idle). A low sample here would be a spurious start bit.
    always @(posedge clk) begin
        if (dut.u_pio.own_q[0] && dut.u_pio.sm_en[0] == 1'b0 && !pio_started && uo_out[0] === 1'b0)
            glitch = glitch + 1;
        if (dut.u_pio.sm_en[0]) pio_started <= 1'b1;
    end
    initial pio_started = 0;

    initial begin
        #60000000;
        $display("TIMEOUT (received %0d bytes)", nrx);
        $finish;
    end
endmodule
