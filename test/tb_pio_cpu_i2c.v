`timescale 1ns/1ps

// tb_pio_cpu_i2c.v -- END-TO-END I2C: real RV32I core + real PIO block in the real top level.
//
// bootload the flash-handoff stub over the GPIO bootloader -> core jumps into external flash ->
// the image from tools/build_pio_i2c.py (assembles pio/i2c.pio, programs PIO SM0 with CPU stores,
// pushes START / 0xA0 / 0xA5 / 0x3C / STOP) -> the core executes EBREAK and HALTS -> SM0 keeps
// clocking the bus.
//
// The bench is an open-drain I2C bus (SDA = uio[4], SCL = uio[5], wired-AND, pulled up) with a
// behavioural slave at 0x50. It checks: every byte is ACKed, the bytes are 0xA0 (addr/W), 0xA5,
// 0x3C, exactly one START and one STOP, no SDA edge while SCL is high other than START/STOP,
// and that the core halted before the STOP (PIO generated it on its own).

module tb_pio_cpu_i2c;
    reg clk = 0;
    reg rst_n;
    reg [7:0] ui_in;
    wire [7:0] uo_out, uio_out, uio_oe;
    always #5 clk = ~clk;

    wire cs0 = uio_out[0], cs1 = uio_out[6], sck = uio_out[3], mosi = uio_out[1];
    wire miso_flash, miso_psram;
    wire miso_bus = (!cs0) ? miso_flash : (!cs1) ? miso_psram : 1'b0;

    // ---- open-drain I2C bus: uio[4] = SDA, uio[5] = SCL ----
    reg  slave_sda_low;
    wire m_sda_low = uio_oe[4] & ~uio_out[4];
    wire m_scl_low = uio_oe[5] & ~uio_out[5];
    wire sda = ~(m_sda_low | slave_sda_low);
    wire scl = ~m_scl_low;
    wire [7:0] uio_in = {2'b0, scl, sda, 1'b0, miso_bus, 2'b0};

    tt_um_agila32 dut (.ui_in(ui_in), .uo_out(uo_out), .uio_in(uio_in),
                       .uio_out(uio_out), .uio_oe(uio_oe), .ena(1'b1),
                       .clk(clk), .rst_n(rst_n));

    spi_ram_model u_flash (.cs_n(cs0), .sck(sck), .mosi(mosi), .miso(miso_flash));
    spi_ram_model u_psram (.cs_n(cs1), .sck(sck), .mosi(mosi), .miso(miso_psram));

    reg [7:0] image [0:1495];
    integer ci;
    initial begin
        $readmemh("pio_i2c_flash_image.hex", image);
        for (ci = 0; ci < 1496; ci = ci + 1) u_flash.mem[ci] = image[ci];
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

    // ---- behavioural I2C slave, address 0x50, write-only ----
    reg [7:0] rx_bytes [0:7];
    integer   nbytes, starts, stops, violations;
    reg       in_txn, ack_phase;
    integer   bitcnt;
    reg [7:0] shreg;
    reg       first_byte;
    reg       last_ack;

    initial begin
        slave_sda_low = 0; nbytes = 0; starts = 0; stops = 0; violations = 0;
        in_txn = 0; bitcnt = 0; shreg = 0; first_byte = 0; last_ack = 0;
    end

    always @(negedge sda) if (rst_n === 1'b1 && scl === 1'b1) begin          // START
        starts = starts + 1; in_txn = 1; bitcnt = 0; shreg = 0; first_byte = 1; slave_sda_low = 0;
    end
    always @(posedge sda) if (rst_n === 1'b1 && scl === 1'b1) begin          // STOP
        stops = stops + 1; in_txn = 0; slave_sda_low = 0;
    end
    always @(posedge scl) if (in_txn) begin
        if (bitcnt < 8) begin shreg = {shreg[6:0], sda}; bitcnt = bitcnt + 1; end
        else            bitcnt = 9;                        // ACK slot sampled by master
    end
    always @(negedge scl) if (in_txn) begin
        if (bitcnt == 8) begin                             // byte complete: record, ACK
            rx_bytes[nbytes] = shreg; nbytes = nbytes + 1;
            last_ack = first_byte ? (shreg[7:1] == 7'h50) : 1'b1;
            first_byte = 0;
            if (last_ack) slave_sda_low = 1;
        end else if (bitcnt == 9) begin                    // ACK slot over: release
            slave_sda_low = 0; bitcnt = 0; shreg = 0;
        end
    end

    // ---- halt tracking ----
    reg       halt_seen;
    integer   nbytes_at_halt, stops_at_halt;
    initial begin halt_seen = 0; nbytes_at_halt = -1; stops_at_halt = -1; end
    always @(posedge clk) if (dut.halted && !halt_seen) begin
        halt_seen = 1; nbytes_at_halt = nbytes; stops_at_halt = stops;
    end

    integer errors, i;
    reg [7:0] exp [0:2];

    initial begin
        exp[0] = 8'hA0; exp[1] = 8'hA5; exp[2] = 8'h3C;
        $readmemh("flash_handoff_stub.hex", stub_bytes);
        errors = 0;
        ui_in = 8'h00;
        rst_n = 0; repeat (10) @(posedge clk); rst_n = 1;
        dut.u_mem.qspi_div_sel = 2'd0;
        repeat (3000) @(posedge clk);

        ui_in[2] = 1;
        repeat (20) @(posedge clk);
        boot_send_byte(STUB_LEN);
        for (ci = 0; ci < STUB_LEN; ci = ci + 1) boot_send_byte(stub_bytes[ci]);

        // wait for STOP (transaction over), with a generous cap
        i = 0;
        while (stops == 0 && i < 400000) begin @(posedge clk); i = i + 1; end
        repeat (200) @(posedge clk);

        if (nbytes != 3) begin errors = errors + 1; $display("FAIL: slave saw %0d bytes, expected 3", nbytes); end
        for (i = 0; i < 3 && i < nbytes; i = i + 1) begin
            if (rx_bytes[i] !== exp[i]) begin
                errors = errors + 1;
                $display("FAIL: byte %0d = 0x%02x, expected 0x%02x", i, rx_bytes[i], exp[i]);
            end else
                $display("PASS: byte %0d = 0x%02x", i, rx_bytes[i]);
        end
        if (starts != 1 || stops != 1) begin errors = errors + 1; $display("FAIL: starts=%0d stops=%0d (expected 1/1)", starts, stops); end
        else $display("PASS: exactly one START and one STOP, so no stray SDA edges while SCL was high");
        // The CPU can only park once every word is queued; with a 4-deep TX FIFO that is the last
        // 4 words = the STOP sequence. So: the core must have halted BEFORE the STOP condition, and
        // the STOP must still have been generated by PIO alone afterwards.
        if (!halt_seen) begin errors = errors + 1; $display("FAIL: core never halted"); end
        else if (stops_at_halt != 0) begin
            errors = errors + 1;
            $display("FAIL: core halted only after the STOP had already been sent");
        end else
            $display("PASS: core halted (EBREAK) before the STOP; PIO generated the STOP with the CPU parked");

        if (errors == 0) $display("PASS tb_pio_cpu_i2c: CPU programmed PIO, halted, PIO wrote A0 A5 3C to slave 0x50");
        else             $display("FAIL tb_pio_cpu_i2c: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #200000000;
        $display("TIMEOUT (bytes=%0d starts=%0d stops=%0d)", nbytes, starts, stops);
        $finish;
    end
endmodule
