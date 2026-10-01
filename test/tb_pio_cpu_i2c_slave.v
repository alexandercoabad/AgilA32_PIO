`timescale 1ns/1ps

// tb_pio_cpu_i2c_slave.v -- END-TO-END I2C slave: real RV32I core + real PIO block in the real top level.
//
// The image from tools/build_pio_i2c_slave.py makes the CPU:
//   phase 1  load pio/i2c_slave_rx.pio (address 0x42, 2 data bytes), enable it, and wait on the RX FIFO;
//            the PIO alone does START detect / address compare / ACK / shifting while the bench's
//            I2C master WRITES  A5 3C.  The CPU reads the two bytes.
//   phase 2  computes byte + 1, swaps in pio/i2c_slave_tx.pio (the two slave programs cannot share the
//            32-word instruction memory), queues A6 3D and EBREAKs.
// The bench then READS two bytes from 0x42 with the CPU halted: they must be A6 3D.
//
// SDA = uio[4], SCL = uio[5], wired-AND with the pads' open-drain drive (PINDIR = 1 pulls low), SCL
// period 128 clocks (the slaves need >= 32, see docs/info.md).

module tb_pio_cpu_i2c_slave;
    reg clk = 0;
    reg rst_n;
    reg [7:0] ui_in;
    wire [7:0] uo_out, uio_out, uio_oe;
    always #5 clk = ~clk;

    wire cs0 = uio_out[0], cs1 = uio_out[6], sck = uio_out[3], mosi = uio_out[1];
    wire miso_flash, miso_psram;
    wire miso_bus = (!cs0) ? miso_flash : (!cs1) ? miso_psram : 1'b0;

    // ---- open-drain I2C bus: uio[4] = SDA, uio[5] = SCL ----
    reg  m_sda_low, m_scl_low;
    wire p_sda_low = uio_oe[4] & ~uio_out[4];
    wire p_scl_low = uio_oe[5] & ~uio_out[5];
    wire sda = ~(p_sda_low | m_sda_low);
    wire scl = ~(p_scl_low | m_scl_low);
    wire [7:0] uio_in = {2'b0, scl, sda, 1'b0, miso_bus, 2'b0};

    tt_um_agila32 dut (.ui_in(ui_in), .uo_out(uo_out), .uio_in(uio_in),
                       .uio_out(uio_out), .uio_oe(uio_oe), .ena(1'b1),
                       .clk(clk), .rst_n(rst_n));

    spi_ram_model u_flash (.cs_n(cs0), .sck(sck), .mosi(mosi), .miso(miso_flash));
    spi_ram_model u_psram (.cs_n(cs1), .sck(sck), .mosi(mosi), .miso(miso_psram));

    reg [7:0] image [0:1979];
    integer ci;
    initial begin
        $readmemh("pio_i2c_slave_flash_image.hex", image);
        for (ci = 0; ci < 1980; ci = ci + 1) u_flash.mem[ci] = image[ci];
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

    // ================= behavioural I2C master (half period H = 64 clocks) =================
    localparam integer H = 64;
    integer scl_stolen;                       // clocks the slave held SCL low while we had released it
    reg     in_write_txn;
    initial begin m_sda_low = 0; m_scl_low = 0; scl_stolen = 0; in_write_txn = 0; end

    task automatic dly(input integer n); begin repeat (n) @(posedge clk); end endtask

    task automatic sclhi;                     // release SCL, wait until it is really high (clock stretching)
        begin
            m_scl_low = 0; @(posedge clk);
            while (scl !== 1'b1) begin
                @(posedge clk);
                if (in_write_txn && p_scl_low) scl_stolen = scl_stolen + 1;
            end
        end
    endtask
    task automatic i2c_start;
        begin m_sda_low = 0; sclhi; dly(H); m_sda_low = 1; dly(H); m_scl_low = 1; dly(H); end
    endtask
    task automatic i2c_stop;
        begin m_sda_low = 1; m_scl_low = 1; dly(H); sclhi; dly(H); m_sda_low = 0; dly(H); end
    endtask
    task automatic i2c_write(input [7:0] b, output ack);
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                m_sda_low = ~b[i]; dly(H); sclhi; dly(H); m_scl_low = 1;
            end
            m_sda_low = 0; dly(H); sclhi; dly(H/2); ack = ~sda; dly(H/2); m_scl_low = 1; dly(H);
        end
    endtask
    task automatic i2c_read(input nack, output [7:0] b);
        integer i;
        begin
            m_sda_low = 0;
            for (i = 7; i >= 0; i = i - 1) begin
                dly(H); sclhi; dly(H/2); b[i] = sda; dly(H/2); m_scl_low = 1;
            end
            m_sda_low = ~nack; dly(H); sclhi; dly(H); m_scl_low = 1; dly(H/2); m_sda_low = 0; dly(H/2);
        end
    endtask

    // ---- halt tracking ----
    reg halt_seen; time t_halt, t_read_start;
    initial begin halt_seen = 0; t_halt = 0; t_read_start = 0; end
    always @(posedge clk) if (dut.halted && !halt_seen) begin halt_seen = 1; t_halt = $time; end

    integer errors;
    reg a0, a1, a2, a3;
    reg [7:0] r0, r1;

    initial begin
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

        // ---- phase 1: wait until the CPU has enabled the RX slave, then WRITE A5 3C to 0x42
        wait (dut.u_pio.sm_en[0] === 1'b1);
        dly(3000);
        in_write_txn = 1;
        i2c_start;
        i2c_write(8'h84, a0);                 // 0x42, W
        i2c_write(8'hA5, a1);
        i2c_write(8'h3C, a2);
        i2c_stop;
        in_write_txn = 0;
        if ({a0, a1, a2} !== 3'b111) begin errors = errors + 1; $display("FAIL: write ACKs addr/d0/d1 = %b%b%b, expected 111", a0, a1, a2); end
        else $display("PASS: phase 1: RX slave ACKed address 0x42 and both data bytes (CPU parked on the RX FIFO, PIO did the bits)");
        if (scl_stolen != 0) begin errors = errors + 1; $display("FAIL: the write-direction slave pulled SCL low (%0d clocks)", scl_stolen); end

        // ---- phase 2: the CPU reloads PIO as the read slave and halts; then READ two bytes
        i = 0;
        wait (halt_seen);
        dly(1500);
        t_read_start = $time;
        i2c_start;
        i2c_write(8'h85, a3);                 // 0x42, R
        i2c_read(1'b0, r0);                   // ACK
        i2c_read(1'b1, r1);                   // NAK on the last byte
        i2c_stop;
        if (a3 !== 1'b1) begin errors = errors + 1; $display("FAIL: read-direction slave did not ACK its address"); end
        if (r0 !== 8'hA6 || r1 !== 8'h3D) begin
            errors = errors + 1; $display("FAIL: read back %02x %02x, expected A6 3D (written A5 3C, +1 by the CPU)", r0, r1);
        end else
            $display("PASS: phase 2: master read back %02x %02x = written bytes + 1, computed by the CPU, served by PIO", r0, r1);
        if (!(halt_seen && t_halt < t_read_start)) begin errors = errors + 1; $display("FAIL: core had not halted before the read"); end
        else $display("PASS: core halted (EBREAK) before the read; the TX slave answered with the CPU parked");

        if (errors == 0) $display("PASS tb_pio_cpu_i2c_slave: write 0x42 A5 3C, CPU +1, read back A6 3D, one PIO, two slave programs");
        else             $display("FAIL tb_pio_cpu_i2c_slave: %0d error(s)", errors);
        $finish;
    end

    integer i;
    initial begin
        #1200000000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
