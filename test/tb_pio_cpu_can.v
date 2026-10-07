`timescale 1ns/1ps

// tb_pio_cpu_can.v -- END-TO-END CAN: real RV32I core + real PIO block (can_tx.pio + can_rx.pio) in the real top level.
//
// The image from tools/build_pio_can.py makes the CPU configure the two state machines (CLKDIV 4: 64 clocks per bit),
// send the standard frame ID 0x123 / DE AD, and then report through LED_OUT
//     the transmit result (0xFC = ACK slot dominant, the 11 bits after it recessive),
//     the RX capture of its own frame (4 bytes per word), then the RX capture of the next frame on the bus.
//
// The bench is the CAN bus (TXD = uo_out[0] AND the node's output, seen by the chip on ui_in[1]) and a second node:
//   * it hard-syncs on the SOF edge of the chip, samples the bus in the middle of every bit of frame A and compares
//     it with the expected stuffed frame (SOF .. CRC delimiter, ACK slot, 11 recessive bits),
//   * it drives the ACK slot dominant, as a receiver that decoded the frame would,
//   * it checks that every TXD edge of the strict part is on the 64 clock bit grid,
//   * a long time later it sends frame B (ID 0x2A5, 01 02 03) as a transmitter; nobody acknowledges it.
// Pass = the LED_OUT writes are exactly the expected bytes (tools/build_pio_can.py computed them with the reference
// model of the RX program), the wire carried frame A bit for bit with its ACK, TXD edges were on the grid, and TXD is
// recessive at the end.  The uo_out[0] pad belongs to the PIO, so the bench records the LED_OUT writes themselves.

module tb_pio_cpu_can;
`include "pio_can_expect.vh"
    reg clk = 0;
    reg rst_n;
    reg [7:0] ui_in;
    wire [7:0] uo_out, uio_out, uio_oe;
    always #5 clk = ~clk;

    wire cs0 = uio_out[0], cs1 = uio_out[6], sck = uio_out[3], mosi = uio_out[1];
    wire miso_flash, miso_psram;
    wire miso_bus = (!cs0) ? miso_flash : (!cs1) ? miso_psram : 1'b0;

    // ---- the CAN bus: TXD (uo_out[0]) AND the node; RXD (ui_in[1]) is the bus after the boot has finished
    wire [7:0] uio_in = {4'b0000, 1'b0, 1'b0, miso_bus, 2'b00};

    tt_um_agila32 dut (.ui_in(ui_in), .uo_out(uo_out), .uio_in(uio_in),
                       .uio_out(uio_out), .uio_oe(uio_oe), .ena(1'b1),
                       .clk(clk), .rst_n(rst_n));

    spi_ram_model u_flash (.cs_n(cs0), .sck(sck), .mosi(mosi), .miso(miso_flash));
    spi_ram_model u_psram (.cs_n(cs1), .sck(sck), .mosi(mosi), .miso(miso_psram));

    reg [7:0] image [0:CAN_IMG_LEN-1];
    integer ci;
    initial begin
        $readmemh("pio_can_flash_image.hex", image);
        for (ci = 0; ci < CAN_IMG_LEN; ci = ci + 1) u_flash.mem[ci] = image[ci];
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

    // ================= the bus and the second node =================
    localparam integer BC  = 16 * CAN_CLKDIV;         // clocks per bit
    localparam integer GAP = 60000;                   // clocks between the end of frame A and the SOF of frame B
    wire txd = uo_out[0];
    wire own0 = (dut.pio_pin_own[0] === 1'b1);        // the pad belongs to the PIO: TXD is meaningful
    reg  node_out;                                    // 1 = recessive
    wire bus = txd & node_out;
    reg  post_boot;
    always @(posedge clk) if (post_boot) ui_in[1] <= bus;

    integer now, t0, tb0, txd_prev, nedge, bad_edge, wire_err, i, errors;
    reg     watch, sof_seen;
    initial begin now = 0; t0 = 0; tb0 = 0; nedge = 0; bad_edge = 0; wire_err = 0; watch = 0; sof_seen = 0;
                  node_out = 1; post_boot = 0; txd_prev = 1; end

    always @(posedge clk) begin
        now = now + 1;
        if (!own0) txd_prev = txd;
        if (own0) begin
            if (!sof_seen && txd_prev && !txd) begin sof_seen = 1; t0 = now; tb0 = now + (CAN_NA + 12) * BC + GAP; end
            if (sof_seen && now < t0 + CAN_NA * BC && txd !== txd_prev) begin
                nedge = nedge + 1;
                if (((now - t0) % BC) != 0) begin bad_edge = bad_edge + 1; if (bad_edge < 6) $display("edge at +%0d (mod %0d = %0d)", now - t0, BC, (now - t0) % BC); end
            end
            txd_prev = txd;
            // second node: the ACK slot of frame A, then frame B
            node_out = 1;
            if (sof_seen && now >= t0 + CAN_NA * BC && now < t0 + (CAN_NA + 1) * BC) node_out = 0;
            if (sof_seen && now >= tb0 && now < tb0 + CAN_NB * BC) node_out = CAN_B_BITS[(now - tb0) / BC];
            // judge the wire of frame A in the middle of each bit
            if (sof_seen && now >= t0 + BC / 2 && now < t0 + (CAN_NA + 12) * BC && ((now - t0 - BC / 2) % BC) == 0)
                if (bus !== CAN_A_WIRE[(now - t0) / BC]) begin
                    wire_err = wire_err + 1;
                    $display("WIRE bit %0d = %b, expected %b", (now - t0) / BC, bus, CAN_A_WIRE[(now - t0) / BC]);
                end
        end
    end

    // ---- every LED_OUT write the CPU makes (the pad uo_out[0] belongs to the PIO, so look at the write itself)
    reg [7:0] seen [0:255];
    integer   nseen;
    initial nseen = 0;
    always @(posedge clk)
        if (watch && dut.mem_valid && dut.mem_we && dut.mem_ready && dut.mem_addr == 8'hF0) begin
            if (nseen < 256) seen[nseen] = dut.mem_wdata[7:0];
            nseen = nseen + 1;
        end

    initial begin
        load_can_expect;
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
        post_boot = 1;

        wait (dut.u_pio.sm_en[0] === 1'b1 && dut.u_pio.sm_en[1] === 1'b1);
        watch = 1;
        wait (dut.halted);
        repeat (2000) @(posedge clk);

        // ---- what the CPU reported
        if (nseen != CAN_NEXP) begin errors = errors + 1; $display("FAIL: the CPU wrote LED_OUT %0d times, expected %0d", nseen, CAN_NEXP); end
        for (i = 0; i < CAN_NEXP && i < nseen; i = i + 1)
            if (seen[i] !== can_exp[i]) begin errors = errors + 1; $display("FAIL: LED_OUT write %0d = %02x, expected %02x", i, seen[i], can_exp[i]); end
        if (nseen == CAN_NEXP && seen[0] === 8'hFC) $display("PASS: transmit result 0xFC (frame acknowledged, tail recessive)");
        if (errors == 0) $display("PASS: the CPU read the RX captures of its own frame and of the bus frame (%0d bytes, as modelled)", CAN_NEXP - 1);

        // ---- what the bus carried
        if (!sof_seen) begin errors = errors + 1; $display("FAIL: no SOF seen on TXD"); end
        if (wire_err != 0) begin errors = errors + 1; $display("FAIL: %0d wire bit(s) of frame A differ", wire_err); end
        else $display("PASS: frame A on the wire bit for bit (%0d bits, ACK slot dominant, 11 recessive bits after it)", CAN_NA + 12);
        if (bad_edge != 0 || nedge < 10) begin errors = errors + 1; $display("FAIL: %0d of %0d TXD edges off the 64 clock bit grid", bad_edge, nedge); end
        else $display("PASS: all %0d TXD edges of frame A on the %0d clock bit grid", nedge, BC);
        if (txd !== 1'b1 || bus !== 1'b1) begin errors = errors + 1; $display("FAIL: the bus is not recessive at the end"); end

        if (errors == 0) $display("PASS tb_pio_cpu_can: the CPU sent and received CAN frames through the PIO");
        else             $display("FAIL tb_pio_cpu_can: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #2000000000;
        $display("TIMEOUT (nseen=%0d sof_seen=%0d)", nseen, sof_seen);
        $finish;
    end
endmodule
