`timescale 1ns/1ps

// tb_pio_cpu_onewire.v -- END-TO-END 1-Wire: real RV32I core + real PIO block in the real top level.
//
// The image from tools/build_pio_onewire.py makes the CPU load pio/onewire.pio (CLKDIV 24 = a 1 us tick
// at 24 MHz), then act as the 1-Wire bus master:
//     reset -> LED_OUT = 0xA0 (a device answered, 0xA1 if nobody did)
//     send READ ROM (0x33)
//     read 8 bytes, LED_OUT = each one in turn.
//
// The bench is a 1-Wire device on uio[4] (open drain, wired-AND with the PIO, pulled up). It is timed in
// clocks (24 clocks = 1 us) and judges the master like a real device would:
//   * a low pulse >= 480 us is a reset; it answers 30 us after the release with a 120 us presence pulse
//   * after a reset it decodes command bits (a write slot is a 0 if the line is still low 30 us after the
//     falling edge, else a 1), LSB first; READ ROM (0x33) makes it send its 64-bit ROM, LSB first, holding
//     each 0 for only 15 us after the falling edge (the spec minimum)
//   * it flags a low pulse < 1 us, a low pulse that is neither write-1/read (<= 15 us), write-0 (60..120 us)
//     nor reset, a slot < 60 us after the previous one, and < 1 us of recovery.
// Pass = the CPU saw presence, the 8 ROM bytes arrive on uo_out in order and their CRC-8 is 0, the device
// saw exactly one reset, the command 0x33, 8 write slots and 64 read slots, and flagged nothing.

module tb_pio_cpu_onewire;
    reg clk = 0;
    reg rst_n;
    reg [7:0] ui_in;
    wire [7:0] uo_out, uio_out, uio_oe;
    always #5 clk = ~clk;

    wire cs0 = uio_out[0], cs1 = uio_out[6], sck = uio_out[3], mosi = uio_out[1];
    wire miso_flash, miso_psram;
    wire miso_bus = (!cs0) ? miso_flash : (!cs1) ? miso_psram : 1'b0;

    // ---- the 1-Wire bus: uio[4], open drain, pulled up. The PIO pulls it low with oe=1, out=0.
    wire dq_master_low = uio_oe[4] & ~uio_out[4];
    reg  dev_low;
    wire dq = ~(dq_master_low | dev_low);
    wire [7:0] uio_in = {3'b000, dq, 1'b0, miso_bus, 2'b00};

    tt_um_agila32 dut (.ui_in(ui_in), .uo_out(uo_out), .uio_in(uio_in),
                       .uio_out(uio_out), .uio_oe(uio_oe), .ena(1'b1),
                       .clk(clk), .rst_n(rst_n));

    spi_ram_model u_flash (.cs_n(cs0), .sck(sck), .mosi(mosi), .miso(miso_flash));
    spi_ram_model u_psram (.cs_n(cs1), .sck(sck), .mosi(mosi), .miso(miso_psram));

    localparam integer IMG_LEN = 2420;
    reg [7:0] image [0:IMG_LEN-1];
    integer ci;
    initial begin
        $readmemh("pio_onewire_flash_image.hex", image);
        for (ci = 0; ci < IMG_LEN; ci = ci + 1) u_flash.mem[ci] = image[ci];
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

    // ================= the device =================
    localparam integer US = 24;                      // clocks per microsecond
    reg [7:0] rom [0:7];                             // rom[0] = family code ... rom[7] = CRC
    integer now;
    reg     ml_prev;
    integer t_fall, t_rise, last_fall, last_rise;
    integer drive_from, drive_until, pres_at;
    integer n_reset, n_wr, n_rd, viol, low_len;
    integer reset_low, first_gap, reset_rise;
    reg     sending, in_cmd;
    reg [7:0] cmd_sh, cmd_byte;
    integer cmd_n, tx_i;
    reg     got_cmd, bitv, first_slot_seen;

    initial begin
        now = 0; ml_prev = 0; dev_low = 0;
        t_fall = 0; t_rise = 0; last_fall = -10000000; last_rise = -10000000;
        drive_from = 0; drive_until = 0; pres_at = -1;
        n_reset = 0; n_wr = 0; n_rd = 0; viol = 0; reset_low = 0; first_gap = 0; reset_rise = 0;
        sending = 0; in_cmd = 0; cmd_n = 0; tx_i = 0; got_cmd = 0; cmd_byte = 0; cmd_sh = 0;
        first_slot_seen = 0;
    end

    task flag(input [255:0] msg);
        begin viol = viol + 1; $display("DEVICE FLAG @%0d clk: %0s", now, msg); end
    endtask

    always @(posedge clk) begin
        now = now + 1;
        dev_low <= (now >= drive_from) && (now < drive_until);
        if (pres_at >= 0 && now >= pres_at) begin
            drive_from = now; drive_until = now + 120 * US; pres_at = -1;
        end
        // ---- falling edge of the master's pull-down
        if (dq_master_low && !ml_prev) begin
            if (now - last_rise < 1 * US) flag("recovery < 1 us");
            if (n_reset > 0 && (now - last_fall) < 60 * US) flag("slot < 60 us after the previous");
            if (n_reset > 0 && !first_slot_seen && !(sending)) begin
                first_slot_seen = 1; first_gap = now - reset_rise;
            end
            t_fall = now; last_fall = now;
            if (sending) begin                        // a read slot: act NOW
                bitv = rom[tx_i / 8][tx_i % 8];
                tx_i = tx_i + 1;
                if (!bitv) begin drive_from = now + 1; drive_until = now + 15 * US; end
            end
        end
        // ---- release
        else if (!dq_master_low && ml_prev) begin
            low_len = now - t_fall; last_rise = now;
            if (low_len >= 480 * US) begin           // RESET
                n_reset = n_reset + 1; reset_low = low_len; reset_rise = now;
                in_cmd = 1; sending = 0; cmd_n = 0; first_slot_seen = 0;
                pres_at = now + 30 * US;
            end else begin
                if (low_len < 1 * US) flag("low pulse < 1 us");
                else if ((low_len > 15 * US && low_len < 60 * US) || low_len > 120 * US)
                    flag("low pulse is neither write-1/read, write-0 nor reset");
                if (sending) begin
                    n_rd = n_rd + 1;
                    if (low_len > 15 * US) flag("read slot low pulse > 15 us");
                end else if (in_cmd) begin
                    bitv = (low_len > 30 * US) ? 1'b0 : 1'b1;
                    cmd_sh = {bitv, cmd_sh[7:1]};     // LSB first
                    cmd_n = cmd_n + 1; n_wr = n_wr + 1;
                    if (cmd_n == 8) begin
                        cmd_byte = cmd_sh; got_cmd = 1; cmd_n = 0;
                        if (cmd_sh == 8'h33) begin sending = 1; tx_i = 0; end
                    end
                end
            end
        end
        ml_prev = dq_master_low;
    end

    // ---- what the CPU shows on uo_out: record every change once the PIO is running ----
    reg [7:0] seen [0:15];
    integer   nseen;
    reg       watch;
    reg [7:0] last_uo;
    initial begin nseen = 0; watch = 0; last_uo = 8'h00; end
    always @(posedge clk) if (watch && uo_out !== last_uo) begin
        if (nseen < 16) seen[nseen] = uo_out;
        nseen = nseen + 1;
        last_uo = uo_out;
    end

    function [7:0] crc8_step(input [7:0] crc, input [7:0] d);
        integer k;
        reg [7:0] c;
        begin
            c = crc ^ d;
            for (k = 0; k < 8; k = k + 1) c = c[0] ? ((c >> 1) ^ 8'h8C) : (c >> 1);
            crc8_step = c;
        end
    endfunction

    integer errors, i;
    reg [7:0] crc;

    initial begin
        rom[0] = 8'h28; rom[1] = 8'h6B; rom[2] = 8'h13; rom[3] = 8'h9C;
        rom[4] = 8'h00; rom[5] = 8'hA4; rom[6] = 8'h07;
        crc = 0;
        for (i = 0; i < 7; i = i + 1) crc = crc8_step(crc, rom[i]);
        rom[7] = crc;

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

        wait (dut.u_pio.sm_en[0] === 1'b1);
        repeat (300) @(posedge clk);
        last_uo = uo_out; watch = 1;
        wait (dut.halted);
        repeat (2000) @(posedge clk);

        // ---- what the CPU reported
        if (nseen != 9) begin errors = errors + 1; $display("FAIL: uo_out changed %0d times, expected 9 (presence + 8 ROM bytes)", nseen); end
        else if (seen[0] !== 8'hA0) begin errors = errors + 1; $display("FAIL: presence flag %02x, expected A0 (a device answered)", seen[0]); end
        else $display("PASS: the CPU saw the presence pulse (LED_OUT = A0)");
        for (i = 0; i < 8 && i + 1 < nseen; i = i + 1)
            if (seen[i + 1] !== rom[i]) begin errors = errors + 1; $display("FAIL: ROM byte %0d = %02x, expected %02x", i, seen[i + 1], rom[i]); end
        crc = 0;
        for (i = 1; i < 9 && i < nseen; i = i + 1) crc = crc8_step(crc, seen[i]);
        if (nseen == 9 && crc !== 8'h00) begin errors = errors + 1; $display("FAIL: CRC-8 of the 8 bytes the CPU read is %02x, expected 00", crc); end
        else if (nseen == 9 && seen[1] === rom[0] && seen[8] === rom[7])
            $display("PASS: the CPU read the ROM %02x %02x %02x %02x %02x %02x %02x %02x, CRC-8 valid",
                     seen[1], seen[2], seen[3], seen[4], seen[5], seen[6], seen[7], seen[8]);

        // ---- what the device saw
        if (n_reset !== 1) begin errors = errors + 1; $display("FAIL: device saw %0d resets, expected 1", n_reset); end
        if (reset_low < 480 * US || reset_low > 960 * US) begin errors = errors + 1; $display("FAIL: reset low %0d us, expected 480..960", reset_low / US); end
        else $display("PASS: reset pulse %0d us low", reset_low / US);
        if (first_gap < 480 * US) begin errors = errors + 1; $display("FAIL: only %0d us between reset release and the first slot", first_gap / US); end
        else $display("PASS: %0d us between the reset release and the first slot (>= 480)", first_gap / US);
        if (!got_cmd || cmd_byte !== 8'h33) begin errors = errors + 1; $display("FAIL: command byte %02x, expected 33", cmd_byte); end
        else $display("PASS: the device decoded the ROM command 0x33 (8 write slots, LSB first)");
        if (n_wr !== 8) begin errors = errors + 1; $display("FAIL: %0d write slots, expected 8", n_wr); end
        if (n_rd !== 64) begin errors = errors + 1; $display("FAIL: %0d read slots, expected 64", n_rd); end
        else $display("PASS: 64 read slots, each 0 held by the device only 15 us and still read correctly");
        if (viol != 0) begin errors = errors + 1; $display("FAIL: the device flagged %0d timing violation(s)", viol); end
        else $display("PASS: no slot timing violation (write-1/read 1..15 us, write-0 60..120 us, >= 60 us slots, >= 1 us recovery)");
        if (dq !== 1'b1) begin errors = errors + 1; $display("FAIL: the bus is not released at the end"); end

        if (errors == 0) $display("PASS tb_pio_cpu_onewire: the CPU read a 1-Wire ROM through the PIO");
        else             $display("FAIL tb_pio_cpu_onewire: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #2000000000;
        $display("TIMEOUT (nseen=%0d n_reset=%0d n_wr=%0d n_rd=%0d)", nseen, n_reset, n_wr, n_rd);
        $finish;
    end
endmodule
