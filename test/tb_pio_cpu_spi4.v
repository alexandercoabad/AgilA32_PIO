`timescale 1ns/1ps

// tb_pio_cpu_spi4.v -- ONE flash image, ONE PIO state machine, FOUR SPI modes (0-3), CPU in the loop.
//
// The image from tools/build_pio_spi4.py makes the CPU reprogram PIO SM0 for SPI mode 0, 1, 2 and 3,
// drive chip-select itself (GPIO_OUT bit 2 = uo_out[2]), and RELAY data: the MISO byte it reads back
// from the RX FIFO in mode N is sent out on MOSI as the first byte of mode N+1.
//
//   MOSI = uo_out[0]   SCK = uo_out[1]   CS_n = uo_out[2]   MISO = ui_in[2]
//
// The slave model below decodes the wire per CPOL/CPHA from the *pins only* (it never looks at the
// PIO program), and checks, for every mode:
//   * SCK idles at CPOL when CS falls and again when CS rises
//   * exactly 16 rising + 16 falling SCK edges inside the CS window
//   * both MOSI bytes are right, MSB first
//   * MOSI is stable >= 4 clocks before and after every sampling edge (setup / hold)
// and overall:
//   * MOSI byte 0 of mode N+1 == MISO byte 0 of mode N  (the CPU really read MISO and re-sent it)
//   * the last MISO byte the CPU received ends up on GPIO_OUT (0xF0)
//   * no SCK edge occurs while CS is high, except the single intentional idle-level change (0 -> 1)
//     between mode 1 and mode 2
//   * the four windows are strictly sequential; the core finishes with EBREAK

module tb_pio_cpu_spi4;
    reg clk = 0;
    reg rst_n;
    reg [7:0] ui_in;
    wire [7:0] uo_out, uio_out, uio_oe;
    always #5 clk = ~clk;

    wire cs0 = uio_out[0], cs1 = uio_out[6], sck_q = uio_out[3], mosi_q = uio_out[1];
    wire miso_flash, miso_psram;
    wire miso_bus = (!cs0) ? miso_flash : (!cs1) ? miso_psram : 1'b0;
    wire [7:0] uio_in = {2'b0, 2'b0, 1'b0, miso_bus, 2'b0};

    tt_um_agila32 dut (.ui_in(ui_in), .uo_out(uo_out), .uio_in(uio_in),
                       .uio_out(uio_out), .uio_oe(uio_oe), .ena(1'b1),
                       .clk(clk), .rst_n(rst_n));

    spi_ram_model u_flash (.cs_n(cs0), .sck(sck_q), .mosi(mosi_q), .miso(miso_flash));
    spi_ram_model u_psram (.cs_n(cs1), .sck(sck_q), .mosi(mosi_q), .miso(miso_psram));

    reg [7:0] image [0:8191];
    integer ci;
    initial begin
        for (ci = 0; ci < 8192; ci = ci + 1) image[ci] = 8'hxx;
        $readmemh("pio_spi4_flash_image.hex", image);
        for (ci = 0; ci < 8192; ci = ci + 1) if (image[ci] !== 8'hxx) u_flash.mem[ci] = image[ci];
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

    // ------------------------------------------------------------------ test vectors (== builder)
    reg [7:0] exp_mosi [0:7];     // {byte0, byte1} per mode
    reg [7:0] miso_a [0:3];       // first MISO byte per mode
    reg [7:0] miso_b [0:3];       // second MISO byte per mode
    initial begin
        exp_mosi[0] = 8'hA5; exp_mosi[1] = 8'h3C;
        exp_mosi[2] = 8'hC3; exp_mosi[3] = 8'h81;
        exp_mosi[4] = 8'h5A; exp_mosi[5] = 8'hE7;
        exp_mosi[6] = 8'h96; exp_mosi[7] = 8'h18;
        miso_a[0] = 8'hC3; miso_a[1] = 8'h5A; miso_a[2] = 8'h96; miso_a[3] = 8'h6D;
        miso_b[0] = 8'hF0; miso_b[1] = 8'h0F; miso_b[2] = 8'hAA; miso_b[3] = 8'h55;
    end

    // ------------------------------------------------------------------ SPI slave (pins only)
    wire       cs_n = uo_out[2];
    wire       sck  = uo_out[1];
    wire       mosi = uo_out[0];

    reg        slave_on;                 // set once the firmware has started the PIO
    reg        active;
    integer    phase, mode, cpol, cpha;
    reg        samp_on_rise;
    integer    sbits, nbytes, mi;
    reg [7:0]  mo_sh;
    reg [7:0]  mo [0:7];
    reg [15:0] mtx;
    integer    rises [0:3], falls [0:3];
    integer    idle_fail, setup_fail, hold_fail, oob_edges, oob_rise_ok;
    time       t_mosi_chg, t_sample, t_cs_fall [0:3], t_cs_rise [0:3];
    integer    win_edge_phase_of_oob;      // phase index the single out-of-window edge followed

    initial begin
        slave_on = 0; active = 0; phase = 0; mode = 0; cpol = 0; cpha = 0; samp_on_rise = 1;
        sbits = 0; nbytes = 0; mi = 0; mo_sh = 0; mtx = 0;
        idle_fail = 0; setup_fail = 0; hold_fail = 0; oob_edges = 0; oob_rise_ok = 0;
        t_mosi_chg = 0; t_sample = 0; win_edge_phase_of_oob = -1;
        for (ci = 0; ci < 4; ci = ci + 1) begin
            rises[ci] = 0; falls[ci] = 0; t_cs_fall[ci] = 0; t_cs_rise[ci] = 0;
        end
        for (ci = 0; ci < 8; ci = ci + 1) mo[ci] = 8'hxx;
    end

    // firmware takes over ui_in[2] as MISO once the PIO is enabled (before that it is the boot START pin)
    always @(posedge clk) if (!slave_on && dut.u_pio.sm_en[0] === 1'b1) begin
        slave_on = 1; ui_in[2] = 1'b0;
    end

    task automatic do_sample;
        begin
            if ($time - t_mosi_chg < 40) setup_fail = setup_fail + 1;
            t_sample = $time;
            mo_sh = {mo_sh[6:0], mosi};
            sbits = sbits + 1;
            if (sbits == 8) begin
                mo[mode * 2 + nbytes] = mo_sh;
                nbytes = nbytes + 1; sbits = 0;
            end
        end
    endtask
    task automatic do_shift;
        begin
            if (mi < 16) begin ui_in[2] = mtx[15 - mi]; mi = mi + 1; end
        end
    endtask

    always @(negedge cs_n) if (slave_on && phase < 4 && rst_n === 1'b1) begin     // CS asserted
        mode = phase; phase = phase + 1;
        cpol = mode >> 1; cpha = mode & 1;
        samp_on_rise = ((cpol ^ cpha) == 0);
        active = 1; sbits = 0; nbytes = 0; mi = 0; mo_sh = 0;
        mtx = {miso_a[mode], miso_b[mode]};
        t_cs_fall[mode] = $time; t_sample = 0;
        if (sck !== cpol[0]) idle_fail = idle_fail + 1;                          // idle level = CPOL
        ui_in[2] = 1'b0;
        if (cpha == 0) do_shift;                                                  // first bit ready before 1st edge
    end
    always @(posedge cs_n) if (active) begin                                     // CS released
        active = 0; t_cs_rise[mode] = $time;
        if (sck !== cpol[0]) idle_fail = idle_fail + 1;
        if (nbytes != 2 || sbits != 0) begin
            $display("FAIL: mode %0d CS released after %0d bytes + %0d bits", mode, nbytes, sbits);
            idle_fail = idle_fail + 1;
        end
        ui_in[2] = 1'b0;
    end
    always @(posedge sck) if (slave_on && phase > 0) begin
        if (active) begin
            rises[mode] = rises[mode] + 1;
            if (samp_on_rise) do_sample; else do_shift;
        end else begin
            oob_edges = oob_edges + 1;
            if (phase == 2) begin oob_rise_ok = oob_rise_ok + 1; win_edge_phase_of_oob = phase; end
        end
    end
    always @(negedge sck) if (slave_on && phase > 0) begin
        if (active) begin
            falls[mode] = falls[mode] + 1;
            if (!samp_on_rise) do_sample; else do_shift;
        end else oob_edges = oob_edges + 1;
    end
    always @(mosi) if (slave_on) begin
        t_mosi_chg = $time;
        if (active && t_sample != 0 && ($time - t_sample) < 40) hold_fail = hold_fail + 1;
    end

    // ------------------------------------------------------------------ halt tracking
    reg halt_seen; time t_halt;
    initial begin halt_seen = 0; t_halt = 0; end
    always @(posedge clk) if (dut.halted && !halt_seen) begin halt_seen = 1; t_halt = $time; end

    // ------------------------------------------------------------------ main
    integer errors, i, m;
    integer slow;
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

        slow = 0;
        while (!halt_seen && slow < 60000000) begin @(posedge clk); slow = slow + 1; end
        repeat (400) @(posedge clk);

        if (!halt_seen) begin errors = errors + 1; $display("FAIL: core never halted (phase=%0d)", phase); end

        // ---- one block of checks per SPI mode
        for (m = 0; m < 4; m = m + 1) begin
            if (mo[m*2] !== exp_mosi[m*2] || mo[m*2+1] !== exp_mosi[m*2+1]) begin
                errors = errors + 1;
                $display("FAIL: mode %0d MOSI = %02x %02x, expected %02x %02x", m,
                         mo[m*2], mo[m*2+1], exp_mosi[m*2], exp_mosi[m*2+1]);
            end
            if (rises[m] != 16 || falls[m] != 16) begin
                errors = errors + 1;
                $display("FAIL: mode %0d SCK edges rise=%0d fall=%0d, expected 16/16", m, rises[m], falls[m]);
            end
            if (mo[m*2] === exp_mosi[m*2] && mo[m*2+1] === exp_mosi[m*2+1] && rises[m] == 16 && falls[m] == 16)
                $display("PASS: SPI mode %0d (CPOL=%0d CPHA=%0d): MOSI %02x %02x, 16 clocks, idle SCK=%0d",
                         m, m >> 1, m & 1, mo[m*2], mo[m*2+1], m >> 1);
        end

        if (phase != 4) begin errors = errors + 1; $display("FAIL: only %0d chip-select windows seen, expected 4", phase); end
        if (idle_fail != 0) begin errors = errors + 1; $display("FAIL: %0d SCK idle-level / framing violations at CS edges", idle_fail); end
        else $display("PASS: SCK idled at CPOL when CS fell and rose, in all four modes");
        if (setup_fail != 0 || hold_fail != 0) begin
            errors = errors + 1; $display("FAIL: MOSI setup violations=%0d hold violations=%0d (min 4 clocks)", setup_fail, hold_fail);
        end else $display("PASS: MOSI stable >= 4 clocks before and after every sampling edge");

        // ---- the CPU relayed MISO -> MOSI
        for (m = 0; m < 3; m = m + 1)
            if (mo[(m+1)*2] !== miso_a[m]) begin
                errors = errors + 1;
                $display("FAIL: relay %0d->%0d: MOSI 0x%02x != MISO 0x%02x", m, m+1, mo[(m+1)*2], miso_a[m]);
            end
        if (mo[2] === miso_a[0] && mo[4] === miso_a[1] && mo[6] === miso_a[2])
            $display("PASS: CPU relayed MISO -> MOSI across modes (0x%02x, 0x%02x, 0x%02x)", miso_a[0], miso_a[1], miso_a[2]);

        // ---- last MISO byte reached the CPU and was shown on the LED pads
        if (dut.u_mem.gpio_out !== miso_a[3]) begin
            errors = errors + 1; $display("FAIL: GPIO_OUT = 0x%02x, expected the last MISO byte 0x%02x", dut.u_mem.gpio_out, miso_a[3]);
        end else $display("PASS: last MISO byte 0x%02x reached the CPU and GPIO_OUT", miso_a[3]);

        // ---- SCK is quiet while CS is high, except the one intentional idle-level change
        if (oob_edges != 1 || oob_rise_ok != 1) begin
            errors = errors + 1; $display("FAIL: %0d SCK edges outside CS windows (allowed: exactly 1, the mode1->mode2 idle change)", oob_edges);
        end else $display("PASS: no SCK edges while CS high, except the single intentional 0->1 idle change before mode 2");

        // ---- windows strictly sequential
        for (m = 1; m < 4; m = m + 1)
            if (!(t_cs_fall[m] > t_cs_rise[m-1] && t_cs_rise[m-1] != 0)) begin
                errors = errors + 1; $display("FAIL: CS windows %0d and %0d overlap", m-1, m);
            end
        if (halt_seen && t_halt > t_cs_rise[3]) $display("PASS: four modes ran strictly in sequence; core then halted (EBREAK)");
        else begin errors = errors + 1; $display("FAIL: halt ordering (halt=%0t last CS rise=%0t)", t_halt, t_cs_rise[3]); end

        if (errors == 0) $display("PASS tb_pio_cpu_spi4: one image, one PIO, SPI modes 0-3, CPU relayed data through every mode");
        else             $display("FAIL tb_pio_cpu_spi4: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #1500000000;
        $display("TIMEOUT (phase=%0d halted=%0d)", phase, halt_seen);
        $finish;
    end
endmodule
