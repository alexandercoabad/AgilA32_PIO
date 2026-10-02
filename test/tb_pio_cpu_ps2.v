`timescale 1ns/1ps

// tb_pio_cpu_ps2.v -- END-TO-END PS/2 receiver: real RV32I core + real PIO block in the real top level.
//
// The image from tools/build_pio_ps2_rx.py makes the CPU load pio/ps2_rx.pio (CLKDIV 17), enable it,
// SLEEP for 170000 clocks, then drain the RX FIFO 7 times, writing each scancode to uo_out.
//
// The bench is a PS/2 keyboard on ui_in[3] (CLOCK) / ui_in[4] (DATA) running at a REAL 10 kHz (2400
// clocks per bit at the 24 MHz this chip is now constrained to), odd parity, DATA changing in the middle
// of CLOCK-high, 7200 clocks between frames. It types   A down, A up, B down, B up, C down :
//     1C | F0 1C | 32 | F0 32 | 21
// The first four frames arrive while the CPU is asleep: they must be sitting in the 4-deep RX FIFO,
// intact, when the CPU wakes up. The rest arrive while it polls. uo_out must show all 7 scancodes in
// order. (The old polled reader needs one ~3000-clock flash page per CLOCK edge and cannot follow 2400.)

module tb_pio_cpu_ps2;
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

    reg [7:0] image [0:967];
    integer ci;
    initial begin
        $readmemh("pio_ps2_rx_flash_image.hex", image);
        for (ci = 0; ci < 968; ci = ci + 1) u_flash.mem[ci] = image[ci];
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

    // ================= PS/2 keyboard model: 10 kHz at 24 MHz = 2400 clocks per bit =================
    localparam integer QUARTER = 600;           // CLOCK: high 1200 (DATA changes after 600), low 1200
    localparam integer GAP     = 7200;          // between frames (> the PIO's 4896-clock idle timeout)

    task automatic kb_bit(input b);
        begin
            ui_in[4] = b;                       // DATA settles while CLOCK is high (600 clocks before the fall)
            repeat (QUARTER) @(posedge clk);
            ui_in[3] = 1'b0;                    // CLOCK low: the receiver samples DATA here
            repeat (2 * QUARTER) @(posedge clk);
            ui_in[3] = 1'b1;
            repeat (QUARTER) @(posedge clk);    // 600 more of CLOCK high (hold) before the next DATA change
        end
    endtask
    task automatic kb_frame(input [7:0] d);
        integer k;
        begin
            kb_bit(1'b0);                                           // start
            for (k = 0; k < 8; k = k + 1) kb_bit(d[k]);             // data, LSB first
            kb_bit(~(^d));                                          // odd parity
            kb_bit(1'b1);                                           // stop
            ui_in[4] = 1'b1;
            repeat (GAP) @(posedge clk);
        end
    endtask

    // ---- what the CPU shows on uo_out: record every change once the receiver is running ----
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

    reg [7:0] exp [0:6];
    integer errors, i, cnt_at_wake;
    reg [31:0] w;
    reg [1:0]  rp;
    reg [10:0] f;

    initial begin
        exp[0] = 8'h1C; exp[1] = 8'hF0; exp[2] = 8'h1C; exp[3] = 8'h32;
        exp[4] = 8'hF0; exp[5] = 8'h32; exp[6] = 8'h21;
        $readmemh("flash_handoff_stub.hex", stub_bytes);
        errors = 0;
        ui_in = 8'h00; ui_in[3] = 1'b1; ui_in[4] = 1'b1;           // PS/2 lines idle high
        rst_n = 0; repeat (10) @(posedge clk); rst_n = 1;
        dut.u_mem.qspi_div_sel = 2'd0;
        repeat (3000) @(posedge clk);

        ui_in[2] = 1;
        repeat (20) @(posedge clk);
        boot_send_byte(STUB_LEN);
        for (ci = 0; ci < STUB_LEN; ci = ci + 1) boot_send_byte(stub_bytes[ci]);

        // the CPU enables the receiver, then sleeps: start typing right away
        wait (dut.u_pio.sm_en[0] === 1'b1);
        repeat (300) @(posedge clk);
        last_uo = uo_out; watch = 1;
        kb_frame(8'h1C);
        kb_frame(8'hF0);
        kb_frame(8'h1C);
        kb_frame(8'h32);
        // all four frames are in, the CPU is still asleep: check what the FIFO is holding
        cnt_at_wake = dut.u_pio.sm_gen[0].u_rxf.cnt;
        rp          = dut.u_pio.sm_gen[0].u_rxf.rp;
        if (dut.halted || nseen != 0) begin errors = errors + 1; $display("FAIL: the CPU was not asleep (uo_out changed %0d times)", nseen); end
        if (cnt_at_wake != 4) begin errors = errors + 1; $display("FAIL: RX FIFO holds %0d frames, expected 4", cnt_at_wake); end
        else begin
            for (i = 0; i < 4; i = i + 1) begin
                w = dut.u_pio.sm_gen[0].u_rxf.mem[(rp + i) & 3];
                f = w[31:21];
                if (f[0] !== 1'b0 || f[10] !== 1'b1 || (^f[9:1]) !== 1'b1 || f[8:1] !== exp[i]) begin
                    errors = errors + 1;
                    $display("FAIL: buffered frame %0d = %03x (start %b data %02x parity %b stop %b), expected data %02x",
                             i, f, f[0], f[8:1], f[9], f[10], exp[i]);
                end
            end
            if (errors == 0) $display("PASS: 4 frames sat intact in the RX FIFO (start/parity/stop valid) while the CPU slept, at a real 10 kHz");
        end
        // the CPU wakes, drains the four, and polls for the rest
        kb_frame(8'hF0);
        kb_frame(8'h32);
        kb_frame(8'h21);
        wait (dut.halted);
        repeat (2000) @(posedge clk);

        if (nseen != 7) begin errors = errors + 1; $display("FAIL: uo_out changed %0d times, expected 7", nseen); end
        for (i = 0; i < 7 && i < nseen; i = i + 1)
            if (seen[i] !== exp[i]) begin errors = errors + 1; $display("FAIL: scancode %0d = %02x, expected %02x", i, seen[i], exp[i]); end
        if (nseen == 7 && errors == 0)
            $display("PASS: the CPU showed 1C F0 1C 32 F0 32 21 on uo_out in order (A down/up, B down/up, C down)");

        if (errors == 0) $display("PASS tb_pio_cpu_ps2: PIO followed a real-rate PS/2 keyboard while the CPU slept; the CPU read every scancode");
        else             $display("FAIL tb_pio_cpu_ps2: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #1500000000;
        $display("TIMEOUT (nseen=%0d)", nseen);
        $finish;
    end
endmodule
