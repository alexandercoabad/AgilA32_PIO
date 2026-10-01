`timescale 1ns/1ps

// tb_pio_cpu_usb.v -- END-TO-END low-speed USB: real RV32I core + real PIO block in the real top level.
//
// The image from tools/build_pio_usb.py makes the CPU load pio/usb_ls.pio, start SM0 and queue an
// IN token, then EBREAK. From there PIO alone: sends the token (NRZI + EOP), flips the same state
// machine to receive, and captures the device's DATA1 reply into the RX FIFO.
//
// The bench is a low-speed USB device on uio[4] (D+) / uio[5] (D-): D+ pulled down, D- pulled up
// (idle = J), 16 clocks per bit. It checks the token bit-for-bit, answers 66 clocks after the host's
// EOP began (24 clocks after it ended -- far less than one CPU flash-page switch), and the bench
// then compares the RX FIFO words with the expected capture and requires that the core had halted
// before the device started to reply.

module tb_pio_cpu_usb;
    reg clk = 0;
    reg rst_n;
    reg [7:0] ui_in;
    wire [7:0] uo_out, uio_out, uio_oe;
    always #5 clk = ~clk;

    wire cs0 = uio_out[0], cs1 = uio_out[6], sck = uio_out[3], mosi = uio_out[1];
    wire miso_flash, miso_psram;
    wire miso_bus = (!cs0) ? miso_flash : (!cs1) ? miso_psram : 1'b0;

    // ---- low-speed USB bus: D+ = uio[4], D- = uio[5] ----
    reg  dev_drv, dev_dp, dev_dm;
    wire dp = dev_drv ? dev_dp : (uio_oe[4] ? uio_out[4] : 1'b0);   // D+ pulled down
    wire dm = dev_drv ? dev_dm : (uio_oe[5] ? uio_out[5] : 1'b1);   // D- pulled up
    wire [7:0] uio_in = {2'b0, dm, dp, 1'b0, miso_bus, 2'b0};

    tt_um_agila32 dut (.ui_in(ui_in), .uo_out(uo_out), .uio_in(uio_in),
                       .uio_out(uio_out), .uio_oe(uio_oe), .ena(1'b1),
                       .clk(clk), .rst_n(rst_n));

    spi_ram_model u_flash (.cs_n(cs0), .sck(sck), .mosi(mosi), .miso(miso_flash));
    spi_ram_model u_psram (.cs_n(cs1), .sck(sck), .mosi(mosi), .miso(miso_psram));

    reg [7:0] image [0:835];
    integer ci;
    initial begin
        $readmemh("pio_usb_flash_image.hex", image);
        for (ci = 0; ci < 836; ci = ci + 1) u_flash.mem[ci] = image[ci];
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

    // ---- vectors from tools/build_pio_usb.py ----
    localparam integer NTOK = 32, NRESP = 101, NEXP = 4;
    reg tok_exp  [0:NTOK-1];
    reg resp_rom [0:NRESP-1];
    reg [31:0] exp_words [0:NEXP-1];
    initial begin
        $readmemb("pio_usb_token.mem", tok_exp);
        $readmemb("pio_usb_response.mem", resp_rom);
        $readmemh("pio_usb_expect.hex", exp_words);
    end

    // ================= behavioural device =================
    // host side: after K, sample every 16 clocks (first at +8); SE0 held >= 6 clocks = EOP.
    reg        rx_active;
    integer    rx_t, rx_n, se0_cnt, bit_errs;
    reg        prev_dp, prev_dm;                    // previous *sampled* level (J = 0/1)
    reg        got_bits [0:63];
    time       t_dev_start, t_halt;
    reg        halt_seen;
    integer    tx_t, tx_i, phase;                   // reply driver
    reg        lvl_k;                               // current driven level: 1 = K
    integer    resp_wait;
    reg        replied;

    initial begin
        dev_drv = 0; dev_dp = 0; dev_dm = 1; rx_active = 0; rx_t = 0; rx_n = 0; se0_cnt = 0;
        bit_errs = 0; prev_dp = 0; prev_dm = 1; t_dev_start = 0; t_halt = 0; halt_seen = 0;
        tx_t = 0; tx_i = 0; phase = 0; lvl_k = 0; resp_wait = -1; replied = 0;
    end

    always @(posedge clk) if (dut.halted && !halt_seen) begin halt_seen = 1; t_halt = $time; end

    always @(posedge clk) if (rst_n === 1'b1) begin
        // ---------- receive the host's token ----------
        if (!rx_active && !replied && dev_drv == 0 && dp === 1'b1 && dm === 1'b0) begin
            rx_active = 1; rx_t = 8; rx_n = 0; se0_cnt = 0; prev_dp = 0; prev_dm = 1;
        end else if (rx_active) begin
            se0_cnt = (dp === 1'b0 && dm === 1'b0) ? se0_cnt + 1 : 0;
            if (rx_t == 0) begin
                // a sample: bit = 1 if the level equals the previous level, else 0
                if (!(dp === 1'b0 && dm === 1'b0)) begin
                    if (rx_n < 64) got_bits[rx_n] = ((dp === prev_dp) && (dm === prev_dm));
                    rx_n = rx_n + 1;
                    prev_dp = dp; prev_dm = dm;
                end
                rx_t = 15;
            end else rx_t = rx_t - 1;
            if (se0_cnt == 6) begin              // EOP: schedule the reply
                rx_active = 0;
                resp_wait = (32 - 6) + 16 + 24;   // rest of SE0 + the J bit + turnaround
            end
        end
        // ---------- reply ----------
        if (resp_wait > 0) resp_wait = resp_wait - 1;
        else if (resp_wait == 0) begin
            resp_wait = -1; phase = 1; tx_i = 0; tx_t = 0; lvl_k = 0; dev_drv = 1;
            t_dev_start = $time; replied = 1;
        end
        if (phase == 1) begin                      // data bits, NRZI from J
            if (tx_t == 0) begin
                if (!resp_rom[tx_i]) lvl_k = ~lvl_k;
                dev_dp = lvl_k; dev_dm = ~lvl_k;
            end
            tx_t = tx_t + 1;
            if (tx_t == 16) begin
                tx_t = 0; tx_i = tx_i + 1;
                if (tx_i == NRESP) begin phase = 2; end
            end
        end else if (phase == 2) begin             // SE0 for 2 bit times
            dev_dp = 0; dev_dm = 0; tx_t = tx_t + 1;
            if (tx_t == 32) begin phase = 3; tx_t = 0; end
        end else if (phase == 3) begin             // J for 1 bit time, then release
            dev_dp = 0; dev_dm = 1; tx_t = tx_t + 1;
            if (tx_t == 16) begin phase = 0; dev_drv = 0; end
        end
    end

    integer errors, i;
    integer cnt;
    reg [31:0] w;
    reg [1:0]  rp;

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

        // wait for the device to finish its reply (phase back to 0 after replied)
        i = 0;
        while (!(replied && phase == 0) && i < 3000000) begin @(posedge clk); i = i + 1; end
        repeat (800) @(posedge clk);                // let the receiver see SE0 and push

        // ---- 1. the token the device saw
        if (rx_n != NTOK) begin errors = errors + 1; $display("FAIL: device saw %0d token bits, expected %0d", rx_n, NTOK); end
        else begin
            bit_errs = 0;
            for (i = 0; i < NTOK; i = i + 1) if (got_bits[i] !== tok_exp[i]) bit_errs = bit_errs + 1;
            if (bit_errs != 0) begin errors = errors + 1; $display("FAIL: token has %0d wrong bits", bit_errs); end
            else $display("PASS: device received the IN token bit-for-bit (%0d stuffed bits, NRZI + EOP)", NTOK);
        end

        // ---- 2. the reply captured in the RX FIFO
        cnt = dut.u_pio.sm_gen[0].u_rxf.cnt;
        rp  = dut.u_pio.sm_gen[0].u_rxf.rp;
        if (cnt < NEXP || cnt > NEXP + 1) begin errors = errors + 1; $display("FAIL: RX FIFO holds %0d words, expected %0d", cnt, NEXP); end
        for (i = 0; i < NEXP && i < cnt; i = i + 1) begin
            w = dut.u_pio.sm_gen[0].u_rxf.mem[(rp + i) & 3];
            if (w !== exp_words[i]) begin errors = errors + 1; $display("FAIL: RX word %0d = %08x, expected %08x", i, w, exp_words[i]); end
        end
        if (errors == 0) $display("PASS: PIO captured the DATA1 reply (%0d RX words match, 8-byte payload incl. stuffed bits)", NEXP);

        // ---- 3. the CPU was already parked when the device answered
        if (!halt_seen || !(t_halt < t_dev_start)) begin
            errors = errors + 1;
            $display("FAIL: core had not halted before the reply started (halt %0t, reply %0t)", t_halt, t_dev_start);
        end else
            $display("PASS: core halted (EBREAK) before the device replied; PIO turned the bus around alone");

        if (errors == 0) $display("PASS tb_pio_cpu_usb: CPU queued an IN token and halted; PIO did the whole low-speed USB transaction");
        else             $display("FAIL tb_pio_cpu_usb: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #900000000;
        $display("TIMEOUT (replied=%0d phase=%0d rx_n=%0d)", replied, phase, rx_n);
        $finish;
    end
endmodule
