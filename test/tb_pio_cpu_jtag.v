`timescale 1ns/1ps

// tb_pio_cpu_jtag.v -- the CPU drives a JTAG target through the PIO, walking the TAP itself.
//
// Firmware (tools/build_pio_jtag.py): TAP reset -> read IDCODE (32 clocks) -> load IR = USER (4) ->
// write the 16-bit USER register with the low half of the IDCODE the CPU just read (a relay: TX FIFO
// data comes from a CPU register) -> read USER back (16) -> show the low byte on GPIO_OUT -> EBREAK.
//
//   TDI = uo_out[0]   TMS = uo_out[1]   TCK = uo_out[2]   TDO = ui_in[3]
//
// The IEEE 1149.1 TAP model below watches the pins only.  It checks:
//   * the state visited on every TCK rising edge, and that each shift scan has the right length
//     (DR 32, IR 4, DR 16, DR 16) and the target ends in Run-Test/Idle
//   * exactly 95 TCK pulses in total; TCK idles low
//   * TDI/TMS never change while TCK is high; >= 30 ns setup before and >= 30 ns hold after every
//     rising edge
//   * the USER register captured 0 the first time and IDCODE[15:0] the second time, and ends holding
//     IDCODE[15:0]  (the CPU relayed the data)
//   * the byte the CPU shows on GPIO_OUT is IDCODE[7:0]  (target -> CPU -> target -> CPU round trip)
//   * the core halts (EBREAK) only after the last TCK pulse

module tb_pio_cpu_jtag;
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
        $readmemh("pio_jtag_flash_image.hex", image);
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

    // ------------------------------------------------------------------ JTAG target (pins only)
    localparam [31:0] IDCODE = 32'hA32C0D5B;
    localparam [3:0] TLR=0, RTI=1, SELDR=2, CAPDR=3, SHDR=4, EX1DR=5, PADR=6, EX2DR=7, UPDR=8,
                     SELIR=9, CAPIR=10, SHIR=11, EX1IR=12, PAIR=13, EX2IR=14, UPIR=15;

    wire tdi = uo_out[0], tms = uo_out[1], tck = uo_out[2];
    reg  tdo;
    always @* ui_in[3] = tdo;

    reg        jtag_on;
    reg [3:0]  st, ir, ir_sh, nxt;
    reg [31:0] dr_sh;
    reg [5:0]  dr_len;
    reg [15:0] user;
    integer    rises, scan_n, nscan;
    integer    scan_len [0:7];
    reg        scan_isir [0:7];
    reg [15:0] user_cap [0:3];
    integer    ncap;
    integer    setup_fail, hold_fail, hi_change_fail;
    time       t_change, t_rise;
    reg [3:0]  ir_at_update [0:7];
    integer    nir;

    initial begin
        jtag_on = 0; st = TLR; ir = 4'h1; ir_sh = 0; dr_sh = 0; dr_len = 32; user = 16'h0; tdo = 1;
        rises = 0; scan_n = 0; nscan = 0; ncap = 0; nir = 0;
        setup_fail = 0; hold_fail = 0; hi_change_fail = 0; t_change = 0; t_rise = 0;
    end

    // the firmware owns ui_in[3] as TDO once the PIO is enabled (ui_in[2] is the boot START pin)
    always @(posedge clk) if (!jtag_on && dut.u_pio.sm_en[0] === 1'b1) jtag_on = 1;

    always @(tdi or tms) if (jtag_on) begin
        if (tck) hi_change_fail = hi_change_fail + 1;
        if (rises > 0 && ($time - t_rise) < 30) hold_fail = hold_fail + 1;
        t_change = $time;
    end

    always @(posedge tck) if (jtag_on) begin
        if (($time - t_change) < 30) setup_fail = setup_fail + 1;
        t_rise = $time;
        rises = rises + 1;
        case (st)
            CAPDR: begin
                scan_n = 0;
                case (ir)
                    4'h1: begin dr_len = 32; dr_sh = IDCODE; end
                    4'h2: begin dr_len = 16; dr_sh = {16'h0, user};
                                if (ncap < 4) begin user_cap[ncap] = user; ncap = ncap + 1; end end
                    default: begin dr_len = 1; dr_sh = 32'h0; end
                endcase
            end
            default: ;
        endcase
        // Shift-DR: shift right by one, the new bit enters at position dr_len-1
        if (st == SHDR) begin
            dr_sh = (dr_sh >> 1);
            dr_sh[dr_len - 1] = tdi;
            scan_n = scan_n + 1;
        end
        if (st == CAPIR) begin ir_sh = 4'b0001; scan_n = 0; end
        if (st == SHIR) begin ir_sh = {tdi, ir_sh[3:1]}; scan_n = scan_n + 1; end
        if (st == UPDR && ir == 4'h2) user = dr_sh[15:0];
        if (st == UPIR) begin
            ir = ir_sh;
            if (nir < 8) begin ir_at_update[nir] = ir_sh; nir = nir + 1; end
        end
        case (st)
            TLR:   nxt = tms ? TLR   : RTI;
            RTI:   nxt = tms ? SELDR : RTI;
            SELDR: nxt = tms ? SELIR : CAPDR;
            CAPDR: nxt = tms ? EX1DR : SHDR;
            SHDR:  nxt = tms ? EX1DR : SHDR;
            EX1DR: nxt = tms ? UPDR  : PADR;
            PADR:  nxt = tms ? EX2DR : PADR;
            EX2DR: nxt = tms ? UPDR  : SHDR;
            UPDR:  nxt = tms ? SELDR : RTI;
            SELIR: nxt = tms ? TLR   : CAPIR;
            CAPIR: nxt = tms ? EX1IR : SHIR;
            SHIR:  nxt = tms ? EX1IR : SHIR;
            EX1IR: nxt = tms ? UPIR  : PAIR;
            PAIR:  nxt = tms ? EX2IR : PAIR;
            EX2IR: nxt = tms ? UPIR  : SHIR;
            default: nxt = tms ? SELDR : RTI;                              // UPIR
        endcase
        if ((st == SHDR || st == SHIR) && !(nxt == SHDR || nxt == SHIR)) begin
            if (nscan < 8) begin scan_len[nscan] = scan_n; scan_isir[nscan] = (st == SHIR); nscan = nscan + 1; end
        end
        if (nxt == TLR) ir = 4'h1;
        st = nxt;
    end

    always @(negedge tck) if (jtag_on)
        tdo <= (st == SHDR) ? dr_sh[0] : (st == SHIR) ? ir_sh[0] : 1'b1;

    // ------------------------------------------------------------------ halt tracking
    reg halt_seen; time t_halt;
    initial begin halt_seen = 0; t_halt = 0; end
    always @(posedge clk) if (dut.halted && !halt_seen) begin halt_seen = 1; t_halt = $time; end

    // ------------------------------------------------------------------ main
    integer errors, slow;
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
        while (!halt_seen && slow < 80000000) begin @(posedge clk); slow = slow + 1; end
        repeat (400) @(posedge clk);

        if (!halt_seen) begin errors = errors + 1; $display("FAIL: core never halted (TCK rises=%0d state=%0d)", rises, st); end

        // ---- clocks, scans, final state
        if (rises != 95) begin errors = errors + 1; $display("FAIL: %0d TCK pulses, expected 95", rises); end
        else $display("PASS: exactly 95 TCK pulses");
        if (nscan != 4 || scan_len[0] != 32 || scan_isir[0] || scan_len[1] != 4 || !scan_isir[1]
            || scan_len[2] != 16 || scan_isir[2] || scan_len[3] != 16 || scan_isir[3]) begin
            errors = errors + 1;
            $display("FAIL: scans = %0d [%0d%s %0d%s %0d%s %0d%s], expected 4 [DR32 IR4 DR16 DR16]", nscan,
                     scan_len[0], scan_isir[0] ? "IR" : "DR", scan_len[1], scan_isir[1] ? "IR" : "DR",
                     scan_len[2], scan_isir[2] ? "IR" : "DR", scan_len[3], scan_isir[3] ? "IR" : "DR");
        end else $display("PASS: scans DR32 (IDCODE), IR4, DR16 (write USER), DR16 (read USER)");
        if (nir != 1 || ir_at_update[0] !== 4'h2) begin
            errors = errors + 1; $display("FAIL: IR updates = %0d, first value 0x%0h, expected exactly one, USER (2)", nir, ir_at_update[0]);
        end else $display("PASS: instruction register loaded once, with USER (0x2)");
        if (st !== RTI) begin errors = errors + 1; $display("FAIL: target ended in state %0d, expected Run-Test/Idle", st); end
        else $display("PASS: TAP ended in Run-Test/Idle");
        if (tck !== 1'b0) begin errors = errors + 1; $display("FAIL: TCK not low when idle"); end

        // ---- wire timing
        if (setup_fail != 0 || hold_fail != 0 || hi_change_fail != 0) begin
            errors = errors + 1;
            $display("FAIL: setup violations=%0d hold violations=%0d changes-while-TCK-high=%0d", setup_fail, hold_fail, hi_change_fail);
        end else $display("PASS: TDI/TMS changed only while TCK low, >= 30 ns setup and hold on every clock");

        // ---- the relay
        if (ncap != 2 || user_cap[0] !== 16'h0000 || user_cap[1] !== IDCODE[15:0]) begin
            errors = errors + 1;
            $display("FAIL: USER captured %04x then %04x (ncap=%0d), expected 0000 then %04x", user_cap[0], user_cap[1], ncap, IDCODE[15:0]);
        end else $display("PASS: USER register was 0000, then held 0x%04x - the IDCODE bits the CPU relayed", IDCODE[15:0]);
        if (user !== IDCODE[15:0]) begin errors = errors + 1; $display("FAIL: USER ends 0x%04x, expected 0x%04x", user, IDCODE[15:0]); end

        // ---- the byte shown on the pads made the whole round trip
        if (dut.u_mem.gpio_out !== IDCODE[7:0]) begin
            errors = errors + 1; $display("FAIL: GPIO_OUT = 0x%02x, expected IDCODE[7:0] = 0x%02x", dut.u_mem.gpio_out, IDCODE[7:0]);
        end else $display("PASS: GPIO_OUT = 0x%02x = IDCODE[7:0], after target -> CPU -> target -> CPU", dut.u_mem.gpio_out);

        if (halt_seen && t_halt > t_rise) $display("PASS: core halted (EBREAK) after the last TCK pulse");
        else begin errors = errors + 1; $display("FAIL: halt ordering (halt=%0t last TCK rise=%0t)", t_halt, t_rise); end

        if (errors == 0) $display("PASS tb_pio_cpu_jtag: CPU drove the TAP through the PIO: IDCODE read, USER written and read back");
        else             $display("FAIL tb_pio_cpu_jtag: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #2000000000;
        $display("TIMEOUT (TCK rises=%0d state=%0d halted=%0d)", rises, st, halt_seen);
        $finish;
    end
endmodule
