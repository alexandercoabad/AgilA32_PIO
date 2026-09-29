// tb_check.v -- covers all three scenarios of the bootloader boot ROM:
// self-test fails safely without a QSPI slave attached, self-test
// passes and the counter runs normally with one attached, and a
// program bootloaded over the DATA/CLOCK/START protocol actually runs
// from the loadable RAM window afterward.
//
// Uses the project's own validated spi_ram_model.v as the QSPI slave
// (same one test/tb_mem_ext.v uses) rather than a hand-rolled one --
// an earlier version of this file had its own embedded slave using
// `always @(posedge clk)` with a `prev_sck`/`sck` comparison to fake
// edge detection instead of true `posedge sck`/`negedge sck` triggers,
// which silently bit-shifted every sampled/driven byte. Same category
// of bug as a same-clock-domain testbench race -- comparing against a
// clock-derived signal one cycle late instead of triggering on the
// signal's own edge directly.

module tb_check;
    reg clk = 0;
    reg rst_n;
    reg [7:0] ui_in;
    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    always #5 clk = ~clk; // 100MHz-ish sim clock, exact freq irrelevant

    reg slave_enabled;
    wire qspi_miso_from_slave;

    // uio_in[2] is the only input pin this design ever reads (MISO);
    // every other uio bit is chip-driven (uio_oe=1 for them), so tying
    // the rest to 0 here is a don't-care, not a real signal.
    wire [7:0] uio_in = {5'b0, (slave_enabled ? qspi_miso_from_slave : 1'b0), 2'b0};

    tt_um_agila32 dut (
        .ui_in   (ui_in),
        .uo_out  (uo_out),
        .uio_in  (uio_in),
        .uio_out (uio_out),
        .uio_oe  (uio_oe),
        .ena     (1'b1),
        .clk     (clk),
        .rst_n   (rst_n)
    );

    wire cs1  = uio_out[6];
    wire sck  = uio_out[3];
    wire mosi = uio_out[1];

    spi_ram_model ram (
        .cs_n (cs1),
        .sck  (sck),
        .mosi (mosi),
        .miso (qspi_miso_from_slave)
    );

    task automatic reset_dut;
        begin
            ui_in = 0; rst_n = 0;
            repeat (10) @(posedge clk);
            rst_n = 1;
        end
    endtask

    // 200-cycle hold per phase: comfortably longer than the core's own
    // GPIO_IN poll loop (~3 instructions x 7 cycles/instr = 21 cycles),
    // so every level is guaranteed to be sampled at least once. Shorter
    // fixed hold times (tried 3, then 40 cycles) let per-bit processing
    // lag accumulate over a long transfer until the core fell behind
    // enough to miss pulses outright -- this protocol is a real
    // handshake, not a fixed baud rate, so erring generous here costs
    // nothing but simulation time.
    task automatic send_bit(input b);
        begin
            ui_in[0] = b; ui_in[1] = 0;
            repeat (200) @(posedge clk);
            ui_in[1] = 1;
            repeat (200) @(posedge clk);
            ui_in[1] = 0;
            repeat (200) @(posedge clk);
        end
    endtask

    task automatic send_byte(input [7:0] b);
        integer bi;
        begin
            for (bi = 7; bi >= 0; bi = bi - 1) send_bit(b[bi]);
        end
    endtask

    integer cyc, i;
    reg [7:0] last_uo;
    reg seen [0:15];

    initial begin
        $dumpfile("tb_check.vcd");
        $dumpvars(0, tb_check);

        // ---- Scenario 1: no slave attached -> self-test should fail,
        //      counter should still run (fallback path). ----
        slave_enabled = 0;
        reset_dut();
        // 15000 cycles: comfortably past the self-test's own SPI
        // transactions, which now run at QSPI_CTRL's reset default
        // (2'd3, slowest/sys_clk-128 -- see mem.v's header) rather
        // than the engine's old fixed always-fastest speed. Measured
        // ~10340 cycles for the self-test to resolve at that default.
        for (cyc = 0; cyc < 15000; cyc = cyc + 1) begin
            @(posedge clk);
            if (uo_out != 0) cyc = 100000; // break
        end
        if (uo_out[7] !== 1'b1)
            $display("FAIL scenario1: expected uo_out[7]=1 (selftest fail) w/o slave, got %b", uo_out);
        else
            $display("PASS scenario1: self-test correctly fails without a slave, uo_out=%b", uo_out);

        // ---- Scenario 2: slave attached -> self-test should pass. ----
        slave_enabled = 1;
        reset_dut();
        for (cyc = 0; cyc < 15000; cyc = cyc + 1) begin // see scenario1's comment
            @(posedge clk);
            if (uo_out != 0) cyc = 100000;
        end
        if (uo_out[7] !== 1'b0)
            $display("FAIL scenario2: expected uo_out[7]=0 (selftest pass) with slave, got %b", uo_out);
        else
            $display("PASS scenario2: self-test correctly passes with a slave, uo_out=%b", uo_out);

        // Confirm counter still wraps 0..15 after self-test passes.
        for (i = 0; i < 16; i = i + 1) seen[i] = 1'b0;
        last_uo = uo_out & 8'h0F;
        seen[last_uo] = 1'b1;
        for (cyc = 0; cyc < 56*20; cyc = cyc + 1) begin
            @(posedge clk);
            if ((uo_out & 8'h0F) !== last_uo) begin
                last_uo = uo_out & 8'h0F;
                seen[last_uo] = 1'b1;
            end
        end
        begin : chk
            integer allseen; allseen = 1;
            for (i = 0; i < 16; i = i + 1) if (!seen[i]) allseen = 0;
            if (allseen) $display("PASS scenario2b: counter visited all of 0..15");
            else $display("FAIL scenario2b: counter did not visit all of 0..15");
        end

        // ---- Scenario 3: bootload a tiny program via ui_in and confirm
        //      it actually runs from the external RAM window. ----
        // Program: addi x1,x0,5 ; addi x2,x0,7 ; add x3,x1,x2 ; sw x3,0xF0(x0) ; jal x0,0 (spin)
        slave_enabled = 1;
        reset_dut();
        // wait past self-test (see scenario1's comment on the budget),
        // then assert START and stream length+program
        repeat (15000) @(posedge clk);
        ui_in[2] = 1; // START
        repeat (20) @(posedge clk);
        send_byte(8'd20); // length: 5 instructions * 4 bytes = 20
        // addi x1,x0,5
        send_byte(8'h93); send_byte(8'h00); send_byte(8'h50); send_byte(8'h00);
        // addi x2,x0,7
        send_byte(8'h13); send_byte(8'h01); send_byte(8'h70); send_byte(8'h00);
        // add x3,x1,x2
        send_byte(8'hb3); send_byte(8'h81); send_byte(8'h20); send_byte(8'h00);
        // sw x3,0xF0(x0)  -- imm[11:5]=0x7 rs2=x3 rs1=x0 funct3=010 imm[4:0]=0x10 opcode=0x23
        send_byte(8'h23); send_byte(8'h28); send_byte(8'h30); send_byte(8'h0e);
        // jal x0,0 (tight spin so the store above is the last effect)
        send_byte(8'h6f); send_byte(8'h00); send_byte(8'h00); send_byte(8'h00);

        // give it time to load and execute a few instructions
        repeat (30000) @(posedge clk);

        if (uo_out[3:0] === 4'd12)
            $display("PASS scenario3: bootloaded program ran, uo_out[3:0]=12 (5+7) as expected");
        else
            $display("FAIL scenario3: expected uo_out[3:0]=12 after bootload, got uo_out=%b (0x%02x)", uo_out, uo_out);

        $finish;
    end

    initial begin
        #10000000;
        $display("TIMEOUT: simulation did not finish in time");
        $finish;
    end
endmodule
