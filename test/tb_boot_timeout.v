`timescale 1ns/1ps

// tb_boot_timeout.v -- end-to-end check of the boot ROM's timeout ->
// automatic flash fallback (feature #2 in the AgilA32<-AgilA8 porting
// roadmap): confirms an UNATTENDED chip (no host ever asserts START)
// eventually gives up on MAIN_LOOP, sets FLASH_MODE itself, and starts
// executing from external flash -- and confirms a host that DOES
// respond before the timeout elapses still gets a normal bootload,
// unaffected by the new fallback path.
//
// Scaffolding lifted from tb_flash_handoff.v (same dual CS0/CS1
// slave-mux setup, same flash_canary.hex image -- see that file for
// why PSRAM (CS1) has to be attached even when a test doesn't care
// about it) and from tb_check.v's scenario 3 (same tiny RAM bootload
// program, same host-response timing).
//
// Unlike tb_flash_handoff.v's stub-bootload path, no bootload happens
// at all in Part 1 here -- the boot ROM itself writes FLASH_MODE and
// jumps to RAM_BASE (== LOAD_BASE) directly from ROM addresses, which
// (per mem.v's FLASH_MODE comment and build_boot_rom.py's own note on
// why this is safe from boot ROM) lands cleanly on flash byte 0, not
// byte 4 the way the handoff-stub path does. flash_canary.hex's byte-0
// NOP just executes harmlessly here instead of staying dead.

module tb_boot_timeout;
    reg clk = 0;
    reg rst_n;
    reg [7:0] ui_in;
    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    always #5 clk = ~clk;

    wire cs0  = uio_out[0]; // flash
    wire cs1  = uio_out[6]; // psram
    wire sck  = uio_out[3];
    wire mosi = uio_out[1];
    wire miso_flash, miso_psram;

    wire miso_bus = (!cs0) ? miso_flash : (!cs1) ? miso_psram : 1'b0;
    wire [7:0] uio_in = {5'b0, miso_bus, 2'b0};

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

    spi_ram_model u_flash (
        .cs_n (cs0),
        .sck  (sck),
        .mosi (mosi),
        .miso (miso_flash)
    );

    spi_ram_model u_psram (
        .cs_n (cs1),
        .sck  (sck),
        .mosi (mosi),
        .miso (miso_psram)
    );

    // Same tiny canary program tb_flash_handoff.v uses (see
    // tools/build_flash_canary.py): byte 0 is a NOP, harmless whether
    // it's dead (handoff-stub path) or actually fetched (this path).
    reg [7:0] canary [0:15];
    integer ci;
    initial begin
        $readmemh("flash_canary.hex", canary);
        for (ci = 0; ci < 16; ci = ci + 1) u_flash.mem[ci] = canary[ci];
    end

    task automatic reset_dut;
        begin
            ui_in = 0; rst_n = 0;
            repeat (10) @(posedge clk);
            rst_n = 1;
            // QSPI_CTRL resets to the slow default (2'd3) on every rst_n
            // deassertion; this testbench's fixed cycle budgets predate
            // that register and assume the engine's original fixed-fast
            // timing, so deposit req_div_sel=0 right after each reset --
            // standing in for "hardware whose real SPI timing has already
            // been confirmed safe" (see CHANGES_feature3.md).
            dut.u_mem.qspi_div_sel = 2'd0;
        end
    endtask

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

    integer cyc;
    reg saw_early_cs0;
    reg pass1, pass2, pass3;

    initial begin
        $dumpfile("tb_boot_timeout.vcd");
        $dumpvars(0, tb_boot_timeout);

        // ---- Part 1: nobody bootloads. START stays low forever, so
        //      MAIN_LOOP should eventually give up on its own, set
        //      FLASH_MODE, and fall into the canary at flash byte 0.
        //      Measured (see build_boot_rom.py's TIMEOUT_SHIFT comment
        //      and this test's own dev notes): flash_mode goes high
        //      around cycle ~32467 with TIMEOUT_SHIFT=9, canary result
        //      visible on uo_out by ~32900. 60000 cycles is comfortable
        //      margin without waiting so long the suite gets slow. ----
        ui_in = 0;
        saw_early_cs0 = 1'b0;
        reset_dut();
        for (cyc = 0; cyc < 20000; cyc = cyc + 1) begin
            @(posedge clk);
            // CS0 (flash) must stay deasserted for a good long while --
            // MAIN_LOOP has no business touching flash before it's
            // actually given up. 20000 cycles is well under the
            // measured ~32467-cycle fallback point, so seeing cs0 low
            // (asserted) any time in this window would mean the
            // timeout fired far too early.
            if (!cs0) saw_early_cs0 = 1'b1;
        end
        repeat (40000) @(posedge clk); // clear the ~32467-cycle fallback point with margin

        pass1 = !saw_early_cs0;
        if (pass1)
            $display("PASS boot_timeout part1a: flash (CS0) stays untouched through the first 20000 cycles of MAIN_LOOP");
        else
            $display("FAIL boot_timeout part1a: flash (CS0) was asserted well before the timeout should have fired");

        pass2 = (dut.u_mem.flash_mode === 1'b1);
        if (pass2)
            $display("PASS boot_timeout part1b: FLASH_MODE latched by the boot ROM itself after the timeout, no bootload involved");
        else
            $display("FAIL boot_timeout part1b: expected flash_mode=1 after timeout, got %b", dut.u_mem.flash_mode);

        pass3 = (uo_out === 8'h2A);
        if (pass3)
            $display("PASS boot_timeout part1c: uo_out=0x%02x -- unattended chip fell through to flash and ran the canary", uo_out);
        else
            $display("FAIL boot_timeout part1c: expected uo_out=0x2a (canary ran from flash), got uo_out=0x%02x", uo_out);

        // ---- Part 2: a host DOES respond, well before the ~32467-
        //      cycle timeout -- confirm the fallback logic doesn't
        //      preempt or otherwise disturb a normal bootload. Same
        //      5-instruction program and timing as tb_check.v's
        //      scenario 3 (addi x1,5; addi x2,7; add x3,x1,x2;
        //      sw x3,0xF0(x0); jal x0,0). ----
        reset_dut();
        repeat (200) @(posedge clk);
        ui_in[2] = 1; // START, far ahead of the timeout
        repeat (20) @(posedge clk);
        send_byte(8'd20); // length: 5 instructions * 4 bytes
        send_byte(8'h93); send_byte(8'h00); send_byte(8'h50); send_byte(8'h00); // addi x1,x0,5
        send_byte(8'h13); send_byte(8'h01); send_byte(8'h70); send_byte(8'h00); // addi x2,x0,7
        send_byte(8'hb3); send_byte(8'h81); send_byte(8'h20); send_byte(8'h00); // add x3,x1,x2
        send_byte(8'h23); send_byte(8'h28); send_byte(8'h30); send_byte(8'h0e); // sw x3,0xF0(x0)
        send_byte(8'h6f); send_byte(8'h00); send_byte(8'h00); send_byte(8'h00); // jal x0,0 (spin)

        repeat (30000) @(posedge clk);

        if ((uo_out[3:0] === 4'd12) && (dut.u_mem.flash_mode === 1'b0))
            $display("PASS boot_timeout part2: a prompt host still gets a normal RAM bootload (uo_out[3:0]=12), flash_mode never set");
        else
            $display("FAIL boot_timeout part2: expected uo_out[3:0]=12 and flash_mode=0, got uo_out=0x%02x flash_mode=%b", uo_out, dut.u_mem.flash_mode);

        $finish;
    end

    initial begin
        // Part 1 alone burns ~600,000ns (60000 cycles at 10ns/cycle)
        // waiting out the boot-ROM timeout; Part 2's 20-byte bootload
        // over the 3-phase/200-cycle-per-phase DATA/CLOCK handshake
        // adds close to another 1,000,000ns on top of that. 2,500,000ns
        // clears both parts with comfortable margin.
        #2500000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
