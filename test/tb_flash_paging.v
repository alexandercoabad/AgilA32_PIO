`timescale 1ns/1ps

// tb_flash_paging.v -- end-to-end check of mem.v's FLASH_PAGE
// bank-switching mechanism (see tools/asm_pineapple.py's PagedAsm and
// mem.v's "Bank-switched flash execution"): bootload the 1-instruction
// flash-handoff stub, let it redirect to a flash image built from
// FOUR pages (tools/build_flash_pagetest.py), and confirm GPIO_OUT
// visits every page's distinct value IN ORDER -- 0x11, then 0x22
// TWICE (a switch_to_computed() self-loop), then 0x33, then settles
// at 0x44 -- not just that it eventually reaches the last one.
//
// Scaffolding lifted from tb_flash_handoff.v (same boot sequence, same
// dual CS0/CS1 slave-mux setup); see that file for why PSRAM (CS1) has
// to be attached even though this test doesn't care about it.

module tb_flash_paging;
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

    // Preload the flash model with the 4-page (176-byte) test image --
    // direct array preload, standing in for "flash programmed by some
    // means outside this bootloader", same as tb_flash_handoff.v does.
    reg [7:0] pagetest [0:175];
    integer ci;
    initial begin
        $readmemh("flash_pagetest.hex", pagetest);
        for (ci = 0; ci < 176; ci = ci + 1) u_flash.mem[ci] = pagetest[ci];
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

    reg [7:0] stub_bytes [0:3];
    localparam integer STUB_LEN = 4;

    // Count actual GPIO_OUT (0xF0) stores, keyed by the value written,
    // via mem.v's own write bus -- distinct-value tracking on uo_out
    // itself can't tell "wrote 0x22 twice in a row" from "wrote it
    // once", which is exactly the case the self-loop needs to prove.
    reg [7:0] seq [0:15];
    integer   seq_len;
    reg       recording;

    always @(posedge clk) begin
        if (!rst_n) begin
            seq_len   <= 0;
            recording <= 1'b0;
        end else if (recording && dut.u_mem.we && dut.u_mem.addr == 8'hF0) begin
            if (seq_len < 16) seq[seq_len] <= dut.u_mem.wdata[7:0];
            seq_len <= seq_len + 1;
        end
    end

    integer k;
    reg pass;

    initial begin
        $dumpfile("tb_flash_paging.vcd");
        $dumpvars(0, tb_flash_paging);

        $readmemh("flash_handoff_stub.hex", stub_bytes);

        reset_dut();
        repeat (3000) @(posedge clk); // past self-test + a few demo-loop iterations

        ui_in[2] = 1; // START
        // Give MAIN_LOOP a full iteration's worth of margin to notice
        // START and jump away to START_SEEN *before* turning on write
        // recording. MAIN_LOOP's body got two instructions longer once
        // the boot-timeout fallback (feature #2) landed (SRLI+BEQ after
        // every LED_OUT store), so the old 20-cycle gap here -- enough
        // margin against the pre-timeout loop body, timing-wise -- was
        // no longer enough to guarantee the in-flight iteration's own
        // LED_OUT store (queued before START was polled) always retires
        // ahead of this window: recording could flip on while that
        // stale write was still in flight, prepending a spurious extra
        // entry (the demo counter's own value) to the sequence this test
        // checks byte-for-byte. 100 cycles clears a full MAIN_LOOP
        // iteration (~63-70 cycles, measured) with margin, so recording
        // never turns on until the core has actually left MAIN_LOOP.
        repeat (100) @(posedge clk);
        recording = 1'b1;
        send_byte(STUB_LEN);
        for (ci = 0; ci < STUB_LEN; ci = ci + 1) send_byte(stub_bytes[ci]);

        // Give the stub time to run, then let the 4-page program walk
        // through all its page switches. Each instruction fetched from
        // flash is a full SPI transaction (~130 cycles), and this
        // program crosses 4 page boundaries (5 flash-resident
        // instructions/page x ~4-9 instructions actually executed per
        // page, worst case ~9 instructions x4 pages), so budget
        // generously.
        repeat (25000) @(posedge clk);

        pass = (seq_len == 5) && (seq[0] === 8'h11) && (seq[1] === 8'h22) &&
               (seq[2] === 8'h22) && (seq[3] === 8'h33) && (seq[4] === 8'h44) &&
               (uo_out === 8'h44);

        $write("GPIO_OUT writes observed:");
        for (k = 0; k < seq_len && k < 16; k = k + 1) $write(" 0x%02x", seq[k]);
        $display("");

        if (pass)
            $display("PASS tb_flash_paging: bank-switching sequence 0x11 -> 0x22 -> 0x22 -> 0x33 -> 0x44 confirmed");
        else
            $display("FAIL tb_flash_paging: expected sequence 0x11,0x22,0x22,0x33,0x44 (5 steps), got %0d steps, final uo_out=0x%02x", seq_len, uo_out);

        $finish;
    end

    initial begin
        #1000000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
