`timescale 1ns/1ps
// tb_pilipinas_flash.v -- power-up test: nobody bootloads, flash byte 0 holds
// the shift-12 PILIPINAS AGILA32 image.  Boot ROM times out, enters flash mode
// and the 7-segment program runs by itself.  No qspi_div_sel override: real
// reset timing (sys_clk/128) is used.
module tb_pilipinas_flash;
    reg clk = 0; reg rst_n; reg [7:0] ui_in;
    wire [7:0] uo_out, uio_out, uio_oe;
    always #5 clk = ~clk;
    wire cs0 = uio_out[0], cs1 = uio_out[6], sck = uio_out[3], mosi = uio_out[1];
    wire miso_flash, miso_psram;
    wire miso_bus = (!cs0) ? miso_flash : (!cs1) ? miso_psram : 1'b0;
    wire [7:0] uio_in = {5'b0, miso_bus, 2'b0};
    tt_um_agila32 dut(.ui_in(ui_in),.uo_out(uo_out),.uio_in(uio_in),.uio_out(uio_out),
        .uio_oe(uio_oe),.ena(1'b1),.clk(clk),.rst_n(rst_n));
    spi_ram_model u_flash(.cs_n(cs0),.sck(sck),.mosi(mosi),.miso(miso_flash));
    spi_ram_model u_psram(.cs_n(cs1),.sck(sck),.mosi(mosi),.miso(miso_psram));

    reg [7:0] img [0:43]; integer i;
    initial begin
        $readmemh("pilipinas_flash_shift12.hex", img);
        for (i = 0; i < 44; i = i + 1) u_flash.mem[i] = img[i];
    end
    reg [7:0] exp [0:15];
    initial begin
        exp[0]=8'h73; exp[1]=8'hB0; exp[2]=8'h38; exp[3]=8'hB0; exp[4]=8'h73; exp[5]=8'hB0;
        exp[6]=8'h37; exp[7]=8'hF7; exp[8]=8'h6D; exp[9]=8'hF7; exp[10]=8'h3D; exp[11]=8'hB0;
        exp[12]=8'h38; exp[13]=8'hF7; exp[14]=8'h4F; exp[15]=8'hDB;
    end
    reg [7:0] last; integer n, bad; time t_first;
    initial begin
        ui_in = 0; rst_n = 0; last = 8'hxx; n = 0; bad = 0;
        repeat (10) @(posedge clk); rst_n = 1;
    end
    always @(posedge clk) if (rst_n && dut.u_mem.flash_mode === 1'b1 && uo_out !== last && ^uo_out !== 1'bx) begin
        last = uo_out;
        if (uo_out !== 8'h00 && n < 16) begin
            if (n == 0) t_first = $time;
            $display("t=%0t ps (%0d cycles)  char %0d  uo_out=0x%02x  expect 0x%02x  %s", $time, $time/10, n, uo_out, exp[n],
                     (uo_out === exp[n]) ? "ok" : "MISMATCH");
            if (uo_out !== exp[n]) bad = bad + 1;
            n = n + 1;
            if (n == 16) begin
                if (bad == 0) $display("PASS tb_pilipinas_flash: all 16 characters PILIPINASAGILA32 shown from flash at power-up");
                else $display("FAIL tb_pilipinas_flash: %0d mismatches", bad);
            end
        end
    end
    initial begin
        #700_000_000;
        if (n < 16) $display("FAIL tb_pilipinas_flash: only %0d characters seen", n);
        $finish;
    end
    always @(n) if (n == 16) begin #1_000_000; $finish; end
endmodule
