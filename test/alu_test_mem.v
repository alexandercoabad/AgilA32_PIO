// alu_test_mem.v -- minimal flat ROM+RAM harness used only by
// tb_alu_test.v, to verify asm_pineapple.py's instruction encodings
// against the real rv32i_core without any of mem.v's external-QSPI or
// FLASH_MODE machinery (irrelevant to what this test checks). Not
// mem.v, not used anywhere else.
//
// Address map (this test module only):
//   0x00-0xBF : ROM (tools/build_alu_test.py's program, via $readmemh)
//   0xC0-0xDF : RAM (scratch for the SH/LH/LHU/SB/LB/LBU checks)

`default_nettype none

module alu_test_mem (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  addr,
    input  wire [31:0] wdata,
    input  wire [1:0]  size,      // 0=byte, 1=half, 2=word
    input  wire        we,
    input  wire        valid,
    output wire         ready,
    output reg  [31:0] rdata
);

    localparam ROM_BYTES = 192;
    localparam RAM_BASE  = 8'hC0;
    localparam RAM_BYTES = 32;

    assign ready = 1'b1; // everything here is on-chip, single-cycle

    reg [7:0] rom [0:ROM_BYTES-1];
    initial $readmemh("alu_test.hex", rom);

    reg [7:0] ram [0:RAM_BYTES-1];
    integer i;
    // synthesis translate_off
    initial for (i = 0; i < RAM_BYTES; i = i + 1) ram[i] = 8'h00;
    // synthesis translate_on

    wire in_rom = (addr < ROM_BYTES);
    wire [7:0] ram_addr = addr - RAM_BASE;

    always @(*) begin
        if (in_rom) begin
            rdata = {rom[addr+8'd3], rom[addr+8'd2], rom[addr+8'd1], rom[addr]};
        end else begin
            rdata = {ram[ram_addr+8'd3], ram[ram_addr+8'd2], ram[ram_addr+8'd1], ram[ram_addr]};
        end
    end

    always @(posedge clk) begin
        if (we && !in_rom) begin
            case (size)
                2'd0: ram[ram_addr] <= wdata[7:0];
                2'd1: begin
                    ram[ram_addr]       <= wdata[7:0];
                    ram[ram_addr+8'd1] <= wdata[15:8];
                end
                default: begin
                    ram[ram_addr]       <= wdata[7:0];
                    ram[ram_addr+8'd1] <= wdata[15:8];
                    ram[ram_addr+8'd2] <= wdata[23:16];
                    ram[ram_addr+8'd3] <= wdata[31:24];
                end
            endcase
        end
    end

endmodule
