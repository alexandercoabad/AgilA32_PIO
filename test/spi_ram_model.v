// spi_ram_model.v -- crude behavioral single-line SPI SRAM model, for
// testing qspi_shared_engine / mem.v against something that actually
// behaves like a chip (auto-incrementing address, real CMD/ADDR/DATA
// phases) instead of a plain logic-analyzer style bit checker.
//
// NOT a substitute for testing against a real vendor behavioral model
// or real hardware -- this only implements exactly the subset of the
// 0x03/0x02 protocol this project's engine actually uses.

`default_nettype none
`timescale 1ns/1ps

module spi_ram_model (
    input  wire cs_n,
    input  wire sck,
    input  wire mosi,
    output reg  miso
);

    // 14-bit addressing (16384 bytes) -- comfortably covers real flash
    // images built through PagedAsm (e.g. the PS/2 ASCII translation
    // table's ~12.3KB image, or the ST7789 driver's ~7.3KB simulation-
    // sized image), not just the small hand-written test programs the
    // original 256-byte model was sized for. Only the low bits of
    // `addr` actually get indexed either way (real flash chips are
    // much bigger than either size), so widening this is a pure
    // capacity change, not a protocol change.
    reg [7:0] mem [0:16383];
    integer i;
    initial for (i = 0; i < 16384; i = i + 1) mem[i] = 8'h00;

    reg [2:0]  bitcnt;
    reg [1:0]  phase;      // 0=cmd, 1=addr, 2=data
    reg [1:0]  addr_byte;  // which of the 3 address bytes we're on
    reg [23:0] addr;
    reg        we;
    reg [7:0]  shift_in;
    reg [7:0]  cur_out;

    // Combinational view of what `addr` is ABOUT to become once this
    // cycle's nonblocking assign to it lands -- needed so the very
    // same cycle that finishes shifting in the 3rd address byte can
    // already index `mem[]` with the complete address instead of a
    // one-byte-stale one (the actual `addr <=` update is nonblocking
    // and only visible starting next cycle).
    wire [23:0] next_addr = {addr[15:0], shift_in[6:0], mosi};

    localparam PHASE_CMD  = 2'd0;
    localparam PHASE_ADDR = 2'd1;
    localparam PHASE_DATA = 2'd2;

    // Sample MOSI on the rising edge (SPI mode 0), same edge the real
    // master's own sampling happens on -- both sides read the line at
    // the same instant, which is fine since MOSI/MISO are separate wires.
    always @(posedge sck or posedge cs_n) begin
        if (cs_n) begin
            phase     <= PHASE_CMD;
            bitcnt    <= 3'd0;
            addr_byte <= 2'd0;
        end else begin
            shift_in <= {shift_in[6:0], mosi};
            if (bitcnt == 3'd7) begin
                bitcnt <= 3'd0;
                case (phase)
                    PHASE_CMD: begin
                        we    <= ({shift_in[6:0], mosi} == 8'h02);
                        phase <= PHASE_ADDR;
                    end
                    PHASE_ADDR: begin
                        addr <= next_addr;
                        if (addr_byte == 2'd2) begin
                            phase   <= PHASE_DATA;
                            cur_out <= mem[next_addr[13:0]]; // low 14 bits of the address just formed
                        end
                        addr_byte <= addr_byte + 2'd1;
                    end
                    default: begin // PHASE_DATA
                        if (we) mem[addr[13:0]] <= {shift_in[6:0], mosi};
                        addr    <= addr + 24'd1;
                        cur_out <= mem[addr[13:0] + 14'd1];
                    end
                endcase
            end else begin
                bitcnt <= bitcnt + 3'd1;
            end
        end
    end

    // Drive MISO on the falling edge, ahead of the next rising/sample
    // edge -- mirrors exactly how this project's own master drives MOSI.
    always @(negedge sck or posedge cs_n) begin
        if (cs_n) begin
            miso <= 1'b0;
        end else if (phase == PHASE_DATA && !we) begin
            miso <= cur_out[3'd7 - bitcnt];
        end else begin
            miso <= 1'b0;
        end
    end

endmodule
