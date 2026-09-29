// rv32i_core.v -- minimal multi-cycle RV32I core for Tiny Tapeout
//
// Implements the full RV32I base integer ISA. FENCE/ECALL still decode
// as NOPs (no trap support in this version), but EBREAK now actually
// halts the core: it parks the FSM in ST_HALTED (no further fetches,
// no memory activity) until the next reset, and a `halted` output goes
// high for as long as the core stays there. This mirrors AgilA8's
// a8_core.v S_HALTED state/`halted` output -- ported here as a genuine
// EBREAK trap rather than AgilA8's dedicated HALT opcode, since this
// core doesn't have a spare opcode to dedicate the way AgilA8's 16
// -instruction encoding did.
//
// Multi-cycle FSM: FETCH -> FETCH_WAIT -> DECODE -> EXEC -> MEM ->
// MEM_WAIT -> WB -> FETCH. Every instruction now takes 7 clock cycles
// (up from 5) even when every access is on-chip -- the *_WAIT states
// exist to let mem_ready stay low for multiple cycles during an
// external QSPI access, and that costs one extra cycle even for the
// on-chip case where mem_ready is already high the moment the state
// is entered. If you're relying on exact cycle-count timing anywhere
// (e.g. a testbench, or the default demo program's LED-count rate),
// re-check it against this new count.

`default_nettype none
`include "rv32i_defs.vh"

module rv32i_core (
    input  wire        clk,
    input  wire        rst_n,

    // memory-mapped bus (see mem.v for the address map)
    output reg  [7:0]  mem_addr,
    output reg  [31:0] mem_wdata,
    output reg  [1:0]  mem_size,
    output reg         mem_we,
    output reg         mem_valid,  // held high for the duration of a real access
    input  wire        mem_ready,  // pulses/holds high once the access completes
                                   // (mem.v ties this high whenever no external
                                   // transaction is outstanding, so on-chip
                                   // accesses see exactly the same 1-cycle
                                   // response as before)
    input  wire [31:0] mem_rdata,

    // High for as long as the core is parked in ST_HALTED (reached only
    // via EBREAK -- see header above). Ported from AgilA8's `halted`
    // signal; consumed by the top level to drive PIN_MUX's halted-status
    // output mode (see mem.v's PIN_MUX register).
    output wire        halted
);

    reg [2:0]  state;
    reg [7:0]  pc;
    reg [31:0] ir;          // latched instruction
    reg [31:0] alu_result;
    reg [7:0]  next_pc;

    // ------------------------------------------------------------
    // Register file (x0 hardwired to 0). `mem2reg` + `ifndef
    // SYNTHESIS` here for the same lint-warning reasons as
    // mem.v's ram_words -- see the comment there.
    // ------------------------------------------------------------
    (* mem2reg *) reg [31:0] regs [1:31];
    integer i;
`ifndef SYNTHESIS
    initial for (i = 1; i <= 31; i = i + 1) regs[i] = 32'h0;
`endif

    wire [4:0] rd    = ir[11:7];
    wire [4:0] rs1   = ir[19:15];
    wire [4:0] rs2   = ir[24:20];
    wire [2:0] funct3 = ir[14:12];
    // Only bit 5 of funct7 (ADD/SUB, SRL/SRA disambiguation) is ever
    // used, so extract just that bit rather than all 7 -- avoids an
    // UNUSEDSIGNAL warning on the other 6.
    wire funct7_b5 = ir[30];
    wire [6:0] opcode = ir[6:0];

    // EBREAK = OP_SYSTEM, funct3=000, imm[11:0]=0x001 (rd/rs1 both 0 per
    // spec, but not checked here -- any OP_SYSTEM word with this exact
    // funct3/imm12 halts, matching how real EBREAK decoders work).
    // ECALL (imm12=0x000) and FENCE (a different opcode entirely) still
    // fall through to the default NOP behavior below.
    wire is_ebreak = (opcode == `OP_SYSTEM) && (funct3 == 3'b000) &&
                      (ir[31:20] == 12'h001);

    assign halted = (state == `ST_HALTED);

    wire [31:0] rs1_val = (rs1 == 5'd0) ? 32'h0 : regs[rs1];
    wire [31:0] rs2_val = (rs2 == 5'd0) ? 32'h0 : regs[rs2];

    // ------------------------------------------------------------
    // Immediate decode (sign-extended)
    // ------------------------------------------------------------
    wire [31:0] imm_i = {{20{ir[31]}}, ir[31:20]};
    wire [31:0] imm_s = {{20{ir[31]}}, ir[31:25], ir[11:7]};
    wire [31:0] imm_u = {ir[31:12], 12'b0};
    // imm_b/imm_j only ever get used through their low 8 bits (PC is
    // 8-bit here), so build just those 8 bits directly instead of the
    // full 32-bit sign-extended immediate -- avoids an UNUSEDSIGNAL
    // warning on bits[31:8], which were computed but never read.
    // Equivalent to the standard B-type imm[7:0] = {imm[10:5][2:0],
    // imm[4:1], 1'b0} and J-type imm[7:0] = {imm[10:1][7:1], 1'b0}
    // slices of the full RISC-V immediates.
    wire [7:0] imm_b = {ir[27], ir[26], ir[25], ir[11], ir[10], ir[9], ir[8], 1'b0};
    wire [7:0] imm_j = {ir[27], ir[26], ir[25], ir[24], ir[23], ir[22], ir[21], 1'b0};

    // ------------------------------------------------------------
    // ALU
    // ------------------------------------------------------------
    reg [31:0] alu_a, alu_b;
    reg [3:0]  alu_op;   // 0=add 1=sub 2=sll 3=slt 4=sltu 5=xor 6=srl 7=sra 8=or 9=and
    reg [31:0] alu_y;

    always @(*) begin
        case (alu_op)
            4'd0: alu_y = alu_a + alu_b;
            4'd1: alu_y = alu_a - alu_b;
            4'd2: alu_y = alu_a << alu_b[4:0];
            4'd3: alu_y = ($signed(alu_a) < $signed(alu_b)) ? 32'd1 : 32'd0;
            4'd4: alu_y = (alu_a < alu_b) ? 32'd1 : 32'd0;
            4'd5: alu_y = alu_a ^ alu_b;
            4'd6: alu_y = alu_a >> alu_b[4:0];
            4'd7: alu_y = $signed(alu_a) >>> alu_b[4:0];
            4'd8: alu_y = alu_a | alu_b;
            4'd9: alu_y = alu_a & alu_b;
            default: alu_y = 32'h0;
        endcase
    end

    // Decode ALU operands/op combinationally from the latched instruction
    always @(*) begin
        alu_a = rs1_val;
        alu_b = rs2_val;
        alu_op = 4'd0;
        case (opcode)
            `OP_IMM: begin
                alu_b = imm_i;
                case (funct3)
                    3'b000: alu_op = 4'd0; // ADDI
                    3'b010: alu_op = 4'd3; // SLTI
                    3'b011: alu_op = 4'd4; // SLTIU
                    3'b100: alu_op = 4'd5; // XORI
                    3'b110: alu_op = 4'd8; // ORI
                    3'b111: alu_op = 4'd9; // ANDI
                    3'b001: alu_op = 4'd2; // SLLI
                    3'b101: alu_op = funct7_b5 ? 4'd7 : 4'd6; // SRAI/SRLI
                    default: alu_op = 4'd0;
                endcase
            end
            `OP_REG: begin
                case (funct3)
                    3'b000: alu_op = funct7_b5 ? 4'd1 : 4'd0; // SUB/ADD
                    3'b001: alu_op = 4'd2; // SLL
                    3'b010: alu_op = 4'd3; // SLT
                    3'b011: alu_op = 4'd4; // SLTU
                    3'b100: alu_op = 4'd5; // XOR
                    3'b101: alu_op = funct7_b5 ? 4'd7 : 4'd6; // SRA/SRL
                    3'b110: alu_op = 4'd8; // OR
                    3'b111: alu_op = 4'd9; // AND
                    default: alu_op = 4'd0;
                endcase
            end
            `OP_LOAD, `OP_STORE: begin
                alu_a = rs1_val;
                alu_b = (opcode == `OP_LOAD) ? imm_i : imm_s;
                alu_op = 4'd0; // address = base + offset
            end
            `OP_BRANCH: begin
                alu_a = rs1_val;
                alu_b = rs2_val;
                alu_op = 4'd1; // subtract, used for eq/lt comparisons below
            end
            default: begin
                alu_a = rs1_val;
                alu_b = rs2_val;
                alu_op = 4'd0;
            end
        endcase
    end

    // Branch condition evaluation (uses rs1_val/rs2_val directly)
    reg branch_cond;
    always @(*) begin
        case (funct3)
            3'b000: branch_cond = (rs1_val == rs2_val);                       // BEQ
            3'b001: branch_cond = (rs1_val != rs2_val);                       // BNE
            3'b100: branch_cond = ($signed(rs1_val) <  $signed(rs2_val));     // BLT
            3'b101: branch_cond = ($signed(rs1_val) >= $signed(rs2_val));     // BGE
            3'b110: branch_cond = (rs1_val < rs2_val);                        // BLTU
            3'b111: branch_cond = (rs1_val >= rs2_val);                       // BGEU
            default: branch_cond = 1'b0;
        endcase
    end

    // ------------------------------------------------------------
    // Main FSM
    // ------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= `ST_FETCH;
            pc       <= 8'h00;
            ir       <= 32'h0;
            mem_we   <= 1'b0;
            mem_addr <= 8'h0;
            mem_size <= 2'd2;
            mem_wdata<= 32'h0;
            mem_valid<= 1'b0;
        end else begin
            case (state)
                `ST_FETCH: begin
                    mem_addr <= pc;
                    mem_we   <= 1'b0;
                    mem_size <= 2'd2;
                    mem_valid<= 1'b1;
                    state    <= `ST_FETCH_WAIT;
                end

                `ST_FETCH_WAIT: begin
                    if (mem_ready) begin
                        mem_valid <= 1'b0;
                        state     <= `ST_DECODE;
                    end
                end

                `ST_DECODE: begin
                    ir    <= mem_rdata;   // latch fetched instruction
                    state <= `ST_EXEC;
                end

                `ST_EXEC: begin
                    if (is_ebreak) begin
                        // Park here for good -- no memory access, no
                        // writeback, no further fetches until rst_n.
                        mem_we    <= 1'b0;
                        mem_valid <= 1'b0;
                        state     <= `ST_HALTED;
                    end else begin
                        alu_result <= alu_y;
                        case (opcode)
                            `OP_JAL:    next_pc <= pc + imm_j;
                            `OP_JALR:   next_pc <= (rs1_val[7:0] + imm_i[7:0]) & 8'hFE;
                            `OP_BRANCH: next_pc <= branch_cond ? (pc + imm_b) : (pc + 8'd4);
                            default:    next_pc <= pc + 8'd4;
                        endcase
                        state <= `ST_MEM;
                    end
                end

                `ST_MEM: begin
                    case (opcode)
                        `OP_LOAD: begin
                            mem_addr <= alu_y[7:0];
                            mem_we   <= 1'b0;
                            mem_size <= (funct3[1:0] == 2'b00) ? 2'd0 :
                                        (funct3[1:0] == 2'b01) ? 2'd1 : 2'd2;
                            mem_valid<= 1'b1;
                        end
                        `OP_STORE: begin
                            mem_addr  <= alu_y[7:0];
                            mem_wdata <= rs2_val;
                            mem_we    <= 1'b1;
                            mem_size  <= (funct3[1:0] == 2'b00) ? 2'd0 :
                                         (funct3[1:0] == 2'b01) ? 2'd1 : 2'd2;
                            mem_valid <= 1'b1;
                        end
                        default: begin
                            mem_we    <= 1'b0;
                            mem_valid <= 1'b0;  // no real access this instruction
                        end
                    endcase
                    state <= `ST_MEM_WAIT;
                end

                `ST_MEM_WAIT: begin
                    if (mem_ready) begin
                        mem_valid <= 1'b0;
                        mem_we    <= 1'b0;
                        state     <= `ST_WB;
                    end
                end

                `ST_WB: begin
                    mem_we <= 1'b0;
                    if (rd != 5'd0) begin
                        case (opcode)
                            `OP_LUI:   regs[rd] <= imm_u;
                            `OP_AUIPC: regs[rd] <= {24'b0, pc} + imm_u;
                            `OP_JAL:   regs[rd] <= {24'b0, pc} + 32'd4;
                            `OP_JALR:  regs[rd] <= {24'b0, pc} + 32'd4;
                            `OP_LOAD: begin
                                case (funct3)
                                    3'b000: regs[rd] <= {{24{mem_rdata[7]}},  mem_rdata[7:0]};   // LB
                                    3'b001: regs[rd] <= {{16{mem_rdata[15]}}, mem_rdata[15:0]};  // LH
                                    3'b010: regs[rd] <= mem_rdata;                               // LW
                                    3'b100: regs[rd] <= {24'b0, mem_rdata[7:0]};                  // LBU
                                    3'b101: regs[rd] <= {16'b0, mem_rdata[15:0]};                 // LHU
                                    default: regs[rd] <= mem_rdata;
                                endcase
                            end
                            `OP_IMM, `OP_REG: regs[rd] <= alu_result;
                            default: ; // BRANCH/STORE/SYSTEM don't write rd
                        endcase
                    end
                    pc    <= next_pc;
                    state <= `ST_FETCH;
                end

                `ST_HALTED: begin
                    // Stay halted until reset -- matches AgilA8's
                    // S_HALTED behavior. No memory activity, no state
                    // exit other than rst_n.
                    mem_valid <= 1'b0;
                    mem_we    <= 1'b0;
                    state     <= `ST_HALTED;
                end

                default: state <= `ST_FETCH;
            endcase
        end
    end

endmodule
