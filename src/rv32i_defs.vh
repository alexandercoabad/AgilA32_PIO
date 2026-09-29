// rv32i_defs.vh -- shared opcode/funct encodings for the AgilA32 core
// Included by rv32i_core.v

`ifndef RV32I_DEFS_VH
`define RV32I_DEFS_VH

// Opcodes (bits [6:0] of the instruction)
`define OP_LUI     7'b0110111
`define OP_AUIPC   7'b0010111
`define OP_JAL     7'b1101111
`define OP_JALR    7'b1100111
`define OP_BRANCH  7'b1100011
`define OP_LOAD    7'b0000011
`define OP_STORE   7'b0100011
`define OP_IMM     7'b0010011
`define OP_REG     7'b0110011
`define OP_SYSTEM  7'b1110011  // ECALL/FENCE still NOP; EBREAK halts -- see
                                // ST_HALTED below and rv32i_core.v's EXEC stage

// FSM states
`define ST_FETCH      3'd0
`define ST_FETCH_WAIT 3'd5   // holds while mem_ready is low (external fetch)
`define ST_DECODE     3'd1
`define ST_EXEC       3'd2
`define ST_MEM        3'd3
`define ST_MEM_WAIT   3'd6   // holds while mem_ready is low (external load/store)
`define ST_WB         3'd4
`define ST_HALTED     3'd7   // EBREAK landed here; stays until rst_n -- see
                              // rv32i_core.v's `halted` output, ported from
                              // AgilA8's a8_core.v S_HALTED/`halted` pattern

`endif
