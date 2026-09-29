// tb_pio_isa.v -- directed instruction-semantics test for the PIO state machine.
//
// Every instruction here is executed via the host INSTR register on a
// *disabled* state machine (host-forced instructions run regardless of
// SM_ENABLE), and the results are read back through the RX FIFO (MOV ISR,x
// then PUSH) or the ADDR / IRQ / PINS_OUT registers. Expected values are
// hand-derived from the RP2040 datasheet (section 3.4), not from the RTL.
//
// Instruction encodings used below (delay = 0, side-set = 0):
//   SET  x,n = E020|n   SET y,n = E040|n   SET pins,n = E000|n  SET pindirs,n = E080|n
//   MOV  dst,src = A000 | dst<<5 | op<<3 | src
//        dst: pins0 x1 y2 exec4 pc5 isr6 osr7   src: pins0 x1 y2 null3 status5 isr6 osr7
//        op : 0 none, 1 invert, 2 bit-reverse
//   IN   src,n = 4000 | src<<5 | n     (src: pins0 x1 y2 null3 isr6 osr7; n=0 means 32)
//   OUT  dst,n = 6000 | dst<<5 | n     (dst: pins0 x1 y2 null3 pindirs4 pc5 isr6 exec7)
//   PUSH = 8000 (|20 block, |40 iffull)   PULL = 8080 (|20 block, |40 ifempty)
//   JMP  cond,a = 0000 | cond<<5 | a   (cond: always0 !x1 x--2 !y3 y--4 x!=y5 pin6 !osre7)
//   IRQ  = C000 | clr<<6 | wait<<5 | idx     WAIT = 2000 | pol<<7 | src<<5 | idx

`timescale 1ns/1ps
`default_nettype none

module tb_pio_isa;
    reg         clk, rst_n;
    reg         valid, we;
    reg  [7:0]  addr;
    reg  [31:0] wdata;
    wire [31:0] rdata;
    wire        sel;
    reg  [9:0]  ext_in;
    wire [9:0]  pin_out, pin_dir, pin_own;

    `include "tb_pio_common.vh"

    pio #(.N_SM(2), .FIFO_LOG2(2)) dut (
        .clk(clk), .rst_n(rst_n),
        .valid(valid), .we(we), .addr(addr), .wdata(wdata),
        .rdata(rdata), .sel(sel),
        .pins_raw(ext_in),
        .pin_out(pin_out), .pin_dir(pin_dir), .pin_own(pin_own));

    // ---- helpers -------------------------------------------------------
    task ex;    input [15:0] ins; begin sm_exec(0, ins); end endtask
    task ex1;   input [15:0] ins; begin sm_exec(1, ins); end endtask

    // OUT EXEC / MOV EXEC leave the produced instruction pending; it executes in
    // place of the SM's next fetch, so briefly enable the SM (imem is all `jmp 0`).
    task run_pending;
        begin
            pio_wr(R_CTRL, 32'h0000_0001);
            repeat (8) @(posedge clk);
            pio_wr(R_CTRL, 32'h0000_0000);
            repeat (2) @(posedge clk);
        end
    endtask

    task get_x;   begin ex(16'hA0C1); ex(16'h8000); pio_rd(smreg(0, SM_RXF)); end endtask   // MOV ISR,X ; PUSH
    task get_y;   begin ex(16'hA0C2); ex(16'h8000); pio_rd(smreg(0, SM_RXF)); end endtask
    task get_osr; begin ex(16'hA0C7); ex(16'h8000); pio_rd(smreg(0, SM_RXF)); end endtask
    task get_isr; begin ex(16'h8000); pio_rd(smreg(0, SM_RXF)); end endtask                 // clears ISR

    task load_osr;                    // OSR <- v via TX FIFO + blocking PULL
        input [31:0] v;
        begin
            pio_wr(smreg(0, SM_TXF), v);
            ex(16'h80A0);
        end
    endtask
    task load_x;                      // X <- v
        input [31:0] v;
        begin load_osr(v); ex(16'h6020); end   // OUT X,32
    endtask

    task get_pc; begin pio_rd(smreg(0, SM_ADDR)); end endtask

    task set_shift;                   // in_right, out_right, push_thr, pull_thr, autopush, autopull
        input in_r; input out_r; input [4:0] pth; input [4:0] plt; input ap; input al;
        begin pio_wr(smreg(0, SM_SHIFT), shiftctrl(ap, al, in_r, out_r, pth, plt)); end
    endtask

    task fifo_clear; begin pio_wr(R_CTRL, 32'h0000_F000); end endtask

    integer i;
    reg [31:0] v;

    initial begin
        valid = 0; we = 0; addr = 0; wdata = 0; ext_in = 0;
        rst_n = 0; repeat (4) @(posedge clk); rst_n = 1; repeat (2) @(posedge clk);

        // ================= reset values (RP2040 datasheet) =================
        pio_rd(smreg(0, SM_CLKDIV));  check(rd_val === 32'h0001_0000, "CLKDIV reset = 1.0");
        pio_rd(smreg(0, SM_EXEC));    check(rd_val === 32'h0001_F000, "EXECCTRL reset: WRAP_TOP=31");
        pio_rd(smreg(0, SM_SHIFT));   check(rd_val === 32'h000C_0000, "SHIFTCTRL reset: both shifts right");
        pio_rd(smreg(0, SM_PINCTRL)); check(rd_val === 32'h1400_0000, "PINCTRL reset: SET_COUNT=5");
        pio_rd(R_CTRL);               check(rd_val === 32'h0, "CTRL reset: all SMs disabled");

        // ================= SET / MOV =================
        ex(16'hE03F & 16'hE03F | 16'h001F);            // SET X,31
        get_x; check(rd_val === 32'd31, "SET X,31");
        ex(16'hE047);                                   // SET Y,7
        get_y; check(rd_val === 32'd7,  "SET Y,7");
        ex(16'hA029);                                   // MOV X,!X
        get_x; check(rd_val === 32'hFFFF_FFE0, "MOV X,!X inverts");
        ex(16'hA052);                                   // MOV Y,::Y  (bit reverse)
        get_y; check(rd_val === 32'hE000_0000, "MOV Y,::Y reverses bits (7 -> E0000000)");
        ex(16'hA02B);                                   // MOV X,!NULL
        get_x; check(rd_val === 32'hFFFF_FFFF, "MOV X,!NULL = all ones");
        ex(16'hA043);                                   // MOV Y,NULL
        get_y; check(rd_val === 32'h0, "MOV Y,NULL = 0");
        ex(16'hA041);                                   // MOV Y,X
        get_y; check(rd_val === 32'hFFFF_FFFF, "MOV Y,X copies");

        // ================= OUT: shift right =================
        set_shift(1, 1, 5'd0, 5'd0, 0, 0);
        ex(16'hA0EB);                                   // MOV OSR,!NULL
        ex(16'h6028);                                   // OUT X,8
        get_x;   check(rd_val === 32'h0000_00FF, "OUT X,8 (right) takes the low byte");
        get_osr; check(rd_val === 32'h00FF_FFFF, "OSR shifted right by 8, zero filled");
        load_osr(32'hDEAD_BEEF);
        ex(16'h6040);                                   // OUT Y,32 (count field 0 means 32)
        get_y;   check(rd_val === 32'hDEAD_BEEF, "OUT Y,0 moves all 32 bits");
        load_osr(32'h1234_5678);
        ex(16'h6044);                                   // OUT Y,4
        get_y;   check(rd_val === 32'h0000_0008, "OUT Y,4 (right) = low nibble 8");

        // ================= OUT: shift left =================
        set_shift(1, 0, 5'd0, 5'd0, 0, 0);
        load_osr(32'h1234_5678);
        ex(16'h6028);                                   // OUT X,8
        get_x;   check(rd_val === 32'h0000_0012, "OUT X,8 (left) takes the top byte");
        get_osr; check(rd_val === 32'h3456_7800, "OSR shifted left by 8, zero filled");

        // ================= IN: shift right / left / count 32 =================
        set_shift(1, 1, 5'd0, 5'd0, 0, 0);
        ex(16'hA0C3);                                   // MOV ISR,NULL
        load_x(32'hAB);  ex(16'h4028);                  // IN X,8
        load_x(32'hCD);  ex(16'h4028);                  // IN X,8
        get_isr; check(rd_val === 32'hCDAB_0000, "IN right: newest byte enters at the top");
        set_shift(0, 1, 5'd0, 5'd0, 0, 0);
        ex(16'hA0C3);
        load_x(32'hAB);  ex(16'h4028);
        load_x(32'hCD);  ex(16'h4028);
        get_isr; check(rd_val === 32'h0000_ABCD, "IN left: newest byte enters at the bottom");
        load_x(32'hCAFE_F00D); ex(16'h4020);            // IN X,32
        get_isr; check(rd_val === 32'hCAFE_F00D, "IN X,0 (32 bits)");
        load_x(32'hFFFF_FFFF); ex(16'h4023);            // IN X,3 (isr was cleared by PUSH)
        get_isr; check(rd_val === 32'h0000_0007, "IN X,3 masks to 3 bits");

        // ================= PULL / PUSH edge cases =================
        set_shift(1, 1, 5'd8, 5'd8, 0, 0);              // push/pull thresholds = 8
        fifo_clear;
        load_x(32'd13);
        ex(16'h8080);                                   // PULL noblock, TX FIFO empty -> OSR <- X
        get_osr; check(rd_val === 32'd13, "noblock PULL on empty FIFO copies X into OSR");
        // IfEmpty when OSR not yet empty is a no-op
        load_osr(32'hAAAA_5555);                        // cnt = 0 (full)
        pio_wr(smreg(0, SM_TXF), 32'h1111_1111);
        ex(16'h80C0);                                   // PULL ifempty noblock
        get_osr; check(rd_val === 32'hAAAA_5555, "PULL IfEmpty is a no-op while OSR is not empty");
        pio_rd(smreg(0, SM_FLEVEL)); check(rd_val[2:0] === 3'd1, "IfEmpty no-op leaves the TX word queued");
        fifo_clear;
        // PUSH IfFull
        ex(16'hA0C3);                                   // ISR <- 0, cnt 0
        ex(16'h8040);                                   // PUSH iffull noblock : cnt 0 < 8 -> no-op
        pio_rd(R_FSTAT); check(rd_val[8] === 1'b1, "PUSH IfFull below threshold pushes nothing");
        load_x(32'h5A); ex(16'h4028);                   // IN X,8 -> cnt 8 == threshold
        ex(16'h8040);                                   // now pushes
        pio_rd(R_FSTAT); check(rd_val[8] === 1'b0, "PUSH IfFull at threshold pushes");
        pio_rd(smreg(0, SM_RXF)); check(rd_val === 32'h5A00_0000, "pushed ISR content (right shift)");
        // PUSH noblock on a FULL RX FIFO drops the data and clears ISR
        fifo_clear;
        for (i = 0; i < 4; i = i + 1) begin ex(16'hE021 + i); ex(16'hA0C1); ex(16'h8000); end
        pio_rd(R_FSTAT); check(rd_val[0] === 1'b1, "RX FIFO full after 4 pushes");
        ex(16'hA0C1);                                   // ISR <- X
        ex(16'h8000);                                   // PUSH noblock into a full FIFO
        pio_rd(smreg(0, SM_FLEVEL)); check(rd_val[6:4] === 3'd4, "PUSH into full FIFO must not overflow it");
        for (i = 0; i < 4; i = i + 1) begin
            pio_rd(smreg(0, SM_RXF));
            check(rd_val === (32'd1 + i) , "RX FIFO keeps the original 4 words in order");
        end
        // (X was set to 1..4 by SET X,(1+i)  ... SET X = E020|n so values 1,2,3,4)
        fifo_clear;

        // ================= eager autopull refill (datasheet 3.5.4.1) =================
        // After an OUT that drives the shift count up to PULL_THRESH the OSR is refilled
        // *immediately* from the FIFO if data is available, so !OSRE sees it as full
        // and the FIFO level drops now, not at the next OUT.
        set_shift(1, 1, 5'd8, 5'd8, 0, 1);              // autopull, threshold 8
        fifo_clear;
        pio_wr(smreg(0, SM_TXF), 32'hA1A2_A3A4);
        pio_wr(smreg(0, SM_TXF), 32'hB1B2_B3B4);
        ex(16'h80A0);                                   // explicit PULL -> OSR=A.., FIFO=[B..]
        ex(16'h6028);                                   // OUT X,8 -> cnt reaches 8 == threshold
        pio_rd(smreg(0, SM_FLEVEL));
        check(rd_val[2:0] === 3'd0, "autopull refills eagerly: TX FIFO drained by the OUT itself");
        get_x;   check(rd_val === 32'h0000_00A4, "OUT X,8 got the low byte of the first word");
        get_osr; check(rd_val === 32'hB1B2_B3B4, "OSR now holds the refilled word, count 0");
        ex(16'h00E5);                                   // JMP !OSRE, 5  : OSR full -> taken
        get_pc;  check(rd_val[4:0] === 5'd5, "JMP !OSRE taken right after eager refill");
        set_shift(1, 1, 5'd0, 5'd0, 0, 0);
        fifo_clear;

        // ================= JMP conditions =================
        ex(16'h0011);                                   // JMP 17
        get_pc; check(rd_val[4:0] === 5'd17, "JMP always");
        load_x(32'd0);
        ex(16'h0029);                                   // JMP !X,9 (X==0 -> taken)
        get_pc; check(rd_val[4:0] === 5'd9,  "JMP !X taken when X==0");
        load_x(32'd5);
        ex(16'h0023);                                   // JMP !X,3 (X!=0 -> not taken)
        get_pc; check(rd_val[4:0] === 5'd9,  "JMP !X not taken when X!=0 (PC unchanged)");
        load_x(32'd2);
        ex(16'h004C);                                   // JMP X--,12
        get_pc; check(rd_val[4:0] === 5'd12, "JMP X-- taken (X=2)");
        get_x;  check(rd_val === 32'd1, "X decremented after taken JMP X--");
        ex(16'h0045);                                   // JMP X--,5 (X=1 -> taken, X=0)
        get_pc; check(rd_val[4:0] === 5'd5, "JMP X-- taken (X=1)");
        get_x;  check(rd_val === 32'd0, "X reaches 0");
        ex(16'h0048);                                   // JMP X--,8 (X==0 -> NOT taken, but still decrements)
        get_pc; check(rd_val[4:0] === 5'd5, "JMP X-- not taken at X==0");
        get_x;  check(rd_val === 32'hFFFF_FFFF, "JMP X-- decrements even when not taken");
        ex(16'hE043);                                   // SET Y,3
        ex(16'h0084);                                   // JMP Y--,4
        get_pc; check(rd_val[4:0] === 5'd4, "JMP Y-- taken");
        get_y;  check(rd_val === 32'd2, "Y decremented");
        ex(16'h0060 | 5'd6);                            // JMP !Y,6  (Y=2 -> not taken)
        get_pc; check(rd_val[4:0] === 5'd4, "JMP !Y not taken when Y!=0");
        ex(16'hE021); ex(16'hE041);                     // X=1, Y=1
        ex(16'h00A0 | 5'd20);                           // JMP X!=Y,20 (equal -> not taken)
        get_pc; check(rd_val[4:0] === 5'd4, "JMP X!=Y not taken when equal");
        ex(16'hE042);                                   // Y=2
        ex(16'h00A0 | 5'd20);
        get_pc; check(rd_val[4:0] === 5'd20, "JMP X!=Y taken when different");
        // JMP PIN uses EXECCTRL.JMP_PIN
        pio_wr(smreg(0, SM_EXEC), execctrl(5'd0, 5'd31, 1'b0, 1'b0, 4'd5));   // JMP_PIN = 5
        pio_rd(smreg(0, SM_EXEC)); check(rd_val[27:24] === 4'd5, "EXECCTRL.JMP_PIN read-back");
        ex(16'h0000);                                   // JMP 0 to reset pc
        ext_in = 10'b00_0010_0000; repeat (4) @(posedge clk);
        ex(16'h00C0 | 5'd22);                           // JMP PIN,22
        get_pc; check(rd_val[4:0] === 5'd22, "JMP PIN taken when pin 5 high");
        ex(16'h0000);
        ext_in = 10'b00_0000_0000; repeat (4) @(posedge clk);
        ex(16'h00C0 | 5'd22);
        get_pc; check(rd_val[4:0] === 5'd0, "JMP PIN not taken when pin 5 low");
        // JMP !OSRE : OSR empty (count 32) vs not
        ex(16'hA0EB);                                   // MOV OSR,!NULL  -> full
        ex(16'h00E0 | 5'd7);
        get_pc; check(rd_val[4:0] === 5'd7, "JMP !OSRE taken when OSR has bits left");
        ex(16'h6020 | 5'd0);                            // OUT NULL... (dst 1 = X) drains all 32 bits
        ex(16'h0000);
        ex(16'h00E0 | 5'd9);
        get_pc; check(rd_val[4:0] === 5'd0, "JMP !OSRE not taken when OSR fully shifted out");

        // ================= pins: OUT/SET/MOV pins, PINDIRS, side-set =================
        pio_wr(smreg(0, SM_PINCTRL), pinctrl(4'd6, 4'd3, 4'd0, 3'd5, 4'd0, 3'd0, 4'd0));
        set_shift(1, 1, 5'd0, 5'd0, 0, 0);
        load_osr(32'h0000_0005);
        ex(16'h6003);                                   // OUT PINS,3 -> pins 6..8 = 1,0,1 (LSB first)
        pio_rd(R_PINS_OUT);
        check(rd_val[9:0] === 10'b01_0100_0000, "OUT PINS,3 writes pins 6,7,8 = 1,0,1");
        ex(16'hE01F);                                   // SET PINS,31 (set_count 5 -> pins 0..4)
        pio_rd(R_PINS_OUT);
        check(rd_val[4:0] === 5'b11111 && rd_val[8:6] === 3'b101, "SET PINS,31 drives pins 0-4, others untouched");
        ex(16'hE08F);                                   // SET PINDIRS,15
        pio_rd(R_PINS_OUT);
        check(rd_val[25:16] === 10'b00_0000_1111, "SET PINDIRS,15 sets dirs for pins 0-3");
        load_x(32'h0000_0002);
        ex(16'hA001);                                   // MOV PINS,X -> OUT window pins 6..8 <- 010
        pio_rd(R_PINS_OUT);
        check(rd_val[8:6] === 3'b010, "MOV PINS,X writes the OUT window");
        // out_base wrap: OUT_BASE=9,count=3 -> pins 9,10,11 ; only pin 9 is connected
        pio_wr(smreg(0, SM_PINCTRL), pinctrl(4'd9, 4'd3, 4'd0, 3'd5, 4'd0, 3'd0, 4'd0));
        load_osr(32'h0000_0007);
        ex(16'h6003);
        pio_rd(R_PINS_OUT);
        check(rd_val[9] === 1'b1, "OUT window at base 9: pin 9 driven");
        // IN pins with in_base rotation: ext_in = 0b1011_0100_00, in_base=4, IN PINS,4 -> pins 4..7
        pio_wr(smreg(0, SM_PINCTRL), pinctrl(4'd0, 4'd0, 4'd0, 3'd0, 4'd0, 3'd0, 4'd4));
        ext_in = 10'b10_1101_0000; repeat (4) @(posedge clk);   // pins 4..7 = 1,0,1,1 (LSB first)
        ex(16'hA0C3);                                   // ISR <- 0
        set_shift(0, 1, 5'd0, 5'd0, 0, 0);              // shift left so IN PINS,4 lands as-is
        ex(16'h4004);                                   // IN PINS,4
        get_isr; check(rd_val === 32'h0000_000D, "IN PINS,4 with IN_BASE=4 reads pins 4..7 = 0xD");
        set_shift(1, 1, 5'd0, 5'd0, 0, 0);

        // side-set overrides a same-pin SET in the same instruction
        pio_wr(smreg(0, SM_PINCTRL), pinctrl(4'd0, 4'd0, 4'd0, 3'd1, 4'd0, 3'd1, 4'd0)); // SET 1 pin @0, SIDE 1 bit @0
        ex(16'hE001 | 16'h1000);                        // SET PINS,1  side 1
        pio_rd(R_PINS_OUT); check(rd_val[0] === 1'b1, "side 1 + SET PINS,1 -> 1");
        ex(16'hE001);                                   // SET PINS,1  side 0  -> side-set wins
        pio_rd(R_PINS_OUT); check(rd_val[0] === 1'b0, "side-set (0) overrides SET PINS,1 on the same pin");
        // side-set to PINDIRS
        pio_wr(smreg(0, SM_EXEC), execctrl(5'd0, 5'd31, 1'b0, 1'b1, 4'd0));   // SIDE_PINDIR
        ex(16'hE000 | 16'h1000);                        // NOP-ish: SET PINS,0 side 1 -> dir bit0 = 1
        pio_rd(R_PINS_OUT); check(rd_val[16] === 1'b1, "SIDE_PINDIR: side-set drives the direction register");
        pio_wr(smreg(0, SM_EXEC), execctrl(5'd0, 5'd31, 1'b0, 1'b0, 4'd0));

        // ================= OUT EXEC / MOV EXEC / OUT PC =================
        pio_wr(smreg(0, SM_PINCTRL), pinctrl(4'd0, 4'd0, 4'd0, 3'd5, 4'd0, 3'd0, 4'd0));
        load_osr(32'h0000_E02A);                        // SET X,10
        ex(16'h60E0 | 5'd16);                           // OUT EXEC,16 -> executes the instruction in OSR
        run_pending;
        get_x; check(rd_val === 32'd10, "OUT EXEC runs the shifted-out instruction (SET X,10)");
        load_x(32'h0000_E03B);                          // X = "SET X,27"
        ex(16'hA081);                                   // MOV EXEC,X
        run_pending;
        get_x; check(rd_val === 32'd27, "MOV EXEC,X runs the instruction held in X");
        load_osr(32'h0000_0013);
        ex(16'h60A0 | 5'd5);                            // OUT PC,5 -> pc = 19
        get_pc; check(rd_val[4:0] === 5'd19, "OUT PC,5 sets the program counter");

        // ================= IRQ =================
        pio_wr(R_IRQ, 32'hFF);
        ex(16'hC003);                                   // IRQ set 3
        pio_rd(R_IRQ); check(rd_val[7:0] === 8'h08, "IRQ set 3");
        ex(16'hC043);                                   // IRQ clear 3
        pio_rd(R_IRQ); check(rd_val[7:0] === 8'h00, "IRQ clear 3");
        ex1(16'hC012);                                  // SM1: IRQ set 2 rel  (0x10 = REL) -> flag (2+1)%4 = 3
        pio_rd(R_IRQ); check(rd_val[7:0] === 8'h08, "IRQ 2 rel on SM1 hits flag 3");
        ex(16'hC012);                                   // SM0: IRQ set 2 rel -> flag 2
        pio_rd(R_IRQ); check(rd_val[7:0] === 8'h0C, "IRQ 2 rel on SM0 hits flag 2");
        pio_wr(R_IRQ, 32'h0C);
        pio_wr(R_IRQ_FORCE, 32'h0000_0081);
        pio_rd(R_IRQ); check(rd_val[7:0] === 8'h81, "IRQ_FORCE sets flags from the host");
        pio_wr(R_IRQ, 32'hFF);
        // IRQ WAIT: sets the flag, then stalls until the host clears it
        pio_wr(smreg(0, SM_INSTR), 32'hC024);           // IRQ wait 4
        repeat (8) @(posedge clk);
        pio_rd(R_IRQ);                check(rd_val[4] === 1'b1, "IRQ wait raises its flag");
        pio_rd(smreg(0, SM_EXEC));    check(rd_val[31] === 1'b1, "IRQ wait stalls while flag is set");
        pio_wr(R_IRQ, 32'h10);
        repeat (8) @(posedge clk);
        pio_rd(smreg(0, SM_EXEC));    check(rd_val[31] === 1'b0, "IRQ wait completes once the flag is cleared");
        // WAIT 1 IRQ n: stalls until set, then clears the flag itself
        pio_wr(smreg(0, SM_INSTR), 32'h20C5);           // WAIT 1 IRQ 5
        repeat (8) @(posedge clk);
        pio_rd(smreg(0, SM_EXEC));    check(rd_val[31] === 1'b1, "WAIT 1 IRQ stalls until the flag is set");
        pio_wr(R_IRQ_FORCE, 32'h20);
        repeat (8) @(posedge clk);
        pio_rd(smreg(0, SM_EXEC));    check(rd_val[31] === 1'b0, "WAIT 1 IRQ completes when the flag is set");
        pio_rd(R_IRQ);                check(rd_val[5] === 1'b0, "WAIT 1 IRQ clears the flag as it completes");
        // WAIT on GPIO and on PIN (relative to IN_BASE)
        ext_in = 10'h000; repeat (4) @(posedge clk);
        pio_wr(smreg(0, SM_INSTR), 32'h2086);           // WAIT 1 GPIO 6
        repeat (8) @(posedge clk);
        pio_rd(smreg(0, SM_EXEC));    check(rd_val[31] === 1'b1, "WAIT 1 GPIO 6 stalls while pin 6 low");
        ext_in = 10'b00_0100_0000; repeat (6) @(posedge clk);
        pio_rd(smreg(0, SM_EXEC));    check(rd_val[31] === 1'b0, "WAIT 1 GPIO 6 releases when pin 6 goes high");
        pio_wr(smreg(0, SM_PINCTRL), pinctrl(4'd0, 4'd0, 4'd0, 3'd5, 4'd0, 3'd0, 4'd4)); // IN_BASE=4
        ext_in = 10'b00_0010_0000; repeat (4) @(posedge clk);                // pin 5 high
        pio_wr(smreg(0, SM_INSTR), 32'h2021);           // WAIT 0 PIN 1  (= pin 5)
        repeat (8) @(posedge clk);
        pio_rd(smreg(0, SM_EXEC));    check(rd_val[31] === 1'b1, "WAIT 0 PIN 1 (IN_BASE 4 -> pin 5) stalls while high");
        ext_in = 10'b00_0000_0000; repeat (6) @(posedge clk);
        pio_rd(smreg(0, SM_EXEC));    check(rd_val[31] === 1'b0, "WAIT 0 PIN 1 releases when pin 5 goes low");

        // ================= MOV STATUS =================
        pio_wr(smreg(0, SM_EXEC), execctrl(5'd0, 5'd31, 1'b0, 1'b0, 4'd0) | 32'h2);   // STATUS_N = 2, TX level < 2
        fifo_clear;
        ex(16'hA025);                                   // MOV X,STATUS  (dst x=1<<5=20, src status=5)
        get_x; check(rd_val === 32'hFFFF_FFFF, "STATUS true when TX level (0) < N (2)");
        pio_wr(smreg(0, SM_TXF), 32'd1); pio_wr(smreg(0, SM_TXF), 32'd2);
        ex(16'hA025);
        get_x; check(rd_val === 32'h0, "STATUS false when TX level (2) >= N (2)");

        repeat (20) @(posedge clk);
        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("FAIL: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #200_000_000;
        $display("FAIL: watchdog timeout");
        $finish;
    end
endmodule
