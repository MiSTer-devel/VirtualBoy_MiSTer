// NEC V810 micro-ROM: the decoder PLA's entry addresses and the micro-programs.
// Included inside necv810_useq.
//
// One word runs per clock and does one operation (35 bits):
//
//   field  bits   meaning
//   op     5:0    the operation. op[5:3] 0: ALU, 1: bus, 2: bit-string cursor unit, 3: the same,
//                 then wait for the bus. op[5] 1: FP helper (necv810_ufp.svh), op[4] 0 for its
//                 mantissa ops on the ALU and shifter, 1 for the exponent, sign and status ops
//   a      8:6    first source: 0, reg1, reg2, T0, T1, PSW, PK = {sign, exponent, 0}, r30
//   b      11:9   second source: 0, reg1, reg2, T0, T1, K (constant), T2, PK
//   dst    14:12  T0, T1, T2, or the routine's result for reg2 or PSW
//   imm    19:15  shift count, hold count, cursor register (26-30), constant index, or in an
//                 END word {trap, flag mode}
//   cond   24:20  jump condition: none, always, equal (last SUB), FP and bit-string tests
//   next   33:25  jump target
//   end    34     last clock: the result commits and interrupts are checked; in the
//                 exception entry, its last step
//
// FSR keeps the low word of {a, b} >> imm: a rotate when a = b, a left shift
// by 32 - imm when b = 0, a right shift when a = 0.

	localparam integer UW_OP   = 0;
	localparam integer UW_A    = 6;
	localparam integer UW_B    = 9;
	localparam integer UW_DST  = 12;
	localparam integer UW_IMM  = 15;
	localparam integer UW_COND = 20;
	localparam integer UW_NEXT = 25;
	localparam integer UW_END  = 34;
	localparam integer UW_BITS = 35;
	/* verilator lint_off UNUSEDPARAM */
	`include "necv810_ufp.svh"
	/* verilator lint_on UNUSEDPARAM */

	// Bus unit codes; 0 is none.
	localparam [2:0] UB_DRAIN  = 3'd1;    // hold until older work is done: retired, bus idle, write buffer empty
	localparam [2:0] UB_READ   = 3'd2;    // lock the bus and read; the data lands in T0
	localparam [2:0] UB_WAIT   = 3'd3;    // hold until the read is done
	localparam [2:0] UB_WRITE  = 3'd4;    // write T1, posted
	localparam [2:0] UB_UNLOCK = 3'd5;
	localparam [2:0] UB_COUNT  = 3'd6;    // hold imm more clocks
	localparam [2:0] UB_SETTLE = 3'd7;    // hold while older work is in MEM or WB; imm 1: while
	                                      // posted stores are buffered or on the bus

	// Bit-string cursor unit codes; 0 is none.
	localparam [2:0] UBS_LD   = 3'd1;     // cursor imm from register imm
	localparam [2:0] UBS_WB   = 3'd2;     // register imm from cursor imm, through the WB port
	localparam [2:0] UBS_RS   = 3'd3;     // read the source word at rp into the window
	localparam [2:0] UBS_RD   = 3'd4;     // read the destination word, merged into T1
	localparam [2:0] UBS_WR   = 3'd5;     // write T1 to the destination word, then step, shifting in T0 if read ahead
	localparam [2:0] UBS_SCAN = 3'd6;     // scan a1 and step
	localparam [2:0] UBS_RSN  = 3'd7;     // read ahead if more lies past this step: into T0, or for a search
	                                      // the window

	// Operations. ALU: a op b; MSTEP: {T0, T1} = (a + reg2 * b[3:0]) shifted in
	// above b >> 4; MSIGN: a - reg2 when b[0], the multiplier's sign bit, is set;
	// SUB sets the routine's {CY, OV, S, Z}.
	localparam [5:0] O_NOP    = 6'o00;
	localparam [5:0] O_PASS   = 6'o01;
	localparam [5:0] O_AND    = 6'o02;
	localparam [5:0] O_OR     = 6'o03;
	localparam [5:0] O_FSR    = 6'o04;
	localparam [5:0] O_MSTEP  = 6'o05;
	localparam [5:0] O_MSIGN  = 6'o06;
	localparam [5:0] O_SUB    = 6'o07;
	localparam [5:0] O_DRAIN  = {3'o1, UB_DRAIN};
	localparam [5:0] O_READ   = {3'o1, UB_READ};
	localparam [5:0] O_WAIT   = {3'o1, UB_WAIT};
	localparam [5:0] O_WRITE  = {3'o1, UB_WRITE};
	localparam [5:0] O_UNLOCK = {3'o1, UB_UNLOCK};
	localparam [5:0] O_COUNT  = {3'o1, UB_COUNT};
	localparam [5:0] O_SETTLE = {3'o1, UB_SETTLE};
	localparam [5:0] O_LD     = {3'o2, UBS_LD};
	localparam [5:0] O_WB     = {3'o2, UBS_WB};
	localparam [5:0] O_RS     = {3'o2, UBS_RS};
	localparam [5:0] O_RD     = {3'o2, UBS_RD};
	localparam [5:0] O_WR     = {3'o2, UBS_WR};
	localparam [5:0] O_SCAN   = {3'o2, UBS_SCAN};
	localparam [5:0] O_RSN    = {3'o2, UBS_RSN};
	localparam [5:0] O_WBW    = {3'o3, UBS_WB};     // write r30 back once the last write is done
	localparam [5:0] O_RSNW   = {3'o3, UBS_RSN};    // read ahead, then wait for the bus

	localparam [2:0] A_ZERO = 3'd0;
	localparam [2:0] A_R1   = 3'd1;
	localparam [2:0] A_R2   = 3'd2;
	localparam [2:0] A_T0   = 3'd3;
	localparam [2:0] A_T1   = 3'd4;
	localparam [2:0] A_PSW  = 3'd5;
	localparam [2:0] A_PK   = 3'd6;
	localparam [2:0] A_R30  = 3'd7;

	localparam [2:0] B_ZERO = 3'd0;
	localparam [2:0] B_R1   = 3'd1;
	localparam [2:0] B_R2   = 3'd2;
	localparam [2:0] B_T0   = 3'd3;
	localparam [2:0] B_T1   = 3'd4;
	localparam [2:0] B_K    = 3'd5;
	localparam [2:0] B_T2   = 3'd6;
	localparam [2:0] B_PK   = 3'd7;

	localparam [2:0] D_NONE = 3'd0;
	localparam [2:0] D_T0   = 3'd1;
	localparam [2:0] D_T1   = 3'd2;
	localparam [2:0] D_R2   = 3'd3;
	localparam [2:0] D_PSW  = 3'd4;
	localparam [2:0] D_T2   = 3'd5;

	// Constant table, by imm[3:0]: source K and the exponent unit's constant; entry 0 is 0.
	localparam [4:0] K_SIGN   = 5'd1;
	localparam [4:0] K_MBIAS  = 5'd2;
	localparam [4:0] K_DBIAS  = 5'd3;
	localparam [4:0] K_WSEXP  = 5'd4;
	localparam [4:0] K_SWFRAC = 5'd5;
	localparam [4:0] K_SWINT  = 5'd6;
	localparam [4:0] K_IMIN   = 5'd7;
	localparam [4:0] K_HI1    = 5'd8;     // REV's bit, pair and nibble masks
	localparam [4:0] K_LO1    = 5'd9;
	localparam [4:0] K_HI2    = 5'd10;
	localparam [4:0] K_LO2    = 5'd11;
	localparam [4:0] K_HI4    = 5'd12;
	localparam [4:0] K_LO4    = 5'd13;
	localparam [4:0] K_NOID   = 5'd14;    // CLI and SEI
	localparam [4:0] K_ID     = 5'd15;

	function [31:0] ukon_fn;
		input [3:0] k;
		begin
			case (k)
				4'd1:    ukon_fn = 32'h8000_0000;   // sign
				4'd2:    ukon_fn = 32'hffff_ff81;   // -127
				4'd3:    ukon_fn = 32'd127;
				4'd4:    ukon_fn = 32'd158;         // 127 + 31: integer to float
				4'd5:    ukon_fn = 32'd150;         // 127 + 23: float to integer, fraction
				4'd6:    ukon_fn = 32'd153;         // the same, integral, from the mantissa << 3
				4'd7:    ukon_fn = 32'hcf00_0000;   // -2^31 as a float
				4'd8:    ukon_fn = 32'haaaa_aaaa;
				4'd9:    ukon_fn = 32'h5555_5555;
				4'd10:   ukon_fn = 32'hcccc_cccc;
				4'd11:   ukon_fn = 32'h3333_3333;
				4'd12:   ukon_fn = 32'hf0f0_f0f0;
				4'd13:   ukon_fn = 32'h0f0f_0f0f;
				4'd14:   ukon_fn = 32'hffff_efff;   // ~PSW.ID
				4'd15:   ukon_fn = 32'h0000_1000;   // PSW.ID
				default: ukon_fn = 32'd0;
			endcase
		end
	endfunction

	// Where an access's read data goes.
	localparam [1:0] UL_NONE  = 2'd0;
	localparam [1:0] UL_T0    = 2'd1;
	localparam [1:0] UL_SRC   = 2'd2;
	localparam [1:0] UL_MERGE = 2'd3;

	localparam [8:0] UA_XB    = 9'd0;
	localparam [8:0] UA_REV   = 9'd4;
	localparam [8:0] UA_MPYHW = 9'd24;
	localparam [8:0] UA_CLI   = 9'd31;
	localparam [8:0] UA_SEI   = 9'd33;
	localparam [8:0] UA_SYNC  = 9'd35;
	localparam [8:0] UA_ENTRY = 9'd43;
	localparam [8:0] UA_WAKE  = UA_ENTRY;
	localparam [8:0] UA_QUICK = UA_ENTRY + 9'd3;  // an interrupt with nothing left to stop
	localparam [8:0] UA_ILL   = 9'd52;
	localparam [8:0] UA_DIV0  = 9'd56;
	localparam [8:0] UA_CAXI  = 9'd62;
	localparam [8:0] UA_CAXI_W = UA_CAXI + 9'd7;
	// The internal steps after the posted write, in clocks less one.
	localparam [4:0] CAXI_STEPS = 5'd12;

	// Bit strings, at 384-511: the arithmetic routine and the search, both on
	// the cursor unit.
	localparam [8:0] UA_BSA    = 9'd384;
	localparam [8:0] UA_BSA_F  = UA_BSA + 9'd10;   // the first destination word
	localparam [8:0] UA_BSA_L  = UA_BSA + 9'd20;   // the words after the second
	localparam [8:0] UA_BSA_X2 = UA_BSA + 9'd23;
	localparam [8:0] UA_BSA_X1 = UA_BSA + 9'd30;
	localparam [8:0] UA_BSA_GT = UA_BSA + 9'd31;
	localparam [8:0] UA_BSA_LT = UA_BSA + 9'd35;
	localparam [8:0] UA_BSU    = 9'd432;
	localparam [8:0] UA_BSD    = UA_BSU + 9'd10;
	localparam [8:0] UA_BSS_C  = UA_BSU + 9'd20;   // first word, then the loop
	localparam [8:0] UA_BSS_L  = UA_BSS_C + 9'd2;
	localparam [8:0] UA_BSS_X  = UA_BSS_L + 9'd4;
	localparam [8:0] UA_BSS_WB = UA_BSS_X + 9'd3;
	localparam [8:0] UA_BSS_F  = UA_BSS_WB + 9'd5;
	localparam [8:0] UA_BSS_C0 = UA_BSS_F + 9'd1;
	localparam [8:0] UA_BSS_P  = UA_BSS_C0 + 9'd1;  // a first word that starts inside it, alone
	localparam [8:0] UA_BSS_P0 = UA_BSS_P + 9'd10;
	// Exposed internal clocks, less one, where the routine runs on an idle bus.
	// Arithmetic set-up and the second-word entry: the 32-bit bus times less
	// the accesses; src ofs > dst ofs, after the extra read. Src ofs < dst ofs
	// waits for the first source word and takes a clock.
	localparam [4:0] BSA_SETUP = 5'd9;
	localparam [4:0] BSA_GT_T  = 5'd4;
	localparam [4:0] BSA_W2_T  = 5'd2;
	// Search set-up up and down: a string in one word, or in more from bit 0 (up)
	// or 31 (down), or from inside its first word, which is scanned alone before
	// the set-up for the rest; and the exits. On WRAM one word takes 33.3 clocks,
	// two 46.6, then 6.2 a word; down, one word is 1.2 cheaper and more words 4.7
	// dearer; SCH0 takes 1.3 more than SCH1.
	localparam [4:0] BSU_ONE   = 5'd2;
	localparam [4:0] BSU_MULTI = 5'd8;
	localparam [4:0] BSD_MULTI = 5'd11;
	localparam [4:0] BSU_REST  = 5'd12;
	localparam [4:0] BSD_REST  = 5'd19;
	localparam [4:0] BSS_MISS  = 5'd2;
	localparam [4:0] BSS_FOUND = 5'd5;

	// FP routines.
	localparam [8:0] UA_FRO   = 9'd338;   // shared trap exits: a hold, then the END
	localparam [8:0] UA_FIV   = 9'd340;
	localparam [8:0] UA_FZD   = 9'd342;   // the END alone: x / 0 holds in the DIVF routine
	localparam [8:0] UA_FOV   = 9'd343;
	// Clocks less one that an FP trap holds before its END.
	localparam [4:0] FX_HOLD  = 5'd7;
	localparam [8:0] UA_FW    = 9'd132;   // waits into an FP END, 7 words
	localparam [8:0] UA_TAIL  = 9'd140;   // normalise, round, pack
	localparam [8:0] UA_UFL   = 9'd149;
	localparam [8:0] UA_UFLD  = 9'd153;
	localparam [8:0] UA_CMPF  = 9'd160;
	localparam [8:0] UA_CMPZ  = UA_CMPF + 9'd4;    // zero dispatch, 4-aligned
	localparam [8:0] UA_CVTWS = 9'd168;
	localparam [8:0] UA_CVTSW = 9'd184;
	localparam [8:0] UA_ADDF  = 9'd216;
	localparam [8:0] UA_ADDA  = UA_ADDF + 9'd3;    // add path
	localparam [8:0] UA_ADDX  = UA_ADDF + 9'd11;   // signs differ: to the subtract path
	localparam [8:0] UA_ADDZ  = UA_ADDF + 9'd16;   // zero dispatch, 4-aligned
	localparam [8:0] UA_SUBF  = 9'd240;
	localparam [8:0] UA_SUBS  = UA_SUBF + 9'd5;    // subtract path
	localparam [8:0] UA_SUBX  = UA_SUBF + 9'd15;   // signs differ: to the add path
	localparam [8:0] UA_SUBZ  = UA_SUBF + 9'd20;   // zero dispatch, 4-aligned
	localparam [8:0] UA_MULF  = 9'd272;
	localparam [8:0] UA_MULZ  = UA_MULF + 9'd16;   // zero dispatch, 4-aligned
	localparam [8:0] UA_DIVF  = 9'd296;
	localparam [8:0] UA_DIVZ  = UA_DIVF + 9'd36;   // zero dispatch, 4-aligned

	// Words that do nothing, ending the routine or not.
	localparam [UW_BITS-1:0] UW_WAIT = {UW_BITS{1'b0}};
	localparam [UW_BITS-1:0] UW_ENDW = {1'b1, {(UW_BITS - 1){1'b0}}};

	// An operation on a and b into dst, then the next word.
	function [UW_BITS-1:0] uw_fn;
		input [5:0] op;
		input [2:0] a;
		input [2:0] b;
		input [2:0] dst;
		input [4:0] imm;
		begin
			uw_fn = {1'b0, 9'd0, C_NEXT, imm, dst, b, a, op};
		end
	endfunction

	// An operation with only imm: bus, bit-string cursor unit, hold.
	function [UW_BITS-1:0] uop_fn;
		input [5:0] op;
		input [4:0] imm;
		begin
			uop_fn = uw_fn(op, A_ZERO, B_ZERO, D_NONE, imm);
		end
	endfunction

	// Adds a jump on a condition.
	function [UW_BITS-1:0] ujmp_fn;
		input [UW_BITS-1:0] w;
		input [4:0]         cond;
		input [8:0]         target;
		begin
			ujmp_fn = w;
			ujmp_fn[UW_COND +: 5] = cond;
			ujmp_fn[UW_NEXT +: 9] = target;
		end
	endfunction

	// FP END word: how CY/OV/S/Z are formed and which trap, if any.
	function [UW_BITS-1:0] uend_fn;
		input [1:0] fm;
		input [2:0] fx;
		begin
			uend_fn = UW_ENDW;
			uend_fn[UW_IMM +: 5] = {fx, fm};
		end
	endfunction

	// Decoder PLA: {has a routine, entry address}. A divide by zero takes its
	// own entry. The uPD70732 has no NVC entries. TRNC.SW shares CVT.SW's
	// routine and branches on its opcode bit.
	function [9:0] upla_fn;
		input [5:0] ue;
		input       div0;
		/* verilator lint_off UNUSEDSIGNAL */
		input [5:0] sub;
		/* verilator lint_on UNUSEDSIGNAL */
		input [2:0] bs;       // bit-string bits {20:19, 16}
		begin
			case (ue)
				UADDR_XB:    upla_fn = {VB_EXT != 0, UA_XB};
				UADDR_REV:   upla_fn = {VB_EXT != 0, UA_REV};
				UADDR_MPYHW: upla_fn = {VB_EXT != 0, UA_MPYHW};
				UADDR_CLI:   upla_fn = {VB_EXT != 0, UA_CLI};
				UADDR_SEI:   upla_fn = {VB_EXT != 0, UA_SEI};
				UADDR_ILL:   upla_fn = {1'b1, UA_ILL};
				UADDR_CAXI:  upla_fn = {1'b1, UA_CAXI};
				UADDR_BITSTR: upla_fn = {1'b1, (bs[2:1] != 2'b00) ? UA_BSA : bs[0] ? UA_BSD : UA_BSU};
				UADDR_DIV,
				UADDR_DIVU:  upla_fn = {div0, UA_DIV0};
				UADDR_CMPF:  upla_fn = {1'b1, UA_CMPF};
				UADDR_CVTWS: upla_fn = {1'b1, UA_CVTWS};
				UADDR_CVTSW: upla_fn = {1'b1, UA_CVTSW};
				UADDR_FPU: begin
					case (sub)
						SUBOP_ADDF: upla_fn = {1'b1, UA_ADDF};
						SUBOP_SUBF: upla_fn = {1'b1, UA_SUBF};
						SUBOP_MULF: upla_fn = {1'b1, UA_MULF};
						SUBOP_DIVF: upla_fn = {1'b1, UA_DIVF};
						default:    upla_fn = 10'd0;
					endcase
				end
				default:     upla_fn = 10'd0;
			endcase
		end
	endfunction

	function [UW_BITS-1:0] urom_fn;
		input [8:0] addr;
		begin
			if (addr[8:7] == 2'd0) urom_fn = urom_lo_fn(addr);
			else if (addr[8:7] == 2'd3) urom_fn = urom_bs_fn(addr);
			else urom_fn = urom_fp_fn(addr);
		end
	endfunction

	function [UW_BITS-1:0] urom_bs_fn;
		input [8:0] addr;
		begin
			case (addr)
				// Arithmetic bit strings. Read r26-r30; a length 0 writes them
				// back. Read the first source word, with the second when the
				// source offset is the higher. Each destination word: read it,
				// read the next source word unless this is the last, and write
				// it once both are in, stepping the cursors; the step is the
				// interrupt point. The end waits for the last write.
				UA_BSA + 9'd0:    urom_bs_fn = uop_fn(O_LD, 5'd28);
				UA_BSA + 9'd1:    urom_bs_fn = uop_fn(O_LD, 5'd27);
				UA_BSA + 9'd2:    urom_bs_fn = uop_fn(O_LD, 5'd26);
				UA_BSA + 9'd3:    urom_bs_fn = uop_fn(O_LD, 5'd29);
				UA_BSA + 9'd4:    urom_bs_fn = uop_fn(O_LD, 5'd30);
				UA_BSA + 9'd5:    urom_bs_fn = ujmp_fn(UW_WAIT, C_LEN0, UA_BSA_X1);
				UA_BSA + 9'd6:    urom_bs_fn = uop_fn(O_COUNT, BSA_SETUP);
				UA_BSA + 9'd7:    urom_bs_fn = ujmp_fn(UW_WAIT, C_SGT, UA_BSA_GT);
				UA_BSA + 9'd8:    urom_bs_fn = ujmp_fn(UW_WAIT, C_SLT, UA_BSA_LT);
				UA_BSA + 9'd9:    urom_bs_fn = uop_fn(O_RS, 5'd0);
				UA_BSA_F + 9'd0:  urom_bs_fn = uop_fn(O_RD, 5'd0);
				UA_BSA_F + 9'd1:  urom_bs_fn = uop_fn(O_RSNW, 5'd0);
				UA_BSA_F + 9'd2:  urom_bs_fn = ujmp_fn(uop_fn(O_WR, 5'd0), C_STOP, UA_BSA_X1);
				// The second word, entered on an idle bus.
				UA_BSA_F + 9'd3:  urom_bs_fn = uop_fn(O_WAIT, 5'd0);
				UA_BSA_F + 9'd4:  urom_bs_fn = uop_fn(O_COUNT, BSA_W2_T);
				UA_BSA_F + 9'd5:  urom_bs_fn = uop_fn(O_RD, 5'd0);
				UA_BSA_F + 9'd6:  urom_bs_fn = uop_fn(O_RSNW, 5'd0);
				UA_BSA_F + 9'd7:  urom_bs_fn = ujmp_fn(uop_fn(O_WR, 5'd0), C_STOP, UA_BSA_X2);
				// The third word, a clock later.
				UA_BSA_F + 9'd8:  urom_bs_fn = uop_fn(O_WAIT, 5'd0);
				UA_BSA_F + 9'd9:  urom_bs_fn = uop_fn(O_COUNT, 5'd0);
				UA_BSA_L + 9'd0:  urom_bs_fn = uop_fn(O_RD, 5'd0);
				UA_BSA_L + 9'd1:  urom_bs_fn = uop_fn(O_RSNW, 5'd0);
				UA_BSA_L + 9'd2:  urom_bs_fn = ujmp_fn(uop_fn(O_WR, 5'd0), C_MORE, UA_BSA_L);
				// Two words or more: a partial last word costs a clock.
				UA_BSA_X2 + 9'd0: urom_bs_fn = ujmp_fn(uop_fn(O_WBW, 5'd30), C_FULL, UA_BSA_X2 + 9'd2);
				UA_BSA_X2 + 9'd1: urom_bs_fn = uop_fn(O_COUNT, 5'd0);
				UA_BSA_X2 + 9'd2: urom_bs_fn = uop_fn(O_WB, 5'd29);
				UA_BSA_X2 + 9'd3: urom_bs_fn = uop_fn(O_WB, 5'd28);
				UA_BSA_X2 + 9'd4: urom_bs_fn = uop_fn(O_WB, 5'd27);
				UA_BSA_X2 + 9'd5: urom_bs_fn = uop_fn(O_WB, 5'd26);
				UA_BSA_X2 + 9'd6: urom_bs_fn = UW_ENDW;
				UA_BSA_X1:        urom_bs_fn = ujmp_fn(uop_fn(O_WBW, 5'd30), C_ALW, UA_BSA_X2 + 9'd2);
				// The second source word, then the window's shift set-up.
				UA_BSA_GT + 9'd0: urom_bs_fn = uop_fn(O_RS, 5'd0);
				UA_BSA_GT + 9'd1: urom_bs_fn = uop_fn(O_RS, 5'd0);
				UA_BSA_GT + 9'd2: urom_bs_fn = uop_fn(O_WAIT, 5'd0);
				UA_BSA_GT + 9'd3: urom_bs_fn = ujmp_fn(uop_fn(O_COUNT, BSA_GT_T), C_ALW, UA_BSA_F);
				UA_BSA_LT + 9'd0: urom_bs_fn = uop_fn(O_RS, 5'd0);
				UA_BSA_LT + 9'd1: urom_bs_fn = uop_fn(O_WAIT, 5'd0);
				UA_BSA_LT + 9'd2: urom_bs_fn = ujmp_fn(uop_fn(O_COUNT, 5'd0), C_ALW, UA_BSA_F);

				// Searches, up and down. Read r27-r30 and set up; then one source
				// word at a time, the next read while this one is scanned. A first
				// word that starts inside it is read and scanned alone, and only
				// then is the rest set up. A hit or the end writes back r27-r30; so
				// does an interrupt at a word boundary.
				UA_BSU + 9'd0:    urom_bs_fn = uop_fn(O_LD, 5'd28);
				UA_BSU + 9'd1:    urom_bs_fn = uop_fn(O_LD, 5'd27);
				UA_BSU + 9'd2:    urom_bs_fn = uop_fn(O_LD, 5'd29);
				UA_BSU + 9'd3:    urom_bs_fn = uop_fn(O_LD, 5'd30);
				UA_BSU + 9'd4:    urom_bs_fn = ujmp_fn(uop_fn(O_COUNT, 5'd2), C_LEN0, UA_BSS_WB);
				UA_BSU + 9'd5:    urom_bs_fn = ujmp_fn(UW_WAIT, C_LAST, UA_BSU + 9'd8);
				UA_BSU + 9'd6:    urom_bs_fn = ujmp_fn(UW_WAIT, C_FPART, UA_BSU + 9'd9);
				UA_BSU + 9'd7:    urom_bs_fn = ujmp_fn(uop_fn(O_COUNT, BSU_MULTI), C_ALW, UA_BSS_C);
				UA_BSU + 9'd8:    urom_bs_fn = ujmp_fn(uop_fn(O_COUNT, BSU_ONE), C_ALW, UA_BSS_C);
				UA_BSU + 9'd9:    urom_bs_fn = ujmp_fn(uop_fn(O_COUNT, BSU_ONE), C_ALW, UA_BSS_P);
				UA_BSD + 9'd0:    urom_bs_fn = uop_fn(O_LD, 5'd28);
				UA_BSD + 9'd1:    urom_bs_fn = uop_fn(O_LD, 5'd27);
				UA_BSD + 9'd2:    urom_bs_fn = uop_fn(O_LD, 5'd29);
				UA_BSD + 9'd3:    urom_bs_fn = uop_fn(O_LD, 5'd30);
				UA_BSD + 9'd4:    urom_bs_fn = ujmp_fn(uop_fn(O_COUNT, 5'd4), C_LEN0, UA_BSS_WB);
				UA_BSD + 9'd5:    urom_bs_fn = ujmp_fn(UW_WAIT, C_LAST, UA_BSS_C);
				UA_BSD + 9'd6:    urom_bs_fn = ujmp_fn(UW_WAIT, C_FPART, UA_BSS_P);
				UA_BSD + 9'd7:    urom_bs_fn = ujmp_fn(uop_fn(O_COUNT, BSD_MULTI), C_ALW, UA_BSS_C);
				// A search for 0 takes a clock more.
				UA_BSS_C + 9'd0:  urom_bs_fn = ujmp_fn(UW_WAIT, C_SCH0, UA_BSS_C0);
				UA_BSS_C + 9'd1:  urom_bs_fn = uop_fn(O_RS, 5'd0);
				UA_BSS_L + 9'd0:  urom_bs_fn = uop_fn(O_WAIT, 5'd0);
				UA_BSS_L + 9'd1:  urom_bs_fn = uop_fn(O_RSN, 5'd0);
				UA_BSS_L + 9'd2:  urom_bs_fn = uop_fn(O_SCAN, 5'd0);
				UA_BSS_L + 9'd3:  urom_bs_fn = ujmp_fn(UW_WAIT, C_MORE, UA_BSS_L);
				UA_BSS_X + 9'd0:  urom_bs_fn = ujmp_fn(UW_WAIT, C_FOUND, UA_BSS_F);
				UA_BSS_X + 9'd1:  urom_bs_fn = ujmp_fn(UW_WAIT, C_ABORT, UA_BSS_WB);
				UA_BSS_X + 9'd2:  urom_bs_fn = uop_fn(O_COUNT, BSS_MISS);
				UA_BSS_WB + 9'd0: urom_bs_fn = uop_fn(O_WB, 5'd30);
				UA_BSS_WB + 9'd1: urom_bs_fn = uop_fn(O_WB, 5'd29);
				UA_BSS_WB + 9'd2: urom_bs_fn = uop_fn(O_WB, 5'd28);
				UA_BSS_WB + 9'd3: urom_bs_fn = uop_fn(O_WB, 5'd27);
				UA_BSS_WB + 9'd4: urom_bs_fn = UW_ENDW;
				UA_BSS_F:         urom_bs_fn = ujmp_fn(uop_fn(O_COUNT, BSS_FOUND), C_ALW, UA_BSS_WB);
				UA_BSS_C0:        urom_bs_fn = ujmp_fn(uop_fn(O_COUNT, 5'd0), C_ALW, UA_BSS_C + 9'd1);
				// The partial first word: read, scan, then set up the rest.
				UA_BSS_P + 9'd0:  urom_bs_fn = ujmp_fn(UW_WAIT, C_SCH0, UA_BSS_P0);
				UA_BSS_P + 9'd1:  urom_bs_fn = uop_fn(O_RS, 5'd0);
				UA_BSS_P + 9'd2:  urom_bs_fn = uop_fn(O_WAIT, 5'd0);
				UA_BSS_P + 9'd3:  urom_bs_fn = uop_fn(O_SCAN, 5'd0);
				UA_BSS_P + 9'd4:  urom_bs_fn = ujmp_fn(UW_WAIT, C_STOP, UA_BSS_X);
				UA_BSS_P + 9'd5:  urom_bs_fn = ujmp_fn(UW_WAIT, C_DOWN, UA_BSS_P + 9'd8);
				UA_BSS_P + 9'd6:  urom_bs_fn = ujmp_fn(uop_fn(O_COUNT, BSU_REST), C_ALW, UA_BSS_P + 9'd9);
				UA_BSS_P + 9'd7:  urom_bs_fn = UW_ENDW;
				UA_BSS_P + 9'd8:  urom_bs_fn = uop_fn(O_COUNT, BSD_REST);
				UA_BSS_P + 9'd9:  urom_bs_fn = ujmp_fn(uop_fn(O_RS, 5'd0), C_ALW, UA_BSS_L);
				UA_BSS_P0:        urom_bs_fn = ujmp_fn(uop_fn(O_COUNT, 5'd0), C_ALW, UA_BSS_P + 9'd1);

				default:          urom_bs_fn = UW_ENDW;
			endcase
		end
	endfunction

	function [UW_BITS-1:0] urom_lo_fn;
		input [8:0] addr;
		begin
			case (addr)
				// XB: swap the low two bytes of reg2.
				UA_XB + 9'd0:     urom_lo_fn = uw_fn(O_FSR,   A_R2,   B_R2,   D_T0,   5'd16); // rot 16
				UA_XB + 9'd1:     urom_lo_fn = uw_fn(O_FSR,   A_R2,   B_T0,   D_T1,   5'd8);
				UA_XB + 9'd2:     urom_lo_fn = uw_fn(O_FSR,   A_T0,   B_T1,   D_R2,   5'd16);
				UA_XB + 9'd3:     urom_lo_fn = UW_ENDW;

				// REV: reg1 bit-reversed into reg2. Swap bits, pairs and nibbles,
				// then reverse the bytes.
				UA_REV + 9'd0:    urom_lo_fn = uw_fn(O_FSR,   A_R1,   B_ZERO, D_T1,   5'd31); // shl 1
				UA_REV + 9'd1:    urom_lo_fn = uw_fn(O_FSR,   A_ZERO, B_R1,   D_T0,   5'd1); // shr 1
				UA_REV + 9'd2:    urom_lo_fn = uw_fn(O_AND,   A_T1,   B_K,    D_T1,   K_HI1);
				UA_REV + 9'd3:    urom_lo_fn = uw_fn(O_AND,   A_T0,   B_K,    D_T0,   K_LO1);
				UA_REV + 9'd4:    urom_lo_fn = uw_fn(O_OR,    A_T0,   B_T1,   D_T0,   5'd0);
				UA_REV + 9'd5:    urom_lo_fn = uw_fn(O_FSR,   A_T0,   B_ZERO, D_T1,   5'd30); // shl 2
				UA_REV + 9'd6:    urom_lo_fn = uw_fn(O_FSR,   A_ZERO, B_T0,   D_T0,   5'd2); // shr 2
				UA_REV + 9'd7:    urom_lo_fn = uw_fn(O_AND,   A_T1,   B_K,    D_T1,   K_HI2);
				UA_REV + 9'd8:    urom_lo_fn = uw_fn(O_AND,   A_T0,   B_K,    D_T0,   K_LO2);
				UA_REV + 9'd9:    urom_lo_fn = uw_fn(O_OR,    A_T0,   B_T1,   D_T0,   5'd0);
				UA_REV + 9'd10:   urom_lo_fn = uw_fn(O_FSR,   A_T0,   B_ZERO, D_T1,   5'd28); // shl 4
				UA_REV + 9'd11:   urom_lo_fn = uw_fn(O_FSR,   A_ZERO, B_T0,   D_T0,   5'd4); // shr 4
				UA_REV + 9'd12:   urom_lo_fn = uw_fn(O_AND,   A_T1,   B_K,    D_T1,   K_HI4);
				UA_REV + 9'd13:   urom_lo_fn = uw_fn(O_AND,   A_T0,   B_K,    D_T0,   K_LO4);
				UA_REV + 9'd14:   urom_lo_fn = uw_fn(O_OR,    A_T0,   B_T1,   D_T0,   5'd0);
				UA_REV + 9'd15:   urom_lo_fn = uw_fn(O_FSR,   A_T0,   B_ZERO, D_T1,   5'd16); // shl 16
				UA_REV + 9'd16:   urom_lo_fn = uw_fn(O_FSR,   A_T0,   B_T1,   D_T1,   5'd24);
				UA_REV + 9'd17:   urom_lo_fn = uw_fn(O_FSR,   A_T1,   B_T1,   D_T1,   5'd24); // rot 24
				UA_REV + 9'd18:   urom_lo_fn = uw_fn(O_FSR,   A_T1,   B_T0,   D_R2,   5'd24);
				UA_REV + 9'd19:   urom_lo_fn = UW_ENDW;

				// MPYHW: reg2 times the signed 17 bits of reg1, low word. Four
				// digits of reg1, then the sign bit, which is left in T1[0].
				UA_MPYHW + 9'd0:  urom_lo_fn = uw_fn(O_MSTEP, A_ZERO, B_R1,   D_T0,   5'd0);
				UA_MPYHW + 9'd1:  urom_lo_fn = uw_fn(O_MSTEP, A_T0,   B_T1,   D_T0,   5'd0);
				UA_MPYHW + 9'd2:  urom_lo_fn = uw_fn(O_MSTEP, A_T0,   B_T1,   D_T0,   5'd0);
				UA_MPYHW + 9'd3:  urom_lo_fn = uw_fn(O_MSTEP, A_T0,   B_T1,   D_T0,   5'd0);
				UA_MPYHW + 9'd4:  urom_lo_fn = uw_fn(O_MSIGN, A_T0,   B_T1,   D_T0,   5'd0);
				UA_MPYHW + 9'd5:  urom_lo_fn = uw_fn(O_FSR,   A_T0,   B_T1,   D_R2,   5'd16);
				UA_MPYHW + 9'd6:  urom_lo_fn = UW_ENDW;

				// CLI and SEI: PSW.ID through a temporary, then the system
				// register write tail that LDSR has.
				UA_CLI + 9'd0:    urom_lo_fn = uw_fn(O_PASS,  A_PSW,  B_ZERO, D_T0,   5'd0);
				UA_CLI + 9'd1:    urom_lo_fn = ujmp_fn(uw_fn(O_AND,   A_T0,   B_K,    D_PSW,  K_NOID), C_ALW, UA_SYNC);
				UA_SEI + 9'd0:    urom_lo_fn = uw_fn(O_PASS,  A_PSW,  B_ZERO, D_T0,   5'd0);
				UA_SEI + 9'd1:    urom_lo_fn = ujmp_fn(uw_fn(O_OR,    A_T0,   B_K,    D_PSW,  K_ID), C_ALW, UA_SYNC);
				UA_SYNC + 9'd0,
				UA_SYNC + 9'd1,
				UA_SYNC + 9'd2,
				UA_SYNC + 9'd3,
				UA_SYNC + 9'd4,
				UA_SYNC + 9'd5,
				UA_SYNC + 9'd6:   urom_lo_fn = UW_WAIT;
				UA_SYNC + 9'd7:   urom_lo_fn = UW_ENDW;

				// Exception entry, shared by interrupts, NMI, the address trap,
				// TRAP and the faults. Three steps stop the pipe and wait for older
				// work to leave MEM and WB; then 5 steps: restore PC and PSW saved,
				// ECR and PSW written, vector chosen; the fifth waits for posted stores
				// to leave the write buffer and the bus. The pipe makes these writes and
				// redirects on the clock after the last. An interrupt taken with the
				// pipe already stopped starts at the fourth. A HALT wake runs them all.
				UA_ENTRY + 9'd0,
				UA_ENTRY + 9'd1:  urom_lo_fn = UW_WAIT;
				UA_ENTRY + 9'd2:  urom_lo_fn = uop_fn(O_SETTLE, 5'd0);
				UA_ENTRY + 9'd3:  urom_lo_fn = UW_WAIT;
				UA_ENTRY + 9'd4:  urom_lo_fn = uop_fn(O_SETTLE, 5'd1);
				UA_ENTRY + 9'd5,
				UA_ENTRY + 9'd6:  urom_lo_fn = UW_WAIT;
				UA_ENTRY + 9'd7:  urom_lo_fn = UW_ENDW;

				// Illegal opcode and divide by zero, before the entry: TRAP + 4
				// and TRAP + 10. The divide shares the illegal opcode's words.
				UA_ILL + 9'd0,
				UA_ILL + 9'd1,
				UA_ILL + 9'd2:    urom_lo_fn = UW_WAIT;
				UA_ILL + 9'd3:    urom_lo_fn = UW_ENDW;
				UA_DIV0 + 9'd0,
				UA_DIV0 + 9'd1,
				UA_DIV0 + 9'd2,
				UA_DIV0 + 9'd3,
				UA_DIV0 + 9'd4:   urom_lo_fn = UW_WAIT;
				UA_DIV0 + 9'd5:   urom_lo_fn = ujmp_fn(uw_fn(O_NOP,   A_ZERO, B_ZERO, D_NONE, 5'd0), C_ALW, UA_ILL);

				// CAXI: once older work is done, a locked read of [reg1 + disp], compared
				// with reg2; then it writes r30 back on a match, or the old value, a
				// clock later, on a mismatch. reg2 gets the old value.
				UA_CAXI + 9'd0:   urom_lo_fn = uop_fn(O_DRAIN, 5'd0);
				UA_CAXI + 9'd1:   urom_lo_fn = uop_fn(O_READ, 5'd0);
				UA_CAXI + 9'd2:   urom_lo_fn = uw_fn(O_PASS,  A_R30,  B_ZERO, D_T1,   5'd0);
				UA_CAXI + 9'd3:   urom_lo_fn = uop_fn(O_WAIT, 5'd0);
				UA_CAXI + 9'd4:   urom_lo_fn = uw_fn(O_SUB,   A_R2,   B_T0,   D_NONE, 5'd0);
				UA_CAXI + 9'd5:   urom_lo_fn = ujmp_fn(UW_WAIT, C_EQ, UA_CAXI_W);
				UA_CAXI + 9'd6:   urom_lo_fn = uw_fn(O_PASS,  A_T0,   B_ZERO, D_T1,   5'd0);
				UA_CAXI_W:        urom_lo_fn = uop_fn(O_WRITE, 5'd0);
				// Internal steps, under which the posted write runs.
				UA_CAXI_W + 9'd1: urom_lo_fn = uop_fn(O_COUNT, CAXI_STEPS);
				UA_CAXI_W + 9'd2: urom_lo_fn = uop_fn(O_UNLOCK, 5'd0);
				UA_CAXI_W + 9'd3: urom_lo_fn = uw_fn(O_PASS,  A_T0,   B_ZERO, D_R2,   5'd0);
				UA_CAXI_W + 9'd4: urom_lo_fn = UW_ENDW;

				default:          urom_lo_fn = UW_ENDW;
			endcase
		end
	endfunction

	function [UW_BITS-1:0] urom_fp_fn;
		input [8:0] addr;
		begin
			case (addr)
				// FP exits. FRO, FIV and FZD leave from the class test, FOV from
				// the tail's pack; each trap holds 8 clocks before its END. The
				// wait chain pads the zero and cancel exits to their lengths.
				UA_FRO,
				UA_FIV,
				UA_FOV:           urom_fp_fn = uop_fn(O_COUNT, FX_HOLD);
				UA_FRO + 9'd1:    urom_fp_fn = uend_fn(FM_NONE, FX_FRO);
				UA_FIV + 9'd1:    urom_fp_fn = uend_fn(FM_NONE, FX_FIV);
				UA_FZD:           urom_fp_fn = uend_fn(FM_NONE, FX_FZD);
				UA_FOV + 9'd1:    urom_fp_fn = uend_fn(FM_FP, FX_FOV);
				UA_FW + 9'd0,
				UA_FW + 9'd1,
				UA_FW + 9'd2,
				UA_FW + 9'd3,
				UA_FW + 9'd4,
				UA_FW + 9'd5:     urom_fp_fn = UW_WAIT;
				UA_FW + 9'd6:     urom_fp_fn = uend_fn(FM_FP, FX_NONE);

				// Tail for a mantissa in T0 with its leading one at bit 26 or 27 or
				// below, bit 0 sticky, exponent in E and sign in SG: normalise,
				// round to nearest even, pack, then overflow and underflow. FOV
				// leaves two words before the normal END.
				UA_TAIL + 9'd0:   urom_fp_fn = uw_fn(E_LZ,    A_T0,   B_ZERO, D_NONE, 5'd0);
				UA_TAIL + 9'd1:   urom_fp_fn = uw_fn(F_NORM,  A_T0,   B_ZERO, D_T0,   5'd0);
				UA_TAIL + 9'd2:   urom_fp_fn = uw_fn(F_RND,   A_T0,   B_ZERO, D_T0,   5'd0);
				UA_TAIL + 9'd3:   urom_fp_fn = uw_fn(E_RND,   A_T0,   B_ZERO, D_NONE, 5'd0);
				UA_TAIL + 9'd4:   urom_fp_fn = uw_fn(F_MANT,  A_T0,   B_ZERO, D_T0,   5'd0);
				UA_TAIL + 9'd5:   urom_fp_fn = ujmp_fn(uw_fn(O_OR,    A_T0,   B_PK,   D_R2,   5'd0), C_OVF, UA_FOV);
				UA_TAIL + 9'd6:   urom_fp_fn = ujmp_fn(UW_WAIT, C_UFL, UA_UFL);
				UA_TAIL + 9'd7:   urom_fp_fn = UW_WAIT;
				UA_TAIL + 9'd8:   urom_fp_fn = uend_fn(FM_FP, FX_NONE);
				// Underflow stores signed zero with FUD and FPR in 2 more clocks, a
				// clock less when FPR is already set. DIVF keeps FPR only for an
				// inexact quotient and takes 2 more clocks, 1 fewer when inexact.
				UA_UFL + 9'd0:    urom_fp_fn = ujmp_fn(uw_fn(O_AND,   A_PK,   B_K,    D_R2,   K_SIGN), C_ALT, UA_UFLD);
				UA_UFL + 9'd1:    urom_fp_fn = ujmp_fn(uw_fn(X_FUD,   A_ZERO, B_ZERO, D_NONE, 5'd0), C_FPR, UA_UFL + 9'd3);
				UA_UFL + 9'd2:    urom_fp_fn = uw_fn(X_FPR,   A_ZERO, B_ZERO, D_NONE, 5'd0);
				UA_UFL + 9'd3:    urom_fp_fn = uend_fn(FM_FP, FX_NONE);
				UA_UFLD + 9'd0:   urom_fp_fn = ujmp_fn(uw_fn(X_FUD,   A_ZERO, B_ZERO, D_NONE, 5'd0), C_FPR, UA_UFLD + 9'd2);
				UA_UFLD + 9'd1,
				UA_UFLD + 9'd2,
				UA_UFLD + 9'd3:   urom_fp_fn = UW_WAIT;
				UA_UFLD + 9'd4:   urom_fp_fn = uend_fn(FM_FP, FX_NONE);

				// CMPF: reg2 - reg1 as signed magnitudes; CY = S = less, Z = equal,
				// so +0 equals -0. 7 clocks without a zero; a zero operand holds
				// before the subtract: 0 vs x 8, x vs 0 10, 0 vs 0 11.
				UA_CMPF + 9'd0:   urom_fp_fn = ujmp_fn(uw_fn(F_SMTC,  A_R2,   B_ZERO, D_T0,   5'd0), C_RSV, UA_FRO);
				UA_CMPF + 9'd1:   urom_fp_fn = ujmp_fn(uw_fn(F_SMTC,  A_R1,   B_ZERO, D_T1,   5'd0), C_ZANY, UA_CMPZ);
				UA_CMPF + 9'd2:   urom_fp_fn = uw_fn(O_SUB,   A_T0,   B_T1,   D_NONE, 5'd0);
				UA_CMPF + 9'd3:   urom_fp_fn = UW_WAIT;
				UA_CMPF + 9'd4:   urom_fp_fn = uend_fn(FM_CMP, FX_NONE);
				UA_CMPZ + 9'd1:   urom_fp_fn = ujmp_fn(uop_fn(O_COUNT, 5'd2), C_ALW, UA_CMPF + 9'd2);
				UA_CMPZ + 9'd2:   urom_fp_fn = ujmp_fn(UW_WAIT, C_ALW, UA_CMPF + 9'd2);
				UA_CMPZ + 9'd3:   urom_fp_fn = ujmp_fn(uop_fn(O_COUNT, 5'd3), C_ALW, UA_CMPF + 9'd2);

				// CVT.WS: |reg1| normalised to bit 26 with E = 158 - lz. Below 2^24
				// it is exact; from 2^24 it rounds through the tail. 0 / below 2^24
				// / from 2^24: 5 / 8 / 16 clocks.
				UA_CVTWS + 9'd0:  urom_fp_fn = ujmp_fn(uw_fn(F_ABSS,  A_R1,   B_ZERO, D_T0,   5'd0),
					C_IZ1, UA_CVTWS + 9'd6);
				UA_CVTWS + 9'd1:  urom_fp_fn = ujmp_fn(uw_fn(E_LZK,   A_T0,   B_ZERO, D_NONE, K_WSEXP),
					C_BIG, UA_CVTWS + 9'd8);
				UA_CVTWS + 9'd2:  urom_fp_fn = uw_fn(F_NORM,  A_T0,   B_ZERO, D_T0,   5'd0);
				UA_CVTWS + 9'd3:  urom_fp_fn = uw_fn(F_MANT,  A_T0,   B_ZERO, D_T0,   5'd0);
				UA_CVTWS + 9'd4:  urom_fp_fn = uw_fn(O_OR,    A_T0,   B_PK,   D_R2,   5'd0);
				UA_CVTWS + 9'd5:  urom_fp_fn = uend_fn(FM_FP, FX_NONE);
				UA_CVTWS + 9'd6:  urom_fp_fn = uw_fn(O_PASS,  A_ZERO, B_ZERO, D_R2,   5'd0);
				UA_CVTWS + 9'd7:  urom_fp_fn = uend_fn(FM_FP, FX_NONE);
				UA_CVTWS + 9'd8,
				UA_CVTWS + 9'd9,
				UA_CVTWS + 9'd10: urom_fp_fn = UW_WAIT;
				UA_CVTWS + 9'd11: urom_fp_fn = ujmp_fn(UW_WAIT, C_ALW, UA_TAIL + 9'd1);

				// CVT.SW and TRNC.SW: reg1's mantissa << 3, shifted by 150 - exponent
				// (fraction: guard bits kept, then rounded or truncated) or by
				// 153 - exponent (integral). From 2^31 FIV, except -2^31. Integral /
				// zero / fraction: 8 / 9 / 14 (TRNC 13) clocks; FIV at +5 / +8
				// over FRO.
				UA_CVTSW + 9'd0:  urom_fp_fn = ujmp_fn(uw_fn(F_UNPK,  A_R1,   B_ZERO, D_T0,   5'd0), C_RSV1, UA_FRO);
				UA_CVTSW + 9'd1:  urom_fp_fn = ujmp_fn(uw_fn(S_S1,    A_ZERO, B_ZERO, D_NONE, 5'd0),
					C_INT, UA_CVTSW + 9'd17);
				UA_CVTSW + 9'd2:  urom_fp_fn = ujmp_fn(uw_fn(E_SHK,   A_ZERO, B_ZERO, D_NONE, K_SWFRAC),
					C_Z1, UA_CVTSW + 9'd13);
				UA_CVTSW + 9'd3:  urom_fp_fn = uw_fn(F_NORM,  A_T0,   B_ZERO, D_T0,   5'd0);
				UA_CVTSW + 9'd4:  urom_fp_fn = ujmp_fn(UW_WAIT, C_ALT, UA_CVTSW + 9'd12);
				UA_CVTSW + 9'd5:  urom_fp_fn = uw_fn(F_RND,   A_T0,   B_ZERO, D_T0,   5'd0);
				UA_CVTSW + 9'd6,
				UA_CVTSW + 9'd7,
				UA_CVTSW + 9'd8:  urom_fp_fn = UW_WAIT;
				UA_CVTSW + 9'd9:  urom_fp_fn = uw_fn(O_FSR,   A_ZERO, B_T0,   D_T0,   5'd3);
				UA_CVTSW + 9'd10: urom_fp_fn = uw_fn(F_CNEG,  A_T0,   B_ZERO, D_R2,   5'd0);
				UA_CVTSW + 9'd11: urom_fp_fn = uend_fn(FM_INT, FX_NONE);
				UA_CVTSW + 9'd12: urom_fp_fn = ujmp_fn(uw_fn(F_INEX,  A_T0,   B_ZERO, D_T0,   5'd0),
					C_ALW, UA_CVTSW + 9'd7);
				UA_CVTSW + 9'd13: urom_fp_fn = uw_fn(O_PASS,  A_ZERO, B_ZERO, D_R2,   5'd0);
				UA_CVTSW + 9'd14,
				UA_CVTSW + 9'd15: urom_fp_fn = UW_WAIT;
				UA_CVTSW + 9'd16: urom_fp_fn = uend_fn(FM_INT, FX_NONE);
				UA_CVTSW + 9'd17: urom_fp_fn = ujmp_fn(uw_fn(E_SHK,   A_ZERO, B_ZERO, D_NONE, K_SWINT),
					C_E158, UA_CVTSW + 9'd21);
				UA_CVTSW + 9'd18: urom_fp_fn = uw_fn(F_NORM,  A_T0,   B_ZERO, D_T0,   5'd0);
				UA_CVTSW + 9'd19: urom_fp_fn = uw_fn(F_CNEG,  A_T0,   B_ZERO, D_R2,   5'd0);
				UA_CVTSW + 9'd20: urom_fp_fn = uend_fn(FM_INT, FX_NONE);
				UA_CVTSW + 9'd21: urom_fp_fn = ujmp_fn(UW_WAIT, C_S1, UA_CVTSW + 9'd25);
				UA_CVTSW + 9'd22: urom_fp_fn = UW_WAIT;
				UA_CVTSW + 9'd23: urom_fp_fn = ujmp_fn(UW_WAIT, C_ALW, UA_FIV);
				UA_CVTSW + 9'd25: urom_fp_fn = uw_fn(O_SUB,   A_R1,   B_K,    D_NONE, K_IMIN);
				UA_CVTSW + 9'd26: urom_fp_fn = ujmp_fn(UW_WAIT, C_EQ, UA_CVTSW + 9'd18);
				UA_CVTSW + 9'd27,
				UA_CVTSW + 9'd28: urom_fp_fn = UW_WAIT;
				UA_CVTSW + 9'd29: urom_fp_fn = ujmp_fn(UW_WAIT, C_ALW, UA_FIV);

				// ADDF and SUBF branch on the raw signs. Equal: ADDF adds, SUBF
				// subtracts. Differ: four words, then the other routine's path. Both
				// paths align the mantissa with the smaller exponent and add or
				// subtract; the difference is made positive and its sign fixed. 22 /
				// 26 / 28 clocks, underflow +2, FOV 6 later for SUBF. Zero operands
				// and an exact cancel leave early.
				// When the smaller operand falls wholly into the sticky bit, the add
				// path skips the larger one's shift by 0: one clock less.
				UA_ADDF + 9'd0:   urom_fp_fn = ujmp_fn(UW_WAIT, C_RSV, UA_FRO);
				UA_ADDF + 9'd1:   urom_fp_fn = ujmp_fn(UW_WAIT, C_ZANY, UA_ADDZ);
				UA_ADDF + 9'd2:   urom_fp_fn = ujmp_fn(UW_WAIT, C_SDIF, UA_ADDX);
				UA_ADDA + 9'd0:   urom_fp_fn = uw_fn(E_ALN2,  A_ZERO, B_ZERO, D_NONE, 5'd0);
				UA_ADDA + 9'd1:   urom_fp_fn = ujmp_fn(uw_fn(F_UNPK,  A_R2,   B_ZERO, D_T0,   5'd0), C_GAP, UA_ADDA + 9'd3);
				UA_ADDA + 9'd2:   urom_fp_fn = uw_fn(F_NORM,  A_T0,   B_ZERO, D_T0,   5'd0);
				UA_ADDA + 9'd3:   urom_fp_fn = uw_fn(E_ALN1,  A_ZERO, B_ZERO, D_NONE, 5'd0);
				UA_ADDA + 9'd4:   urom_fp_fn = ujmp_fn(uw_fn(F_UNPK,  A_R1,   B_ZERO, D_T1,   5'd0), C_GAP, UA_ADDA + 9'd6);
				UA_ADDA + 9'd5:   urom_fp_fn = uw_fn(F_NORM,  A_T1,   B_ZERO, D_T1,   5'd0);
				UA_ADDA + 9'd6:   urom_fp_fn = uw_fn(F_ADD,   A_T0,   B_T1,   D_T0,   5'd0);
				UA_ADDA + 9'd7:   urom_fp_fn = ujmp_fn(uw_fn(S_S2,    A_ZERO, B_ZERO, D_NONE, 5'd0), C_ALW, UA_TAIL);
				UA_ADDX + 9'd0:   urom_fp_fn = ujmp_fn(UW_WAIT, C_MEQ, UA_ADDZ + 9'd5);
				UA_ADDX + 9'd1,
				UA_ADDX + 9'd2:   urom_fp_fn = UW_WAIT;
				UA_ADDX + 9'd3:   urom_fp_fn = ujmp_fn(UW_WAIT, C_ALW, UA_SUBS);
				// x + 0, 0 + x, 0 + 0 (sign: both negative), x + -x (+0).
				UA_ADDZ + 9'd1:   urom_fp_fn = ujmp_fn(uw_fn(O_PASS,  A_R2,   B_ZERO, D_R2,   5'd0),
					C_ALW, UA_FW + 9'd3);
				UA_ADDZ + 9'd2:   urom_fp_fn = ujmp_fn(uw_fn(O_PASS,  A_R1,   B_ZERO, D_R2,   5'd0),
					C_ALW, UA_FW + 9'd3);
				UA_ADDZ + 9'd3:   urom_fp_fn = uw_fn(O_AND,   A_R2,   B_R1,   D_T0,   5'd0);
				UA_ADDZ + 9'd4:   urom_fp_fn = ujmp_fn(uw_fn(O_AND,   A_T0,   B_K,    D_R2,   K_SIGN),
					C_ALW, UA_FW + 9'd2);
				UA_ADDZ + 9'd5:   urom_fp_fn = ujmp_fn(uw_fn(O_PASS,  A_ZERO, B_ZERO, D_R2,   5'd0),
					C_ALW, UA_FW + 9'd3);

				// SUBF: T2 = -reg1 for the zero exits.
				UA_SUBF + 9'd0:   urom_fp_fn = ujmp_fn(UW_WAIT, C_RSV, UA_FRO);
				UA_SUBF + 9'd1:   urom_fp_fn = uw_fn(F_XOR,   A_R1,   B_K,    D_T2,   K_SIGN);
				UA_SUBF + 9'd2:   urom_fp_fn = ujmp_fn(UW_WAIT, C_ZANY, UA_SUBZ);
				UA_SUBF + 9'd3:   urom_fp_fn = UW_WAIT;
				UA_SUBF + 9'd4:   urom_fp_fn = ujmp_fn(UW_WAIT, C_SDIF, UA_SUBX);
				UA_SUBS + 9'd0:   urom_fp_fn = ujmp_fn(UW_WAIT, C_MEQ, UA_SUBZ + 9'd5);
				UA_SUBS + 9'd1:   urom_fp_fn = uw_fn(E_ALN2,  A_ZERO, B_ZERO, D_NONE, 5'd0);
				UA_SUBS + 9'd2:   urom_fp_fn = uw_fn(F_UNPK,  A_R2,   B_ZERO, D_T0,   5'd0);
				UA_SUBS + 9'd3:   urom_fp_fn = uw_fn(F_NORM,  A_T0,   B_ZERO, D_T0,   5'd0);
				UA_SUBS + 9'd4:   urom_fp_fn = uw_fn(E_ALN1,  A_ZERO, B_ZERO, D_NONE, 5'd0);
				UA_SUBS + 9'd5:   urom_fp_fn = uw_fn(F_UNPK,  A_R1,   B_ZERO, D_T1,   5'd0);
				UA_SUBS + 9'd6:   urom_fp_fn = uw_fn(F_NORM,  A_T1,   B_ZERO, D_T1,   5'd0);
				UA_SUBS + 9'd7:   urom_fp_fn = uw_fn(O_SUB,   A_T0,   B_T1,   D_T0,   5'd0);
				UA_SUBS + 9'd8:   urom_fp_fn = uw_fn(F_ABSS,  A_T0,   B_ZERO, D_T0,   5'd0);
				UA_SUBS + 9'd9:   urom_fp_fn = ujmp_fn(uw_fn(S_X2,    A_ZERO, B_ZERO, D_NONE, 5'd0), C_ALW, UA_TAIL);
				UA_SUBX + 9'd0,
				UA_SUBX + 9'd1,
				UA_SUBX + 9'd2:   urom_fp_fn = UW_WAIT;
				UA_SUBX + 9'd3:   urom_fp_fn = ujmp_fn(UW_WAIT, C_ALW, UA_ADDA);
				// x - 0, 0 - x, 0 - 0 (sign: reg2 negative, reg1 not), x - x (+0).
				UA_SUBZ + 9'd1:   urom_fp_fn = ujmp_fn(uw_fn(O_PASS,  A_R2,   B_ZERO, D_R2,   5'd0),
					C_ALW, UA_FW + 9'd1);
				UA_SUBZ + 9'd2:   urom_fp_fn = ujmp_fn(uw_fn(O_OR,    A_ZERO, B_T2,   D_R2,   5'd0), C_ALW, UA_FW);
				UA_SUBZ + 9'd3:   urom_fp_fn = uw_fn(O_AND,   A_R2,   B_T2,   D_T0,   5'd0);
				UA_SUBZ + 9'd4:   urom_fp_fn = ujmp_fn(uw_fn(O_AND,   A_T0,   B_K,    D_R2,   K_SIGN),
					C_ALW, UA_FW + 9'd1);
				UA_SUBZ + 9'd5:   urom_fp_fn = ujmp_fn(uw_fn(O_PASS,  A_ZERO, B_ZERO, D_R2,   5'd0),
					C_ALW, UA_FW + 9'd5);

				// MULF: the mantissas << 3 multiplied 4 bits a clock on the multiply
				// step, 8 steps; product bits 53:26 to the tail, the rest sticky. 26
				// clocks for every operand, underflow +2, or +1 from an
				// exponent sum already below range.
				UA_MULF + 9'd0:   urom_fp_fn = ujmp_fn(UW_WAIT, C_RSV, UA_FRO);
				UA_MULF + 9'd1:   urom_fp_fn = ujmp_fn(UW_WAIT, C_ZANY, UA_MULZ);
				UA_MULF + 9'd2:   urom_fp_fn = uw_fn(F_UNPK,  A_R2,   B_ZERO, D_T2,   5'd0);
				UA_MULF + 9'd3:   urom_fp_fn = uw_fn(F_UNPK,  A_R1,   B_ZERO, D_T1,   5'd0);
				UA_MULF + 9'd4:   urom_fp_fn = uw_fn(E_MUL,   A_ZERO, B_ZERO, D_NONE, K_MBIAS);
				UA_MULF + 9'd5:   urom_fp_fn = uw_fn(S_X12,   A_ZERO, B_ZERO, D_NONE, 5'd0);
				UA_MULF + 9'd6:   urom_fp_fn = uw_fn(F_MSTEP, A_ZERO, B_T1,   D_T0,   5'd0);
				UA_MULF + 9'd7,
				UA_MULF + 9'd8,
				UA_MULF + 9'd9,
				UA_MULF + 9'd10,
				UA_MULF + 9'd11,
				UA_MULF + 9'd12,
				UA_MULF + 9'd13:  urom_fp_fn = uw_fn(F_MSTEP, A_T0,   B_T1,   D_T0,   5'd0);
				UA_MULF + 9'd14:  urom_fp_fn = ujmp_fn(uw_fn(F_SHRK,  A_T0,   B_T1,   D_T0,   5'd26), C_ALW, UA_TAIL);
				UA_MULZ + 9'd1,
				UA_MULZ + 9'd2,
				UA_MULZ + 9'd3:   urom_fp_fn = ujmp_fn(uw_fn(S_X12,   A_ZERO, B_ZERO, D_NONE, 5'd0),
					C_ALW, UA_MULZ + 9'd4);
				UA_MULZ + 9'd4:   urom_fp_fn = ujmp_fn(uw_fn(O_AND,   A_PK,   B_K,    D_R2,   K_SIGN),
					C_ALW, UA_FW + 9'd3);

				// DIVF: restoring division 1 bit a clock, a first step without a
				// shift and 25 more: quotient bits q0..q25 in T1, the remainder in
				// T0 sticky. 44 clocks for every operand, underflow +4.
				// x / 0 and 0 / 0 trap from the class test; +0 / x and -0 / x take
				// 13 and 12.
				UA_DIVF + 9'd0:   urom_fp_fn = ujmp_fn(UW_WAIT, C_RSV, UA_FRO);
				UA_DIVF + 9'd1:   urom_fp_fn = ujmp_fn(UW_WAIT, C_ZANY, UA_DIVZ);
				UA_DIVF + 9'd2:   urom_fp_fn = uw_fn(F_UNPK,  A_R1,   B_ZERO, D_T2,   5'd0);
				UA_DIVF + 9'd3:   urom_fp_fn = uw_fn(F_UNPK,  A_R2,   B_ZERO, D_T0,   5'd0);
				UA_DIVF + 9'd4:   urom_fp_fn = uw_fn(E_DIV,   A_ZERO, B_ZERO, D_NONE, K_DBIAS);
				UA_DIVF + 9'd5:   urom_fp_fn = uw_fn(S_X12,   A_ZERO, B_ZERO, D_NONE, 5'd0);
				UA_DIVF + 9'd6:   urom_fp_fn = uw_fn(F_DST0,  A_T0,   B_T2,   D_T0,   5'd0);
				UA_DIVF + 9'd7,
				UA_DIVF + 9'd8,
				UA_DIVF + 9'd9,
				UA_DIVF + 9'd10,
				UA_DIVF + 9'd11,
				UA_DIVF + 9'd12,
				UA_DIVF + 9'd13,
				UA_DIVF + 9'd14,
				UA_DIVF + 9'd15,
				UA_DIVF + 9'd16,
				UA_DIVF + 9'd17,
				UA_DIVF + 9'd18,
				UA_DIVF + 9'd19,
				UA_DIVF + 9'd20,
				UA_DIVF + 9'd21,
				UA_DIVF + 9'd22,
				UA_DIVF + 9'd23,
				UA_DIVF + 9'd24,
				UA_DIVF + 9'd25,
				UA_DIVF + 9'd26,
				UA_DIVF + 9'd27,
				UA_DIVF + 9'd28,
				UA_DIVF + 9'd29,
				UA_DIVF + 9'd30,
				UA_DIVF + 9'd31:  urom_fp_fn = uw_fn(F_DSTEP, A_T0,   B_T2,   D_T0,   5'd0);
				UA_DIVF + 9'd32:  urom_fp_fn = ujmp_fn(uw_fn(F_SHRK,  A_T1,   B_T0,   D_T0,   5'd31), C_ALW, UA_TAIL);
				// x / 0 traps as early as a reserved operand; 0 / 0 two clocks later.
				UA_DIVZ + 9'd1:   urom_fp_fn = ujmp_fn(uop_fn(O_COUNT, FX_HOLD - 5'd1), C_ALW, UA_FZD);
				UA_DIVZ + 9'd2:   urom_fp_fn = ujmp_fn(uw_fn(S_X12,   A_ZERO, B_ZERO, D_NONE, 5'd0),
					C_ALW, UA_DIVZ + 9'd4);
				UA_DIVZ + 9'd3:   urom_fp_fn = ujmp_fn(UW_WAIT, C_ALW, UA_FIV);
				UA_DIVZ + 9'd4:   urom_fp_fn = ujmp_fn(uw_fn(O_AND,   A_PK,   B_K,    D_R2,   K_SIGN),
					C_S2, UA_FW + 9'd1);
				UA_DIVZ + 9'd5:   urom_fp_fn = ujmp_fn(UW_WAIT, C_ALW, UA_FW + 9'd1);

				default:          urom_fp_fn = UW_ENDW;
			endcase
		end
	endfunction
