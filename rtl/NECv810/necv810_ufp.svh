// NEC V810 micro-word FP codes: the FP helper's operations (op[5] set), the jump
// conditions, the END word's flag modes and traps. Included by
// necv810_urom.svh and necv810_ufp. Subtract and compare use the ALU's SUB and its
// flags; C_EQ tests their Z.

	// Mantissa ops.
	localparam [5:0] F_ADD   = 6'h21;
	localparam [5:0] F_XOR   = 6'h22;
	localparam [5:0] F_UNPK  = 6'h23;       // {5'd0, 1, a[22:0], 3'd0}: hidden bit, 3 guard bits
	localparam [5:0] F_MANT  = 6'h24;       // a[25:3]
	localparam [5:0] F_ABSS  = 6'h25;       // |a|, sign to SG
	localparam [5:0] F_CNEG  = 6'h26;       // -a when SG
	localparam [5:0] F_SMTC  = 6'h27;       // sign-magnitude a to two's complement
	localparam [5:0] F_NORM  = 6'h28;       // shift a by SC, right with sticky when positive
	localparam [5:0] F_SHRK  = 6'h29;       // {a, b} >> imm with sticky from b
	localparam [5:0] F_RND   = 6'h2a;       // round a[26:3] to nearest even on a[2:0]
	localparam [5:0] F_INEX  = 6'h2b;       // a, noting a[2:0] lost
	localparam [5:0] F_MSTEP = 6'h2c;       // multiply step, multiplicand T2
	localparam [5:0] F_DST0  = 6'h2d;       // first divide step: compare T0 with b, no shift
	localparam [5:0] F_DSTEP = 6'h2e;       // divide step on {T0, T1}, divisor b
	// Exponent unit, sign and status.
	localparam [5:0] E_ALN2  = 6'h30;       // E = larger exponent, SC = reg2's gap
	localparam [5:0] E_ALN1  = 6'h31;       // SC = reg1's gap
	localparam [5:0] E_MUL   = 6'h32;       // E = e2 + e1 + K
	localparam [5:0] E_DIV   = 6'h33;       // E = e2 - e1 + K
	localparam [5:0] E_LZ    = 6'h34;       // SC = 5 - lz(a), E += SC
	localparam [5:0] E_LZK   = 6'h35;       // SC = 5 - lz(a), E = K - lz(a)
	localparam [5:0] E_RND   = 6'h36;       // E += a[27], the rounding carry
	localparam [5:0] E_SHK   = 6'h37;       // SC = K - e1
	localparam [5:0] S_S2    = 6'h38;       // SG = reg2's sign
	localparam [5:0] S_S1    = 6'h39;
	localparam [5:0] S_X12   = 6'h3a;       // SG = reg2's sign ^ reg1's
	localparam [5:0] S_X2    = 6'h3b;       // SG ^= reg2's sign
	localparam [5:0] X_FUD   = 6'h3c;
	localparam [5:0] X_FPR   = 6'h3d;

	// Jump conditions; C_NEXT does not jump.
	localparam [4:0] C_NEXT  = 5'd0;
	localparam [4:0] C_ALW   = 5'd1;
	localparam [4:0] C_EQ    = 5'd2;        // the last SUB found its operands equal
	localparam [4:0] C_RSV   = 5'd3;        // either operand reserved
	localparam [4:0] C_RSV1  = 5'd4;
	localparam [4:0] C_ZANY  = 5'd5;        // either zero: jump to next + {reg2 zero, reg1 zero}
	localparam [4:0] C_SDIF  = 5'd6;        // raw signs differ
	localparam [4:0] C_MEQ   = 5'd7;        // equal magnitudes
	localparam [4:0] C_Z1    = 5'd8;
	localparam [4:0] C_IZ1   = 5'd9;        // reg1 is integer zero
	localparam [4:0] C_INT   = 5'd10;       // reg1 has no fraction bits: exponent >= 150
	localparam [4:0] C_E158  = 5'd11;       // reg1 is 2^31 or more in magnitude
	localparam [4:0] C_S1    = 5'd12;
	localparam [4:0] C_S2    = 5'd13;
	localparam [4:0] C_BIG   = 5'd14;       // T0 is 2^24 or more
	localparam [4:0] C_OVF   = 5'd15;       // E >= 255
	localparam [4:0] C_UFL   = 5'd16;       // E <= 0
	localparam [4:0] C_ALT   = 5'd17;       // TRNC.SW in CVT.SW's routine, DIVF.S in the shared underflow
	localparam [4:0] C_GAP   = 5'd18;       // the last aligned operand is the larger, the other 27 or more below
	localparam [4:0] C_FPR   = 5'd19;       // FPR already set: the result is inexact
	// Bit strings, tested in the sequencer. More and stop look past a step made
	// in the same word; stopping for an interrupt with length left aborts.
	localparam [4:0] C_LEN0  = 5'd20;       // length 0
	localparam [4:0] C_MORE  = 5'd21;       // length left, no hit and no interrupt
	localparam [4:0] C_STOP  = 5'd22;
	localparam [4:0] C_SGT   = 5'd23;       // source offset above destination offset
	localparam [4:0] C_SLT   = 5'd24;
	localparam [4:0] C_FOUND = 5'd25;       // the search found its bit
	localparam [4:0] C_FULL  = 5'd26;       // destination offset 0: the last word was whole
	localparam [4:0] C_LAST  = 5'd27;       // the string ends in this word
	localparam [4:0] C_SCH0  = 5'd28;       // a search for 0
	localparam [4:0] C_ABORT = 5'd29;       // an interrupt stopped it
	localparam [4:0] C_FPART = 5'd30;       // the search starts inside a word: up off bit 0, down off bit 31
	localparam [4:0] C_DOWN  = 5'd31;       // a search down

	localparam [1:0] FM_NONE = 2'd0;
	localparam [1:0] FM_FP   = 2'd1;
	localparam [1:0] FM_INT  = 2'd2;
	localparam [1:0] FM_CMP  = 2'd3;

	localparam [2:0] FX_NONE = 3'd0;
	localparam [2:0] FX_FRO  = 3'd1;
	localparam [2:0] FX_FIV  = 3'd2;
	localparam [2:0] FX_FZD  = 3'd3;
	localparam [2:0] FX_FOV  = 3'd4;
