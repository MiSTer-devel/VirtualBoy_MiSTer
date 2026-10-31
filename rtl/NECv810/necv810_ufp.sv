// Copyright (c) 2026 Jamie Blanks

// NEC V810 FP helper beside the micro-sequencer's datapath: operand class
// tests, the exponent unit, the leading-zero encoder, the result sign and the
// sticky FPR/FUD bits. The mantissa work runs on the shared ALU and shifter.
//
//   reg1, reg2 --> class tests (zero, reserved, sign, exponent range) --> cond
//   exponents  --> exponent adder/compare --> E, shift count
//   a          --> leading zeros ------------^
//   E, sign    --> packed {sign, exponent} source for the result
module necv810_ufp
(
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        ce_i,
	input  wire        start_i,      // a routine starts: clear the status
	input  wire        run_i,        // a routine word runs this clock
	input  wire [5:0]  op_i,         // the word's op
	input  wire [4:0]  cond_i,       // its jump condition
	input  wire [9:0]  k_i,          // its constant, low bits
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [31:0] a_i,          // first source
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire [31:0] lz_i,         // the leading-zero count's input: a, or a bit-string scan
	input  wire        big_i,        // T0 is 2^24 or more
	input  wire [31:0] r1_i,
	input  wire [31:0] r2_i,
	input  wire        alt_i,        // the instruction is TRNC.SW or DIVF.S
	input  wire        inexact_i,    // the rounder dropped nonzero bits
	output wire [31:0] pk_o,         // {sign, exponent, 23'd0}
	output wire [5:0]  sc_o,         // signed shift: right when positive
	output wire        sg_o,
	output wire        cond_o,
	output wire [1:0]  zcls_o,       // {reg2 zero, reg1 zero}
	output wire        fpr_o,
	output wire        fud_o,
	output wire [5:0]  lz_o          // leading zeros of lz_i
);
	/* verilator lint_off UNUSEDPARAM */
	`include "necv810_ufp.svh"
	/* verilator lint_on UNUSEDPARAM */

	reg signed [9:0] e_q;            // result exponent, biased
	reg        [5:0] sc_q;
	reg              sg_q;
	reg              fpr_q;
	reg              fud_q;
	reg              far_q;          // the exponents are 27 or more apart

	wire signed [9:0] k10_w = $signed(k_i);

	// Operand classes.
	wire [7:0]  e1_w = r1_i[30:23];
	wire [7:0]  e2_w = r2_i[30:23];
	wire        z1_w = (r1_i[30:0] == 31'd0);
	wire        z2_w = (r2_i[30:0] == 31'd0);
	wire        rsv1_w = (e1_w == 8'hff) || ((e1_w == 8'd0) && !z1_w);
	wire        rsv2_w = (e2_w == 8'hff) || ((e2_w == 8'd0) && !z2_w);

	// Leading zeros of a, 0 to 32.
	function [5:0] lz_fn;
		input [31:0] v;
		reg   [15:0] h16;
		reg   [7:0]  h8;
		reg   [3:0]  h4;
		reg   [1:0]  h2;
		begin
			lz_fn = 6'd0;
			if (v[31:16] == 16'd0) begin lz_fn[4] = 1'b1; h16 = v[15:0]; end else h16 = v[31:16];
			if (h16[15:8] == 8'd0) begin lz_fn[3] = 1'b1; h8 = h16[7:0]; end else h8 = h16[15:8];
			if (h8[7:4] == 4'd0) begin lz_fn[2] = 1'b1; h4 = h8[3:0]; end else h4 = h8[7:4];
			if (h4[3:2] == 2'd0) begin lz_fn[1] = 1'b1; h2 = h4[1:0]; end else h2 = h4[3:2];
			if (!h2[1]) begin
				if (h2[0]) lz_fn[0] = 1'b1;
				else lz_fn = lz_fn + 6'd2;
			end
		end
	endfunction
	wire [5:0]  lz_w = lz_fn(lz_i);
	// Shift that puts a's leading one at bit 26.
	wire signed [9:0] nadj_w = 10'sd5 - $signed({4'd0, lz_w});

	// Exponent adder: two operands from the exponent fields, E, the constant
	// and the leading-zero count.
	reg  signed [9:0] ex_v;
	reg  signed [9:0] ey_v;
	reg  signed [9:0] ez_v;
	always @* begin
		ex_v = $signed({2'd0, e2_w});
		ey_v = 10'sd0;
		ez_v = 10'sd0;
		case (op_i)
			E_ALN2:  ey_v = -$signed({2'd0, e1_w});
			E_ALN1:  ey_v = -$signed({2'd0, e1_w});
			E_MUL:   begin ey_v = $signed({2'd0, e1_w}); ez_v = k10_w; end
			E_DIV:   begin ey_v = -$signed({2'd0, e1_w}); ez_v = k10_w; end
			E_LZ:    begin ex_v = e_q; ey_v = nadj_w; end
			E_LZK:   begin ex_v = k10_w; ey_v = -$signed({4'd0, lz_w}); end
			E_RND:   begin ex_v = e_q; ey_v = {9'd0, a_i[27]}; end
			E_SHK:   begin ex_v = k10_w; ey_v = -$signed({2'd0, e1_w}); end
			default: ;
		endcase
	end
	wire signed [9:0] esum_w = ex_v + ey_v + ez_v;

	// Shift counts kept within one word: right shifts of 31 or more drop all
	// of a 27-bit mantissa into the sticky bit.
	function [5:0] clip_fn;
		input signed [9:0] v;
		input              right_only;
		begin
			if (v > 10'sd31) clip_fn = 6'd31;
			else if (v < 10'sd0 && right_only) clip_fn = 6'd0;
			else if (v < -10'sd31) clip_fn = 6'b100001;
			else clip_fn = v[5:0];
		end
	endfunction

	always @(posedge clk_i or posedge reset_i) begin
		if (reset_i) begin
			e_q <= 10'sd0;
			sc_q <= 6'd0;
			sg_q <= 1'b0;
			fpr_q <= 1'b0;
			fud_q <= 1'b0;
			far_q <= 1'b0;
		end else if (ce_i) begin
			if (start_i) begin
				fpr_q <= 1'b0;
				fud_q <= 1'b0;
			end else if (run_i) begin
				case (op_i)
					// E = the larger exponent; shift reg2's mantissa by the gap.
					E_ALN2: begin
						e_q <= (esum_w < 10'sd0) ? $signed({2'd0, e1_w}) : $signed({2'd0, e2_w});
						sc_q <= clip_fn(-esum_w, 1'b1);
						far_q <= (esum_w >= 10'sd27) || (esum_w <= -10'sd27);
					end
					E_ALN1:  sc_q <= clip_fn(esum_w, 1'b1);
					// A product exponent already below range can only flush to
					// zero, so the result is known to be inexact here.
					E_MUL: begin
						e_q <= esum_w;
						if (esum_w < 10'sd0) fpr_q <= 1'b1;
					end
					E_DIV,
					E_RND:   e_q <= esum_w;
					E_LZ,
					E_LZK: begin
						e_q <= esum_w;
						sc_q <= nadj_w[5:0];
					end
					E_SHK:   sc_q <= clip_fn(esum_w, 1'b0);
					S_S2:    sg_q <= r2_i[31];
					S_S1:    sg_q <= r1_i[31];
					S_X12:   sg_q <= r2_i[31] ^ r1_i[31];
					S_X2:    sg_q <= sg_q ^ r2_i[31];
					F_ABSS:  sg_q <= a_i[31];
					X_FUD:   fud_q <= 1'b1;
					X_FPR:   fpr_q <= 1'b1;
					default: ;
				endcase
				if (inexact_i) fpr_q <= 1'b1;
			end
		end
	end

	// The packed exponent wraps by 192 on overflow, as the trap expects.
	wire [7:0]  pexp_w = (e_q >= 10'sd255) ? (e_q[7:0] - 8'd192) : e_q[7:0];

	reg         cond_v;
	always @* begin
		case (cond_i)
			C_RSV:   cond_v = rsv1_w || rsv2_w;
			C_RSV1:  cond_v = rsv1_w;
			C_ZANY:  cond_v = z1_w || z2_w;
			C_SDIF:  cond_v = r2_i[31] != r1_i[31];
			C_MEQ:   cond_v = r2_i[30:0] == r1_i[30:0];
			C_Z1:    cond_v = z1_w;
			C_IZ1:   cond_v = (r1_i == 32'd0);
			C_INT:   cond_v = (e1_w >= 8'd150);
			C_E158:  cond_v = (e1_w >= 8'd158);
			C_S1:    cond_v = r1_i[31];
			C_S2:    cond_v = r2_i[31];
			C_BIG:   cond_v = big_i;
			C_OVF:   cond_v = (e_q >= 10'sd255);
			C_UFL:   cond_v = (e_q <= 10'sd0);
			C_ALT:   cond_v = alt_i;
			C_GAP:   cond_v = far_q && (sc_q == 6'd0);
			C_FPR:   cond_v = fpr_q;
			default: cond_v = 1'b1;
		endcase
	end

	assign pk_o = {sg_q, pexp_w, 23'd0};
	assign sc_o = sc_q;
	assign sg_o = sg_q;
	assign cond_o = cond_v;
	assign zcls_o = {z2_w, z1_w};
	assign fpr_o = fpr_q;
	assign fud_o = fud_q;
	assign lz_o = lz_w;
endmodule
