// Copyright (c) 2026 Jamie Blanks

// NEC V810 effective address unit, beside the decoder.
//
//   ID: base (register file port A, never forwarded) or the instruction's PC,
//       plus the decoder's displacement (disp16, disp9, disp26, or 0 for JMP)
//   EX: the registered sum is the load/store address or the transfer target
//
// A base written by the instruction just before is read only once it reaches
// the register file, which is the chip's 2-clock base interlock.
module necv810_ea
(
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        ce_i,
	input  wire        load_i,     // the instruction in ID moves to EX
	input  wire        pc_rel_i,   // Bcond, JR, JAL
	input  wire [31:0] pc_i,
	input  wire [31:0] base_i,
	input  wire [31:0] disp_i,
	output reg  [31:0] ea_o
);
	wire [31:0] sum_w = (pc_rel_i ? pc_i : base_i) + disp_i;

	always @(posedge clk_i or posedge reset_i) begin
		if (reset_i)
			ea_o <= 32'd0;
		else if (ce_i && load_i)
			ea_o <= sum_w;
	end
endmodule
