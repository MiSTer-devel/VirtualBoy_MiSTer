// Copyright (c) 2026 Jamie Blanks
//
// Virtual Boy analog output coupling: a 100 ohm / 220 uF RC high-pass after
// the VSU DAC, stepped once per VSU sample (T = 480 / 20 MHz = 24 us).
//
//   y[n] = a * (y[n-1] + x[n] - x[n-1]),  a = RC / (RC + T)
//
// 1 - a = 24 / 22024, close to 1143 / 2^20. y carries 16 fraction bits.

module vsu_rc_highpass
(
	input  wire               clk_i,
	input  wire               sample_i,  // VSU publish strobe; data settles one clock later
	input  wire [9:0]         left_i,
	input  wire [9:0]         right_i,
	output reg                sample_o = 1'b0,
	output reg  signed [17:0] left_o   = 18'sd0, // y * 128
	output reg  signed [17:0] right_o  = 18'sd0
);
	reg        [2:0]  stage_q = 3'd0;
	reg        [9:0]  x_l_q   = 10'd0;
	reg        [9:0]  x_r_q   = 10'd0;
	reg signed [27:0] s_l_q   = 28'sd0;
	reg signed [27:0] s_r_q   = 28'sd0;
	reg signed [27:0] y_l_q   = 28'sd0;
	reg signed [27:0] y_r_q   = 28'sd0;

	function signed [27:0] input_step_fn;
		input signed [27:0] y;
		input [9:0]         x;
		input [9:0]         x_prev;
		reg signed [10:0] delta_v;
		begin
			delta_v = $signed({1'b0, x}) - $signed({1'b0, x_prev});
			input_step_fn = y + $signed({delta_v, 16'd0});
		end
	endfunction

	// s - s * 1143 / 2^20
	function signed [27:0] decay_fn;
		input signed [27:0] s;
		reg signed [38:0] wide_v;
		reg signed [38:0] leak_v;
		begin
			wide_v   = $signed({{11{s[27]}}, s});
			leak_v   = ((wide_v <<< 10) + (wide_v <<< 7)) - ((wide_v <<< 3) + wide_v);
			leak_v   = leak_v >>> 20;
			decay_fn = s - $signed(leak_v[27:0]);
		end
	endfunction

	function signed [17:0] scale_fn;
		input signed [27:0] y;
		reg signed [27:0] rounded_v;
		begin
			rounded_v = (y + 28'sd256) >>> 9;
			scale_fn  = rounded_v[17:0];
		end
	endfunction

	always @(posedge clk_i) begin
		stage_q  <= {stage_q[1:0], sample_i};
		sample_o <= stage_q[2];

		if (stage_q[0]) begin
			s_l_q <= input_step_fn(y_l_q, left_i, x_l_q);
			s_r_q <= input_step_fn(y_r_q, right_i, x_r_q);
			x_l_q <= left_i;
			x_r_q <= right_i;
		end
		if (stage_q[1]) begin
			y_l_q <= decay_fn(s_l_q);
			y_r_q <= decay_fn(s_r_q);
		end
		if (stage_q[2]) begin
			left_o  <= scale_fn(y_l_q);
			right_o <= scale_fn(y_r_q);
		end
	end

endmodule
