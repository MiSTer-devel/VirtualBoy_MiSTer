// Copyright (c) 2026 Jamie Blanks
//
// Band-limited rate converter from the VSU sample rate (125000/3 Hz, core
// clock) to 48 kHz on CLK_AUDIO (24.576 MHz). Point-sampling the 41.7 kHz
// staircase at 48 kHz folds its images back into the audible band.
//
// 48000 / (125000/3) = 144 / 125, so each output moves 125/144 of an input.
// A low-pass at 144x the source rate is split into 144 phases of 42 taps;
// phase p, tap i weights x[q - i], where q is the newest input reached.
//
//   core clk: sample -> mailbox --toggle--> history RAM (64 x L, 64 x R)
//   CLK_AUDIO: every 512 clocks advance (p, q), then 2 x 42 MACs -> out
//
// The two clocks share a crystal, so q normally trails the newest input by a
// steady slack. When the core pauses, q slows and then holds; it creeps back
// once samples return, so stale history is never read.

module audio_resampler
#(
	parameter COEFF_FILE = "rtl/Audio/audio_resampler_coeffs.hex"
)
(
	input  wire               src_clk_i,
	input  wire               src_sample_i,
	input  wire signed [17:0] src_left_i,
	input  wire signed [17:0] src_right_i,

	input  wire               clk_i,
	output reg  signed [15:0] left_o  = 16'sd0,
	output reg  signed [15:0] right_o = 16'sd0
);
	localparam [7:0] PHASES       = 8'd144;
	localparam [7:0] STEP         = 8'd125;
	localparam [5:0] TAP_LAST     = 6'd41;
	localparam [6:0] SLACK_TARGET = 7'd4;
	localparam [6:0] SLACK_MAX    = 7'd16;

	// Source side: hold the sample, then flip the toggle a clock later.
	reg        [35:0] mbox_q         = 36'd0;
	reg               mbox_pending_q = 1'b0;
	reg               src_toggle_q   = 1'b0;

	always @(posedge src_clk_i) begin
		mbox_pending_q <= src_sample_i;
		if (src_sample_i) begin
			mbox_q <= {src_left_i, src_right_i};
		end
		if (mbox_pending_q) begin
			src_toggle_q <= ~src_toggle_q;
		end
	end

	// Audio side: write L then R, then count the sample as available.
	reg [2:0] toggle_sync_q = 3'd0;
	reg       write_r_q     = 1'b0;
	reg [6:0] wr_q          = 7'd0;
	wire      arrive_w      = toggle_sync_q[2] ^ toggle_sync_q[1];

	always @(posedge clk_i) begin
		toggle_sync_q <= {toggle_sync_q[1:0], src_toggle_q};
		write_r_q     <= arrive_w;
		if (write_r_q) begin
			wr_q <= wr_q + 7'd1;
		end
	end

	// Output timing and read position.
	reg [8:0] div_q  = 9'd0;
	reg [7:0] p_q    = 8'd0;
	reg [6:0] q_q    = 7'd0;
	reg       start_q = 1'b0;
	wire [6:0] slack_w = wr_q - q_q - 7'd1;
	reg  [7:0] step_w;
	wire [8:0] p_sum_w = {1'b0, p_q} + {1'b0, step_w};

	always @(*) begin
		if (slack_w == 7'd0) begin
			step_w = 8'd0;
		end else if (slack_w < SLACK_TARGET - 7'd1) begin
			step_w = STEP - 8'd1;
		end else if (slack_w > SLACK_TARGET + 7'd1) begin
			step_w = STEP + 8'd1;
		end else begin
			step_w = STEP;
		end
	end

	always @(posedge clk_i) begin
		div_q   <= div_q + 9'd1;
		start_q <= (div_q == 9'd0);
		if (div_q == 9'd0) begin
			if (slack_w > SLACK_MAX) begin
				q_q <= wr_q - SLACK_TARGET - 7'd1;
			end else if (p_sum_w >= {1'b0, PHASES}) begin
				p_q <= p_sum_w[7:0] - PHASES;
				q_q <= q_q + 7'd1;
			end else begin
				p_q <= p_sum_w[7:0];
			end
		end
	end

	// Phases 72..143 are phases 71..0 with the taps reversed.
	reg         busy_q   = 1'b0;
	reg         ch_q     = 1'b0;
	reg  [5:0]  tap_q    = 6'd0;
	reg         mirror_q = 1'b0;
	reg  [11:0] base_q   = 12'd0;
	wire [7:0]  row_full_w = (p_q < 8'd72) ? p_q : (8'd143 - p_q);
	wire [6:0]  row_w      = row_full_w[6:0];
	wire [5:0]  coeff_tap_w  = mirror_q ? (TAP_LAST - tap_q) : tap_q;
	wire [11:0] coeff_addr_w = base_q + {6'd0, coeff_tap_w};
	wire [6:0]  hist_index_w = q_q - {1'b0, tap_q};

	always @(posedge clk_i) begin
		if (start_q) begin
			busy_q   <= 1'b1;
			ch_q     <= 1'b0;
			tap_q    <= 6'd0;
			mirror_q <= (p_q >= 8'd72);
			// row * 42
			base_q   <= {row_w, 5'd0} + {1'b0, row_w, 3'd0} + {4'd0, row_w, 1'b0};
		end else if (busy_q) begin
			if (tap_q == TAP_LAST) begin
				tap_q  <= 6'd0;
				ch_q   <= 1'b1;
				busy_q <= !ch_q;
			end else begin
				tap_q <= tap_q + 6'd1;
			end
		end
	end

	// Depth slices of 1K x 10 fit 3024 words in 6 blocks; width slices take 9.
	(* ramstyle = "M10K", max_depth = 1024 *) reg [17:0] coeff_rom [0:3023];
	initial $readmemh(COEFF_FILE, coeff_rom);

	reg signed [17:0] coeff_q;
	wire       [17:0] hist_w;

	always @(posedge clk_i) begin
		coeff_q <= coeff_rom[coeff_addr_w];
	end

	cache_ram_dp #(
		.ADDR_WIDTH (7),
		.DATA_WIDTH (18)
	) u_history (
		.clk_i     (clk_i),
		.addr_a_i  ({write_r_q, wr_q[5:0]}),
		.wren_a_i  (arrive_w | write_r_q),
		.wdata_a_i (write_r_q ? mbox_q[17:0] : mbox_q[35:18]),
		.q_a_o     (),
		.addr_b_i  ({ch_q, hist_index_w[5:0]}),
		.wren_b_i  (1'b0),
		.wdata_b_i (18'd0),
		.q_b_o     (hist_w)
	);

	// MAC pipeline: memory read, multiply, accumulate, round.
	reg        [2:0]  read_q  = 3'd0; // {valid, first, last}
	reg        [2:0]  mul_q   = 3'd0;
	reg               acc_last_q = 1'b0;
	reg               acc_ch_q   = 1'b0;
	reg               read_ch_q  = 1'b0;
	reg               mul_ch_q   = 1'b0;
	reg signed [35:0] prod_q;
	reg signed [37:0] acc_q;
	reg signed [15:0] left_hold_q;

	// Saturate acc / 2^18 to 16 bits.
	function signed [15:0] output_fn;
		input signed [37:0] acc;
		reg signed [37:0] rounded_v;
		begin
			rounded_v = (acc + 38'sd131072) >>> 18;
			if (rounded_v > 38'sd32767) begin
				output_fn = 16'sh7fff;
			end else if (rounded_v < -38'sd32768) begin
				output_fn = 16'sh8000;
			end else begin
				output_fn = rounded_v[15:0];
			end
		end
	endfunction

	always @(posedge clk_i) begin
		read_q     <= {busy_q, tap_q == 6'd0, tap_q == TAP_LAST};
		read_ch_q  <= ch_q;
		mul_q      <= read_q;
		mul_ch_q   <= read_ch_q;
		prod_q     <= coeff_q * $signed(hist_w);
		acc_last_q <= mul_q[2] && mul_q[0];
		acc_ch_q   <= mul_ch_q;
		if (mul_q[2]) begin
			acc_q <= (mul_q[1] ? 38'sd0 : acc_q) + $signed({{2{prod_q[35]}}, prod_q});
		end
		if (acc_last_q) begin
			if (acc_ch_q) begin
				left_o  <= left_hold_q;
				right_o <= output_fn(acc_q);
			end else begin
				left_hold_q <= output_fn(acc_q);
			end
		end
	end

endmodule
