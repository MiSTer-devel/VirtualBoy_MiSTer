// Copyright (c) 2026 Jamie Blanks

`timescale 1ns/1ps

// Double-buffered strip buffer: 2 halves x 2 eyes x 384 columns. A word holds
// one column of the strip, row r in bits 2r+1:2r, the framebuffer's own layout,
// so the transfer reads and clears one column per access.
//
// Columns live in 8 banks by x mod 8, so the 8 pixels of a row write land in 8
// different banks and go in together whatever their alignment. Each half of a
// bank is its own RAM, so the transfer reads one half while drawing uses the other.
//
//   bank k, half h RAM:  port A: frame clear (left eye), read clears, merge reads
//                        port B: frame clear (right eye), transfer reads, merge write-back
//
//   accept -> read A -> merge the pixel into its row -> write B
//
// The clear before a frame's drawing writes one column per clock in both halves
// and both eyes, so it takes the 384 clocks of the screen's width.
//
// A transfer read into the RAM a merge is waiting on stalls that merge for a
// clock. Columns outside 0..383 are dropped.

module vip_strip
(
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        ce_i,
	input  wire        abort_i,          // Drops the clear and any writes in flight.

	input  wire        clear_start_i,    // Fills both halves with the background colour.
	input  wire [1:0]  clear_color_i,
	output wire        clear_busy_o,

	input  wire        wr_half_i,        // Half that pixel writes go to.
	input  wire        wr_valid_i,
	output wire        wr_ready_o,
	input  wire        wr_eye_i,
	input  wire [2:0]  wr_row_i,
	input  wire [9:0]  wr_x_i,           // Signed x of pixel 0.
	input  wire [15:0] wr_pixels_i,      // Pixel i in bits 2i+1:2i.
	input  wire [7:0]  wr_opaque_i,      // Bit i set: pixel i replaces the buffer.
	output wire        wr_busy_o,        // Accepted writes not yet in the RAM.

	input  wire        rd_valid_i,       // Always taken.
	input  wire        rd_half_i,
	input  wire        rd_eye_i,
	input  wire [8:0]  rd_col_i,
	input  wire [1:0]  rd_color_i,       // Written behind each read.
	output wire        rd_rvalid_o,      // rd_data_o holds the read taken last clock.
	output wire [15:0] rd_data_o
);

	reg        clear_q;
	reg [8:0]  clr_col_q;                // Column the clear writes next.
	reg        rd_rvalid_q;
	reg [3:0]  rd_sel_q;

	wire [7:0]  stall_w;
	wire [7:0]  s1_busy_w;
	wire [7:0]  rc_busy_w;               // A bank's port A clears behind a read.
	wire [15:0] q_b_w [0:15];            // Indexed {half, bank}.

	assign clear_busy_o = clear_q;
	assign wr_ready_o   = !clear_q && (stall_w == 8'd0) && (rc_busy_w == 8'd0);
	assign wr_busy_o    = s1_busy_w != 8'd0;
	assign rd_rvalid_o  = rd_rvalid_q;
	assign rd_data_o    = q_b_w[rd_sel_q];

	genvar k, h;
	generate
		for (k = 0; k < 8; k = k + 1) begin : g_bank
			reg        a_half_q;
			reg        a_both_q;           // The clear writes both halves.
			reg        rc_q;               // A transfer read last clock: clear its word.
			reg        rc_half_q;
			reg [6:0]  rc_addr_q;
			reg [6:0]  a_addr_q;
			reg        a_we_q;
			reg [15:0] a_wdata_q;
			reg        s1_valid_q;         // Word read last clock, merged this clock.
			reg [1:0]  s1_pix_q;
			reg        s1_opaque_q;
			reg [2:0]  s1_row_q;
			reg [6:0]  b_addr_q  [0:1];
			reg        b_we_q    [0:1];
			reg [15:0] b_wdata_q [0:1];
			wire [15:0] q_a_w [0:1];
			localparam [2:0] K = k;

			// This bank holds pixel (k - x) mod 8 of the write, one word on when that wraps.
			wire [3:0]  pix_d_w   = {1'b0, K} - {1'b0, wr_x_i[2:0]};
			wire [2:0]  pix_w     = pix_d_w[2:0];
			wire [6:0]  word_w    = wr_x_i[9:3] + {6'd0, pix_d_w[3]};
			wire        take_w    = !word_w[6] && (word_w[5:0] <= 6'd47);
			wire [6:0]  wr_addr_w = {wr_eye_i, word_w[5:0]};
			wire        rd_here_w = rd_valid_i && (rd_col_i[2:0] == K);
			wire        clr_here_w = clear_q && (clr_col_q[2:0] == K);

			// The write-back registered last clock lands in the same clock as the read.
			wire [15:0] old_w = (b_we_q[a_half_q] && (b_addr_q[a_half_q] == a_addr_q)) ?
				b_wdata_q[a_half_q] : q_a_w[a_half_q];
			wire [15:0] mask_w   = {14'd0, {2{s1_opaque_q}}} << {s1_row_q, 1'b0};
			wire [15:0] merged_w = (old_w & ~mask_w) | ({8{s1_pix_q}} & mask_w);

			assign stall_w[k]   = s1_valid_q && rd_here_w && (rd_half_i == a_half_q);
			assign s1_busy_w[k] = s1_valid_q;
			assign rc_busy_w[k] = rc_q;

			for (h = 0; h < 2; h = h + 1) begin : g_half
				cache_ram_dp #(
					.ADDR_WIDTH (7),
					.DATA_WIDTH (16)
				) u_ram (
					.clk_i     (clk_i),
					.addr_a_i  (a_addr_q),
					.wren_a_i  (a_we_q && (a_both_q || (a_half_q == (h == 1)))),
					.wdata_a_i (a_wdata_q),
					.q_a_o     (q_a_w[h]),
					.addr_b_i  (b_addr_q[h]),
					.wren_b_i  (b_we_q[h]),
					.wdata_b_i (b_wdata_q[h]),
					.q_b_o     (q_b_w[8*h + k])
				);
			end

			integer hi;
			always @(posedge clk_i) begin
				if (reset_i) begin
					a_half_q    <= 1'b0;
					a_both_q    <= 1'b0;
					rc_q        <= 1'b0;
					rc_half_q   <= 1'b0;
					rc_addr_q   <= 7'd0;
					a_addr_q    <= 7'd0;
					a_we_q      <= 1'b0;
					a_wdata_q   <= 16'd0;
					s1_valid_q  <= 1'b0;
					s1_pix_q    <= 2'd0;
					s1_opaque_q <= 1'b0;
					s1_row_q    <= 3'd0;
					for (hi = 0; hi < 2; hi = hi + 1) begin
						b_addr_q[hi]  <= 7'd0;
						b_we_q[hi]    <= 1'b0;
						b_wdata_q[hi] <= 16'd0;
					end
				end else if (ce_i) begin
					if (abort_i) begin
						a_we_q     <= 1'b0;
						rc_q       <= 1'b0;
						s1_valid_q <= 1'b0;
						b_we_q[0]  <= 1'b0;
						b_we_q[1]  <= 1'b0;
					end else begin
						// Port A: clear, a read's clear, a stalled merge holds, then a new write.
						a_we_q   <= 1'b0;
						a_both_q <= 1'b0;
						rc_q     <= rd_here_w;
						if (rd_here_w) begin
							rc_half_q <= rd_half_i;
							rc_addr_q <= {rd_eye_i, rd_col_i[8:3]};
						end
						if (!stall_w[k]) begin
							s1_valid_q <= 1'b0;
						end
						if (clr_here_w) begin
							a_we_q    <= 1'b1;
							a_both_q  <= 1'b1;
							a_addr_q  <= {1'b0, clr_col_q[8:3]};
							a_wdata_q <= {8{clear_color_i}};
						end else if (rc_q) begin
							a_we_q    <= 1'b1;
							a_half_q  <= rc_half_q;
							a_addr_q  <= rc_addr_q;
							a_wdata_q <= {8{rd_color_i}};
						end else if (wr_valid_i && wr_ready_o && take_w) begin
							a_half_q    <= wr_half_i;
							a_addr_q    <= wr_addr_w;
							s1_valid_q  <= 1'b1;
							s1_pix_q    <= wr_pixels_i[{pix_w, 1'b0} +: 2];
							s1_opaque_q <= wr_opaque_i[pix_w];
							s1_row_q    <= wr_row_i;
						end

						// Port B: the clear's right eye, the transfer, otherwise the merged word.
						for (hi = 0; hi < 2; hi = hi + 1) begin
							b_we_q[hi] <= 1'b0;
							if (clr_here_w) begin
								b_we_q[hi]    <= 1'b1;
								b_addr_q[hi]  <= {1'b1, clr_col_q[8:3]};
								b_wdata_q[hi] <= {8{clear_color_i}};
							end else if (rd_here_w && (rd_half_i == (hi == 1))) begin
								b_addr_q[hi] <= {rd_eye_i, rd_col_i[8:3]};
							end else if (s1_valid_q && (a_half_q == (hi == 1))) begin
								b_we_q[hi]    <= 1'b1;
								b_addr_q[hi]  <= a_addr_q;
								b_wdata_q[hi] <= merged_w;
							end
						end
					end
				end
			end
		end
	endgenerate

	always @(posedge clk_i) begin
		if (reset_i) begin
			clear_q     <= 1'b0;
			clr_col_q   <= 9'd0;
			rd_rvalid_q <= 1'b0;
			rd_sel_q    <= 4'd0;
		end else if (ce_i) begin
			if (abort_i) begin
				clear_q     <= 1'b0;
				rd_rvalid_q <= 1'b0;
			end else begin
				rd_rvalid_q <= rd_valid_i;
				if (rd_valid_i) begin
					rd_sel_q <= {rd_half_i, rd_col_i[2:0]};
				end

				if (clear_q) begin
					clr_col_q <= clr_col_q + 9'd1;
					if (clr_col_q == 9'd383) begin
						clear_q <= 1'b0;
					end
				end
				if (clear_start_i) begin
					clear_q   <= 1'b1;
					clr_col_q <= 9'd0;
				end
			end
		end
	end

endmodule
