// Copyright (c) 2026 Jamie Blanks

`timescale 1ns/1ps

// Display process. One engine shows one eye at a time:
//
//   SYNC  START_DELAY   group 0         group 1               group 95
//   |-----------------|---------------|---------------| ... |----------| end (L/RFBEND)
//   CTA read 0         SAM cols 0-3    SAM cols 4-7
//                      CTA read 1      CTA read 2
//
// A group lasts 16 x (LENGTH + 1) clocks from its column table entry. The
// entry for the next group is read from DRAM while a group runs; the pointer
// starts at the servo's CTA (or the CTA register under LOCK) and steps down.
// SAM reads take one VRAM port B halfword per clock in column order and never
// run ahead of the groups started. Each group also loads the SAM with one VRAM
// port A cycle. An eye still running is cut by the other eye's SYNC or by
// FRAMESTART. Everything shown is also copied to the presentation stream as it
// happens.
module vip_dp
#(
	parameter [7:0]   CTA_SERVO_L = 8'hb1,  // servo's column table start, left eye
	parameter [7:0]   CTA_SERVO_R = 8'haf,  // right eye
	parameter integer START_DELAY = 500,    // SYNC to the first column (fitted)
	parameter [5:0]   SS_WORD     = 6'd28
)
(
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        ce_i,

	// Scanner, one CE period each.
	input  wire        frame_start_i,
	input  wire        l_sync_i,
	input  wire        r_sync_i,

	// Active register values and commands from the host.
	input  wire        disp_i,
	input  wire        synce_i,
	input  wire        lock_i,
	input  wire        dprst_i,
	input  wire [3:0]  frmcyc_i,
	input  wire [31:0] brt_i,            // {REST, BRTC, BRTB, BRTA}
	input  wire [15:0] cta_i,            // CTA register {CTA_R, CTA_L}
	input  wire        displayed_fb_i,

	// Events and status to the host, one CE period each.
	output wire        ev_gamestart_o,   // also GCLK to the XP
	output reg         ev_lfbend_o,
	output reg         ev_rfbend_o,
	output wire [3:0]  dpbsy_o,          // {R1BSY, L1BSY, R0BSY, L0BSY}
	output wire        brt_latch_o,      // a group starts: host takes new BRTA/B/C and REST
	output wire [1:0]  cta_we_o,         // eye SYNC: host latches cta_o unless LOCK
	output wire [7:0]  cta_o,

	// DRAM DP port.
	output wire        dram_req_o,
	output wire [15:0] dram_addr_o,
	input  wire        dram_gnt_i,
	input  wire        dram_rvalid_i,
	input  wire [15:0] dram_rdata_i,

	// VRAM port A: the SAM load of each group.
	output wire        vram_req_o,
	output wire [15:0] vram_addr_o,
	input  wire        vram_gnt_i,

	// VRAM port B (SAM): data for the address of one CE cycle arrives by the next CE edge.
	output wire [15:0] sam_addr_o,
	input  wire [15:0] sam_rdata_i,

	// Presentation stream, one CE period per item.
	output reg         pr_frame_o,       // FRAMESTART; follows any eye end of the same clock
	output reg         pr_end_o,         // an eye ended, one CE after its last word
	output reg         pr_end_eye_o,
	output reg         pr_word_o,        // one framebuffer halfword (8 pixels down a column)
	output reg         pr_eye_o,
	output reg  [8:0]  pr_x_o,
	output reg  [4:0]  pr_strip_o,
	output reg  [15:0] pr_data_o,
	output reg         pr_group_o,       // LED exposure of one group
	output reg         pr_group_eye_o,
	output reg  [6:0]  pr_group_idx_o,
	output reg  [32:0] pr_exposure_o,    // {E3, E2, E1}, 20 MHz ticks per column for codes 1-3

	output wire        idle_o,           // no eye running and nothing in flight

	input  wire [5:0]  ss_addr_i,
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [63:0] ss_wdata_i,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        ss_we_i,
	output wire [63:0] ss_rdata_o
);
	localparam integer START_LAST_I = START_DELAY - 1;
	localparam [9:0]   START_LAST   = START_LAST_I[9:0];

	localparam [1:0] RD_IDLE = 2'd0;
	localparam [1:0] RD_REQ  = 2'd1;
	localparam [1:0] RD_WAIT = 2'd2;

	localparam [2:0] B_IDLE  = 3'd0;
	localparam [2:0] B_SUM   = 3'd1;
	localparam [2:0] B_UNIT  = 3'd2;
	localparam [2:0] B_REP   = 3'd3;
	localparam [2:0] B_CLIPA = 3'd4;
	localparam [2:0] B_CLIPB = 3'd5;
	localparam [2:0] B_CLIPC = 3'd6;

	// Frame and game frame.
	reg        waiting_q;      // after reset or DPRST, until FCLK rises
	reg [3:0]  gcnt_q;         // display frames since GCLK

	// Eye engine.
	reg        eye_on_q;
	reg        eye_q;          // 0 left, 1 right
	reg        fb_q;
	reg        run_q;          // groups running
	reg [9:0]  start_q;
	reg [6:0]  grp_q;
	reg [11:0] tmr_q;          // clocks left in the group
	reg [7:0]  ptr_q;          // next column table index to read
	reg [15:0] next_q;         // entry for the next group
	reg        next_ok_q;

	// Column table read.
	reg [1:0]  rd_st_q;
	reg        need_q;
	reg        drop_q;

	// SAM reads.
	reg [8:0]  sam_x_q;
	reg [4:0]  sam_s_q;
	reg [8:0]  sam_lim_q;      // columns released by started groups
	reg        sam_vld_q;
	reg [15:0] sam_addr_q;
	reg        sam_load_q;     // the group's SAM load waits for VRAM

	// Stream markers, delayed one CE behind the last SAM word.
	reg        end_d_q;
	reg        end_eye_d_q;
	reg        frame_d_q;

	// Brightness exposure.
	reg        job_q;
	reg        job_eye_q;
	reg [6:0]  job_grp_q;
	reg [15:0] job_ctc_q;
	reg        job_disp_q;
	reg [2:0]  b_st_q;
	reg        b_eye_q;
	reg [6:0]  b_grp_q;
	reg [7:0]  b_a_q;
	reg [7:0]  b_b_q;
	reg [7:0]  b_c_q;
	reg [7:0]  b_rest_q;
	reg [9:0]  b_abc_q;
	reg [10:0] b_unit_q;
	reg [10:0] b_rem_q;
	reg [8:0]  b_rep_q;
	reg [10:0] b_part_q;
	reg [10:0] b_e1_q;
	reg [10:0] b_e2_q;
	reg [10:0] b_e3_q;

	// SYNC reaches the VIP only with SYNCE set.
	wire sync_w      = (l_sync_i || r_sync_i) && synce_i;
	wire start_eye_w = sync_w && !waiting_q && !dprst_i;
	wire last_grp_w  = (grp_q == 7'd95);
	wire finish_w    = run_q && (tmr_q == 12'd0) && last_grp_w;
	wire end_w       = eye_on_q && !dprst_i && (finish_w || frame_start_i || sync_w);
	wire abort_w     = end_w || dprst_i;
	wire grp_start_w = eye_on_q && !abort_w && next_ok_q &&
		(run_q ? ((tmr_q == 12'd0) && !last_grp_w) : (start_q == 10'd0));
	wire [6:0] grp_new_w = run_q ? grp_q + 7'd1 : 7'd0;
	wire sam_go_w    = eye_on_q && !abort_w && (sam_x_q < sam_lim_q);
	wire [7:0] servo_cta_w = r_sync_i ? CTA_SERVO_R : CTA_SERVO_L;
	wire [7:0] start_ptr_w = !lock_i ? servo_cta_w : (r_sync_i ? cta_i[15:8] : cta_i[7:0]);
	wire gamestart_w = frame_start_i && !dprst_i && (waiting_q || (gcnt_q >= frmcyc_i));

	assign ev_gamestart_o = gamestart_w;
	assign dpbsy_o        = eye_on_q ? (4'b0001 << {fb_q, eye_q}) : 4'd0;
	assign brt_latch_o    = grp_start_w;
	assign cta_we_o       = {start_eye_w && r_sync_i, start_eye_w && !r_sync_i};
	assign cta_o          = servo_cta_w;
	assign dram_req_o     = (rd_st_q == RD_REQ);
	assign dram_addr_o    = {7'b1110111, eye_q, ptr_q};  // 0x3DC00 / 0x3DE00 + index x 2
	assign sam_addr_o     = sam_addr_q;
	assign vram_req_o     = sam_load_q;
	assign vram_addr_o    = {eye_q, fb_q, grp_q, 7'd0};
	assign idle_o         = !eye_on_q && (rd_st_q == RD_IDLE) && !sam_vld_q && !sam_load_q && !end_d_q && !frame_d_q &&
		!job_q && (b_st_q == B_IDLE);
	assign ss_rdata_o     = (ss_addr_i == SS_WORD) ? {59'd0, waiting_q, gcnt_q} : 64'd0;

	// Exposure sums keep the carry so the 11-bit truncation is explicit.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [11:0] e1_full_w = {1'b0, b_e1_q} + {4'd0, b_a_q};
	wire [11:0] e2_full_w = {1'b0, b_e2_q} + {4'd0, b_b_q};
	wire [11:0] e3_full_w = {1'b0, b_e3_q} + {2'd0, b_abc_q};
	wire [11:0] e1_part_w = {1'b0, b_e1_q} + {1'b0, b_part_q};
	wire [11:0] e2_part_w = {1'b0, b_e2_q} + {1'b0, b_part_q};
	wire [11:0] e3_part_w = {1'b0, b_e3_q} + {1'b0, b_part_q};
	wire [11:0] e3_a_w    = {1'b0, b_e3_q} + {4'd0, b_a_q};
	wire [11:0] e3_b_w    = {1'b0, b_e3_q} + {4'd0, b_b_q};
	wire [11:0] e3_c_w    = {1'b0, b_e3_q} + {4'd0, b_c_q};
	/* verilator lint_on UNUSEDSIGNAL */
	wire [10:0] a_off_w   = {3'd0, b_a_q} + 11'd1;
	wire [10:0] b_off_w   = {3'd0, b_b_q} + 11'd1;

	always @(posedge clk_i) begin
		if (reset_i) begin
			waiting_q      <= 1'b1;
			gcnt_q         <= 4'd0;
			eye_on_q       <= 1'b0;
			eye_q          <= 1'b0;
			fb_q           <= 1'b0;
			run_q          <= 1'b0;
			start_q        <= 10'd0;
			grp_q          <= 7'd0;
			tmr_q          <= 12'd0;
			ptr_q          <= 8'd0;
			next_q         <= 16'd0;
			next_ok_q      <= 1'b0;
			rd_st_q        <= RD_IDLE;
			need_q         <= 1'b0;
			drop_q         <= 1'b0;
			sam_x_q        <= 9'd0;
			sam_s_q        <= 5'd0;
			sam_lim_q      <= 9'd0;
			sam_vld_q      <= 1'b0;
			sam_addr_q     <= 16'd0;
			sam_load_q     <= 1'b0;
			end_d_q        <= 1'b0;
			end_eye_d_q    <= 1'b0;
			frame_d_q      <= 1'b0;
			ev_lfbend_o    <= 1'b0;
			ev_rfbend_o    <= 1'b0;
			pr_frame_o     <= 1'b0;
			pr_end_o       <= 1'b0;
			pr_end_eye_o   <= 1'b0;
			pr_word_o      <= 1'b0;
			pr_eye_o       <= 1'b0;
			pr_x_o         <= 9'd0;
			pr_strip_o     <= 5'd0;
			pr_data_o      <= 16'd0;
			job_q          <= 1'b0;
			job_eye_q      <= 1'b0;
			job_grp_q      <= 7'd0;
			job_ctc_q      <= 16'd0;
			job_disp_q     <= 1'b0;
			b_st_q         <= B_IDLE;
			pr_group_o     <= 1'b0;
			pr_group_eye_o <= 1'b0;
			pr_group_idx_o <= 7'd0;
			pr_exposure_o  <= 33'd0;
		end else if (ss_we_i) begin
			// Pause happens with the DP idle; only the frame counters carry state.
			if (ss_addr_i == SS_WORD) begin
				gcnt_q    <= ss_wdata_i[3:0];
				waiting_q <= ss_wdata_i[4];
				eye_on_q  <= 1'b0;
				rd_st_q   <= RD_IDLE;
				need_q    <= 1'b0;
				sam_vld_q <= 1'b0;
				sam_load_q <= 1'b0;
				job_q     <= 1'b0;
				b_st_q    <= B_IDLE;
			end
		end else if (ce_i) begin
			ev_lfbend_o <= 1'b0;
			ev_rfbend_o <= 1'b0;
			pr_group_o  <= 1'b0;

			// LED exposure per column of a group: (REPEAT + 1) pulse trains of
			// A, B, C, each followed by one off clock, then REST + 1, clipped to
			// the column length. Code 3 lights through A, B and C.
			case (b_st_q)
				B_IDLE: begin
					if (job_q) begin
						job_q    <= 1'b0;
						b_eye_q  <= job_eye_q;
						b_grp_q  <= job_grp_q;
						b_a_q    <= brt_i[7:0];
						b_b_q    <= brt_i[15:8];
						b_c_q    <= brt_i[23:16];
						b_rest_q <= brt_i[31:24];
						b_rem_q  <= {1'b0, job_ctc_q[7:0], 2'b00} + 11'd4;
						b_rep_q  <= {1'b0, job_ctc_q[15:8]} + 9'd1;
						b_e1_q   <= 11'd0;
						b_e2_q   <= 11'd0;
						b_e3_q   <= 11'd0;
						if (!job_disp_q || (job_ctc_q[7:0] < 8'h2c)) begin
							pr_group_o     <= 1'b1;
							pr_group_eye_o <= job_eye_q;
							pr_group_idx_o <= job_grp_q;
							pr_exposure_o  <= 33'd0;
						end else begin
							b_st_q <= B_SUM;
						end
					end
				end
				B_SUM: begin
					b_abc_q <= {2'd0, b_a_q} + {2'd0, b_b_q} + {2'd0, b_c_q};
					b_st_q  <= B_UNIT;
				end
				B_UNIT: begin
					b_unit_q <= {1'b0, b_abc_q} + {3'd0, b_rest_q} + 11'd5;
					b_st_q   <= B_REP;
				end
				B_REP: begin
					if (b_rem_q <= b_unit_q) begin
						b_part_q <= b_rem_q;
						b_st_q   <= B_CLIPA;
					end else if (b_rep_q == 9'd1) begin
						pr_group_o     <= 1'b1;
						pr_group_eye_o <= b_eye_q;
						pr_group_idx_o <= b_grp_q;
						pr_exposure_o  <= {e3_full_w[10:0], e2_full_w[10:0], e1_full_w[10:0]};
						b_st_q         <= B_IDLE;
					end else begin
						b_rem_q <= b_rem_q - b_unit_q;
						b_rep_q <= b_rep_q - 9'd1;
						b_e1_q  <= e1_full_w[10:0];
						b_e2_q  <= e2_full_w[10:0];
						b_e3_q  <= e3_full_w[10:0];
					end
				end
				B_CLIPA: begin
					if (b_part_q >= {3'd0, b_a_q}) begin
						b_e1_q   <= e1_full_w[10:0];
						b_e3_q   <= e3_a_w[10:0];
						b_part_q <= (b_part_q > a_off_w) ? b_part_q - a_off_w : 11'd0;
					end else begin
						b_e1_q   <= e1_part_w[10:0];
						b_e3_q   <= e3_part_w[10:0];
						b_part_q <= 11'd0;
					end
					b_st_q <= B_CLIPB;
				end
				B_CLIPB: begin
					if (b_part_q >= {3'd0, b_b_q}) begin
						b_e2_q   <= e2_full_w[10:0];
						b_e3_q   <= e3_b_w[10:0];
						b_part_q <= (b_part_q > b_off_w) ? b_part_q - b_off_w : 11'd0;
					end else begin
						b_e2_q   <= e2_part_w[10:0];
						b_e3_q   <= e3_part_w[10:0];
						b_part_q <= 11'd0;
					end
					b_st_q <= B_CLIPC;
				end
				B_CLIPC: begin
					pr_group_o     <= 1'b1;
					pr_group_eye_o <= b_eye_q;
					pr_group_idx_o <= b_grp_q;
					pr_exposure_o  <= {(b_part_q >= {3'd0, b_c_q}) ? e3_c_w[10:0] : e3_part_w[10:0],
						b_e2_q, b_e1_q};
					b_st_q         <= B_IDLE;
				end
				default: begin
					b_st_q <= B_IDLE;
				end
			endcase

			// FCLK rise counts display frames for GCLK; DPRST waits for the next one.
			if (dprst_i) begin
				waiting_q <= 1'b1;
				gcnt_q    <= 4'd0;
			end else if (frame_start_i) begin
				waiting_q <= 1'b0;
				gcnt_q    <= gamestart_w ? 4'd0 : gcnt_q + 4'd1;
			end

			// Group timing.
			if (eye_on_q && !run_q && (start_q != 10'd0)) begin
				start_q <= start_q - 10'd1;
			end
			if (eye_on_q && run_q && (tmr_q != 12'd0)) begin
				tmr_q <= tmr_q - 12'd1;
			end
			if (grp_start_w) begin
				run_q     <= 1'b1;
				grp_q     <= grp_new_w;
				next_ok_q <= 1'b0;
				tmr_q     <= {next_q[7:0], 4'hf};
				need_q    <= (grp_new_w != 7'd95);
				sam_lim_q <= {grp_new_w + 7'd1, 2'b00};
				job_q     <= 1'b1;
				job_eye_q <= eye_q;
				job_grp_q <= grp_new_w;
				job_ctc_q <= next_q;
				job_disp_q <= disp_i;
			end

			// Column table read: one at a time; an aborted read is ignored on return.
			case (rd_st_q)
				RD_IDLE: begin
					if (need_q && !abort_w) begin
						rd_st_q <= RD_REQ;
						need_q  <= 1'b0;
					end
				end
				RD_REQ: begin
					if (dram_gnt_i) begin
						rd_st_q <= RD_WAIT;
						drop_q  <= abort_w;
					end else if (abort_w) begin
						rd_st_q <= RD_IDLE;
					end
				end
				default: begin
					if (dram_rvalid_i) begin
						rd_st_q <= RD_IDLE;
						drop_q  <= 1'b0;
						if (!drop_q && !abort_w) begin
							next_q    <= dram_rdata_i;
							next_ok_q <= 1'b1;
							ptr_q     <= ptr_q - 8'd1;
						end
					end else if (abort_w) begin
						drop_q <= 1'b1;
					end
				end
			endcase

			if (grp_start_w) begin
				sam_load_q <= 1'b1;
			end else if (vram_gnt_i || abort_w) begin
				sam_load_q <= 1'b0;
			end

			// SAM reads in column order, 28 halfwords a column.
			sam_vld_q <= sam_go_w;
			if (sam_go_w) begin
				sam_addr_q <= {eye_q, fb_q, sam_x_q, sam_s_q};
				if (sam_s_q == 5'd27) begin
					sam_s_q <= 5'd0;
					sam_x_q <= sam_x_q + 9'd1;
				end else begin
					sam_s_q <= sam_s_q + 5'd1;
				end
			end
			pr_word_o  <= sam_vld_q;
			pr_eye_o   <= sam_addr_q[15];
			pr_x_o     <= sam_addr_q[13:5];
			pr_strip_o <= sam_addr_q[4:0];
			// DISP off leaves the LEDs dark.
			pr_data_o  <= disp_i ? sam_rdata_i : 16'd0;

			end_d_q      <= end_w;
			end_eye_d_q  <= eye_q;
			frame_d_q    <= frame_start_i;
			pr_end_o     <= end_d_q;
			pr_end_eye_o <= end_eye_d_q;
			pr_frame_o   <= frame_d_q;

			if (end_w) begin
				ev_lfbend_o <= !eye_q;
				ev_rfbend_o <= eye_q;
			end
			if (abort_w) begin
				eye_on_q  <= 1'b0;
				need_q    <= 1'b0;
				next_ok_q <= 1'b0;
				job_q     <= 1'b0;
				b_st_q    <= B_IDLE;
			end
			if (start_eye_w) begin
				eye_on_q  <= 1'b1;
				eye_q     <= r_sync_i;
				fb_q      <= displayed_fb_i;
				run_q     <= 1'b0;
				grp_q     <= 7'd0;
				start_q   <= START_LAST;
				ptr_q     <= start_ptr_w;
				need_q    <= 1'b1;
				next_ok_q <= 1'b0;
				sam_x_q   <= 9'd0;
				sam_s_q   <= 5'd0;
				sam_lim_q <= 9'd0;
			end

		end
	end

endmodule
