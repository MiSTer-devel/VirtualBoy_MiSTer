// Copyright (c) 2026 Jamie Blanks

`timescale 1ns/1ps

// Drawing process control.
//
//   GCLK -> clear both halves to BKCOL, then strip 0..27, each in strip
//   buffer half (strip & 1):
//     walk worlds 31..0 (attributes from DRAM)
//       END: stop, dummy: skip, else BG or OBJ unit
//     hand the half to the transfer; the next strip starts when it ends
//   transfer: left eye columns 0..383, then right, one strip buffer column
//   read and cleared per FB halfword
//   after the last transfer: XPEND
//
// Each strip buffer column is cleared to the current BKCOL as the transfer
// reads it, so a half is clean again for the strip after next. SBCOUNT steps
// as a strip's walk starts.
//
// Only one unit runs at a time. DRAM goes to the world walker or the running
// unit; VRAM goes to the running unit first, then the transfer.

module vip_xp
#(
	// Fitted step costs in clocks.
	parameter integer STRIP_START_COST = 4,  // A strip's start to its first world read.
	parameter integer WORLD_COST       = 3,  // A world's last attribute to its dispatch.
	parameter integer END_COST         = 8,  // An END header to the end of the strip.
	parameter integer DUMMY_COST       = 17, // A dummy header to the next world.
	parameter integer XFER_START_COST  = 17, // A transfer's start to its first strip buffer read.
	parameter integer XPRST_BUSY       = 24  // XPBSY stays set this long after XPRST.
)
(
	input  wire         clk_i,
	input  wire         reset_i,
	input  wire         ce_i,

	input  wire         game_start_i,        // GCLK rise, one clock.
	input  wire         xpen_i,
	input  wire         xprst_i,             // One clock.
	input  wire [4:0]   sbcmp_i,
	input  wire [1:0]   bkcol_i,
	input  wire [2:0]   parallax_scale_i,    // MiSTer stereo scale, applied to GP and MP here.

	output wire [1:0]   xpbsy_o,
	output wire         overtime_o,
	output wire         sbout_o,
	output wire [4:0]   sbcount_o,
	output wire         sbhit_o,             // Events are one clock each.
	output wire         xpend_o,
	output wire         timeerr_o,
	output wire         display_fb_o,        // Framebuffer the DP shows.
	output wire         draw_fb_o,           // Framebuffer being drawn, or last drawn.
	output wire         drawing_o,

	output wire         dram_req_o,
	output wire         dram_we_o,
	output wire [15:0]  dram_addr_o,
	output wire [1:0]   dram_be_o,
	output wire [15:0]  dram_wdata_o,
	input  wire         dram_gnt_i,
	input  wire         dram_rvalid_i,
	input  wire [15:0]  dram_rdata_i,

	output wire         vram_req_o,
	output wire         vram_we_o,
	output wire [15:0]  vram_addr_o,
	output wire [1:0]   vram_be_o,
	output wire [15:0]  vram_wdata_o,
	input  wire         vram_gnt_i,
	input  wire         vram_rvalid_i,
	input  wire [15:0]  vram_rdata_i,

	// Unit commands, shared by both units.
	output wire         bg_cmd_valid_o,
	output wire         obj_cmd_valid_o,
	output wire [4:0]   cmd_world_o,
	output wire [4:0]   cmd_strip_o,
	output wire [255:0] cmd_attr_o,          // Words 0-15 as read, GP and MP scaled.
	output wire [1:0]   cmd_obj_group_o,
	output wire         cmd_obj_wrap_o,      // The OBJ group counter has wrapped in this strip.
	output wire [2:0]   parallax_scale_o,    // Scale for per-row offsets and JP.
	output wire         unit_abort_o,        // XPRST: units drop their work.
	input  wire         bg_done_i,
	input  wire         obj_done_i,

	input  wire         bg_dram_req_i,
	input  wire         bg_dram_we_i,
	input  wire [15:0]  bg_dram_addr_i,
	input  wire [1:0]   bg_dram_be_i,
	input  wire [15:0]  bg_dram_wdata_i,
	output wire         bg_dram_gnt_o,
	output wire         bg_dram_rvalid_o,
	output wire [15:0]  bg_dram_rdata_o,
	input  wire         bg_vram_req_i,
	input  wire         bg_vram_we_i,
	input  wire [15:0]  bg_vram_addr_i,
	input  wire [1:0]   bg_vram_be_i,
	input  wire [15:0]  bg_vram_wdata_i,
	output wire         bg_vram_gnt_o,
	output wire         bg_vram_rvalid_o,
	output wire [15:0]  bg_vram_rdata_o,

	input  wire         obj_dram_req_i,
	input  wire         obj_dram_we_i,
	input  wire [15:0]  obj_dram_addr_i,
	input  wire [1:0]   obj_dram_be_i,
	input  wire [15:0]  obj_dram_wdata_i,
	output wire         obj_dram_gnt_o,
	output wire         obj_dram_rvalid_o,
	output wire [15:0]  obj_dram_rdata_o,
	input  wire         obj_vram_req_i,
	input  wire         obj_vram_we_i,
	input  wire [15:0]  obj_vram_addr_i,
	input  wire [1:0]   obj_vram_be_i,
	input  wire [15:0]  obj_vram_wdata_i,
	output wire         obj_vram_gnt_o,
	output wire         obj_vram_rvalid_o,
	output wire [15:0]  obj_vram_rdata_o,

	// Strip buffer writes: pixel i of the word lands at x + i.
	input  wire         bg_sb_valid_i,
	input  wire         bg_sb_eye_i,
	input  wire [2:0]   bg_sb_row_i,
	input  wire [9:0]   bg_sb_x_i,           // Signed.
	input  wire [15:0]  bg_sb_pixels_i,      // Pixel i in bits 2i+1:2i.
	input  wire [7:0]   bg_sb_opaque_i,
	output wire         bg_sb_ready_o,
	input  wire         obj_sb_valid_i,
	input  wire         obj_sb_eye_i,
	input  wire [2:0]   obj_sb_row_i,
	input  wire [9:0]   obj_sb_x_i,
	input  wire [15:0]  obj_sb_pixels_i,
	input  wire [7:0]   obj_sb_opaque_i,
	output wire         obj_sb_ready_o,

	input  wire         pause_req_i,         // Hold off new draws.
	output wire         pause_ready_o,       // No draw, transfer or XPRST busy time left.
	input  wire [5:0]   ss_addr_i,
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [63:0]  ss_wdata_i,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire         ss_we_i,
	output wire [63:0]  ss_rdata_o
);

	localparam [5:0] SS_ADDR = 6'd36;

	localparam [3:0]
		S_IDLE      = 4'd0,
		S_COST      = 4'd1,
		S_HEAD      = 4'd2,
		S_ATTR      = 4'd3,
		S_DISPATCH  = 4'd4,
		S_UNIT      = 4'd5,
		S_STRIP_END = 4'd6,
		S_LAST      = 4'd7,
		S_XFER      = 4'd8;

	// What S_COST leads to.
	localparam [1:0]
		N_HEAD     = 2'd0,
		N_NEXT     = 2'd1,
		N_DISPATCH = 2'd2,
		N_END      = 2'd3;

	// World attributes start at DRAM 0x3D800, halfword 0xEC00 of the DRAM.
	localparam [6:0] WORLD_BASE = 7'b1110110;

	reg [3:0]   state_q;
	reg [7:0]   cost_q;
	reg [1:0]   cost_next_q;
	reg         drawing_q;
	reg         draw_fb_q;
	reg         display_fb_q;
	reg         pend_valid_q;
	reg         pend_fb_q;
	reg         overtime_q;
	reg [7:0]   recovery_q;
	reg [4:0]   strip_q;
	reg [4:0]   world_q;
	reg [255:0] attr_q;
	reg         req_done_q;
	reg [3:0]   req_idx_q;
	reg [3:0]   rcv_idx_q;
	reg [1:0]   group_q;
	reg         obj_seen_q;
	reg [1:0]   cmd_group_q;
	reg         cmd_wrap_q;
	reg         cmd_valid_q;
	reg         unit_run_q;
	reg         active_obj_q;
	reg [2:0]   parallax_q;
	reg         clear_start_q;
	reg         xpend_q;
	reg         timeerr_q;

	reg         xfer_active_q;
	reg [4:0]   xfer_strip_q;
	reg         xfer_half_q;
	reg         xfer_fb_q;
	reg [7:0]   xfer_wait_q;
	reg         rd_eye_q;
	reg [8:0]   rd_col_q;
	reg         rd_done_q;
	reg [1:0]   rd_out_q;            // Columns read and not yet written to VRAM.
	reg [15:0]  col_q [0:1];         // Two-word queue of read columns.
	reg         col_wp_q;
	reg         col_rp_q;
	reg [1:0]   col_cnt_q;
	reg         wr_eye_q;
	reg [8:0]   wr_col_q;
	reg         sbout_q;
	reg [4:0]   sbcount_q;
	reg         sbhit_q;

	wire        sb_clear_busy_w;
	wire        sb_wr_ready_w;
	wire        sb_wr_busy_w;
	wire        sb_rd_rvalid_w;
	wire [15:0] sb_rd_data_w;

	// Memory muxes.
	wire walker_req_w    = ((state_q == S_HEAD) || (state_q == S_ATTR)) && !req_done_q;
	wire unit_dram_req_w = unit_run_q && (active_obj_q ? obj_dram_req_i : bg_dram_req_i);
	wire unit_vram_req_w = unit_run_q && (active_obj_q ? obj_vram_req_i : bg_vram_req_i);
	wire walker_gnt_w    = dram_gnt_i && walker_req_w;
	wire unit_dram_gnt_w = dram_gnt_i && !walker_req_w && unit_dram_req_w;
	wire walker_rvalid_w = dram_rvalid_i && !unit_run_q;
	wire xfer_gnt_w      = vram_gnt_i && !unit_vram_req_w;

	wire [3:0]  walker_word_w = (state_q == S_HEAD) ? 4'd0 : req_idx_q;
	wire [15:0] walker_addr_w = {WORLD_BASE, world_q, walker_word_w};

	assign dram_req_o   = walker_req_w || unit_dram_req_w;
	assign dram_we_o    = walker_req_w ? 1'b0 : (active_obj_q ? obj_dram_we_i : bg_dram_we_i);
	assign dram_addr_o  = walker_req_w ? walker_addr_w : (active_obj_q ? obj_dram_addr_i : bg_dram_addr_i);
	assign dram_be_o    = walker_req_w ? 2'b11 : (active_obj_q ? obj_dram_be_i : bg_dram_be_i);
	assign dram_wdata_o = active_obj_q ? obj_dram_wdata_i : bg_dram_wdata_i;

	assign bg_dram_gnt_o     = unit_dram_gnt_w && !active_obj_q;
	assign obj_dram_gnt_o    = unit_dram_gnt_w && active_obj_q;
	assign bg_dram_rvalid_o  = dram_rvalid_i && unit_run_q && !active_obj_q;
	assign obj_dram_rvalid_o = dram_rvalid_i && unit_run_q && active_obj_q;
	assign bg_dram_rdata_o   = dram_rdata_i;
	assign obj_dram_rdata_o  = dram_rdata_i;

	// Transfer write: FB base + x * 32 + strip, bases L0/L1/R0/R1 at 0x0000/0x4000/0x8000/0xC000.
	wire        col_ready_w  = col_cnt_q != 2'd0;
	wire [15:0] xfer_addr_w  = {wr_eye_q, xfer_fb_q, wr_col_q, xfer_strip_q};
	wire [15:0] xfer_wdata_w = col_q[col_rp_q];

	assign vram_req_o   = unit_vram_req_w || col_ready_w;
	assign vram_we_o    = unit_vram_req_w ? (active_obj_q ? obj_vram_we_i : bg_vram_we_i) : 1'b1;
	assign vram_addr_o  = unit_vram_req_w ? (active_obj_q ? obj_vram_addr_i : bg_vram_addr_i) : xfer_addr_w;
	assign vram_be_o    = unit_vram_req_w ? (active_obj_q ? obj_vram_be_i : bg_vram_be_i) : 2'b11;
	assign vram_wdata_o = unit_vram_req_w ? (active_obj_q ? obj_vram_wdata_i : bg_vram_wdata_i) : xfer_wdata_w;

	assign bg_vram_gnt_o     = vram_gnt_i && unit_vram_req_w && !active_obj_q;
	assign obj_vram_gnt_o    = vram_gnt_i && unit_vram_req_w && active_obj_q;
	assign bg_vram_rvalid_o  = vram_rvalid_i && !active_obj_q;
	assign obj_vram_rvalid_o = vram_rvalid_i && active_obj_q;
	assign bg_vram_rdata_o   = vram_rdata_i;
	assign obj_vram_rdata_o  = vram_rdata_i;

	// Strip buffer write mux.
	wire       sb_wr_valid_w  = unit_run_q && (active_obj_q ? obj_sb_valid_i : bg_sb_valid_i);
	wire       sb_wr_eye_w    = active_obj_q ? obj_sb_eye_i : bg_sb_eye_i;
	wire [2:0] sb_wr_row_w    = active_obj_q ? obj_sb_row_i : bg_sb_row_i;
	wire [9:0] sb_wr_x_w      = active_obj_q ? obj_sb_x_i : bg_sb_x_i;
	wire [15:0] sb_wr_pix_w   = active_obj_q ? obj_sb_pixels_i : bg_sb_pixels_i;
	wire [7:0] sb_wr_opaque_w = active_obj_q ? obj_sb_opaque_i : bg_sb_opaque_i;

	assign bg_sb_ready_o  = sb_wr_ready_w && unit_run_q && !active_obj_q;
	assign obj_sb_ready_o = sb_wr_ready_w && unit_run_q && active_obj_q;

	// Parallax is scaled as GP (word 2) and MP (word 5) arrive.
	wire [9:0]  gp_scaled_w;
	wire [14:0] mp_scaled_w;

	vip_stereo_scale_signed #(.WIDTH(10)) u_gp_scale (
		.value_i (dram_rdata_i[9:0]),
		.scale_i (parallax_q),
		.value_o (gp_scaled_w)
	);

	vip_stereo_scale_signed #(.WIDTH(15)) u_mp_scale (
		.value_i (dram_rdata_i[14:0]),
		.scale_i (parallax_q),
		.value_o (mp_scaled_w)
	);

	wire [15:0] attr_word_w =
		(rcv_idx_q == 4'd2) ? {dram_rdata_i[15:10], gp_scaled_w} :
		(rcv_idx_q == 4'd5) ? {dram_rdata_i[15], mp_scaled_w} : dram_rdata_i;

	wire clear_pend_w  = clear_start_q || sb_clear_busy_w;
	wire strip_idle_w  = !clear_pend_w && !sb_wr_busy_w;
	wire unit_done_w   = active_obj_q ? obj_done_i : bg_done_i;
	wire attr_obj_w    = attr_q[13:12] == 2'd3;
	wire display_next_w = (xpen_i && pend_valid_q) ? pend_fb_q : display_fb_q;
	wire start_w = game_start_i && xpen_i && !drawing_q && (recovery_q == 8'd0) && !pause_req_i;
	wire xfer_start_w = (state_q == S_STRIP_END) && strip_idle_w && !xfer_active_q;
	wire xfer_gnt_col_w = xfer_active_q && col_ready_w && xfer_gnt_w;
	wire xfer_end_w = xfer_gnt_col_w && wr_eye_q && (wr_col_q == 9'd383);
	wire walk_start_w = (state_q == S_COST) && (cost_next_q == N_HEAD) && !clear_pend_w && (cost_q <= 8'd1);

	assign xpbsy_o          = (drawing_q || (recovery_q != 8'd0)) ? {draw_fb_q, !draw_fb_q} : 2'b00;
	assign overtime_o       = overtime_q;
	assign sbout_o          = sbout_q;
	assign sbcount_o        = sbcount_q;
	assign sbhit_o          = sbhit_q;
	assign xpend_o          = xpend_q;
	assign timeerr_o        = timeerr_q;
	assign display_fb_o     = display_fb_q;
	assign draw_fb_o        = draw_fb_q;
	assign drawing_o        = drawing_q;
	assign bg_cmd_valid_o   = cmd_valid_q && !active_obj_q;
	assign obj_cmd_valid_o  = cmd_valid_q && active_obj_q;
	assign cmd_world_o      = world_q;
	assign cmd_strip_o      = strip_q;
	assign cmd_attr_o       = attr_q;
	assign cmd_obj_group_o  = cmd_group_q;
	assign cmd_obj_wrap_o   = cmd_wrap_q;
	assign parallax_scale_o = parallax_q;
	assign unit_abort_o     = xprst_i;
	assign pause_ready_o    = !drawing_q && (recovery_q == 8'd0) && !xfer_active_q;
	assign ss_rdata_o       = (ss_addr_i == SS_ADDR) ?
		{53'd0, draw_fb_q, sbcount_q, 2'd0, pend_fb_q, pend_valid_q, display_fb_q} : 64'd0;

	wire rd_accept_w = xfer_active_q && (xfer_wait_q == 8'd0) && !rd_done_q && (rd_out_q != 2'd2);

	vip_strip u_strip (
		.clk_i         (clk_i),
		.reset_i       (reset_i),
		.ce_i          (ce_i),
		.abort_i       (xprst_i),
		.clear_start_i (clear_start_q),
		.clear_color_i (bkcol_i),
		.clear_busy_o  (sb_clear_busy_w),
		.wr_half_i     (strip_q[0]),
		.wr_valid_i    (sb_wr_valid_w),
		.wr_ready_o    (sb_wr_ready_w),
		.wr_eye_i      (sb_wr_eye_w),
		.wr_row_i      (sb_wr_row_w),
		.wr_x_i        (sb_wr_x_w),
		.wr_pixels_i   (sb_wr_pix_w),
		.wr_opaque_i   (sb_wr_opaque_w),
		.wr_busy_o     (sb_wr_busy_w),
		.rd_valid_i    (rd_accept_w),
		.rd_half_i     (xfer_half_q),
		.rd_eye_i      (rd_eye_q),
		.rd_col_i      (rd_col_q),
		.rd_color_i    (bkcol_i),
		.rd_rvalid_o   (sb_rd_rvalid_w),
		.rd_data_o     (sb_rd_data_w)
	);

	// Enter a strip: restart the world walk at 31.
	task enter_strip_task;
		begin
			world_q       <= 5'd31;
			group_q       <= 2'd3;
			obj_seen_q    <= 1'b0;
			cost_q        <= STRIP_START_COST[7:0];
			cost_next_q   <= N_HEAD;
			state_q       <= S_COST;
		end
	endtask

	task next_world_task;
		begin
			req_done_q <= 1'b0;
			if (world_q == 5'd0) begin
				state_q <= S_STRIP_END;
			end else begin
				world_q <= world_q - 5'd1;
				state_q <= S_HEAD;
			end
		end
	endtask

	// Draw control and world walk.
	always @(posedge clk_i) begin
		if (reset_i) begin
			state_q       <= S_IDLE;
			cost_q        <= 8'd0;
			cost_next_q   <= N_HEAD;
			drawing_q     <= 1'b0;
			draw_fb_q     <= 1'b1;
			display_fb_q  <= 1'b0;
			pend_valid_q  <= 1'b0;
			pend_fb_q     <= 1'b0;
			overtime_q    <= 1'b0;
			recovery_q    <= 8'd0;
			strip_q       <= 5'd0;
			world_q       <= 5'd31;
			attr_q        <= 256'd0;
			req_done_q    <= 1'b0;
			req_idx_q     <= 4'd0;
			rcv_idx_q     <= 4'd0;
			group_q       <= 2'd3;
			obj_seen_q    <= 1'b0;
			cmd_group_q   <= 2'd3;
			cmd_wrap_q    <= 1'b0;
			cmd_valid_q   <= 1'b0;
			unit_run_q    <= 1'b0;
			active_obj_q  <= 1'b0;
			parallax_q    <= 3'd0;
			clear_start_q <= 1'b0;
			xpend_q       <= 1'b0;
			timeerr_q     <= 1'b0;
		end else if (ss_we_i && (ss_addr_i == SS_ADDR)) begin
			state_q       <= S_IDLE;
			drawing_q     <= 1'b0;
			overtime_q    <= 1'b0;
			recovery_q    <= 8'd0;
			cmd_valid_q   <= 1'b0;
			unit_run_q    <= 1'b0;
			clear_start_q <= 1'b0;
			xpend_q       <= 1'b0;
			timeerr_q     <= 1'b0;
			display_fb_q  <= ss_wdata_i[0];
			pend_valid_q  <= ss_wdata_i[1];
			pend_fb_q     <= ss_wdata_i[2];
			draw_fb_q     <= ss_wdata_i[10];
		end else if (ce_i) begin
			cmd_valid_q   <= 1'b0;
			clear_start_q <= 1'b0;
			xpend_q       <= 1'b0;
			timeerr_q     <= 1'b0;
			if (recovery_q != 8'd0) begin
				recovery_q <= recovery_q - 8'd1;
			end

			if (xprst_i) begin
				state_q    <= S_IDLE;
				drawing_q  <= 1'b0;
				unit_run_q <= 1'b0;
				overtime_q <= 1'b0;
				recovery_q <= XPRST_BUSY[7:0];
				draw_fb_q  <= !display_fb_q;
				// The next draw goes to bank 1, with bank 0 shown.
				pend_valid_q <= 1'b1;
				pend_fb_q    <= 1'b0;
			end else begin
				if (game_start_i && drawing_q) begin
					timeerr_q  <= 1'b1;
					overtime_q <= 1'b1;
				end
				if (game_start_i && xpen_i && pend_valid_q) begin
					display_fb_q <= pend_fb_q;
					pend_valid_q <= 1'b0;
				end
				if (start_w) begin
					drawing_q  <= 1'b1;
					draw_fb_q  <= !display_next_w;
					overtime_q <= 1'b0;
					parallax_q <= parallax_scale_i;
					strip_q    <= 5'd0;
					clear_start_q <= 1'b1;
					enter_strip_task;
				end

				case (state_q)
					S_COST: begin
						if (cost_next_q == N_HEAD && clear_pend_w) begin
							// Strip 0's walk starts once both halves are cleared.
						end else if (cost_q <= 8'd1) begin
							case (cost_next_q)
								N_HEAD: begin
									req_done_q <= 1'b0;
									state_q    <= S_HEAD;
								end
								N_NEXT:     next_world_task;
								N_DISPATCH: state_q <= S_DISPATCH;
								default:    state_q <= S_STRIP_END;
							endcase
						end else begin
							cost_q <= cost_q - 8'd1;
						end
					end

					S_HEAD: begin
						if (walker_gnt_w) begin
							req_done_q <= 1'b1;
						end
						if (walker_rvalid_w) begin
							attr_q[15:0] <= dram_rdata_i;
							state_q      <= S_COST;
							if (dram_rdata_i[6]) begin
								cost_q      <= END_COST[7:0];
								cost_next_q <= N_END;
							end else if (dram_rdata_i[15:14] == 2'b00) begin
								cost_q      <= DUMMY_COST[7:0];
								cost_next_q <= N_NEXT;
							end else if (dram_rdata_i[13:12] == 2'b11) begin
								// An OBJ world needs only its header.
								cost_q      <= WORLD_COST[7:0];
								cost_next_q <= N_DISPATCH;
							end else begin
								req_done_q <= 1'b0;
								req_idx_q  <= 4'd1;
								rcv_idx_q  <= 4'd1;
								state_q    <= S_ATTR;
							end
						end
					end

					S_ATTR: begin
						if (walker_gnt_w) begin
							if (req_idx_q == 4'd15) begin
								req_done_q <= 1'b1;
							end else begin
								req_idx_q <= req_idx_q + 4'd1;
							end
						end
						if (walker_rvalid_w) begin
							attr_q[{rcv_idx_q, 4'd0} +: 16] <= attr_word_w;
							rcv_idx_q <= rcv_idx_q + 4'd1;
							if (rcv_idx_q == 4'd15) begin
								cost_q      <= WORLD_COST[7:0];
								cost_next_q <= N_DISPATCH;
								state_q     <= S_COST;
							end
						end
					end

					S_DISPATCH: begin
						cmd_valid_q  <= 1'b1;
						unit_run_q   <= 1'b1;
						active_obj_q <= attr_obj_w;
						state_q      <= S_UNIT;
						if (attr_obj_w) begin
							// OBJ worlds take groups 3, 2, 1, 0, then wrap to 3.
							cmd_group_q <= group_q;
							cmd_wrap_q  <= obj_seen_q && (group_q == 2'd3);
							group_q     <= group_q - 2'd1;
							obj_seen_q  <= 1'b1;
						end
					end

					S_UNIT: begin
						if (unit_done_w) begin
							unit_run_q <= 1'b0;
							next_world_task;
						end
					end

					S_STRIP_END: begin
						if (xfer_start_w) begin
							state_q <= (strip_q == 5'd27) ? S_LAST : S_XFER;
						end
					end

					// Hardware draws, then writes the strip out, then draws the next.
					S_XFER: begin
						if (xfer_end_w) begin
							strip_q <= strip_q + 5'd1;
							enter_strip_task;
						end
					end

					S_LAST: begin
						if (xfer_end_w) begin
							state_q      <= S_IDLE;
							drawing_q    <= 1'b0;
							overtime_q   <= 1'b0;
							pend_valid_q <= 1'b1;
							pend_fb_q    <= draw_fb_q;
							xpend_q      <= 1'b1;
						end
					end

					default: begin
					end
				endcase
			end
		end
	end

	// Transfer of a finished strip to the framebuffer.
	integer qi;
	always @(posedge clk_i) begin
		if (reset_i) begin
			xfer_active_q <= 1'b0;
			xfer_strip_q  <= 5'd0;
			xfer_half_q   <= 1'b0;
			xfer_fb_q     <= 1'b0;
			xfer_wait_q   <= 8'd0;
			rd_eye_q      <= 1'b0;
			rd_col_q      <= 9'd0;
			rd_done_q     <= 1'b0;
			rd_out_q      <= 2'd0;
			for (qi = 0; qi < 2; qi = qi + 1) begin
				col_q[qi] <= 16'd0;
			end
			col_wp_q      <= 1'b0;
			col_rp_q      <= 1'b0;
			col_cnt_q     <= 2'd0;
			wr_eye_q      <= 1'b0;
			wr_col_q      <= 9'd0;
			sbout_q       <= 1'b0;
			sbcount_q     <= 5'd0;
			sbhit_q       <= 1'b0;
		end else if (ss_we_i && (ss_addr_i == SS_ADDR)) begin
			xfer_active_q <= 1'b0;
			xfer_wait_q   <= 8'd0;
			rd_out_q      <= 2'd0;
			col_cnt_q     <= 2'd0;
			sbout_q       <= 1'b0;
			sbhit_q       <= 1'b0;
			sbcount_q     <= ss_wdata_i[9:5];
		end else if (ce_i) begin
			sbhit_q      <= 1'b0;
			// SBCOUNT keeps the strip just written out, or 27 from the last draw,
			// until the next strip starts its world walk.
			if (walk_start_w) begin
				sbcount_q <= strip_q;
			end

			if (xprst_i) begin
				xfer_active_q <= 1'b0;
				rd_out_q      <= 2'd0;
				col_cnt_q     <= 2'd0;
				sbout_q       <= 1'b0;
			end else if (xfer_start_w) begin
				xfer_active_q <= 1'b1;
				xfer_strip_q  <= strip_q;
				xfer_half_q   <= strip_q[0];
				xfer_fb_q     <= draw_fb_q;
				xfer_wait_q   <= XFER_START_COST[7:0];
				rd_eye_q      <= 1'b0;
				rd_col_q      <= 9'd0;
				rd_done_q     <= 1'b0;
				rd_out_q      <= 2'd0;
				col_wp_q      <= 1'b0;
				col_rp_q      <= 1'b0;
				col_cnt_q     <= 2'd0;
				wr_eye_q      <= 1'b0;
				wr_col_q      <= 9'd0;
				sbout_q       <= 1'b1;
				sbhit_q       <= strip_q == sbcmp_i;
			end else if (xfer_active_q) begin
				if (xfer_wait_q != 8'd0) begin
					xfer_wait_q <= xfer_wait_q - 8'd1;
				end

				// Read columns up to two ahead of the VRAM writes.
				if (rd_accept_w) begin
					if (rd_col_q == 9'd383) begin
						rd_col_q  <= 9'd0;
						rd_eye_q  <= 1'b1;
						rd_done_q <= rd_eye_q;
					end else begin
						rd_col_q <= rd_col_q + 9'd1;
					end
				end
				rd_out_q <= rd_out_q + {1'b0, rd_accept_w} - {1'b0, xfer_gnt_col_w};

				if (sb_rd_rvalid_w) begin
					col_q[col_wp_q] <= sb_rd_data_w;
					col_wp_q        <= !col_wp_q;
				end
				if (xfer_gnt_col_w) begin
					col_rp_q <= !col_rp_q;
					if (wr_col_q == 9'd383) begin
						wr_col_q <= 9'd0;
						wr_eye_q <= 1'b1;
					end else begin
						wr_col_q <= wr_col_q + 9'd1;
					end
				end
				col_cnt_q <= col_cnt_q + {1'b0, sb_rd_rvalid_w} - {1'b0, xfer_gnt_col_w};

				if (xfer_end_w) begin
					xfer_active_q <= 1'b0;
					sbout_q       <= 1'b0;
				end
			end
		end
	end

endmodule
