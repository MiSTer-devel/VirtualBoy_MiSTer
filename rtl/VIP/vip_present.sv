// Copyright (c) 2026 Jamie Blanks

`timescale 1ns/1ps

// MiSTer presentation: keeps what the DP displayed, taking its stream as it
// comes, and scans it out on the video enables.
//
//   DP stream ──> packer ──> eye banks (20 pixels a 40-bit word) ──> scan converter ──> video
//   group exposure ──> luma table ──> level RAM ───────────────────────┘
//
// Each eye bank holds one eye column-major, pixel index x * 224 + y, so a
// column's 224 pixels run on from the previous column's last word. The right
// eye has two banks and the left eye LEFT_BANKS (2 or 3). A frame is published
// at the right eye's end (or at the next FRAMESTART if it never ended), and the
// reader takes the newest published frame at its own frame start. With three
// left banks the next left eye never lands in a bank the reader may still be
// showing; with two the left eye can tear while the free-running raster drifts
// against the servo frame.
module vip_present
#(
	parameter integer H_VISIBLE     = 384,
	parameter integer H_TOTAL       = 1280,
	parameter integer H_SYNC_BEG    = 1056,
	parameter integer H_SYNC_END    = 1152,
	parameter integer V_VISIBLE     = 224,
	parameter integer V_TOTAL       = 312,
	parameter integer V_SYNC_BEG    = 289,
	parameter integer V_SYNC_END    = 292,
	parameter         FLAT_OUTPUT   = 1'b0,
	parameter integer LEFT_BANKS    = 3,
	parameter         LUMA_EVEN_MIF = "rtl/VIP/vip_luma_temporal_sdr22_hlg700_e1023_even.mif",
	parameter         LUMA_ODD_MIF  = "rtl/VIP/vip_luma_temporal_sdr22_hlg700_e1023_odd.mif",
	parameter         LUMA_EVEN_HEX = "rtl/VIP/vip_luma_temporal_sdr22_hlg700_e1023_even.hex",
	parameter         LUMA_ODD_HEX  = "rtl/VIP/vip_luma_temporal_sdr22_hlg700_e1023_odd.hex",
	parameter [5:0]   SS_WORD       = 6'd48   // uses SS_WORD and SS_WORD + 1
)
(
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        ce_i,             // console CE: the DP stream
	input  wire        video_ce_i,
	input  wire        video_phi1_i,

	// DP stream (see vip_dp).
	input  wire        pr_frame_i,
	input  wire        pr_end_i,
	input  wire        pr_end_eye_i,
	input  wire        pr_word_i,
	input  wire        pr_eye_i,
	input  wire [8:0]  pr_x_i,
	input  wire [4:0]  pr_strip_i,
	input  wire [15:0] pr_data_i,
	input  wire        pr_group_i,
	input  wire        pr_group_eye_i,
	input  wire [6:0]  pr_group_idx_i,
	input  wire [32:0] pr_exposure_i,

	input  wire        flat_dual_eye_i,
	input  wire        side_by_side_i,
	input  wire        compat_60hz_i,
	input  wire        presentation_hlg_i,
	input  wire        brightness_table_legacy_sdr_i,
	input  wire        brightness_table_download_i,
	input  wire        brightness_table_write_i,
	input  wire [9:0]  brightness_table_addr_i,
	input  wire [15:0] brightness_table_data_i,

	// A stop on a video CE at the frame start returns the raster to its bootstrap.
	input  wire        savestate_boundary_stop_i,
	output wire        scan_stop_point_o,

	output reg  [1:0]  video_raw_l_o,
	output reg  [1:0]  video_raw_r_o,
	output reg  [7:0]  video_luma_l_o,
	output reg  [7:0]  video_luma_r_o,
	output reg         video_hblank_o,
	output reg         video_vblank_o,
	output reg         video_hsync_o,
	output reg         video_vsync_o,

	input  wire [5:0]  ss_addr_i,
	input  wire [63:0] ss_wdata_i,
	input  wire        ss_we_i,
	output wire [63:0] ss_rdata_o
);
	localparam integer NEB     = LEFT_BANKS + 2;
	localparam integer EBW     = (NEB > 4) ? 3 : 2;
	localparam integer TAIL_AW = EBW + 8;
	localparam integer LEV_AW  = EBW + 7;
	localparam [2:0]   RIGHT0  = LEFT_BANKS[2:0];

	localparam [15:0] HV  = H_VISIBLE[15:0];
	localparam [15:0] HT  = H_TOTAL[15:0];
	localparam [15:0] HSB = H_SYNC_BEG[15:0];
	localparam [15:0] HSE = H_SYNC_END[15:0];
	localparam [15:0] VV  = V_VISIBLE[15:0];
	localparam [15:0] VT  = V_TOTAL[15:0];
	localparam [15:0] VSB = V_SYNC_BEG[15:0];
	localparam [15:0] VSE = V_SYNC_END[15:0];
	localparam [5:0]  SS_WORD1 = SS_WORD + 6'd1;

	function [1:0] pick_left_fn;
		input [1:0] a_v;
		input [1:0] b_v;
		begin
			if (LEFT_BANKS == 2) begin
				pick_left_fn = {1'b0, !a_v[0]};
			end else if ((a_v != 2'd0) && (b_v != 2'd0)) begin
				pick_left_fn = 2'd0;
			end else if ((a_v != 2'd1) && (b_v != 2'd1)) begin
				pick_left_fn = 2'd1;
			end else begin
				pick_left_fn = 2'd2;
			end
		end
	endfunction

	function [EBW-1:0] left_eb_fn;
		input [1:0] l_v;
		/* verilator lint_off UNUSEDSIGNAL */
		reg   [2:0] e_v;
		/* verilator lint_on UNUSEDSIGNAL */
		begin
			e_v        = {1'b0, l_v};
			left_eb_fn = e_v[EBW-1:0];
		end
	endfunction

	function [EBW-1:0] right_eb_fn;
		input       r_v;
		/* verilator lint_off UNUSEDSIGNAL */
		reg   [2:0] e_v;
		/* verilator lint_on UNUSEDSIGNAL */
		begin
			e_v         = RIGHT0 + {2'd0, r_v};
			right_eb_fn = e_v[EBW-1:0];
		end
	endfunction

	// Exposure to a table entry: {HLG, E[9:1]} plus the byte lane E[0].
	function [10:0] lut_index_fn;
		input [10:0] e_v;
		input        hlg_v;
		input        legacy_v;
		begin
			if (!hlg_v && legacy_v && (e_v >= 11'd255)) begin
				lut_index_fn = {1'b0, 9'd127, 1'b1};
			end else if (e_v >= 11'd1012) begin
				lut_index_fn = {hlg_v, 9'd506, 1'b0};
			end else begin
				lut_index_fn = {hlg_v, e_v[9:0]};
			end
		end
	endfunction

	function [1:0] pixel_fn;
		input [39:0] w_v;
		input [4:0]  o_v;
		begin
			pixel_fn = w_v[{o_v, 1'b0} +: 2];
		end
	endfunction

	function [7:0] luma_fn;
		input [23:0] lv_v;
		input [1:0]  p_v;
		begin
			case (p_v)
				2'd1:    luma_fn = lv_v[7:0];
				2'd2:    luma_fn = lv_v[15:8];
				2'd3:    luma_fn = lv_v[23:16];
				default: luma_fn = 8'd0;
			endcase
		end
	endfunction

	// ------------------------------------------------------------------
	// Writer: banks, packer, luma conversion (console CE).

	reg [1:0]  wl_q;           // left bank being written
	reg        wr_q;           // right bank being written
	reg [8:0]  wcl_q;          // columns completed this frame
	reg [8:0]  wcr_q;
	reg        wpub_q;         // this frame is already published
	reg [1:0]  pub_l_q;        // newest published frame
	reg        pub_r_q;
	reg [8:0]  pub_cl_q;
	reg [8:0]  pub_cr_q;
	reg        pub_ok_q;

	reg [4:0]  pk_fill_q;      // pixels waiting in pk_acc_q
	reg [37:0] pk_acc_q;
	reg [12:0] pk_addr_q;      // word they belong to

	// Two write slots per CE: a full word, then the partial word ending a column.
	reg            w1_q;
	reg [EBW-1:0]  w1_eb_q;
	reg [12:0]     w1_addr_q;
	reg [39:0]     w1_data_q;
	reg            w2_q;
	reg [EBW-1:0]  w2_eb_q;
	reg [12:0]     w2_addr_q;
	reg [39:0]     w2_data_q;

	reg [1:0]      lut_st_q;
	reg [EBW-1:0]  lj_eb_q;
	reg [6:0]      lj_grp_q;
	reg [10:0]     lj_e2_q;
	reg [10:0]     lj_e3_q;
	reg            lj_hlg_q;
	reg            lj_legacy_q;
	reg [9:0]      lut_addr_q;
	reg            lut_odd_q;
	reg [7:0]      lv1_q;
	reg [7:0]      lv2_q;
	reg            lev_we_q;
	reg [LEV_AW-1:0] lev_waddr_q;
	reg [23:0]     lev_wdata_q;

	// Reader descriptor (video side), declared here for the bank choice.
	reg [1:0]  rl_q;
	reg        rr_q;
	reg [8:0]  rcl_q;
	reg [8:0]  rcr_q;
	reg        rok_q;

	wire        pub_now_w = ce_i && !wpub_q && ((pr_end_i && pr_end_eye_i) || pr_frame_i);
	wire [1:0]  pub_l_w   = pub_now_w ? wl_q  : pub_l_q;
	wire        pub_r_w   = pub_now_w ? wr_q  : pub_r_q;
	wire [8:0]  pub_cl_w  = pub_now_w ? wcl_q : pub_cl_q;
	wire [8:0]  pub_cr_w  = pub_now_w ? wcr_q : pub_cr_q;
	wire        pub_ok_w  = pub_now_w || pub_ok_q;

	// Each eye's stream starts at column 0, strip 0 and runs in order.
	wire        pk_first_w = (pr_x_i == 9'd0) && (pr_strip_i == 5'd0);
	wire [4:0]  pk_f_w     = pk_first_w ? 5'd0 : pk_fill_q;
	wire [37:0] pk_a_w     = pk_first_w ? 38'd0 : pk_acc_q;
	wire [12:0] pk_w_w     = pk_first_w ? 13'd0 : pk_addr_q;
	wire [53:0] pk_comb_w  = {16'd0, pk_a_w} | ({38'd0, pr_data_i} << {pk_f_w, 1'b0});
	wire        pk_full_w  = (pk_f_w >= 5'd12);
	wire [4:0]  pk_fn_w    = pk_full_w ? pk_f_w - 5'd12 : pk_f_w + 5'd8;
	wire [37:0] pk_an_w    = pk_full_w ? {24'd0, pk_comb_w[53:40]} : pk_comb_w[37:0];
	wire [12:0] pk_wn_w    = pk_full_w ? pk_w_w + 13'd1 : pk_w_w;
	wire        pk_colend_w = (pr_strip_i == 5'd27);
	wire [EBW-1:0] pk_eb_w = pr_eye_i ? right_eb_fn(wr_q) : left_eb_fn(wl_q);

	wire           wr_en_w   = w1_q || w2_q;
	wire [EBW-1:0] wr_eb_w   = w1_q ? w1_eb_q   : w2_eb_q;
	wire [12:0]    wr_addr_w = w1_q ? w1_addr_q : w2_addr_q;
	wire [39:0]    wr_data_w = w1_q ? w1_data_q : w2_data_q;

	wire [10:0] lut_e1_w   = lut_index_fn(pr_exposure_i[10:0], presentation_hlg_i, brightness_table_legacy_sdr_i);
	wire [10:0] lut_e2_w   = lut_index_fn(lj_e2_q, lj_hlg_q, lj_legacy_q);
	wire [10:0] lut_e3_w   = lut_index_fn(lj_e3_q, lj_hlg_q, lj_legacy_q);
	wire [9:0]  lut_ram_addr_w = brightness_table_download_i ? brightness_table_addr_i : lut_addr_q;
	wire        lut_ram_we_w   = brightness_table_download_i && brightness_table_write_i;
	wire [7:0]  lut_even_w;
	wire [7:0]  lut_odd_w;
	wire [7:0]  lut_data_w = lut_odd_q ? lut_odd_w : lut_even_w;

	always @(posedge clk_i) begin
		if (w1_q) begin
			w1_q <= 1'b0;
		end else begin
			w2_q <= 1'b0;
		end
		lev_we_q <= 1'b0;

		if (reset_i || (ss_we_i && (ss_addr_i == SS_WORD))) begin
			// Banks are not saved; show black until a new frame is published.
			wl_q      <= 2'd0;
			wr_q      <= 1'b0;
			wcl_q     <= 9'd0;
			wcr_q     <= 9'd0;
			wpub_q    <= 1'b0;
			pub_l_q   <= 2'd0;
			pub_r_q   <= 1'b0;
			pub_cl_q  <= 9'd0;
			pub_cr_q  <= 9'd0;
			pub_ok_q  <= 1'b0;
			pk_fill_q <= 5'd0;
			pk_acc_q  <= 38'd0;
			pk_addr_q <= 13'd0;
			w1_q      <= 1'b0;
			w2_q      <= 1'b0;
			lut_st_q  <= 2'd0;
		end else begin
			if (ce_i) begin
				if (pub_now_w) begin
					pub_l_q  <= wl_q;
					pub_r_q  <= wr_q;
					pub_cl_q <= wcl_q;
					pub_cr_q <= wcr_q;
					pub_ok_q <= 1'b1;
					wpub_q   <= 1'b1;
				end
				if (pr_frame_i) begin
					wl_q   <= pick_left_fn(pub_l_w, rl_q);
					wr_q   <= !pub_r_w;
					wcl_q  <= 9'd0;
					wcr_q  <= 9'd0;
					wpub_q <= 1'b0;
				end

				if (pr_word_i) begin
					pk_fill_q <= pk_fn_w;
					pk_acc_q  <= pk_an_w;
					pk_addr_q <= pk_wn_w;
					if (pk_full_w) begin
						w1_q      <= 1'b1;
						w1_eb_q   <= pk_eb_w;
						w1_addr_q <= pk_w_w;
						w1_data_q <= pk_comb_w[39:0];
					end
					if (pk_colend_w) begin
						if (pr_eye_i) begin
							wcr_q <= pr_x_i + 9'd1;
						end else begin
							wcl_q <= pr_x_i + 9'd1;
						end
						if (pk_fn_w != 5'd0) begin
							w2_q      <= 1'b1;
							w2_eb_q   <= pk_eb_w;
							w2_addr_q <= pk_wn_w;
							w2_data_q <= {2'd0, pk_an_w};
						end
					end
				end
			end

			// Three table reads per group, one per CE so each read has settled.
			if (brightness_table_download_i) begin
				lut_st_q <= 2'd0;
			end else if (ce_i) begin
				case (lut_st_q)
					2'd0: begin
						if (pr_group_i) begin
							lj_eb_q     <= pr_group_eye_i ? right_eb_fn(wr_q) : left_eb_fn(wl_q);
							lj_grp_q    <= pr_group_idx_i;
							lj_e2_q     <= pr_exposure_i[21:11];
							lj_e3_q     <= pr_exposure_i[32:22];
							lj_hlg_q    <= presentation_hlg_i;
							lj_legacy_q <= brightness_table_legacy_sdr_i;
							lut_addr_q  <= lut_e1_w[10:1];
							lut_odd_q   <= lut_e1_w[0];
							lut_st_q    <= 2'd1;
						end
					end
					2'd1: begin
						lv1_q      <= lut_data_w;
						lut_addr_q <= lut_e2_w[10:1];
						lut_odd_q  <= lut_e2_w[0];
						lut_st_q   <= 2'd2;
					end
					2'd2: begin
						lv2_q      <= lut_data_w;
						lut_addr_q <= lut_e3_w[10:1];
						lut_odd_q  <= lut_e3_w[0];
						lut_st_q   <= 2'd3;
					end
					default: begin
						lev_we_q    <= 1'b1;
						lev_waddr_q <= {lj_eb_q, lj_grp_q};
						lev_wdata_q <= {lut_data_w, lv2_q, lv1_q};
						lut_st_q    <= 2'd0;
					end
				endcase
			end
		end
	end

	// ------------------------------------------------------------------
	// Reader: raster and fetch (video CE).

	reg        sbs_q;
	reg        compat_q;
	reg        started_q;
	reg [15:0] fx_q;           // next pixel to fetch
	reg [15:0] fy_q;
	reg [15:0] px_q;           // pixel fetched at the last video CE
	reg [15:0] py_q;
	reg [12:0] cw_q;           // packed word and offset of the fetch pixel
	reg [4:0]  co_q;
	reg [12:0] rbw_q;          // same for x = 0 of line fy
	reg [4:0]  rbo_q;

	reg [12:0] p_w_q;
	reg [4:0]  p_o_q;
	reg [6:0]  p_g_q;
	reg        p_eye_q;
	reg        p_visl_q;
	reg        p_visr_q;

	reg            fq_d_q;     // last clock read for the fetch pixel
	reg            rq_d_q;     // last clock read for the right eye
	reg [EBW-1:0]  rsel_eb_q;
	reg            rsel_tail_q;
	reg [39:0]     fhold_q;
	reg [23:0]     fhold_lev_q;
	reg [39:0]     rhold_q;
	reg [23:0]     rhold_lev_q;

	wire [15:0] hvis_w   = sbs_q ? {HV[14:0], 1'b0} : HV;
	wire [15:0] htb_w    = compat_q ? 16'd1270 : HT;
	wire [15:0] htot_w   = sbs_q ? {htb_w[14:0], 1'b0} : htb_w;
	wire [15:0] hsb_w    = sbs_q ? {HSB[14:0], 1'b0} : HSB;
	wire [15:0] hse_w    = sbs_q ? {HSE[14:0], 1'b0} : HSE;
	wire [15:0] vtot_w   = compat_q ? 16'd262 : VT;
	wire [15:0] vsb_w    = compat_q ? 16'd243 : VSB;
	wire [15:0] vse_w    = compat_q ? 16'd246 : VSE;
	wire [15:0] last_x_w = htot_w - 16'd1;
	wire [15:0] last_y_w = vtot_w - 16'd1;

	wire        f_start_w = (fx_q == 16'd0) && (fy_q == 16'd0);
	wire        f_last_w  = (fx_q == last_x_w) && (fy_q == last_y_w);
	wire        f_eye_w   = sbs_q && (fx_q >= HV);
	wire [15:0] f_nx_w    = f_eye_w ? fx_q - HV : fx_q;
	wire        f_vis_w   = (fx_q < hvis_w) && (fy_q < VV) && (f_nx_w < 16'd384) && (fy_q < 16'd224);
	wire        f_okl_w   = rok_q && (f_nx_w[8:0] < rcl_q);
	wire        f_okr_w   = rok_q && (f_nx_w[8:0] < rcr_q);
	wire [EBW-1:0] f_eb_w = f_eye_w ? right_eb_fn(rr_q) : left_eb_fn(rl_q);

	// x + 1 moves 224 pixels: 11 words and 4 pixels.
	wire        c_wrap_w  = (co_q >= 5'd16);
	wire [12:0] c_nw_w    = cw_q + (c_wrap_w ? 13'd12 : 13'd11);
	wire [4:0]  c_no_w    = c_wrap_w ? co_q - 5'd16 : co_q + 5'd4;
	// y + 1 moves one pixel.
	wire        r_wrap_w  = (rbo_q == 5'd19);
	wire [12:0] r_nw_w    = r_wrap_w ? rbw_q + 13'd1 : rbw_q;
	wire [4:0]  r_no_w    = r_wrap_w ? 5'd0 : rbo_q + 5'd1;

	// The fetch reads in the CE clock, the right eye in the following phi1 clock.
	wire           req_right_w = !video_ce_i && video_phi1_i && !sbs_q;
	wire [12:0]    req_w_w     = req_right_w ? p_w_q : cw_q;
	wire [EBW-1:0] req_eb_w    = req_right_w ? right_eb_fn(rr_q) : f_eb_w;
	wire [6:0]     req_g_w     = req_right_w ? p_g_q : f_nx_w[8:2];

	wire [39:0] main_q_w [0:NEB-1];
	wire [39:0] tail_q_w;
	wire [23:0] lev_q_w;
	reg  [39:0] main_sel_t;
	integer     sel_i;
	always @* begin
		main_sel_t = 40'd0;
		for (sel_i = 0; sel_i < NEB; sel_i = sel_i + 1) begin
			if (rsel_eb_q == sel_i[EBW-1:0]) begin
				main_sel_t = main_q_w[sel_i];
			end
		end
	end
	wire [39:0] ram_q_w = rsel_tail_q ? tail_q_w : main_sel_t;

	wire [39:0] fword_w = fq_d_q ? ram_q_w : fhold_q;
	wire [23:0] flev_w  = fq_d_q ? lev_q_w : fhold_lev_q;
	wire [39:0] rword_w = rq_d_q ? ram_q_w : rhold_q;
	wire [23:0] rlev_w  = rq_d_q ? lev_q_w : rhold_lev_q;
	wire [1:0]  fpix_w  = pixel_fn(fword_w, p_o_q);
	wire [1:0]  rpix_w  = pixel_fn(rword_w, p_o_q);
	wire        left_only_w = FLAT_OUTPUT && !flat_dual_eye_i;

	wire        restart_w = reset_i || (side_by_side_i != sbs_q) ||
		(video_ce_i && savestate_boundary_stop_i && f_start_w);

	assign scan_stop_point_o = f_start_w;
	assign ss_rdata_o = (ss_addr_i == SS_WORD)  ? {fy_q, fx_q, py_q, px_q} :
	                    (ss_addr_i == SS_WORD1) ? {25'd0, compat_q, sbs_q, started_q, rbo_q, rbw_q, co_q, cw_q} :
	                    64'd0;

	always @(posedge clk_i) begin
		fq_d_q      <= video_ce_i;
		rq_d_q      <= req_right_w;
		rsel_eb_q   <= req_eb_w;
		rsel_tail_q <= req_w_w[12];
		if (fq_d_q) begin
			fhold_q     <= ram_q_w;
			fhold_lev_q <= lev_q_w;
		end
		if (rq_d_q) begin
			rhold_q     <= ram_q_w;
			rhold_lev_q <= lev_q_w;
		end

		if (restart_w || (ss_we_i && (ss_addr_i == SS_WORD))) begin
			if (restart_w) begin
				sbs_q     <= side_by_side_i;
				compat_q  <= compat_60hz_i;
				started_q <= 1'b0;
				fx_q      <= 16'd0;
				fy_q      <= 16'd0;
				px_q      <= 16'd0;
				py_q      <= 16'd0;
				cw_q      <= 13'd0;
				co_q      <= 5'd0;
				rbw_q     <= 13'd0;
				rbo_q     <= 5'd0;
			end else begin
				fy_q <= ss_wdata_i[63:48];
				fx_q <= ss_wdata_i[47:32];
				py_q <= ss_wdata_i[31:16];
				px_q <= ss_wdata_i[15:0];
			end
			rl_q           <= reset_i ? 2'd0 : pub_l_w;
			rr_q           <= reset_i ? 1'b0 : pub_r_w;
			rcl_q          <= reset_i ? 9'd0 : pub_cl_w;
			rcr_q          <= reset_i ? 9'd0 : pub_cr_w;
			rok_q          <= !reset_i && !ss_we_i && pub_ok_w;
			video_hblank_o <= 1'b1;
			video_vblank_o <= 1'b1;
			video_hsync_o  <= 1'b0;
			video_vsync_o  <= 1'b0;
			video_raw_l_o  <= 2'd0;
			video_raw_r_o  <= 2'd0;
			video_luma_l_o <= 8'd0;
			video_luma_r_o <= 8'd0;
		end else if (ss_we_i && (ss_addr_i == SS_WORD1)) begin
			cw_q      <= ss_wdata_i[12:0];
			co_q      <= ss_wdata_i[17:13];
			rbw_q     <= ss_wdata_i[30:18];
			rbo_q     <= ss_wdata_i[35:31];
			started_q <= ss_wdata_i[36];
			sbs_q     <= ss_wdata_i[37];
			compat_q  <= ss_wdata_i[38];
		end else if (video_ce_i) begin
			// Output the pixel fetched at the previous video CE.
			if (!started_q) begin
				started_q      <= 1'b1;
				video_hblank_o <= 1'b1;
				video_vblank_o <= 1'b1;
				video_hsync_o  <= 1'b0;
				video_vsync_o  <= 1'b0;
				video_raw_l_o  <= 2'd0;
				video_raw_r_o  <= 2'd0;
				video_luma_l_o <= 8'd0;
				video_luma_r_o <= 8'd0;
			end else begin
				video_hblank_o <= (px_q >= hvis_w);
				video_vblank_o <= (py_q >= VV);
				video_hsync_o  <= (px_q >= hsb_w) && (px_q < hse_w);
				video_vsync_o  <= (py_q >= vsb_w) && (py_q < vse_w);
				video_raw_l_o  <= 2'd0;
				video_raw_r_o  <= 2'd0;
				video_luma_l_o <= 8'd0;
				video_luma_r_o <= 8'd0;
				if (sbs_q) begin
					if (!p_eye_q && p_visl_q) begin
						video_raw_l_o  <= fpix_w;
						video_luma_l_o <= luma_fn(flev_w, fpix_w);
					end
					if (p_eye_q && p_visr_q) begin
						video_raw_r_o  <= fpix_w;
						video_luma_r_o <= luma_fn(flev_w, fpix_w);
					end
				end else begin
					if (p_visl_q) begin
						video_raw_l_o  <= fpix_w;
						video_luma_l_o <= luma_fn(flev_w, fpix_w);
					end
					if (p_visr_q) begin
						video_raw_r_o  <= rpix_w;
						video_luma_r_o <= left_only_w ? 8'd0 : luma_fn(rlev_w, rpix_w);
					end
				end
			end

			px_q     <= fx_q;
			py_q     <= fy_q;
			p_w_q    <= cw_q;
			p_o_q    <= co_q;
			p_g_q    <= f_nx_w[8:2];
			p_eye_q  <= f_eye_w;
			p_visl_q <= f_vis_w && (sbs_q ? (!f_eye_w && f_okl_w) : f_okl_w);
			p_visr_q <= f_vis_w && (sbs_q ? (f_eye_w && f_okr_w) : f_okr_w);

			if (fx_q == last_x_w) begin
				fx_q <= 16'd0;
				if (fy_q == last_y_w) begin
					fy_q  <= 16'd0;
					rbw_q <= 13'd0;
					rbo_q <= 5'd0;
					cw_q  <= 13'd0;
					co_q  <= 5'd0;
				end else begin
					fy_q  <= fy_q + 16'd1;
					rbw_q <= r_nw_w;
					rbo_q <= r_no_w;
					cw_q  <= r_nw_w;
					co_q  <= r_no_w;
				end
			end else begin
				fx_q <= fx_q + 16'd1;
				if (sbs_q && (fx_q + 16'd1 == HV)) begin
					cw_q <= rbw_q;
					co_q <= rbo_q;
				end else begin
					cw_q <= c_nw_w;
					co_q <= c_no_w;
				end
			end

			// The last fetch of a frame is in blanking: take the newest frame there.
			if (f_last_w) begin
				compat_q <= compat_60hz_i;
				rl_q     <= pub_l_w;
				rr_q     <= pub_r_w;
				rcl_q    <= pub_cl_w;
				rcr_q    <= pub_cr_w;
				rok_q    <= pub_ok_w;
			end
		end
	end

	// ------------------------------------------------------------------
	// Memories.

	genvar eb_g;
	generate
		for (eb_g = 0; eb_g < NEB; eb_g = eb_g + 1) begin : g_bank
			cache_ram_dp #(
				.ADDR_WIDTH (12),
				.DATA_WIDTH (40)
			) u_main (
				.clk_i     (clk_i),
				.addr_a_i  (wr_addr_w[11:0]),
				.wren_a_i  (wr_en_w && !wr_addr_w[12] && ({{(32-EBW){1'b0}}, wr_eb_w} == eb_g)),
				.wdata_a_i (wr_data_w),
				/* verilator lint_off PINCONNECTEMPTY */
				.q_a_o     (),
				/* verilator lint_on PINCONNECTEMPTY */
				.addr_b_i  (req_w_w[11:0]),
				.wren_b_i  (1'b0),
				.wdata_b_i (40'd0),
				.q_b_o     (main_q_w[eb_g])
			);
		end
	endgenerate

	// Words 4096 and up of every bank (205 per bank).
	cache_ram_dp #(
		.ADDR_WIDTH (TAIL_AW),
		.DATA_WIDTH (40)
	) u_tail (
		.clk_i     (clk_i),
		.addr_a_i  ({wr_eb_w, wr_addr_w[7:0]}),
		.wren_a_i  (wr_en_w && wr_addr_w[12]),
		.wdata_a_i (wr_data_w),
		/* verilator lint_off PINCONNECTEMPTY */
		.q_a_o     (),
		/* verilator lint_on PINCONNECTEMPTY */
		.addr_b_i  ({req_eb_w, req_w_w[7:0]}),
		.wren_b_i  (1'b0),
		.wdata_b_i (40'd0),
		.q_b_o     (tail_q_w)
	);

	// Luma levels {code 3, code 2, code 1} per bank and group.
	cache_ram_dp #(
		.ADDR_WIDTH (LEV_AW),
		.DATA_WIDTH (24)
	) u_levels (
		.clk_i     (clk_i),
		.addr_a_i  (lev_waddr_q),
		.wren_a_i  (lev_we_q),
		.wdata_a_i (lev_wdata_q),
		/* verilator lint_off PINCONNECTEMPTY */
		.q_a_o     (),
		/* verilator lint_on PINCONNECTEMPTY */
		.addr_b_i  ({req_eb_w, req_g_w}),
		.wren_b_i  (1'b0),
		.wdata_b_i (24'd0),
		.q_b_o     (lev_q_w)
	);

	cache_ram #(
		.ADDR_WIDTH    (10),
		.DATA_WIDTH    (8),
		.MEM_INIT_FILE (LUMA_EVEN_MIF),
		.SIM_INIT_FILE (LUMA_EVEN_HEX)
	) u_luma_even (
		.clk_i   (clk_i),
		.addr_i  (lut_ram_addr_w),
		.wren_i  (lut_ram_we_w),
		.wdata_i (brightness_table_data_i[7:0]),
		.q_o     (lut_even_w)
	);

	cache_ram #(
		.ADDR_WIDTH    (10),
		.DATA_WIDTH    (8),
		.MEM_INIT_FILE (LUMA_ODD_MIF),
		.SIM_INIT_FILE (LUMA_ODD_HEX)
	) u_luma_odd (
		.clk_i   (clk_i),
		.addr_i  (lut_ram_addr_w),
		.wren_i  (lut_ram_we_w),
		.wdata_i (brightness_table_data_i[15:8]),
		.q_o     (lut_odd_w)
	);

endmodule
