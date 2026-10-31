// Copyright (c) 2026 Jamie Blanks

// Virtual Image Processor.
//
//   V810 bus -> host (registers, interrupts) -> CPU ports of the two memories
//   VRAM (framebuffers, characters): port A slots for the DP's SAM loads, the
//     XP and the CPU; port B is the SAM the display reads columns through.
//   DRAM (maps, parameters, worlds, column table, OAM): slots for the DP's
//     column table, the XP and the CPU.
//   XP draws strips into its strip buffer and writes them to the framebuffer.
//   Scanner and DP time the display; presentation turns the DP's column
//   stream into MiSTer video.
module VIP
#(
	parameter integer H_VISIBLE   = 384,
	parameter integer H_TOTAL     = 512,
	parameter integer H_SYNC_BEG  = 416,
	parameter integer H_SYNC_END  = 480,
	parameter integer V_VISIBLE   = 224,
	parameter integer V_TOTAL     = 262,
	parameter integer V_SYNC_BEG  = 240,
	parameter integer V_SYNC_END  = 244,
	parameter         FLAT_OUTPUT = 1'b0,
	// Fitted padding, time the console shows that no mechanism here explains yet:
	//   vip_xp   STRIP_START_COST, WORLD_COST, END_COST, DUMMY_COST, XFER_START_COST, XPRST_BUSY
	//   vip_bg   WORLD_EXTRA_CLOCKS, BG_ROW_SETUP, AFFINE_ROW_EXTRA
	//   vip_obj  SETUP_CLOCKS, CONT_CLOCKS, MIN_CLOCKS
	//   vip_dp   START_DELAY
	//   here     VRAM_RD_REOPEN, DRAM_XP_WAIT
	// Memory slot lengths in clocks.
	parameter integer CPU_SLOT       = 4,  // CPU bus cycle of a VIP memory access
	parameter integer DRAM_CPU_SLOT  = 3,  // DRAM time of a CPU access: one random cycle
	parameter integer VRAM_XP_SLOT   = 2,
	parameter integer VRAM_ROW_SHIFT = 7,  // 128-word VRAM rows: four framebuffer columns, one SAM load
	parameter integer VRAM_SAM_SLOT_ASSUMED = 3, // SAM load, one per 4-column group
	parameter integer DRAM_XP_SLOT   = 2,  // a page beat in the open row
	parameter integer DRAM_DP_SLOT   = 3,  // a lone random cycle
	parameter integer VRAM_NEW_ROW   = 2,  // a VRAM access outside the open row: a 4-clock random cycle
	parameter integer VRAM_OWN_RD    = 0,  // an XP read moving to another row on its own: drawing cost does not depend on the characters
	parameter integer VRAM_RD_REOPEN = 1,  // the XP's new row right after a CPU read, which overlaps the read's recovery (fitted)
	parameter integer DRAM_XP_WAIT   = 4,  // DRAM accesses a waiting XP lets the CPU take (fitted)
	parameter integer DRAM_NEW_ROW   = 1,  // a DRAM access outside the open row: a random cycle is one clock longer than a beat
	parameter integer REFRESH_PERIOD = 288 // clocks between refreshes of each memory
)
(
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        ce_i,
	input  wire        phi1_i,
	input  wire        video_ce_i,
	input  wire        video_phi1_i,
	input  wire        savestate_pause_req_i,
	input  wire        savestate_boundary_stop_i,
	output wire        savestate_pause_drain_ready_o,
	output wire        savestate_pause_scan_boundary_o,
	output wire        savestate_pause_ready_o,
	input  wire        savestate_restore_i,
	input  wire [5:0]  savestate_state_addr_i,
	input  wire [63:0] savestate_state_wdata_i,
	input  wire        savestate_state_wren_i,
	output wire [63:0] savestate_state_rdata_o,
	input  wire        savestate_mem_active_i,
	input  wire        savestate_mem_dram_i,
	input  wire [16:0] savestate_mem_addr_i,
	input  wire        savestate_mem_rden_i,
	input  wire        savestate_mem_wren_i,
	input  wire [7:0]  savestate_mem_wdata_i,
	output wire [7:0]  savestate_mem_rdata_o,

	input  wire        cs_i,
	input  wire [23:1] a_i,
	input  wire [15:0] din_i,
	input  wire [3:0]  be_i,
	input  wire [1:0]  st_i,
	input  wire        da_i,
	input  wire        mrq_i,
	input  wire        rw_i,
	input  wire        bcyst_i,
	output wire [15:0] dout_o,
	output wire        ready_o,

	output wire        irq_o,

	input  wire        scanner_spinup_i,
	input  wire        flat_dual_eye_i,
	input  wire        side_by_side_i,
	input  wire        compat_60hz_i,
	input  wire [2:0]  parallax_scale_i,
	input  wire        presentation_hlg_i,
	input  wire        brightness_table_legacy_sdr_i,
	input  wire        brightness_table_download_i,
	input  wire        brightness_table_write_i,
	input  wire [9:0]  brightness_table_addr_i,
	input  wire [15:0] brightness_table_data_i,

	output wire [1:0]  video_raw_l_o,
	output wire [1:0]  video_raw_r_o,
	output wire [7:0]  video_luma_l_o,
	output wire [7:0]  video_luma_r_o,
	output wire        video_hblank_o,
	output wire        video_vblank_o,
	output wire        video_hsync_o,
	output wire        video_vsync_o
);
	// A restore starts from reset, then the state words are written back.
	wire rst_w = reset_i || savestate_restore_i;

	// ------------------------------------------------------------------
	// Wires between units.
	// ------------------------------------------------------------------
	// Host register outputs.
	wire        dprst_w, xprst_w, disp_w, synce_w, re_w, lock_w, xpen_w;
	wire [4:0]  sbcmp_w;
	wire [31:0] brt_w, gplt_w, jplt_w;
	wire [3:0]  frmcyc_w;
	wire [39:0] spt_w;
	wire [1:0]  bkcol_w;
	wire [15:0] cta_w;
	wire        host_busy_w;

	// Scanner.
	wire        fclk_w, frame_start_w, l_sync_w, r_sync_w, scanrdy_w, scanerr_w;

	// DP.
	wire        gamestart_w, lfbend_w, rfbend_w, brt_latch_w, dp_idle_w;
	wire [3:0]  dpbsy_w;
	wire [1:0]  dp_cta_we_w;
	wire [7:0]  dp_cta_w;
	wire        pr_frame_w, pr_end_w, pr_end_eye_w, pr_word_w, pr_eye_w;
	wire [8:0]  pr_x_w;
	wire [4:0]  pr_strip_w;
	wire [15:0] pr_data_w;
	wire        pr_group_w, pr_group_eye_w;
	wire [6:0]  pr_group_idx_w;
	wire [32:0] pr_exposure_w;

	// XP.
	wire [1:0]  xpbsy_w;
	wire        overtime_w, sbout_w, sbhit_w, xpend_w, timeerr_w;
	wire [4:0]  sbcount_w;
	wire        display_fb_w, drawing_w, xp_pause_ready_w;
	wire        savestate_pause_scan_boundary_w;

	// Memory clients: CPU, XP and DP on the two controllers.
	wire        cpu_v_req, cpu_v_we, cpu_v_cont, cpu_v_hold, cpu_v_wait, cpu_v_gnt, cpu_v_rvalid;
	wire [15:0] cpu_v_addr, cpu_v_wdata, cpu_v_rdata;
	wire [1:0]  cpu_v_be;
	wire        cpu_d_req, cpu_d_we, cpu_d_cont, cpu_d_hold, cpu_d_wait, cpu_d_gnt, cpu_d_rvalid;
	wire [15:0] cpu_d_addr, cpu_d_wdata, cpu_d_rdata;
	wire [1:0]  cpu_d_be;
	wire        xp_v_req, xp_v_we, xp_v_gnt, xp_v_rvalid;
	wire [15:0] xp_v_addr, xp_v_wdata, xp_v_rdata;
	wire [1:0]  xp_v_be;
	wire        xp_d_req, xp_d_we, xp_d_gnt, xp_d_rvalid;
	wire [15:0] xp_d_addr, xp_d_wdata, xp_d_rdata;
	wire [1:0]  xp_d_be;
	wire        dp_d_req, dp_d_gnt, dp_d_rvalid;
	wire [15:0] dp_d_addr, dp_d_rdata;
	wire [15:0] sam_addr_w, sam_rdata_w;
	wire        dp_v_req, dp_v_gnt;
	wire [15:0] dp_v_addr;
	wire        vram_busy_w, dram_busy_w;
	wire        dram_lo_we_w;
	wire [15:0] dram_lo_addr_w;
	wire [7:0]  dram_lo_wdata_w;
	wire [7:0]  vram_ssm_rdata_w, dram_ssm_rdata_w;

	// Units under the XP.
	wire        bg_cmd_valid_w, obj_cmd_valid_w, cmd_obj_wrap_w, unit_abort_w;
	wire [4:0]  cmd_world_w, cmd_strip_w;
	wire [255:0] cmd_attr_w;
	wire [1:0]  cmd_obj_group_w;
	wire [2:0]  unit_parallax_w;
	wire        bg_done_w, obj_done_w;
	wire        bg_dram_req, bg_dram_we, bg_dram_gnt, bg_dram_rvalid;
	wire [15:0] bg_dram_addr, bg_dram_wdata, bg_dram_rdata;
	wire [1:0]  bg_dram_be;
	wire        bg_vram_req, bg_vram_we, bg_vram_gnt, bg_vram_rvalid;
	wire [15:0] bg_vram_addr, bg_vram_wdata, bg_vram_rdata;
	wire [1:0]  bg_vram_be;
	wire        obj_dram_req, obj_dram_we, obj_dram_gnt, obj_dram_rvalid;
	wire [15:0] obj_dram_addr, obj_dram_wdata, obj_dram_rdata;
	wire [1:0]  obj_dram_be;
	wire        obj_vram_req, obj_vram_we, obj_vram_gnt, obj_vram_rvalid;
	wire [15:0] obj_vram_addr, obj_vram_wdata, obj_vram_rdata;
	wire [1:0]  obj_vram_be;
	wire        bg_sb_valid, bg_sb_eye, bg_sb_ready;
	wire [2:0]  bg_sb_row;
	wire [9:0]  bg_sb_x;
	wire [15:0] bg_sb_pixels;
	wire [7:0]  bg_sb_opaque;
	wire        obj_sb_valid, obj_sb_eye, obj_sb_ready;
	wire [2:0]  obj_sb_row;
	wire [9:0]  obj_sb_x;
	wire [15:0] obj_sb_pixels;
	wire [7:0]  obj_sb_opaque;

	/* verilator lint_off UNUSEDSIGNAL */
	wire vram_work_wait_w, dram_work_wait_w;   // no XP work holds the memories
	/* verilator lint_on UNUSEDSIGNAL */

	// Savestate words.
	wire [63:0] ss_host_w, ss_vram_w, ss_dram_w, ss_scan_w, ss_dp_w, ss_xp_w, ss_pres_w;

	// The host takes SPT as a draw starts.
	reg drawing_q;
	always @(posedge clk_i) begin
		if (rst_w) drawing_q <= 1'b0;
		else if (ce_i) drawing_q <= drawing_w;
	end
	wire xp_start_w = drawing_w && !drawing_q;

	// ------------------------------------------------------------------
	// Host.
	// ------------------------------------------------------------------
	vip_host #(
		.WR_SLOT(CPU_SLOT), .CPU_RD_SLOT(CPU_SLOT), .RD_LAG_D(CPU_SLOT - DRAM_CPU_SLOT)
	) u_host (
		.clk_i(clk_i), .reset_i(rst_w), .ce_i(ce_i), .phi1_i(phi1_i),
		.cs_i(cs_i), .a_i(a_i), .din_i(din_i), .be_i(be_i), .st_i(st_i), .da_i(da_i),
		.mrq_i(mrq_i), .rw_i(rw_i), .bcyst_i(bcyst_i), .dout_o(dout_o), .ready_o(ready_o),
		.irq_o(irq_o),
		.vram_req_o(cpu_v_req), .vram_we_o(cpu_v_we), .vram_cont_o(cpu_v_cont), .vram_hold_o(cpu_v_hold), .vram_wait_o(cpu_v_wait), .vram_addr_o(cpu_v_addr),
		.vram_be_o(cpu_v_be), .vram_wdata_o(cpu_v_wdata), .vram_gnt_i(cpu_v_gnt),
		.vram_rvalid_i(cpu_v_rvalid), .vram_rdata_i(cpu_v_rdata),
		.dram_req_o(cpu_d_req), .dram_we_o(cpu_d_we), .dram_cont_o(cpu_d_cont), .dram_hold_o(cpu_d_hold), .dram_wait_o(cpu_d_wait), .dram_addr_o(cpu_d_addr),
		.dram_be_o(cpu_d_be), .dram_wdata_o(cpu_d_wdata), .dram_gnt_i(cpu_d_gnt),
		.dram_rvalid_i(cpu_d_rvalid), .dram_rdata_i(cpu_d_rdata),
		.ev_scanerr_i(scanerr_w), .ev_lfbend_i(lfbend_w), .ev_rfbend_i(rfbend_w),
		.ev_gamestart_i(gamestart_w), .ev_framestart_i(frame_start_w),
		.ev_sbhit_i(sbhit_w), .ev_xpend_i(xpend_w), .ev_timeerr_i(timeerr_w),
		.dpbsy_i(dpbsy_w), .scanrdy_i(scanrdy_w), .fclk_i(fclk_w),
		.xpbsy_i(xpbsy_w), .sbcount_i(sbcount_w), .sbout_i(sbout_w), .overtime_i(overtime_w),
		.dp_brt_latch_i(brt_latch_w), .dp_cta_we_i(dp_cta_we_w), .dp_cta_i({dp_cta_w, dp_cta_w}),
		.xp_start_i(xp_start_w),
		// The XP latches BKCOL itself at each strip handoff.
		.xp_bkcol_latch_i(1'b1),
		.dprst_o(dprst_w), .xprst_o(xprst_w), .disp_o(disp_w), .synce_o(synce_w),
		.re_o(re_w), .lock_o(lock_w), .xpen_o(xpen_w), .sbcmp_o(sbcmp_w),
		.brt_o(brt_w), .frmcyc_o(frmcyc_w), .spt_o(spt_w), .gplt_o(gplt_w),
		.jplt_o(jplt_w), .bkcol_o(bkcol_w), .cta_o(cta_w),
		.ss_addr_i(savestate_state_addr_i), .ss_wdata_i(savestate_state_wdata_i),
		.ss_we_i(savestate_state_wren_i), .ss_rdata_o(ss_host_w),
		.int_pending_o(), .busy_o(host_busy_w)
	);

	// ------------------------------------------------------------------
	// Memories.
	// ------------------------------------------------------------------
	vip_mem_ctrl #(
		.CPU_RD_SLOT(CPU_SLOT), .CPU_WR_SLOT(CPU_SLOT), .OPEN_ROW(1), .XP_NEW_ROW(VRAM_NEW_ROW), .XP_REOPEN_RD(VRAM_RD_REOPEN), .XP_OWN_ROW_RD(VRAM_OWN_RD),
		.XP_RD_SLOT(VRAM_XP_SLOT), .XP_WR_SLOT(VRAM_XP_SLOT), .DP_RD_SLOT(VRAM_SAM_SLOT_ASSUMED), .ROW_SHIFT(VRAM_ROW_SHIFT),
		.REFRESH_PERIOD(REFRESH_PERIOD), .SS_WORD(6'd22)
	) u_vram (
		.clk_i(clk_i), .reset_i(rst_w), .ce_i(ce_i), .phi1_i(phi1_i), .refresh_en_i(re_w),
		.dp_req_i(dp_v_req), .dp_addr_i(dp_v_addr), .dp_gnt_o(dp_v_gnt), .dp_rvalid_o(), .dp_rdata_o(),
		.xp_req_i(xp_v_req), .xp_we_i(xp_v_we), .xp_addr_i(xp_v_addr), .xp_be_i(xp_v_be),
		.xp_wdata_i(xp_v_wdata), .xp_gnt_o(xp_v_gnt), .xp_rvalid_o(xp_v_rvalid),
		.xp_rdata_o(xp_v_rdata), .xp_work_i(1'b0), .xp_work_wait_o(vram_work_wait_w),
		.cpu_req_i(cpu_v_req), .cpu_we_i(cpu_v_we), .cpu_cont_i(cpu_v_cont), .cpu_hold_i(cpu_v_hold), .cpu_wait_i(cpu_v_wait), .cpu_addr_i(cpu_v_addr), .cpu_be_i(cpu_v_be),
		.cpu_wdata_i(cpu_v_wdata), .cpu_gnt_o(cpu_v_gnt), .cpu_rvalid_o(cpu_v_rvalid),
		.cpu_rdata_o(cpu_v_rdata),
		.b_addr_i(sam_addr_w), .b_rdata_o(sam_rdata_w),
		.ssm_active_i(savestate_mem_active_i && !savestate_mem_dram_i),
		.ssm_addr_i(savestate_mem_addr_i), .ssm_we_i(savestate_mem_wren_i),
		.ssm_wdata_i(savestate_mem_wdata_i), .ssm_rdata_o(vram_ssm_rdata_w),
		.ss_addr_i(savestate_state_addr_i), .ss_wdata_i(savestate_state_wdata_i),
		.ss_we_i(savestate_state_wren_i), .ss_rdata_o(ss_vram_w),
		.lo_we_o(), .lo_addr_o(), .lo_wdata_o(),
		.busy_o(vram_busy_w), .owner_o()
	);

	vip_mem_ctrl #(
		.CPU_RD_SLOT(DRAM_CPU_SLOT), .CPU_RD_HOLD(0), .CPU_WR_SLOT(DRAM_CPU_SLOT),
		.XP_RD_SLOT(DRAM_XP_SLOT), .XP_WR_SLOT(DRAM_XP_SLOT), .DP_RD_SLOT(DRAM_DP_SLOT),
		.XP_MAX_WAIT(DRAM_XP_WAIT), .OPEN_ROW(1), .XP_NEW_ROW(DRAM_NEW_ROW),
		.REFRESH_PERIOD(REFRESH_PERIOD), .SS_WORD(6'd23)
	) u_dram (
		.clk_i(clk_i), .reset_i(rst_w), .ce_i(ce_i), .phi1_i(phi1_i), .refresh_en_i(re_w),
		.dp_req_i(dp_d_req), .dp_addr_i(dp_d_addr), .dp_gnt_o(dp_d_gnt),
		.dp_rvalid_o(dp_d_rvalid), .dp_rdata_o(dp_d_rdata),
		.xp_req_i(xp_d_req), .xp_we_i(xp_d_we), .xp_addr_i(xp_d_addr), .xp_be_i(xp_d_be),
		.xp_wdata_i(xp_d_wdata), .xp_gnt_o(xp_d_gnt), .xp_rvalid_o(xp_d_rvalid),
		.xp_rdata_o(xp_d_rdata), .xp_work_i(1'b0), .xp_work_wait_o(dram_work_wait_w),
		.cpu_req_i(cpu_d_req), .cpu_we_i(cpu_d_we), .cpu_cont_i(cpu_d_cont), .cpu_hold_i(cpu_d_hold), .cpu_wait_i(cpu_d_wait), .cpu_addr_i(cpu_d_addr), .cpu_be_i(cpu_d_be),
		.cpu_wdata_i(cpu_d_wdata), .cpu_gnt_o(cpu_d_gnt), .cpu_rvalid_o(cpu_d_rvalid),
		.cpu_rdata_o(cpu_d_rdata),
		.b_addr_i(16'd0), .b_rdata_o(),
		.ssm_active_i(savestate_mem_active_i && savestate_mem_dram_i),
		.ssm_addr_i(savestate_mem_addr_i), .ssm_we_i(savestate_mem_wren_i),
		.ssm_wdata_i(savestate_mem_wdata_i), .ssm_rdata_o(dram_ssm_rdata_w),
		.ss_addr_i(savestate_state_addr_i), .ss_wdata_i(savestate_state_wdata_i),
		.ss_we_i(savestate_state_wren_i), .ss_rdata_o(ss_dram_w),
		.lo_we_o(dram_lo_we_w), .lo_addr_o(dram_lo_addr_w), .lo_wdata_o(dram_lo_wdata_w),
		.busy_o(dram_busy_w), .owner_o()
	);

	// ------------------------------------------------------------------
	// Display.
	// ------------------------------------------------------------------
	vip_scanner u_scanner (
		.clk_i(clk_i), .reset_i(rst_w), .ce_i(ce_i), .spinup_en_i(scanner_spinup_i),
		.fclk_o(fclk_w), .frame_start_o(frame_start_w), .l_sync_o(l_sync_w), .r_sync_o(r_sync_w),
		.scanrdy_o(scanrdy_w), .scanerr_o(scanerr_w),
		.ss_addr_i(savestate_state_addr_i), .ss_wdata_i(savestate_state_wdata_i),
		.ss_we_i(savestate_state_wren_i), .ss_rdata_o(ss_scan_w)
	);

	vip_dp u_dp (
		.clk_i(clk_i), .reset_i(rst_w), .ce_i(ce_i),
		.frame_start_i(frame_start_w), .l_sync_i(l_sync_w), .r_sync_i(r_sync_w),
		.disp_i(disp_w), .synce_i(synce_w), .lock_i(lock_w), .dprst_i(dprst_w),
		.frmcyc_i(frmcyc_w), .brt_i(brt_w), .cta_i(cta_w), .displayed_fb_i(display_fb_w),
		.ev_gamestart_o(gamestart_w), .ev_lfbend_o(lfbend_w), .ev_rfbend_o(rfbend_w),
		.dpbsy_o(dpbsy_w), .brt_latch_o(brt_latch_w), .cta_we_o(dp_cta_we_w), .cta_o(dp_cta_w),
		.dram_req_o(dp_d_req), .dram_addr_o(dp_d_addr), .dram_gnt_i(dp_d_gnt),
		.dram_rvalid_i(dp_d_rvalid), .dram_rdata_i(dp_d_rdata),
		.vram_req_o(dp_v_req), .vram_addr_o(dp_v_addr), .vram_gnt_i(dp_v_gnt),
		.sam_addr_o(sam_addr_w), .sam_rdata_i(sam_rdata_w),
		.pr_frame_o(pr_frame_w), .pr_end_o(pr_end_w), .pr_end_eye_o(pr_end_eye_w),
		.pr_word_o(pr_word_w), .pr_eye_o(pr_eye_w), .pr_x_o(pr_x_w), .pr_strip_o(pr_strip_w),
		.pr_data_o(pr_data_w), .pr_group_o(pr_group_w), .pr_group_eye_o(pr_group_eye_w),
		.pr_group_idx_o(pr_group_idx_w), .pr_exposure_o(pr_exposure_w), .idle_o(dp_idle_w),
		.ss_addr_i(savestate_state_addr_i), .ss_wdata_i(savestate_state_wdata_i),
		.ss_we_i(savestate_state_wren_i), .ss_rdata_o(ss_dp_w)
	);

	vip_present #(
		.H_VISIBLE(H_VISIBLE), .H_TOTAL(H_TOTAL), .H_SYNC_BEG(H_SYNC_BEG), .H_SYNC_END(H_SYNC_END),
		.V_VISIBLE(V_VISIBLE), .V_TOTAL(V_TOTAL), .V_SYNC_BEG(V_SYNC_BEG), .V_SYNC_END(V_SYNC_END),
		.FLAT_OUTPUT(FLAT_OUTPUT)
	) u_present (
		.clk_i(clk_i), .reset_i(rst_w), .ce_i(ce_i), .video_ce_i(video_ce_i),
		.video_phi1_i(video_phi1_i),
		.pr_frame_i(pr_frame_w), .pr_end_i(pr_end_w), .pr_end_eye_i(pr_end_eye_w),
		.pr_word_i(pr_word_w), .pr_eye_i(pr_eye_w), .pr_x_i(pr_x_w), .pr_strip_i(pr_strip_w),
		.pr_data_i(pr_data_w), .pr_group_i(pr_group_w), .pr_group_eye_i(pr_group_eye_w),
		.pr_group_idx_i(pr_group_idx_w), .pr_exposure_i(pr_exposure_w),
		.flat_dual_eye_i(flat_dual_eye_i), .side_by_side_i(side_by_side_i),
		.compat_60hz_i(compat_60hz_i), .presentation_hlg_i(presentation_hlg_i),
		.brightness_table_legacy_sdr_i(brightness_table_legacy_sdr_i),
		.brightness_table_download_i(brightness_table_download_i),
		.brightness_table_write_i(brightness_table_write_i),
		.brightness_table_addr_i(brightness_table_addr_i),
		.brightness_table_data_i(brightness_table_data_i),
		.savestate_boundary_stop_i(savestate_boundary_stop_i),
		.scan_stop_point_o(savestate_pause_scan_boundary_w),
		.video_raw_l_o(video_raw_l_o), .video_raw_r_o(video_raw_r_o),
		.video_luma_l_o(video_luma_l_o), .video_luma_r_o(video_luma_r_o),
		.video_hblank_o(video_hblank_o), .video_vblank_o(video_vblank_o),
		.video_hsync_o(video_hsync_o), .video_vsync_o(video_vsync_o),
		.ss_addr_i(savestate_state_addr_i), .ss_wdata_i(savestate_state_wdata_i),
		.ss_we_i(savestate_state_wren_i), .ss_rdata_o(ss_pres_w)
	);

	// ------------------------------------------------------------------
	// Drawing.
	// ------------------------------------------------------------------
	vip_xp u_xp (
		.clk_i(clk_i), .reset_i(rst_w), .ce_i(ce_i),
		.game_start_i(gamestart_w), .xpen_i(xpen_w), .xprst_i(xprst_w), .sbcmp_i(sbcmp_w),
		.bkcol_i(bkcol_w), .parallax_scale_i(parallax_scale_i),
		.xpbsy_o(xpbsy_w), .overtime_o(overtime_w), .sbout_o(sbout_w), .sbcount_o(sbcount_w),
		.sbhit_o(sbhit_w), .xpend_o(xpend_w), .timeerr_o(timeerr_w),
		.display_fb_o(display_fb_w), .draw_fb_o(), .drawing_o(drawing_w),
		.dram_req_o(xp_d_req), .dram_we_o(xp_d_we), .dram_addr_o(xp_d_addr), .dram_be_o(xp_d_be),
		.dram_wdata_o(xp_d_wdata), .dram_gnt_i(xp_d_gnt), .dram_rvalid_i(xp_d_rvalid),
		.dram_rdata_i(xp_d_rdata),
		.vram_req_o(xp_v_req), .vram_we_o(xp_v_we), .vram_addr_o(xp_v_addr), .vram_be_o(xp_v_be),
		.vram_wdata_o(xp_v_wdata), .vram_gnt_i(xp_v_gnt), .vram_rvalid_i(xp_v_rvalid),
		.vram_rdata_i(xp_v_rdata),
		.bg_cmd_valid_o(bg_cmd_valid_w), .obj_cmd_valid_o(obj_cmd_valid_w),
		.cmd_world_o(cmd_world_w), .cmd_strip_o(cmd_strip_w), .cmd_attr_o(cmd_attr_w),
		.cmd_obj_group_o(cmd_obj_group_w), .cmd_obj_wrap_o(cmd_obj_wrap_w),
		.parallax_scale_o(unit_parallax_w), .unit_abort_o(unit_abort_w),
		.bg_done_i(bg_done_w), .obj_done_i(obj_done_w),
		.bg_dram_req_i(bg_dram_req), .bg_dram_we_i(bg_dram_we), .bg_dram_addr_i(bg_dram_addr),
		.bg_dram_be_i(bg_dram_be), .bg_dram_wdata_i(bg_dram_wdata), .bg_dram_gnt_o(bg_dram_gnt),
		.bg_dram_rvalid_o(bg_dram_rvalid), .bg_dram_rdata_o(bg_dram_rdata),
		.bg_vram_req_i(bg_vram_req), .bg_vram_we_i(bg_vram_we), .bg_vram_addr_i(bg_vram_addr),
		.bg_vram_be_i(bg_vram_be), .bg_vram_wdata_i(bg_vram_wdata), .bg_vram_gnt_o(bg_vram_gnt),
		.bg_vram_rvalid_o(bg_vram_rvalid), .bg_vram_rdata_o(bg_vram_rdata),
		.obj_dram_req_i(obj_dram_req), .obj_dram_we_i(obj_dram_we),
		.obj_dram_addr_i(obj_dram_addr), .obj_dram_be_i(obj_dram_be),
		.obj_dram_wdata_i(obj_dram_wdata), .obj_dram_gnt_o(obj_dram_gnt),
		.obj_dram_rvalid_o(obj_dram_rvalid), .obj_dram_rdata_o(obj_dram_rdata),
		.obj_vram_req_i(obj_vram_req), .obj_vram_we_i(obj_vram_we),
		.obj_vram_addr_i(obj_vram_addr), .obj_vram_be_i(obj_vram_be),
		.obj_vram_wdata_i(obj_vram_wdata), .obj_vram_gnt_o(obj_vram_gnt),
		.obj_vram_rvalid_o(obj_vram_rvalid), .obj_vram_rdata_o(obj_vram_rdata),
		.bg_sb_valid_i(bg_sb_valid), .bg_sb_eye_i(bg_sb_eye), .bg_sb_row_i(bg_sb_row),
		.bg_sb_x_i(bg_sb_x), .bg_sb_pixels_i(bg_sb_pixels), .bg_sb_opaque_i(bg_sb_opaque),
		.bg_sb_ready_o(bg_sb_ready),
		.obj_sb_valid_i(obj_sb_valid), .obj_sb_eye_i(obj_sb_eye), .obj_sb_row_i(obj_sb_row),
		.obj_sb_x_i(obj_sb_x), .obj_sb_pixels_i(obj_sb_pixels), .obj_sb_opaque_i(obj_sb_opaque),
		.obj_sb_ready_o(obj_sb_ready),
		.pause_req_i(savestate_pause_req_i), .pause_ready_o(xp_pause_ready_w),
		.ss_addr_i(savestate_state_addr_i), .ss_wdata_i(savestate_state_wdata_i),
		.ss_we_i(savestate_state_wren_i), .ss_rdata_o(ss_xp_w)
	);

	vip_bg u_bg (
		.clk_i(clk_i), .reset_i(rst_w), .ce_i(ce_i), .abort_i(unit_abort_w),
		.cmd_valid_i(bg_cmd_valid_w), .cmd_world_i(cmd_world_w), .cmd_attr_i(cmd_attr_w),
		.cmd_strip_i(cmd_strip_w), .done_o(bg_done_w),
		.gplt0_i(gplt_w[7:0]), .gplt1_i(gplt_w[15:8]), .gplt2_i(gplt_w[23:16]),
		.gplt3_i(gplt_w[31:24]), .parallax_scale_i(unit_parallax_w),
		.dram_req_o(bg_dram_req), .dram_we_o(bg_dram_we), .dram_addr_o(bg_dram_addr),
		.dram_be_o(bg_dram_be), .dram_wdata_o(bg_dram_wdata), .dram_gnt_i(bg_dram_gnt),
		.dram_rvalid_i(bg_dram_rvalid), .dram_rdata_i(bg_dram_rdata),
		.vram_req_o(bg_vram_req), .vram_we_o(bg_vram_we), .vram_addr_o(bg_vram_addr),
		.vram_be_o(bg_vram_be), .vram_wdata_o(bg_vram_wdata), .vram_gnt_i(bg_vram_gnt),
		.vram_rvalid_i(bg_vram_rvalid), .vram_rdata_i(bg_vram_rdata),
		.sb_valid_o(bg_sb_valid), .sb_eye_o(bg_sb_eye), .sb_row_o(bg_sb_row), .sb_x_o(bg_sb_x),
		.sb_pixels_o(bg_sb_pixels), .sb_opaque_o(bg_sb_opaque), .sb_ready_i(bg_sb_ready)
	);

	// The OBJ unit's JY copy takes every write to the low byte of an OAM word 2.
	wire jy_we_w = dram_lo_we_w && (dram_lo_addr_w[15:12] == 4'hf) && (dram_lo_addr_w[1:0] == 2'd2);

	vip_obj u_obj (
		.clk_i(clk_i), .reset_i(rst_w), .ce_i(ce_i), .abort_i(unit_abort_w),
		.cmd_valid_i(obj_cmd_valid_w), .cmd_world_i(cmd_world_w), .cmd_attr_i(cmd_attr_w),
		.cmd_strip_i(cmd_strip_w), .cmd_obj_group_i(cmd_obj_group_w),
		.cmd_obj_wrap_i(cmd_obj_wrap_w), .done_o(obj_done_w),
		.spt0_i(spt_w[9:0]), .spt1_i(spt_w[19:10]), .spt2_i(spt_w[29:20]), .spt3_i(spt_w[39:30]),
		.jplt0_i(jplt_w[7:0]), .jplt1_i(jplt_w[15:8]), .jplt2_i(jplt_w[23:16]),
		.jplt3_i(jplt_w[31:24]), .parallax_scale_i(unit_parallax_w),
		.jy_we_i(jy_we_w), .jy_widx_i(dram_lo_addr_w[11:2]), .jy_wdata_i(dram_lo_wdata_w),
		.dram_req_o(obj_dram_req), .dram_we_o(obj_dram_we), .dram_addr_o(obj_dram_addr),
		.dram_be_o(obj_dram_be), .dram_wdata_o(obj_dram_wdata), .dram_gnt_i(obj_dram_gnt),
		.dram_rvalid_i(obj_dram_rvalid), .dram_rdata_i(obj_dram_rdata),
		.vram_req_o(obj_vram_req), .vram_we_o(obj_vram_we), .vram_addr_o(obj_vram_addr),
		.vram_be_o(obj_vram_be), .vram_wdata_o(obj_vram_wdata), .vram_gnt_i(obj_vram_gnt),
		.vram_rvalid_i(obj_vram_rvalid), .vram_rdata_i(obj_vram_rdata),
		.sb_valid_o(obj_sb_valid), .sb_eye_o(obj_sb_eye), .sb_row_o(obj_sb_row),
		.sb_x_o(obj_sb_x), .sb_pixels_o(obj_sb_pixels), .sb_opaque_o(obj_sb_opaque),
		.sb_ready_i(obj_sb_ready)
	);

	// ------------------------------------------------------------------
	// Savestate.
	// ------------------------------------------------------------------
	assign savestate_state_rdata_o = ss_host_w | ss_vram_w | ss_dram_w | ss_scan_w | ss_dp_w |
		ss_xp_w | ss_pres_w;
	assign savestate_mem_rdata_o = savestate_mem_dram_i ? dram_ssm_rdata_w : vram_ssm_rdata_w;

	// Pause once no draw, display eye, bus beat or memory slot is in progress.
	wire pause_quiet_w = xp_pause_ready_w && dp_idle_w && !host_busy_w &&
		!vram_busy_w && !dram_busy_w;
	assign savestate_pause_drain_ready_o = savestate_pause_req_i && pause_quiet_w;
	assign savestate_pause_scan_boundary_o = savestate_pause_req_i &&
		savestate_pause_scan_boundary_w;
	assign savestate_pause_ready_o = savestate_pause_req_i && pause_quiet_w;
endmodule
