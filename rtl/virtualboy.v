// Copyright (c) 2026 Jamie Blanks

// Structural top of the Virtual Boy console: the NVC CPU package, VIP video,
// VSU sound, WRAM, the rumble accessory and the cheat engine. Everything
// outside the console (SDRAM, DDR3, HPS, presentation) attaches through the
// external bus and video/audio ports.
module virtualboy
#(
	parameter [31:0]  CPU_RESET_PC          = 32'hffff_fff0,
	parameter [31:0]  CPU_RESET_PSW         = 32'h0000_8000,
	parameter         PAD_DISCONNECTED      = 1'b0,
	parameter [15:0]  PAD_DISCONNECTED_DATA = 16'h0000,
	parameter         LINK_DISCONNECTED     = 1'b1,
	parameter integer VIDEO_H_VISIBLE       = 384,
	parameter integer VIDEO_H_TOTAL         = 1280,
	parameter integer VIDEO_H_SYNC_BEG      = 1056,
	parameter integer VIDEO_H_SYNC_END      = 1152,
	parameter integer VIDEO_V_VISIBLE       = 224,
	parameter integer VIDEO_V_TOTAL         = 312,
	parameter integer VIDEO_V_SYNC_BEG      = 289,
	parameter integer VIDEO_V_SYNC_END      = 292,
	parameter         VIDEO_FLAT_OUTPUT     = 1'b1
)
(
	input  wire         clk_i,
	input  wire         reset_i,
	input  wire         ce_i,
	input  wire         video_ce_i,
	input  wire         savestate_pause_req_i,
	output wire         savestate_pause_drain_ready_o,
	output wire         savestate_pause_ready_o,
	input  wire         savestate_restore_i,
	input  wire [6:0]   savestate_state_addr_i,
	input  wire [63:0]  savestate_state_wdata_i,
	input  wire         savestate_state_wren_i,
	output reg  [63:0]  savestate_state_rdata_o,
	input  wire         savestate_mem_active_i,
	input  wire [2:0]   savestate_mem_type_i,
	input  wire [24:0]  savestate_mem_addr_i,
	input  wire         savestate_mem_rden_i,
	input  wire         savestate_mem_wren_i,
	input  wire [7:0]   savestate_mem_wdata_i,
	output reg  [7:0]   savestate_mem_rdata_o,
	input  wire         cheat_clear_i,
	input  wire [128:0] cheat_code_i,

	input  wire [15:0]  pad_buttons_i,
	input  wire         pad_serial_enable_i,
	input  wire         pad_serial_i,
	input  wire         cart_irq_i,
	input  wire         rumble_enable_i,
	input  wire         link_comcnt_i,
	input  wire         link_clk_i,
	input  wire         link_rx_i,
	input  wire         link_sync_i,
	input  wire         flat_dual_eye_i,
	input  wire         side_by_side_i,
	input  wire         compat_60hz_i,
	input  wire [2:0]   parallax_scale_i,
	input  wire         scanner_spinup_i,
	input  wire         presentation_hlg_i,
	input  wire         brightness_table_legacy_sdr_i,
	input  wire         brightness_table_download_i,
	input  wire         brightness_table_write_i,
	input  wire [9:0]   brightness_table_addr_i,
	input  wire [15:0]  brightness_table_data_i,

	output wire         pad_latch_o,
	output wire         pad_clock_o,
	output wire         link_comcnt_o,
	output wire         link_clk_o,
	output wire         link_tx_o,
	output wire         link_sync_o,
	output wire [15:0]  rumble_o,

	output wire [23:1]  ext_a_o,
	output wire [15:0]  ext_dout_o,
	output wire         ext_dout_oe_o,
	output wire [3:0]   ext_be_o,
	output wire [1:0]   ext_st_o,
	output wire         ext_da_o,
	output wire         ext_mrq_o,
	output wire         ext_rw_o,
	output wire         wram_cs_o,
	output wire         cart_exp_cs_o,
	output wire         cart_ram_cs_o,
	output wire         cart_rom_cs_o,
	output wire         ext_cycle_tag_o,
	input  wire [15:0]  ext_din_i,
	input  wire         ext_ready_i,
	output wire         cart_refresh_window_o,

	output wire [1:0]   video_raw_l_o,
	output wire [1:0]   video_raw_r_o,
	output wire [7:0]   video_luma_l_o,
	output wire [7:0]   video_luma_r_o,
	output wire         video_hblank_o,
	output wire         video_vblank_o,
	output wire         video_hsync_o,
	output wire         video_vsync_o,

	output wire [15:0]  audio_l_o,
	output wire [15:0]  audio_r_o,
	output wire         audio_sample_valid_o,
	output wire         timer_tick_o,
	output wire         timer_zero_o,
	output wire         timer_irq_o,
	output wire         cpu_bus_mrq_o,
	output wire         cpu_bus_ready_o,
	output wire [2:0]   cpu_bus_region_o,
	output wire         cpu_bus_da_o,
	output wire         cpu_bus_rw_o,
	output wire [1:0]   cpu_bus_st_o,
	output wire         cpu_bus_bcyst_o,

	input  wire [4:0]   dbg_reg_addr_i,
	output wire [31:0]  dbg_reg_data_o,
	output wire [31:0]  dbg_pc_o,
	output wire [31:0]  dbg_psw_o,
	output wire         trace_valid_o,
	output wire [31:0]  trace_pc_o,
	output wire [15:0]  trace_insn_o,
	output wire         cpu_halted_o,
	output wire         cpu_illegal_o
);

	// NVC package pins, split per the FPGA bidirectional-pin convention.
	wire [26:1] cpu_pin_a_w;
	wire [15:0] cpu_pin_dout_w;
	wire        cpu_pin_dout_oe_w;
	wire [3:0]  cpu_pin_be_w;
	wire [1:0]  cpu_pin_st_w;
	wire        cpu_pin_da_w;
	wire        cpu_pin_mrq_w;
	wire        cpu_pin_rw_w;
	wire        cpu_pin_bcyst_w;
	wire        cpu_pin_cycle_tag_w;
	wire        cpu_dev_mrq_w;
	wire        cpu_vip_mrq_w;
	wire [15:0] cpu_bus_din_raw_w;
	wire [15:0] cpu_bus_din_w;
	wire        cpu_bus_ready_w;

	wire        misc_link_sync_w;
	wire        link_tx_done_w;
	wire [7:0]  link_tx_byte_w;

	// Bit 0 carries the controller battery-low signal. Normal MiSTer joystick
	// input leaves it low; VBTAS playback can provide the recorded state.
	wire [15:0] pad_sample_w = PAD_DISCONNECTED ? PAD_DISCONNECTED_DATA :
		{pad_buttons_i[15:2], 1'b1, pad_buttons_i[0]};
	// An open cable leaves COMCNT and COMCLK high. CC-Sig loops back locally.
	wire        link_comcnt_eff_w = LINK_DISCONNECTED ? 1'b1 : link_comcnt_i;
	wire        link_clk_eff_w    = LINK_DISCONNECTED ? 1'b1 : link_clk_i;
	wire        link_rx_eff_w     = LINK_DISCONNECTED ? 1'b0 : link_rx_i;
	wire        link_sync_eff_w   = LINK_DISCONNECTED ? misc_link_sync_w : link_sync_i;

	wire        vip_bus_cs_w;
	wire        vsu_bus_cs_w;
	wire        exp_bus_cs_w;
	wire        wram_bus_cs_w;
	wire        sram_bus_cs_w;
	wire        rom_bus_cs_w;

	wire [15:0] vip_bus_dout_w;
	wire        vip_bus_ready_w;
	wire        vip_irq_w;

	wire [15:0] vsu_bus_dout_w;

	reg         cpu_phi1_q;
	reg         video_phi1_q;
	wire        cpu_phi1_w   = cpu_phi1_q;
	wire        video_phi1_w = video_phi1_q;

	// Savestate pause: drain the CPU first, then stop video at a VIP scan
	// boundary so scanout ownership is committed before the state is frozen.
	wire        savestate_cpu_ready_w;
	wire        savestate_vip_ready_w;
	wire        savestate_vip_drain_ready_w;
	wire        savestate_vip_scan_boundary_w;
	reg         savestate_core_drain_q;
	reg         savestate_video_drain_q;
	wire        savestate_boundary_stop_w = savestate_pause_req_i &&
		savestate_core_drain_q && !savestate_video_drain_q &&
		video_ce_i && savestate_vip_scan_boundary_w;
	wire        core_ce_w       = ce_i && !savestate_core_drain_q;
	wire        core_video_ce_w = video_ce_i && !savestate_video_drain_q;

	// Savestate scalar-state address windows:
	//   0-15  CPU        16-37 VUE I/O     38    wait control
	//   39-63 VSU        64-112 VIP
	// VUE rows 21-33 belong to the rumble accessory; row 23 also holds WCR.
	wire [63:0] savestate_cpu_state_rdata_w;
	wire [63:0] savestate_io_state_rdata_w;
	wire [63:0] savestate_rumble_state_rdata_w;
	wire [63:0] savestate_vue_state_rdata_w =
		savestate_io_state_rdata_w | savestate_rumble_state_rdata_w;
	wire [63:0] savestate_wait_state_rdata_w;
	wire [63:0] savestate_vsu_state_rdata_w;
	wire [63:0] savestate_vip_state_rdata_w;
	wire [7:0]  savestate_cpu_mem_rdata_w;
	wire [7:0]  savestate_vsu_mem_rdata_w;
	wire [7:0]  savestate_vip_mem_rdata_w;
	wire [7:0]  wram_peek_u_w;
	wire [7:0]  wram_peek_l_w;
	wire        savestate_cpu_mem_active_w = savestate_mem_active_i &&
		(savestate_mem_type_i == 3'd5);
	wire        savestate_vsu_mem_active_w = savestate_mem_active_i &&
		(savestate_mem_type_i == 3'd4);
	wire        savestate_wram_mem_active_w = savestate_mem_active_i &&
		(savestate_mem_type_i == 3'd0);
	wire        savestate_vip_mem_active_w = savestate_mem_active_i &&
		((savestate_mem_type_i == 3'd1) ||
		 (savestate_mem_type_i == 3'd2));
	wire        savestate_cpu_state_wren_w = savestate_state_wren_i &&
		(savestate_state_addr_i < 7'd16);
	wire        savestate_vue_state_wren_w = savestate_state_wren_i &&
		(savestate_state_addr_i >= 7'd16) &&
		(savestate_state_addr_i < 7'd38);
	wire        savestate_wait_state_wren_w = savestate_state_wren_i &&
		(savestate_state_addr_i == 7'd38);
	wire        savestate_vsu_state_wren_w = savestate_state_wren_i &&
		(savestate_state_addr_i >= 7'd39) &&
		(savestate_state_addr_i < 7'd64);
	wire        savestate_vip_state_wren_w = savestate_state_wren_i &&
		(savestate_state_addr_i >= 7'd64) &&
		(savestate_state_addr_i < 7'd113);
	wire [4:0]  savestate_vue_state_addr_w =
		savestate_state_addr_i[4:0] - 5'd16;
	wire [4:0]  savestate_vsu_state_addr_w =
		savestate_state_addr_i[4:0] - 5'd7;
	wire [5:0]  savestate_vip_state_addr_w =
		savestate_state_addr_i[5:0];

	// WRAM byte lanes, shared between the CPU bus and the savestate walker.
	wire        wram_bus_write_w = wram_bus_cs_w && cpu_pin_da_w &&
		!cpu_pin_rw_w;
	wire        wram_upper_lane_w = (cpu_pin_be_w != 4'b1110);
	wire        wram_lower_lane_w = (cpu_pin_be_w != 4'b1101);
	wire [14:0] wram_bus_addr_w = cpu_pin_a_w[15:1];
	wire [14:0] wram_addr_w = savestate_wram_mem_active_w ?
		savestate_mem_addr_i[15:1] : wram_bus_addr_w;
	wire [7:0]  wram_u_din_w = savestate_wram_mem_active_w ?
		savestate_mem_wdata_i : cpu_pin_dout_w[15:8];
	wire [7:0]  wram_l_din_w = savestate_wram_mem_active_w ?
		savestate_mem_wdata_i : cpu_pin_dout_w[7:0];
	wire        wram_u_wren_w = savestate_wram_mem_active_w ?
		(savestate_mem_wren_i && savestate_mem_addr_i[0]) :
		(wram_bus_write_w && wram_upper_lane_w);
	wire        wram_l_wren_w = savestate_wram_mem_active_w ?
		(savestate_mem_wren_i && !savestate_mem_addr_i[0]) :
		(wram_bus_write_w && wram_lower_lane_w);
	wire [15:0] wram_read_data_w = {wram_peek_u_w, wram_peek_l_w};

	wire        ext_bus_cycle_w = exp_bus_cs_w || sram_bus_cs_w || rom_bus_cs_w;

	assign ext_a_o        = cpu_pin_a_w[23:1];
	assign ext_dout_o     = cpu_pin_dout_w;
	assign ext_dout_oe_o  = ext_bus_cycle_w && cpu_pin_dout_oe_w;
	assign ext_be_o       = cpu_pin_be_w;
	assign ext_st_o       = cpu_pin_st_w;
	assign ext_da_o       = cpu_pin_da_w;
	assign ext_mrq_o      = ext_bus_cycle_w;
	assign ext_rw_o       = cpu_pin_rw_w;
	assign link_sync_o    = misc_link_sync_w;
	assign wram_cs_o      = wram_bus_cs_w;
	assign cart_exp_cs_o  = exp_bus_cs_w;
	assign cart_ram_cs_o  = sram_bus_cs_w;
	assign cart_rom_cs_o  = rom_bus_cs_w;

	// NVC register reads are answered inside the NVC.
	assign cpu_bus_din_raw_w =
		vip_bus_cs_w    ? vip_bus_dout_w   :
		vsu_bus_cs_w    ? vsu_bus_dout_w   :
		wram_bus_cs_w   ? wram_read_data_w :
		ext_bus_cycle_w ? ext_din_i        :
		16'h0000;

	vb_cheat_engine u_cheat_engine
	(
		.clk_sys_i   (clk_i),
		.clear_i     (cheat_clear_i),
		.enable_i    (cpu_pin_mrq_w && cpu_pin_rw_w),
		.code_i      (cheat_code_i),
		.addr_i      (cpu_pin_a_w[26:1]),
		.data_i      (cpu_bus_din_raw_w),
		.data_o      (cpu_bus_din_w),
		.available_o ()
	);

	assign savestate_pause_ready_o =
		savestate_core_drain_q && savestate_video_drain_q &&
		savestate_cpu_ready_w && savestate_vip_ready_w;
	assign savestate_pause_drain_ready_o = savestate_cpu_ready_w &&
		savestate_vip_drain_ready_w;

	always @(posedge clk_i or posedge reset_i) begin
		if (reset_i) begin
			savestate_core_drain_q <= 1'b0;
			savestate_video_drain_q <= 1'b0;
		end else if (!savestate_pause_req_i) begin
			savestate_core_drain_q <= 1'b0;
			savestate_video_drain_q <= 1'b0;
		end else if (!savestate_core_drain_q &&
			!savestate_video_drain_q) begin
			if (savestate_cpu_ready_w && savestate_vip_drain_ready_w) begin
				savestate_core_drain_q <= 1'b1;
			end
		end else if (savestate_core_drain_q &&
			!savestate_video_drain_q) begin
			if (savestate_boundary_stop_w) begin
				// Commit display ownership, then return scanout to its start state.
				savestate_video_drain_q <= 1'b1;
			end
		end
	end

	always @* begin
		if (savestate_state_addr_i < 7'd16) begin
			savestate_state_rdata_o = savestate_cpu_state_rdata_w;
		end else if (savestate_state_addr_i < 7'd38) begin
			savestate_state_rdata_o = savestate_vue_state_rdata_w;
		end else if (savestate_state_addr_i == 7'd38) begin
			savestate_state_rdata_o = savestate_wait_state_rdata_w;
		end else if (savestate_state_addr_i < 7'd64) begin
			savestate_state_rdata_o = savestate_vsu_state_rdata_w;
		end else if (savestate_state_addr_i < 7'd113) begin
			savestate_state_rdata_o = savestate_vip_state_rdata_w;
		end else begin
			savestate_state_rdata_o = 64'd0;
		end
	end

	always @* begin
		case (savestate_mem_type_i)
			3'd0: savestate_mem_rdata_o = savestate_mem_addr_i[0] ?
				wram_peek_u_w : wram_peek_l_w;
			3'd1,
			3'd2: savestate_mem_rdata_o = savestate_vip_mem_rdata_w;
			3'd4: savestate_mem_rdata_o = savestate_vsu_mem_rdata_w;
			3'd5: savestate_mem_rdata_o = savestate_cpu_mem_rdata_w;
			default: savestate_mem_rdata_o = 8'd0;
		endcase
	end

	// Optional timing outputs used by VirtualBoyApp diagnostics.
	assign cpu_bus_mrq_o    = cpu_pin_mrq_w;
	assign cpu_bus_ready_o  = cpu_bus_ready_w;
	assign cpu_bus_region_o = cpu_pin_a_w[26:24];
	assign cpu_bus_da_o     = cpu_pin_da_w;
	assign cpu_bus_rw_o     = cpu_pin_rw_w;
	assign cpu_bus_st_o     = cpu_pin_st_w;
	assign cpu_bus_bcyst_o  = cpu_pin_bcyst_w;
	assign ext_cycle_tag_o  = cpu_pin_cycle_tag_w;

	// The SDRAM backup hold is a MiSTer addition, so it joins the NVC's READY
	// from outside its wait model. It applies only to
	// SDRAM-backed regions (ROM and cartridge SRAM) and stays combinational:
	// data returned in the last moment before a CPU CE edge is consumed at
	// that edge without an extra wait cycle, while a delayed return (an
	// unlucky refresh) stretches READY so the CPU never latches incomplete
	// data.
	wire cpu_ext_ready_w = (!rom_bus_cs_w && !sram_bus_cs_w) || ext_ready_i;

	vb_nvc #(
		.CPU_RESET_PC  (CPU_RESET_PC),
		.CPU_RESET_PSW (CPU_RESET_PSW)
	) u_nvc (
		.clk_i                   (clk_i),
		.reset_i                 (reset_i),
		.ce_i                    (core_ce_w),
		.phi1_i                  (cpu_phi1_w),

		.a_o                     (cpu_pin_a_w),
		.dout_o                  (cpu_pin_dout_w),
		.dout_oe_o               (cpu_pin_dout_oe_w),
		.be_o                    (cpu_pin_be_w),
		.st_o                    (cpu_pin_st_w),
		.da_o                    (cpu_pin_da_w),
		.rw_o                    (cpu_pin_rw_w),
		.mrq_o                   (cpu_pin_mrq_w),
		.bcyst_o                 (cpu_pin_bcyst_w),
		.cycle_tag_o             (cpu_pin_cycle_tag_w),
		.dev_mrq_o               (cpu_dev_mrq_w),
		.vip_mrq_o               (cpu_vip_mrq_w),
		.vip_cs_o                (vip_bus_cs_w),
		.vsu_cs_o                (vsu_bus_cs_w),
		.exp_cs_o                (exp_bus_cs_w),
		.wram_cs_o               (wram_bus_cs_w),
		.sram_cs_o               (sram_bus_cs_w),
		.rom_cs_o                (rom_bus_cs_w),
		.din_i                   (cpu_bus_din_w),
		.vip_ready_i             (vip_bus_ready_w),
		.ext_ready_i             (cpu_ext_ready_w),
		.refresh_busy_o          (cart_refresh_window_o),
		.ready_o                 (cpu_bus_ready_w),

		.vip_irq_i               (vip_irq_w),
		.cart_irq_i              (cart_irq_i),

		.pad_sample_i            (pad_sample_w),
		.pad_serial_enable_i     (pad_serial_enable_i),
		.pad_serial_i            (pad_serial_i),
		.pad_latch_o             (pad_latch_o),
		.pad_clock_o             (pad_clock_o),

		.link_comcnt_i           (link_comcnt_eff_w),
		.link_clk_i              (link_clk_eff_w),
		.link_rx_i               (link_rx_eff_w),
		.link_sync_i             (link_sync_eff_w),
		.link_comcnt_o           (link_comcnt_o),
		.link_clk_o              (link_clk_o),
		.link_tx_o               (link_tx_o),
		.link_sync_o             (misc_link_sync_w),
		.link_tx_done_o          (link_tx_done_w),
		.link_tx_byte_o          (link_tx_byte_w),

		.timer_tick_o            (timer_tick_o),
		.timer_zero_o            (timer_zero_o),
		.timer_irq_o             (timer_irq_o),

		.savestate_pause_req_i   (savestate_pause_req_i),
		.savestate_pause_ready_o (savestate_cpu_ready_w),
		.savestate_restore_i     (savestate_restore_i),
		.savestate_state_wdata_i (savestate_state_wdata_i),
		.savestate_cpu_addr_i    (savestate_state_addr_i[3:0]),
		.savestate_cpu_wren_i    (savestate_cpu_state_wren_w),
		.savestate_cpu_rdata_o   (savestate_cpu_state_rdata_w),
		.savestate_io_addr_i     (savestate_vue_state_addr_w),
		.savestate_io_wren_i     (savestate_vue_state_wren_w),
		.savestate_io_rdata_o    (savestate_io_state_rdata_w),
		.savestate_wait_wren_i   (savestate_wait_state_wren_w),
		.savestate_wait_rdata_o  (savestate_wait_state_rdata_w),
		.savestate_mem_active_i  (savestate_cpu_mem_active_w),
		.savestate_mem_addr_i    (savestate_mem_addr_i[10:0]),
		.savestate_mem_rden_i    (savestate_mem_rden_i),
		.savestate_mem_wren_i    (savestate_mem_wren_i),
		.savestate_mem_wdata_i   (savestate_mem_wdata_i),
		.savestate_mem_rdata_o   (savestate_cpu_mem_rdata_w),

		.dbg_reg_addr_i          (dbg_reg_addr_i),
		.dbg_reg_data_o          (dbg_reg_data_o),
		.dbg_pc_o                (dbg_pc_o),
		.dbg_psw_o               (dbg_psw_o),
		.trace_valid_o           (trace_valid_o),
		.trace_pc_o              (trace_pc_o),
		.trace_insn_o            (trace_insn_o),
		.halted_o                (cpu_halted_o),
		.illegal_o               (cpu_illegal_o)
	);

	VIP #(
		.H_VISIBLE   (VIDEO_H_VISIBLE),
		.H_TOTAL     (VIDEO_H_TOTAL),
		.H_SYNC_BEG  (VIDEO_H_SYNC_BEG),
		.H_SYNC_END  (VIDEO_H_SYNC_END),
		.V_VISIBLE   (VIDEO_V_VISIBLE),
		.V_TOTAL     (VIDEO_V_TOTAL),
		.V_SYNC_BEG  (VIDEO_V_SYNC_BEG),
		.V_SYNC_END  (VIDEO_V_SYNC_END),
		.FLAT_OUTPUT (VIDEO_FLAT_OUTPUT)
	) u_vip (
		.clk_i                           (clk_i),
		.reset_i                         (reset_i),
		.ce_i                            (core_ce_w),
		.phi1_i                          (cpu_phi1_w),
		.video_ce_i                      (core_video_ce_w),
		.video_phi1_i                    (video_phi1_w),
		.savestate_pause_req_i           (savestate_pause_req_i),
		.savestate_boundary_stop_i       (savestate_boundary_stop_w),
		.savestate_pause_drain_ready_o   (savestate_vip_drain_ready_w),
		.savestate_pause_scan_boundary_o (savestate_vip_scan_boundary_w),
		.savestate_pause_ready_o         (savestate_vip_ready_w),
		.savestate_restore_i             (savestate_restore_i),
		.savestate_state_addr_i          (savestate_vip_state_addr_w),
		.savestate_state_wdata_i         (savestate_state_wdata_i),
		.savestate_state_wren_i          (savestate_vip_state_wren_w),
		.savestate_state_rdata_o         (savestate_vip_state_rdata_w),
		.savestate_mem_active_i          (savestate_vip_mem_active_w),
		.savestate_mem_dram_i            (savestate_mem_type_i == 3'd2),
		.savestate_mem_addr_i            (savestate_mem_addr_i[16:0]),
		.savestate_mem_rden_i            (savestate_mem_rden_i),
		.savestate_mem_wren_i            (savestate_mem_wren_i),
		.savestate_mem_wdata_i           (savestate_mem_wdata_i),
		.savestate_mem_rdata_o           (savestate_vip_mem_rdata_w),
		.cs_i                            (vip_bus_cs_w),
		.a_i                             (cpu_pin_a_w[23:1]),
		.din_i                           (cpu_pin_dout_w),
		.be_i                            (cpu_pin_be_w),
		.st_i                            (cpu_pin_st_w),
		.da_i                            (cpu_pin_da_w),
		.mrq_i                           (cpu_vip_mrq_w),
		.rw_i                            (cpu_pin_rw_w),
		.bcyst_i                         (cpu_pin_bcyst_w),
		.dout_o                          (vip_bus_dout_w),
		.ready_o                         (vip_bus_ready_w),
		.irq_o                           (vip_irq_w),
		.flat_dual_eye_i                 (flat_dual_eye_i),
		.side_by_side_i                  (side_by_side_i),
		.compat_60hz_i                   (compat_60hz_i),
		.parallax_scale_i                (parallax_scale_i),
		.presentation_hlg_i              (presentation_hlg_i),
		.brightness_table_legacy_sdr_i   (brightness_table_legacy_sdr_i),
		.brightness_table_download_i     (brightness_table_download_i),
		.brightness_table_write_i        (brightness_table_write_i),
		.brightness_table_addr_i         (brightness_table_addr_i),
		.brightness_table_data_i         (brightness_table_data_i),
		.video_raw_l_o                   (video_raw_l_o),
		.video_raw_r_o                   (video_raw_r_o),
		.video_luma_l_o                  (video_luma_l_o),
		.video_luma_r_o                  (video_luma_r_o),
		.video_hblank_o                  (video_hblank_o),
		.video_vblank_o                  (video_vblank_o),
		.video_hsync_o                   (video_hsync_o),
		.video_vsync_o                   (video_vsync_o),
		.scanner_spinup_i                (scanner_spinup_i)
	);

	VSU u_vsu
	(
		.clk_i                   (clk_i),
		.reset_i                 (reset_i),
		.ce_i                    (core_ce_w),

		.cs_i                    (vsu_bus_cs_w),
		.a_i                     (cpu_pin_a_w[23:1]),
		.din_i                   (cpu_pin_dout_w),
		.be_i                    (cpu_pin_be_w),
		.st_i                    (cpu_pin_st_w),
		.da_i                    (cpu_pin_da_w),
		.mrq_i                   (cpu_dev_mrq_w),
		.rw_i                    (cpu_pin_rw_w),
		.dout_o                  (vsu_bus_dout_w),
		.ready_o                 (),

		.audio_l_o               (audio_l_o),
		.audio_r_o               (audio_r_o),
		.audio_sample_valid_o    (audio_sample_valid_o),
		.savestate_state_addr_i  (savestate_vsu_state_addr_w),
		.savestate_state_wdata_i (savestate_state_wdata_i),
		.savestate_state_wren_i  (savestate_vsu_state_wren_w),
		.savestate_state_rdata_o (savestate_vsu_state_rdata_w),
		.savestate_mem_active_i  (savestate_vsu_mem_active_w),
		.savestate_mem_addr_i    (savestate_mem_addr_i[8:0]),
		.savestate_mem_rden_i    (savestate_mem_rden_i),
		.savestate_mem_wren_i    (savestate_mem_wren_i),
		.savestate_mem_wdata_i   (savestate_mem_wdata_i),
		.savestate_mem_rdata_o   (savestate_vsu_mem_rdata_w)
	);

	// The Virtual Boy WRAM has a fixed one-wait interface. Keeping its two byte
	// lanes in local block RAM makes that fixed edge independent of cartridge
	// SDRAM refresh and arbitration.
	cache_ram #(
		.ADDR_WIDTH (15),
		.DATA_WIDTH (8)
	) u_wram_upper (
		.clk_i   (clk_i),
		.addr_i  (wram_addr_w),
		.wren_i  (wram_u_wren_w),
		.wdata_i (wram_u_din_w),
		.q_o     (wram_peek_u_w)
	);

	cache_ram #(
		.ADDR_WIDTH (15),
		.DATA_WIDTH (8)
	) u_wram_lower (
		.clk_i   (clk_i),
		.addr_i  (wram_addr_w),
		.wren_i  (wram_l_wren_w),
		.wdata_i (wram_l_din_w),
		.q_o     (wram_peek_l_w)
	);

	vb_rumble_pak u_rumble
	(
		.clk_i                   (clk_i),
		.reset_i                 (reset_i),
		.ce_i                    (core_ce_w),
		.enable_i                (rumble_enable_i),
		.byte_valid_i            (link_tx_done_w),
		.byte_i                  (link_tx_byte_w),
		.rumble_o                (rumble_o),
		.savestate_state_addr_i  (savestate_vue_state_addr_w),
		.savestate_state_wdata_i (savestate_state_wdata_i),
		.savestate_state_wren_i  (savestate_vue_state_wren_w),
		.savestate_state_rdata_o (savestate_rumble_state_rdata_w)
	);

	always @(posedge clk_i or posedge reset_i) begin
		if (reset_i) begin
			cpu_phi1_q <= 1'b0;
			video_phi1_q <= 1'b0;
		end else begin
			cpu_phi1_q <= core_ce_w;
			video_phi1_q <= core_video_ce_w;
		end
	end

endmodule
