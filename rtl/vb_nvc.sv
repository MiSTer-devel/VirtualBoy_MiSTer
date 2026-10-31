// Copyright (c) 2026 Jamie Blanks

// NVC: the V810 core plus Nintendo's logic around it, as the Virtual Boy CPU.
//
//   VIP, cart INT --> irq encoder --> INT  +---------+ --> A, D, BE, ST, DA, R/W, MRQ
//   timer, pad, link, WCR <-- internal bus | NECv810 | --> chip selects, device MRQ
//   wait control, refresh ---------> READY +---------+ <-- D, VIP READY, external READY
//
// The internal registers sit on the core's bus with their own select and one
// wait; their read data does not leave the package.
module vb_nvc
#(
	parameter [31:0] CPU_RESET_PC  = 32'hffff_fff0,
	parameter [31:0] CPU_RESET_PSW = 32'h0000_8000
)
(
	input  wire         clk_i,
	input  wire         reset_i,
	input  wire         ce_i,
	input  wire         phi1_i,

	output wire [26:1]  a_o,
	output wire [15:0]  dout_o,
	output wire         dout_oe_o,
	output wire [3:0]   be_o,
	output wire [1:0]   st_o,
	output wire         da_o,
	output wire         rw_o,
	output wire         mrq_o,
	output wire         bcyst_o,
	output wire         cycle_tag_o,
	output wire         dev_mrq_o,         // MRQ for on-board devices
	output wire         vip_mrq_o,         // dev_mrq_o, held off the VIP during refresh
	output wire         vip_cs_o,
	output wire         vsu_cs_o,
	output wire         exp_cs_o,
	output wire         wram_cs_o,
	output wire         sram_cs_o,
	output wire         rom_cs_o,
	input  wire [15:0]  din_i,
	input  wire         vip_ready_i,
	input  wire         ext_ready_i,       // ANDed into the wait generator's READY
	output wire         refresh_busy_o,    // the console bus refresh window
	output wire         ready_o,

	input  wire         vip_irq_i,
	input  wire         cart_irq_i,

	input  wire [15:0]  pad_sample_i,
	input  wire         pad_serial_enable_i,
	input  wire         pad_serial_i,
	output wire         pad_latch_o,
	output wire         pad_clock_o,

	input  wire         link_comcnt_i,
	input  wire         link_clk_i,
	input  wire         link_rx_i,
	input  wire         link_sync_i,
	output wire         link_comcnt_o,
	output wire         link_clk_o,
	output wire         link_tx_o,
	output wire         link_sync_o,
	output wire         link_tx_done_o,    // CE pulse: the last bit of a byte was sent
	output wire [7:0]   link_tx_byte_o,

	output wire         timer_tick_o,
	output wire         timer_zero_o,
	output wire         timer_irq_o,

	input  wire         savestate_pause_req_i,
	output wire         savestate_pause_ready_o,
	input  wire         savestate_restore_i,
	input  wire [63:0]  savestate_state_wdata_i,
	input  wire [3:0]   savestate_cpu_addr_i,
	input  wire         savestate_cpu_wren_i,
	output wire [63:0]  savestate_cpu_rdata_o,
	input  wire [4:0]   savestate_io_addr_i,
	input  wire         savestate_io_wren_i,
	output wire [63:0]  savestate_io_rdata_o,
	input  wire         savestate_wait_wren_i,
	output wire [63:0]  savestate_wait_rdata_o,
	input  wire         savestate_mem_active_i,
	input  wire [10:0]  savestate_mem_addr_i,
	input  wire         savestate_mem_rden_i,
	input  wire         savestate_mem_wren_i,
	input  wire [7:0]   savestate_mem_wdata_i,
	output wire [7:0]   savestate_mem_rdata_o,

	input  wire [4:0]   dbg_reg_addr_i,
	output wire [31:0]  dbg_reg_data_o,
	output wire [31:0]  dbg_pc_o,
	output wire [31:0]  dbg_psw_o,
	output wire         trace_valid_o,
	output wire [31:0]  trace_pc_o,
	output wire [15:0]  trace_insn_o,
	output wire         halted_o,
	output wire         illegal_o
);

	wire [31:1] a_w;
	wire [31:0] dout_full_w;
	wire        bus_t1_w;
	wire        refresh_busy_w;
	wire        refresh_tick_w;
	wire        wait_ready_w;
	wire        misc_cs_w;
	wire        unused_cs_w;
	wire [15:0] misc_dout_w;
	wire        timer_irq_w;
	wire        pad_irq_w;
	wire        comm_irq_w;
	wire        wait_exp1_w;
	wire        wait_rom1_w;

	assign a_o    = a_w[26:1];
	assign dout_o = dout_full_w[15:0];
	// On-board devices see a chained cycle only after its T1; the cartridge
	// SDRAM request starts with the address.
	assign dev_mrq_o   = mrq_o && !bus_t1_w;
	assign vip_mrq_o   = dev_mrq_o && !refresh_busy_w;
	assign refresh_busy_o = refresh_busy_w;
	assign ready_o     = wait_ready_w && ext_ready_i;
	assign timer_irq_o = timer_irq_w;

	wire [15:0] core_din_w = misc_cs_w ? misc_dout_w : din_i;

	// Fixed priority: VIP 4, link 3, cartridge 2, timer 1, pad 0.
	wire        irq_valid_w = vip_irq_i | comm_irq_w | cart_irq_i | timer_irq_w | pad_irq_w;
	wire [3:0]  irq_level_w =
		vip_irq_i   ? 4'd4 :
		comm_irq_w  ? 4'd3 :
		cart_irq_i  ? 4'd2 :
		timer_irq_w ? 4'd1 :
		4'd0;

	// The CPU samples its interrupt pins through two registers.
	reg  [4:0]  irq_s1_q, irq_s2_q;
	always @(posedge clk_i) begin
		if (reset_i || savestate_restore_i) begin
			irq_s1_q <= 5'd0;
			irq_s2_q <= 5'd0;
		end else if (ce_i) begin
			irq_s1_q <= {irq_valid_w, irq_level_w};
			irq_s2_q <= irq_s1_q;
		end
	end

	// WCR sets the base wait; the VIP and the caller's READY may hold it longer.
	NECv810 #(
		.RESET_PC  (CPU_RESET_PC),
		.RESET_PSW (CPU_RESET_PSW)
	) u_cpu (
		.clk_i                   (clk_i),
		.reset_i                 (reset_i),
		.ce_i                    (ce_i),
		.phi1_i                  (phi1_i),
		.ready_i                 (ready_o),
		.din_i                   ({16'd0, core_din_w}),
		.hldrq_i                 (1'b0),
		.icheen_i                (1'b1),
		.siz16b_i                (1'b1),
		.szrq_i                  (1'b1),
		.irq_valid_i             (irq_s2_q[4]),
		.irq_level_i             (irq_s2_q[3:0]),
		.nmi_i                   (1'b0),
		.savestate_pause_req_i   (savestate_pause_req_i),
		.savestate_pause_ready_o (savestate_pause_ready_o),
		.savestate_restore_i     (savestate_restore_i),
		.savestate_state_addr_i  (savestate_cpu_addr_i),
		.savestate_state_wdata_i (savestate_state_wdata_i),
		.savestate_state_wren_i  (savestate_cpu_wren_i),
		.savestate_state_rdata_o (savestate_cpu_rdata_o),
		.savestate_mem_active_i  (savestate_mem_active_i),
		.savestate_mem_addr_i    (savestate_mem_addr_i),
		.savestate_mem_rden_i    (savestate_mem_rden_i),
		.savestate_mem_wren_i    (savestate_mem_wren_i),
		.savestate_mem_wdata_i   (savestate_mem_wdata_i),
		.savestate_mem_rdata_o   (savestate_mem_rdata_o),
		.a_o                     (a_w),
		.a_oe_o                  (),
		.dout_o                  (dout_full_w),
		.dout_oe_o               (dout_oe_o),
		.dout_lane_oe_o          (),
		.be_o                    (be_o),
		.be_oe_o                 (),
		.st_o                    (st_o),
		.st_oe_o                 (),
		.da_o                    (da_o),
		.da_oe_o                 (),
		.mrq_o                   (mrq_o),
		.mrq_oe_o                (),
		.rw_o                    (rw_o),
		.rw_oe_o                 (),
		.bcyst_o                 (bcyst_o),
		.bcyst_oe_o              (),
		.adrs_err_n_o            (),
		.cycle_tag_o             (cycle_tag_o),
		.bus_t1_o                (bus_t1_w),
		.block_o                 (),
		.hldak_o                 (),
		.dbg_reg_addr_i          (dbg_reg_addr_i),
		.dbg_reg_data_o          (dbg_reg_data_o),
		.dbg_pc_o                (dbg_pc_o),
		.dbg_psw_o               (dbg_psw_o),
		.trace_valid_o           (trace_valid_o),
		.trace_pc_o              (trace_pc_o),
		.trace_insn_o            (trace_insn_o),
		.halted_o                (halted_o),
		.illegal_o               (illegal_o)
	);

	vb_vue_addr_decode u_vue_decode
	(
		.mrq_i       (mrq_o),
		.region_i    (a_w[26:24]),
		.vip_cs_o    (vip_cs_o),
		.vsu_cs_o    (vsu_cs_o),
		.misc_cs_o   (misc_cs_w),
		.unused_cs_o (unused_cs_w),
		.exp_cs_o    (exp_cs_o),
		.wram_cs_o   (wram_cs_o),
		.sram_cs_o   (sram_cs_o),
		.rom_cs_o    (rom_cs_o)
	);

	// Timer, game pad, link port and WCR.
	vb_vue_io u_misc
	(
		.clk_i                   (clk_i),
		.reset_i                 (reset_i),
		.ce_i                    (ce_i),

		.pad_sample_i            (pad_sample_i),
		.pad_serial_enable_i     (pad_serial_enable_i),
		.pad_serial_i            (pad_serial_i),
		.link_comcnt_i           (link_comcnt_i),
		.link_clk_i              (link_clk_i),
		.link_rx_i               (link_rx_i),
		.link_sync_i             (link_sync_i),

		.cs_i                    (misc_cs_w),
		.a_i                     (a_w[23:1]),
		.din_i                   (dout_o),
		.be_i                    (be_o),
		.st_i                    (st_o),
		.da_i                    (da_o),
		.mrq_i                   (dev_mrq_o),
		.rw_i                    (rw_o),
		.ready_i                 (ready_o),
		.dout_o                  (misc_dout_w),

		.pad_latch_o             (pad_latch_o),
		.pad_clock_o             (pad_clock_o),
		.link_comcnt_o           (link_comcnt_o),
		.link_clk_o              (link_clk_o),
		.link_tx_o               (link_tx_o),
		.link_sync_o             (link_sync_o),
		.link_tx_done_o          (link_tx_done_o),
		.link_tx_byte_o          (link_tx_byte_o),

		.refresh_tick_o          (refresh_tick_w),
		.timer_tick_o            (timer_tick_o),
		.timer_zero_o            (timer_zero_o),
		.timer_irq_o             (timer_irq_w),
		.pad_irq_o               (pad_irq_w),
		.comm_irq_o              (comm_irq_w),
		.wait_exp1_o             (wait_exp1_w),
		.wait_rom1_o             (wait_rom1_w),
		.savestate_state_addr_i  (savestate_io_addr_i),
		.savestate_state_wdata_i (savestate_state_wdata_i[50:0]),
		.savestate_state_wren_i  (savestate_io_wren_i),
		.savestate_state_rdata_o (savestate_io_rdata_o)
	);

	vb_vue_wait_control u_wait_control
	(
		.clk_i                   (clk_i),
		.reset_i                 (reset_i),
		.ce_i                    (ce_i),

		.mrq_i                   (dev_mrq_o),
		.bcyst_i                 (bcyst_o),
		.a_i                     (a_w[26:1]),
		.be_i                    (be_o),
		.st_i                    (st_o),
		.da_i                    (da_o),
		.rw_i                    (rw_o),

		.vip_cs_i                (vip_cs_o),
		.vsu_cs_i                (vsu_cs_o),
		.misc_cs_i               (misc_cs_w),
		.unused_cs_i             (unused_cs_w),
		.exp_cs_i                (exp_cs_o),
		.wram_cs_i               (wram_cs_o),
		.sram_cs_i               (sram_cs_o),
		.rom_cs_i                (rom_cs_o),

		.wait_exp1_i             (wait_exp1_w),
		.wait_rom1_i             (wait_rom1_w),
		.vip_ready_i             (vip_ready_i),
		.ready_o                 (wait_ready_w),
		.refresh_tick_i          (refresh_tick_w),
		.refresh_busy_o          (refresh_busy_w),
		.savestate_state_wdata_i (savestate_state_wdata_i),
		.savestate_state_wren_i  (savestate_wait_wren_i),
		.savestate_state_rdata_o (savestate_wait_rdata_o)
	);

endmodule
