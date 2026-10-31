// Copyright (c) 2026 Jamie Blanks

`timescale 1ns/1ps
`default_nettype none

// Cartridge ROM and SRAM on the external SDRAM.
//
// clk_ram is 2x clk_sys from the same PLL, so every clk_sys edge is a clk_ram
// edge and signals cross with one register or none.
//
//   CPU ROM read  --> p0 \
//   CPU SRAM r/w  --> p1  > sdram (80 MHz, CL2) --> read registers --> 40 MHz latches
//   loader / bg   --> p2 /                     <-- refresh pulses
//
// A CPU request is taken on the first clk_ram edge it is seen and only once
// per bus cycle tag. With the address at T1+25 ns the read register is loaded
// at T1+112.5 ns and the 40 MHz latch at T1+125 ns, so the 1-wait accept edge
// at T1+150 ns always has the data; READY needs nothing from here.
//
// Refresh: three pulses at the start of each console refresh window, when no
// new bus cycle can start, and three every 16 us while the CPU is stopped.
module vb_cart_sdram
(
	input  wire        clk_sys,
	input  wire        clk_ram,
	input  wire        clk_ram_pin,
	input  wire        reset_i,

	input  wire        rom_req_i,
	input  wire        rom_tag_i,
	input  wire [23:0] rom_addr_i,
	output reg  [15:0] rom_data_o = 16'd0,
	output reg         rom_valid_o = 1'b0,

	input  wire        sram_req_i,
	input  wire        sram_we_i,
	input  wire [25:0] sram_addr_i,
	input  wire [15:0] sram_data_i,
	input  wire [1:0]  sram_be_i,
	input  wire        sram_lane_i,
	input  wire        sram_tag_i,
	output reg  [15:0] sram_data_o = 16'hffff,
	output reg         sram_valid_o = 1'b0,
	output wire        sram_taken_o,

	input  wire        load_req_i,
	input  wire [25:0] load_addr_i,
	input  wire [15:0] load_data_i,
	output wire        load_ready_o,
	output reg         load_done_o = 1'b0,

	input  wire        bg_req_i,
	input  wire        bg_we_i,
	input  wire [25:0] bg_addr_i,
	input  wire [15:0] bg_data_i,
	input  wire [1:0]  bg_be_i,
	output wire        bg_ready_o,
	output reg         bg_done_o = 1'b0,
	output reg  [15:0] bg_data_o = 16'hffff,

	input  wire        refresh_window_i,
	input  wire        cpu_stopped_i,
	output wire        foreground_idle_o,
	output wire        quiet_o,

	inout  wire [15:0] SDRAM_DQ,
	output wire [12:0] SDRAM_A,
	output wire        SDRAM_DQML,
	output wire        SDRAM_DQMH,
	output wire  [1:0] SDRAM_BA,
	output wire        SDRAM_nCS,
	output wire        SDRAM_nWE,
	output wire        SDRAM_nRAS,
	output wire        SDRAM_nCAS,
	output wire        SDRAM_CKE,
	output wire        SDRAM_CLK
);

	localparam integer CLK_RAM_HZ = 80_000_000;
	// 16 us at 80 MHz, the console refresh window period.
	localparam [10:0] STOPPED_REFRESH_LAST = 11'd1279;

	// The controller's reset leaves on a clk_ram edge.
	reg [1:0] reset_ram_q = 2'b11;
	always @(posedge clk_ram or posedge reset_i) begin
		if (reset_i) reset_ram_q <= 2'b11;
		else reset_ram_q <= {reset_ram_q[0], 1'b0};
	end
	wire reset_ram_w = reset_ram_q[1];

	wire [15:0] p0_dout_w, p1_dout_w, p2_dout_w;
	wire        p0_busy_w, p1_busy_w, p2_busy_w;
	wire        p0_ready_w, p1_ready_w, p2_ready_w;
	wire        init_done_w;
	wire        sdram_idle_w;

	// One take per bus cycle: the tag of a taken request is remembered until
	// the request drops.
	reg rom_seen_q = 1'b0, rom_seen_tag_q = 1'b0;
	reg sram_seen_q = 1'b0, sram_seen_tag_q = 1'b0;
	wire rom_taken_w  = rom_seen_q && (rom_seen_tag_q == rom_tag_i);
	wire sram_taken_w = sram_seen_q && (sram_seen_tag_q == sram_tag_i);
	wire p0_req_w = rom_req_i && !rom_taken_w;
	wire p1_req_w = sram_req_i && !sram_taken_w;
	assign sram_taken_o = sram_req_i && sram_taken_w;

	// p2 is one registered 40 MHz request at a time, from the ROM loader or
	// the background SRAM sequencer, started only while the CPU is stopped so
	// it can never delay a CPU access. The tag separates it from the last one.
	reg        p2_active_q = 1'b0;
	reg        p2_bg_q = 1'b0;
	reg        p2_tag_q = 1'b0;
	reg        p2_we_q = 1'b0;
	reg [25:0] p2_addr_q = 26'd0;
	reg [15:0] p2_data_q = 16'd0;
	reg [1:0]  p2_be_q = 2'b11;
	reg        p2_taken_tag_q = 1'b0;
	reg        bg_seen_q = 1'b0;
	wire p2_req_w = p2_active_q && (p2_tag_q != p2_taken_tag_q);
	wire p2_done_w = p2_active_q && (p2_tag_q == p2_taken_tag_q) && p2_ready_w;

	reg       window_old_q = 1'b0;
	reg [10:0] stopped_ctr_q = 11'd0;
	reg [2:0] refresh_phase_q = 3'd0;
	wire refresh_burst_w = (refresh_window_i && !window_old_q) ||
		(cpu_stopped_i && (stopped_ctr_q == STOPPED_REFRESH_LAST));

	assign load_ready_o = init_done_w && cpu_stopped_i && !p2_active_q;
	assign bg_ready_o = init_done_w && cpu_stopped_i && !p2_active_q && !load_req_i;
	assign foreground_idle_o = p0_ready_w && p1_ready_w;
	assign quiet_o = sdram_idle_w && !p2_active_q && (refresh_phase_q == 3'd0);

	always @(posedge clk_ram) begin
		if (reset_ram_w) begin
			rom_seen_q <= 1'b0;
			sram_seen_q <= 1'b0;
			p2_taken_tag_q <= 1'b0;
			window_old_q <= 1'b0;
			stopped_ctr_q <= 11'd0;
			refresh_phase_q <= 3'd0;
		end else begin
			if (!rom_req_i) rom_seen_q <= 1'b0;
			else if (p0_req_w && !p0_busy_w) begin
				rom_seen_q <= 1'b1;
				rom_seen_tag_q <= rom_tag_i;
			end
			if (!sram_req_i) sram_seen_q <= 1'b0;
			else if (p1_req_w && !p1_busy_w) begin
				sram_seen_q <= 1'b1;
				sram_seen_tag_q <= sram_tag_i;
			end
			if (p2_req_w && !p2_busy_w) p2_taken_tag_q <= p2_tag_q;

			// Three high-low pulses; the controller counts rising edges.
			window_old_q <= refresh_window_i;
			stopped_ctr_q <= (stopped_ctr_q == STOPPED_REFRESH_LAST) ?
				11'd0 : stopped_ctr_q + 11'd1;
			if (refresh_phase_q != 3'd0) refresh_phase_q <= refresh_phase_q - 3'd1;
			else if (refresh_burst_w) refresh_phase_q <= 3'd6;
		end
	end

	// The read registers hold until their port's next read, so a free-running
	// 40 MHz copy is enough; the valid flags follow the same edge.
	always @(posedge clk_sys) begin
		rom_data_o <= p0_dout_w;
		rom_valid_o <= rom_req_i && rom_taken_w && p0_ready_w;
		sram_data_o <= {8'hff, sram_lane_i ? p1_dout_w[15:8] : p1_dout_w[7:0]};
		sram_valid_o <= sram_req_i && sram_taken_w && p1_ready_w;
	end

	always @(posedge clk_sys) begin
		load_done_o <= 1'b0;
		bg_done_o <= 1'b0;
		if (!bg_req_i) bg_seen_q <= 1'b0;

		if (reset_i) begin
			p2_active_q <= 1'b0;
			p2_tag_q <= 1'b0;
			bg_seen_q <= 1'b0;
		end else if (p2_done_w) begin
			p2_active_q <= 1'b0;
			if (p2_bg_q) begin
				bg_done_o <= 1'b1;
				bg_data_o <= p2_dout_w;
			end else begin
				load_done_o <= 1'b1;
			end
		end else if (!p2_active_q && init_done_w && cpu_stopped_i) begin
			if (load_req_i) begin
				p2_active_q <= 1'b1;
				p2_bg_q <= 1'b0;
				p2_tag_q <= !p2_tag_q;
				p2_we_q <= 1'b1;
				p2_addr_q <= load_addr_i;
				p2_data_q <= load_data_i;
				p2_be_q <= 2'b11;
			end else if (bg_req_i && !bg_seen_q) begin
				p2_active_q <= 1'b1;
				p2_bg_q <= 1'b1;
				p2_tag_q <= !p2_tag_q;
				p2_we_q <= bg_we_i;
				p2_addr_q <= bg_addr_i;
				p2_data_q <= bg_data_i;
				p2_be_q <= bg_be_i;
				bg_seen_q <= 1'b1;
			end
		end
	end

	sdram #(
		.CLK_FREQ_HZ(CLK_RAM_HZ),
		.SDRAM_TIMING_GRADE(7),
		.DQ_CAPTURE_PIPELINE(2),
		.DQ_CAPTURE_RISING(1'b0)
	) u_core (
		.clk(clk_ram),
		.clk_pin(clk_ram_pin),
		.reset(reset_ram_w),
		.refresh(refresh_phase_q[0]),
		.p0_req(p0_req_w),
		.p0_we(1'b0),
		.p0_addr({2'b00, rom_addr_i}),
		.p0_din(16'd0),
		.p0_be(2'b11),
		.p0_dout(p0_dout_w),
		.p0_busy(p0_busy_w),
		.p0_ready(p0_ready_w),
		.p1_req(p1_req_w),
		.p1_we(sram_we_i),
		.p1_addr(sram_addr_i),
		.p1_din(sram_data_i),
		.p1_be(sram_be_i),
		.p1_dout(p1_dout_w),
		.p1_busy(p1_busy_w),
		.p1_ready(p1_ready_w),
		.p2_req(p2_req_w),
		.p2_we(p2_we_q),
		.p2_addr(p2_addr_q),
		.p2_din(p2_data_q),
		.p2_be(p2_be_q),
		.p2_dout(p2_dout_w),
		.p2_busy(p2_busy_w),
		.p2_ready(p2_ready_w),
		.init_done(init_done_w),
		.idle(sdram_idle_w),
		.SDRAM_CLK(SDRAM_CLK),
		.SDRAM_CKE(SDRAM_CKE),
		.SDRAM_A(SDRAM_A),
		.SDRAM_BA(SDRAM_BA),
		.SDRAM_DQ(SDRAM_DQ),
		.SDRAM_DQML(SDRAM_DQML),
		.SDRAM_DQMH(SDRAM_DQMH),
		.SDRAM_nCS(SDRAM_nCS),
		.SDRAM_nCAS(SDRAM_nCAS),
		.SDRAM_nRAS(SDRAM_nRAS),
		.SDRAM_nWE(SDRAM_nWE)
	);

endmodule

`default_nettype wire
