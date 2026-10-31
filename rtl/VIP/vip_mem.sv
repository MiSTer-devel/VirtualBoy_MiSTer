// Copyright (c) 2026 Jamie Blanks

`timescale 1ns/1ps

// One VIP memory (VRAM or DRAM): 64K halfwords in two byte lanes.
//
// Port A is shared by refresh, the DP, the XP and the CPU. A grant gives the
// memory to one client for that client's slot length. The XP works in page
// bursts: while it asks for the open row it keeps the memory, up to XP_BURST
// beats. Anywhere else the CPU goes first, so the CPU keeps the memory while
// its accesses come back to back, and the XP opens a new burst at the first
// boundary the CPU leaves free; a held CPU beat that waits for its own bus
// cycle (cpu_wait_i) keeps the memory too, idle. Refresh and the DP go ahead
// of both and close the XP's page. The XP's first beat after another client's
// slot takes XP_REOPEN more clocks to open its row again, XP_REOPEN_CPU after a
// CPU write and XP_REOPEN_RD after a CPU read. A beat to a row other than the
// XP's last one takes XP_NEW_ROW more. With OPEN_ROW the memory instead keeps
// the row of its last access open, whoever made it, until a refresh: an XP beat
// in that row is a page beat, any other takes XP_NEW_ROW more: at most
// XP_REOPEN_RD right after a CPU read or a refresh, which leave the RAM
// precharged, and XP_OWN_ROW_RD (when set) for an XP read that changes row on
// its own.
// With XP_MAX_WAIT > 0 a waiting XP goes before the CPU once that many CPU
// accesses were granted ahead of it; the high half of a word (cpu_cont_i) is
// part of its low half's access and is never split from it.
// XP work that needs no memory but holds its data path (xp_work_i, the strip
// buffer clear) keeps the memory from the CPU like a burst, XP_WORK_UNIT clocks
// at a time, and waits (xp_work_wait_o) while the CPU has the memory.
// CPU writes come from the host's write latch. A CPU read returns its data at
// clock CPU_RD_SLOT and keeps the memory CPU_RD_HOLD clocks longer to recover,
// which fits inside the CPU's bus cycle. A refresh may start after the first of
// those clocks, the RAM's precharge; the rest is the bus turning round. It may
// also take the clocks of a CPU write slot beyond the RAM's 3-clock cycle.
//
//   CE edge     G-1        G          G+1   ...   G+L-1       G+L
//               |  req,gnt |  clock 1 |     ...   |  clock L  |
//                          ^ taken                   rvalid   ^ data sampled
//
// gnt is high in the CE cycle that ends at the taking edge G. The address is
// registered at G and reaches the RAM on the next phi1, where writes land.
// rvalid is high in the slot's last CE cycle, so a client registers rdata at
// G+L. A new grant can be taken at G+L.
//
// Port B reads the address presented in a CE cycle by the next CE edge. While
// ssm_active_i is set it serves savestate byte reads and writes instead.
//
// lo_we_o shows each low-byte write in the clock it lands, from either port.
/* verilator lint_off DECLFILENAME */
module vip_mem_ctrl
#(
	parameter integer CPU_RD_SLOT    = 4,     // memory occupancy of a CPU access; read data latency
	parameter integer CPU_RD_HOLD    = 2,     // further CPU read occupancy after its data
	parameter integer CPU_WR_SLOT    = 4,
	parameter integer XP_RD_SLOT     = 2,     // XP char/map reads, FB transfer writes
	parameter integer XP_WR_SLOT     = 2,
	parameter integer DP_RD_SLOT     = 2,     // DRAM column-table read
	parameter integer REFRESH_PERIOD = 96,
	parameter integer REFRESH_SLOT   = 5,
	parameter integer ROW_SHIFT      = 8,     // address bits below the row (8: 256-word rows)
	parameter integer XP_BURST       = 8,     // XP page beats a waiting CPU lets go ahead
	parameter integer XP_MAX_WAIT    = 0,     // 0: the CPU always goes first; N>0: the XP wins after N CPU accesses
	parameter integer XP_REOPEN      = 1,     // extra clocks for the XP's first beat after another client
	parameter integer XP_REOPEN_CPU  = 1,     // the same after a CPU write
	parameter integer XP_REOPEN_RD   = 1,     // the same after a CPU read
	parameter integer XP_NEW_ROW     = 0,     // extra clocks for an XP beat that moves to another row
	parameter integer XP_WORK_UNIT   = 8,     // clocks of XP work a waiting CPU lets go ahead
	parameter integer XP_OWN_ROW_RD  = -1,    // >= 0: replaces XP_NEW_ROW for an XP read changing row on its own
	parameter integer OPEN_ROW       = 0,     // 1: an XP beat outside the memory's open row takes XP_NEW_ROW more
	parameter [5:0]   SS_WORD        = 6'd22  // savestate word holding the slot and refresh state
)
(
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        ce_i,
	input  wire        phi1_i,
	input  wire        refresh_en_i,     // DPCTRL.RE

	input  wire        dp_req_i,
	input  wire [15:0] dp_addr_i,
	output wire        dp_gnt_o,
	output wire        dp_rvalid_o,
	output wire [15:0] dp_rdata_o,

	input  wire        xp_req_i,
	input  wire        xp_we_i,
	input  wire [15:0] xp_addr_i,
	input  wire [1:0]  xp_be_i,
	input  wire [15:0] xp_wdata_i,
	output wire        xp_gnt_o,
	output wire        xp_rvalid_o,
	output wire [15:0] xp_rdata_o,
	input  wire        xp_work_i,        // XP work that holds the data path without a request
	output wire        xp_work_wait_o,   // that work waits: the CPU has the memory

	input  wire        cpu_req_i,
	input  wire        cpu_we_i,
	input  wire        cpu_cont_i,       // high half of a word: same access as the last CPU grant
	input  wire        cpu_hold_i,       // the CPU's access came straight after its last one
	input  wire        cpu_wait_i,       // a CPU beat is on the bus but not yet asking
	input  wire [15:0] cpu_addr_i,
	input  wire [1:0]  cpu_be_i,
	input  wire [15:0] cpu_wdata_i,
	output wire        cpu_gnt_o,
	output wire        cpu_rvalid_o,
	output wire [15:0] cpu_rdata_o,

	input  wire [15:0] b_addr_i,         // SAM or observer reads
	output wire [15:0] b_rdata_o,

	input  wire        ssm_active_i,     // paused byte access through port B
	input  wire [16:0] ssm_addr_i,
	input  wire        ssm_we_i,
	input  wire [7:0]  ssm_wdata_i,
	output wire [7:0]  ssm_rdata_o,

	input  wire [5:0]  ss_addr_i,
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [63:0] ss_wdata_i,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        ss_we_i,
	output wire [63:0] ss_rdata_o,

	output wire        lo_we_o,
	output wire [15:0] lo_addr_o,
	output wire [7:0]  lo_wdata_o,

	output wire        busy_o,           // a slot is running
	output wire [1:0]  owner_o           // 0 refresh, 1 DP, 2 XP, 3 CPU
);
	localparam [1:0] OWN_REFRESH = 2'd0;
	localparam [1:0] OWN_DP      = 2'd1;
	localparam [1:0] OWN_XP      = 2'd2;
	localparam [1:0] OWN_CPU     = 2'd3;

	localparam integer PERIOD_LAST_I = REFRESH_PERIOD - 1;
	localparam [9:0]   PERIOD_LAST   = PERIOD_LAST_I[9:0];
	localparam integer LEN_CPU_RD_I  = CPU_RD_SLOT + CPU_RD_HOLD;
	localparam [3:0]   LEN_CPU_RD    = LEN_CPU_RD_I[3:0];
	localparam [3:0]   CPU_RD_LEFT   = CPU_RD_HOLD[3:0];
	localparam integer PRE_LEFT_I    = (CPU_RD_HOLD > 1) ? CPU_RD_HOLD - 1 : 0;
	localparam [3:0]   PRE_LEFT      = PRE_LEFT_I[3:0];   // hold clocks left after the precharge
	localparam [3:0]   LEN_CPU_WR    = CPU_WR_SLOT[3:0];
	localparam integer WR_PRE_LEFT_I = (CPU_WR_SLOT > 3) ? CPU_WR_SLOT - 3 : 0;
	localparam [3:0]   WR_PRE_LEFT   = WR_PRE_LEFT_I[3:0];   // write slot clocks after the RAM's 3-clock cycle
	localparam [3:0]   LEN_XP_RD     = XP_RD_SLOT[3:0];
	localparam [3:0]   LEN_XP_WR     = XP_WR_SLOT[3:0];
	localparam [3:0]   LEN_DP_RD     = DP_RD_SLOT[3:0];
	localparam [3:0]   LEN_REFRESH   = REFRESH_SLOT[3:0];
	localparam [3:0]   XP_WAIT       = XP_MAX_WAIT[3:0];
	localparam [3:0]   BURST         = XP_BURST[3:0];
	localparam [3:0]   REOPEN        = XP_REOPEN[3:0];
	localparam [3:0]   REOPEN_CPU    = XP_REOPEN_CPU[3:0];
	localparam [3:0]   REOPEN_RD     = XP_REOPEN_RD[3:0];
	localparam [3:0]   NEW_ROW       = XP_NEW_ROW[3:0];
	localparam [3:0]   RD_ROW        = (XP_REOPEN_RD < XP_NEW_ROW) ? REOPEN_RD : NEW_ROW;
	localparam integer OWN_RD_I      = (XP_OWN_ROW_RD < 0) ? XP_NEW_ROW : XP_OWN_ROW_RD;
	localparam [3:0]   OWN_RD        = OWN_RD_I[3:0];
	localparam [4:0]   WORK_UNIT     = XP_WORK_UNIT[4:0];

	reg        busy_q;
	reg [1:0]  owner_q;
	reg [3:0]  left_q;        // slot clocks after the current one
	reg        we_q;
	reg        wr_pend_q;     // write lands on the next phi1
	reg [15:0] addr_q;
	reg [1:0]  be_q;
	reg [15:0] wdata_q;
	reg [9:0]  phase_q;
	reg        refresh_due_q;
	reg [3:0]  cpu_wins_q;    // CPU accesses granted while the XP waits
	reg        cpu_own_q;     // the CPU, not the XP, had the memory last
	reg        xp_open_q;     // the XP's page burst is open
	reg        xp_lost_q;     // another client closed the XP's row since its last beat
	reg        lost_cpu_q;    // the last of them was the CPU
	reg        lost_rd_q;     // and it read or refreshed: the RAM is precharged
	reg [8:0]  xp_row_q;
	reg [3:0]  xp_beats_q;
	reg [4:0]  work_clks_q;   // clocks of the XP work's current unit
	reg        ssm_hi_q;
	reg [8:0]  open_row_q;    // row of the last access
	reg        open_ok_q;     // it is open (refresh closes it)

	reg        take_t;
	reg [1:0]  pick_t;
	reg [3:0]  len_t;

	wire refresh_early_w = refresh_due_q && busy_q && (owner_q == OWN_CPU) &&
		(left_q <= (we_q ? WR_PRE_LEFT : PRE_LEFT));
	wire free_w      = !busy_q || (left_q == 4'd0) || refresh_early_w;
	/* verilator lint_off UNUSEDSIGNAL */
	wire [15:0] xp_row_full_w = xp_addr_i >> ROW_SHIFT;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [8:0]  xp_row_w      = xp_row_full_w[8:0];
	/* verilator lint_off UNUSEDSIGNAL */
	wire [15:0] cpu_row_full_w = cpu_addr_i >> ROW_SHIFT;
	wire [15:0] dp_row_full_w  = dp_addr_i >> ROW_SHIFT;
	/* verilator lint_on UNUSEDSIGNAL */
	wire        xp_in_row_w   = open_ok_q && (xp_row_w == open_row_q);
	wire xp_hit_w    = xp_open_q && xp_req_i && (xp_row_w == xp_row_q) && (xp_beats_q != BURST);
	wire xp_due_w    = (XP_MAX_WAIT > 0) && xp_req_i && (cpu_wins_q >= XP_WAIT) && !cpu_cont_i;
	wire work_w      = xp_work_i && !xp_req_i;
	wire work_hit_w  = work_w && (work_clks_q != WORK_UNIT) && !(cpu_hold_i && cpu_own_q);
	// The CPU waits for the XP's burst or work unit to end; once it has the
	// memory it keeps it only while its accesses come back to back.
	wire cpu_ask_w   = cpu_req_i || (cpu_wait_i && cpu_hold_i && cpu_own_q);
	wire cpu_first_w = cpu_ask_w && !xp_hit_w && !work_hit_w && !xp_due_w &&
		(cpu_hold_i || !cpu_own_q || !(xp_req_i || xp_work_i));
	wire grant_w     = ce_i && !reset_i && free_w && take_t;
	wire rd_slot_w   = ce_i && busy_q && !we_q;
	wire last_rd_w   = rd_slot_w && (left_q == 4'd0);

	always @* begin
		take_t = 1'b1;
		pick_t = OWN_CPU;
		if (refresh_due_q) begin
			pick_t = OWN_REFRESH;
		end else if (dp_req_i) begin
			pick_t = OWN_DP;
		end else if (xp_req_i && !cpu_first_w) begin
			pick_t = OWN_XP;
		end else if (!cpu_req_i || (work_w && !cpu_first_w)) begin
			take_t = 1'b0;
		end

		case (pick_t)
			OWN_REFRESH: len_t = LEN_REFRESH;
			OWN_DP:      len_t = LEN_DP_RD;
			OWN_XP:      len_t = (xp_we_i ? LEN_XP_WR : LEN_XP_RD) +
				((OPEN_ROW != 0) ? (xp_in_row_w ? 4'd0 : (xp_lost_q && lost_rd_q) ? RD_ROW : (!xp_lost_q && !xp_we_i) ? OWN_RD : NEW_ROW) :
				(xp_lost_q && !xp_hit_w) ? (lost_rd_q ? REOPEN_RD : lost_cpu_q ? REOPEN_CPU : REOPEN) :
				(xp_row_w != xp_row_q) ? NEW_ROW : 4'd0);
			default:     len_t = cpu_we_i ? LEN_CPU_WR : LEN_CPU_RD;
		endcase
	end

	assign dp_gnt_o     = grant_w && (pick_t == OWN_DP);
	assign xp_gnt_o     = grant_w && (pick_t == OWN_XP);
	assign cpu_gnt_o    = grant_w && (pick_t == OWN_CPU);
	assign dp_rvalid_o  = last_rd_w && (owner_q == OWN_DP);
	assign xp_rvalid_o  = last_rd_w && (owner_q == OWN_XP);
	assign cpu_rvalid_o = rd_slot_w && (left_q == CPU_RD_LEFT) && (owner_q == OWN_CPU);
	assign busy_o       = busy_q;
	assign owner_o      = owner_q;
	assign xp_work_wait_o = busy_q && (owner_q == OWN_CPU);

	assign ss_rdata_o = (ss_addr_i == SS_WORD) ?
		{8'd0, open_row_q, open_ok_q, lost_rd_q, work_clks_q, lost_cpu_q, xp_lost_q, cpu_own_q, xp_beats_q, xp_row_q, xp_open_q, cpu_wins_q, refresh_due_q, phase_q, we_q, left_q, owner_q, busy_q} : 64'd0;

	always @(posedge clk_i) begin
		if (reset_i) begin
			busy_q        <= 1'b0;
			owner_q       <= OWN_REFRESH;
			left_q        <= 4'd0;
			we_q          <= 1'b0;
			wr_pend_q     <= 1'b0;
			addr_q        <= 16'd0;
			be_q          <= 2'd0;
			wdata_q       <= 16'd0;
			phase_q       <= 10'd0;
			refresh_due_q <= 1'b0;
			cpu_wins_q    <= 4'd0;
			cpu_own_q     <= 1'b0;
			xp_lost_q     <= 1'b0;
			lost_cpu_q    <= 1'b0;
			lost_rd_q     <= 1'b0;
			work_clks_q   <= 5'd0;
			xp_open_q     <= 1'b0;
			xp_row_q      <= 9'd0;
			xp_beats_q    <= 4'd0;
			open_row_q    <= 9'd0;
			open_ok_q     <= 1'b0;
		end else if (ss_we_i && (ss_addr_i == SS_WORD)) begin
			busy_q        <= ss_wdata_i[0];
			owner_q       <= ss_wdata_i[2:1];
			left_q        <= ss_wdata_i[6:3];
			we_q          <= ss_wdata_i[7];
			phase_q       <= ss_wdata_i[17:8];
			refresh_due_q <= ss_wdata_i[18];
			cpu_wins_q    <= ss_wdata_i[22:19];
			xp_open_q     <= ss_wdata_i[23];
			xp_row_q      <= ss_wdata_i[32:24];
			xp_beats_q    <= ss_wdata_i[36:33];
			cpu_own_q     <= ss_wdata_i[37];
			xp_lost_q     <= ss_wdata_i[38];
			lost_cpu_q    <= ss_wdata_i[39];
			work_clks_q   <= ss_wdata_i[44:40];
			lost_rd_q     <= ss_wdata_i[45];
			open_ok_q     <= ss_wdata_i[46];
			open_row_q    <= ss_wdata_i[55:47];
			wr_pend_q     <= 1'b0;
		end else begin
			if (phi1_i) begin
				wr_pend_q <= 1'b0;
			end

			if (ce_i) begin
				phase_q <= (phase_q == PERIOD_LAST) ? 10'd0 : phase_q + 10'd1;

				if (grant_w) begin
					busy_q    <= 1'b1;
					owner_q   <= pick_t;
					left_q    <= len_t - 4'd1;
					we_q      <= 1'b1;
					wr_pend_q <= 1'b0;
					case (pick_t)
						OWN_DP: begin
							we_q   <= 1'b0;
							addr_q <= dp_addr_i;
						end
						OWN_XP: begin
							we_q      <= xp_we_i;
							wr_pend_q <= xp_we_i;
							addr_q    <= xp_addr_i;
							be_q      <= xp_be_i;
							wdata_q   <= xp_wdata_i;
						end
						OWN_CPU: begin
							we_q      <= cpu_we_i;
							wr_pend_q <= cpu_we_i;
							addr_q    <= cpu_addr_i;
							be_q      <= cpu_be_i;
							wdata_q   <= cpu_wdata_i;
						end
						default: begin
						end
					endcase
				end else if (busy_q) begin
					if (left_q == 4'd0) begin
						busy_q <= 1'b0;
					end else begin
						left_q <= left_q - 4'd1;
					end
				end

				// The memory keeps the row of its last access open; refresh closes it.
				if (grant_w) begin
					open_ok_q  <= (pick_t != OWN_REFRESH);
					open_row_q <= (pick_t == OWN_XP) ? xp_row_w :
						(pick_t == OWN_DP) ? dp_row_full_w[8:0] : cpu_row_full_w[8:0];
				end

				// A refresh is due once per period while RE is set.
				if (grant_w && (pick_t == OWN_REFRESH)) begin
					refresh_due_q <= 1'b0;
				end
				if (!refresh_en_i) begin
					refresh_due_q <= 1'b0;
				end else if (phase_q == PERIOD_LAST) begin
					refresh_due_q <= 1'b1;
				end

				if (cpu_gnt_o) begin
					cpu_own_q <= 1'b1;
				end else if (xp_gnt_o) begin
					cpu_own_q <= 1'b0;
				end
				if (xp_gnt_o) begin
					xp_lost_q <= 1'b0;
				end else if (grant_w) begin
					xp_lost_q  <= 1'b1;
					lost_cpu_q <= cpu_gnt_o;
					lost_rd_q  <= (cpu_gnt_o && !cpu_we_i) || (pick_t == OWN_REFRESH);
				end

				// The XP's page stays open while it keeps the memory.
				if (xp_gnt_o) begin
					xp_open_q  <= 1'b1;
					xp_row_q   <= xp_row_w;
					xp_beats_q <= xp_hit_w ? xp_beats_q + 4'd1 : 4'd1;
				end else if (free_w) begin
					xp_open_q  <= 1'b0;
				end

				// The XP work runs in units like a burst and gives the CPU a new
				// chance at each unit's end.
				if (!xp_work_i || cpu_gnt_o) begin
					work_clks_q <= 5'd0;
				end else if (!xp_work_wait_o) begin
					work_clks_q <= (work_clks_q == WORK_UNIT) ? 5'd1 : work_clks_q + 5'd1;
				end
				if (xp_work_i && !xp_work_wait_o && !cpu_gnt_o) begin
					cpu_own_q <= 1'b0;
				end

				if (!xp_req_i || xp_gnt_o) begin
					cpu_wins_q <= 4'd0;
				end else if (cpu_gnt_o && !cpu_cont_i && (cpu_wins_q != 4'hf)) begin
					cpu_wins_q <= cpu_wins_q + 4'd1;
				end
			end
		end
	end

	// Savestate byte lane select follows the RAM read latency.
	always @(posedge clk_i) begin
		ssm_hi_q <= ssm_addr_i[0];
	end

	wire [15:0] ram_b_addr_w = ssm_active_i ? ssm_addr_i[16:1] : b_addr_i;
	wire        ssm_wr_w     = ssm_active_i && ssm_we_i;
	wire        a_lo_we_w    = phi1_i && wr_pend_q && be_q[0];
	wire        b_lo_we_w    = ssm_wr_w && !ssm_addr_i[0];
	wire [15:0] ram_a_q_w;
	wire [15:0] ram_b_q_w;

	cache_ram_dp #(
		.ADDR_WIDTH (16),
		.DATA_WIDTH (8)
	) u_lane_lo (
		.clk_i     (clk_i),
		.addr_a_i  (addr_q),
		.wren_a_i  (a_lo_we_w),
		.wdata_a_i (wdata_q[7:0]),
		.q_a_o     (ram_a_q_w[7:0]),
		.addr_b_i  (ram_b_addr_w),
		.wren_b_i  (ssm_wr_w && !ssm_addr_i[0]),
		.wdata_b_i (ssm_wdata_i),
		.q_b_o     (ram_b_q_w[7:0])
	);

	cache_ram_dp #(
		.ADDR_WIDTH (16),
		.DATA_WIDTH (8)
	) u_lane_hi (
		.clk_i     (clk_i),
		.addr_a_i  (addr_q),
		.wren_a_i  (phi1_i && wr_pend_q && be_q[1]),
		.wdata_a_i (wdata_q[15:8]),
		.q_a_o     (ram_a_q_w[15:8]),
		.addr_b_i  (ram_b_addr_w),
		.wren_b_i  (ssm_wr_w && ssm_addr_i[0]),
		.wdata_b_i (ssm_wdata_i),
		.q_b_o     (ram_b_q_w[15:8])
	);

	assign lo_we_o    = a_lo_we_w || b_lo_we_w;
	assign lo_addr_o  = b_lo_we_w ? ram_b_addr_w : addr_q;
	assign lo_wdata_o = b_lo_we_w ? ssm_wdata_i : wdata_q[7:0];

	assign dp_rdata_o  = ram_a_q_w;
	assign xp_rdata_o  = ram_a_q_w;
	assign cpu_rdata_o = ram_a_q_w;
	assign b_rdata_o   = ram_b_q_w;
	assign ssm_rdata_o = ssm_hi_q ? ram_b_q_w[15:8] : ram_b_q_w[7:0];

`ifndef SYNTHESIS
	initial begin
		if ((CPU_RD_SLOT < 1) || (CPU_RD_HOLD < 0) || (LEN_CPU_RD_I > 15) || (CPU_WR_SLOT < 1) || (CPU_WR_SLOT > 15) ||
			(XP_RD_SLOT < 1) || (XP_RD_SLOT > 15) || (XP_WR_SLOT < 1) || (XP_WR_SLOT > 15) ||
			(DP_RD_SLOT < 1) || (DP_RD_SLOT > 15) || (REFRESH_SLOT < 1) || (REFRESH_SLOT > 15)) begin
			$error("vip_mem_ctrl slot lengths must be 1..15");
		end
		if ((REFRESH_PERIOD < 2) || (REFRESH_PERIOD > 1024) || (XP_BURST < 1) || (XP_BURST > 15) ||
			(XP_MAX_WAIT < 0) || (XP_MAX_WAIT > 15) || (XP_WORK_UNIT < 1) || (XP_WORK_UNIT > 31)) begin
			$error("vip_mem_ctrl REFRESH_PERIOD must be 2..1024, XP_BURST 1..15, XP_MAX_WAIT 0..15, XP_WORK_UNIT 1..31");
		end
		if ((XP_REOPEN_CPU < 0) || (XP_REOPEN_CPU > 7) || (XP_REOPEN_RD < 0) || (XP_REOPEN_RD > 7) ||
			(XP_NEW_ROW < 0) || (XP_NEW_ROW > 7)) begin
			$error("vip_mem_ctrl XP_REOPEN_CPU, XP_REOPEN_RD and XP_NEW_ROW must be 0..7");
		end
	end
`endif

endmodule
