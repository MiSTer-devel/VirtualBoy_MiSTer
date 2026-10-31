// Copyright (c) 2026 Jamie Blanks

`timescale 1ns/1ps

// VIP host side: V810 bus frontend, address decode, registers and interrupts.
//
// Bus timing, in 50 ns clocks counted from the clock the address is on the
// pins (clock 1) to the CE edge that ends the access with READY:
//
//   register or unmapped read   REG_TOTAL     (3)
//   write                       2             when the write path is free
//   memory read                 MEM_RD_TOTAL  (7)  when granted at the end of clock 1
//
// The beat is captured on the phi1 inside clock 1. Writes are posted. A memory
// write goes into a one-halfword latch that asks its controller in the same
// clock, and keeps the write path busy WR_SLOT clocks from its grant, as does a
// register write. A write or read that finds the path busy waits for it, and
// tells its memory so (wait_o). A read ends one clock after rvalid, never
// before its total; a DRAM read keeps its data RD_LAG_D clocks longer first, so
// it ends when a VRAM read would. READY is a registered one-CE pulse published one
// clock before the accepting edge, and only while the live pins still show the
// captured beat. Outside VIP cycles ready_o stays low until a taken write's
// slot ends (at least WR_SLOT clocks), so the next bus cycle to anywhere waits
// for it. In CPU clocks:
//
//   clock      0    1    2    3    4    5    6    7    8    9   10
//   st.h       T1   T2   Tw                                          slot 2-5
//   st.h                 T1   T2   Tw   Tw                           slot 6-9
//   WRAM ld.h                           T1   T2   Tw   Tw            held in 9
//
// A memory beat that follows the last one to the same memory straight away
// (only its own T1 between them) holds that memory for the CPU. A halfword
// beat at A1 = 1 that follows the beat at A1 = 0 of the same word that way is
// the high half of a word, and the memory treats both halves as one access.
module vip_host
#(
	parameter integer REG_TOTAL    = 3,
	parameter integer MEM_RD_TOTAL = 7,
	parameter integer WR_SLOT      = 4,   // CPU write cycle, at least the controllers' CPU write slot
	parameter integer CPU_RD_SLOT  = 4,   // VRAM read data clock; DRAM read data clock plus RD_LAG_D
	parameter integer RD_LAG_D     = 0
)
(
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        ce_i,
	input  wire        phi1_i,

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

	// CPU client ports of the VRAM and DRAM controllers.
	output wire        vram_req_o,
	output wire        vram_we_o,
	output wire        vram_cont_o,      // the high half of a word
	output wire        vram_hold_o,      // the beat follows the last VRAM beat back to back
	output wire        vram_wait_o,      // a beat waits for the write path
	output wire [15:0] vram_addr_o,
	output wire [1:0]  vram_be_o,
	output wire [15:0] vram_wdata_o,
	input  wire        vram_gnt_i,
	input  wire        vram_rvalid_i,
	input  wire [15:0] vram_rdata_i,
	output wire        dram_req_o,
	output wire        dram_we_o,
	output wire        dram_cont_o,
	output wire        dram_hold_o,
	output wire        dram_wait_o,
	output wire [15:0] dram_addr_o,
	output wire [1:0]  dram_be_o,
	output wire [15:0] dram_wdata_o,
	input  wire        dram_gnt_i,
	input  wire        dram_rvalid_i,
	input  wire [15:0] dram_rdata_i,

	// Interrupt events, one-clock pulses on CE.
	input  wire        ev_scanerr_i,
	input  wire        ev_lfbend_i,
	input  wire        ev_rfbend_i,
	input  wire        ev_gamestart_i,   // GCLK rise; FRMCYC takes effect
	input  wire        ev_framestart_i,  // FCLK rise; DISP and SYNCE take effect
	input  wire        ev_sbhit_i,
	input  wire        ev_xpend_i,
	input  wire        ev_timeerr_i,

	// Unit status shown in DPSTTS and XPSTTS.
	input  wire [3:0]  dpbsy_i,          // {R1BSY, L1BSY, R0BSY, L0BSY}
	input  wire        scanrdy_i,
	input  wire        fclk_i,
	input  wire [1:0]  xpbsy_i,
	input  wire [4:0]  sbcount_i,
	input  wire        sbout_i,
	input  wire        overtime_i,

	// Points where the units take new register values.
	input  wire        dp_brt_latch_i,   // DP starts a column group: BRTA/B/C and REST take effect
	input  wire [1:0]  dp_cta_we_i,      // eye start at SYNC: bit 0 loads CTA_L, bit 1 CTA_R
	input  wire [15:0] dp_cta_i,         // {CTA_R, CTA_L} start values
	input  wire        xp_start_i,       // draw start: SPT takes effect
	input  wire        xp_bkcol_latch_i, // first strip of a draw done: BKCOL takes effect

	// Command pulses, one CE period long.
	output reg         dprst_o,
	output reg         xprst_o,

	// Register values for the units.
	output wire        disp_o,           // active DISP
	output wire        synce_o,          // active SYNCE
	output wire        re_o,
	output wire        lock_o,
	output wire        xpen_o,
	output wire [4:0]  sbcmp_o,
	output wire [31:0] brt_o,            // active {REST, BRTC, BRTB, BRTA}
	output wire [3:0]  frmcyc_o,         // active FRMCYC
	output wire [39:0] spt_o,            // active {SPT3, SPT2, SPT1, SPT0}
	output wire [31:0] gplt_o,           // {GPLT3, ..., GPLT0}, as written
	output wire [31:0] jplt_o,           // {JPLT3, ..., JPLT0}, as written
	output wire [1:0]  bkcol_o,          // active BKCOL
	output wire [15:0] cta_o,            // CTA as read, {CTA_R, CTA_L}; held while LOCK is set

	input  wire [5:0]  ss_addr_i,
	input  wire [63:0] ss_wdata_i,
	input  wire        ss_we_i,
	output reg  [63:0] ss_rdata_o,

	output wire [15:0] int_pending_o,
	output wire        busy_o            // a beat, a posted write or a DPRST register seed is in progress
);
	localparam [15:0] INT_MASK     = 16'he01f;
	localparam [15:0] DPRST_CLEAR  = 16'h801f;
	localparam [15:0] XPRST_CLEAR  = 16'he000;

	// Minimum age, in CE edges after the first, at which READY may be published.
	localparam integer REG_MIN_I   = REG_TOTAL - 2;
	localparam integer RD_MIN_I    = MEM_RD_TOTAL - 2;
	localparam [3:0]   REG_MIN     = REG_MIN_I[3:0];
	localparam [3:0]   RD_MIN      = RD_MIN_I[3:0];
	localparam [2:0]   WR_LEN      = WR_SLOT[2:0];
	localparam [1:0]   LAG_D       = RD_LAG_D[1:0];

	// Register word index, a[6:1] within 0x5F800.
	localparam [5:0] R_INTPND = 6'h00;
	localparam [5:0] R_INTENB = 6'h01;
	localparam [5:0] R_INTCLR = 6'h02;
	localparam [5:0] R_DPSTTS = 6'h10;
	localparam [5:0] R_DPCTRL = 6'h11;
	localparam [5:0] R_BRTA   = 6'h12;
	localparam [5:0] R_BRTB   = 6'h13;
	localparam [5:0] R_BRTC   = 6'h14;
	localparam [5:0] R_REST   = 6'h15;
	localparam [5:0] R_FRMCYC = 6'h17;
	localparam [5:0] R_CTA    = 6'h18;
	localparam [5:0] R_XPSTTS = 6'h20;
	localparam [5:0] R_XPCTRL = 6'h21;
	localparam [5:0] R_VER    = 6'h22;
	localparam [5:0] R_BKCOL  = 6'h38;

	// ------------------------------------------------------------------
	// Bus frontend

	reg        act_q;      // a beat is captured
	reg        ret_q;      // READY was sent for the beat still on the pins
	reg        gnt_q;      // memory took the read
	reg        rd_ok_q;    // read data is in dout_q and may end the read
	reg [1:0]  rd_lag_q;   // clocks left before it may
	reg        in_q;       // the write is in the latch
	reg        done_q;     // work done, waiting for the pins to show the beat again
	reg        src_ok_q;   // src_q holds the write data
	reg        ready_q;
	reg [3:0]  age_q;      // CE edges since the first one after capture
	reg [15:0] dout_q;
	reg [23:1] a_q;
	reg [3:0]  be_q;
	reg [1:0]  st_q;
	reg        da_q;
	reg        wr_q;
	reg        byte_q;
	reg        hi_q;
	reg [15:0] src_q;
	reg        vram_q;
	reg        dram_q;
	reg        regs_q;
	reg        cont_q;     // the beat is the high half of a word
	reg        hold_q;     // the beat follows the last beat to its memory back to back
	reg        mem_ok_q;   // the last beat was a memory beat, ended just now
	reg        mem_v_q;    // it went to VRAM
	reg        lo_ok_q;    // it was a word's low half
	reg        gap_q;      // a phi1 without a beat since (the next beat's T1)
	reg [23:2] lo_a_q;
	reg        lo_wr_q;

	// One posted CPU write per VIP, to either memory.
	reg        wl_full_q;
	reg [2:0]  wl_left_q;  // clocks left in the posted write's slot
	reg        wl_mem_q;   // and that write went to a memory
	reg        wl_vram_q;
	reg        wl_cont_q;
	reg        wl_hold_q;
	reg [15:0] wl_addr_q;
	reg [1:0]  wl_be_q;
	reg [15:0] wl_data_q;

	wire byte_lo_w = (be_i == 4'b1110);
	wire byte_hi_w = (be_i == 4'b1101);
	wire bus_wr_w  = !rw_i && da_i && (st_i == 2'b10);
	wire beat_w    = cs_i && mrq_i && (byte_lo_w || byte_hi_w || (be_i == 4'b1100)) && (rw_i || bus_wr_w);
	// Write data settles after the phi1 capture, so the first CE takes it.
	wire [15:0] src_w = src_ok_q ? src_q : din_i;
	wire in_vram_w = (a_i[18:17] == 2'b00) || (a_i[18:15] == 4'b1111);
	wire in_dram_w = (a_i[18:17] == 2'b01);
	wire hold_w = mem_ok_q && (mem_v_q ? in_vram_w : in_dram_w);
	wire cont_w = hold_w && lo_ok_q && a_i[1] && (a_i[23:2] == lo_a_q[23:2]) && (be_i == 4'b1100) && (bus_wr_w == lo_wr_q);
	wire same_w = beat_w && (a_i == a_q) && (be_i == be_q) && (st_i == st_q) && (da_i == da_q) &&
		(bus_wr_w == wr_q) && (!wr_q || (din_i == src_w));

	wire        mem_w    = vram_q || dram_q;
	wire        wl_gnt_w = wl_full_q && (wl_vram_q ? vram_gnt_i : dram_gnt_i);
	// The write path frees at the edge that ends the posted write's slot.
	wire        wl_free_w = !wl_full_q && (wl_left_q <= 3'd1);
	// A memory write that may enter the latch asks its controller directly.
	wire        dir_w    = act_q && mem_w && wr_q && !in_q && !done_q && wl_free_w;
	wire        dir_gnt_w = dir_w && ((vram_q && vram_gnt_i) || (dram_q && dram_gnt_i));
	// A read waits for the bus cycle of the last write to its memory.
	wire        mem_path_w = wl_mem_q && (wl_left_q > 3'd1) && (wl_vram_q == vram_q);
	wire        bus_wait_w = act_q && mem_w && !done_q &&
		(wr_q ? (!in_q && !wl_free_w) : (!gnt_q && !wl_full_q && mem_path_w));
	wire        rd_req_w = act_q && mem_w && !wr_q && !gnt_q && !wl_full_q && !mem_path_w;
	wire        rd_gnt_w = rd_req_w && ((vram_q && vram_gnt_i) || (dram_q && dram_gnt_i));
	wire        rvalid_w = gnt_q && !rd_ok_q && (rd_lag_q == 2'd0) && ((vram_q && vram_rvalid_i) || (dram_q && dram_rvalid_i));
	wire [15:0] rdata_w  = vram_q ? vram_rdata_i : dram_rdata_i;
	wire        enter_w  = ce_i && dir_w;
	wire        fire_w   = ce_i && act_q && !done_q && (
		!mem_w ? (wr_q ? wl_free_w : (age_q == REG_MIN)) :
		wr_q   ? (in_q || enter_w) :
		         (rd_ok_q && (age_q >= RD_MIN)));
	wire        commit_w = fire_w && regs_q && wr_q;

	// 0x78000-0x7FFFF repeats the four 8 KiB character blocks of VRAM.
	wire        alias_w  = (a_q[18:15] == 4'b1111);
	wire [15:0] maddr_w  = alias_w ? {a_q[14:13], 2'b11, a_q[12:1]} : a_q[16:1];
	wire [7:0]  wbyte_w  = hi_q ? src_w[15:8] : src_w[7:0];
	wire [15:0] mwdata_w = byte_q ? {wbyte_w, wbyte_w} : src_w;
	wire [1:0]  mbe_w    = byte_q ? (hi_q ? 2'b10 : 2'b01) : 2'b11;
	// Odd-address register byte writes move the byte to the high half.
	wire [15:0] rwdata_w = (byte_q && hi_q) ? {src_w[15:8], 8'd0} : src_w;
	wire [5:0]  ridx_w   = a_q[6:1];

	reg  [15:0] reg_rd_t;
	wire [15:0] resp_w   = !mem_w ? (regs_q ? reg_rd_t : 16'd0) : rdata_w;
	wire [15:0] shaped_w = byte_q ? (hi_q ? {resp_w[15:8], 8'd0} : {8'd0, resp_w[7:0]}) : resp_w;

	// While the latch is full it owns both client ports; reads wait for it.
	assign vram_req_o   = wl_full_q ? wl_vram_q : ((dir_w || rd_req_w) && vram_q);
	assign dram_req_o   = wl_full_q ? !wl_vram_q : ((dir_w || rd_req_w) && dram_q);
	assign vram_we_o    = wl_full_q || dir_w;
	assign dram_we_o    = wl_full_q || dir_w;
	assign vram_cont_o  = wl_full_q ? wl_cont_q : cont_q;
	assign dram_cont_o  = wl_full_q ? wl_cont_q : cont_q;
	assign vram_hold_o  = wl_full_q ? wl_hold_q : hold_q;
	assign dram_hold_o  = wl_full_q ? wl_hold_q : hold_q;
	assign vram_wait_o  = bus_wait_w && vram_q;
	assign dram_wait_o  = bus_wait_w && dram_q;
	assign vram_addr_o  = wl_full_q ? wl_addr_q : maddr_w;
	assign dram_addr_o  = wl_full_q ? wl_addr_q : maddr_w;
	assign vram_be_o    = wl_full_q ? wl_be_q : mbe_w;
	assign dram_be_o    = wl_full_q ? wl_be_q : mbe_w;
	assign vram_wdata_o = wl_full_q ? wl_data_q : mwdata_w;
	assign dram_wdata_o = wl_full_q ? wl_data_q : mwdata_w;
	assign dout_o       = dout_q;
	assign ready_o      = ready_q || (!cs_i && !wl_full_q && (wl_left_q == 3'd0));

	always @(posedge clk_i) begin
		if (reset_i) begin
			act_q     <= 1'b0;
			ret_q     <= 1'b0;
			gnt_q     <= 1'b0;
			rd_ok_q   <= 1'b0;
			rd_lag_q  <= 2'd0;
			in_q      <= 1'b0;
			done_q    <= 1'b0;
			src_ok_q  <= 1'b0;
			ready_q   <= 1'b0;
			age_q     <= 4'd0;
			dout_q    <= 16'd0;
			a_q       <= 23'd0;
			be_q      <= 4'd0;
			st_q      <= 2'd0;
			da_q      <= 1'b0;
			wr_q      <= 1'b0;
			byte_q    <= 1'b0;
			hi_q      <= 1'b0;
			src_q     <= 16'd0;
			vram_q    <= 1'b0;
			dram_q    <= 1'b0;
			regs_q    <= 1'b0;
			cont_q    <= 1'b0;
			hold_q    <= 1'b0;
			mem_ok_q  <= 1'b0;
			mem_v_q   <= 1'b0;
			lo_ok_q   <= 1'b0;
			gap_q     <= 1'b0;
			lo_a_q    <= 22'd0;
			lo_wr_q   <= 1'b0;
			wl_full_q <= 1'b0;
			wl_left_q <= 3'd0;
			wl_mem_q  <= 1'b0;
			wl_vram_q <= 1'b0;
			wl_cont_q <= 1'b0;
			wl_hold_q <= 1'b0;
			wl_addr_q <= 16'd0;
			wl_be_q   <= 2'd0;
			wl_data_q <= 16'd0;
		end else begin
			// A new beat, or BCYST on an identical one, starts an access.
			if (phi1_i && !act_q) begin
				if (!beat_w) begin
					ret_q    <= 1'b0;
					gap_q    <= 1'b1;
					if (gap_q) begin
						mem_ok_q <= 1'b0;
						lo_ok_q  <= 1'b0;
					end
				end else if (bcyst_i || !ret_q || !same_w) begin
					cont_q   <= cont_w;
					hold_q   <= hold_w;
					mem_ok_q <= 1'b0;
					lo_ok_q  <= 1'b0;
					act_q    <= 1'b1;
					ret_q    <= 1'b0;
					gnt_q    <= 1'b0;
					rd_ok_q  <= 1'b0;
					rd_lag_q <= 2'd0;
					in_q     <= 1'b0;
					done_q   <= 1'b0;
					src_ok_q <= 1'b0;
					age_q    <= 4'd0;
					a_q      <= a_i;
					be_q     <= be_i;
					st_q     <= st_i;
					da_q     <= da_i;
					wr_q     <= bus_wr_w;
					byte_q   <= byte_lo_w || byte_hi_w;
					hi_q     <= byte_hi_w;
					src_q    <= din_i;
					vram_q   <= (a_i[18:17] == 2'b00) || (a_i[18:15] == 4'b1111);
					dram_q   <= (a_i[18:17] == 2'b01);
					regs_q   <= (a_i[18:7] == 12'hbf0);
				end
			end

			if (ce_i) begin
				ready_q <= 1'b0;

				if (enter_w) begin
					wl_full_q <= !dir_gnt_w;
					wl_vram_q <= vram_q;
					wl_cont_q <= cont_q;
					wl_hold_q <= hold_q;
					wl_addr_q <= maddr_w;
					wl_be_q   <= mbe_w;
					wl_data_q <= mwdata_w;
				end else if (wl_gnt_w) begin
					wl_full_q <= 1'b0;
				end

				if (dir_gnt_w || wl_gnt_w || (fire_w && wr_q && !mem_w)) begin
					wl_left_q <= WR_LEN;
					wl_mem_q  <= dir_gnt_w || wl_gnt_w;
				end else if (wl_left_q != 3'd0) begin
					wl_left_q <= wl_left_q - 3'd1;
				end

				if (act_q) begin
					if (age_q != 4'hf) begin
						age_q <= age_q + 4'd1;
					end
					if (!src_ok_q) begin
						src_q    <= din_i;
						src_ok_q <= 1'b1;
					end
					if (enter_w) begin
						in_q <= 1'b1;
					end
					if (rd_gnt_w) begin
						gnt_q <= 1'b1;
					end
					if (rvalid_w) begin
						rd_ok_q  <= vram_q || (RD_LAG_D == 0);
						rd_lag_q <= vram_q ? 2'd0 : LAG_D;
						dout_q   <= shaped_w;
					end else if (rd_lag_q != 2'd0) begin
						rd_ok_q  <= (rd_lag_q == 2'd1);
						rd_lag_q <= rd_lag_q - 2'd1;
					end else if (fire_w && !mem_w) begin
						dout_q <= shaped_w;
					end

					if (fire_w || done_q) begin
						if (same_w) begin
							act_q    <= 1'b0;
							done_q   <= 1'b0;
							ret_q    <= 1'b1;
							ready_q  <= 1'b1;
							mem_ok_q <= mem_w;
							mem_v_q  <= vram_q;
							lo_ok_q  <= mem_w && !a_q[1] && (be_q == 4'b1100) && !cont_q;
							gap_q    <= 1'b0;
							lo_a_q   <= a_q[23:2];
							lo_wr_q  <= wr_q;
						end else begin
							done_q <= 1'b1;
						end
					end
				end
			end
		end
	end

	// ------------------------------------------------------------------
	// Registers and interrupts

	reg [15:0] pend_q;
	reg [15:0] enb_q;
	reg        disp_q;
	reg        disp_a_q;
	reg        synce_q;
	reg        synce_a_q;
	reg        re_q;
	reg        lock_q;
	reg        xpen_q;
	reg [4:0]  sbcmp_q;
	reg [31:0] brt_q;      // {REST, BRTC, BRTB, BRTA}
	reg [31:0] brt_a_q;
	reg [3:0]  frmcyc_q;
	reg [3:0]  frmcyc_a_q;
	reg [7:0]  cta_l_q;
	reg [7:0]  cta_r_q;
	reg [39:0] spt_q;
	reg [39:0] spt_a_q;
	reg [31:0] gplt_q;
	reg [31:0] jplt_q;
	reg [1:0]  bkcol_q;
	reg [1:0]  bkcol_a_q;
	reg [1:0]  seed_cnt_q;
	reg [15:0] seed_q;

	always @* begin
		reg_rd_t = 16'd0;
		case (ridx_w)
			R_INTPND: reg_rd_t = pend_q;
			R_INTENB: reg_rd_t = enb_q;
			R_DPSTTS: reg_rd_t = {5'd0, lock_q, synce_a_q, re_q, fclk_i, scanrdy_i, dpbsy_i, disp_a_q, 1'b0};
			R_BRTA:   reg_rd_t = {8'd0, brt_q[7:0]};
			R_BRTB:   reg_rd_t = {8'd0, brt_q[15:8]};
			R_BRTC:   reg_rd_t = {8'd0, brt_q[23:16]};
			R_REST:   reg_rd_t = {8'd0, brt_q[31:24]};
			R_FRMCYC: reg_rd_t = {12'd0, frmcyc_q};
			R_CTA:    reg_rd_t = {cta_r_q, cta_l_q};
			R_XPSTTS: reg_rd_t = {sbout_i, 2'd0, sbcount_i, 3'd0, overtime_i, xpbsy_i, xpen_q, 1'b0};
			R_VER:    reg_rd_t = 16'h0002;
			6'h24:    reg_rd_t = {6'd0, spt_q[9:0]};
			6'h25:    reg_rd_t = {6'd0, spt_q[19:10]};
			6'h26:    reg_rd_t = {6'd0, spt_q[29:20]};
			6'h27:    reg_rd_t = {6'd0, spt_q[39:30]};
			6'h30:    reg_rd_t = {8'd0, gplt_q[7:0]};
			6'h31:    reg_rd_t = {8'd0, gplt_q[15:8]};
			6'h32:    reg_rd_t = {8'd0, gplt_q[23:16]};
			6'h33:    reg_rd_t = {8'd0, gplt_q[31:24]};
			6'h34:    reg_rd_t = {8'd0, jplt_q[7:0]};
			6'h35:    reg_rd_t = {8'd0, jplt_q[15:8]};
			6'h36:    reg_rd_t = {8'd0, jplt_q[23:16]};
			6'h37:    reg_rd_t = {8'd0, jplt_q[31:24]};
			R_BKCOL:  reg_rd_t = {14'd0, bkcol_q};
			default:  reg_rd_t = 16'd0;
		endcase
	end

	wire        wr_dpctrl_w = commit_w && (ridx_w == R_DPCTRL);
	wire        wr_xpctrl_w = commit_w && (ridx_w == R_XPCTRL);
	wire        dprst_w     = wr_dpctrl_w && rwdata_w[0];
	wire        xprst_w     = wr_xpctrl_w && rwdata_w[0];
	wire [15:0] events_w    = {ev_timeerr_i, ev_xpend_i, ev_sbhit_i, 8'd0, ev_framestart_i,
		ev_gamestart_i, ev_rfbend_i, ev_lfbend_i, ev_scanerr_i};
	wire [15:0] pclr_w      = ((commit_w && (ridx_w == R_INTCLR)) ? rwdata_w : 16'd0) |
		(dprst_w ? DPRST_CLEAR : 16'd0) | (xprst_w ? XPRST_CLEAR : 16'd0);
	wire [15:0] eclr_w      = (dprst_w ? DPRST_CLEAR : 16'd0) | (xprst_w ? XPRST_CLEAR : 16'd0);
	wire [15:0] enb_next_w  = (commit_w && (ridx_w == R_INTENB)) ? (rwdata_w & INT_MASK) : enb_q;
	// After DPRST the display registers settle to whatever the data bus carries for a few clocks.
	wire        busv_w      = (mrq_i && !rw_i) || (ready_q && !wr_q);
	wire [15:0] seed_w      = (mrq_i && !rw_i) ? din_i : (busv_w ? dout_q : seed_q);
	wire [7:0]  gpw_w       = rwdata_w[7:0] & 8'hfc;

	always @(posedge clk_i) begin
		if (reset_i) begin
			pend_q     <= 16'd0;
			enb_q      <= 16'd0;
			disp_q     <= 1'b0;
			disp_a_q   <= 1'b0;
			synce_q    <= 1'b0;
			synce_a_q  <= 1'b0;
			re_q       <= 1'b0;
			lock_q     <= 1'b0;
			xpen_q     <= 1'b0;
			sbcmp_q    <= 5'd0;
			brt_q      <= 32'd0;
			brt_a_q    <= 32'd0;
			frmcyc_q   <= 4'd0;
			frmcyc_a_q <= 4'd0;
			cta_l_q    <= 8'd0;
			cta_r_q    <= 8'd0;
			spt_q      <= 40'd0;
			spt_a_q    <= 40'd0;
			gplt_q     <= 32'd0;
			jplt_q     <= 32'd0;
			bkcol_q    <= 2'd0;
			bkcol_a_q  <= 2'd0;
			seed_cnt_q <= 2'd0;
			seed_q     <= 16'd0;
			dprst_o    <= 1'b0;
			xprst_o    <= 1'b0;
		end else if (ss_we_i && (ss_addr_i < 6'd24)) begin
			case (ss_addr_i)
				6'd0: begin
					pend_q     <= ss_wdata_i[15:0];
					enb_q      <= ss_wdata_i[31:16];
				end
				6'd1: begin
					{xpen_q, sbcmp_q, lock_q, re_q, synce_q, synce_a_q, disp_q, disp_a_q} <= ss_wdata_i[11:0];
					{frmcyc_q, frmcyc_a_q, bkcol_q, bkcol_a_q} <= ss_wdata_i[23:12];
					{cta_r_q, cta_l_q} <= ss_wdata_i[39:24];
				end
				6'd2: {brt_a_q, brt_q}   <= ss_wdata_i;
				6'd3: spt_q              <= ss_wdata_i[39:0];
				6'd4: spt_a_q            <= ss_wdata_i[39:0];
				6'd5: gplt_q             <= ss_wdata_i[31:0];
				6'd6: jplt_q             <= ss_wdata_i[31:0];
				default: begin
				end
			endcase
			seed_cnt_q <= 2'd0;
			dprst_o    <= 1'b0;
			xprst_o    <= 1'b0;
		end else if (ce_i) begin
			dprst_o <= dprst_w;
			xprst_o <= xprst_w;

			// An event in the same clock as its clear stays pending.
			pend_q <= ((pend_q & ~pclr_w) | events_w) & INT_MASK;
			enb_q  <= enb_next_w & ~eclr_w;

			if (commit_w) begin
				case (ridx_w)
					R_DPCTRL: begin
						lock_q  <= rwdata_w[10];
						synce_q <= rwdata_w[9];
						re_q    <= rwdata_w[8];
						disp_q  <= rwdata_w[1];
					end
					R_BRTA:   brt_q[7:0]   <= rwdata_w[7:0];
					R_BRTB:   brt_q[15:8]  <= rwdata_w[7:0];
					R_BRTC:   brt_q[23:16] <= rwdata_w[7:0];
					R_REST:   brt_q[31:24] <= rwdata_w[7:0];
					R_FRMCYC: frmcyc_q     <= rwdata_w[3:0];
					R_XPCTRL: begin
						sbcmp_q <= rwdata_w[12:8];
						xpen_q  <= rwdata_w[1];
					end
					6'h24:    spt_q[9:0]     <= rwdata_w[9:0];
					6'h25:    spt_q[19:10]   <= rwdata_w[9:0];
					6'h26:    spt_q[29:20]   <= rwdata_w[9:0];
					6'h27:    spt_q[39:30]   <= rwdata_w[9:0];
					6'h30:    gplt_q[7:0]    <= gpw_w;
					6'h31:    gplt_q[15:8]   <= gpw_w;
					6'h32:    gplt_q[23:16]  <= gpw_w;
					6'h33:    gplt_q[31:24]  <= gpw_w;
					6'h34:    jplt_q[7:0]    <= gpw_w;
					6'h35:    jplt_q[15:8]   <= gpw_w;
					6'h36:    jplt_q[23:16]  <= gpw_w;
					6'h37:    jplt_q[31:24]  <= gpw_w;
					R_BKCOL:  bkcol_q        <= rwdata_w[1:0];
					default: begin
					end
				endcase
			end

			// Active copies take the programmed value held before this edge.
			if (ev_framestart_i) begin
				synce_a_q <= synce_q;
				disp_a_q  <= disp_q;
			end
			if (dp_brt_latch_i) begin
				brt_a_q <= brt_q;
			end
			if (ev_gamestart_i) begin
				frmcyc_a_q <= frmcyc_q;
			end
			if (dp_cta_we_i[0] && !lock_q) begin
				cta_l_q <= dp_cta_i[7:0];
			end
			if (dp_cta_we_i[1] && !lock_q) begin
				cta_r_q <= dp_cta_i[15:8];
			end
			if (xp_start_i) begin
				spt_a_q  <= spt_q;
			end
			if (xp_bkcol_latch_i) begin
				bkcol_a_q <= bkcol_q;
			end

			if (dprst_w) begin
				seed_cnt_q <= 2'd3;
				seed_q     <= rwdata_w;
			end else if (seed_cnt_q != 2'd0) begin
				seed_cnt_q <= seed_cnt_q - 2'd1;
				seed_q     <= seed_w;
				if (seed_cnt_q == 2'd1) begin
					brt_q    <= {4{seed_w[7:0]}};
					frmcyc_q <= seed_w[3:0];
					cta_l_q  <= seed_w[7:0];
					cta_r_q  <= seed_w[15:8];
				end
			end
		end
	end

	always @* begin
		case (ss_addr_i)
			6'd0:    ss_rdata_o = {32'd0, enb_q, pend_q};
			6'd1:    ss_rdata_o = {24'd0, cta_r_q, cta_l_q, frmcyc_q, frmcyc_a_q, bkcol_q, bkcol_a_q,
				xpen_q, sbcmp_q, lock_q, re_q, synce_q, synce_a_q, disp_q, disp_a_q};
			6'd2:    ss_rdata_o = {brt_a_q, brt_q};
			6'd3:    ss_rdata_o = {24'd0, spt_q};
			6'd4:    ss_rdata_o = {24'd0, spt_a_q};
			6'd5:    ss_rdata_o = {32'd0, gplt_q};
			6'd6:    ss_rdata_o = {32'd0, jplt_q};
			default: ss_rdata_o = 64'd0;
		endcase
	end

	assign irq_o         = |(pend_q & enb_q);
	assign int_pending_o = pend_q;
	assign busy_o        = act_q || wl_full_q || (wl_left_q != 3'd0) || (seed_cnt_q != 2'd0);
	assign disp_o        = disp_a_q;
	assign synce_o       = synce_a_q;
	assign re_o          = re_q;
	assign lock_o        = lock_q;
	assign xpen_o        = xpen_q;
	assign sbcmp_o       = sbcmp_q;
	assign brt_o         = brt_a_q;
	assign frmcyc_o      = frmcyc_a_q;
	assign spt_o         = spt_a_q;
	assign gplt_o        = gplt_q;
	assign jplt_o        = jplt_q;
	assign bkcol_o       = bkcol_a_q;
	assign cta_o         = {cta_r_q, cta_l_q};

`ifndef SYNTHESIS
	initial begin
		if ((REG_MIN_I < 0) || (RD_MIN_I < CPU_RD_SLOT + 1) || (RD_LAG_D < 0) || (RD_LAG_D > 3) || (WR_SLOT < 1) ||
			(REG_MIN_I > 15) || (RD_MIN_I > 15) || (WR_SLOT > 7)) begin
			$error("vip_host bus totals do not fit the slot lengths");
		end
	end
`endif

endmodule
