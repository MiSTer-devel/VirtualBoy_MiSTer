// Copyright (c) 2026 Jamie Blanks

module vb_vue_addr_decode
(
	input  wire       mrq_i,
	input  wire [2:0] region_i,

	output wire       vip_cs_o,
	output wire       vsu_cs_o,
	output wire       misc_cs_o,
	output wire       unused_cs_o,
	output wire       exp_cs_o,
	output wire       wram_cs_o,
	output wire       sram_cs_o,
	output wire       rom_cs_o
);

	assign vip_cs_o    = mrq_i && (region_i == 3'b000);
	assign vsu_cs_o    = mrq_i && (region_i == 3'b001);
	assign misc_cs_o   = mrq_i && (region_i == 3'b010);
	assign unused_cs_o = mrq_i && (region_i == 3'b011);
	assign exp_cs_o    = mrq_i && (region_i == 3'b100);
	assign wram_cs_o   = mrq_i && (region_i == 3'b101);
	assign sram_cs_o   = mrq_i && (region_i == 3'b110);
	assign rom_cs_o    = mrq_i && (region_i == 3'b111);

endmodule

// Hardware wait-state generator for the VUE bus regions.
//
// Models only original-hardware READY behavior: the WCR-programmed ROM/SRAM/EXP
// waits (2 waits, or 1 with the WCR bit set), the fixed one-wait regions, and
// the VIP holding READY during its own accesses. The MiSTer SDRAM backup hold
// is applied outside this module, ANDed into the CPU READY pin.
// Timing runs on CE: a one-wait access spends one CE cycle reading and one
// waiting, and the CPU latches DIN at the CE edge that ends the wait.
//
// Every 320 clocks the WRAM refresh pulses for 6 clocks, and no cycle starts
// until 2 clocks after the pulse. It starts on an idle clock, or on a T1 once
// it has waited behind a cycle, so back-to-back cycles cannot keep it off; a
// T1 on the clock it falls due goes first. A cycle in any region waits it
// out, then takes its normal wait; VIP accesses are held off the VIP until
// it ends.
module vb_vue_wait_control
(
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        ce_i,

	input  wire        mrq_i,
	input  wire        bcyst_i,
	input  wire [26:1] a_i,
	input  wire [3:0]  be_i,
	input  wire [1:0]  st_i,
	input  wire        da_i,
	input  wire        rw_i,

	input  wire        vip_cs_i,
	input  wire        vsu_cs_i,
	input  wire        misc_cs_i,
	input  wire        unused_cs_i,
	input  wire        exp_cs_i,
	input  wire        wram_cs_i,
	input  wire        sram_cs_i,
	input  wire        rom_cs_i,

	input  wire        wait_exp1_i,
	input  wire        wait_rom1_i,
	input  wire        vip_ready_i,
	output wire        ready_o,
	input  wire        refresh_tick_i,    // The prescaler's 320-clock tick: refresh falls due.
	output wire        refresh_busy_o,
	input  wire [63:0] savestate_state_wdata_i,
	input  wire        savestate_state_wren_i,
	output wire [63:0] savestate_state_rdata_o
);

	localparam [1:0] WAIT_NONE = 2'd0;
	localparam [1:0] WAIT_1    = 2'd1;
	localparam [1:0] WAIT_2    = 2'd2;

	reg        txn_active_q;
	reg [26:1] txn_addr_q;
	reg [3:0]  txn_be_q;
	reg [1:0]  txn_st_q;
	reg        txn_da_q;
	reg        txn_rw_q;
	reg [1:0]  wait_count_q;
	reg        wait_use_vip_q;
	reg [1:0]  wait_count_start_r;
	reg        wait_use_vip_r;
	reg        refresh_due_q;
	reg [3:0]  refresh_left_q;
	reg        refresh_wait_q;

	// Every cycle and split beat shows a T1 first, and MRQ is masked in T1,
	// so a new access always starts from an idle transaction.
	wire       txn_start_w = mrq_i && !txn_active_q;
`ifndef SYNTHESIS
	// synthesis translate_off
	always @(posedge clk_i) begin
		if (!reset_i && ce_i && mrq_i && txn_active_q && (bcyst_i || (txn_addr_q != a_i) ||
			(txn_be_q != be_i) || (txn_st_q != st_i) || (txn_da_q != da_i) || (txn_rw_q != rw_i))) begin
			$display("FAIL wait control: a new access without an idle transaction at %h", {a_i, 1'b0});
			$fatal(1);
		end
	end
	// synthesis translate_on
`endif
	// Pulse and the two clocks after it.
	localparam [3:0] REFRESH_HOLD = 4'd8;
	wire       refresh_start_w = refresh_due_q && !mrq_i && (!bcyst_i || refresh_wait_q);
	wire       refresh_block_w = txn_start_w && (refresh_left_q != 4'd0);
	assign     refresh_busy_o = (refresh_left_q != 4'd0);

	always @(*) begin
		wait_count_start_r = WAIT_NONE;
		wait_use_vip_r     = 1'b0;

		// WCR sets only the ROM and expansion waits; cartridge RAM is a fixed 1.
		if (rom_cs_i) begin
			wait_count_start_r = wait_rom1_i ? WAIT_1 : WAIT_2;
		end else if (exp_cs_i) begin
			wait_count_start_r = wait_exp1_i ? WAIT_1 : WAIT_2;
		end else if (sram_cs_i || wram_cs_i || unused_cs_i || misc_cs_i || vsu_cs_i) begin
			wait_count_start_r = WAIT_1;
		end else if (vip_cs_i) begin
			wait_use_vip_r     = 1'b1;
		end
	end

	// READY is a live logical acceptance signal: combinational after the
	// registered wait count (and the VIP hold) so the CPU can consume it with
	// DIN at the very CE edge the wait expires. The VIP's READY gates every
	// cycle: outside its own it stays low for a few clocks after a VIP write.
	assign ready_o =
		!mrq_i ? 1'b1 :
		refresh_block_w ? 1'b0 :
		(txn_start_w ?
		 ((wait_count_start_r == WAIT_NONE) && vip_ready_i) :
		 ((wait_count_q == WAIT_NONE) && vip_ready_i));
	// Bit 37 is reserved, keeping the savestate layout.
	assign savestate_state_rdata_o = {
		10'd0, refresh_wait_q, refresh_left_q, refresh_due_q, 9'd0,
		wait_use_vip_q, 1'b0, wait_count_q,
		txn_rw_q, txn_da_q, txn_st_q, txn_be_q, txn_addr_q, txn_active_q};

	always @(posedge clk_i or posedge reset_i) begin
		if (reset_i) begin
			refresh_due_q   <= 1'b0;
			refresh_left_q  <= 4'd0;
			refresh_wait_q  <= 1'b0;
		end else if (savestate_state_wren_i) begin
			refresh_due_q   <= savestate_state_wdata_i[48];
			refresh_left_q  <= savestate_state_wdata_i[52:49];
			refresh_wait_q  <= savestate_state_wdata_i[53];
		end else if (ce_i) begin
			if (refresh_tick_i) begin
				refresh_due_q <= 1'b1;
			end
			if (refresh_left_q != 4'd0) begin
				refresh_left_q <= refresh_left_q - 4'd1;
			end else if (refresh_start_w) begin
				refresh_left_q <= REFRESH_HOLD;
				refresh_due_q  <= 1'b0;
				refresh_wait_q <= 1'b0;
			end else if (refresh_due_q) begin
				refresh_wait_q <= 1'b1;
			end
		end
	end

	always @(posedge clk_i or posedge reset_i) begin
		if (reset_i) begin
			txn_active_q      <= 1'b0;
			txn_addr_q        <= 26'd0;
			txn_be_q          <= 4'd0;
			txn_st_q          <= 2'd0;
			txn_da_q          <= 1'b0;
			txn_rw_q          <= 1'b0;
			wait_count_q      <= 2'd0;
			wait_use_vip_q    <= 1'b0;
		end else if (savestate_state_wren_i) begin
			txn_active_q <= savestate_state_wdata_i[0];
			txn_addr_q <= savestate_state_wdata_i[26:1];
			txn_be_q <= savestate_state_wdata_i[30:27];
			txn_st_q <= savestate_state_wdata_i[32:31];
			txn_da_q <= savestate_state_wdata_i[33];
			txn_rw_q <= savestate_state_wdata_i[34];
			wait_count_q <= savestate_state_wdata_i[36:35];
			wait_use_vip_q <= savestate_state_wdata_i[38];
		end else if (ce_i) begin
			if (!mrq_i || refresh_block_w) begin
				txn_active_q     <= 1'b0;
				wait_count_q     <= 2'd0;
				wait_use_vip_q   <= 1'b0;
			end else if (txn_start_w) begin
				txn_active_q     <= 1'b1;
				txn_addr_q       <= a_i;
				txn_be_q         <= be_i;
				txn_st_q         <= st_i;
				txn_da_q         <= da_i;
				txn_rw_q         <= rw_i;
				// The previous CE started the access; the next CE accepts it.
				wait_count_q     <= (wait_count_start_r != WAIT_NONE) ?
					(wait_count_start_r - 2'd1) : WAIT_NONE;
				wait_use_vip_q   <= wait_use_vip_r;
			end else if (wait_count_q != 2'd0) begin
				wait_count_q <= wait_count_q - 2'd1;
			end
		end
	end

endmodule

// NVC timer, game pad, link port and WCR registers.
module vb_vue_io
(
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        ce_i,

	input  wire [15:0] pad_sample_i,
	input  wire        pad_serial_enable_i,
	input  wire        pad_serial_i,
	input  wire        link_comcnt_i,
	input  wire        link_clk_i,
	input  wire        link_rx_i,
	input  wire        link_sync_i,

	input  wire        cs_i,
	input  wire [23:1] a_i,
	input  wire [15:0] din_i,
	input  wire [3:0]  be_i,
	input  wire [1:0]  st_i,
	input  wire        da_i,
	input  wire        mrq_i,
	input  wire        rw_i,
	input  wire        ready_i,
	output wire [15:0] dout_o,

	output wire        pad_latch_o,
	output wire        pad_clock_o,
	output wire        link_comcnt_o,
	output wire        link_clk_o,
	output wire        link_tx_o,
	output wire        link_sync_o,
	output wire        link_tx_done_o,
	output wire [7:0]  link_tx_byte_o,

	output wire        refresh_tick_o,    // The prescaler's 320-clock tick, to the WRAM refresh.
	output wire        timer_tick_o,
	output wire        timer_zero_o,
	output wire        timer_irq_o,
	output wire        pad_irq_o,
	output wire        comm_irq_o,
	output wire        wait_exp1_o,
	output wire        wait_rom1_o,
	input  wire [4:0]  savestate_state_addr_i,
	input  wire [50:0] savestate_state_wdata_i,
	input  wire        savestate_state_wren_i,
	output reg  [63:0] savestate_state_rdata_o
);

	// One free-running prescaler from reset: a 40-clock stage, then /10 for the
	// timer's 20 us tick, /8 for the WRAM refresh and the pad half-period, and /5
	// for the link half-period. A read or transfer starts mid-period and runs on
	// the shared ticks. The reset phases put the first refresh due 47 clocks after
	// reset and the first 20 us tick 328 clocks after it.
	localparam [5:0]  PRE40_RESET = 6'd33;
	localparam [2:0]  PRE8_RESET  = 3'd6;
	localparam [3:0]  PRE10_RESET = 4'd1;
	reg [5:0]  pre40_q;
	reg [2:0]  pre8_q;
	reg [3:0]  pre10_q;
	reg [2:0]  pre5_q;
	reg        timer_base_q;     // The 20 us strobe, a clock after its prescaler tick.
	reg        timer_clk_sel_q;
	reg        timer_irq_en_q;
	reg        timer_enabled_q;
	reg        timer_zero_q;
	reg [15:0] timer_reload_q;
	reg [15:0] timer_counter_q;
	reg        timer_reload_pending_q;
	reg [2:0]  timer_subtick_q;
	reg        timer_irq_q;

	reg        pad_irq_en_n_q;
	reg        pad_para_si_q;
	reg        pad_soft_ck_q;
	reg        pad_abort_q;
	reg        pad_busy_q;
	reg        pad_irq_q;
	reg [15:0] pad_data_q;
	reg [15:0] pad_shift_q;
	reg [15:0] pad_capture_q;
	reg [4:0]  pad_shift_count_q;
	reg [5:0]  pad_hw_edge_count_q;
	reg        pad_hw_clock_q;
	reg        pad_hw_latch_q;

	reg        comm_irq_en_n_q;
	reg        comm_clk_sel_q;
	reg        comm_busy_q;
	reg [3:0]  comm_bit_count_q;
	reg        comm_clk_phase_q;
	reg        link_clk_prev_q;
	reg        comm_cc_int_inh_q;
	reg        comm_cc_int_lev_q;
	reg        comm_cc_sig_q;
	reg        comm_cc_smp_q;
	reg        comm_cc_wr_q;
	reg        comm_c_irq_q;
	reg        comm_cc_irq_q;
	reg [7:0]  comm_tx_q;
	reg [7:0]  comm_tx_latched_q;
	reg [7:0]  comm_tx_shift_q;
	reg [7:0]  comm_rx_q;
	reg [7:0]  comm_rx_shift_q;

	reg        wait_exp1_q;
	reg        wait_rom1_q;
	reg [7:0]  read8_bus_r;
	wire        bus_cycle_w = cs_i && mrq_i;
	// Commit a register write only when the CPU accepts it with READY.
	wire        bus_write_w = bus_cycle_w && da_i && !rw_i && ready_i;
	wire        byte_lo_w = (be_i == 4'b1110);
	wire        byte_hi_w = (be_i == 4'b1101);
	wire        half_w = (be_i == 4'b1100);
	// Register byte address selected by the byte enables. A halfword access
	// keeps the even address, so this also indexes halfword reads.
	wire [5:0]  bus_addr_lo_w = {a_i[5:1], byte_hi_w};
	wire [5:0]  half_addr_lo_w = {a_i[5:1], 1'b0};
	wire [7:0]  write_byte_w = byte_hi_w ? din_i[15:8] : din_i[7:0];
	wire        pre_tick_w  = pre40_q == 6'd39;
	wire        pre_320_w   = pre_tick_w && (pre8_q == 3'd7);
	wire        pre_400_w   = pre_tick_w && (pre10_q == 4'd9);
	wire        pre_200_w   = pre_tick_w && (pre5_q == 3'd4);
	assign      refresh_tick_o = pre_320_w;
	wire        timer_base_tick_w = timer_base_q;
	wire        timer_counter_tick_w =
		timer_base_tick_w && timer_enabled_q &&
		(timer_clk_sel_q || (timer_subtick_q == 3'd4));
	wire        timer_reload_write_w =
		bus_write_w &&
		(((byte_lo_w || byte_hi_w) &&
		  ((bus_addr_lo_w == 6'h18) || (bus_addr_lo_w == 6'h1c))) ||
		 (half_w &&
		  ((half_addr_lo_w == 6'h18) || (half_addr_lo_w == 6'h1c))));
	wire        timer_tick_zero_hit_w =
		timer_counter_tick_w && !timer_reload_pending_q &&
		!timer_reload_write_w && (timer_counter_q == 16'h0001);
	wire        timer_subtick_after_ce_nonzero_w = timer_base_tick_w ?
		(timer_subtick_q != 3'd4) : (timer_subtick_q != 3'd0);
	wire        timer_tcr_write_w =
		bus_write_w &&
		(((byte_lo_w || byte_hi_w) && (bus_addr_lo_w == 6'h20)) ||
		 (half_w && (half_addr_lo_w == 6'h20)));
	wire        timer_tcr_clk_sel_w = half_w ? din_i[4] : write_byte_w[4];
	// STS quirk: switching from 100 us to 20 us consumes one count when the
	// modulo-five phase is nonzero. The phase advances before a same-edge write.
	wire        timer_clk_sel_tick_w =
		timer_tcr_write_w && !timer_clk_sel_q && timer_tcr_clk_sel_w &&
		timer_subtick_after_ce_nonzero_w;
	wire        timer_clk_sel_zero_hit_w =
		timer_clk_sel_tick_w && !timer_reload_pending_q &&
		(timer_counter_q == 16'h0001);
	wire        pad_abort_write_set_w =
		bus_write_w &&
		((((byte_lo_w || byte_hi_w) && (bus_addr_lo_w == 6'h28)) && write_byte_w[0]) ||
		 ((half_w && (half_addr_lo_w == 6'h28)) && din_i[0]));
	wire        pad_abort_rise_w =
		pad_busy_q && pre_320_w &&
		!pad_hw_latch_q && !pad_hw_clock_q &&
		(pad_abort_q || pad_abort_write_set_w);

	// STS: IRQ is high when bits 15:4 contain a press and bits 3:1 are clear.
	wire        pad_irq_shift_match_w = (|pad_shift_q[15:4]) && (~|pad_shift_q[3:1]);
	// Virtual Boy pads use the NES/SNES active-low serial data convention.
	wire        pad_serial_bit_w = ~pad_serial_i;
	wire        comm_cc_rd_w = comm_cc_wr_q & link_comcnt_i;
	// A link bit shifts on this edge; the eighth one ends the byte.
	wire        comm_shift_w = comm_busy_q && (comm_clk_sel_q ?
		(!link_clk_prev_q && link_clk_i) :
		(pre_200_w && !comm_clk_phase_q));

	always @(*) begin
		read8_bus_r = 8'hff;
		case (bus_addr_lo_w)
			6'h00: read8_bus_r = {comm_irq_en_n_q, 2'b11, comm_clk_sel_q, 1'b1, 1'b1, comm_busy_q, 1'b1};
			// In single-player mode, read status follows the local output.
			6'h04: read8_bus_r = {comm_cc_int_inh_q, 2'b11, comm_cc_int_lev_q, comm_cc_sig_q, comm_cc_smp_q, comm_cc_wr_q, comm_cc_rd_w};
			6'h08: read8_bus_r = comm_tx_q;
			6'h0c: read8_bus_r = comm_rx_q;
			6'h10: read8_bus_r = pad_data_q[7:0];
			6'h14: read8_bus_r = pad_data_q[15:8];
			6'h18: read8_bus_r = timer_counter_q[7:0];
			6'h1c: read8_bus_r = timer_counter_q[15:8];
			6'h20: read8_bus_r = {3'b111, timer_clk_sel_q, timer_irq_en_q, 1'b1, timer_zero_q, timer_enabled_q};
			6'h24: read8_bus_r = {6'b111111, wait_exp1_q, wait_rom1_q};
			6'h28: read8_bus_r = {pad_irq_en_n_q, 1'b1, pad_para_si_q, pad_soft_ck_q, 1'b1, 1'b1, pad_busy_q, pad_abort_q};
			default: begin
			end
		endcase
	end

	task comm_shift_bit_task;
		input rx_bit;
		reg [7:0] next_rx;
		reg       next_cc_smp;
		begin
			next_rx = {comm_rx_shift_q[6:0], rx_bit};
			next_cc_smp = 1'b0;
			comm_tx_shift_q <= {comm_tx_shift_q[6:0], 1'b0};
			comm_rx_shift_q <= next_rx;
			comm_bit_count_q <= comm_bit_count_q - 4'd1;
			if (comm_bit_count_q == 4'd1) begin
				comm_busy_q      <= 1'b0;
				comm_clk_phase_q <= 1'b1;
				comm_rx_q        <= next_rx;
				// STS: CC-Smp is both CC-Sig values ANDed with CC-Rd.
				next_cc_smp      = comm_cc_sig_q & link_sync_i & comm_cc_rd_w;
				comm_cc_smp_q    <= next_cc_smp;
				if (!comm_irq_en_n_q) begin
					comm_c_irq_q <= 1'b1;
				end
				if (!comm_cc_int_inh_q && (next_cc_smp == comm_cc_int_lev_q)) begin
					comm_cc_irq_q <= 1'b1;
				end
			end
		end
	endtask

	task write8_task;
		input [5:0] addr;
		input [7:0]  value;
		reg   [15:0] next_timer_counter;
		reg          timer_write_zero_hit;
		reg          timer_zero_clear_hit;
		begin
			next_timer_counter  = 16'h0000;
			timer_write_zero_hit = 1'b0;
			timer_zero_clear_hit = 1'b0;
			case (addr)
				6'h00: begin
					comm_irq_en_n_q <= value[7];
					comm_clk_sel_q  <= value[4];
					if (value[7]) begin
						comm_c_irq_q <= 1'b0;
					end
					if (!comm_busy_q && value[2]) begin
						comm_busy_q         <= 1'b1;
						comm_bit_count_q    <= 4'd8;
						comm_clk_phase_q    <= 1'b1;
						comm_tx_latched_q   <= comm_tx_q;
						comm_tx_shift_q     <= comm_tx_q;
						comm_rx_shift_q     <= 8'h00;
					end
					if (comm_busy_q && comm_clk_sel_q && !value[4]) begin
						comm_clk_phase_q    <= 1'b1;
					end
					if (comm_busy_q && !comm_clk_sel_q && value[4]) begin
						comm_clk_phase_q <= 1'b1;
					end
				end
				6'h04: begin
					comm_cc_int_inh_q <= value[7];
					comm_cc_int_lev_q <= value[4];
					comm_cc_sig_q     <= value[3];
					comm_cc_wr_q      <= value[1];
					if (value[7]) begin
						comm_cc_irq_q <= 1'b0;
					end
				end
				6'h08: comm_tx_q <= value[7:0];
				6'h18: begin
					next_timer_counter = {timer_reload_q[15:8], value[7:0]};
					timer_write_zero_hit =
						(timer_counter_q != 16'h0000) &&
						(next_timer_counter == 16'h0000);
					timer_reload_q[7:0] <= value[7:0];
					timer_counter_q     <= next_timer_counter;
					// The 20 us divider keeps running across a preset. The
					// first selected count loads the combined preset without
					// decrementing it.
					timer_reload_pending_q <= 1'b1;
					if (timer_write_zero_hit) begin
						if (timer_enabled_q) begin
							timer_zero_q <= 1'b1;
						end
						if (timer_irq_en_q) begin
							timer_irq_q <= 1'b1;
						end
					end
				end
				6'h1c: begin
					next_timer_counter = {value[7:0], timer_reload_q[7:0]};
					timer_write_zero_hit =
						(timer_counter_q != 16'h0000) &&
						(next_timer_counter == 16'h0000);
					timer_reload_q[15:8] <= value[7:0];
					timer_counter_q      <= next_timer_counter;
					timer_reload_pending_q <= 1'b1;
					if (timer_write_zero_hit) begin
						if (timer_enabled_q) begin
							timer_zero_q <= 1'b1;
						end
						if (timer_irq_en_q) begin
							timer_irq_q <= 1'b1;
						end
					end
				end
				6'h20: begin
					timer_clk_sel_q <= value[4];
					timer_irq_en_q  <= value[3];
					timer_enabled_q <= value[0];
					if (timer_clk_sel_tick_w) begin
						if (timer_reload_pending_q) begin
							timer_counter_q <= timer_reload_q;
							timer_reload_pending_q <= 1'b0;
						end else if (timer_counter_q == 16'h0000) begin
							timer_counter_q <= timer_reload_q;
						end else begin
							timer_counter_q <= timer_counter_q - 16'd1;
							if (timer_counter_q == 16'h0001) begin
								if (timer_enabled_q) begin
									timer_zero_q <= 1'b1;
								end
								if (timer_irq_en_q) begin
									timer_irq_q <= 1'b1;
								end
							end
						end
					end
					timer_zero_clear_hit =
						value[2] &&
						(!timer_enabled_q || (value[0] && (timer_counter_q != 16'h0000))) &&
						!timer_tick_zero_hit_w && !timer_clk_sel_zero_hit_w;
					if (timer_zero_clear_hit) begin
						timer_zero_q <= 1'b0;
					end
					if (!value[3] || timer_zero_clear_hit) begin
						timer_irq_q <= 1'b0;
					end
				end
				6'h24: begin
					wait_exp1_q <= value[1];
					wait_rom1_q <= value[0];
				end
				6'h28: begin
					pad_irq_en_n_q <= value[7];
					if (value[5]) begin
						pad_para_si_q     <= 1'b1;
						pad_shift_q       <= pad_serial_enable_i ? 16'h0000 : pad_sample_i;
						pad_capture_q     <= 16'h0000;
						pad_shift_count_q <= 5'd0;
					end else begin
						pad_para_si_q <= 1'b0;
						if (pad_soft_ck_q && !value[4] && (pad_shift_count_q < 5'd16)) begin
							pad_capture_q <= {pad_capture_q[14:0],
								pad_serial_enable_i ? pad_serial_bit_w : pad_shift_q[15]};
							pad_shift_q   <= {pad_shift_q[14:0],
								pad_serial_enable_i ? pad_serial_bit_w : 1'b0};
							if (pad_shift_count_q == 5'd15) begin
								pad_data_q <= {pad_capture_q[14:0],
									pad_serial_enable_i ? pad_serial_bit_w : pad_shift_q[15]};
							end
							pad_shift_count_q <= pad_shift_count_q + 5'd1;
						end
					end
					pad_soft_ck_q <= value[4];
					pad_abort_q   <= value[0];
					if (value[2] && !value[0] && !pad_abort_q &&
						!pad_busy_q) begin
						pad_busy_q          <= 1'b1;
						pad_shift_q         <= pad_serial_enable_i ? 16'h0000 : pad_sample_i;
						pad_capture_q       <= 16'h0000;
						pad_shift_count_q   <= 5'd0;
						pad_hw_edge_count_q <= 6'd0;
						pad_hw_clock_q      <= 1'b1;
						pad_hw_latch_q      <= 1'b1;
					end
					if (value[7]) begin
						pad_irq_q <= 1'b0;
					end
				end
				default: begin
				end
			endcase
		end
	endtask

	// I/O registers are 8-bit at four-byte stride. Mirror the defined byte
	// above it. For halfword reads byte_hi_w is low, so read8_bus_r already
	// holds the even-address register byte.
	assign dout_o = half_w ?
		{read8_bus_r, read8_bus_r} :
		(bus_addr_lo_w[0] ? {read8_bus_r, 8'd0} : {8'd0, read8_bus_r});

	// Timing pulses for work consumed on this enabled edge.
	assign timer_tick_o = ce_i && !reset_i &&
		(timer_counter_tick_w || timer_clk_sel_tick_w);
	assign timer_zero_o = ce_i && !reset_i &&
		(timer_tick_zero_hit_w || timer_clk_sel_zero_hit_w);
	assign timer_irq_o = timer_irq_q;
	assign pad_irq_o   = pad_irq_q;
	assign comm_irq_o  = comm_c_irq_q | comm_cc_irq_q;
	assign wait_exp1_o = wait_exp1_q;
	assign wait_rom1_o = wait_rom1_q;
	assign pad_latch_o = pad_busy_q ? pad_hw_latch_q : pad_para_si_q;
	assign pad_clock_o = pad_busy_q ? pad_hw_clock_q : ~pad_soft_ck_q;
	assign link_comcnt_o = comm_cc_wr_q;
	assign link_clk_o    = (!comm_clk_sel_q && comm_busy_q) ? comm_clk_phase_q : 1'b1;
	assign link_tx_o     = comm_busy_q ? comm_tx_shift_q[7] : 1'b0;
	assign link_sync_o   = comm_cc_sig_q;
	assign link_tx_done_o = ce_i && !reset_i && comm_shift_w && (comm_bit_count_q == 4'd1);
	assign link_tx_byte_o = comm_tx_latched_q;

	always @* begin
		savestate_state_rdata_o = 64'd0;
		case (savestate_state_addr_i)
			5'd0: savestate_state_rdata_o = {
				14'd0, timer_reload_pending_q, timer_irq_q,
				timer_subtick_q, pre8_q, pre40_q,
				timer_counter_q, timer_reload_q, timer_zero_q,
				timer_enabled_q, timer_irq_en_q, timer_clk_sel_q};
			5'd1: savestate_state_rdata_o = {
				13'd0, pad_shift_q, pad_data_q, pad_irq_q, 13'd0, pad_busy_q,
				pad_abort_q, pad_soft_ck_q, pad_para_si_q, pad_irq_en_n_q};
			5'd2: savestate_state_rdata_o = {
				26'd0, pad_hw_latch_q, pad_hw_clock_q, pad_hw_edge_count_q,
				1'b0, timer_base_q, pre5_q, pre10_q, pad_shift_count_q, pad_capture_q};
			5'd3: savestate_state_rdata_o = {
				16'd0, comm_irq_en_n_q, comm_clk_sel_q, comm_busy_q,
				comm_bit_count_q, 8'd0, comm_clk_phase_q,
				link_clk_prev_q, comm_cc_int_inh_q, comm_cc_int_lev_q,
				comm_cc_sig_q, comm_cc_smp_q, comm_cc_wr_q,
				comm_c_irq_q, comm_cc_irq_q, comm_tx_q,
				comm_tx_latched_q, comm_tx_shift_q};
			5'd4: savestate_state_rdata_o = {
				48'd0, comm_rx_q, comm_rx_shift_q};
			5'd7: savestate_state_rdata_o = {62'd0, wait_exp1_q, wait_rom1_q};
			default: begin
			end
		endcase
	end

	always @(posedge clk_i or posedge reset_i) begin
		if (reset_i) begin
			timer_clk_sel_q   <= 1'b0;
			timer_irq_en_q    <= 1'b0;
			timer_enabled_q   <= 1'b0;
			timer_zero_q      <= 1'b0;
			timer_reload_q    <= 16'hffff;
			timer_counter_q   <= 16'hffff;
			timer_reload_pending_q <= 1'b0;
			pre40_q           <= PRE40_RESET;
			pre8_q            <= PRE8_RESET;
			pre10_q           <= PRE10_RESET;
			pre5_q            <= 3'd0;
			timer_base_q      <= 1'b0;
			timer_subtick_q   <= 3'd0;
			timer_irq_q       <= 1'b0;

			pad_irq_en_n_q    <= 1'b0;
			pad_para_si_q     <= 1'b0;
			pad_soft_ck_q     <= 1'b0;
			pad_abort_q       <= 1'b0;
			pad_busy_q        <= 1'b0;
			pad_irq_q         <= 1'b0;
			pad_data_q        <= 16'h0000;
			pad_shift_q       <= 16'h0000;
			pad_capture_q     <= 16'h0000;
			pad_shift_count_q <= 5'd0;
			pad_hw_edge_count_q <= 6'd0;
			pad_hw_clock_q    <= 1'b1;
			pad_hw_latch_q    <= 1'b0;

			comm_irq_en_n_q   <= 1'b0;
			comm_clk_sel_q    <= 1'b0;
			comm_busy_q       <= 1'b0;
			comm_bit_count_q  <= 4'd0;
			comm_clk_phase_q  <= 1'b1;
			link_clk_prev_q   <= 1'b1;
			comm_cc_int_inh_q <= 1'b1;
			comm_cc_int_lev_q <= 1'b1;
			comm_cc_sig_q     <= 1'b1;
			comm_cc_smp_q     <= 1'b1;
			comm_cc_wr_q      <= 1'b1;
			comm_c_irq_q      <= 1'b0;
			comm_cc_irq_q     <= 1'b0;
			comm_tx_q         <= 8'h00;
			comm_tx_latched_q <= 8'h00;
			comm_tx_shift_q   <= 8'h00;
			comm_rx_q         <= 8'h00;
			comm_rx_shift_q   <= 8'h00;

			wait_exp1_q       <= 1'b0;
			wait_rom1_q       <= 1'b0;
		end else if (savestate_state_wren_i) begin
			case (savestate_state_addr_i)
				5'd0: begin
					timer_clk_sel_q <= savestate_state_wdata_i[0];
					timer_irq_en_q <= savestate_state_wdata_i[1];
					timer_enabled_q <= savestate_state_wdata_i[2];
					timer_zero_q <= savestate_state_wdata_i[3];
					timer_reload_q <= savestate_state_wdata_i[19:4];
					timer_counter_q <= savestate_state_wdata_i[35:20];
					pre40_q <= savestate_state_wdata_i[41:36];
					pre8_q <= savestate_state_wdata_i[44:42];
					timer_subtick_q <= savestate_state_wdata_i[47:45];
					timer_irq_q <= savestate_state_wdata_i[48];
					timer_reload_pending_q <= savestate_state_wdata_i[49];
				end
				5'd1: begin
					pad_irq_en_n_q <= savestate_state_wdata_i[0];
					pad_para_si_q <= savestate_state_wdata_i[1];
					pad_soft_ck_q <= savestate_state_wdata_i[2];
					pad_abort_q <= savestate_state_wdata_i[3];
					pad_busy_q <= |savestate_state_wdata_i[17:4];
					pad_irq_q <= savestate_state_wdata_i[18];
					pad_data_q <= savestate_state_wdata_i[34:19];
					pad_shift_q <= savestate_state_wdata_i[50:35];
				end
				5'd2: begin
					pad_capture_q <= savestate_state_wdata_i[15:0];
					pad_shift_count_q <= savestate_state_wdata_i[20:16];
					pre10_q <= savestate_state_wdata_i[24:21];
					pre5_q <= savestate_state_wdata_i[27:25];
					timer_base_q <= savestate_state_wdata_i[28];
					pad_hw_edge_count_q <= savestate_state_wdata_i[35:30];
					pad_hw_clock_q <= savestate_state_wdata_i[36];
					pad_hw_latch_q <= savestate_state_wdata_i[37];
				end
				5'd3: begin
					comm_tx_shift_q <= savestate_state_wdata_i[7:0];
					comm_tx_latched_q <= savestate_state_wdata_i[15:8];
					comm_tx_q <= savestate_state_wdata_i[23:16];
					comm_cc_irq_q <= savestate_state_wdata_i[24];
					comm_c_irq_q <= savestate_state_wdata_i[25];
					comm_cc_wr_q <= savestate_state_wdata_i[26];
					comm_cc_smp_q <= savestate_state_wdata_i[27];
					comm_cc_sig_q <= savestate_state_wdata_i[28];
					comm_cc_int_lev_q <= savestate_state_wdata_i[29];
					comm_cc_int_inh_q <= savestate_state_wdata_i[30];
					link_clk_prev_q <= savestate_state_wdata_i[31];
					comm_clk_phase_q <= savestate_state_wdata_i[32];
					comm_bit_count_q <= savestate_state_wdata_i[44:41];
					comm_busy_q <= savestate_state_wdata_i[45];
					comm_clk_sel_q <= savestate_state_wdata_i[46];
					comm_irq_en_n_q <= savestate_state_wdata_i[47];
				end
				5'd4: begin
					comm_rx_shift_q <= savestate_state_wdata_i[7:0];
					comm_rx_q <= savestate_state_wdata_i[15:8];
				end
				5'd7: begin
					wait_rom1_q <= savestate_state_wdata_i[0];
					wait_exp1_q <= savestate_state_wdata_i[1];
				end
				default: begin
				end
			endcase
		end else if (ce_i) begin
			pre40_q      <= pre_tick_w ? 6'd0 : pre40_q + 6'd1;
			timer_base_q <= pre_400_w;
			if (pre_tick_w) begin
				pre8_q  <= pre8_q + 3'd1;
				pre10_q <= (pre10_q == 4'd9) ? 4'd0 : pre10_q + 4'd1;
				pre5_q  <= (pre5_q == 3'd4) ? 3'd0 : pre5_q + 3'd1;
			end

			if (timer_base_tick_w) begin
				// The 100 us stage counts only while 100 us is selected, so
				// switching back to 20 us counts a tick only if one passed.
				if (timer_clk_sel_q || (timer_subtick_q == 3'd4)) begin
					timer_subtick_q <= 3'd0;
				end else begin
					timer_subtick_q <= timer_subtick_q + 3'd1;
				end

				if (timer_counter_tick_w) begin
					if (timer_reload_pending_q) begin
						timer_counter_q <= timer_reload_q;
						timer_reload_pending_q <= 1'b0;
					end else if (timer_counter_q == 16'h0000) begin
						timer_counter_q <= timer_reload_q;
					end else begin
						timer_counter_q <= timer_counter_q - 16'd1;
						if (timer_counter_q == 16'h0001) begin
							timer_zero_q <= 1'b1;
							if (timer_irq_en_q) begin
								timer_irq_q <= 1'b1;
							end
						end
					end
				end
			end

			if (pad_busy_q) begin
				if (pad_abort_rise_w) begin
					// Suspend only on the rising serial clock, without committing data or IRQ.
					pad_busy_q          <= 1'b0;
					pad_hw_edge_count_q <= 6'd0;
					pad_hw_clock_q      <= 1'b1;
					pad_hw_latch_q      <= 1'b0;
				end else begin
					if (pre_320_w) begin
						if (pad_hw_latch_q) begin
							// Latch the parallel input before the sixteen serial clocks.
							if (pad_hw_edge_count_q == 6'd0) begin
								pad_hw_edge_count_q <= 6'd1;
							end else begin
								pad_hw_latch_q      <= 1'b0;
								pad_hw_edge_count_q <= 6'd0;
							end
						end else begin
							// Sample before the next rising edge advances the pad.
							if (pad_serial_enable_i && pad_hw_clock_q) begin
								pad_shift_q <= {pad_shift_q[14:0], pad_serial_bit_w};
							end
							pad_hw_clock_q <= ~pad_hw_clock_q;
							if (pad_hw_edge_count_q != 6'd31) begin
								pad_hw_edge_count_q <= pad_hw_edge_count_q + 6'd1;
							end else begin
								// The sixteenth bit is in: the read ends.
								pad_busy_q <= 1'b0;
								pad_data_q <= pad_shift_q;
								// Key IRQ needs a new press in 15:4 with bits 3:1 clear.
								if (!pad_irq_en_n_q && pad_irq_shift_match_w) begin
									pad_irq_q <= 1'b1;
								end
							end
						end
					end
				end
			end

			link_clk_prev_q <= link_clk_i;

			if (comm_busy_q) begin
				if (comm_clk_sel_q) begin
					if (!link_clk_prev_q && link_clk_i) begin
						comm_shift_bit_task(link_rx_i);
					end
				end else if (pre_200_w) begin
					comm_clk_phase_q    <= ~comm_clk_phase_q;
					if (!comm_clk_phase_q) begin
						comm_shift_bit_task(link_rx_i);
					end
				end
			end

			if (bus_write_w && (byte_lo_w || byte_hi_w)) begin
				write8_task(bus_addr_lo_w, write_byte_w);
			end else if (bus_write_w && half_w) begin
				write8_task(half_addr_lo_w + 6'd0, din_i[7:0]);
				write8_task(half_addr_lo_w + 6'd1, din_i[15:8]);
			end
		end
	end

endmodule

// Rumble accessory on the link port. It acts on each byte the link port
// sends and drives the two motor levels.
module vb_rumble_pak
(
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        ce_i,

	input  wire        enable_i,
	input  wire        byte_valid_i,
	input  wire [7:0]  byte_i,
	output wire [15:0] rumble_o,

	// VUE I/O savestate window rows; row 7 also holds WCR, read as zero here.
	input  wire [4:0]  savestate_state_addr_i,
	input  wire [63:0] savestate_state_wdata_i,
	input  wire        savestate_state_wren_i,
	output reg  [63:0] savestate_state_rdata_o
);

	localparam [1:0]  RUMBLE_FREQ_160HZ = 2'd0;
	localparam [1:0]  RUMBLE_FREQ_240HZ = 2'd1;
	localparam [1:0]  RUMBLE_FREQ_320HZ = 2'd2;
	localparam [7:0]  RUMBLE_CMD_STOP = 8'h00;
	localparam [7:0]  RUMBLE_CMD_PLAY = 8'h7c;
	localparam [7:0]  RUMBLE_CMD_CHAIN_EFFECT_0 = 8'h80;
	localparam [7:0]  RUMBLE_CMD_CHAIN_EFFECT_4 = 8'h84;
	localparam [7:0]  RUMBLE_CMD_FREQ_160HZ = 8'h90;
	localparam [7:0]  RUMBLE_CMD_FREQ_400HZ = 8'h93;
	localparam [7:0]  RUMBLE_CMD_WRITE_EFFECT_CHAIN = 8'hb0;
	localparam [7:0]  RUMBLE_CMD_WRITE_EFFECT_LOOPS_CHAIN = 8'hb1;
	localparam [7:0]  RUMBLE_EFFECT_CHAIN_END = 8'hff;
	localparam [2:0]  RUMBLE_WRITE_IDLE = 3'd0;
	localparam [2:0]  RUMBLE_WRITE_EFFECT_CHAIN_SEL = 3'd1;
	localparam [2:0]  RUMBLE_WRITE_EFFECT_BYTES = 3'd2;
	localparam [2:0]  RUMBLE_WRITE_LOOP_CHAIN_SEL = 3'd3;
	localparam [2:0]  RUMBLE_WRITE_LOOP_BYTES = 3'd4;
	localparam [24:0] RUMBLE_WRITE_TIMEOUT_CYCLES = 25'd20000000;
	localparam [27:0] RUMBLE_TICKS_35MS    = 28'd700000;
	localparam [27:0] RUMBLE_TICKS_50MS    = 28'd1000000;
	localparam [27:0] RUMBLE_TICKS_75MS    = 28'd1500000;
	localparam [27:0] RUMBLE_TICKS_100MS   = 28'd2000000;
	localparam [27:0] RUMBLE_TICKS_150MS   = 28'd3000000;
	localparam [27:0] RUMBLE_TICKS_250MS   = 28'd5000000;
	localparam [27:0] RUMBLE_TICKS_500MS   = 28'd10000000;
	localparam [27:0] RUMBLE_TICKS_750MS   = 28'd15000000;
	localparam [27:0] RUMBLE_TICKS_1000MS  = 28'd20000000;
	localparam [27:0] RUMBLE_TICKS_10000MS = 28'd200000000;

	reg [7:0]  rumble_chain_effect_q [0:39];
	reg [2:0]  rumble_chain_loop_q [0:39];
	reg [1:0]  rumble_freq_q;
	reg [7:0]  rumble_last_effect_q;
	reg [2:0]  rumble_last_chain_q;
	reg        rumble_last_is_chain_q;
	reg [2:0]  rumble_write_state_q;
	reg [2:0]  rumble_write_chain_q;
	reg [2:0]  rumble_write_slot_q;
	reg [24:0] rumble_write_timeout_q;
	reg        rumble_play_active_q;
	reg        rumble_chain_active_q;
	reg [2:0]  rumble_chain_play_q;
	reg [2:0]  rumble_chain_slot_play_q;
	reg [2:0]  rumble_chain_repeat_q;
	reg [7:0]  rumble_effect_play_q;
	reg [27:0] rumble_ticks_q;
	reg [7:0]  rumble_large_q;
	reg [7:0]  rumble_small_q;
	reg        rumble_init_busy_q;
	reg [5:0]  rumble_init_idx_q;
	integer    savestate_index_v;
	wire [5:0] savestate_rumble_index_w =
		{savestate_state_addr_i[2:0], 3'b000};

	// Effect amplitudes:
	// 100%=0xff, 80%=0xcc, 60%=0x99, 50%=0x80, 40%=0x66, 30%=0x4d, 20%=0x33, 10%=0x1a.
	// Effects 70-117 use a fixed 50% approximation of their ramps.
	function [7:0] rumble_effect_level_fn;
		input [7:0] effect;
		begin
			case (effect)
				8'd1, 8'd4, 8'd7, 8'd10, 8'd12, 8'd14, 8'd15, 8'd16,
				8'd17, 8'd21, 8'd24, 8'd27, 8'd31, 8'd34, 8'd37, 8'd41,
				8'd44, 8'd47, 8'd52, 8'd54, 8'd56, 8'd58, 8'd64, 8'd118:
					rumble_effect_level_fn = 8'hff;
				8'd18, 8'd22, 8'd25, 8'd28, 8'd32, 8'd35, 8'd38, 8'd42,
				8'd45, 8'd48, 8'd59, 8'd65:
					rumble_effect_level_fn = 8'hcc;
				8'd2, 8'd5, 8'd8, 8'd11, 8'd13, 8'd19, 8'd23, 8'd26,
				8'd29, 8'd33, 8'd36, 8'd39, 8'd43, 8'd46, 8'd49, 8'd53,
				8'd55, 8'd57, 8'd60, 8'd66:
					rumble_effect_level_fn = 8'h99;
				8'd50, 8'd61, 8'd67, 8'd120:
					rumble_effect_level_fn = 8'h66;
				8'd3, 8'd6, 8'd9, 8'd20, 8'd30, 8'd40, 8'd121:
					rumble_effect_level_fn = 8'h4d;
				8'd51, 8'd62, 8'd68, 8'd122:
					rumble_effect_level_fn = 8'h33;
				8'd63, 8'd69, 8'd123:
					rumble_effect_level_fn = 8'h1a;
				default:
					rumble_effect_level_fn = 8'h80;
			endcase
		end
	endfunction

	// Most ranges approximate each effect family. Effect 118 lasts ten seconds.
	function [27:0] rumble_effect_ticks_fn;
		input [7:0] effect;
		begin
			if ((effect >= 8'd1) && (effect <= 8'd13)) begin
				rumble_effect_ticks_fn = RUMBLE_TICKS_35MS;
			end else if (effect == 8'd14) begin
				rumble_effect_ticks_fn = RUMBLE_TICKS_500MS;
			end else if (effect == 8'd15) begin
				rumble_effect_ticks_fn = RUMBLE_TICKS_750MS;
			end else if (effect == 8'd16) begin
				rumble_effect_ticks_fn = RUMBLE_TICKS_1000MS;
			end else if ((effect >= 8'd17) && (effect <= 8'd46)) begin
				rumble_effect_ticks_fn = RUMBLE_TICKS_50MS;
			end else if ((effect >= 8'd47) && (effect <= 8'd57)) begin
				rumble_effect_ticks_fn = RUMBLE_TICKS_100MS;
			end else if ((effect >= 8'd58) && (effect <= 8'd69)) begin
				rumble_effect_ticks_fn = RUMBLE_TICKS_75MS;
			end else if ((effect >= 8'd70) && (effect <= 8'd117)) begin
				rumble_effect_ticks_fn = RUMBLE_TICKS_150MS;
			end else if (effect == 8'd118) begin
				rumble_effect_ticks_fn = RUMBLE_TICKS_10000MS;
			end else begin
				rumble_effect_ticks_fn = RUMBLE_TICKS_250MS;
			end
		end
	endfunction

	function [7:0] rumble_small_level_fn;
		input [7:0] large_level;
		input [1:0] freq_sel;
		begin
			case (freq_sel)
				RUMBLE_FREQ_160HZ: rumble_small_level_fn = {2'b00, large_level[7:2]};
				RUMBLE_FREQ_240HZ: rumble_small_level_fn = {1'b0, large_level[7:1]};
				RUMBLE_FREQ_320HZ: rumble_small_level_fn = large_level;
				// Saturate at 0xff so large values do not wrap.
				default:           rumble_small_level_fn = large_level[7] ? 8'hff : {large_level[6:0], 1'b0};
			endcase
		end
	endfunction

	task rumble_stop_task;
		begin
			rumble_play_active_q <= 1'b0;
			rumble_chain_active_q <= 1'b0;
			rumble_chain_repeat_q <= 3'd0;
			rumble_ticks_q <= 28'd0;
			rumble_large_q <= 8'd0;
			rumble_small_q <= 8'd0;
		end
	endtask

	task rumble_start_effect_task;
		input [7:0] effect;
		input       silent;
		reg [7:0]   large_level;
		begin
			large_level = rumble_effect_level_fn(effect);
			rumble_effect_play_q <= effect;
			rumble_ticks_q <= rumble_effect_ticks_fn(effect);
			rumble_play_active_q <= 1'b1;
			rumble_large_q <= silent ? 8'd0 : large_level;
			rumble_small_q <= silent ? 8'd0 : rumble_small_level_fn(large_level, rumble_freq_q);
		end
	endtask

	task rumble_begin_chain_slot_task;
		input [2:0] chain_num;
		input [2:0] slot_num;
		reg [5:0] idx;
		reg [7:0] effect;
		reg [2:0] loops;
		begin
			idx = {chain_num, slot_num};
			effect = rumble_chain_effect_q[idx];
			loops = rumble_chain_loop_q[idx];
			if ((effect >= 8'h01) && (effect <= 8'h7b)) begin
				rumble_chain_active_q <= 1'b1;
				rumble_chain_play_q <= chain_num;
				rumble_chain_slot_play_q <= slot_num;
				rumble_chain_repeat_q <= (loops > 3'd1) ? (loops - 3'd1) : 3'd0;
				rumble_start_effect_task(effect, (loops == 3'd0));
			end else begin
				rumble_stop_task();
			end
		end
	endtask

	task rumble_play_last_task;
		begin
			if (rumble_last_is_chain_q) begin
				rumble_begin_chain_slot_task(rumble_last_chain_q, 3'd0);
			end else if ((rumble_last_effect_q >= 8'h01) && (rumble_last_effect_q <= 8'h7b)) begin
				rumble_chain_active_q <= 1'b0;
				rumble_start_effect_task(rumble_last_effect_q, 1'b0);
			end else begin
				rumble_stop_task();
			end
		end
	endtask

	task rumble_clear_effect_chain_task;
		input [2:0] chain_num;
		integer j;
		begin
			for (j = 0; j < 8; j = j + 1) begin
				rumble_chain_effect_q[{chain_num, j[2:0]}] <= 8'h00;
			end
		end
	endtask

	task rumble_clear_loop_chain_task;
		input [2:0] chain_num;
		integer j;
		begin
			for (j = 0; j < 8; j = j + 1) begin
				rumble_chain_loop_q[{chain_num, j[2:0]}] <= 3'd1;
			end
		end
	endtask

	task rumble_receive_byte_task;
		input [7:0] value;
		reg [5:0]   chain_idx;
		begin
			chain_idx = 6'd0;
			case (rumble_write_state_q)
				RUMBLE_WRITE_EFFECT_CHAIN_SEL: begin
					if (value <= 8'h04) begin
						rumble_write_chain_q <= value[2:0];
						rumble_write_slot_q <= 3'd0;
						rumble_write_state_q <= RUMBLE_WRITE_EFFECT_BYTES;
						rumble_write_timeout_q <= RUMBLE_WRITE_TIMEOUT_CYCLES;
						rumble_clear_effect_chain_task(value[2:0]);
					end else begin
						rumble_write_state_q <= RUMBLE_WRITE_IDLE;
						rumble_write_timeout_q <= 25'd0;
					end
				end
				RUMBLE_WRITE_EFFECT_BYTES: begin
					if (value == RUMBLE_EFFECT_CHAIN_END) begin
						rumble_write_state_q <= RUMBLE_WRITE_IDLE;
						rumble_write_timeout_q <= 25'd0;
					end else begin
						rumble_write_timeout_q <= RUMBLE_WRITE_TIMEOUT_CYCLES;
						if ((value >= 8'h01) && (value <= 8'h7b)) begin
							chain_idx = {rumble_write_chain_q, rumble_write_slot_q};
							rumble_chain_effect_q[chain_idx] <= value;
							if (rumble_write_slot_q != 3'd7) begin
								rumble_write_slot_q <= rumble_write_slot_q + 3'd1;
							end
						end
					end
				end
				RUMBLE_WRITE_LOOP_CHAIN_SEL: begin
					if (value <= 8'h04) begin
						rumble_write_chain_q <= value[2:0];
						rumble_write_slot_q <= 3'd0;
						rumble_write_state_q <= RUMBLE_WRITE_LOOP_BYTES;
						rumble_write_timeout_q <= RUMBLE_WRITE_TIMEOUT_CYCLES;
						rumble_clear_loop_chain_task(value[2:0]);
					end else begin
						rumble_write_state_q <= RUMBLE_WRITE_IDLE;
						rumble_write_timeout_q <= 25'd0;
					end
				end
				RUMBLE_WRITE_LOOP_BYTES: begin
					if (value == RUMBLE_EFFECT_CHAIN_END) begin
						rumble_write_state_q <= RUMBLE_WRITE_IDLE;
						rumble_write_timeout_q <= 25'd0;
					end else begin
						rumble_write_timeout_q <= RUMBLE_WRITE_TIMEOUT_CYCLES;
						if (value <= 8'h04) begin
							chain_idx = {rumble_write_chain_q, rumble_write_slot_q};
							rumble_chain_loop_q[chain_idx] <= value[2:0];
							if (rumble_write_slot_q != 3'd7) begin
								rumble_write_slot_q <= rumble_write_slot_q + 3'd1;
							end
						end
					end
				end
				default: begin
					case (value)
						RUMBLE_CMD_STOP: rumble_stop_task();
						RUMBLE_CMD_PLAY: rumble_play_last_task();
						RUMBLE_CMD_WRITE_EFFECT_CHAIN: begin
							rumble_write_state_q <= RUMBLE_WRITE_EFFECT_CHAIN_SEL;
							rumble_write_timeout_q <= RUMBLE_WRITE_TIMEOUT_CYCLES;
						end
						RUMBLE_CMD_WRITE_EFFECT_LOOPS_CHAIN: begin
							rumble_write_state_q <= RUMBLE_WRITE_LOOP_CHAIN_SEL;
							rumble_write_timeout_q <= RUMBLE_WRITE_TIMEOUT_CYCLES;
						end
						default: begin
							if ((value >= 8'h01) && (value <= 8'h7b)) begin
								rumble_last_effect_q <= value;
								rumble_last_is_chain_q <= 1'b0;
								rumble_chain_active_q <= 1'b0;
								rumble_start_effect_task(value, 1'b0);
							end else if ((value >= RUMBLE_CMD_CHAIN_EFFECT_0) && (value <= RUMBLE_CMD_CHAIN_EFFECT_4)) begin
								rumble_last_chain_q <= value[2:0];
								rumble_last_is_chain_q <= 1'b1;
								rumble_begin_chain_slot_task(value[2:0], 3'd0);
							end else if ((value >= RUMBLE_CMD_FREQ_160HZ) && (value <= RUMBLE_CMD_FREQ_400HZ)) begin
								rumble_freq_q <= value[1:0];
							end
						end
					endcase
				end
			endcase
		end
	endtask

	assign rumble_o = enable_i ?
		{rumble_large_q, rumble_small_q} : 16'd0;

	always @* begin
		savestate_state_rdata_o = 64'd0;
		case (savestate_state_addr_i)
			5'd5: savestate_state_rdata_o = {
				16'd0, rumble_write_timeout_q, rumble_write_slot_q,
				rumble_write_chain_q, rumble_write_state_q,
				rumble_last_is_chain_q, rumble_last_chain_q,
				rumble_last_effect_q, rumble_freq_q};
			5'd6: savestate_state_rdata_o = {
				17'd0, rumble_play_active_q, rumble_chain_active_q,
				rumble_chain_play_q, rumble_chain_slot_play_q,
				rumble_chain_repeat_q, rumble_effect_play_q, rumble_ticks_q};
			5'd7: savestate_state_rdata_o = {
				39'd0, rumble_large_q, rumble_small_q, rumble_init_busy_q,
				rumble_init_idx_q, 2'b00};
			5'd8, 5'd9, 5'd10, 5'd11, 5'd12:
				savestate_state_rdata_o = {
					rumble_chain_effect_q[savestate_rumble_index_w + 6'd7],
					rumble_chain_effect_q[savestate_rumble_index_w + 6'd6],
					rumble_chain_effect_q[savestate_rumble_index_w + 6'd5],
					rumble_chain_effect_q[savestate_rumble_index_w + 6'd4],
					rumble_chain_effect_q[savestate_rumble_index_w + 6'd3],
					rumble_chain_effect_q[savestate_rumble_index_w + 6'd2],
					rumble_chain_effect_q[savestate_rumble_index_w + 6'd1],
					rumble_chain_effect_q[savestate_rumble_index_w]};
			5'd13, 5'd14, 5'd15, 5'd16, 5'd17:
				savestate_state_rdata_o = {
					40'd0,
					rumble_chain_loop_q[savestate_rumble_index_w - 6'd40 + 6'd7],
					rumble_chain_loop_q[savestate_rumble_index_w - 6'd40 + 6'd6],
					rumble_chain_loop_q[savestate_rumble_index_w - 6'd40 + 6'd5],
					rumble_chain_loop_q[savestate_rumble_index_w - 6'd40 + 6'd4],
					rumble_chain_loop_q[savestate_rumble_index_w - 6'd40 + 6'd3],
					rumble_chain_loop_q[savestate_rumble_index_w - 6'd40 + 6'd2],
					rumble_chain_loop_q[savestate_rumble_index_w - 6'd40 + 6'd1],
					rumble_chain_loop_q[savestate_rumble_index_w - 6'd40]};
			default: begin
			end
		endcase
	end

	always @(posedge clk_i or posedge reset_i) begin
		if (reset_i) begin
			rumble_freq_q <= RUMBLE_FREQ_160HZ;
			rumble_last_effect_q <= 8'h00;
			rumble_last_chain_q <= 3'd0;
			rumble_last_is_chain_q <= 1'b0;
			rumble_write_state_q <= RUMBLE_WRITE_IDLE;
			rumble_write_chain_q <= 3'd0;
			rumble_write_slot_q <= 3'd0;
			rumble_write_timeout_q <= 25'd0;
			rumble_play_active_q <= 1'b0;
			rumble_chain_active_q <= 1'b0;
			rumble_chain_play_q <= 3'd0;
			rumble_chain_slot_play_q <= 3'd0;
			rumble_chain_repeat_q <= 3'd0;
			rumble_effect_play_q <= 8'h00;
			rumble_ticks_q <= 28'd0;
			rumble_large_q <= 8'd0;
			rumble_small_q <= 8'd0;
			rumble_init_busy_q <= 1'b1;
			rumble_init_idx_q <= 6'd0;
		end else if (savestate_state_wren_i) begin
			case (savestate_state_addr_i)
				5'd5: begin
					rumble_freq_q <= savestate_state_wdata_i[1:0];
					rumble_last_effect_q <= savestate_state_wdata_i[9:2];
					rumble_last_chain_q <= savestate_state_wdata_i[12:10];
					rumble_last_is_chain_q <= savestate_state_wdata_i[13];
					rumble_write_state_q <= savestate_state_wdata_i[16:14];
					rumble_write_chain_q <= savestate_state_wdata_i[19:17];
					rumble_write_slot_q <= savestate_state_wdata_i[22:20];
					rumble_write_timeout_q <= savestate_state_wdata_i[47:23];
				end
				5'd6: begin
					rumble_ticks_q <= savestate_state_wdata_i[27:0];
					rumble_effect_play_q <= savestate_state_wdata_i[35:28];
					rumble_chain_repeat_q <= savestate_state_wdata_i[38:36];
					rumble_chain_slot_play_q <= savestate_state_wdata_i[41:39];
					rumble_chain_play_q <= savestate_state_wdata_i[44:42];
					rumble_chain_active_q <= savestate_state_wdata_i[45];
					rumble_play_active_q <= savestate_state_wdata_i[46];
				end
				5'd7: begin
					rumble_init_idx_q <= savestate_state_wdata_i[7:2];
					rumble_init_busy_q <= savestate_state_wdata_i[8];
					rumble_small_q <= savestate_state_wdata_i[16:9];
					rumble_large_q <= savestate_state_wdata_i[24:17];
				end
				5'd8, 5'd9, 5'd10, 5'd11, 5'd12: begin
					for (savestate_index_v = 0; savestate_index_v < 8;
						savestate_index_v = savestate_index_v + 1) begin
						rumble_chain_effect_q[
							{26'd0, savestate_rumble_index_w} + savestate_index_v] <=
							savestate_state_wdata_i[(savestate_index_v * 8) +: 8];
					end
				end
				5'd13, 5'd14, 5'd15, 5'd16, 5'd17: begin
					for (savestate_index_v = 0; savestate_index_v < 8;
						savestate_index_v = savestate_index_v + 1) begin
						rumble_chain_loop_q[
							{26'd0, savestate_rumble_index_w} - 32'd40 +
							savestate_index_v] <=
							savestate_state_wdata_i[(savestate_index_v * 3) +: 3];
					end
				end
				default: begin
				end
			endcase
		end else if (ce_i) begin
			if (rumble_init_busy_q) begin
				rumble_chain_effect_q[rumble_init_idx_q] <= 8'h00;
				rumble_chain_loop_q[rumble_init_idx_q] <= 3'd1;
				if (rumble_init_idx_q == 6'd39) begin
					rumble_init_busy_q <= 1'b0;
				end else begin
					rumble_init_idx_q <= rumble_init_idx_q + 6'd1;
				end
			end else if (!enable_i) begin
				// None keeps serial timing active but disconnects the rumble device.
				rumble_stop_task();
				rumble_write_state_q <= RUMBLE_WRITE_IDLE;
				rumble_write_timeout_q <= 25'd0;
			end else begin
				if (rumble_write_state_q != RUMBLE_WRITE_IDLE) begin
					if (rumble_write_timeout_q != 25'd0) begin
						rumble_write_timeout_q <= rumble_write_timeout_q - 25'd1;
						if (rumble_write_timeout_q == 25'd1) begin
							rumble_write_state_q <= RUMBLE_WRITE_IDLE;
						end
					end
				end

				if (rumble_play_active_q) begin
					if (rumble_ticks_q != 28'd0) begin
						rumble_ticks_q <= rumble_ticks_q - 28'd1;
						if (rumble_ticks_q == 28'd1) begin
							if (rumble_chain_active_q) begin
								if (rumble_chain_repeat_q != 3'd0) begin
									rumble_chain_repeat_q <= rumble_chain_repeat_q - 3'd1;
									rumble_start_effect_task(rumble_effect_play_q, 1'b0);
								end else if (rumble_chain_slot_play_q == 3'd7) begin
									rumble_stop_task();
								end else begin
									rumble_begin_chain_slot_task(rumble_chain_play_q, rumble_chain_slot_play_q + 3'd1);
								end
							end else begin
								rumble_play_active_q <= 1'b0;
								rumble_large_q <= 8'd0;
								rumble_small_q <= 8'd0;
							end
						end
					end
				end
			end

			if (byte_valid_i && enable_i && !rumble_init_busy_q) begin
				rumble_receive_byte_task(byte_i);
			end
		end
	end

endmodule
