// Copyright (c) 2026 Jamie Blanks
//
// Nintendo Virtual Boy VSU sound chip.
//
// Six channels: five 32-sample PCM wave channels (channel 5 adds the
// sweep/modulation unit) and one LFSR noise channel, each with duration,
// envelope, and stereo volume controls. A single serial mixer scans all six
// channels once per 480-cycle sample window (41.7 kHz at 20 MHz).
//
// The chip is kept as one module to mirror the real single-chip VSU: the
// channel sequencers, sweep unit, and CPU register writes all update the
// same registers inside one clocked process so hardware write priority
// (CPU writes win over same-edge channel events) stays explicit. File
// layout: declarations and bus decode, savestate access, helper
// functions/tasks, RAMs, rate dividers, the channel/sweep/register engine,
// and finally the mixer.

module VSU
#(
	parameter [2:0]  PCM_BASE_TICK_CYCLES       = 3'd4,
	parameter [5:0]  NOISE_BASE_TICK_CYCLES     = 6'd40,
	parameter [16:0] DURATION_TICK_CYCLES       = 17'd76800,
	parameter [18:0] ENVELOPE_TICK_CYCLES       = 19'd307200,
	parameter [20:0] SWEEP_SMALL_TICK_CYCLES    = 21'd19200,
	parameter [20:0] SWEEP_LARGE_TICK_CYCLES    = 21'd153600,
	// Divide 20 MHz by 480 for the documented 41.7 kHz output rate.
	parameter [8:0]  SAMPLE_TICK_CYCLES          = 9'd480
)
(
	input  wire              clk_i,
	input  wire              reset_i,
	input  wire              ce_i,

	input  wire              cs_i,
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [23:1]       a_i,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire [15:0]       din_i,
	input  wire [3:0]        be_i,
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [1:0]        st_i,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire              da_i,
	input  wire              mrq_i,
	input  wire              rw_i,
	output wire [15:0]       dout_o,
	output wire              ready_o,

	output reg  [15:0]       audio_l_o,
	output reg  [15:0]       audio_r_o,
	output wire              audio_sample_valid_o,

	input  wire [4:0]        savestate_state_addr_i,
	input  wire [63:0]       savestate_state_wdata_i,
	input  wire              savestate_state_wren_i,
	output reg  [63:0]       savestate_state_rdata_o,
	input  wire              savestate_mem_active_i,
	input  wire [8:0]        savestate_mem_addr_i,
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire              savestate_mem_rden_i,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire              savestate_mem_wren_i,
	input  wire [7:0]        savestate_mem_wdata_i,
	output reg  [7:0]        savestate_mem_rdata_o
);

	localparam integer CHANNEL_COUNT = 6;
	localparam integer PCM_COUNT     = 5;
	localparam integer MOD_SAMPLES   = 32;

	localparam [2:0] CHANNEL_LAST = 3'd5;
	localparam [2:0] PCM_COUNT_U3 = 3'd5;
	localparam [2:0] WAVE_COUNT_U3 = 3'd5;

	localparam [10:0] ADDR_WAVE_END  = 11'h27f;
	localparam [10:0] ADDR_MOD_BEGIN = 11'h280;
	localparam [10:0] ADDR_MOD_END   = 11'h2ff;
	localparam [10:0] ADDR_REG_BEGIN = 11'h400;
	localparam [10:0] ADDR_REG_END   = 11'h57f;
	localparam [10:0] ADDR_SSTOP     = 11'h580;

	localparam [5:0] REG_INT = 6'h00;
	localparam [5:0] REG_LRV = 6'h04;
	localparam [5:0] REG_FQL = 6'h08;
	localparam [5:0] REG_FQH = 6'h0c;
	localparam [5:0] REG_EV0 = 6'h10;
	localparam [5:0] REG_EV1 = 6'h14;
	localparam [5:0] REG_RAM = 6'h18;
	localparam [5:0] REG_SWP = 6'h1c;

	localparam [1:0] MOD_LOCK_NONE = 2'd0;
	localparam [1:0] MOD_LOCK_LOW  = 2'd1;
	localparam [1:0] MOD_LOCK_HIGH = 2'd2;

	localparam [2:0] MIX_IDLE  = 3'd0;
	localparam [2:0] MIX_READ  = 3'd1;
	localparam [2:0] MIX_GAIN  = 3'd2;
	localparam [2:0] MIX_SCALE = 3'd3;
	localparam [2:0] MIX_ACCUM = 3'd4;

	// ---------------------------------------------------------------------
	// Per-channel and channel 5 sweep/modulation state
	// ---------------------------------------------------------------------

	reg        active_q         [0:CHANNEL_COUNT - 1];
	reg        auto_q           [0:CHANNEL_COUNT - 1];
	reg [4:0]  duration_set_q   [0:CHANNEL_COUNT - 1];
	reg [4:0]  duration_count_q [0:CHANNEL_COUNT - 1];
	reg [16:0] duration_phase_q [0:CHANNEL_COUNT - 1];

	reg [3:0] vol_l_q [0:CHANNEL_COUNT - 1];
	reg [3:0] vol_r_q [0:CHANNEL_COUNT - 1];

	reg [10:0] freq_reg_q   [0:CHANNEL_COUNT - 1];
	reg [10:0] freq_cur_q   [0:CHANNEL_COUNT - 1];
	// Base clocks left after the next tick. Loading ~FQ preserves the active
	// period when software changes FQ.
	reg [10:0] freq_count_q [0:CHANNEL_COUNT - 1];

	reg [3:0]  env_reload_q     [0:CHANNEL_COUNT - 1];
	reg        env_dir_up_q     [0:CHANNEL_COUNT - 1];
	reg        env_repeat_q     [0:CHANNEL_COUNT - 1];
	reg        env_enable_q     [0:CHANNEL_COUNT - 1];
	reg [2:0]  env_interval_q   [0:CHANNEL_COUNT - 1];
	reg [2:0]  env_count_q      [0:CHANNEL_COUNT - 1];
	reg [3:0]  env_level_q      [0:CHANNEL_COUNT - 1];
	reg        env_terminal_q   [0:CHANNEL_COUNT - 1];
	reg [18:0] envelope_phase_q [0:CHANNEL_COUNT - 1];

	reg [2:0] wave_sel_q [0:PCM_COUNT - 1];
	reg [4:0] phase_q    [0:PCM_COUNT - 1];

	reg [14:0] noise_shift_q;
	reg [2:0]  noise_tap_q;
	reg [5:0]  noise_sample_q;

	reg        swp_enable_q;
	reg        swp_repeat_q;
	reg        swp_mod_q;
	reg        swp_slow_q;
	reg [2:0]  swp_interval_q;
	reg        swp_dir_up_q;
	reg [2:0]  swp_shift_q;
	reg [20:0] swp_elapsed_q;
	reg [20:0] swp_frame_cycles_q;
	reg [20:0] swp_next_frame_cycles_q;
	reg [10:0] swp_candidate_freq_q;
	reg        swp_candidate_valid_q;
	reg [4:0]  swp_mod_index_q;
	// 0 = idle, 1 = one full 32-step pass consumed, 2 = terminal latch.
	reg [1:0]  swp_mod_mask_q;

	// Register modulation selection before signed arithmetic.
	reg [4:0]        swp_mod_read_addr_q;
	reg              swp_mod_read_pending_q;
	reg              swp_mod_calc_pending_q;
	reg [10:0]       swp_mod_base_freq_q;
	reg [1:0]        swp_mod_lock_mode_q;
	reg signed [7:0] mod_ram_read_q;
	reg [4:0]        mod_ram_read_tag_q;
	reg              mod_ram_read_valid_q;
	reg signed [7:0] swp_mod_sample_q;
	reg              swp_mod_sample_valid_q;

	reg [2:0]  pcm_base_count_q;
	reg [5:0]  noise_base_count_q;
	reg [8:0]  sample_count_q;

	// The 32-byte modulation store is small enough for MLAB/distributed RAM.
	(* ramstyle = "MLAB, no_rw_check" *)
	reg signed [7:0] mod_ram_shadow_q [0:MOD_SAMPLES - 1];

	reg [2:0]  mix_state_q;
	reg [2:0]  mix_index_q;
	reg [13:0] mix_acc_l_q;
	reg [13:0] mix_acc_r_q;
	reg [5:0]  mix_sample_q;
	reg [4:0]  mix_gain_l_q;
	reg [4:0]  mix_gain_r_q;
	reg [10:0] mix_scaled_l_q;
	reg [10:0] mix_scaled_r_q;
	reg        mix_force_silence_q;
	reg        mix_active_q   [0:CHANNEL_COUNT - 1];
	reg [3:0]  mix_vol_l_q    [0:CHANNEL_COUNT - 1];
	reg [3:0]  mix_vol_r_q    [0:CHANNEL_COUNT - 1];
	reg [3:0]  mix_env_q      [0:CHANNEL_COUNT - 1];
	reg [7:0]  mix_wave_addr_q[0:CHANNEL_COUNT - 1];
	reg [5:0]  mix_noise_q;

	integer chan_idx;
	integer mix_idx;

	// ---------------------------------------------------------------------
	// Bus write decode
	// ---------------------------------------------------------------------

	wire bus_cycle_w = cs_i && mrq_i;
	wire bus_write_w = bus_cycle_w && da_i && !rw_i;
	wire byte_lo_w   = be_i == 4'b1110;
	wire byte_hi_w   = be_i == 4'b1101;
	// ST.W uses two bus beats. Only the first beat writes waveform RAM.
	wire word_low_w    = (be_i == 4'b1100) && !a_i[1];
	wire write_valid_w = byte_lo_w || byte_hi_w || word_low_w;

	// The VSU repeats every 0x800 bytes and ignores address bit 1.
	wire [10:0] addr_w = {a_i[10:2], 1'b0, byte_hi_w};
	wire [7:0] write_byte_w = byte_hi_w ? din_i[15:8] : din_i[7:0];
	wire bus_write_accept_w = ce_i && !reset_i && bus_write_w && write_valid_w;
	wire reg_write_w = bus_write_accept_w &&
		(addr_w >= ADDR_REG_BEGIN) && (addr_w <= ADDR_REG_END);
	wire sstop_write_w = bus_write_accept_w && (addr_w == ADDR_SSTOP);
	wire [2:0] write_ch_w = addr_w[8:6];
	wire [5:0] write_reg_w = addr_w[5:0];

	// Channel 5 (index 4) register strobes; its sweep unit reacts to these
	// on the same edge as the write itself.
	wire ch5_int_write_w = reg_write_w &&
		(write_ch_w == 3'd4) && (write_reg_w == REG_INT);
	wire ch5_fql_write_w = reg_write_w &&
		(write_ch_w == 3'd4) && (write_reg_w == REG_FQL);
	wire ch5_fqh_write_w = reg_write_w &&
		(write_ch_w == 3'd4) && (write_reg_w == REG_FQH);
	wire ch5_swp_write_w = reg_write_w &&
		(write_ch_w == 3'd4) && (write_reg_w == REG_SWP);

	wire any_active_w = active_q[0] | active_q[1] | active_q[2] |
		active_q[3] | active_q[4] | active_q[5];

	wire wave_ram_wren_w = bus_write_accept_w &&
		(addr_w <= ADDR_WAVE_END) && (addr_w[1:0] == 2'b00) && !any_active_w;
	wire mod_ram_wren_w = bus_write_accept_w &&
		(addr_w >= ADDR_MOD_BEGIN) && (addr_w <= ADDR_MOD_END) &&
		(addr_w[1:0] == 2'b00) && !active_q[4];

	// ---------------------------------------------------------------------
	// Savestate address mapping and wave RAM port sharing
	// ---------------------------------------------------------------------

	wire savestate_wave_active_w = savestate_mem_active_i &&
		(savestate_mem_addr_i < 9'd256);
	wire savestate_mod_active_w = savestate_mem_active_i &&
		(savestate_mem_addr_i >= 9'd256);
	// Local words 19..24 map to mixer slots 0..5.
	wire [2:0] savestate_mix_index_w =
		savestate_state_addr_i[2:0] - 3'd3;
	// Local words 6..11 map to envelope/volume slots for channels 0..5.
	wire [2:0] savestate_env_index_w =
		savestate_state_addr_i[2:0] - 3'd6;
	wire [7:0] wave_ram_write_addr_w = savestate_wave_active_w ?
		savestate_mem_addr_i[7:0] : addr_w[9:2];
	wire [7:0] wave_ram_read_addr_w = mix_wave_addr_q[mix_index_q];
	/* verilator lint_off UNUSEDSIGNAL */
	wire [5:0] wave_ram_write_q_w;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [5:0] wave_ram_q_w;
	wire wave_ram_port_a_wren_w = savestate_wave_active_w ?
		savestate_mem_wren_i : wave_ram_wren_w;
	wire [5:0] wave_ram_port_a_wdata_w = savestate_wave_active_w ?
		savestate_mem_wdata_i[5:0] : write_byte_w[5:0];

	// ---------------------------------------------------------------------
	// Shared base-rate ticks
	// ---------------------------------------------------------------------

	wire pcm_tick_w = ce_i &&
		(pcm_base_count_q == (PCM_BASE_TICK_CYCLES - 3'd1));
	wire noise_tick_w = ce_i &&
		(noise_base_count_q == (NOISE_BASE_TICK_CYCLES - 6'd1));
	wire sample_tick_w = ce_i &&
		(sample_count_q == (SAMPLE_TICK_CYCLES - 9'd1));

	// ---------------------------------------------------------------------
	// Channel 5 sweep/modulation combinational view
	//
	// A CPU write can land on the same edge as a sweep frame tick. The
	// hardware behaves as if the frame is processed first and the write
	// second, so alongside the live state there is a "post-frame" view
	// (swp_post_frame_*) describing the position counter, pass mask, and
	// frequency as they will stand after this edge's frame advance. The
	// write-reaction logic below uses that view.
	// ---------------------------------------------------------------------

	// Frame processing is enabled and the current pass has not gone terminal.
	wire swp_processing_w = swp_enable_q && (swp_interval_q != 3'd0) &&
		((swp_mod_mask_q != 2'd2) || (swp_mod_q && swp_repeat_q));
	wire ch5_frame_due_w = active_q[4] && !ch5_int_write_w &&
		(swp_elapsed_q >= (swp_frame_cycles_q - 21'd1));
	// The 32-entry position counter is shared by both functions and advances
	// on every processed frame (red-viper, shrooms).
	wire swp_frame_advance_w = ch5_frame_due_w &&
		swp_processing_w && swp_candidate_valid_q;
	wire [4:0] swp_post_frame_mod_index_w = !swp_frame_advance_w ?
		swp_mod_index_q :
		((swp_mod_index_q == 5'd31) ?
			(swp_repeat_q ? 5'd0 : 5'd31) : (swp_mod_index_q + 5'd1));
	// A modulation-mode frame that consumes entry 31 goes terminal at once;
	// a sweep-mode frame only records the consumed pass (shrooms modmask).
	wire [1:0] swp_post_frame_mask_w =
		(swp_frame_advance_w && (swp_mod_index_q == 5'd31)) ?
		(swp_mod_q ? 2'd2 : 2'd1) : swp_mod_mask_q;
	wire swp_post_frame_mod_open_w = swp_post_frame_mask_w == 2'd0;
	wire swp_post_frame_processing_w = swp_enable_q &&
		(swp_interval_q != 3'd0) &&
		(swp_post_frame_mod_open_w || (swp_mod_q && swp_repeat_q));
	wire [10:0] ch5_post_frame_freq_w = swp_frame_advance_w ?
		swp_candidate_freq_q : freq_cur_q[4];

	// Operand selection for the shared sweep datapath: a same-edge S5FQL,
	// S5FQH, or S5SWP write feeds the just-written value in place of the
	// held frequency, direction, or shift amount.
	wire [10:0] ch5_fql_freq_w = {ch5_post_frame_freq_w[10:8], write_byte_w};
	wire [10:0] ch5_fqh_freq_w = {write_byte_w[2:0], ch5_post_frame_freq_w[7:0]};
	wire [10:0] ch5_fql_written_w = {freq_reg_q[4][10:8], write_byte_w};
	wire [10:0] ch5_fqh_written_w = {write_byte_w[2:0], freq_reg_q[4][7:0]};
	wire [10:0] swp_candidate_source_w = ch5_fql_write_w ?
		ch5_fql_freq_w : (ch5_fqh_write_w ? ch5_fqh_freq_w : ch5_post_frame_freq_w);
	wire swp_candidate_dir_w = ch5_swp_write_w ? write_byte_w[3] : swp_dir_up_q;
	wire [2:0] swp_candidate_shift_w = ch5_swp_write_w ?
		write_byte_w[2:0] : swp_shift_q;

	// Channel 5 staging writes recompute the modulated candidate while the
	// modulation function is (or becomes) selected and processing.
	wire ch5_mod_recalc_common_w = reg_write_w && (write_ch_w == 3'd4) &&
		((write_reg_w == REG_FQL) || (write_reg_w == REG_FQH) ||
		 (write_reg_w == REG_EV0));
	wire ch5_mod_recalc_ev1_w = reg_write_w && (write_ch_w == 3'd4) &&
		(write_reg_w == REG_EV1) && write_byte_w[4] && write_byte_w[6] &&
		(swp_interval_q != 3'd0) &&
		(swp_post_frame_mod_open_w || write_byte_w[5]);
	wire ch5_mod_recalc_swp_w = ch5_swp_write_w && swp_mod_q &&
		swp_enable_q && (write_byte_w[6:4] != 3'd0) &&
		(swp_post_frame_mod_open_w || swp_repeat_q);
	// Any channel 5 staging write promotes a consumed pass to the terminal
	// mask while the modulation function is selected after the write
	// (shrooms vsuNextFreqMod); S5INT instead clears the mask outright.
	wire ch5_staging_write_w = reg_write_w && (write_ch_w == 3'd4) &&
		((write_reg_w == REG_EV0) || (write_reg_w == REG_EV1) ||
		 (write_reg_w == REG_FQL) || (write_reg_w == REG_FQH) ||
		 (write_reg_w == REG_SWP));
	wire ch5_func_after_write_w = (write_reg_w == REG_EV1) ?
		write_byte_w[4] : swp_mod_q;
	wire ch5_mask_promote_w = ch5_staging_write_w && ch5_func_after_write_w &&
		(swp_post_frame_mask_w == 2'd1);
	// The frequency-byte lock persists until the next even-byte VSU write of
	// any address arms or clears it (red-viper sound_write, shrooms vsuWrite).
	wire [1:0] mod_lock_next_w = ch5_fql_write_w ? MOD_LOCK_LOW :
		(ch5_fqh_write_w ? MOD_LOCK_HIGH :
		((bus_write_accept_w && !addr_w[0]) ?
			MOD_LOCK_NONE : swp_mod_lock_mode_q));
	wire ch5_mod_recalc_write_w =
		(ch5_mod_recalc_common_w && swp_mod_q && swp_post_frame_processing_w) ||
		ch5_mod_recalc_ev1_w || ch5_mod_recalc_swp_w;
	wire [10:0] ch5_mod_recalc_base_w = ch5_fql_write_w ?
		ch5_fql_written_w : (ch5_fqh_write_w ? ch5_fqh_written_w : freq_reg_q[4]);
	wire [1:0] ch5_mod_recalc_lock_w = ch5_fql_write_w ?
		MOD_LOCK_LOW : (ch5_fqh_write_w ? MOD_LOCK_HIGH : MOD_LOCK_NONE);
	wire ch5_mod_sample_cached_w =
		swp_mod_sample_valid_q && !swp_frame_advance_w;
	wire ch5_mod_sample_prefetched_w = mod_ram_read_valid_q &&
		(mod_ram_read_tag_q == swp_post_frame_mod_index_w);
	wire ch5_mod_recalc_sample_ready_w =
		ch5_mod_sample_cached_w || ch5_mod_sample_prefetched_w;
	wire signed [7:0] ch5_mod_recalc_sample_w = ch5_mod_sample_cached_w ?
		swp_mod_sample_q : mod_ram_read_q;

	wire [13:0] mix_accum_l_next_w = mix_acc_l_q + {3'd0, mix_scaled_l_q};
	wire [13:0] mix_accum_r_next_w = mix_acc_r_q + {3'd0, mix_scaled_r_q};

	// ---------------------------------------------------------------------
	// Bus and audio outputs
	// ---------------------------------------------------------------------

	// The VSU is a write-only device; this implementation answers reads
	// with zero and no wait states.
	assign dout_o = 16'h0000;
	assign ready_o = bus_cycle_w;
	// This strobe marks the edge that publishes the output sample.
	assign audio_sample_valid_o = ce_i && !reset_i &&
		(mix_state_q == MIX_ACCUM) && (mix_index_q == CHANNEL_LAST);

	// ---------------------------------------------------------------------
	// Waveform RAM: five 32-sample tables. Port A carries CPU and savestate
	// writes (and savestate readback); port B feeds the mixer.
	// ---------------------------------------------------------------------

	cache_ram_dp
	#(
		.ADDR_WIDTH(8),
		.DATA_WIDTH(6)
	)
	u_wave_ram
	(
		.clk_i(clk_i),
		.addr_a_i(wave_ram_write_addr_w),
		.wren_a_i(wave_ram_port_a_wren_w),
		.wdata_a_i(wave_ram_port_a_wdata_w),
		.q_a_o(wave_ram_write_q_w),
		.addr_b_i(wave_ram_read_addr_w),
		.wren_b_i(1'b0),
		.wdata_b_i(6'd0),
		.q_b_o(wave_ram_q_w)
	);

	// ---------------------------------------------------------------------
	// Savestate readback: packed state words, then wave/modulation memory
	// ---------------------------------------------------------------------

	always @* begin
		savestate_state_rdata_o = 64'd0;
		if (savestate_state_addr_i < 5'd6) begin
			savestate_state_rdata_o = {
				2'd0,
				duration_phase_q[savestate_state_addr_i[2:0]],
				duration_count_q[savestate_state_addr_i[2:0]],
				duration_set_q[savestate_state_addr_i[2:0]],
				auto_q[savestate_state_addr_i[2:0]],
				active_q[savestate_state_addr_i[2:0]],
				freq_count_q[savestate_state_addr_i[2:0]],
				freq_cur_q[savestate_state_addr_i[2:0]],
				freq_reg_q[savestate_state_addr_i[2:0]]};
		end else if (savestate_state_addr_i < 5'd12) begin
			savestate_state_rdata_o = {
				11'd0,
				envelope_phase_q[savestate_env_index_w],
				env_terminal_q[savestate_env_index_w],
				env_level_q[savestate_env_index_w],
				env_count_q[savestate_env_index_w],
				env_interval_q[savestate_env_index_w],
				env_enable_q[savestate_env_index_w],
				env_repeat_q[savestate_env_index_w],
				env_dir_up_q[savestate_env_index_w],
				env_reload_q[savestate_env_index_w],
				vol_r_q[savestate_env_index_w],
				vol_l_q[savestate_env_index_w],
				(savestate_state_addr_i < 5'd11) ?
					phase_q[savestate_env_index_w] : 5'd0,
				(savestate_state_addr_i < 5'd11) ?
					wave_sel_q[savestate_env_index_w] : 3'd0};
		end else begin
			case (savestate_state_addr_i)
				5'd12: savestate_state_rdata_o = {
					22'd0, sample_count_q, noise_base_count_q, pcm_base_count_q,
					noise_sample_q, noise_tap_q, noise_shift_q};
				5'd13: savestate_state_rdata_o = {
					swp_elapsed_q, swp_frame_cycles_q,
					swp_next_frame_cycles_q, swp_enable_q};
				5'd14: savestate_state_rdata_o = {
					swp_mod_mask_q[0], swp_mod_sample_valid_q, swp_mod_sample_q,
					mod_ram_read_valid_q, mod_ram_read_tag_q,
					swp_mod_lock_mode_q, swp_mod_base_freq_q,
					swp_mod_calc_pending_q, swp_mod_read_pending_q,
					swp_mod_read_addr_q, swp_mod_mask_q[1], swp_mod_index_q,
					swp_candidate_valid_q, swp_candidate_freq_q,
					swp_shift_q, swp_dir_up_q, swp_interval_q,
					swp_slow_q, swp_mod_q, swp_repeat_q};
				5'd15: savestate_state_rdata_o = {56'd0, mod_ram_read_q};
				5'd16: savestate_state_rdata_o = {
					26'd0, audio_l_o, audio_r_o, mix_state_q, mix_index_q};
				5'd17: savestate_state_rdata_o = {
					13'd0, mix_acc_l_q, mix_acc_r_q, mix_sample_q,
					mix_gain_l_q, mix_gain_r_q, mix_force_silence_q, mix_noise_q};
				5'd18: savestate_state_rdata_o = {
					42'd0, mix_scaled_l_q, mix_scaled_r_q};
				5'd19, 5'd20, 5'd21, 5'd22, 5'd23, 5'd24:
					savestate_state_rdata_o = {
						43'd0,
						mix_active_q[savestate_mix_index_w],
						mix_vol_l_q[savestate_mix_index_w],
						mix_vol_r_q[savestate_mix_index_w],
						mix_env_q[savestate_mix_index_w],
						mix_wave_addr_q[savestate_mix_index_w]};
				default: begin
				end
			endcase
		end
	end

	always @* begin
		if (savestate_wave_active_w) begin
			savestate_mem_rdata_o = {2'd0, wave_ram_write_q_w};
		end else if (savestate_mod_active_w) begin
			savestate_mem_rdata_o =
				mod_ram_shadow_q[savestate_mem_addr_i[4:0]];
		end else begin
			savestate_mem_rdata_o = 8'd0;
		end
	end

	// ---------------------------------------------------------------------
	// Helper functions and tasks
	// ---------------------------------------------------------------------

	function [3:0] noise_tap_bit_fn;
		input [2:0] tap_sel;
		begin
			case (tap_sel)
				3'd0: noise_tap_bit_fn = 4'd14;
				3'd1: noise_tap_bit_fn = 4'd10;
				3'd2: noise_tap_bit_fn = 4'd13;
				3'd3: noise_tap_bit_fn = 4'd4;
				3'd4: noise_tap_bit_fn = 4'd8;
				3'd5: noise_tap_bit_fn = 4'd6;
				3'd6: noise_tap_bit_fn = 4'd9;
				default: noise_tap_bit_fn = 4'd11;
			endcase
		end
	endfunction

	function noise_feedback_fn;
		input [14:0] shift_value;
		input [2:0] tap_sel;
		begin
			noise_feedback_fn =
				~(shift_value[7] ^ shift_value[noise_tap_bit_fn(tap_sel)]);
		end
	endfunction

	function [20:0] sweep_frame_cycles_fn;
		input       slow_clk;
		input [2:0] interval_value;
		reg [20:0] unit_cycles_v;
		begin
			unit_cycles_v = slow_clk ?
				SWEEP_LARGE_TICK_CYCLES : SWEEP_SMALL_TICK_CYCLES;
			case (interval_value)
				3'd0: sweep_frame_cycles_fn = unit_cycles_v;
				3'd1: sweep_frame_cycles_fn = unit_cycles_v;
				3'd2: sweep_frame_cycles_fn = unit_cycles_v << 1;
				3'd3: sweep_frame_cycles_fn =
					(unit_cycles_v << 1) + unit_cycles_v;
				3'd4: sweep_frame_cycles_fn = unit_cycles_v << 2;
				3'd5: sweep_frame_cycles_fn =
					(unit_cycles_v << 2) + unit_cycles_v;
				3'd6: sweep_frame_cycles_fn =
					(unit_cycles_v << 2) + (unit_cycles_v << 1);
				default: sweep_frame_cycles_fn =
					(unit_cycles_v << 2) + (unit_cycles_v << 1) + unit_cycles_v;
			endcase
		end
	endfunction

	function [11:0] sweep_candidate_fn;
		input [10:0] frequency_value;
		input        direction_up;
		input [2:0]  shift_amount;
		reg [10:0] step_v;
		reg [11:0] sum_v;
		begin
			step_v = frequency_value >> shift_amount;
			sum_v = 12'd0;
			if (direction_up) begin
				sum_v = {1'b0, frequency_value} + {1'b0, step_v};
				if (sum_v > 12'd2047) begin
					sweep_candidate_fn = {1'b1, 11'd0};
				end else begin
					sweep_candidate_fn = {1'b0, sum_v[10:0]};
				end
			end else if (frequency_value >= step_v) begin
				sweep_candidate_fn =
					{1'b0, frequency_value - step_v};
			end else begin
				sweep_candidate_fn = 12'd0;
			end
		end
	endfunction

	function [10:0] modulation_candidate_fn;
		input [10:0]       base_frequency;
		input signed [7:0] modulation_sample;
		input [1:0]        lock_mode;
		reg [10:0] sample_v;
		reg [10:0] result_v;
		begin
			// Keep the low 11 bits after adding the signed sample.
			sample_v = {{3{modulation_sample[7]}}, modulation_sample};
			result_v = base_frequency + sample_v;
			if (lock_mode == MOD_LOCK_LOW) begin
				result_v[7:0] = base_frequency[7:0];
			end else if (lock_mode == MOD_LOCK_HIGH) begin
				result_v[10:8] = base_frequency[10:8];
			end
			modulation_candidate_fn = result_v;
		end
	endfunction

	function [4:0] mix_gain_fn;
		input [3:0] volume_value;
		input [3:0] env_value;
		reg [7:0] pair01_v;
		reg [7:0] pair23_v;
		/* verilator lint_off UNUSEDSIGNAL */
		reg [7:0] product_v;
		/* verilator lint_on UNUSEDSIGNAL */
		begin
			pair01_v = (env_value[0] ? {4'd0, volume_value} : 8'd0) +
				(env_value[1] ? {3'd0, volume_value, 1'b0} : 8'd0);
			pair23_v = (env_value[2] ? {2'd0, volume_value, 2'b00} : 8'd0) +
				(env_value[3] ? {1'd0, volume_value, 3'b000} : 8'd0);
			product_v = pair01_v + pair23_v;
			if ((volume_value == 4'd0) || (env_value == 4'd0)) begin
				mix_gain_fn = 5'd0;
			end else begin
				mix_gain_fn = product_v[7:3] + 5'd1;
			end
		end
	endfunction

	function [10:0] scale_sample_fn;
		input [5:0] sample_value;
		input [4:0] gain_value;
		reg [10:0] pair01_v;
		reg [10:0] pair23_v;
		reg [10:0] pair4_v;
		begin
			pair01_v = (gain_value[0] ? {5'd0, sample_value} : 11'd0) +
				(gain_value[1] ? {4'd0, sample_value, 1'b0} : 11'd0);
			pair23_v = (gain_value[2] ? {3'd0, sample_value, 2'b00} : 11'd0) +
				(gain_value[3] ? {2'd0, sample_value, 3'b000} : 11'd0);
			pair4_v = gain_value[4] ?
				{1'd0, sample_value, 4'b0000} : 11'd0;
			scale_sample_fn = (pair01_v + pair23_v) + pair4_v;
		end
	endfunction

	wire [10:0] ch5_mod_recalc_candidate_w = modulation_candidate_fn(
		ch5_mod_recalc_base_w, ch5_mod_recalc_sample_w, ch5_mod_recalc_lock_w);

	// One sweep datapath handles frame, restart, frequency, and S5SWP updates.
	wire [11:0] swp_selected_candidate_w = sweep_candidate_fn(
		swp_candidate_source_w, swp_candidate_dir_w, swp_candidate_shift_w);

	wire noise_feedback_w = noise_feedback_fn(noise_shift_q, noise_tap_q);

	// Latch the sweep datapath result as the pending channel 5 candidate.
	// Overflow in the up direction (top bit set) also shuts the channel off.
	// Called only from the main register/sweep clocked process.
	task ch5_apply_sweep_result;
		begin
			swp_candidate_freq_q  <= swp_selected_candidate_w[10:0];
			swp_candidate_valid_q <= !swp_selected_candidate_w[11];
			if (swp_selected_candidate_w[11]) begin
				active_q[4] <= 1'b0;
			end
		end
	endtask

	// Start fetching a modulation RAM entry for channel 5. The candidate is
	// invalid until the read and the signed add complete on the next two
	// enabled edges. Called only from the main register/sweep clocked process.
	task ch5_start_mod_fetch;
		input [4:0]  entry_index;
		input [10:0] base_frequency;
		begin
			swp_mod_read_addr_q    <= entry_index;
			swp_mod_base_freq_q    <= base_frequency;
			swp_mod_read_pending_q <= 1'b1;
			swp_mod_calc_pending_q <= 1'b0;
			swp_candidate_valid_q  <= 1'b0;
			swp_mod_sample_valid_q <= 1'b0;
		end
	endtask

	// ---------------------------------------------------------------------
	// Modulation RAM (32 signed bytes) and its one-entry read register.
	// Hardware reset contents are unknown, so software must initialize it.
	// ---------------------------------------------------------------------

	always @(posedge clk_i) begin
		if (savestate_mod_active_w && savestate_mem_wren_i) begin
			mod_ram_shadow_q[savestate_mem_addr_i[4:0]] <=
				savestate_mem_wdata_i;
		end else if (savestate_state_wren_i &&
			(savestate_state_addr_i == 5'd15)) begin
			mod_ram_read_q <= savestate_state_wdata_i[7:0];
		end else if (ce_i) begin
			if (mod_ram_wren_w) begin
				mod_ram_shadow_q[addr_w[6:2]] <= write_byte_w;
			end
			mod_ram_read_q <= mod_ram_shadow_q[swp_mod_read_addr_q];
		end
	end

	// ---------------------------------------------------------------------
	// Base-rate dividers. Source clocks are shared; duration and envelope
	// counters remain per channel.
	// ---------------------------------------------------------------------

	always @(posedge clk_i) begin
		if (reset_i) begin
			pcm_base_count_q   <= 3'd0;
			noise_base_count_q <= 6'd0;
			sample_count_q     <= 9'd0;
		end else if (savestate_state_wren_i &&
			(savestate_state_addr_i == 5'd12)) begin
			pcm_base_count_q <= savestate_state_wdata_i[26:24];
			noise_base_count_q <= savestate_state_wdata_i[32:27];
			sample_count_q <= savestate_state_wdata_i[41:33];
		end else if (ce_i) begin
			if (pcm_tick_w) begin
				pcm_base_count_q <= 3'd0;
			end else begin
				pcm_base_count_q <= pcm_base_count_q + 3'd1;
			end

			if (noise_tick_w) begin
				noise_base_count_q <= 6'd0;
			end else begin
				noise_base_count_q <= noise_base_count_q + 6'd1;
			end

			if (sample_tick_w) begin
				sample_count_q <= 9'd0;
			end else begin
				sample_count_q <= sample_count_q + 9'd1;
			end
		end
	end

	// ---------------------------------------------------------------------
	// Main engine: duration/envelope sequencers, PCM phase and noise
	// timers, the channel 5 sweep frame, and CPU register writes. One
	// process, ordered so CPU writes come last and win over same-edge
	// channel updates.
	// ---------------------------------------------------------------------

	always @(posedge clk_i) begin
		if (reset_i) begin
			noise_shift_q  <= 15'd0;
			noise_tap_q    <= 3'd0;
			noise_sample_q <= 6'h3f;

			swp_enable_q            <= 1'b0;
			swp_repeat_q            <= 1'b0;
			swp_mod_q               <= 1'b0;
			swp_slow_q              <= 1'b0;
			swp_interval_q          <= 3'd0;
			swp_dir_up_q            <= 1'b0;
			swp_shift_q             <= 3'd0;
			swp_elapsed_q           <= 21'd0;
			swp_frame_cycles_q      <= SWEEP_SMALL_TICK_CYCLES;
			swp_next_frame_cycles_q <= SWEEP_SMALL_TICK_CYCLES;
			swp_candidate_freq_q    <= 11'd0;
			swp_candidate_valid_q   <= 1'b1;
			swp_mod_index_q         <= 5'd0;
			swp_mod_mask_q          <= 2'd0;
			swp_mod_read_addr_q     <= 5'd0;
			swp_mod_read_pending_q  <= 1'b0;
			swp_mod_calc_pending_q  <= 1'b0;
			swp_mod_base_freq_q     <= 11'd0;
			swp_mod_lock_mode_q     <= MOD_LOCK_NONE;
			mod_ram_read_tag_q      <= 5'd0;
			mod_ram_read_valid_q    <= 1'b0;
			swp_mod_sample_q        <= 8'sd0;
			swp_mod_sample_valid_q  <= 1'b0;

			for (chan_idx = 0; chan_idx < CHANNEL_COUNT; chan_idx = chan_idx + 1) begin
				active_q[chan_idx]         <= 1'b0;
				auto_q[chan_idx]           <= 1'b0;
				duration_set_q[chan_idx]   <= 5'd0;
				duration_count_q[chan_idx] <= 5'd0;
				duration_phase_q[chan_idx] <= 17'd0;
				vol_l_q[chan_idx]          <= 4'd0;
				vol_r_q[chan_idx]          <= 4'd0;
				freq_reg_q[chan_idx]       <= 11'd0;
				freq_cur_q[chan_idx]       <= 11'd0;
				freq_count_q[chan_idx]     <= 11'd0;
				env_reload_q[chan_idx]     <= 4'd0;
				env_dir_up_q[chan_idx]     <= 1'b0;
				env_repeat_q[chan_idx]     <= 1'b0;
				env_enable_q[chan_idx]     <= 1'b0;
				env_interval_q[chan_idx]   <= 3'd0;
				env_count_q[chan_idx]      <= 3'd0;
				env_level_q[chan_idx]      <= 4'd0;
				env_terminal_q[chan_idx]   <= 1'b0;
				envelope_phase_q[chan_idx] <= 19'd0;
			end

			for (chan_idx = 0; chan_idx < PCM_COUNT; chan_idx = chan_idx + 1) begin
				wave_sel_q[chan_idx] <= 3'd0;
				phase_q[chan_idx]    <= 5'd0;
			end
		end else if (savestate_state_wren_i) begin
			if (savestate_state_addr_i < 5'd6) begin
				freq_reg_q[savestate_state_addr_i[2:0]] <=
					savestate_state_wdata_i[10:0];
				freq_cur_q[savestate_state_addr_i[2:0]] <=
					savestate_state_wdata_i[21:11];
				freq_count_q[savestate_state_addr_i[2:0]] <=
					savestate_state_wdata_i[32:22];
				active_q[savestate_state_addr_i[2:0]] <=
					savestate_state_wdata_i[33];
				auto_q[savestate_state_addr_i[2:0]] <=
					savestate_state_wdata_i[34];
				duration_set_q[savestate_state_addr_i[2:0]] <=
					savestate_state_wdata_i[39:35];
				duration_count_q[savestate_state_addr_i[2:0]] <=
					savestate_state_wdata_i[44:40];
				duration_phase_q[savestate_state_addr_i[2:0]] <=
					savestate_state_wdata_i[61:45];
			end else if (savestate_state_addr_i < 5'd12) begin
				vol_l_q[savestate_env_index_w] <=
					savestate_state_wdata_i[11:8];
				vol_r_q[savestate_env_index_w] <=
					savestate_state_wdata_i[15:12];
				env_reload_q[savestate_env_index_w] <=
					savestate_state_wdata_i[19:16];
				env_dir_up_q[savestate_env_index_w] <=
					savestate_state_wdata_i[20];
				env_repeat_q[savestate_env_index_w] <=
					savestate_state_wdata_i[21];
				env_enable_q[savestate_env_index_w] <=
					savestate_state_wdata_i[22];
				env_interval_q[savestate_env_index_w] <=
					savestate_state_wdata_i[25:23];
				env_count_q[savestate_env_index_w] <=
					savestate_state_wdata_i[28:26];
				env_level_q[savestate_env_index_w] <=
					savestate_state_wdata_i[32:29];
				env_terminal_q[savestate_env_index_w] <=
					savestate_state_wdata_i[33];
				envelope_phase_q[savestate_env_index_w] <=
					savestate_state_wdata_i[52:34];
				if (savestate_state_addr_i < 5'd11) begin
					wave_sel_q[savestate_env_index_w] <=
						savestate_state_wdata_i[2:0];
					phase_q[savestate_env_index_w] <=
						savestate_state_wdata_i[7:3];
				end
			end else begin
				case (savestate_state_addr_i)
					5'd12: begin
						noise_shift_q <= savestate_state_wdata_i[14:0];
						noise_tap_q <= savestate_state_wdata_i[17:15];
						noise_sample_q <= savestate_state_wdata_i[23:18];
					end
					5'd13: begin
						swp_enable_q <= savestate_state_wdata_i[0];
						swp_next_frame_cycles_q <= savestate_state_wdata_i[21:1];
						swp_frame_cycles_q <= savestate_state_wdata_i[42:22];
						swp_elapsed_q <= savestate_state_wdata_i[63:43];
					end
					5'd14: begin
						swp_repeat_q <= savestate_state_wdata_i[0];
						swp_mod_q <= savestate_state_wdata_i[1];
						swp_slow_q <= savestate_state_wdata_i[2];
						swp_interval_q <= savestate_state_wdata_i[5:3];
						swp_dir_up_q <= savestate_state_wdata_i[6];
						swp_shift_q <= savestate_state_wdata_i[9:7];
						swp_candidate_freq_q <= savestate_state_wdata_i[20:10];
						swp_candidate_valid_q <= savestate_state_wdata_i[21];
						swp_mod_index_q <= savestate_state_wdata_i[26:22];
						swp_mod_mask_q <= {savestate_state_wdata_i[27],
							savestate_state_wdata_i[63]};
						swp_mod_read_addr_q <= savestate_state_wdata_i[32:28];
						swp_mod_read_pending_q <= savestate_state_wdata_i[33];
						swp_mod_calc_pending_q <= savestate_state_wdata_i[34];
						swp_mod_base_freq_q <= savestate_state_wdata_i[45:35];
						swp_mod_lock_mode_q <= savestate_state_wdata_i[47:46];
						mod_ram_read_tag_q <= savestate_state_wdata_i[52:48];
						mod_ram_read_valid_q <= savestate_state_wdata_i[53];
						swp_mod_sample_q <= savestate_state_wdata_i[61:54];
						swp_mod_sample_valid_q <= savestate_state_wdata_i[62];
					end
					default: begin
					end
				endcase
			end
		end else if (ce_i) begin
			// Track the modulation RAM output even while sweep mode is selected.
			mod_ram_read_tag_q <= swp_mod_read_addr_q;
			mod_ram_read_valid_q <= 1'b1;

			swp_mod_lock_mode_q <= mod_lock_next_w;

			// Duration and envelope engines run in parallel.
			for (chan_idx = 0; chan_idx < CHANNEL_COUNT; chan_idx = chan_idx + 1) begin
				if (active_q[chan_idx] && auto_q[chan_idx]) begin
					if (duration_phase_q[chan_idx] >=
						(DURATION_TICK_CYCLES - 17'd1)) begin
						duration_phase_q[chan_idx] <= 17'd0;
						if (duration_count_q[chan_idx] >= duration_set_q[chan_idx]) begin
							active_q[chan_idx] <= 1'b0;
							duration_count_q[chan_idx] <= 5'd0;
						end else begin
							duration_count_q[chan_idx] <=
								duration_count_q[chan_idx] + 5'd1;
						end
					end else begin
						duration_phase_q[chan_idx] <=
							duration_phase_q[chan_idx] + 17'd1;
					end
				end

				// The base clock keeps running; channel state gates envelope steps.
				if (envelope_phase_q[chan_idx] >=
					(ENVELOPE_TICK_CYCLES - 19'd1)) begin
					envelope_phase_q[chan_idx] <= 19'd0;
					// Automatic shutoff suppresses a coincident envelope step.
					if (active_q[chan_idx] && env_enable_q[chan_idx] &&
						!env_terminal_q[chan_idx] &&
						!(auto_q[chan_idx] &&
						  (duration_phase_q[chan_idx] >=
							(DURATION_TICK_CYCLES - 17'd1)) &&
						  (duration_count_q[chan_idx] >=
							duration_set_q[chan_idx]))) begin
						if (env_count_q[chan_idx] >= env_interval_q[chan_idx]) begin
							env_count_q[chan_idx] <= 3'd0;
							if (env_dir_up_q[chan_idx]) begin
								if (env_level_q[chan_idx] != 4'hf) begin
									env_level_q[chan_idx] <=
										env_level_q[chan_idx] + 4'd1;
								end else if (env_repeat_q[chan_idx]) begin
									env_level_q[chan_idx] <= env_reload_q[chan_idx];
								end else begin
									env_terminal_q[chan_idx] <= 1'b1;
								end
							end else if (env_level_q[chan_idx] != 4'h0) begin
								env_level_q[chan_idx] <=
									env_level_q[chan_idx] - 4'd1;
							end else if (env_repeat_q[chan_idx]) begin
								env_level_q[chan_idx] <= env_reload_q[chan_idx];
							end else begin
								env_terminal_q[chan_idx] <= 1'b1;
							end
						end else begin
							env_count_q[chan_idx] <= env_count_q[chan_idx] + 3'd1;
						end
					end
				end else begin
					envelope_phase_q[chan_idx] <=
						envelope_phase_q[chan_idx] + 19'd1;
				end
			end

			if (pcm_tick_w) begin
				for (chan_idx = 0; chan_idx < PCM_COUNT; chan_idx = chan_idx + 1) begin
					if (active_q[chan_idx]) begin
						if (freq_count_q[chan_idx] == 11'd0) begin
							freq_count_q[chan_idx] <= ~freq_cur_q[chan_idx];
							phase_q[chan_idx] <= phase_q[chan_idx] + 5'd1;
						end else begin
							freq_count_q[chan_idx] <= freq_count_q[chan_idx] - 11'd1;
						end
					end
				end
			end

			if (noise_tick_w && active_q[5]) begin
				if (freq_count_q[5] == 11'd0) begin
					freq_count_q[5] <= ~freq_cur_q[5];
					noise_shift_q  <= {noise_shift_q[13:0], noise_feedback_w};
					noise_sample_q <= noise_feedback_w ? 6'h3f : 6'h00;
				end else begin
					freq_count_q[5] <= freq_count_q[5] - 11'd1;
				end
			end

			// Two enabled edges separate the RAM read from signed arithmetic.
			if (swp_mod_read_pending_q) begin
				swp_mod_read_pending_q <= 1'b0;
				swp_mod_calc_pending_q <= 1'b1;
			end
			if (swp_mod_calc_pending_q) begin
				swp_candidate_freq_q <= modulation_candidate_fn(
					swp_mod_base_freq_q, mod_ram_read_q, swp_mod_lock_mode_q);
				swp_candidate_valid_q <= 1'b1;
				swp_mod_calc_pending_q <= 1'b0;
				swp_mod_sample_q <= mod_ram_read_q;
				swp_mod_sample_valid_q <= 1'b1;
			end

			// S5INT restarts the frame and discards the same-edge candidate.
			if (active_q[4] && !ch5_int_write_w) begin
				if (ch5_frame_due_w) begin
					if (swp_frame_advance_w) begin
						freq_cur_q[4] <= swp_candidate_freq_q;
					end
					swp_elapsed_q <= 21'd0;
					swp_frame_cycles_q <= swp_next_frame_cycles_q;

					if (swp_mod_q) begin
						if (swp_frame_advance_w) begin
							if (swp_mod_index_q == 5'd31) begin
								// Consuming entry 31 goes terminal at once;
								// repeat restarts a fresh pass from entry 0.
								swp_mod_mask_q <= 2'd2;
								if (swp_repeat_q) begin
									swp_mod_index_q <= 5'd0;
									ch5_start_mod_fetch(5'd0, freq_reg_q[4]);
								end else begin
									swp_mod_read_pending_q <= 1'b0;
									swp_mod_calc_pending_q <= 1'b0;
									swp_candidate_valid_q  <= 1'b0;
								end
							end else begin
								swp_mod_index_q <= swp_mod_index_q + 5'd1;
								ch5_start_mod_fetch(
									swp_mod_index_q + 5'd1, freq_reg_q[4]);
							end
						end
					end else begin
						ch5_apply_sweep_result;
						// The position counter also consumes sweep frames
						// (red-viper, shrooms); the pass mark terminates a
						// later non-repeat modulation switch.
						if (swp_frame_advance_w) begin
							// Keep the modulation entry ready as sweep advances the shared index.
							swp_mod_read_addr_q <= swp_post_frame_mod_index_w;
							if (swp_mod_index_q == 5'd31) begin
								swp_mod_mask_q <= 2'd1;
								if (swp_repeat_q) begin
									swp_mod_index_q <= 5'd0;
								end
							end else begin
								swp_mod_index_q <= swp_mod_index_q + 5'd1;
							end
						end
					end
				end else begin
					swp_elapsed_q <= swp_elapsed_q + 21'd1;
				end
			end

			// CPU register writes override all same-edge channel events.
			if (reg_write_w) begin
				case (write_reg_w)
					REG_INT: begin
						active_q[write_ch_w]         <= write_byte_w[7];
						auto_q[write_ch_w]           <= write_byte_w[5];
						duration_set_q[write_ch_w]   <= write_byte_w[4:0];
						duration_count_q[write_ch_w] <= 5'd0;
						duration_phase_q[write_ch_w] <= 17'd0;
						freq_count_q[write_ch_w]     <= ~freq_cur_q[write_ch_w];
						env_count_q[write_ch_w]      <= 3'd0;
						env_terminal_q[write_ch_w]   <= 1'b0;

						if (write_ch_w < PCM_COUNT_U3) begin
							phase_q[write_ch_w] <= 5'd0;
						end else begin
							// An all-zero seed produces feedback one on S6INT.
							noise_shift_q  <= 15'd1;
							noise_sample_q <= 6'h3f;
						end

						if (write_ch_w == 3'd4) begin
							swp_elapsed_q <= 21'd0;
							swp_frame_cycles_q <=
								sweep_frame_cycles_fn(swp_slow_q, swp_interval_q);
							swp_next_frame_cycles_q <=
								sweep_frame_cycles_fn(swp_slow_q, swp_interval_q);
							swp_mod_index_q <= 5'd0;
							swp_mod_mask_q <= 2'd0;
							swp_mod_read_addr_q <= 5'd0;
							swp_candidate_freq_q <= freq_cur_q[4];
							swp_candidate_valid_q <= 1'b0;
							swp_mod_read_pending_q <= 1'b0;
							swp_mod_calc_pending_q <= 1'b0;
							swp_mod_sample_valid_q <= 1'b0;

							if (swp_mod_q) begin
								if (swp_enable_q && (swp_interval_q != 3'd0)) begin
									ch5_start_mod_fetch(5'd0, freq_reg_q[4]);
								end
							end else begin
								ch5_apply_sweep_result;
							end
						end
					end

					REG_LRV: begin
						vol_l_q[write_ch_w] <= write_byte_w[7:4];
						vol_r_q[write_ch_w] <= write_byte_w[3:0];
					end

					REG_FQL: begin
						freq_reg_q[write_ch_w][7:0] <= write_byte_w;
						freq_cur_q[write_ch_w][7:0] <= write_byte_w;
						if ((write_ch_w == 3'd4) && !swp_mod_q) begin
							ch5_apply_sweep_result;
						end
					end

					REG_FQH: begin
						freq_reg_q[write_ch_w][10:8] <= write_byte_w[2:0];
						freq_cur_q[write_ch_w][10:8] <= write_byte_w[2:0];
						if ((write_ch_w == 3'd4) && !swp_mod_q) begin
							ch5_apply_sweep_result;
						end
					end

					REG_EV0: begin
						// Load the level within the running envelope frame.
						env_reload_q[write_ch_w]   <= write_byte_w[7:4];
						env_dir_up_q[write_ch_w]   <= write_byte_w[3];
						env_interval_q[write_ch_w] <= write_byte_w[2:0];
						env_level_q[write_ch_w]    <= write_byte_w[7:4];

						if ((write_ch_w == 3'd4) && !swp_mod_q) begin
							ch5_apply_sweep_result;
						end
					end

					REG_EV1: begin
						env_repeat_q[write_ch_w] <= write_byte_w[1];
						env_enable_q[write_ch_w] <= write_byte_w[0];

						if (write_ch_w == 3'd4) begin
							swp_enable_q <= write_byte_w[6];
							swp_repeat_q <= write_byte_w[5];
							swp_mod_q    <= write_byte_w[4];
							swp_mod_read_pending_q <= 1'b0;
							swp_mod_calc_pending_q <= 1'b0;

							if (write_byte_w[4]) begin
								swp_candidate_valid_q <= 1'b0;
							end else begin
								swp_mod_sample_valid_q <= 1'b0;
								ch5_apply_sweep_result;
							end
						end else if (write_ch_w == 3'd5) begin
							noise_tap_q    <= write_byte_w[6:4];
							// S6EV1 uses the same zero-seed step as S6INT.
							noise_shift_q  <= 15'd1;
							noise_sample_q <= 6'h3f;
						end
					end

					REG_RAM: begin
						if (write_ch_w < PCM_COUNT_U3) begin
							wave_sel_q[write_ch_w] <= write_byte_w[2:0];
						end
					end

					REG_SWP: begin
						if (write_ch_w == 3'd4) begin
							swp_slow_q     <= write_byte_w[7];
							swp_interval_q <= write_byte_w[6:4];
							swp_dir_up_q   <= write_byte_w[3];
							swp_shift_q    <= write_byte_w[2:0];
							swp_next_frame_cycles_q <=
								sweep_frame_cycles_fn(write_byte_w[7], write_byte_w[6:4]);
							if (swp_elapsed_q <
								sweep_frame_cycles_fn(write_byte_w[7], write_byte_w[6:4])) begin
								swp_frame_cycles_q <=
									sweep_frame_cycles_fn(write_byte_w[7], write_byte_w[6:4]);
							end

							swp_mod_read_pending_q <= 1'b0;
							swp_mod_calc_pending_q <= 1'b0;
							if (swp_mod_q) begin
								swp_candidate_valid_q <= 1'b0;
							end else begin
								ch5_apply_sweep_result;
							end
						end
					end

					default: begin
					end
				endcase

				// Recompute the modulated candidate after a channel 5 staging
				// write: reuse a cached/prefetched sample when one matches,
				// otherwise start a fresh modulation RAM fetch.
				if (ch5_mod_recalc_write_w) begin
					if (ch5_mod_recalc_sample_ready_w) begin
						swp_mod_base_freq_q    <= ch5_mod_recalc_base_w;
						swp_candidate_freq_q   <= ch5_mod_recalc_candidate_w;
						swp_candidate_valid_q  <= 1'b1;
						swp_mod_read_pending_q <= 1'b0;
						swp_mod_calc_pending_q <= 1'b0;
					end else begin
						ch5_start_mod_fetch(
							swp_post_frame_mod_index_w, ch5_mod_recalc_base_w);
					end
				end

				if (ch5_mask_promote_w) begin
					swp_mod_mask_q <= 2'd2;
				end
			end else if (sstop_write_w && write_byte_w[0]) begin
				for (chan_idx = 0; chan_idx < CHANNEL_COUNT; chan_idx = chan_idx + 1) begin
					active_q[chan_idx] <= 1'b0;
				end
			end
		end
	end

	// ---------------------------------------------------------------------
	// Serial mixer. Each sample window snapshots all channel controls,
	// then one shared datapath runs READ, GAIN, SCALE, ACCUM per channel
	// (four enabled edges each). Inactive channels pass zero through the
	// same schedule so timing never depends on channel state.
	// ---------------------------------------------------------------------

	always @(posedge clk_i) begin
		if (reset_i) begin
			audio_l_o      <= 16'd0;
			audio_r_o      <= 16'd0;
			mix_state_q    <= MIX_IDLE;
			mix_index_q    <= 3'd0;
			mix_acc_l_q    <= 14'd0;
			mix_acc_r_q    <= 14'd0;
			mix_sample_q   <= 6'd0;
			mix_gain_l_q   <= 5'd0;
			mix_gain_r_q   <= 5'd0;
			mix_scaled_l_q <= 11'd0;
			mix_scaled_r_q <= 11'd0;
			mix_force_silence_q <= 1'b0;
			mix_noise_q    <= 6'd0;
			for (mix_idx = 0; mix_idx < CHANNEL_COUNT; mix_idx = mix_idx + 1) begin
				mix_active_q[mix_idx]    <= 1'b0;
				mix_vol_l_q[mix_idx]     <= 4'd0;
				mix_vol_r_q[mix_idx]     <= 4'd0;
				mix_env_q[mix_idx]       <= 4'd0;
				mix_wave_addr_q[mix_idx] <= 8'd0;
			end
		end else if (savestate_state_wren_i) begin
			case (savestate_state_addr_i)
				5'd16: begin
					mix_index_q <= savestate_state_wdata_i[2:0];
					mix_state_q <= savestate_state_wdata_i[5:3];
					audio_r_o <= savestate_state_wdata_i[21:6];
					audio_l_o <= savestate_state_wdata_i[37:22];
				end
				5'd17: begin
					mix_noise_q <= savestate_state_wdata_i[5:0];
					mix_force_silence_q <= savestate_state_wdata_i[6];
					mix_gain_r_q <= savestate_state_wdata_i[11:7];
					mix_gain_l_q <= savestate_state_wdata_i[16:12];
					mix_sample_q <= savestate_state_wdata_i[22:17];
					mix_acc_r_q <= savestate_state_wdata_i[36:23];
					mix_acc_l_q <= savestate_state_wdata_i[50:37];
				end
				5'd18: begin
					mix_scaled_r_q <= savestate_state_wdata_i[10:0];
					mix_scaled_l_q <= savestate_state_wdata_i[21:11];
				end
				5'd19, 5'd20, 5'd21, 5'd22, 5'd23, 5'd24: begin
					mix_wave_addr_q[savestate_mix_index_w] <=
						savestate_state_wdata_i[7:0];
					mix_env_q[savestate_mix_index_w] <=
						savestate_state_wdata_i[11:8];
					mix_vol_r_q[savestate_mix_index_w] <=
						savestate_state_wdata_i[15:12];
					mix_vol_l_q[savestate_mix_index_w] <=
						savestate_state_wdata_i[19:16];
					mix_active_q[savestate_mix_index_w] <=
						savestate_state_wdata_i[20];
				end
				default: begin
				end
			endcase
		end else if (ce_i) begin
			// Silence any old mixer snapshot still in flight during a wave write.
			if (wave_ram_wren_w && (mix_state_q != MIX_IDLE)) begin
				mix_force_silence_q <= 1'b1;
			end

			if (sample_tick_w && (mix_state_q == MIX_IDLE)) begin
				mix_force_silence_q <= 1'b0;
				mix_state_q <= MIX_READ;
				mix_index_q <= 3'd0;
				mix_acc_l_q <= 14'd0;
				mix_acc_r_q <= 14'd0;
				mix_noise_q <= noise_sample_q;
				for (mix_idx = 0; mix_idx < CHANNEL_COUNT; mix_idx = mix_idx + 1) begin
					mix_active_q[mix_idx] <= active_q[mix_idx];
					mix_vol_l_q[mix_idx]  <= vol_l_q[mix_idx];
					mix_vol_r_q[mix_idx]  <= vol_r_q[mix_idx];
					mix_env_q[mix_idx]    <= env_level_q[mix_idx];
				end
				for (mix_idx = 0; mix_idx < PCM_COUNT; mix_idx = mix_idx + 1) begin
					mix_wave_addr_q[mix_idx] <= {wave_sel_q[mix_idx], phase_q[mix_idx]};
				end
				mix_wave_addr_q[5] <= 8'd0;
			end

			case (mix_state_q)
				MIX_IDLE: begin
				end

				MIX_READ: begin
					mix_state_q <= MIX_GAIN;
				end

				MIX_GAIN: begin
					if (mix_index_q < PCM_COUNT_U3) begin
						if (mix_active_q[mix_index_q] &&
							(mix_wave_addr_q[mix_index_q][7:5] < WAVE_COUNT_U3)) begin
							mix_sample_q <= wave_ram_q_w;
						end else begin
							mix_sample_q <= 6'd0;
						end
					end else if (mix_active_q[mix_index_q]) begin
						mix_sample_q <= mix_noise_q;
					end else begin
						mix_sample_q <= 6'd0;
					end
					mix_gain_l_q <= mix_gain_fn(
						mix_vol_l_q[mix_index_q], mix_env_q[mix_index_q]);
					mix_gain_r_q <= mix_gain_fn(
						mix_vol_r_q[mix_index_q], mix_env_q[mix_index_q]);
					mix_state_q <= MIX_SCALE;
				end

				MIX_SCALE: begin
					mix_scaled_l_q <= scale_sample_fn(mix_sample_q, mix_gain_l_q);
					mix_scaled_r_q <= scale_sample_fn(mix_sample_q, mix_gain_r_q);
					mix_state_q <= MIX_ACCUM;
				end

				MIX_ACCUM: begin
					if (mix_index_q == CHANNEL_LAST) begin
						// Zero-fill the 10-bit unsigned VSU output for MiSTer.
						if (mix_force_silence_q || wave_ram_wren_w) begin
							audio_l_o <= 16'd0;
							audio_r_o <= 16'd0;
						end else begin
							audio_l_o <= {mix_accum_l_next_w[13:4], 6'b000000};
							audio_r_o <= {mix_accum_r_next_w[13:4], 6'b000000};
						end
						mix_force_silence_q <= 1'b0;
						mix_state_q <= MIX_IDLE;
					end else begin
						mix_acc_l_q <= mix_accum_l_next_w;
						mix_acc_r_q <= mix_accum_r_next_w;
						mix_index_q <= mix_index_q + 3'd1;
						mix_state_q <= MIX_READ;
					end
				end

				default: begin
					mix_state_q <= MIX_IDLE;
				end
			endcase

`ifndef SYNTHESIS
			if (sample_tick_w && (mix_state_q != MIX_IDLE)) begin
				$error("VSU mixer overrun: sample tick arrived before fixed pipeline retired");
			end
`endif
		end
	end

endmodule
