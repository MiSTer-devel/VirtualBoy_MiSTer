// Copyright (c) 2026 Jamie Blanks

// NEC V810 instruction fetch unit.
//
// With the cache on, one aligned word a clock comes from the 1 KB
// direct-mapped cache (128 lines of 8 bytes, a valid bit per word) into a
// five-word queue. Reads run up to a window 6 bytes past the instruction in
// decode, 2 bytes further for each clock decode is held in that line (up to
// 14). Any word in the window that misses starts a fill of four words
// forward from it, across lines, each marked valid as it is written; a RETI
// or exception target fills from its line's base. A fill, once on the bus,
// always completes, also when the jump in decode redirects. A taken
// conditional branch drops one that has not started; its line keeps the new
// tag with no word valid. With the cache off, aligned words stream from the
// bus through necv810_fetch_stream.
//
//   redirect -> fpc -> cache read (addr at phi1, data at next CE) -> queue
//                       | miss                                         |
//                       v                                              v
//                  fill engine -> bus (4 words)              decode (dpc)
module necv810_fetch (
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        ce_i,

	// Decode side. A redirect at a CE drops everything and fetches from pc.
	input  wire        redirect_i,
	input  wire [31:0] redirect_pc_i,
	// The redirect comes from the micro-sequencer (exception vector, RETI).
	input  wire        redirect_seq_i,
	input  wire        take_i,
	output wire        inst_valid_o,
	output wire [31:0] inst_pc_o,
	output wire [31:0] inst_iw_o,
	// Stop starting new fetches (bus hold, checkpoint, maintenance).
	input  wire        hold_i,
	// Savestate pause: as hold_i, but a running fill still completes.
	input  wire        pause_i,
	// Stores are buffered or one ended in the last two clocks: no fill word
	// starts yet.
	input  wire        fill_hold_i,
	// A taken conditional branch waits in EX behind a load.
	input  wire        bcond_load_wait_i,
	// A taken conditional branch (not BR) redirects this clock.
	input  wire        bcond_redirect_i,
	// The instruction in decode, and whether it is held this clock for a
	// reason that lets the fetch run on (not RETI or other microcode).
	input  wire        id_valid_i,
	input  wire [31:0] id_pc_i,
	input  wire        id_hold_i,
	input  wire        id_jump_i,
	// BR, JR or JAL in decode.
	input  wire        id_rel_jump_i,
	input  wire        id_long_i,

	input  wire        cache_on_i,
	// A CHCW clear, dump or restore is running: no fetch starts.
	input  wire        restart_hold_i,
	// An LDSR to CHCW is in decode or EX: a miss starts no fill, as the code
	// after it is fetched again.
	input  wire        fill_stop_i,
	// A data request is waiting: the stream starts no new word.
	input  wire        data_wait_i,
	// The data request waiting is a load reading ahead of a long op.
	input  wire        data_ahead_i,

	// One bus request port; the parent arbitrates data first.
	output wire        bus_req_o,
	output wire [31:0] bus_addr_o,
	output wire        bus_branch_o,
	input  wire        bus_accept_i,
	input  wire        bus_complete_i,
	input  wire [31:0] bus_rdata_i,
	output wire        busy_o,

	// Cache maintenance: clear tags first..last (CHCW ICC). Reset leaves the tags.
	input  wire        clear_i,
	input  wire [6:0]  clear_first_i,
	input  wire [6:0]  clear_last_i,
	output wire        clear_busy_o,

	// CHCW dump (ICD) and restore (ICR): 128 lines, then 128 tags, at sa.
	// Words go through the parent's system bus port; xfer_bus_o is set while
	// this unit owns the bus.
	input  wire        xfer_i,
	input  wire        xfer_dump_i,
	input  wire [23:0] xfer_sa_i,
	output wire        xfer_busy_o,
	output wire        xfer_bus_o,
	output wire        xfer_req_o,
	output wire [31:0] xfer_addr_o,
	output wire        xfer_write_o,
	output wire [31:0] xfer_wdata_o,
	input  wire        xfer_accept_i,
	input  wire        xfer_complete_i,

	// Savestate access to cache data (1 KB) and tags (512 B).
	input  wire        savestate_active_i,
	input  wire [10:0] savestate_addr_i,
	input  wire        savestate_rden_i,
	input  wire        savestate_wren_i,
	input  wire [7:0]  savestate_wdata_i,
	output wire [7:0]  savestate_rdata_o
);
	localparam integer DEPTH = 5;
	// Clocks a CHCW clear waits before its first line.
	localparam [3:0] CLEAR_SETUP = 4'd15;

	// Decode PC: the next instruction offered to decode.
	reg  [31:0] dpc_q;

	// ---------------------------------------------------------------
	// Cache path.
	// ---------------------------------------------------------------
	// The queue is a ring of words in address order from decode's word (dpc).
	// Each slot keeps two predecoded bits per halfword: a 32-bit opcode starts
	// there, and a RETI starts there.
	(* ramstyle = "logic" *) reg [31:0] q_data_q [0:DEPTH-1];
	reg  [1:0]  q_long_q [0:DEPTH-1];
	reg  [1:0]  q_reti_q [0:DEPTH-1];
	reg  [2:0]  q_head_q;       // slot of dpc's word
	reg  [2:0]  q_count_q;      // words held from dpc's word on
	// Halfwords requested past dpc: fpc x 2 - dpc. The read issue compares it
	// with the window, so the next read address comes from registered state.
	reg  [5:0]  lvl_q;
	// A read of a word decode has already left (the first read after the cache
	// is switched on, or a missed word read again after its fill): the word is
	// spent when it arrives.
	reg         rd_stale_q;

	reg  [29:0] fpc_q;          // next word to read from the cache
	reg         rd_valid_q;     // a read was issued last clock
	reg  [29:0] rd_addr_q;
	reg         redirected_q;   // first read after a redirect (branch status)
	reg         seq_tgt_q;      // that redirect came from the micro-sequencer

	reg         fill_q;         // fill engine busy
	reg  [28:0] fill_line_q;    // line of the word on the bus
	reg         fill_word_q;    // that word within its line
	reg  [1:0]  fill_n_q;       // words already written
	reg         fill_branch_q;  // first fill word follows a redirect
	reg         fill_active_q;  // a fill word is on the bus

	reg  [6:0]  clear_index_q;
	reg         clear_odd_q;    // second clock of the line being cleared
	reg  [6:0]  clear_last_q;
	reg         clear_q;
	reg  [3:0]  clear_wait_q;   // setup clocks left

	reg         xfer_q;         // dump or restore running
	reg         xfer_dump_q;
	reg  [23:0] xfer_sa_q;
	reg  [8:0]  xfer_idx_q;     // words 0-255 are line data, 256-383 tags
	reg         xfer_active_q;  // a word is on the bus
	reg  [31:0] xfer_lo_q;      // restore: first word of the line

	wire [63:0] ic_data_w;
	wire [31:0] ic_tag_w;
	reg         ic_data_we_v;
	reg  [6:0]  ic_data_wa_v;
	reg  [63:0] ic_data_wd_v;
	reg         ic_tag_we_v;
	reg  [6:0]  ic_tag_wa_v;
	reg  [31:0] ic_tag_wd_v;
	wire [7:0]  ss_data_rdata_w;
	wire [7:0]  ss_tag_rdata_w;

	// The read address must be stable from the CE so the RAM takes it at phi1.
	wire [6:0]  xfer_index_w = xfer_idx_q[8] ? xfer_idx_q[6:0] : xfer_idx_q[7:1];
	// During a fill the RAMs read the line being filled, to merge its other word.
	wire [6:0]  rd_index_w = xfer_q ? xfer_index_w : fill_q ? fill_line_q[6:0] : rd_addr_q[7:1];
	wire        rd_tag_hit_w = (ic_tag_w[21:0] == rd_addr_q[29:8]) &&
		(rd_addr_q[0] ? ic_tag_w[23] : ic_tag_w[22]);
	// A read issued as a fill starts is dropped when it returns.
	wire        rd_hit_w = cache_on_i && rd_valid_q && rd_tag_hit_w && !clear_q && !xfer_q && !fill_q;
	wire        rd_miss_w = cache_on_i && rd_valid_q && !rd_tag_hit_w && !clear_q && !xfer_q && !fill_q;
	wire [31:0] rd_word_w = rd_addr_q[0] ?
		{ic_data_w[15:0], ic_data_w[31:16]} : {ic_data_w[47:32], ic_data_w[63:48]};

	// Ring slot n places after slot h.
	function automatic [2:0] slot_fn(input [2:0] h, input [2:0] n);
		reg [3:0] t;
		begin
			t = {1'b0, h} + {1'b0, n};
			slot_fn = (t >= 4'd5) ? t[2:0] - 3'd5 : t[2:0];
		end
	endfunction
	wire [2:0]  h0_w = q_head_q;
	wire [2:0]  h1_w = slot_fn(q_head_q, 3'd1);
	wire [2:0]  tail_w = slot_fn(q_head_q, q_count_q);
	// The word read this clock lands at the tail and is usable at once.
	wire        rd_push_w = rd_hit_w && !rd_stale_q;
	wire        hit0_v = (q_count_q != 3'd0) || (rd_push_w && (q_count_q == 3'd0));
	wire        hit1_v = (q_count_q >= 3'd2) || (rd_push_w && (q_count_q == 3'd1));
	wire [31:0] word0_v = (q_count_q != 3'd0) ? q_data_q[h0_w] : rd_word_w;
	wire [31:0] word1_v = (q_count_q >= 3'd2) ? q_data_q[h1_w] : rd_word_w;
	// Halfword order in a word: the lower address is the low half.
	wire [15:0] c_half0_w = dpc_q[1] ? word0_v[31:16] : word0_v[15:0];
	wire [15:0] c_half1_w = dpc_q[1] ? word1_v[15:0] : word0_v[31:16];
	wire        c_long_w = (c_half0_w[15:13] == 3'b101) || (c_half0_w[15:13] == 3'b110) ||
		(c_half0_w[15:13] == 3'b111);
	wire        c_need_next_w = c_long_w && dpc_q[1];
	wire        c_ready_w = hit0_v && (!c_need_next_w || hit1_v);
	wire [31:0] c_iw_w = {c_half0_w, c_long_w ? c_half1_w : 16'd0};
	// Predecode of the word read this clock, kept beside it in the ring.
	wire [1:0]  rd_long_w = {rd_word_w[31:29] >= 3'b101, rd_word_w[15:13] >= 3'b101};
	// TRAP (011000) and RETI (011001) both end the window.
	wire [1:0]  rd_reti_w = {rd_word_w[31:27] == 5'b01100, rd_word_w[15:11] == 5'b01100};

	// Fetch window, in halfwords: 6 bytes past the instruction in decode, plus
	// a halfword for each clock decode was held in its line since the last
	// redirect, up to four. Example: bz at byte 4 held 4 clocks reads to byte 18.
	// It is worked as a small offset from dpc, from registered state, and the
	// read issue compares it with lvl_q.
	reg  [2:0]  ext_q;
	reg  [1:0]  ext_line_q;     // low line bits of the window reference
	wire [1:0]  win_ref_w = id_valid_i ? id_pc_i[4:3] : dpc_q[4:3];
	// The reference moves at most one line a clock between redirects, so its
	// low line bits tell a new line.
	wire        ext_new_line_w = (win_ref_w != ext_line_q);
	wire [2:0]  ext_w = ext_new_line_w ? 3'd0 : ext_q;
	// The instruction in decode ends at dpc, one or two halfwords on: the
	// base reach is 1-7 halfwords past dpc.
	wire [1:0]  id_back_w = !id_valid_i ? 2'd0 : id_long_i ? 2'd2 : 2'd1;
	wire [2:0]  win_base_w = 3'd3 + ext_w - {1'b0, id_back_w};
	// Straight-line code also gets the instruction after the next one, so 32-bit
	// instructions run one a clock; past a jump in decode it does not. A word
	// arriving now is taken as a hit: if it misses, this clock's read is dropped.
	wire        head_some_w = (q_count_q != 3'd0) || (rd_valid_q && !rd_stale_q);
	wire [1:0]  head_long_w = (q_count_q != 3'd0) ? q_long_q[h0_w] : rd_long_w;
	wire [1:0]  head_reti_w = (q_count_q != 3'd0) ? q_reti_q[h0_w] : rd_reti_w;
	wire        dpc_long_w = head_some_w && head_long_w[dpc_q[1]];
	// lvl_q within the base reach, or within the instruction after next.
	wire        lvl_neg_w = lvl_q[5];
	wire        run_base_w = lvl_neg_w || ((lvl_q[4:3] == 2'd0) && (lvl_q[2:0] <= win_base_w));
	wire        run_dem_w = !(id_valid_i && id_jump_i) &&
		(lvl_neg_w || ((lvl_q[4:2] == 3'd0) && (lvl_q[1:0] <= (dpc_long_w ? 2'd3 : 2'd2))));
	// RETI and TRAP read their own word and the next one: from when one is next
	// in line until the fetch is redirected, the window ends there.
	reg         reti_q;
	reg  [5:0]  reti_end_q;     // low halfword bits of the end
	wire        reti_next_w = head_some_w && head_reti_w[dpc_q[1]];
	wire [5:0]  reti_end_w = reti_q ? reti_end_q - dpc_q[6:1] : 6'd2;
	wire        reti_ok_w = !(reti_q || reti_next_w) || ($signed(lvl_q) <= $signed(reti_end_w));
	wire        in_win_w = (run_base_w || run_dem_w) && reti_ok_w;
	// Room for one more read: queued words plus the read in flight.
	wire        room_w = (({1'b0, q_count_q} + {3'd0, rd_valid_q}) < 4'd5) && in_win_w;
	wire        cache_mode_w = cache_on_i;
	// No read starts as a clear or transfer begins: its result would be dropped.
	wire        read_issue_w = cache_mode_w && !redirect_i && !fill_q &&
		!clear_i && !clear_q && !hold_i && !pause_i && room_w && !xfer_i && !xfer_q && !restart_hold_i;

	// Fill bus requests: one word cycle at a time.
	// Fill words chain: the next one is asked for as the current one completes.
	wire        fill_done_w = fill_active_q && bus_complete_i;
	wire        fill_last_w = (fill_n_q == 2'd3);
	wire [28:0] fill_next_line_w = fill_word_q ? fill_line_q + 29'd1 : fill_line_q;
	wire [31:0] fill_addr_w = fill_done_w ? {fill_next_line_w, !fill_word_q, 2'b00} :
		{fill_line_q, fill_word_q, 2'b00};
	// Under a savestate pause a fill runs to its end, even in HALT.
	// A fill's first word waits while BR, JR or JAL is in decode and on the clock
	// it redirects, but not for a savestate pause or a CHCW command, which wait
	// for the fill.
	reg         rel_jump_q;     // BR, JR or JAL was in decode last clock
	wire        fill_jump_wait_w = (fill_n_q == 2'd0) && !fill_active_q && !pause_i && !restart_hold_i &&
		((redirect_i && rel_jump_q) || (id_valid_i && id_rel_jump_i));
	wire        fill_req_w = fill_q && (!hold_i || pause_i) && !fill_hold_i && !fill_jump_wait_w &&
		(!fill_active_q || (fill_done_w && !fill_last_w));
	// A read-ahead miss fills too, even as the jump that passes it redirects.
	// A taken conditional branch drops the fill if its first word has not
	// started: a miss seen on the redirect clock, or a fill waiting for the
	// load the branch waited for. The line still takes the new tag.
	wire        fill_drop_w = redirect_i && bcond_load_wait_i && fill_q && (fill_n_q == 2'd0) &&
		!fill_active_q && !(fill_req_w && bus_accept_i);
	wire        miss_drop_w = rd_miss_w && !fill_q && redirect_i && bcond_redirect_i;
	wire        fill_start_w = rd_miss_w && !fill_q && !fill_stop_i && !miss_drop_w;

	// A dump or restore starts once a running fill or clear is done.
	wire        xfer_go_w = xfer_q && !fill_q && !clear_q;
	wire        xfer_done_w = xfer_active_q && xfer_complete_i;
	wire        xfer_line_we_w = xfer_done_w && !xfer_dump_q && !xfer_idx_q[8] && xfer_idx_q[0];
	wire        xfer_tag_we_w = xfer_done_w && !xfer_dump_q && xfer_idx_q[8];
	assign xfer_busy_o = xfer_q;
	assign xfer_bus_o = xfer_go_w;
	// The RAM takes a new index at phi1, before the next word can launch.
	assign xfer_req_o = xfer_go_w && !xfer_active_q;
	assign xfer_addr_o = {xfer_sa_q, 8'd0} + {21'd0, xfer_idx_q, 2'b00};
	assign xfer_write_o = xfer_dump_q;
	// Lines keep the halves of each word swapped; memory order is word 0, word 1.
	assign xfer_wdata_o = xfer_idx_q[8] ? ic_tag_w :
		xfer_idx_q[0] ? {ic_data_w[15:0], ic_data_w[31:16]} : {ic_data_w[47:32], ic_data_w[63:48]};

	// A fill word keeps its line's other word valid only under the same tag.
	wire        fill_same_w = (ic_tag_w[21:0] == fill_line_q[28:7]);
	wire [1:0]  fill_valid_w = (fill_same_w ? ic_tag_w[23:22] : 2'b00) |
		(fill_word_q ? 2'b10 : 2'b01);
	always @* begin
		ic_data_we_v = 1'b0;
		ic_data_wa_v = fill_line_q[6:0];
		ic_data_wd_v = fill_word_q ? {ic_data_w[63:32], bus_rdata_i[15:0], bus_rdata_i[31:16]} :
			{bus_rdata_i[15:0], bus_rdata_i[31:16], ic_data_w[31:0]};
		ic_tag_we_v = 1'b0;
		ic_tag_wa_v = fill_line_q[6:0];
		ic_tag_wd_v = {8'd0, fill_valid_w, fill_line_q[28:7]};
		if (clear_q) begin
			ic_tag_we_v = ce_i && (clear_wait_q == 4'd0);
			ic_tag_wa_v = clear_index_q;
			ic_tag_wd_v = 32'd0;
		end else if (xfer_go_w) begin
			ic_data_we_v = xfer_line_we_w;
			ic_data_wa_v = xfer_idx_q[7:1];
			ic_data_wd_v = {xfer_lo_q[15:0], xfer_lo_q[31:16], bus_rdata_i[15:0], bus_rdata_i[31:16]};
			// Tag memory holds 28 bits.
			ic_tag_we_v = xfer_tag_we_w;
			ic_tag_wa_v = xfer_idx_q[6:0];
			ic_tag_wd_v = {4'd0, bus_rdata_i[27:0]};
		end else if (fill_done_w) begin
			ic_data_we_v = 1'b1;
			ic_tag_we_v = 1'b1;
		end else if (fill_drop_w && !fill_same_w) begin
			ic_tag_we_v = ce_i;
			ic_tag_wd_v = {10'd0, fill_line_q[28:7]};
		end else if (miss_drop_w && !fill_stop_i && (ic_tag_w[21:0] != rd_addr_q[29:8])) begin
			ic_tag_we_v = ce_i;
			ic_tag_wa_v = rd_addr_q[7:1];
			ic_tag_wd_v = {10'd0, rd_addr_q[29:8]};
		end
	end

	NECv810_icache_data_ram u_data (
		.clk_i(clk_i),
		.read_addr_i(rd_index_w),
		.write_addr_i(ic_data_wa_v),
		.write_en_i(ic_data_we_v),
		.write_data_i(ic_data_wd_v),
		.q_o(ic_data_w),
		.savestate_active_i(savestate_active_i && !savestate_addr_i[10]),
		.savestate_addr_i(savestate_addr_i[9:0]),
		.savestate_rden_i(savestate_rden_i && !savestate_addr_i[10]),
		.savestate_wren_i(savestate_wren_i && !savestate_addr_i[10]),
		.savestate_wdata_i(savestate_wdata_i),
		.savestate_rdata_o(ss_data_rdata_w)
	);

	NECv810_icache_tag_ram u_tag (
		.clk_i(clk_i),
		.read_addr_i(rd_index_w),
		.write_addr_i(ic_tag_wa_v),
		.write_en_i(ic_tag_we_v),
		.write_data_i(ic_tag_wd_v),
		.q_o(ic_tag_w),
		.savestate_active_i(savestate_active_i && savestate_addr_i[10]),
		.savestate_addr_i(savestate_addr_i[8:0]),
		.savestate_rden_i(savestate_rden_i && savestate_addr_i[10]),
		.savestate_wren_i(savestate_wren_i && savestate_addr_i[10]),
		.savestate_wdata_i(savestate_wdata_i),
		.savestate_rdata_o(ss_tag_rdata_w)
	);
	assign savestate_rdata_o = savestate_addr_i[10] ? ss_tag_rdata_w : ss_data_rdata_w;

	// ---------------------------------------------------------------
	// Stream path (cache off).
	// ---------------------------------------------------------------
	wire        s_ready_w;
	wire [31:0] s_iw_w;
	wire        s_req_w;
	wire [31:0] s_addr_w;
	wire        s_branch_w;
	wire        s_busy_w;
	wire        stream_on_w = !cache_on_i;

	necv810_fetch_stream u_stream (
		.clk_i(clk_i),
		.reset_i(reset_i),
		.ce_i(ce_i),
		.enable_i(stream_on_w && !hold_i && !pause_i && !restart_hold_i),
		.want_pc_i(redirect_i ? redirect_pc_i[31:1] : dpc_q[31:1]),
		.redirect_i(redirect_i),
		.data_wait_i(data_wait_i),
		.data_ahead_i(data_ahead_i),
		.reti_take_i(take_i && stream_on_w && !fill_q && (s_iw_w[31:27] == 5'b01100)),
		.launch_accept_i(bus_accept_i && stream_on_w && !fill_q),
		.complete_i(bus_complete_i && !fill_active_q),
		.read_data_i(bus_rdata_i),
		.busy_o(s_busy_w),
		.ready_o(s_ready_w),
		.iw_o(s_iw_w),
		.request_o(s_req_w),
		.address_o(s_addr_w),
		.branch_o(s_branch_w)
	);

	// ---------------------------------------------------------------
	// Outputs.
	// ---------------------------------------------------------------
	wire        stream_out_w = stream_on_w && !fill_q;
	assign inst_valid_o = !redirect_i && !clear_q && !xfer_i && !xfer_q &&
		(stream_out_w ? s_ready_w : c_ready_w);
	assign inst_iw_o = stream_out_w ? s_iw_w : c_iw_w;
	assign inst_pc_o = dpc_q;
	assign bus_req_o = fill_q ? fill_req_w : (stream_on_w && s_req_w);
	assign bus_addr_o = fill_q ? fill_addr_w : s_addr_w;
	assign bus_branch_o = fill_q ? (fill_branch_q && !fill_active_q && (fill_n_q == 2'd0)) :
		s_branch_w;
	// A read in flight still counts: its miss can start a fill on the next clock.
	assign busy_o = fill_q || s_busy_w || rd_valid_q;
	assign clear_busy_o = clear_q;

	wire [31:0] take_len_w = (stream_out_w ? (s_iw_w[31:29] >= 3'b101) : c_long_w) ? 32'd4 : 32'd2;
	// Decode moves past its word: the head slot is spent.
	wire        pop_w = take_i && (dpc_q[1] || take_len_w[2]);
	wire        redirect_read_w = cache_mode_w && (!fill_q || fill_drop_w) && !fill_start_w && !clear_i && !clear_q &&
		!xfer_i && !xfer_q && !hold_i && !pause_i && !restart_hold_i;
	wire [31:0] dpc_n_w = redirect_i ? redirect_pc_i : take_i ? dpc_q + take_len_w : dpc_q;
	// Words between the read issued now and decode's next word.
	wire [4:0]  rd_behind_w = dpc_n_w[6:2] - fpc_q[4:0];
	// A missing word is read again, after its fill or when it got none;
	// switching to the cache restarts reads at decode.
	wire [29:0] fpc_n_w = redirect_i ? redirect_pc_i[31:2] + {29'd0, redirect_read_w} :
		rd_miss_w ? rd_addr_q :
		!cache_mode_w ? dpc_q[31:2] :
		read_issue_w ? fpc_q + 30'd1 : fpc_q;

	always @(posedge clk_i or posedge reset_i) begin
		if (reset_i) begin
			dpc_q <= 32'd0;
			q_head_q <= 3'd0;
			q_count_q <= 3'd0;
			lvl_q <= 6'd0;
			rd_stale_q <= 1'b0;
			fpc_q <= 30'd0;
			rd_valid_q <= 1'b0;
			rd_addr_q <= 30'd0;
			redirected_q <= 1'b0;
			seq_tgt_q <= 1'b0;
			clear_odd_q <= 1'b0;
			fill_q <= 1'b0;
			fill_line_q <= 29'd0;
			fill_word_q <= 1'b0;
			fill_n_q <= 2'd0;
			fill_branch_q <= 1'b0;
			fill_active_q <= 1'b0;
			ext_q <= 3'd0;
			ext_line_q <= 2'd0;
			rel_jump_q <= 1'b0;
			reti_q <= 1'b0;
			reti_end_q <= 6'd0;
			clear_index_q <= 7'd0;
			clear_last_q <= 7'd127;
			clear_q <= 1'b0;
			clear_wait_q <= 4'd0;
			xfer_q <= 1'b0;
			xfer_dump_q <= 1'b0;
			xfer_sa_q <= 24'd0;
			xfer_idx_q <= 9'd0;
			xfer_active_q <= 1'b0;
			xfer_lo_q <= 32'd0;
		end else begin
			// Tag clear: a setup wait, then one line every two clocks.
			if (ce_i) begin
				if (clear_q && (clear_wait_q != 4'd0)) begin
					clear_wait_q <= clear_wait_q - 4'd1;
				end else if (clear_q) begin
					clear_odd_q <= !clear_odd_q;
					if (clear_odd_q) begin
						clear_index_q <= clear_index_q + 7'd1;
						if (clear_index_q == clear_last_q) clear_q <= 1'b0;
					end
				end else if (clear_i) begin
					clear_index_q <= clear_first_i;
					clear_last_q <= clear_last_i;
					clear_q <= 1'b1;
					clear_odd_q <= 1'b0;
					clear_wait_q <= CLEAR_SETUP;
				end
			end

			if (ce_i) begin
				rd_valid_q <= 1'b0;

				// Dump or restore, one word at a time.
				if (!xfer_q) begin
					if (xfer_i) begin
						xfer_q <= 1'b1;
						xfer_dump_q <= xfer_dump_i;
						xfer_sa_q <= xfer_sa_i;
						xfer_idx_q <= 9'd0;
					end
				end else if (xfer_done_w) begin
					xfer_active_q <= 1'b0;
					if (!xfer_idx_q[8] && !xfer_idx_q[0]) xfer_lo_q <= bus_rdata_i;
					xfer_idx_q <= xfer_idx_q + 9'd1;
					if (xfer_idx_q == 9'd383) xfer_q <= 1'b0;
				end else if (xfer_req_o && xfer_accept_i) begin
					xfer_active_q <= 1'b1;
				end

				// Fill engine. A fill always runs to its end.
				if (fill_done_w) begin
					fill_active_q <= 1'b0;
					fill_n_q <= fill_n_q + 2'd1;
					fill_word_q <= !fill_word_q;
					if (fill_word_q) fill_line_q <= fill_line_q + 29'd1;
					if (fill_last_w) fill_q <= 1'b0;
				end
				if (fill_req_w && bus_accept_i) fill_active_q <= 1'b1;
				if (fill_drop_w) fill_q <= 1'b0;
				if (fill_start_w) begin
					fill_q <= 1'b1;
					fill_line_q <= rd_addr_q[29:1];
					// A sequencer redirect's target fills from its line's base.
					fill_word_q <= rd_addr_q[0] && !(redirected_q && seq_tgt_q);
					fill_n_q <= 2'd0;
					fill_branch_q <= redirected_q;
				end
				dpc_q <= dpc_n_w;
				rd_stale_q <= !redirect_i && read_issue_w && (rd_behind_w != 5'd0) && !rd_behind_w[4];
				fpc_q <= fpc_n_w;
				lvl_q <= {fpc_n_w[4:0], 1'b0} - dpc_n_w[6:1];

				// Queue: the word read now goes to the tail; decode spends the head.
				if (redirect_i || !cache_mode_w) begin
					q_head_q <= 3'd0;
					q_count_q <= 3'd0;
				end else begin
					if (rd_push_w) begin
						q_data_q[tail_w] <= rd_word_w;
						q_long_q[tail_w] <= rd_long_w;
						q_reti_q[tail_w] <= rd_reti_w;
					end
					if (pop_w) q_head_q <= h1_w;
					q_count_q <= q_count_q + {2'd0, rd_push_w} - {2'd0, pop_w && (q_count_q != 3'd0 || rd_push_w)};
				end

				ext_line_q <= win_ref_w;
				rel_jump_q <= id_valid_i && id_rel_jump_i && !pause_i;
				if (redirect_i) reti_q <= 1'b0;
				else if (take_i && reti_next_w && !reti_q) begin
					reti_q <= 1'b1;
					reti_end_q <= dpc_q[6:1] + 6'd2;
				end
				if (redirect_i) begin
					ext_q <= 3'd0;
					ext_line_q <= redirect_pc_i[4:3];
				end else if (id_hold_i && (ext_w != 3'd4)) ext_q <= ext_w + 3'd1;
				else ext_q <= ext_w;
				if (redirect_i) begin
					redirected_q <= 1'b1;
					seq_tgt_q <= redirect_seq_i;
					if (redirect_read_w) begin
						rd_valid_q <= 1'b1;
						rd_addr_q <= redirect_pc_i[31:2];
					end
				end else begin
					if (rd_hit_w) redirected_q <= 1'b0;
					if (read_issue_w) begin
						rd_valid_q <= 1'b1;
						rd_addr_q <= fpc_q;
					end
				end
			end
		end
	end
endmodule
