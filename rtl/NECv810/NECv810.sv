// Copyright (c) 2026 Jamie Blanks

// NEC V810 as a five-stage load/store pipeline.
//
//   fetch unit -> ID -> EX -> MEM -> WB
//                 |      |      |      +- one commit point: GPRs at phi1, PSW, retire
//                 |      |      +- loads wait here for the bus; stores stage here or wait
//                 |      +- ALU, redirect
//                 +- decode, Bcond judged against PSW, register read (address
//                    registered at phi1), effective address, next PC, interlock
//
// Timing comes from stage occupancy. ALU results forward to the next EX, load
// data from the end of MEM, flags from WB (a clock later into a Bcond while
// the lag is armed). Address bases and JMP targets are read from the register
// file, so they wait for the producer's commit. The bus engine owns the pins,
// beat splitting and the store staging register; data requests beat fetch
// requests at each cycle boundary.
module NECv810
#(
	parameter [31:0] RESET_PC  = 32'hffff_fff0,
	parameter [31:0] RESET_PSW = 32'h0000_8000,
	// The parameters are the differences between the V810 parts.
	parameter        IO_MRQ    = 0,            // 1: MRQ inactive on IN/OUT cycles.
	parameter        VB_EXT    = 1             // 1: NVC: extras (CLI, SEI, XB, XH, REV, MPYHW, PIR 5346) and its load timing.
)
(
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        ce_i,
	input  wire        phi1_i,
	input  wire        ready_i,
	input  wire [31:0] din_i,
	input  wire        hldrq_i,
	input  wire        icheen_i,
	input  wire        siz16b_i,
	input  wire        szrq_i,
	input  wire        irq_valid_i,
	input  wire [3:0]  irq_level_i,
	input  wire        nmi_i,
	input  wire        savestate_pause_req_i,
	output wire        savestate_pause_ready_o,
	input  wire        savestate_restore_i,
	input  wire [3:0]  savestate_state_addr_i,
	input  wire [63:0] savestate_state_wdata_i,
	input  wire        savestate_state_wren_i,
	output reg  [63:0] savestate_state_rdata_o,
	input  wire        savestate_mem_active_i,
	input  wire [10:0] savestate_mem_addr_i,
	input  wire        savestate_mem_rden_i,
	input  wire        savestate_mem_wren_i,
	input  wire [7:0]  savestate_mem_wdata_i,
	output reg  [7:0]  savestate_mem_rdata_o,
	output wire [31:1] a_o,
	output wire        a_oe_o,
	output wire [31:0] dout_o,
	output wire        dout_oe_o,
	output wire [3:0]  dout_lane_oe_o,
	output wire [3:0]  be_o,
	output wire [3:0]  be_oe_o,
	output wire [1:0]  st_o,
	output wire        st_oe_o,
	output wire        da_o,
	output wire        da_oe_o,
	output wire        mrq_o,
	output wire        mrq_oe_o,
	output wire        rw_o,
	output wire        rw_oe_o,
	output wire        bcyst_o,
	output wire        bcyst_oe_o,
	output wire        adrs_err_n_o,
	output wire        cycle_tag_o,
	output wire        bus_t1_o,
	output wire        block_o,
	output wire        hldak_o,
	input  wire [4:0]  dbg_reg_addr_i,
	output wire [31:0] dbg_reg_data_o,
	output wire [31:0] dbg_pc_o,
	output wire [31:0] dbg_psw_o,
	output reg         trace_valid_o,
	output reg  [31:0] trace_pc_o,
	output reg  [15:0] trace_insn_o,
	output wire        halted_o,
	output reg         illegal_o
);
	`include "necv810_defs.svh"

	wire cpu_reset_w = reset_i || savestate_restore_i;

	// What an instruction does at its commit in WB, beyond GPRs and flags.
	localparam [2:0] WK_NONE = 3'd0;
	localparam [2:0] WK_EXC  = 3'd1;
	localparam [2:0] WK_RETI = 3'd3;
	localparam [2:0] WK_HALT = 3'd4;

	reg        siz16b_q;
	reg        exc_q;            // exception entry on this CE
	reg [15:0] exc_code_q;
	reg [31:0] exc_rpc_q;
	reg [31:0] exc_pc_q;         // the trapping instruction's own PC
	// System bus cycles: the HALT acknowledge, and the fatal-exception dump
	// of three words to 0, 4 and 8. Both run on a drained pipe.
	localparam [2:0] SB_IDLE = 3'd0;
	localparam [2:0] SB_HALT = 3'd1;
	localparam [2:0] SB_HALT_WAIT = 3'd2;
	localparam [2:0] SB_FATAL = 3'd3;
	localparam [2:0] SB_FATAL_WAIT = 3'd4;
	reg [2:0]  sb_q;
	reg [1:0]  sb_index_q;
	reg [31:0] sb_addr_q;
	reg [15:0] sb_data_q;
	reg        start_q;          // fetch from arch_pc at the first CE (reset, restore, resume)
	reg        ss_hold_q;        // drained and paused for a savestate
	// Lag: set as a load, store, shift, SETF, LDSR, STSR, MUL/DIV or EXT op
	// leaves EX. A flag writer leaving EX while it is set (or setting it) is
	// late: a Bcond on its flags waits one clock more.
	reg        lag_q;
	reg        mem_late_q;
	reg        wb_late_q;
	// The instruction behind a load waits in EX until the load's last T2 and
	// one clock more, while the loaded data takes EX, unless it is a load other
	// than IN (its bus cycle starts at once) or a taken transfer (it redirects fetch).
	reg        load_tail_q;
	// A load in ID behind a long EX op, or behind such a load, reads ahead:
	// its address goes into the EA unit, its bus cycle starts from ID and the
	// data waits in rd_data_q. In EX it then takes one clock and goes on as an
	// ALU result.
	reg        rd_ea_q;          // the load in ID has its address in the EA unit
	reg        rd_go_q;          // a read-ahead has started
	reg        rd_ex_q;          // its load is in EX
	reg        rd_done_q;        // its data is in
	reg [1:0]  rd_size_q;
	reg        rd_sext_q;
	// Keep the accepted format while the next load prepares its address.
	reg [1:0]  rd_read_size_q;
	reg        rd_read_sext_q;
	reg [31:0] rd_data_q;
	reg        reti_tgt_q;       // EX holds RETI's return target, first clock
	reg        reti_ph_q;        // RETI waiting on the drain: the clock between checks
	reg        ex_redir_q;       // the taken transfer in EX has redirected fetch already
	reg        redir_gap_q;      // the clock after such a transfer leaves EX: nothing issues
	reg        tgt_pend_q;       // a taken transfer's target has not issued yet
	reg        ex_tgt_q;         // EX holds a taken transfer's target
	reg        mem_tgt_q;        // MEM holds a taken transfer's target
	reg        halt_ph_q;        // HALT: the clock between interrupt checks
	reg        wake_gap_q;       // the clock after a HALT wake: fetch still held
	reg        data_busy_q;      // data_busy_w last clock
	// A load result reaches an address base or JMP target two clocks after
	// it reaches the ALU.
	reg [4:0]  ld_base_dest_q;
	reg [1:0]  ld_base_wait_q;
	reg [31:0] arch_pc_q;        // next instruction to retire

	// ---------------------------------------------------------------
	// Stage registers.
	// ---------------------------------------------------------------
	// ID
	reg        id_v_q;
	reg [31:0] id_pc_q;
	reg [31:0] id_iw_q;
	// EX
	reg        ex_v_q;
	reg [31:0] ex_pc_q;
	reg [31:16] ex_iw_q;
	reg [31:0] ex_seq_pc_q;       // the next instruction's PC, from the PC incrementer
	// The microword, immediate and condition field are decoded in ID and held for EX.
	reg [31:0] ex_uc_q;
	reg [31:0] ex_imm_q;
	reg        ex_cc_on_q;        // SETF
	reg [3:0]  ex_cc_q;
	reg        ex_taken_q;        // a Bcond the decoder judged taken
	reg [31:0] ex_r1_q;           // reg1 value
	reg [31:0] ex_r2_q;           // reg2 value
	reg [5:0]  ex_cnt_q;          // EX clocks left, 1 on the last
	reg        ex_first_q;
	reg [31:0] ex_r30_q;          // CAXI's new value
	// MEM
	reg        mem_v_q;
	reg [31:0] mem_pc_q;
	reg [31:16] mem_iw_q;
	reg        mem_load_q;
	reg        mem_launched_q;
	reg        drop_rd_q;         // a dropped load's read is still on the bus
	reg        mem_store_q;       // ST or OUT
	reg [31:0] mem_wdata_q;
	reg [31:0] mem_addr_q;
	reg [1:0]  mem_size_q;
	reg        mem_sext_q;
	reg        mem_io_q;
	reg        mem_addr_err_q;
	reg        mem_wr_q;          // writes a GPR
	reg [4:0]  mem_dest_q;
	reg [31:0] mem_data_q;
	reg        mem_wr2_q;         // writes r30 as well
	reg [31:0] mem_data2_q;
	reg [9:0]  mem_flags_q;
	reg [9:0]  mem_fmask_q;
	reg [31:0] mem_next_pc_q;
	reg [2:0]  mem_wk_q;
	reg [15:0] mem_code_q;        // exception code
	reg [31:0] mem_rpc_q;         // exception restore PC
	reg        mem_ser_q;
	// WB
	reg        wb_v_q;
	reg [31:0] wb_pc_q;
	reg        wb_wr_q;
	reg [4:0]  wb_dest_q;
	reg        wb_wr2_q;
	reg [9:0]  wb_flags_q;
	reg [9:0]  wb_fmask_q;
	reg [31:0] wb_next_pc_q;
	reg [2:0]  wb_wk_q;
	reg [15:0] wb_code_q;
	reg [31:0] wb_rpc_q;
	reg        wb_ser_q;

	// ---------------------------------------------------------------
	// System control: PSW, system registers, exceptions, interrupts.
	// ---------------------------------------------------------------
	wire [31:0] psw_w;
	wire [31:0] sr_value_w;
	wire [31:0] reti_pc_w;
	wire [31:0] exc_handler_w;
	wire        exc_fatal_w;
	wire        async_pending_w;
	wire        addr_trap_w;
	wire        irq_wait_w;       // an accepted IRQ the system unit holds off: stop issue
	wire [15:0] async_code_w;
	wire        halted_w;
	wire        cache_enable_w;
	wire        cache_pending_w;
	wire        cache_cmd_valid_w;
	wire        chcw_cmd_w;
	wire        exc_take_w;
	wire [15:0] exc_code_w;
	wire [31:0] exc_rpc_w;
	wire        irq_take_w;
	// Savestate memory window: GPRs 0-127, cache data 128-1151, tags 1152-1663.
	wire        ss_gpr_w = savestate_mem_active_i && (savestate_mem_addr_i < 11'd128);
	wire        ss_icd_w = savestate_mem_active_i && (savestate_mem_addr_i >= 11'd128) &&
		(savestate_mem_addr_i < 11'd1152);
	wire        ss_ict_w = savestate_mem_active_i && (savestate_mem_addr_i >= 11'd1152);
	wire [10:0] ss_icd_addr_w = savestate_mem_addr_i - 11'd128;
	wire [10:0] ss_ict_addr_w = savestate_mem_addr_i - 11'd1152;
	wire [7:0]  ss_gpr_rdata_w;
	wire [63:0] ss_sys_word_w;
	wire        fatal_w;
	wire [31:0] fatal_word_w;
	wire        sb_busy_w = (sb_q != SB_IDLE);
	// CHCW dump and restore words from the fetch unit share this port.
	wire        f_xfer_busy_w, f_xfer_bus_w, f_xfer_req_w, f_xfer_write_w;
	wire [31:0] f_xfer_addr_w, f_xfer_wdata_w;
	wire        sb_req_w = ((sb_q == SB_HALT) || (sb_q == SB_FATAL) || f_xfer_req_w) &&
		!eng_active_w && !eng_store_pending_w;

	// ---------------------------------------------------------------
	// Fetch unit.
	// ---------------------------------------------------------------
	wire        f_valid_w;
	wire [31:0] f_pc_w;
	wire [31:0] f_iw_w;
	wire        f_req_w;
	wire [31:0] f_addr_w;
	wire        f_branch_w;
	wire        f_busy_w;
	wire        f_clear_busy_w;
	wire        redirect_w;
	wire [31:0] redirect_pc_w;
	wire        id_load_w;
	wire        data_wait_w;
	wire        data_ahead_w;
	wire        f_accept_w;
	wire        eng_complete_w;
	wire        eng_first_beat_w;
	wire [31:0] eng_rdata_w;
	wire        eng_active_fetch_w;
	wire        eng_split_pending_w;
	wire [7:0]  f_ss_rdata_w;
	wire [31:0] cache_sa_w;

	wire        f_id_hold_w;
	wire        f_id_jump_w;
	wire        f_id_rel_jump_w;
	wire        f_fill_stop_w;
	// A taken conditional branch waits in EX behind a load in MEM.
	wire        f_bcond_load_wait_w;
	// A taken conditional branch, not BR, redirects the fetch this clock.
	wire        f_bcond_redirect_w;
	// The bus is not free for a cache fill while a store waits or a data cycle
	// runs, nor on the clock after.
	wire        data_busy_w;

	necv810_fetch u_fetch (
		.clk_i(clk_i),
		.reset_i(cpu_reset_w),
		.ce_i(ce_i),
		.redirect_i(redirect_w),
		.redirect_pc_i(redirect_pc_w),
		.redirect_seq_i(exc_take_w || wb_redirect_w),
		.take_i(id_load_w),
		.inst_valid_o(f_valid_w),
		.inst_pc_o(f_pc_w),
		.inst_iw_o(f_iw_w),
		.hold_i(start_q || halted_w || wake_gap_q || sb_busy_w || fatal_w),
		.pause_i(savestate_pause_req_i || ss_hold_q),
		.fill_hold_i(data_busy_w || data_busy_q),
		.bcond_load_wait_i(f_bcond_load_wait_w),
		.bcond_redirect_i(f_bcond_redirect_w),
		.id_valid_i(id_v_q),
		.id_pc_i(id_pc_q),
		.id_hold_i(f_id_hold_w),
		.id_jump_i(f_id_jump_w),
		.id_rel_jump_i(f_id_rel_jump_w),
		.id_long_i(id_iw_q[31:29] >= 3'b101),
		// The fetch restarts after an enabling LDSR CHCW through the cache.
		.cache_on_i((cache_enable_w || cache_pending_w) && icheen_i),
		.restart_hold_i((ex_v_q && (ex_kind_w == MK_LDSR) && chcw_cmd_w) || cache_cmd_valid_w ||
			f_clear_busy_w || f_xfer_busy_w),
		.fill_stop_i(f_fill_stop_w),
		.data_wait_i(data_wait_w),
		.data_ahead_i(data_ahead_w),
		.bus_req_o(f_req_w),
		.bus_addr_o(f_addr_w),
		.bus_branch_o(f_branch_w),
		.bus_accept_i(f_accept_w),
		.bus_complete_i(eng_complete_w && eng_active_fetch_w),
		.bus_rdata_i(eng_rdata_w),
		.busy_o(f_busy_w),
		.clear_i(cache_cmd_valid_w && (cache_cmd_w == 2'd1)),
		.clear_first_i(cache_clear_first_w),
		.clear_last_i(cache_clear_last_w),
		.clear_busy_o(f_clear_busy_w),
		.xfer_i(cache_cmd_valid_w && cache_cmd_w[1]),
		.xfer_dump_i(!cache_cmd_w[0]),
		.xfer_sa_i(cache_sa_w[31:8]),
		.xfer_busy_o(f_xfer_busy_w),
		.xfer_bus_o(f_xfer_bus_w),
		.xfer_req_o(f_xfer_req_w),
		.xfer_addr_o(f_xfer_addr_w),
		.xfer_write_o(f_xfer_write_w),
		.xfer_wdata_o(f_xfer_wdata_w),
		.xfer_accept_i(eng_launch_accept_w && f_xfer_req_w),
		.xfer_complete_i(eng_complete_w && !eng_active_fetch_w && !eng_active_posted_w),
		.savestate_active_i(ss_icd_w || ss_ict_w),
		.savestate_addr_i(ss_ict_w ? {1'b1, ss_ict_addr_w[9:0]} : {1'b0, ss_icd_addr_w[9:0]}),
		.savestate_rden_i(savestate_mem_rden_i),
		.savestate_wren_i(savestate_mem_wren_i),
		.savestate_wdata_i(savestate_mem_wdata_i),
		.savestate_rdata_o(f_ss_rdata_w)
	);

	// ---------------------------------------------------------------
	// Decode (ID).
	// ---------------------------------------------------------------
	wire [5:0]  id_ue_w = decode_uentry_fn(id_iw_q);
	wire [31:0] id_uc_w = ucode_fn(id_ue_w);
	wire [31:0] id_imm_w = issue_immediate_fn(id_ue_w, id_iw_q);
	wire        id_is_bcond_w = (id_ue_w == UADDR_BCOND);
	wire        id_cc_on_w = (id_ue_w == UADDR_SETF);
	wire [3:0]  id_cc_w = id_iw_q[19:16];
	wire [4:0]  id_reg1_w = id_iw_q[20:16];
	wire [4:0]  id_reg2_w = id_iw_q[25:21];
	wire [3:0]  id_kind_w = id_uc_w[3:0];
	wire [1:0]  id_srca_w = id_uc_w[9:8];
	wire [3:0]  id_srcb_w = id_uc_w[13:10];
	wire        id_is_mem_w = (id_kind_w == MK_MEM);
	wire        id_is_store_w = id_is_mem_w && id_uc_w[19];
	wire        id_is_jmp_w = (id_ue_w == UADDR_JMP);
	wire        id_is_long_w = (id_kind_w == MK_LONG);
	// Which registers this instruction reads, and how.
	wire        id_use_r1_alu_w = (((id_srca_w == SA_REG1) || (id_srcb_w == SB_REG1)) &&
		!id_is_mem_w && !id_is_jmp_w) || id_is_long_w;
	wire        id_use_r1_base_w = id_is_mem_w || id_is_jmp_w || (id_kind_w == MK_CAXI);
	wire        id_use_r2_w = (id_srca_w == SA_REG2) || id_is_store_w || id_is_long_w ||
		(id_kind_w == MK_LDSR) || (id_kind_w == MK_CAXI);
	// Only a Bcond waits for flags; BR (always) and NOP (never) read none.
	wire        id_use_flags_w = (id_ue_w == UADDR_BCOND) &&
		(id_iw_q[28:25] != 4'd5) && (id_iw_q[28:25] != 4'd13);
	// The decoder judges a Bcond against PSW merged with the flags WB writes
	// this clock, as the chip writes PSW in phi1 and decodes in phi2.
	wire [9:0]  id_flags_w = (wb_v_q && (wb_fmask_q != 10'd0)) ?
		((psw_w[9:0] & ~wb_fmask_q) | (wb_flags_q & wb_fmask_q)) : psw_w[9:0];
	wire        id_taken_w = id_is_bcond_w && branch_taken_fn(id_iw_q[28:25], {22'd0, id_flags_w});
	// CAXI waits for older loads, stores and r30 writes. TRAP, RETI, HALT and
	// illegal opcodes wait in EX instead, before they act. A bit string waits
	// only for writers of r26-r30 still in EX or MEM; the mirror catches up as
	// they leave WB.
	wire        id_ser_w = (id_kind_w == MK_CAXI);

	wire [31:0] rf_a_w;
	wire [31:0] rf_b_w;

	// The exception entry is a micro-routine: interrupts start it directly,
	// TRAP after its own two clocks. TRAP and RETI take 13 and 10 with the
	// refetch.
	localparam [5:0] TRAP_CYCLES  = 6'd2;
	localparam [5:0] RETI_CYCLES  = 6'd8;

	// LDSR and STSR EX clocks by register, as measured on the console. A clear
	// written to CHCW adds its own time in the fetch unit after these 16.
	function [5:0] sr_cycles_fn;
		input       ldsr;
		input [4:0] id;
		begin
			case (id)
				SYS_EIPC,
				SYS_FEPC,
				SYS_TKCW:  sr_cycles_fn = 6'd1;
				SYS_EIPSW,
				SYS_FEPSW: sr_cycles_fn = ldsr ? 6'd2 : 6'd1;
				SYS_ECR:   sr_cycles_fn = ldsr ? 6'd1 : 6'd3;
				SYS_PSW:   sr_cycles_fn = ldsr ? 6'd10 : 6'd2;
				SYS_PIR:   sr_cycles_fn = ldsr ? 6'd1 : 6'd2;
				SYS_CHCW:  sr_cycles_fn = ldsr ? 6'd16 : 6'd1;
				SYS_ADTRE: sr_cycles_fn = 6'd8;
				5'd29:     sr_cycles_fn = ldsr ? 6'd8 : 6'd1;
				5'd30:     sr_cycles_fn = ldsr ? 6'd12 : 6'd1;
				5'd31:     sr_cycles_fn = ldsr ? 6'd7 : 6'd11;
				default:   sr_cycles_fn = 6'd7;    // 8-23 and 26-28
			endcase
		end
	endfunction

	// EX occupancy in clocks, from the instruction tables.
	function [5:0] ex_cycles_fn;
		input [5:0]  ue;
		input [31:0] iw;
		input [31:0] r1;
		reg   [31:0] uc;
		begin
			uc = ucode_fn(ue);
			case (uc[3:0])
				MK_LDSR: ex_cycles_fn = sr_cycles_fn(1'b1, iw[20:16]);
				MK_STSR: ex_cycles_fn = sr_cycles_fn(1'b0, iw[20:16]);
				MK_TRAP: ex_cycles_fn = TRAP_CYCLES;
				MK_RETI: ex_cycles_fn = RETI_CYCLES;
				MK_LONG: begin
					case (uc[7:4])
						// DIV by 1 skips the divide steps.
						LONG_DIV, LONG_DIVU:
							ex_cycles_fn = (r1 == 32'd0) ? 6'd1 :
								((uc[7:4] == LONG_DIV) && (r1 == 32'd1)) ? 6'd11 :
								long_issue_cycles_fn(uc[7:4]);
						LONG_MUL, LONG_MULU: ex_cycles_fn = long_issue_cycles_fn(uc[7:4]);
						// XH takes one clock; micro-routines last until their END word.
						default: ex_cycles_fn = 6'd1;
					endcase
				end
				default: ex_cycles_fn = 6'd1;
			endcase
		end
	endfunction

	// ---------------------------------------------------------------
	// Execute (EX).
	// ---------------------------------------------------------------
	wire [31:0] ex_uc_w = ex_uc_q;
	wire [3:0]  ex_kind_w = ex_uc_w[3:0];
	wire [3:0]  ex_alu_w = ex_uc_w[7:4];
	wire [1:0]  ex_srca_w = ex_uc_w[9:8];
	wire [3:0]  ex_srcb_w = ex_uc_w[13:10];
	wire        ex_wgpr_w = ex_uc_w[14];
	wire        ex_wpsw_w = ex_uc_w[15];
	wire        ex_link_w = ex_uc_w[16];
	wire        ex_pc_res_w = ex_uc_w[17];
	wire        ex_pc_bcond_w = ex_uc_w[18];
	wire        ex_mem_wr_w = ex_uc_w[19];
	wire        ex_mem_io_w = ex_uc_w[20];
	wire [1:0]  ex_mem_size_w = ex_uc_w[22:21];
	wire        ex_mem_sext_w = ex_uc_w[23];
	wire        ex_abortable_w = ex_uc_w[31];
	wire        ex_is_mem_w = (ex_kind_w == MK_MEM);
	wire        ex_is_load_w = ex_is_mem_w && !ex_mem_wr_w;
	wire        ex_rd_w = ex_v_q && ex_is_load_w && rd_ex_q;
	wire        ex_is_store_w = ex_is_mem_w && ex_mem_wr_w;
	wire        ex_is_long_w = (ex_kind_w == MK_LONG);
	wire [4:0]  ex_reg1_w = ex_iw_q[20:16];
	wire [4:0]  ex_reg2_w = ex_iw_q[25:21];
	wire        ex_is_caxi_w = (ex_kind_w == MK_CAXI);
	wire        ex_is_bs_w = (ex_kind_w == MK_BITSTR);
	wire        us_routine_w, us_end_w, us_psw_we_w;
	wire        ent_busy_w;       // the exception entry routine runs
	wire        us_exc_w;         // its last step
	wire [31:0] us_res_w;
	wire        us_fp_we_w, us_fp_exc_w;
	wire [9:0]  us_fp_flags_w, us_fp_fmask_w;
	wire [15:0] us_fp_code_w;
	wire [3:0]  us_flags_w;
	wire        ex_last_w = (ex_cnt_q <= 6'd1) && (!us_routine_w || us_end_w);

	wire [31:0] ex_imm_w = ex_imm_q;
	reg  [31:0] ex_a_v;
	reg  [31:0] ex_b_v;
	always @* begin
		case (ex_srca_w)
			SA_REG1: ex_a_v = ex_r1_q;
			SA_REG2: ex_a_v = ex_r2_q;
			default: ex_a_v = 32'd0;
		endcase
		case (ex_srcb_w)
			SB_REG1: ex_b_v = ex_r1_q;
			SB_ZERO: ex_b_v = 32'd0;
			default: ex_b_v = ex_imm_w;
		endcase
	end
	wire [33:0] ex_addsub_w = alu_addsub_fn(ex_a_v, ex_b_v, ex_alu_w == ALU_SUB);
	wire [32:0] ex_shift_w = barrel_shift_fn(ex_a_v, ex_b_v[4:0], ex_alu_w == ALU_SHL,
		ex_alu_w == ALU_SAR);
	// SETF and STSR PSW in EX see the flags still in MEM and WB.
	wire [9:0]  ex_flags_w = (mem_v_q && (mem_fmask_q != 10'd0)) ?
		((id_flags_w & ~mem_fmask_q) | (mem_flags_q & mem_fmask_q)) : id_flags_w;
	wire        ex_cond_w = ex_cc_on_q && branch_taken_fn(ex_cc_q, {psw_w[31:10], ex_flags_w});

	// The integer engine: started in the first EX clock, finished in the last.
	wire        ex_int_kind_w = ex_is_long_w && ((ex_alu_w == LONG_MUL) ||
		(ex_alu_w == LONG_MULU) || (ex_alu_w == LONG_DIV) || (ex_alu_w == LONG_DIVU));
	wire        ex_div_w = ex_is_long_w && ((ex_alu_w == LONG_DIV) || (ex_alu_w == LONG_DIVU));
	wire        ex_div0_w = ex_div_w && (ex_r1_q == 32'd0);
	wire        ex_flush_w;
	wire        ex_kill_w;
	// An interrupt abandons an abortable routine in EX; it runs again after RETI.
	wire        ex_abort_w = ex_v_q && ex_abortable_w;
	wire        int_busy_w, int_done_w, int_zero_w, int_sign_w, int_ovf_w;
	wire [31:0] int_hi_w, int_lo_w;

	necv810_integer_engine u_int (
		.clk_i(clk_i),
		.reset_i(cpu_reset_w),
		.clk_en_i(ce_i),
		.phi1_i(phi1_i),
		.start_i(ex_v_q && ex_first_q && ex_int_kind_w && !ex_div0_w && !int_busy_w),
		.kill_i(ex_kill_w && ex_int_kind_w),
		.finish_i(ex_v_q && ex_last_w && ex_int_kind_w && !ex_div0_w && !ex_kill_w),
		.kind_i(ex_alu_w),
		.lhs_i(ex_r2_q),
		.rhs_i(ex_r1_q),
		.start_accept_o(),
		.busy_o(int_busy_w),
		.done_o(int_done_w),
		.step_o(),
		.result_hi_o(int_hi_w),
		.result_lo_o(int_lo_w),
		.result_zero_o(int_zero_w),
		.result_sign_o(int_sign_w),
		.result_overflow_o(int_ovf_w)
	);

	// Results and commit actions of the instruction leaving EX.
	reg  [31:0] ex_res_v;
	reg         ex_wr_v;
	reg  [4:0]  ex_dest_v;
	reg         ex_wr2_v;
	reg  [9:0]  ex_flags_v;          // PSW bits 9:0; ALU uses {CY, OV, S, Z}
	reg  [9:0]  ex_fmask_v;
	reg  [2:0]  ex_wk_v;
	reg  [15:0] ex_code_v;
	reg  [31:0] ex_rpc_v;
	always @* begin
		ex_res_v = 32'd0;
		ex_wr_v = ex_wgpr_w || ex_link_w || ex_is_load_w;
		ex_dest_v = ex_link_w ? 5'd31 : ex_reg2_w;
		ex_wr2_v = 1'b0;
		ex_flags_v = 10'd0;
		ex_fmask_v = 10'd0;
		ex_wk_v = WK_NONE;
		ex_code_v = 16'd0;
		ex_rpc_v = {ex_pc_q[31:1], 1'b0};
		case (ex_kind_w)
			MK_ALU, MK_BRANCH, MK_MEM: begin
				case (ex_alu_w)
					ALU_PASS_A: ex_res_v = ex_a_v;
					ALU_PASS_B: ex_res_v = ex_b_v;
					ALU_ADD, ALU_SUB: begin
						ex_res_v = ex_addsub_w[31:0];
						ex_flags_v[3:0] = {ex_addsub_w[32], ex_addsub_w[33], ex_addsub_w[31],
							ex_addsub_w[31:0] == 32'd0};
						ex_fmask_v[3:0] = 4'b1111;
					end
					ALU_OR, ALU_AND, ALU_XOR, ALU_NOT: begin
						case (ex_alu_w)
							ALU_OR:  ex_res_v = ex_a_v | ex_b_v;
							ALU_AND: ex_res_v = ex_a_v & ex_b_v;
							ALU_XOR: ex_res_v = ex_a_v ^ ex_b_v;
							default: ex_res_v = ~ex_a_v;
						endcase
						ex_flags_v[3:0] = {1'b0, 1'b0, ex_res_v[31], ex_res_v == 32'd0};
						ex_fmask_v[3:0] = 4'b0111;
					end
					ALU_SHL, ALU_SHR, ALU_SAR: begin
						ex_res_v = ex_shift_w[31:0];
						ex_flags_v[3:0] = {ex_shift_w[32], 1'b0, ex_shift_w[31],
							ex_shift_w[31:0] == 32'd0};
						ex_fmask_v[3:0] = 4'b1111;
					end
					ALU_SETF: ex_res_v = {31'd0, ex_cond_w};
					default: ex_res_v = 32'd0;
				endcase
				if (!ex_wpsw_w) ex_fmask_v = 10'd0;
				if (ex_link_w) ex_res_v = ex_seq_pc_q;
			end
			MK_LONG: begin
				ex_wr_v = 1'b0;
				case (ex_alu_w)
					LONG_CLI, LONG_SEI: ;
					// reg2 is busy from the first clock; the routine's result is ready at END.
					LONG_XB, LONG_REV, LONG_MPYHW: begin
						ex_wr_v = 1'b1;
						ex_res_v = us_res_w;
					end
					LONG_XH: begin
						ex_wr_v = 1'b1;
						ex_res_v = {ex_r2_q[15:0], ex_r2_q[31:16]};
					end
					LONG_MUL, LONG_MULU, LONG_DIV, LONG_DIVU: begin
						if (ex_div0_w) begin
							ex_wk_v = WK_EXC;
							ex_code_v = EXC_CODE_ZERODIV;
						end else begin
							ex_wr_v = 1'b1;
							ex_res_v = int_lo_w;
							// With reg2 = r30 the low word or quotient is kept.
							ex_wr2_v = (ex_reg2_w != 5'd30);
							ex_flags_v[2:0] = {int_ovf_w, int_sign_w, int_zero_w};
							ex_fmask_v[2:0] = 3'b111;
						end
					end
					default: begin
						// Floating point: result, PSW bits, and maybe a trap.
						ex_wr_v = us_fp_we_w;
						ex_res_v = us_res_w;
						ex_flags_v = us_fp_flags_w;
						ex_fmask_v = us_fp_fmask_w;
						if (us_fp_exc_w) begin
							ex_wk_v = WK_EXC;
							ex_code_v = us_fp_code_w;
						end
					end
				endcase
			end
			MK_CAXI: begin
				ex_wr_v = 1'b1;
				ex_res_v = us_res_w;
				// Flags as for CMP of reg2 against the loaded word.
				ex_flags_v[3:0] = us_flags_w;
				ex_fmask_v[3:0] = 4'b1111;
			end
			MK_BITSTR: ex_wr_v = 1'b0;
			MK_LDSR: ex_wr_v = 1'b0;
			MK_STSR: begin
				ex_wr_v = 1'b1;
				ex_res_v = (ex_reg1_w == SYS_PSW) ? {sr_value_w[31:10], ex_flags_w} : sr_value_w;
			end
			MK_TRAP: begin
				ex_wr_v = 1'b0;
				ex_wk_v = WK_EXC;
				ex_code_v = trap_code_w;
				ex_rpc_v = trap_rpc_w;
			end
			MK_RETI: begin
				ex_wr_v = 1'b0;
				ex_wk_v = WK_RETI;
			end
			MK_HALT: begin
				ex_wr_v = 1'b0;
				ex_wk_v = WK_HALT;
			end
			MK_NOP: ex_wr_v = 1'b0;
			default: begin
				ex_wr_v = 1'b0;
				ex_wk_v = WK_EXC;
				ex_code_v = EXC_CODE_ILLEGAL;
			end
		endcase
		if (ex_is_load_w) ex_res_v = rd_data_q;
	end

	// Control transfer: a Bcond comes judged from ID; it and the jumps redirect from EX.
	wire        ex_bcond_taken_w = ex_pc_bcond_w && ex_taken_q;
	wire        ex_jump_w = ex_pc_res_w;
	wire        ex_taken_w = ex_bcond_taken_w || ex_jump_w;
	// Targets and operand addresses arrive registered from the EA unit.
	wire [31:0] ex_ea_w;
	wire [31:0] ex_target_w = {ex_ea_w[31:1], 1'b0};
	wire [31:0] ex_mem_addr_w = align_addr_fn(ex_ea_w, ex_mem_size_w);
	wire        ex_addr_err_w = address_error_fn(ex_ea_w, ex_mem_size_w);

	// EX destination for forwarding (valid on the last EX clock).
	wire        ex_writes_w = ex_v_q && ex_wr_v;
	wire        ex_ready_w = ex_last_w && (!ex_is_load_w || (rd_ex_q && rd_done_q)) &&
		(!ex_int_kind_w || int_done_w);

	// ---------------------------------------------------------------
	// Bus engine and request arbitration.
	// ---------------------------------------------------------------
	wire        eng_launch_accept_w;
	wire        eng_posted_ready_w;
	wire        eng_active_w;
	wire        eng_active_posted_w;
	wire        eng_store_pending_w;
	wire        eng_store_due_w;
	wire [31:1] eng_a_w;
	wire [3:0]  eng_be_w;
	wire [1:0]  eng_st_w;
	wire        eng_da_w;
	wire        eng_mrq_w;
	wire        eng_rw_w;
	wire        eng_bcyst_w;
	wire        eng_adrs_err_n_w;
	wire        eng_dout_oe_w;
	wire        eng_bus_t1_w;
	wire        eng_hldak_w;
	reg         hold_data_q;

	// MEM issues a load at the edge it is entered, or keeps asking.
	wire        ex_adv_w;
	wire        mem_retry_w = mem_v_q && mem_load_q && !mem_launched_q;
	// The micro-routines of CAXI and the bit strings make word accesses; a load
	// waiting in MEM goes first.
	wire        us_bus_req_w, us_bus_write_w, us_lock_w;
	wire [31:0] us_addr_w, us_wdata_w;
	wire        seq_req_w = ex_v_q && us_routine_w && us_bus_req_w && !ex_kill_w && !mem_retry_w;
	wire        cx_write_req_w = seq_req_w && us_bus_write_w;
	// The read-ahead asks from ID, the clock after its address is in, once
	// the one before it is leaving EX.
	wire        rd_ask_w = id_v_q && rd_ea_q && !(rd_go_q && !rd_ex_q) && !savestate_pause_req_i && !ss_hold_q;
	wire        rd_req_w = rd_ask_w && (!rd_go_q || (ex_rd_w && ex_adv_w));
	wire [31:0] rd_addr_w = align_addr_fn(ex_ea_w, rd_size_q);
	wire        data_req_w = mem_retry_w || (ex_adv_w && ex_is_load_w && !rd_ex_q) || seq_req_w || rd_req_w;
	wire [31:0] data_addr_w = mem_retry_w ? mem_addr_q : seq_req_w ? us_addr_w : rd_ask_w ? rd_addr_w :
		ex_mem_addr_w;
	wire [1:0]  data_size_w = mem_retry_w ? mem_size_q : seq_req_w ? SIZE_W : rd_ask_w ? rd_size_q :
		ex_mem_size_w;
	wire        data_io_w = mem_retry_w ? mem_io_q : !seq_req_w && !rd_ask_w && ex_mem_io_w;
	wire        data_err_w = mem_retry_w ? mem_addr_err_q : rd_ask_w ? address_error_fn(ex_ea_w, rd_size_q) :
		!(seq_req_w && ex_is_bs_w) && ex_addr_err_w;
	wire        data_accept_w = eng_launch_accept_w && data_req_w && !sb_req_w && !seq_req_w;
	wire        rd_accept_w = data_accept_w && rd_req_w && !mem_retry_w;
	wire        cx_accept_w = eng_launch_accept_w && seq_req_w && !sb_req_w;
	// The sequence access in flight (CAXI or bit string) completes.
	wire        seq_done_w = eng_complete_w && !eng_active_fetch_w && !eng_active_posted_w;
	assign f_accept_w = eng_launch_accept_w && !data_req_w && !sb_req_w;
	// The stream starts no word while EX holds a load, or a data request waits.
	assign data_wait_w = data_req_w || (ex_v_q && ex_is_load_w && !rd_ex_q) || eng_store_due_w;
	assign data_ahead_w = rd_req_w;
	assign data_busy_w = (mem_v_q && mem_store_q) || eng_store_pending_w ||
		(eng_active_w && !eng_active_fetch_w);

	// Stores stage as they leave MEM.
	wire        mem_adv_w;
	wire        push_w = mem_adv_w && mem_store_q;

	necv810_bus_engine #(
		.IO_MRQ(IO_MRQ)
	) u_bus (
		.clk_i(clk_i),
		.reset_i(cpu_reset_w),
		.clk_en_i(ce_i),
		.phi1_i(phi1_i),
		.hldrq_i(hldrq_i),
		.lock_i(us_lock_w),
		.hldak_o(eng_hldak_w),
		.launch_i(sb_req_w || ((data_req_w || f_req_w) && !start_q && !sb_busy_w && !f_xfer_bus_w)),
		.launch_addr_i(f_xfer_req_w ? f_xfer_addr_w : sb_req_w ? sb_addr_q : data_req_w ? data_addr_w : f_addr_w),
		.launch_fetch_i(!sb_req_w && !data_req_w),
		.launch_fetch_branch_i(!sb_req_w && !data_req_w && f_branch_w),
		.launch_write_i((sb_req_w && (!f_xfer_req_w || f_xfer_write_w)) || cx_write_req_w),
		.launch_io_i(!sb_req_w && data_req_w && data_io_w),
		.launch_sys_i((!sb_req_w || f_xfer_req_w) ? 2'b00 : (sb_q == SB_HALT) ? 2'b11 : 2'b01),
		.release_halt_i(halted_w && exc_take_w),
		.launch_addr_error_i(!sb_req_w && data_req_w && data_err_w),
		.launch_size_i(sb_req_w ? ((sb_q == SB_HALT) ? SIZE_H : SIZE_W) :
			data_req_w ? data_size_w : SIZE_W),
		.launch_wdata_i(cx_write_req_w ? us_wdata_w :
			f_xfer_req_w ? f_xfer_wdata_w : (sb_q == SB_HALT) ? {16'd0, sb_data_q} : fatal_word_w),
		.push_i(push_w),
		.push_addr_i(mem_addr_q),
		.push_io_i(mem_io_q),
		.push_addr_error_i(mem_addr_err_q),
		.push_size_i(mem_size_q),
		.push_wdata_i(mem_wdata_q),
		.fixed_16_i(siz16b_q),
		.szrq_i(szrq_i),
		.ready_i(ready_i),
		.din_i(din_i),
		.idle_addr_i(arch_pc_q[31:1]),
		.posted_ready_o(eng_posted_ready_w),
		.launch_accept_o(eng_launch_accept_w),
		.first_beat_o(eng_first_beat_w),
		.complete_o(eng_complete_w),
		.read_data_o(eng_rdata_w),
		.active_o(eng_active_w),
		.active_fetch_o(eng_active_fetch_w),
		.active_posted_o(eng_active_posted_w),
		.posted_pending_o(eng_store_pending_w),
		.posted_due_o(eng_store_due_w),
		.split_pending_o(eng_split_pending_w),
		.a_o(eng_a_w),
		.dout_o(dout_o),
		.dout_oe_o(eng_dout_oe_w),
		.be_o(eng_be_w),
		.st_o(eng_st_w),
		.da_o(eng_da_w),
		.mrq_o(eng_mrq_w),
		.rw_o(eng_rw_w),
		.bcyst_o(eng_bcyst_w),
		.adrs_err_n_o(eng_adrs_err_n_w),
		.cycle_tag_o(cycle_tag_o),
		.beat_t1_o(eng_bus_t1_w)
	);

	// The load in MEM completes on its last beat.
	wire        mem_load_done_w = mem_v_q && mem_load_q && mem_launched_q &&
		eng_complete_w && !eng_active_fetch_w && !eng_active_posted_w && !sb_busy_w;
	wire        sb_done_w = ((sb_q == SB_HALT_WAIT) || (sb_q == SB_FATAL_WAIT)) && eng_complete_w;
	wire [31:0] mem_load_data_w = mem_read_data_fn(mem_size_q, mem_sext_q, eng_rdata_w);
	// The read-ahead's data comes in.
	wire        rd_capture_w = rd_go_q && eng_complete_w && !eng_active_fetch_w && !eng_active_posted_w;

	// ---------------------------------------------------------------
	// Interlock and stage advance.
	// ---------------------------------------------------------------
	// WB redirects (exception, RETI) and the start of the exception entry drop
	// everything younger.
	wire        wb_redirect_w;
	wire        wb_exc_w = wb_v_q && (wb_wk_q == WK_EXC);
	assign ex_flush_w = exc_take_w;
	wire        ex_end_take_w;
	// An interrupt taken other than at a PSW routine's END kills EX.
	wire        irq_take_base_w;
	assign ex_kill_w = ex_flush_w || (irq_take_base_w && !ex_end_take_w) || wb_exc_w;
	// A store waits in MEM until the store before it has written its first halfword.
	assign      mem_adv_w = mem_v_q && (!mem_load_q || mem_load_done_w) &&
		(!mem_store_q || eng_posted_ready_w);
	wire        mem_free_w = !mem_v_q || mem_adv_w;
	// A store held in MEM this clock; the stall clears the lag.
	wire        mem_st_stall_w = mem_v_q && mem_store_q && !eng_posted_ready_w;
	// Nothing leaves EX while an older instruction may still trap in MEM or WB.
	wire        older_trap_w = (mem_v_q && mem_ser_q) || (wb_v_q && wb_ser_q) || exc_q;
	// System routines, illegal opcodes and zero divides act as they leave EX,
	// once everything older has committed; they do not go on to MEM. Of the
	// LDSRs only PSW and CHCW are; the rest write in EX and flow on like an ALU op.
	wire        ex_psw_op_w = ((ex_kind_w == MK_LDSR) &&
		((ex_reg1_w == SYS_PSW) || (ex_reg1_w == SYS_CHCW))) ||
		(ex_is_long_w && ((ex_alu_w == LONG_CLI) || (ex_alu_w == LONG_SEI)));
	wire        ex_sys_w = ex_v_q && ((ex_kind_w == MK_TRAP) || (ex_kind_w == MK_RETI) ||
		(ex_kind_w == MK_HALT) || (ex_kind_w == MK_ILL) || (ex_is_long_w && ex_div0_w) ||
		ex_psw_op_w);
	wire        wbuf_queued_w;
	// TRAP, an illegal opcode or a zero divide waits for older work to leave
	// MEM, not WB: WB commits as the entry starts.
	wire        ex_trap_w = ex_sys_w && (ex_wk_v == WK_EXC);
	// Bit strings retire as they leave EX, having written r26-r30 themselves.
	// Behind a read and its tail clock only a load or a taken transfer leaves EX;
	// an IN waits, as it does not overlap the previous EX.
	wire        ex_go_ok_w = ex_v_q && ex_last_w && mem_free_w && !older_trap_w &&
		(!((load_tail_q && VB_EXT) || (mem_v_q && mem_load_q)) || (ex_is_load_w && !ex_mem_io_w) ||
		 ex_taken_w) && (!ex_rd_w || rd_done_q) &&
		(!(ex_sys_w || ex_is_bs_w) || (!mem_v_q && (!wb_v_q || ex_trap_w)));
	// The same as ex_go_ok_w && !ex_kill_w: once EX may go, its END take is
	// just "is a PSW routine", so READY reaches ex_adv_w only through ex_go_ok_w.
	assign ex_adv_w = ex_go_ok_w && !ex_flush_w && !wb_exc_w &&
		!(irq_take_base_w && !(ex_sys_w && ex_psw_op_w));
	wire        ex_sys_go_w = ex_adv_w && ex_sys_w;
	// LDSR ends on its count and CLI and SEI on their END word, which keeps interrupt
	// acceptance off the bit-string abort path.
	wire        ex_end_w = ex_v_q && (ex_cnt_q <= 6'd1) && (!us_routine_w || us_end_w);
	wire        ex_free_w = !ex_v_q || ex_adv_w;

	// Forwarding into EX: from EX (results on its last clock) and MEM. On the
	// NVC, load data reaches a consumer a clock after the last T2, through WB
	// and the register file; on the uPD70732 it forwards from that T2.
	wire        mem_writes_w = mem_v_q && mem_wr_q;
	wire [31:0] mem_value_w = mem_load_q ? mem_load_data_w : mem_data_q;
	function [32:0] fwd_fn;          // {stall, value}
		input [4:0]  r;
		input [31:0] rf;
		begin
			if (r == 5'd0) begin
				fwd_fn = 33'd0;
			end else if (ex_v_q && ex_wr2_v && (r == 5'd30)) begin
				fwd_fn = {!ex_ready_w, int_hi_w};
			end else if (ex_writes_w && (ex_dest_v == r)) begin
				fwd_fn = {!ex_ready_w, ex_res_v};
			end else if (mem_v_q && mem_wr2_q && (r == 5'd30)) begin
				fwd_fn = {1'b0, mem_data2_q};
			end else if (mem_writes_w && (mem_dest_q == r)) begin
				fwd_fn = {mem_load_q && !(!VB_EXT && mem_load_done_w), mem_value_w};
			end else begin
				fwd_fn = {1'b0, rf};
			end
		end
	endfunction
	// Address bases and JMP targets come from the register file only. On the NVC
	// they are read while the writer is in WB, unless the writer is late; the
	// uPD70732 waits until the writer has left WB.
	function base_wait_fn;
		input [4:0] r;
		begin
			base_wait_fn = (r != 5'd0) &&
				((ex_writes_w && (ex_dest_v == r)) || (mem_writes_w && (mem_dest_q == r)) ||
				 ((ld_base_wait_q != 2'd0) && (ld_base_dest_q == r)) ||
				 ((!VB_EXT || wb_late_q) && wb_v_q && wb_wr_q && (wb_dest_q == r)) ||
				 (ex_v_q && ex_wr2_v && (r == 5'd30)) || (mem_v_q && mem_wr2_q && (r == 5'd30)));
		end
	endfunction
	wire [32:0] fwd_r1_w = fwd_fn(id_reg1_w, rf_a_w);
	wire [32:0] fwd_r2_w = fwd_fn(id_reg2_w, rf_b_w);
	// A Bcond waits while either of the two older stages may still change PSW;
	// WB writes it this clock. LDSR, CLI and SEI write PSW as they leave EX.
	// The decoder's Bcond also waits for a search setting Z as it leaves EX,
	// and for a late writer in WB. XB, XH, REV and MPYHW write no flags.
	wire        ex_long_flags_w = ex_is_long_w && (ex_alu_w != LONG_XB) && (ex_alu_w != LONG_XH) &&
		(ex_alu_w != LONG_REV) && (ex_alu_w != LONG_MPYHW);
	wire        flags_busy_w = (ex_v_q && (ex_wpsw_w || ex_long_flags_w || (ex_kind_w == MK_LDSR) ||
		ex_is_caxi_w)) || (mem_v_q && (mem_fmask_q != 10'd0));
	wire        bcond_busy_w = flags_busy_w || (ex_v_q && ex_is_bs_w && (ex_iw_q[20:19] == 2'b00)) ||
		(wb_late_q && wb_v_q && (wb_fmask_q != 10'd0));
	wire        ser_busy_w = (ex_v_q && (ex_kind_w == MK_TRAP || ex_kind_w == MK_RETI ||
		ex_kind_w == MK_HALT || ex_kind_w == MK_ILL || ex_wk_v != WK_NONE)) ||
		(mem_v_q && mem_ser_q) || (wb_v_q && wb_ser_q) || exc_q;
	wire        pipe_busy_w = ex_v_q || mem_v_q || wb_v_q || exc_q || sb_busy_w || ent_busy_w;
	// A store beat waits behind the one on the bus: a queued store, or the
	// second half of a word store. RETI, interrupts and EX-taken exceptions wait for none.
	assign      wbuf_queued_w = eng_store_pending_w || (eng_active_posted_w && eng_split_pending_w);
	wire        wbuf_busy_w = (ex_v_q && ex_is_store_w) || (mem_v_q && mem_store_q) || wbuf_queued_w;
	// CAXI's read follows the write buffer's drain, and it takes r30 from the
	// register file in its first EX clock.
	wire        cx_busy_w = (ex_v_q && ex_is_mem_w) || (mem_v_q && (mem_load_q || mem_store_q)) ||
		eng_store_pending_w || (eng_active_w && eng_active_posted_w) ||
		(ex_writes_w && (ex_dest_v == 5'd30)) || (ex_v_q && ex_wr2_v) ||
		(mem_writes_w && (mem_dest_q == 5'd30)) || (mem_v_q && mem_wr2_q) ||
		(wb_v_q && wb_wr_q && (wb_dest_q == 5'd30)) || (wb_v_q && wb_wr2_q);
	function bs_reg_fn;
		input [4:0] r;
		begin
			bs_reg_fn = (r >= 5'd26) && (r <= 5'd30);
		end
	endfunction
	// A bit string reads r26-r30 itself from its third clock; a writer of them
	// still in MEM then holds it.
	wire        bs_regs_busy_w = (mem_writes_w && bs_reg_fn(mem_dest_q)) || (mem_v_q && mem_wr2_q);
	wire        id_hazard_w =
		(id_use_r1_alu_w && fwd_r1_w[32]) ||
		(id_use_r1_base_w && base_wait_fn(id_reg1_w)) ||
		(id_use_r2_w && fwd_r2_w[32]) ||
		(id_use_flags_w && bcond_busy_w) ||
		ser_busy_w || (id_ser_w && cx_busy_w) ||
		addr_trap_w || async_pending_w || irq_wait_w || ent_busy_w || savestate_pause_req_i || ss_hold_q;
	// RETI waits for a load ahead of it to finish, then for the bus unit to go
	// idle: no store left to stage, nothing on the bus. It checks the bus every
	// other clock from the first check.
	wire        id_go_w = id_v_q && ex_free_w && !id_hazard_w && !redirect_w &&
		!(ex_v_q && ex_redir_q) && !redir_gap_q;
	wire        load_ahead_w = (ex_v_q && ex_is_load_w) || (mem_v_q && mem_load_q);
	wire        bus_idle_w = !wbuf_busy_w && !eng_active_w;
	wire        reti_wait_w = (id_kind_w == MK_RETI) && (load_ahead_w || !bus_idle_w || reti_ph_q);
	// After an early redirect the target issues when it would have after a late one.
	wire        id_adv_w = id_go_w && !reti_wait_w;
	wire        id_free_w = !id_v_q || id_adv_w;
	// A load (not IN) reads ahead while EX holds, and keeps, a long op that
	// does not use the bus, or a load that read ahead.
	wire        ex_quiet_long_w = ex_v_q && ex_is_long_w && !ex_sys_w && (ex_alu_w != LONG_XH);
	wire        rd_ea_go_w = id_v_q && id_is_mem_w && !id_is_store_w && !id_uc_w[20] &&
		!rd_ea_q && (ex_quiet_long_w || ex_rd_w) && !ex_adv_w && !id_hazard_w;
	// Decode held with the fetch free to run on; microcoded work stops it.
	assign f_id_hold_w = id_v_q && !id_adv_w && !redirect_w && !id_ser_w && !ent_busy_w &&
		!irq_wait_w && !async_pending_w && (id_kind_w != MK_RETI) && (id_kind_w != MK_TRAP) &&
		(id_kind_w != MK_HALT);
	// BR, JR, JAL and JMP always leave the line; all but JMP carry their target
	// in the opcode.
	assign f_id_rel_jump_w = ((id_ue_w == UADDR_BCOND) && (id_iw_q[28:25] == 4'h5)) ||
		(id_ue_w == UADDR_JR) || (id_ue_w == UADDR_JAL);
	assign f_id_jump_w = f_id_rel_jump_w || id_is_jmp_w;
	assign id_load_w = f_valid_w && id_free_w && !redirect_w && !start_q && !halted_w &&
		!sb_busy_w && !fatal_w;

	// Effective address unit: loads, stores, CAXI and JMP add to a register
	// base, Bcond, JR and JAL to their own PC.
	necv810_ea u_ea (
		.clk_i(clk_i),
		.reset_i(cpu_reset_w),
		.ce_i(ce_i),
		.load_i(id_adv_w || rd_ea_go_w),
		.pc_rel_i(id_is_bcond_w || (id_ue_w == UADDR_JR) || (id_ue_w == UADDR_JAL)),
		.pc_i(id_pc_q),
		.base_i(rf_a_w),
		.disp_i(id_imm_w),
		.ea_o(ex_ea_w)
	);

	// ---------------------------------------------------------------
	// Commit (WB) and exception control.
	// ---------------------------------------------------------------
	// An FP trap commits in WB like any instruction and starts the entry there.
	assign wb_redirect_w = ex_sys_go_w && ((ex_kind_w == MK_RETI) || (ex_kind_w == MK_HALT));
	// An interrupt is taken at a boundary. At an instruction's END it
	// finishes (a PSW routine against the PSW it writes, a taken transfer
	// returning to its target); system ops and routines leave EX first and a
	// load waits for its data. The other boundaries drop EX: an idle or
	// abortable EX (the routine is rerun), RETI's target reaching EX (arch_pc
	// is its PC), or a drop set that is not empty. Stores and older work
	// finish during the entry, which waits for the stores at its fifth step.
	assign ex_end_take_w = ex_go_ok_w && ex_sys_w && ex_psw_op_w;
	wire        ex_plain_take_w = ex_go_ok_w && !ex_sys_w && !ex_is_bs_w &&
		!ex_is_caxi_w && !us_routine_w && !ex_abort_w && !ex_is_load_w;
	wire        irq_end_w = ex_end_take_w || ex_plain_take_w;
	wire        irq_idle_w = (!ex_v_q || ex_abort_w) && !older_trap_w && !(mem_v_q && mem_load_q) &&
		!load_tail_q;
	wire        reti_take_w = reti_tgt_q && ex_v_q && !mem_v_q && !wb_v_q;
	// The drop set: what has no result yet is dropped and run again after
	// RETI. A dropped read still runs out on the bus, and the entry waits for
	// it at its settle step. IN is never dropped.
	// - A load in MEM still in the first half of its read (not started, or a
	//   word load whose low half has not come in), with work waiting in EX.
	// - On a load's tail clock: the load that followed it into MEM, or the
	//   instruction waiting behind it in EX.
	// Two exceptions: a load at a jump's target completes, and a not-taken
	// conditional branch behind a reading load drops that load at once.
	wire        ex_held_w = ex_v_q && !ex_sys_w && !ex_is_bs_w && !ex_is_caxi_w && !us_routine_w &&
		!ex_is_long_w && !ex_taken_w && !ex_is_load_w;
	wire        ex_waits_w = ex_held_w || (ex_v_q && ex_is_load_w && !rd_ex_q);
	wire        mem_low_wait_w = !mem_launched_q || (eng_split_pending_w && !eng_first_beat_w &&
		!eng_active_fetch_w && !eng_active_posted_w);
	wire        mem_first_half_w = mem_low_wait_w && ex_waits_w;
	wire        mem_after_load_w = load_tail_q && (!ex_v_q || ex_waits_w);
	wire        jump_target_w = mem_tgt_q;
	wire        bcond_behind_read_w = ex_v_q && (ex_kind_w == MK_BRANCH) && !ex_taken_w;
	wire        drop_mem_w = !older_trap_w && mem_v_q && mem_load_q && !mem_io_q &&
		((mem_first_half_w && !jump_target_w) || mem_after_load_w || bcond_behind_read_w);
	wire        drop_ex_w = !older_trap_w && load_tail_q && !(mem_v_q && mem_load_q) && ex_held_w;
	// The return PC: after an END the next instruction; otherwise the oldest
	// dropped one, or the one after the youngest that finishes.
	wire [31:0] irq_rpc_w = (async_code_w == EXC_CODE_ADDRTRAP) ? id_pc_q :
		(ex_end_take_w || !irq_take_base_w) ? (ex_taken_w ? ex_target_w : ex_seq_pc_q) :
		drop_mem_w ? mem_pc_q : (drop_ex_w || (ex_v_q && ex_abort_w)) ? ex_pc_q :
		mem_v_q ? mem_next_pc_q : wb_v_q ? wb_next_pc_q : arch_pc_q;
	// HALT checks for an interrupt every other clock, counted from its first.
	wire        irq_ok_w = async_pending_w && !(halted_w && halt_ph_q) &&
		!exc_q && !ent_busy_w && !start_q && !sb_busy_w && !fatal_w &&
		!savestate_pause_req_i && !ss_hold_q;
	assign irq_take_base_w = irq_ok_w && (irq_idle_w || reti_take_w || drop_mem_w || drop_ex_w);
	assign irq_take_w = irq_take_base_w || (irq_ok_w && irq_end_w);
	assign exc_take_w = exc_q;
	assign exc_code_w = exc_code_q;
	assign exc_rpc_w = exc_rpc_q;

	// A taken transfer redirects fetch as it leaves EX. BR (Bcond 5), JR, JAL
	// and JMP redirect at once while a load still holds MEM; younger work is
	// dropped and the target waits in ID.
	wire        ex_early_w = ex_v_q && ex_last_w && ex_taken_w && !ex_redir_q &&
		mem_v_q && mem_load_q && !mem_load_done_w &&
		!older_trap_w && !ex_kill_w && (ex_jump_w || (ex_iw_q[28:25] == 4'h5));
	// A taken conditional branch held in EX by a load in MEM, and one (not BR)
	// redirecting the fetch: the fetch drops read-ahead fills not yet started.
	assign      f_bcond_load_wait_w = ex_v_q && ex_bcond_taken_w && !ex_redir_q && mem_v_q && mem_load_q &&
		!ex_early_w;
	assign      f_bcond_redirect_w = ex_adv_w && ex_bcond_taken_w && !ex_redir_q && (ex_iw_q[28:25] != 4'h5);
	// An LDSR to CHCW restarts the fetch at the next instruction as it leaves
	// EX; nothing new is fetched until its clear, dump or restore is done.
	wire        ex_chcw_w = ex_v_q && (ex_kind_w == MK_LDSR) && (ex_reg1_w == SYS_CHCW);
	assign f_fill_stop_w = (id_v_q && (id_kind_w == MK_LDSR) && (id_reg1_w == SYS_CHCW)) || ex_chcw_w;
	assign redirect_w = start_q || exc_take_w || wb_redirect_w ||
		(ex_adv_w && ex_taken_w && !ex_redir_q) || ex_early_w || (ex_adv_w && ex_chcw_w);
	// A trigger sets the lag as it leaves EX, an LDSR on its first EX clock.
	// A flow change other than an early branch clears it, and so does a stall:
	// EX held past an instruction's first clock (a long LDSR clears its own),
	// decode held by a hazard with EX free, or a store held in MEM. A read on
	// the bus ahead is not a stall; it paces what follows.
	wire        lag_trigger_w = ex_v_q && pipe_lag_activator_fn(ex_iw_q[31:26]);
	wire        lag_set_w = lag_trigger_w && ((ex_kind_w == MK_LDSR) ? ex_first_q : ex_adv_w);
	wire        read_busy_w = (ex_v_q && ex_is_load_w) || (mem_v_q && mem_load_q) || load_tail_q ||
		(ex_rd_w && !rd_done_q);
	wire        lag_stall_w = ((ex_v_q && !ex_adv_w && !ex_first_q) || (id_v_q && ex_free_w && id_hazard_w)) &&
		!read_busy_w;
	wire        lag_clear_w = (redirect_w && !ex_early_w) || lag_stall_w || mem_st_stall_w;
	wire        ex_late_w = lag_q || lag_trigger_w;
	assign redirect_pc_w = start_q ? arch_pc_q :
		exc_take_w ? exc_handler_w :
		(ex_sys_go_w && (ex_kind_w == MK_RETI)) ? reti_pc_w :
		(ex_sys_go_w && (ex_kind_w == MK_HALT)) ? ex_seq_pc_q :
		(ex_adv_w && ex_chcw_w) ? ex_seq_pc_q :
		ex_target_w;

	// CLI, SEI, XB, REV, MPYHW, CAXI, the illegal opcode and the zero divide run
	// on the micro-sequencer while they hold EX; the exception entry runs with EX
	// empty. Every exception starts the entry on the clock it is taken.
	wire        ex_exc_go_w = ex_sys_go_w && (ex_wk_v == WK_EXC);
	// Taken with the pipe already stopped, at CLI, SEI or LDSR PSW or at RETI's
	// return target: the entry skips its first three steps.
	wire        irq_quick_w = irq_take_w && !wb_exc_w && !ex_exc_go_w && (ex_end_take_w || reti_take_w);
	wire        ent_start_w = irq_take_w || wb_exc_w || ex_exc_go_w;
	// Bit strings use read port A and the WB write port, one register a clock.
	wire        us_rf_rd_w, us_rf_we_w, us_found_w, us_abort_w;
	wire [4:0]  us_rf_addr_w;
	wire [31:0] us_rf_wdata_w;
	// A search sets Z as it retires: 1 when nothing was found.
	wire        bs_z_we_w = ex_adv_w && ex_is_bs_w && (ex_iw_q[20:19] == 2'b00) && !us_abort_w;
	necv810_useq #(
		.VB_EXT(VB_EXT)
	) u_useq (
		.clk_i(clk_i),
		.reset_i(cpu_reset_w),
		.ce_i(ce_i),
		.issue_i(id_adv_w),
		.id_ue_i(id_ue_w),
		.id_sub_i(id_iw_q[15:10]),
		.id_div0_i(fwd_r1_w[31:0] == 32'd0),
		.ex_v_i(ex_v_q),
		.ex_first_i(ex_first_q),
		.ex_stop_i(ex_adv_w || ex_kill_w),
		.ent_i(ent_start_w && !halted_w),
		.wake_i(irq_take_w && halted_w),
		.quick_i(irq_quick_w),
		.r1_i(ex_r1_q),
		.r2_i(ex_r2_q),
		.psw_i(psw_w),
		.r30_i(ex_r30_q),
		.drained_i(!eng_active_w && !eng_store_pending_w && !mem_v_q && !wb_v_q),
		.older_i(mem_v_q || wb_v_q || drop_rd_q),
		.wbuf_i(eng_store_pending_w || (eng_active_w && eng_active_posted_w)),
		.bus_accept_i(cx_accept_w),
		.bus_done_i(ex_v_q && us_routine_w && seq_done_w),
		.bus_rdata_i(eng_rdata_w),
		.ea_i(ex_mem_addr_w),
		.id_bs_i({id_iw_q[20:19], id_iw_q[16]}),
		.ex_bs_i(ex_iw_q[20:16]),
		.irq_i(async_pending_w),
		.rf_i(rf_a_w),
		.rf_busy_i(bs_regs_busy_w),
		.rf_free_i(!mem_v_q),
		.routine_o(us_routine_w),
		.end_o(us_end_w),
		.result_o(us_res_w),
		.psw_we_o(us_psw_we_w),
		.flags_o(us_flags_w),
		.bus_req_o(us_bus_req_w),
		.bus_write_o(us_bus_write_w),
		.bus_addr_o(us_addr_w),
		.bus_wdata_o(us_wdata_w),
		.lock_o(us_lock_w),
		.rf_rd_o(us_rf_rd_w),
		.rf_we_o(us_rf_we_w),
		.rf_addr_o(us_rf_addr_w),
		.rf_wdata_o(us_rf_wdata_w),
		.found_o(us_found_w),
		.abort_o(us_abort_w),
		.ent_busy_o(ent_busy_w),
		.exc_o(us_exc_w),
		.fp_we_o(us_fp_we_w),
		.fp_flags_o(us_fp_flags_w),
		.fp_fmask_o(us_fp_fmask_w),
		.fp_exc_o(us_fp_exc_w),
		.fp_code_o(us_fp_code_w)
	);

	wire [1:0]  cache_cmd_w;
	wire [6:0]  cache_clear_first_w;
	wire [6:0]  cache_clear_last_w;
	wire [15:0] trap_code_w;
	wire [31:0] trap_rpc_w;
	wire        reti_defer_w;
	necv810_sysctl #(
		.RESET_PSW(RESET_PSW),
		.VB_EXT(VB_EXT)
	) u_sys (
		.clk_i(clk_i),
		.reset_i(cpu_reset_w),
		.ce_i(ce_i),
		.sr_id_i(ex_reg1_w),
		.sr_value_o(sr_value_w),
		.psw_o(psw_w),
		.reti_pc_o(reti_pc_w),
		.reti_psw_o(),
		.fpsw_we_i((wb_v_q && (wb_fmask_q != 10'd0)) || bs_z_we_w),
		.fpsw_mask_i(bs_z_we_w ? 10'd1 : wb_fmask_q),
		.fpsw_i(bs_z_we_w ? {9'd0, !us_found_w} : wb_flags_q),
		// CLI and SEI write PSW from their routine through the LDSR path.
		.ldsr_we_i(ex_adv_w && ((ex_kind_w == MK_LDSR) || us_psw_we_w)),
		.ldsr_id_i(us_psw_we_w ? SYS_PSW : ex_reg1_w),
		.ldsr_value_i(us_psw_we_w ? us_res_w : ex_r2_q),
		.issue_i(id_adv_w),
		.cache_cmd_o(cache_cmd_w),
		.cache_cmd_valid_o(cache_cmd_valid_w),
		.cache_clear_first_o(cache_clear_first_w),
		.cache_clear_last_o(cache_clear_last_w),
		.cache_sa_o(cache_sa_w),
		.cache_enable_o(cache_enable_w),
		.cache_pending_o(cache_pending_w),
		.chcw_cmd_o(chcw_cmd_w),
		.exc_take_i(exc_take_w),
		.exc_code_i(exc_code_w),
		.exc_restore_pc_i(exc_rpc_w),
		.exc_handler_o(exc_handler_w),
		.exc_fatal_o(exc_fatal_w),
		.trap_vector_i(ex_reg1_w),
		.trap_pc_i(ex_pc_q),
		.trap_code_o(trap_code_w),
		.trap_restore_pc_o(trap_rpc_w),
		.reti_take_i(ex_sys_go_w && (ex_kind_w == MK_RETI)),
		.reti_defer_o(reti_defer_w),
		.end_ldsr_i(ex_end_w && ((ex_kind_w == MK_LDSR) || us_psw_we_w)),
		.fatal_index_i(sb_index_q),
		.fatal_done_i(sb_done_w && (sb_q == SB_FATAL_WAIT) && (sb_index_q == 2'd2)),
		.fatal_word_o(fatal_word_w),
		.fatal_o(fatal_w),
		.irq_valid_i(irq_valid_i),
		.irq_level_i(irq_level_i),
		.nmi_i(nmi_i),
		// Stop a matching instruction in decode while older work commits.
		.boundary_pc_i(id_pc_q),
		.boundary_valid_i(id_v_q),
		.boundary_ready_i(!ex_v_q && !mem_v_q && !wb_v_q),
		.addr_trap_o(addr_trap_w),
		.irq_hold_i(1'b0),
		.async_pending_o(async_pending_w),
		.async_code_o(async_code_w),
		.async_is_irq_o(),
		.async_is_nmi_o(),
		.irq_wait_o(irq_wait_w),
		.halt_i(sb_done_w && (sb_q == SB_HALT_WAIT)),
		.halted_o(halted_w),
		.pc_i(exc_q ? exc_pc_q : arch_pc_q),
		.ss_word_addr_i(savestate_state_addr_i),
		.ss_word_o(ss_sys_word_w),
		.ss_word_we_i(savestate_state_wren_i),
		.ss_word_i(savestate_state_wdata_i)
	);
	// ---------------------------------------------------------------
	// Register file. Reads follow ID; results are written as they enter WB.
	// ---------------------------------------------------------------
	// A result is written as its instruction enters WB, MUL/DIV's r30 beside
	// it; a bit string writes r26-r30 while MEM is empty.
	// A dropped load drains its read without publishing the result.
	wire rf_enter_w = mem_adv_w && !ex_flush_w && !(irq_take_w && drop_mem_w);
	necv810_regfile u_gpr (
		.clk_i(clk_i),
		.ce_i(ce_i),
		// CAXI takes r30 on its first EX clock; after that the next instruction reads.
		.read_addr_a_i(halted_w ? dbg_reg_addr_i : us_rf_rd_w ? us_rf_addr_w :
			(ex_v_q && ex_is_caxi_w && ex_first_q) ? 5'd30 : id_reg1_w),
		.read_addr_b_i(id_reg2_w),
		.read_data_a_o(rf_a_w),
		.read_data_b_o(rf_b_w),
		.write_sel_a_i(us_rf_we_w || (mem_v_q && mem_wr_q)),
		.write_a_i(us_rf_we_w || (rf_enter_w && mem_wr_q)),
		.write_addr_a_i(us_rf_we_w ? us_rf_addr_w : mem_dest_q),
		.write_data_a_i(us_rf_we_w ? us_rf_wdata_w : mem_value_w),
		.write_sel_b_i(mem_v_q && mem_wr2_q),
		.write_b_i(rf_enter_w && mem_wr2_q),
		.write_data_b_i(mem_data2_q),
		.savestate_active_i(ss_gpr_w),
		.savestate_addr_i(savestate_mem_addr_i[6:0]),
		.savestate_rden_i(savestate_mem_rden_i),
		.savestate_wren_i(savestate_mem_wren_i),
		.savestate_wdata_i(savestate_mem_wdata_i),
		.savestate_rdata_o(ss_gpr_rdata_w)
	);

	// ---------------------------------------------------------------
	// Pipeline registers.
	// ---------------------------------------------------------------
	// Younger work waits for system state to commit.
	wire mem_ser_next_w = (ex_wk_v != WK_NONE);
	// The bus-width pin is sampled while reset is held.
	always @(posedge clk_i) begin
		if (cpu_reset_w)
			siz16b_q <= siz16b_i;
	end

	always @(posedge clk_i or posedge cpu_reset_w) begin
		if (cpu_reset_w) begin
			ss_hold_q <= 1'b0;
			lag_q <= 1'b0;
			mem_late_q <= 1'b0;
			wb_late_q <= 1'b0;
			load_tail_q <= 1'b0;
			rd_ea_q <= 1'b0;
			rd_go_q <= 1'b0;
			rd_ex_q <= 1'b0;
			rd_done_q <= 1'b0;
			rd_size_q <= 2'd0;
			rd_sext_q <= 1'b0;
			rd_read_size_q <= 2'd0;
			rd_read_sext_q <= 1'b0;
			rd_data_q <= 32'd0;
			reti_tgt_q <= 1'b0;
			reti_ph_q <= 1'b0;
			ex_redir_q <= 1'b0;
			redir_gap_q <= 1'b0;
			tgt_pend_q <= 1'b0;
			ex_tgt_q <= 1'b0;
			mem_tgt_q <= 1'b0;
			halt_ph_q <= 1'b0;
			wake_gap_q <= 1'b0;
			data_busy_q <= 1'b0;
			ld_base_dest_q <= 5'd0;
			ld_base_wait_q <= 2'd0;
			exc_q <= 1'b0;
			exc_code_q <= 16'd0;
			exc_rpc_q <= 32'd0;
			exc_pc_q <= 32'd0;
			sb_q <= SB_IDLE;
			sb_index_q <= 2'd0;
			sb_addr_q <= 32'd0;
			sb_data_q <= 16'd0;
			start_q <= 1'b1;
			arch_pc_q <= RESET_PC;
			id_v_q <= 1'b0;
			id_pc_q <= 32'd0;
			id_iw_q <= 32'd0;
			ex_v_q <= 1'b0;
			ex_pc_q <= 32'd0;
			ex_iw_q <= 16'd0;
			ex_seq_pc_q <= 32'd0;
			ex_uc_q <= ucode_fn(6'd0);
			ex_imm_q <= issue_immediate_fn(6'd0, 32'd0);
			ex_cc_on_q <= 1'b0;
			ex_cc_q <= 4'd0;
			ex_taken_q <= 1'b0;
			ex_r1_q <= 32'd0;
			ex_r2_q <= 32'd0;
			ex_cnt_q <= 6'd0;
			ex_first_q <= 1'b0;
			ex_r30_q <= 32'd0;
			mem_v_q <= 1'b0;
			mem_pc_q <= 32'd0;
			mem_iw_q <= 16'd0;
			mem_load_q <= 1'b0;
			mem_launched_q <= 1'b0;
			drop_rd_q <= 1'b0;
			mem_store_q <= 1'b0;
			mem_wdata_q <= 32'd0;
			mem_addr_q <= 32'd0;
			mem_size_q <= 2'd0;
			mem_sext_q <= 1'b0;
			mem_io_q <= 1'b0;
			mem_addr_err_q <= 1'b0;
			mem_wr_q <= 1'b0;
			mem_dest_q <= 5'd0;
			mem_data_q <= 32'd0;
			mem_wr2_q <= 1'b0;
			mem_data2_q <= 32'd0;
			mem_flags_q <= 10'd0;
			mem_fmask_q <= 10'd0;
			mem_next_pc_q <= 32'd0;
			mem_wk_q <= WK_NONE;
			mem_code_q <= 16'd0;
			mem_rpc_q <= 32'd0;
			mem_ser_q <= 1'b0;
			wb_v_q <= 1'b0;
			wb_pc_q <= 32'd0;
			wb_wr_q <= 1'b0;
			wb_dest_q <= 5'd0;
			wb_wr2_q <= 1'b0;
			wb_flags_q <= 10'd0;
			wb_fmask_q <= 10'd0;
			wb_next_pc_q <= 32'd0;
			wb_wk_q <= WK_NONE;
			wb_code_q <= 16'd0;
			wb_rpc_q <= 32'd0;
			wb_ser_q <= 1'b0;
			trace_valid_o <= 1'b0;
			trace_pc_o <= 32'd0;
			trace_insn_o <= 16'd0;
		end else if (savestate_state_wren_i) begin
			// Restored PC; the PSW and system registers live in u_sys.
			// Leave the restore held, as the save was, so both resume alike.
			if (savestate_state_addr_i == 4'd0) begin
				arch_pc_q <= {savestate_state_wdata_i[31:1], 1'b0};
				ss_hold_q <= 1'b1;
				start_q <= 1'b0;
			end
			// HALT's check phase rides in a spare bit of word 5.
			if (savestate_state_addr_i == 4'd5) halt_ph_q <= savestate_state_wdata_i[6];
		end else if (ce_i) begin
			start_q <= 1'b0;
			trace_valid_o <= 1'b0;
			// Savestate pause: hold once everything in flight has drained, and
			// restart fetch at arch_pc when the request drops.
			if (savestate_pause_req_i && !ss_hold_q && !pipe_busy_w && !eng_active_w &&
				!eng_store_pending_w && !f_busy_w &&
				!cache_cmd_valid_w && !f_clear_busy_w && !f_xfer_busy_w &&
				!int_busy_w)
				ss_hold_q <= 1'b1;
			if (!savestate_pause_req_i && ss_hold_q) begin
				ss_hold_q <= 1'b0;
				start_q <= 1'b1;
			end

			// WB: retire. GPRs were written at phi1; PSW and system state now.
			// The entry routine's last step hands over to the redirect.
			exc_q <= us_exc_w;
			if (irq_take_w) begin
				exc_code_q <= async_code_w;
				exc_rpc_q <= irq_rpc_w;
				exc_pc_q <= irq_rpc_w;
			end
			if (wb_exc_w) begin
				exc_code_q <= wb_code_q;
				exc_rpc_q <= wb_rpc_q;
				exc_pc_q <= wb_pc_q;
			end
			if (ex_exc_go_w) begin
				exc_code_q <= ex_code_v;
				exc_rpc_q <= ex_rpc_v;
				exc_pc_q <= {ex_pc_q[31:1], 1'b0};
			end
			if (ex_sys_go_w) begin
				arch_pc_q <= (ex_kind_w == MK_RETI) ? reti_pc_w : ex_seq_pc_q;
				// System routines retire here; other traps do not.
				if ((ex_kind_w == MK_TRAP) || (ex_kind_w == MK_RETI) || (ex_kind_w == MK_HALT) ||
					ex_psw_op_w) begin
					trace_valid_o <= 1'b1;
					trace_pc_o <= ex_pc_q;
					trace_insn_o <= ex_iw_q;
				end
			end
			// System bus sequencer.
			case (sb_q)
				SB_HALT, SB_FATAL: if (sb_req_w && eng_launch_accept_w && !f_xfer_req_w) sb_q <= sb_q + 3'd1;
				SB_HALT_WAIT: if (sb_done_w) sb_q <= SB_IDLE;
				SB_FATAL_WAIT: if (sb_done_w) begin
					if (sb_index_q == 2'd2) begin
						sb_q <= SB_IDLE;
					end else begin
						sb_index_q <= sb_index_q + 2'd1;
						sb_addr_q <= sb_addr_q + 32'd4;
						sb_q <= SB_FATAL;
					end
				end
				default: begin
					if (ex_sys_go_w && (ex_kind_w == MK_HALT)) begin
						sb_q <= SB_HALT;
						sb_addr_q <= {ex_pc_q[31:1], 1'b0};
						sb_data_q <= psw_w[15:0];
					end else if (exc_take_w && exc_fatal_w) begin
						sb_q <= SB_FATAL;
						sb_index_q <= 2'd0;
						sb_addr_q <= 32'd0;
					end
				end
			endcase
			if (wb_v_q) begin
				arch_pc_q <= (wb_wk_q == WK_EXC) ? wb_rpc_q :
					(wb_wk_q == WK_RETI) ? reti_pc_w : wb_next_pc_q;
			end
			if (exc_take_w) arch_pc_q <= exc_handler_w;
			wb_v_q <= rf_enter_w;
			// The retire pulse marks the WB clock, when GPRs commit. TRAP
			// retires as it enters its handler; other traps do not.
			if (rf_enter_w && (mem_wk_q != WK_EXC)) begin
				trace_valid_o <= 1'b1;
				trace_pc_o <= mem_pc_q;
				trace_insn_o <= mem_iw_q[31:16];
			end
			if (mem_adv_w) begin
				wb_pc_q <= mem_pc_q;
				wb_wr_q <= mem_wr_q;
				wb_dest_q <= mem_dest_q;
				wb_wr2_q <= mem_wr2_q;
				wb_flags_q <= mem_flags_q;
				wb_fmask_q <= mem_fmask_q;
				wb_next_pc_q <= mem_next_pc_q;
				wb_wk_q <= mem_wk_q;
				wb_code_q <= mem_code_q;
				wb_rpc_q <= mem_rpc_q;
				wb_ser_q <= mem_ser_q;
				wb_late_q <= mem_late_q;
			end

			// MEM.
			if (data_accept_w) mem_launched_q <= 1'b1;
			// A dropped load's read is still on the bus until its last beat.
			if (irq_take_w && drop_mem_w && mem_launched_q &&
				!(eng_complete_w && !eng_active_fetch_w && !eng_active_posted_w))
				drop_rd_q <= 1'b1;
			else if (eng_complete_w && !eng_active_fetch_w && !eng_active_posted_w)
				drop_rd_q <= 1'b0;
			if (mem_adv_w || ex_flush_w || (irq_take_w && drop_mem_w)) mem_v_q <= 1'b0;
			if (ex_adv_w && !ex_sys_w && !ex_is_bs_w) begin
				mem_v_q <= 1'b1;
				mem_pc_q <= ex_pc_q;
				mem_iw_q <= ex_iw_q;
				mem_load_q <= ex_is_load_w && !rd_ex_q;
				mem_tgt_q <= ex_tgt_q;
				mem_launched_q <= ex_is_load_w && data_accept_w;
				mem_store_q <= ex_is_store_w;
				mem_wdata_q <= ex_r2_q;
				mem_addr_q <= ex_mem_addr_w;
				mem_size_q <= ex_mem_size_w;
				mem_sext_q <= ex_mem_sext_w;
				mem_io_q <= ex_mem_io_w;
				mem_addr_err_q <= ex_addr_err_w;
				mem_wr_q <= ex_wr_v;
				mem_dest_q <= ex_dest_v;
				mem_data_q <= ex_res_v;
				mem_wr2_q <= ex_wr2_v;
				mem_data2_q <= int_hi_w;
				mem_flags_q <= ex_flags_v;
				mem_fmask_q <= ex_fmask_v;
				mem_next_pc_q <= ex_taken_w ? ex_target_w : ex_seq_pc_q;
				mem_wk_q <= ex_wk_v;
				mem_code_q <= ex_code_v;
				mem_rpc_q <= ex_rpc_v;
				mem_ser_q <= mem_ser_next_w;
				mem_late_q <= ex_late_w;
			end

			// A bit string retires as it leaves EX, unless an interrupt
			// stopped it; it then runs again from its own PC.
			if (ex_adv_w && ex_is_bs_w) begin
				arch_pc_q <= us_abort_w ? {ex_pc_q[31:1], 1'b0} : ex_seq_pc_q;
				if (!us_abort_w) begin
					trace_valid_o <= 1'b1;
					trace_pc_o <= ex_pc_q;
					trace_insn_o <= ex_iw_q;
				end
			end

			load_tail_q <= rf_enter_w && mem_load_q;
			if (rd_ea_go_w) begin
				rd_ea_q <= 1'b1;
				rd_size_q <= id_uc_w[22:21];
				rd_sext_q <= id_uc_w[23];
			end
			if (id_adv_w || redirect_w) rd_ea_q <= 1'b0;
			if (rd_capture_w) begin
				rd_done_q <= 1'b1;
				rd_data_q <= mem_read_data_fn(rd_read_size_q, rd_read_sext_q, eng_rdata_w);
			end
			if (ex_rd_w && (ex_adv_w || ex_kill_w)) begin
				rd_go_q <= 1'b0;
				rd_ex_q <= 1'b0;
				rd_done_q <= 1'b0;
			end
			if (id_adv_w) rd_ex_q <= (rd_go_q && !rd_ex_q) || rd_accept_w;
			if (rd_accept_w) begin
				rd_go_q <= 1'b1;
				rd_done_q <= 1'b0;
				rd_read_size_q <= rd_size_q;
				rd_read_sext_q <= rd_sext_q;
			end
			// A read whose load was dropped still finishes on the bus, unclaimed.
			if (redirect_w) begin
				rd_go_q <= 1'b0;
				rd_ex_q <= 1'b0;
				rd_done_q <= 1'b0;
			end
			reti_tgt_q <= reti_defer_w && id_adv_w;
			reti_ph_q <= id_go_w && reti_wait_w && !reti_ph_q && !load_ahead_w;
			data_busy_q <= data_busy_w;
			if (ld_base_wait_q != 2'd0) ld_base_wait_q <= ld_base_wait_q - 2'd1;
			if (rf_enter_w && mem_load_q && mem_wr_q) begin
				ld_base_dest_q <= mem_dest_q;
				ld_base_wait_q <= 2'd3;
			end
			if (lag_clear_w) lag_q <= 1'b0;
			else if (lag_set_w) lag_q <= 1'b1;

			// EX.
			if (ex_v_q && (ex_cnt_q > 6'd1)) ex_cnt_q <= ex_cnt_q - 6'd1;
			if (ex_v_q) ex_first_q <= 1'b0;
			if (ex_v_q && ex_is_caxi_w && ex_first_q) ex_r30_q <= rf_a_w;
			if (ex_adv_w || ex_kill_w) ex_v_q <= 1'b0;
			redir_gap_q <= ex_adv_w && ex_redir_q && !ex_kill_w && !exc_take_w;
			if ((ex_adv_w && ex_taken_w) || ex_early_w) tgt_pend_q <= 1'b1;
			else if (id_adv_w || exc_take_w || wb_redirect_w) tgt_pend_q <= 1'b0;
			halt_ph_q <= halted_w && !halt_ph_q;
			wake_gap_q <= halted_w && exc_take_w;
			if (ex_adv_w || ex_kill_w || exc_take_w) ex_redir_q <= 1'b0;
			else if (ex_early_w) ex_redir_q <= 1'b1;
			if (id_adv_w) begin
				ex_v_q <= 1'b1;
				ex_tgt_q <= tgt_pend_q;
				ex_pc_q <= id_pc_q;
				ex_iw_q <= id_iw_q[31:16];
				ex_seq_pc_q <= id_pc_q + instr_len_bytes_fn(id_iw_q[31:16]);
				ex_uc_q <= id_uc_w;
				ex_imm_q <= id_imm_w;
				ex_cc_on_q <= id_cc_on_w;
				ex_cc_q <= id_cc_w;
				ex_taken_q <= id_taken_w;
				ex_r1_q <= fwd_r1_w[31:0];
				ex_r2_q <= fwd_r2_w[31:0];
				ex_cnt_q <= ex_cycles_fn(id_ue_w, id_iw_q, fwd_r1_w[31:0]);
				ex_first_q <= 1'b1;
			end

			// ID.
			if (id_adv_w || redirect_w) id_v_q <= 1'b0;
			if (id_load_w) begin
				id_v_q <= 1'b1;
				id_pc_q <= f_pc_w;
				id_iw_q <= f_iw_w;
			end
		end else begin
			// The retire pulse lasts one clock.
			trace_valid_o <= 1'b0;
		end
	end

	// ---------------------------------------------------------------
	// Pins.
	// ---------------------------------------------------------------
	// Data floats and returns at phi1, half a clock after the other bus pins.
	always @(posedge clk_i) begin
		if (cpu_reset_w) hold_data_q <= hldrq_i;
		else if (phi1_i) hold_data_q <= eng_hldak_w;
	end
	wire package_fixed_16_w = reset_i ? siz16b_i : siz16b_q;
	wire package_bus_drive_w = reset_i ? !hldrq_i : !eng_hldak_w;
	wire package_data_drive_w = reset_i ? !hldrq_i : !hold_data_q;
	assign a_o            = reset_i ? {{30{1'b1}}, siz16b_i} : eng_a_w;
	assign be_o           = reset_i ? 4'b1111 : eng_be_w;
	assign st_o           = reset_i ? 2'b11 : eng_st_w;
	assign da_o           = eng_da_w;
	assign mrq_o          = eng_mrq_w;
	assign rw_o           = eng_rw_w;
	assign bcyst_o        = eng_bcyst_w;
	assign adrs_err_n_o   = eng_adrs_err_n_w;
	assign dout_oe_o      = package_data_drive_w && eng_dout_oe_w;
	assign bus_t1_o       = eng_bus_t1_w;
	assign a_oe_o         = package_bus_drive_w;
	assign dout_lane_oe_o = dout_oe_o ?
		(package_fixed_16_w ? 4'b0011 : 4'b1111) : 4'b0000;
	assign be_oe_o        = package_bus_drive_w ?
		(package_fixed_16_w ? 4'b0011 : 4'b1111) : 4'b0000;
	assign st_oe_o        = package_bus_drive_w;
	assign da_oe_o        = package_bus_drive_w;
	assign mrq_oe_o       = package_bus_drive_w;
	assign rw_oe_o        = package_bus_drive_w;
	assign bcyst_oe_o     = package_bus_drive_w;
	assign block_o        = us_lock_w && !eng_hldak_w;
	assign hldak_o        = reset_i ? hldrq_i : eng_hldak_w;

	assign dbg_reg_data_o = rf_a_w;
	assign dbg_pc_o       = arch_pc_q;
	assign dbg_psw_o      = psw_w;
	assign halted_o       = halted_w;
	always @(posedge clk_i or posedge cpu_reset_w) begin
		if (cpu_reset_w) illegal_o <= 1'b0;
		else if (ce_i) illegal_o <= ex_sys_go_w && (ex_kind_w == MK_ILL);
	end
	assign savestate_pause_ready_o = ss_hold_q && !eng_hldak_w && !hold_data_q;
	always @* begin
		savestate_state_rdata_o = (savestate_state_addr_i <= 4'd5) ? ss_sys_word_w : 64'd0;
		if (savestate_state_addr_i == 4'd5) savestate_state_rdata_o[6] = halt_ph_q;
		savestate_mem_rdata_o = ss_gpr_w ? ss_gpr_rdata_w : f_ss_rdata_w;
	end
endmodule
