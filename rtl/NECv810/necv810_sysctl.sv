// Copyright (c) 2026 Jamie Blanks

// NEC V810 system control: PSW, the system registers, CHCW cache mode,
// interrupt and NMI acceptance, address trap, HALT and the fatal state.
//
// The pipe decides when things happen and raises one strobe at a CE. Updates
// arriving at one CE land in program order: WB flags, then the FP PSW bits,
// then the single EX-side event (exception entry, RETI or LDSR, which CLI and SEI use too).
// Exception entry and RETI therefore save or replace a PSW that already holds
// the older instruction's flags.

// The entry decoder ignores PC bit 0.
`ifdef VERILATOR
`verilator_config
lint_off -rule UNUSEDSIGNAL -file "*necv810_exception_entry.sv" -match "*restore_pc_i*"
`verilog
`endif

module necv810_sysctl
#(
	parameter [31:0] RESET_PSW = 32'h0000_8000,
	parameter        VB_EXT    = 1
)
(
	input  wire        clk_i,
	/* verilator lint_off SYNCASYNCNET */
	input  wire        reset_i,
	/* verilator lint_on SYNCASYNCNET */
	input  wire        ce_i,

	input  wire [4:0]  sr_id_i,            // STSR register number
	output wire [31:0] sr_value_o,         // STSR value
	output wire [31:0] psw_o,
	output wire [31:0] reti_pc_o,          // FEPC when PSW.NP, else EIPC
	output wire [31:0] reti_psw_o,         // FEPSW when PSW.NP, else EIPSW, masked

	input  wire        fpsw_we_i,          // FP engine PSW bits 9..0
	input  wire [9:0]  fpsw_mask_i,
	input  wire [9:0]  fpsw_i,

	input  wire        ldsr_we_i,
	input  wire [4:0]  ldsr_id_i,
	input  wire [31:0] ldsr_value_i,

	// An instruction issued: turns on a cache enable left pending by an
	// earlier CE and ends RETI's interrupt hold-off.
	input  wire        issue_i,
	output reg  [1:0]  cache_cmd_o,        // CACHE_CMD_*, held for one CE with valid
	output reg         cache_cmd_valid_o,
	output reg  [6:0]  cache_clear_first_o,
	output reg  [6:0]  cache_clear_last_o,
	output reg  [31:0] cache_sa_o,         // Dump/restore base, 256-byte aligned
	output wire        cache_enable_o,     // Cache lookups allowed
	output wire        cache_pending_o,    // Enabled by CHCW, lookups start after the next issue
	output wire        chcw_cmd_o,         // The LDSR in EX writes CHCW with a clear, dump or restore

	input  wire        exc_take_i,
	input  wire [15:0] exc_code_i,
	input  wire [31:0] exc_restore_pc_i,
	output wire [31:0] exc_handler_o,      // For exc_code_i and the current PSW
	output wire        exc_fatal_o,

	// TRAP: present the TRAP's own PC and vector, then take
	// exc_code_i = trap_code_o, exc_restore_pc_i = trap_restore_pc_o.
	input  wire [4:0]  trap_vector_i,
	input  wire [31:0] trap_pc_i,
	output wire [15:0] trap_code_o,
	output wire [31:0] trap_restore_pc_o,

	input  wire        reti_take_i,
	output wire        reti_defer_o,       // RETI left and nothing has issued since
	// A PSW routine in its last clock: interrupts see the PSW it writes.
	input  wire        end_ldsr_i,

	// Fatal dump: three word writes, word n at address n * 4, then done.
	input  wire [1:0]  fatal_index_i,
	input  wire        fatal_done_i,
	output wire [31:0] fatal_word_o,
	output wire        fatal_o,

	input  wire        irq_valid_i,
	input  wire [3:0]  irq_level_i,
	input  wire        nmi_i,
	input  wire [31:0] boundary_pc_i,
	input  wire        boundary_valid_i,
	input  wire        boundary_ready_i,
	output wire        addr_trap_o,
	input  wire        irq_hold_i,         // IRQ entry must wait, e.g. posted writes busy
	output wire        async_pending_o,
	output wire [15:0] async_code_o,
	output wire        async_is_irq_o,
	output wire        async_is_nmi_o,
	output wire        irq_wait_o,         // An accepted IRQ waits on irq_hold_i; stop issue

	input  wire        halt_i,
	output wire        halted_o,

	// Savestate words 0-5. Word 0 carries pc_i, which the
	// pipe restores itself. Word 5 holds only this unit's bits, zero elsewhere.
	// Writes apply without a CE.
	input  wire [31:0] pc_i,               // Architectural PC, also the fatal dump PC
	input  wire [3:0]  ss_word_addr_i,
	output reg  [63:0] ss_word_o,
	input  wire        ss_word_we_i,
	input  wire [63:0] ss_word_i
);
	/* verilator lint_off UNUSEDSIGNAL */
	`include "necv810_defs.svh"
	/* verilator lint_on UNUSEDSIGNAL */

	localparam [1:0] CACHE_CMD_NONE    = 2'd0;
	localparam [1:0] CACHE_CMD_CLEAR   = 2'd1;
	localparam [1:0] CACHE_CMD_DUMP    = 2'd2;
	localparam [1:0] CACHE_CMD_RESTORE = 2'd3;

	reg [31:0] psw_q;
	reg [31:0] eipc_q;
	reg [31:0] eipsw_q;
	reg [31:0] fepc_q;
	reg [31:0] fepsw_q;
	reg [31:0] ecr_q;
	reg [31:0] adtre_q;
	reg [31:0] sr29_q;
	reg [31:0] sr31_q;
	reg        cache_enable_q;
	reg        cache_pending_q;
	reg        halted_q;
	reg        fatal_q;
	reg [15:0] fatal_code_q;
	reg [31:0] fatal_pc_q;
	reg        nmi_prev_q;
	reg        nmi_pending_q;
	reg        irq_defer_q;

	// Older WB updates in this CE, seen by the EX-side event.
	reg [31:0] psw_wb_v;
	always @* begin
		psw_wb_v = psw_q;
		if (fpsw_we_i)
			psw_wb_v[9:0] = (psw_wb_v[9:0] & ~fpsw_mask_i) | (fpsw_i & fpsw_mask_i);
	end

	wire        entry_fatal_w;
	wire        entry_save_ei_w;
	wire        entry_save_fe_w;
	wire [31:0] entry_saved_pc_w;
	wire [31:0] entry_saved_psw_w;
	wire [31:0] entry_ecr_w;
	wire        entry_psw_write_w;
	wire [31:0] entry_psw_w;
	wire        entry_pc_write_w;
	wire        entry_clear_nmi_w;
	wire        entry_clear_halt_w;
	wire        entry_is_nmi_w;
	wire        entry_is_irq_w;

	necv810_exception_entry u_entry (
		.code_i              (exc_code_i),
		.restore_pc_i        (exc_restore_pc_i),
		.psw_i               (psw_wb_v),
		.ecr_i               (ecr_q),
		.fatal_o             (entry_fatal_w),
		.save_ei_o           (entry_save_ei_w),
		.save_fe_o           (entry_save_fe_w),
		.saved_pc_o          (entry_saved_pc_w),
		.saved_psw_o         (entry_saved_psw_w),
		.next_ecr_o          (entry_ecr_w),
		.psw_write_o         (entry_psw_write_w),
		.next_psw_o          (entry_psw_w),
		.pc_write_o          (entry_pc_write_w),
		.handler_o           (exc_handler_o),
		.clear_nmi_pending_o (entry_clear_nmi_w),
		.clear_halt_o        (entry_clear_halt_w),
		.is_nmi_o            (entry_is_nmi_w),
		.is_interrupt_o      (entry_is_irq_w)
	);

	assign exc_fatal_o = entry_fatal_w;
	assign psw_o = psw_q;
	assign reti_pc_o = psw_q[15] ? fepc_q : eipc_q;
	assign reti_psw_o = (psw_q[15] ? fepsw_q : eipsw_q) & PSW_WRITE_MASK;
	assign sr_value_o = sysreg_read_fn(sr_id_i, eipc_q, eipsw_q, fepc_q, fepsw_q, ecr_q, psw_q,
		adtre_q, sr29_q, sr31_q, cache_enable_q || cache_pending_q);

	// TRAP saves its own PC when PSW.EP is set, else the next instruction's.
	assign trap_code_o = 16'hffa0 + {11'd0, trap_vector_i};
	assign trap_restore_pc_o = psw_q[14] ? {trap_pc_i[31:1], 1'b0} : ({trap_pc_i[31:1], 1'b0} + 32'd2);

	assign fatal_word_o = fatal_wdata_fn(fatal_index_i, fatal_code_q, psw_q, fatal_pc_q);
	assign fatal_o = fatal_q;
	assign halted_o = halted_q;
	assign cache_enable_o = cache_enable_q;
	assign cache_pending_o = cache_pending_q;
	assign reti_defer_o = irq_defer_q;

	// NMI > IRQ > address trap. A halted CPU wakes only for NMI or IRQ, and so
	// do mid-instruction abort points (async_is_nmi_o or async_is_irq_o).
	reg [31:0] psw_chk_v;
	always @* begin
		psw_chk_v = psw_q;
		if (end_ldsr_i && (ldsr_id_i == SYS_PSW)) psw_chk_v = ldsr_value_i;
	end
	wire       irq_accept_w = interrupt_accepted_fn(irq_valid_i, irq_level_i, psw_chk_v);
	wire       irq_now_w = irq_accept_w && !irq_defer_q;
	wire [3:0] irq_level_w = irq_level_i;
	wire       nmi_edge_w = nmi_i && !nmi_prev_q;
	wire       nmi_accept_w = nmi_accepted_fn(nmi_pending_q || nmi_edge_w, psw_chk_v);
	wire       addr_trap_w = boundary_valid_i && !halted_q && psw_q[13] &&
		(boundary_pc_i[31:1] == adtre_q[31:1]);
	wire [1:0] async_kind_w = fatal_q ? ASYNC_EXC_NONE :
		async_exception_kind_fn(addr_trap_w && boundary_ready_i, nmi_accept_w, irq_now_w && !irq_hold_i);

	assign addr_trap_o = addr_trap_w;
	assign async_pending_o = (async_kind_w != ASYNC_EXC_NONE);
	assign async_code_o = async_exception_code_fn(async_kind_w, irq_level_w);
	assign async_is_irq_o = (async_kind_w == ASYNC_EXC_IRQ);
	assign async_is_nmi_o = (async_kind_w == ASYNC_EXC_NMI);
	assign irq_wait_o = !fatal_q && irq_now_w && irq_hold_i;

	// CHCW: ICC (bit 0) clears CEN (19:8) tags from entry CEC (31:20), ICD (bit 4)
	// dumps and ICR (bit 5) restores at SA (31:8). ICE (bit 1) sets the mode in
	// any accepted write; an unsupported mix keeps the current mode.
	wire [11:0] chcw_count_w = ldsr_value_i[19:8];
	wire        chcw_clear_w = ldsr_value_i[0] && !ldsr_value_i[4] && !ldsr_value_i[5] &&
		(ldsr_value_i[31:20] < 12'd128) && (chcw_count_w != 12'd0);
	wire        chcw_xfer_w = !ldsr_value_i[0] && (ldsr_value_i[4] ^ ldsr_value_i[5]);
	wire        chcw_ice_w = (chcw_clear_w || chcw_xfer_w || !chcw_invalid_combination_fn(ldsr_value_i)) ?
		ldsr_value_i[1] : (cache_enable_q || cache_pending_q);
	wire        chcw_clip_w = ({1'b0, ldsr_value_i[31:20]} + {1'b0, chcw_count_w}) > 13'd128;
	assign chcw_cmd_o = (ldsr_id_i == SYS_CHCW) && (chcw_clear_w || chcw_xfer_w);

	// One EX-side event per CE; the earlier one here wins if the pipe sends two.
	wire ex_exc_w  = exc_take_i;
	wire ex_reti_w = reti_take_i && !ex_exc_w;
	wire ex_ldsr_w = ldsr_we_i && !ex_exc_w && !reti_take_i;
	wire chcw_w    = ex_ldsr_w && (ldsr_id_i == SYS_CHCW);

	reg [31:0] psw_next_v;
	reg        cache_enable_v;
	reg        cache_pending_v;

	always @* begin
		psw_next_v = psw_wb_v;
		if (ex_exc_w && entry_psw_write_w) psw_next_v = entry_psw_w;
		if (ex_reti_w) psw_next_v = reti_psw_o;
		if (ex_ldsr_w && (ldsr_id_i == SYS_PSW)) psw_next_v = ldsr_value_i;

		cache_enable_v = cache_enable_q;
		cache_pending_v = cache_pending_q;
		if (issue_i && cache_pending_q) begin
			cache_enable_v = 1'b1;
			cache_pending_v = 1'b0;
		end
		// Enabling waits for the next issue; disabling is immediate.
		if (chcw_w) begin
			if (!chcw_ice_w) begin
				cache_enable_v = 1'b0;
				cache_pending_v = 1'b0;
			end else if (!cache_enable_v) begin
				cache_pending_v = 1'b1;
			end
		end
	end

	always @(posedge clk_i or posedge reset_i) begin
		if (reset_i) begin
			psw_q <= RESET_PSW;
			// NEC leaves these undefined; zero keeps FPGA starts repeatable.
			eipc_q <= 32'd0;
			eipsw_q <= 32'd0;
			fepc_q <= 32'd0;
			fepsw_q <= 32'd0;
			ecr_q <= 32'h0000_fff0;
			adtre_q <= 32'd0;
			sr29_q <= 32'd0;
			sr31_q <= 32'd0;
			cache_enable_q <= 1'b0;
			cache_pending_q <= 1'b0;
			cache_cmd_o <= CACHE_CMD_NONE;
			cache_cmd_valid_o <= 1'b0;
			cache_clear_first_o <= 7'd0;
			cache_clear_last_o <= 7'd0;
			cache_sa_o <= 32'd0;
			halted_q <= 1'b0;
			fatal_q <= 1'b0;
			fatal_code_q <= 16'd0;
			fatal_pc_q <= 32'd0;
			nmi_pending_q <= 1'b0;
			irq_defer_q <= 1'b0;
		end else if (ss_word_we_i) begin
			case (ss_word_addr_i)
				4'd0: psw_q <= ss_word_i[63:32] & PSW_WRITE_MASK;
				4'd1: begin
					eipc_q <= {ss_word_i[31:1], 1'b0};
					eipsw_q <= ss_word_i[63:32] & PSW_WRITE_MASK;
				end
				4'd2: begin
					fepc_q <= {ss_word_i[31:1], 1'b0};
					fepsw_q <= ss_word_i[63:32] & PSW_WRITE_MASK;
				end
				4'd3: begin
					ecr_q <= ss_word_i[31:0];
					adtre_q <= {ss_word_i[63:33], 1'b0};
				end
				4'd4: begin
					sr29_q <= ss_word_i[31:0];
					sr31_q <= ss_word_i[63:32];
				end
				4'd5: begin
					halted_q <= ss_word_i[0];
					cache_enable_q <= ss_word_i[1];
					cache_pending_q <= ss_word_i[2];
					irq_defer_q <= ss_word_i[3];
					nmi_pending_q <= ss_word_i[4];
					fatal_q <= ss_word_i[39];
				end
				default: begin end
			endcase
		end else if (ce_i) begin
			psw_q <= psw_next_v & PSW_WRITE_MASK;
			cache_enable_q <= cache_enable_v;
			cache_pending_q <= cache_pending_v;
			cache_cmd_valid_o <= 1'b0;

			if (nmi_edge_w) nmi_pending_q <= 1'b1;
			if (issue_i) irq_defer_q <= 1'b0;
			if (halt_i || fatal_done_i) halted_q <= 1'b1;

			if (ex_exc_w) begin
				if (entry_fatal_w) begin
					fatal_q <= 1'b1;
					fatal_code_q <= exc_code_i;
					fatal_pc_q <= pc_i;
				end else begin
					ecr_q <= entry_ecr_w;
					if (entry_save_fe_w) begin
						fepc_q <= entry_saved_pc_w;
						fepsw_q <= entry_saved_psw_w;
					end else if (entry_save_ei_w) begin
						eipc_q <= entry_saved_pc_w;
						eipsw_q <= entry_saved_psw_w;
					end
					if (entry_clear_halt_w) halted_q <= 1'b0;
					if (entry_clear_nmi_w) nmi_pending_q <= 1'b0;
				end
			end
			if (ex_reti_w) begin
				halted_q <= 1'b0;
				irq_defer_q <= 1'b1;
			end
			if (ex_ldsr_w) begin
				case (ldsr_id_i)
					SYS_EIPC:  eipc_q <= {ldsr_value_i[31:1], 1'b0};
					SYS_EIPSW: eipsw_q <= ldsr_value_i & PSW_WRITE_MASK;
					SYS_FEPC:  fepc_q <= {ldsr_value_i[31:1], 1'b0};
					SYS_FEPSW: fepsw_q <= ldsr_value_i & PSW_WRITE_MASK;
					SYS_ADTRE: adtre_q <= {ldsr_value_i[31:1], 1'b0};
					SYS_SR29:  sr29_q <= ldsr_value_i;
					SYS_SR31:  sr31_q <= ldsr_value_i;
					// PSW and CHCW are handled above; ECR, PIR, TKCW and SR30 are read-only.
					default: begin end
				endcase
			end
			if (chcw_w && (chcw_clear_w || chcw_xfer_w)) begin
				cache_cmd_valid_o <= 1'b1;
				cache_cmd_o <= chcw_clear_w ? CACHE_CMD_CLEAR :
					ldsr_value_i[4] ? CACHE_CMD_DUMP : CACHE_CMD_RESTORE;
				cache_clear_first_o <= ldsr_value_i[26:20];
				cache_clear_last_o <= chcw_clip_w ? 7'd127 : (ldsr_value_i[26:20] + chcw_count_w[6:0] - 7'd1);
				cache_sa_o <= {ldsr_value_i[31:8], 8'd0};
			end
		end
	end

	// Follows the NMI level through reset so release cannot make an edge.
	always @(posedge clk_i) begin
		if (reset_i) begin
			nmi_prev_q <= nmi_i;
		end else if (ss_word_we_i) begin
			if (ss_word_addr_i == 4'd5) nmi_prev_q <= ss_word_i[5];
		end else if (ce_i) begin
			nmi_prev_q <= nmi_i;
		end
	end

	always @* begin
		case (ss_word_addr_i)
			4'd0: ss_word_o = {psw_q, pc_i};
			4'd1: ss_word_o = {eipsw_q, eipc_q};
			4'd2: ss_word_o = {fepsw_q, fepc_q};
			4'd3: ss_word_o = {adtre_q, ecr_q};
			4'd4: ss_word_o = {sr31_q, sr29_q};
			4'd5: ss_word_o = {24'd0, fatal_q, 33'd0,
				nmi_prev_q, nmi_pending_q, irq_defer_q, cache_pending_q, cache_enable_q, halted_q};
			default: ss_word_o = 64'd0;
		endcase
	end

	/* verilator lint_off UNUSEDSIGNAL */
	wire unused_ok_w = &{1'b0, entry_pc_write_w, entry_is_nmi_w, entry_is_irq_w, boundary_pc_i[0], trap_pc_i[0], ss_word_i};
	/* verilator lint_on UNUSEDSIGNAL */

endmodule
