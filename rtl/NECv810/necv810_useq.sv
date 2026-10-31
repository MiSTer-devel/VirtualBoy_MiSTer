// Copyright (c) 2026 Jamie Blanks

// NEC V810 micro-sequencer, in EX, for the microprogrammed instructions and
// the exception entry.
//
//   decoder entry -> PLA -> (registered at issue) --+
//   exception entry, HALT wake ---------------------+
//                                                   v
//   word -> jmp ? next : upc + 1 -------------> ROM -> word (registered)
//    |
//    +-> sources -> ALU / funnel shift / multiply step / divide step -> T0, T1, T2, result
//    +-> FP helper: class tests, exponent unit, leading zeros, sign, FPR/FUD
//    +-> bus read / write of the instruction's address, waits, lock
//
// One word runs per CE clock, and its op drives one unit. The ROM read is
// registered, so each clock picks the next word's address. Entry takes two
// clocks: the PLA address is chosen in the instruction's second EX clock and
// its first word runs in the third. END is the last clock; it holds until the
// pipe lets the instruction go, and the pipe then commits the result to reg2,
// or to PSW through the LDSR write.
//
// The exception entry starts on the clock the pipe takes an exception, with no
// instruction in EX, and runs to its last step (exc_o).
//
// A bus word holds until its condition is met: older work done, the bus idle,
// or its count run. A word that starts an access holds while an earlier one
// waits for the bus. CAXI's read lands in T0; a write sends T1.
//
// Bit strings keep r26-r30 in cursor registers, read and written back one a
// clock through the register file ports:
//
//   r30 -> sa (source word), rp (next source read)   r29 -> da (destination word, or count)
//   r28 -> ln (length)   r27 -> so (source offset)   r26 -> do (destination offset)
//
// Source words shift into a two-word window {a1, a0}; a word read ahead waits
// in T0 until its step ends. A destination word read lands merged into T1: the
// window, through the funnel shifter to the destination offset, combined with
// it under the step's mask. A search orders a1 through the funnel shifter from
// its offset and takes the first hit from the FP leading-zero count.
//
// The FP routines report at END: the result and whether reg2 takes it, the PSW
// bits (CY/OV/S/Z by the END word's mode, the sticky FP bits set) and a trap.
module necv810_useq
#(
	parameter VB_EXT = 1
)
(
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        ce_i,
	input  wire        issue_i,      // the instruction in ID moves to EX
	input  wire [5:0]  id_ue_i,      // its decode entry
	input  wire [5:0]  id_sub_i,     // and FP sub-opcode
	input  wire        id_div0_i,    // it divides by zero
	input  wire        ex_v_i,       // an instruction is in EX
	input  wire        ex_first_i,   // and this is its first EX clock
	input  wire        ex_stop_i,    // it leaves EX or is dropped at this CE
	input  wire        ent_i,        // start the exception entry
	input  wire        wake_i,       // start it from HALT, where it parked
	input  wire        quick_i,      // with ent_i: the pipe has nothing left to stop
	input  wire [31:0] r1_i,         // reg1 and reg2 as EX holds them
	input  wire [31:0] r2_i,
	input  wire [31:0] psw_i,
	input  wire [31:0] r30_i,
	input  wire        drained_i,    // older instructions retired, the bus idle, the write buffer empty
	input  wire        older_i,      // older instructions still in MEM or WB
	input  wire        wbuf_i,       // posted stores still buffered or on the bus
	input  wire        bus_accept_i, // the bus takes the request
	input  wire        bus_done_i,   // the access it took completes
	input  wire [31:0] bus_rdata_i,
	input  wire [31:0] ea_i,         // the instruction's data address
	input  wire [2:0]  id_bs_i,      // bit-string kind in ID: instruction bits {20:19, 16}
	input  wire [4:0]  ex_bs_i,      // and the bit-string subop in EX, bits 20:16
	input  wire        irq_i,        // an interrupt waits
	input  wire [31:0] rf_i,         // register file read port A
	input  wire        rf_busy_i,    // MEM holds a writer of r26-r30
	input  wire        rf_free_i,    // the WB write port is free this clock
	output wire        routine_o,    // the instruction in EX has a micro-routine
	output wire        end_o,        // its END word runs this clock
	output wire [31:0] result_o,     // what it writes to reg2 or PSW
	output wire        psw_we_o,     // the result goes to PSW
	output wire [3:0]  flags_o,      // {CY, OV, S, Z} of the last SUB
	output wire        bus_req_o,    // a word access of the instruction's address
	output wire        bus_write_o,
	output wire [31:0] bus_addr_o,
	output wire [31:0] bus_wdata_o,
	output wire        lock_o,
	output wire        rf_rd_o,      // read port A reads rf_addr_o this clock
	output wire        rf_we_o,      // and the WB port writes rf_addr_o
	output wire [4:0]  rf_addr_o,
	output wire [31:0] rf_wdata_o,
	output wire        found_o,      // the search found its bit
	output wire        abort_o,      // the routine ends for an interrupt, to run again
	output wire        ent_busy_o,   // the exception entry runs
	output wire        exc_o,        // its last step: the pipe enters the handler next
	output wire        fp_we_o,      // FP routine: reg2 takes the result
	output wire [9:0]  fp_flags_o,   // PSW bits 9:0 and which of them are written
	output wire [9:0]  fp_fmask_o,
	output wire        fp_exc_o,     // and it traps
	output wire [15:0] fp_code_o
);
	/* verilator lint_off UNUSEDSIGNAL */
	`include "necv810_defs.svh"
	/* verilator lint_on UNUSEDSIGNAL */
	`include "necv810_urom.svh"

	reg                run_q;        // the routine's words are running
	reg                ent_q;        // and it is the exception entry
	reg  [9:0]         pla_q;        // PLA entry of the instruction in EX
	reg                alt_q;        // it is TRNC.SW or DIVF.S
	reg                bs_q;         // it is a bit string
	reg  [8:0]         upc_q;        // address of the word in uw_q
	reg  [UW_BITS-1:0] uw_q;         // the word running this clock
	reg  [31:0]        t0_q;
	reg  [31:0]        t1_q;
	reg  [31:0]        t2_q;
	reg  [31:0]        res_q;
	reg                psw_we_q;
	reg                r2w_q;        // the routine wrote its reg2 result
	reg  [3:0]         fl_q;
	reg                req_q;        // a bus request not yet taken
	reg                wr_q;
	reg                out_q;        // a taken access not yet done
	reg                lock_q;
	reg  [4:0]         held_q;       // clocks the word has held
	reg  [1:0]         rq_land_q;    // where the waiting request's read data goes
	reg  [1:0]         out_land_q;   // and the taken access's
	reg  [31:0]        ad_q;         // the waiting request's address and data
	reg  [31:0]        wd_q;
	// Bit-string cursors and source window.
	reg  [31:0]        sa_q;
	reg  [31:0]        rp_q;
	reg  [31:0]        da_q;
	reg  [31:0]        ln_q;
	reg  [4:0]         so_q;
	reg  [4:0]         do_q;
	reg  [31:0]        a0_q;
	reg  [31:0]        a1_q;
	reg                found_q;
	reg                abort_q;
	reg                rsn_q;        // the word's read ahead has started

	assign routine_o = ex_v_i && pla_q[9];

	wire [5:0]  op_w   = uw_q[UW_OP +: 6];
	wire [2:0]  dst_w  = uw_q[UW_DST +: 3];
	wire [4:0]  imm_w  = uw_q[UW_IMM +: 5];
	wire [4:0]  cond_w = uw_q[UW_COND +: 5];
	// The bus and the bit-string cursor unit take the low bits of their ops;
	// bit-string ops in group 3 also wait for the bus.
	wire [2:0]  bus_w  = (op_w[5:3] == 3'o1) ? op_w[2:0] : (op_w[5:3] == 3'o3) ? UB_WAIT : 3'd0;
	wire [2:0]  bs_w   = (op_w[5:4] == 2'b01) ? op_w[2:0] : 3'd0;
	wire [31:0] k_w    = ukon_fn(imm_w[3:0]);

	// Bit-string decode: searches are subops 0-3, bit 0 down, bit 1 the bit sought.
	wire        search_w = (ex_bs_i[4:3] == 2'b00);
	wire        down_w = search_w && ex_bs_i[0];
	// The step: bits to the end of the destination word, or for a search of
	// the source word, or what is left.
	wire [5:0]  astep_w = bs_step_fn(ln_q, do_q);
	wire [5:0]  sstep_w = bs_search_step_fn(ln_q, so_q, down_w);
	wire [5:0]  step_w = search_w ? sstep_w : astep_w;
	// A read ahead is needed while more of the string lies past this step.
	wire        ahead_w = (ln_q != {26'd0, step_w});
	// A destination write in the same word leaves ln - step.
	wire        left_w = (bs_w == UBS_WR) ? ahead_w : (ln_q != 32'd0);
	wire        stop_w = !left_w || found_q || irq_i;
	reg         bs_cond_v;
	always @* begin
		case (cond_w)
			C_LEN0:  bs_cond_v = (ln_q == 32'd0);
			C_MORE:  bs_cond_v = !stop_w;
			C_STOP:  bs_cond_v = stop_w;
			C_SGT:   bs_cond_v = (so_q > do_q);
			C_SLT:   bs_cond_v = (so_q < do_q);
			C_FOUND: bs_cond_v = found_q;
			C_FULL:  bs_cond_v = (do_q == 5'd0);
			C_LAST:  bs_cond_v = !ahead_w;
			C_SCH0:  bs_cond_v = !ex_bs_i[1];
			C_ABORT: bs_cond_v = abort_q;
			C_FPART: bs_cond_v = down_w ? (so_q != 5'd31) : (so_q != 5'd0);
			C_DOWN:  bs_cond_v = down_w;
			default: bs_cond_v = 1'b1;
		endcase
	end

	wire        bs_issue_w = (bs_w == UBS_RS) || (bs_w == UBS_RD) || (bs_w == UBS_WR) ||
		((bs_w == UBS_RSN) && ahead_w && !rsn_q);
	// A word waits for older work, an idle bus or its count. A word that starts
	// an access waits for a free request slot, except a wait word, which starts
	// its access first and then waits for it. A register read waits for a
	// writer still in MEM; a register write waits for the WB port, and in a
	// wait word for a bus idle from the start of the clock.
	//
	// Only a wait word's advance depends on an access completing this clock;
	// a word's own work, a new request and END never do, which keeps READY off
	// the request, the datapath, the ROM address and the pipe.
	wire        idle_w = !req_q && (!out_q || bus_done_i);
	wire        rf_go_w = rf_free_i && ((bus_w != UB_WAIT) || (!req_q && !out_q));
	wire        hold_w = (bus_w == UB_DRAIN) ? !drained_i : (bus_w == UB_COUNT) ? (held_q != imm_w) :
		(bus_w == UB_SETTLE) ? (imm_w[0] ? wbuf_i : older_i) : 1'b0;
	wire        work_w = run_q && !hold_w && (!bs_issue_w || !req_q) && ((bs_w != UBS_WB) || rf_go_w) &&
		((bs_w != UBS_LD) || !rf_busy_i);
	wire        wait_w = (bus_w == UB_WAIT) && (bs_w != UBS_WB) && (!idle_w || bs_issue_w);
	wire        go_w = work_w && !wait_w;
	// The routine ends only with its accesses done before this clock.
	wire        end_w = run_q && uw_q[UW_END] && !req_q && !out_q;
	wire        issue_w = run_q && ((bus_w == UB_WAIT) ? (bs_issue_w && !req_q) :
		(work_w && ((bus_w == UB_READ) || (bus_w == UB_WRITE) || bs_issue_w)));
	wire        issue_wr_w = (bus_w == UB_WRITE) || (bs_w == UBS_WR);
	wire [1:0]  issue_land_w = (bus_w == UB_READ) ? UL_T0 : (bs_w == UBS_RD) ? UL_MERGE :
		((bs_w == UBS_RS) || ((bs_w == UBS_RSN) && search_w)) ? UL_SRC : (bs_w == UBS_RSN) ? UL_T0 : UL_NONE;
	wire [31:0] issue_addr_w = ((bs_w == UBS_RS) || (bs_w == UBS_RSN)) ? rp_q :
		((bs_w == UBS_RD) || (bs_w == UBS_WR)) ? da_q : ea_i;
	wire        fp_cond_w;
	wire        fp_fpr_w;
	wire        fp_fud_w;
	wire [1:0]  fp_zcls_w;
	wire        jump_w = (cond_w != C_NEXT) && ((cond_w != C_EQ) || fl_q[0]) && fp_cond_w && bs_cond_v;

	wire        dispatch_w = routine_o && !ex_first_i && !run_q;
	// A zero dispatch lands on next + {reg2 zero, reg1 zero}.
	wire [8:0]  target_w = uw_q[UW_NEXT +: 9] | ((cond_w == C_ZANY) ? {7'd0, fp_zcls_w} : 9'd0);
	// The ROM address assumes the word goes; whether it does only enables the
	// word register. Idle, it holds the PLA entry's first word.
	wire [8:0]  rom_addr_w = !run_q ? pla_q[8:0] : jump_w ? target_w : (upc_q + 9'd1);
	wire        advance_w = dispatch_w || (go_w && !uw_q[UW_END]);

	wire [31:0] fp_pk_w;

	reg  [31:0] a_v;
	reg  [31:0] b_v;
	always @* begin
		case (uw_q[UW_A +: 3])
			A_R1:    a_v = r1_i;
			A_R2:    a_v = r2_i;
			A_T0:    a_v = t0_q;
			A_T1:    a_v = t1_q;
			A_PSW:   a_v = psw_i;
			A_PK:    a_v = fp_pk_w;
			A_R30:   a_v = r30_i;
			default: a_v = 32'd0;
		endcase
		case (uw_q[UW_B +: 3])
			B_R1:    b_v = r1_i;
			B_R2:    b_v = r2_i;
			B_T0:    b_v = t0_q;
			B_T1:    b_v = t1_q;
			B_K:     b_v = k_w;
			B_T2:    b_v = t2_q;
			B_PK:    b_v = fp_pk_w;
			default: b_v = 32'd0;
		endcase
	end

	wire [5:0]  fp_sc_w;
	wire        fp_sg_w;

	function [31:0] rev32_fn;
		input [31:0] v;
		integer i;
		begin
			for (i = 0; i < 32; i = i + 1) rev32_fn[i] = v[31 - i];
		end
	endfunction

	// One funnel shifter: low word of {hi, lo} >> n. FP shifts also OR the bits
	// of lo shifted out into bit 0. A normalise by SC shifts {0, a} right when
	// SC is positive and {a, 0} right by 32 + SC (a left shift) when negative.
	// A bit string has it to itself: the arithmetic ops align {a1, a0} by src
	// ofs - dst ofs; a search brings a1's bit at the offset to bit 0 going up,
	// or to bit 31 going down ({a1, 0} >> ofs + 1).
	wire        norm_w = (op_w == F_NORM);
	wire        shk_w = (op_w == F_SHRK);
	wire        bs_dn_w = down_w && (so_q != 5'd31);
	reg  [31:0] fs_hi_w;
	reg  [31:0] fs_lo_w;
	reg  [4:0]  fs_n_w;
	always @* begin
		fs_hi_w = (norm_w && !fp_sc_w[5]) ? 32'd0 : a_v;
		fs_lo_w = norm_w ? (fp_sc_w[5] ? 32'd0 : a_v) : b_v;
		fs_n_w = norm_w ? fp_sc_w[4:0] : imm_w;
		if (bs_q) begin
			fs_hi_w = (!search_w || bs_dn_w) ? a1_q : 32'd0;
			fs_lo_w = !search_w ? a0_q : bs_dn_w ? 32'd0 : a1_q;
			fs_n_w = !search_w ? (so_q - do_q) : down_w ? (so_q + 5'd1) : so_q;
		end
	end
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] fs_w = {fs_hi_w, fs_lo_w} >> fs_n_w;
	/* verilator lint_on UNUSEDSIGNAL */
	wire        fs_sticky_w = ((fs_lo_w & ~(32'hffff_ffff << fs_n_w)) != 32'd0) && (norm_w || shk_w);
	wire [31:0] fsr_w = {fs_w[31:1], fs_w[0] | fs_sticky_w};

	// Arithmetic step: the aligned window under the step's mask, merged into
	// the destination word as it is read.
	wire [31:0] aligned_w = (so_q == do_q) ? a1_q : fs_w[31:0];
	wire [31:0] mask_w = bit_mask32_fn(astep_w) << do_q;
	wire [31:0] merged_w = bitstr_merge_aligned_fn(ex_bs_i[2:0], aligned_w & mask_w, bus_rdata_i, mask_w);
	wire [5:0]  asum_do_w = {1'b0, do_q} + astep_w;
	wire [5:0]  asum_so_w = {1'b0, so_q} + astep_w;
	// Search step: the bits sought, first at bit 0, reversed so the FP
	// leading-zero count gives the bits skipped; a hit moves past its bit.
	wire [31:0] sord_w = down_w ? rev32_fn(fs_w[31:0]) : fs_w[31:0];
	wire [31:0] smatch_w = (ex_bs_i[1] ? sord_w : ~sord_w) & bit_mask32_fn(sstep_w);
	wire        sfound_w = (smatch_w != 32'd0);
	wire [5:0]  fp_lz_w;
	wire [5:0]  skip_w = sfound_w ? fp_lz_w : sstep_w;
	wire [5:0]  adv_w = sfound_w ? (fp_lz_w + 6'd1) : sstep_w;

	reg  [31:0] rf_wdata_v;
	always @* begin
		case (imm_w)
			5'd26:   rf_wdata_v = {27'd0, do_q};
			5'd27:   rf_wdata_v = {27'd0, so_q};
			5'd28:   rf_wdata_v = ln_q;
			5'd29:   rf_wdata_v = da_q;
			default: rf_wdata_v = sa_q;
		endcase
	end

	// One radix-16 multiply step: each of b's four low bits adds the
	// multiplicand (reg2, or T2 for MULF) at its weight.
	wire [31:0] mcand_w = (op_w == F_MSTEP) ? t2_q : r2_i;
	wire [35:0] mstep_w = {4'd0, a_v} +
		({36{b_v[0]}} & {4'd0, mcand_w}) +
		({36{b_v[1]}} & {3'd0, mcand_w, 1'b0}) +
		({36{b_v[2]}} & {2'd0, mcand_w, 2'd0}) +
		({36{b_v[3]}} & {1'b0, mcand_w, 3'd0});

	// One 33-bit adder: SUB, and the FP ops' add, negate forms, rounding
	// increment and the divide step's trial subtraction.
	wire        rnd_up_w = a_v[2] && (a_v[1] || a_v[0] || a_v[3]);
	wire        dstep_w = (op_w == F_DSTEP);
	reg  [32:0] ad_a_v;
	reg  [32:0] ad_b_v;
	reg         ad_sub_v;
	always @* begin
		ad_a_v = {1'b0, a_v};
		ad_b_v = {1'b0, b_v};
		ad_sub_v = (op_w == O_SUB);
		case (op_w)
			F_ABSS: begin
				ad_a_v = 33'd0;
				ad_b_v = {1'b0, a_v};
				ad_sub_v = a_v[31];
			end
			F_CNEG: begin
				ad_a_v = 33'd0;
				ad_b_v = {1'b0, a_v};
				ad_sub_v = fp_sg_w;
			end
			F_SMTC: begin
				ad_a_v = 33'd0;
				ad_b_v = {2'd0, a_v[30:0]};
				ad_sub_v = a_v[31];
			end
			F_RND:   ad_b_v = {29'd0, rnd_up_w, 3'd0};
			F_DST0,
			F_DSTEP: begin
				ad_a_v = dstep_w ? {t0_q, t1_q[31]} : {1'b0, t0_q};
				ad_sub_v = 1'b1;
			end
			default: ;
		endcase
	end
	wire [33:0] ad_w = {1'b0, ad_a_v} + {1'b0, ad_b_v ^ {33{ad_sub_v}}} + {33'd0, ad_sub_v};
	wire [32:0] sub_w = ad_w[32:0];
	// Divide: a quotient bit when the trial subtraction does not borrow.
	wire        dq_w = ad_w[33];

	reg  [31:0] fpa_v;
	always @* begin
		case (op_w)
			F_ADD, F_ABSS, F_CNEG, F_SMTC, F_RND:
				fpa_v = ad_w[31:0];
			F_XOR:   fpa_v = a_v ^ b_v;
			F_UNPK:  fpa_v = {5'd0, 1'b1, a_v[22:0], 3'd0};
			F_MANT:  fpa_v = {9'd0, a_v[25:3]};
			F_NORM,
			F_SHRK:  fpa_v = fsr_w;
			F_INEX:  fpa_v = a_v;
			F_MSTEP: fpa_v = mstep_w[35:4];
			F_DST0,
			F_DSTEP: fpa_v = dq_w ? ad_w[31:0] : ad_a_v[31:0];
			default: fpa_v = 32'd0;
		endcase
	end
	// The FP mantissa ops run on this ALU, shifter and adder.
	wire        fp_alu_w = (op_w[5:4] == 2'b10);
	wire        inexact_w = work_w && ((op_w == F_RND) || (op_w == F_INEX)) && (a_v[2:0] != 3'd0);

	reg  [31:0] alu_v;
	always @* begin
		case (op_w)
			O_PASS:    alu_v = a_v;
			O_AND:     alu_v = a_v & b_v;
			O_OR:      alu_v = a_v | b_v;
			O_FSR:     alu_v = fsr_w;
			O_MSTEP:   alu_v = mstep_w[35:4];
			O_MSIGN:   alu_v = a_v - ({32{b_v[0]}} & r2_i);
			O_SUB:     alu_v = sub_w[31:0];
			default:   alu_v = 32'd0;
		endcase
		if (fp_alu_w) alu_v = fpa_v;
	end

	always @(posedge clk_i or posedge reset_i) begin
		if (reset_i) begin
			run_q <= 1'b0;
			ent_q <= 1'b0;
			pla_q <= 10'd0;
			alt_q <= 1'b0;
			bs_q <= 1'b0;
			upc_q <= 9'd0;
			t0_q <= 32'd0;
			t1_q <= 32'd0;
			t2_q <= 32'd0;
			res_q <= 32'd0;
			psw_we_q <= 1'b0;
			r2w_q <= 1'b0;
			fl_q <= 4'd0;
			req_q <= 1'b0;
			wr_q <= 1'b0;
			out_q <= 1'b0;
			lock_q <= 1'b0;
			held_q <= 5'd0;
			rq_land_q <= UL_NONE;
			out_land_q <= UL_NONE;
			ad_q <= 32'd0;
			wd_q <= 32'd0;
			sa_q <= 32'd0;
			rp_q <= 32'd0;
			da_q <= 32'd0;
			ln_q <= 32'd0;
			so_q <= 5'd0;
			do_q <= 5'd0;
			a0_q <= 32'd0;
			a1_q <= 32'd0;
			found_q <= 1'b0;
			abort_q <= 1'b0;
			rsn_q <= 1'b0;
		end else if (ce_i) begin
			if (ent_i) upc_q <= quick_i ? UA_QUICK : UA_ENTRY;
			else if (wake_i) upc_q <= UA_WAKE;
			else if (advance_w) upc_q <= rom_addr_w;
			held_q <= (work_w || !run_q) ? 5'd0 : (held_q + 5'd1);
			if (issue_i) begin
				pla_q <= upla_fn(id_ue_i, id_div0_i, id_sub_i, id_bs_i);
				alt_q <= (id_sub_i == SUBOP_TRNCSW) || (id_sub_i == SUBOP_DIVF);
				bs_q <= (id_ue_i == UADDR_BITSTR);
			end
			if (ent_i || wake_i) begin
				run_q <= 1'b1;
				ent_q <= 1'b1;
			end else if (ent_q) begin
				if (exc_o) begin
					run_q <= 1'b0;
					ent_q <= 1'b0;
				end
			end else if (ex_stop_i) begin
				run_q <= 1'b0;
			end else if (dispatch_w) begin
				run_q <= 1'b1;
			end
			if (dispatch_w || ent_i || wake_i) begin
				psw_we_q <= 1'b0;
				r2w_q <= 1'b0;
				found_q <= 1'b0;
				abort_q <= 1'b0;
			end

			// One access on the bus and one request waiting; a request may be
			// taken as the access before it completes.
			if (bus_req_o && bus_accept_i) begin
				req_q <= 1'b0;
				out_land_q <= req_q ? rq_land_q : issue_land_w;
			end else if (issue_w) begin
				req_q <= 1'b1;
				wr_q <= issue_wr_w;
				rq_land_q <= issue_land_w;
				ad_q <= issue_addr_w;
				wd_q <= t1_q;
			end
			out_q <= (out_q && !bus_done_i) || (bus_req_o && bus_accept_i);
			if (out_q && bus_done_i) begin
				case (out_land_q)
					UL_T0:    t0_q <= bus_rdata_i;
					UL_SRC: begin
						a0_q <= a1_q;
						a1_q <= bus_rdata_i;
					end
					UL_MERGE: t1_q <= merged_w;
					default: ;
				endcase
			end
			if (ex_stop_i) begin
				req_q <= 1'b0;
				out_q <= 1'b0;
			end
			if (issue_w && ((bs_w == UBS_RS) || (bs_w == UBS_RSN)))
				rp_q <= down_w ? (rp_q - 32'd4) : (rp_q + 32'd4);
			if (go_w || ex_stop_i) rsn_q <= 1'b0;
			else if (issue_w && (bs_w == UBS_RSN)) rsn_q <= 1'b1;
			// A late write keeps the lock until END drains it.
			if (ex_stop_i || (work_w && (bus_w == UB_UNLOCK) && !req_q && !out_q)) lock_q <= 1'b0;
			else if (work_w && (bus_w == UB_READ)) lock_q <= 1'b1;

			if (work_w) begin
				case (dst_w)
					D_T0:    t0_q <= alu_v;
					D_T1:    t1_q <= alu_v;
					D_T2:    t2_q <= alu_v;
					D_R2: begin
						res_q <= alu_v;
						r2w_q <= 1'b1;
					end
					D_PSW: begin
						res_q <= alu_v;
						psw_we_q <= 1'b1;
					end
					default: ;
				endcase
				// The multiplier shifts down as product bits come in above it.
				if ((op_w == O_MSTEP) || (op_w == F_MSTEP)) t1_q <= {mstep_w[3:0], b_v[31:4]};
				// Quotient bits shift in from below.
				if (op_w == F_DST0) t1_q <= {31'd0, dq_w};
				if (dstep_w) t1_q <= {t1_q[30:0], dq_w};
				if (op_w == O_SUB) fl_q <= {sub_w[32], (a_v[31] != b_v[31]) && (sub_w[31] != a_v[31]),
					sub_w[31], sub_w[31:0] == 32'd0};
				if (((cond_w == C_MORE) || (cond_w == C_STOP)) && irq_i && left_w && !found_q)
					abort_q <= 1'b1;
				case (bs_w)
					UBS_LD: begin
						case (imm_w)
							5'd26: do_q <= rf_i[4:0];
							5'd27: so_q <= rf_i[4:0];
							5'd28: ln_q <= rf_i;
							5'd29: da_q <= search_w ? rf_i : {rf_i[31:2], 2'b00};
							default: begin
								sa_q <= {rf_i[31:2], 2'b00};
								rp_q <= {rf_i[31:2], 2'b00};
							end
						endcase
					end
					UBS_WR: begin
						ln_q <= ln_q - {26'd0, astep_w};
						do_q <= asum_do_w[4:0];
						so_q <= asum_so_w[4:0];
						if (asum_do_w[5]) da_q <= da_q + 32'd4;
						if (asum_so_w[5]) sa_q <= sa_q + 32'd4;
						if (ahead_w) begin
							a0_q <= a1_q;
							a1_q <= t0_q;
						end
					end
					UBS_SCAN: begin
						ln_q <= ln_q - {26'd0, adv_w};
						so_q <= bs_search_ofs_advance_fn(so_q, adv_w, down_w);
						sa_q <= bs_search_addr_advance_fn(sa_q, so_q, adv_w, down_w);
						da_q <= da_q + {26'd0, skip_w};
						found_q <= sfound_w;
					end
					default: ;
				endcase
			end
		end
	end

	// No reset, so the ROM can sit in block RAM; run_q gates the word.
	always @(posedge clk_i) begin
		if (ce_i) begin
			if (ent_i) uw_q <= urom_fn(quick_i ? UA_QUICK : UA_ENTRY);
			else if (wake_i) uw_q <= urom_fn(UA_WAKE);
			else if (advance_w || !run_q) uw_q <= urom_fn(rom_addr_w);
		end
	end

	assign end_o = end_w && !ent_q;
	assign rf_rd_o = run_q && !ent_q && (bs_w == UBS_LD);
	assign rf_we_o = run_q && !ent_q && (bs_w == UBS_WB) && rf_go_w;
	assign rf_addr_o = imm_w;
	assign rf_wdata_o = rf_wdata_v;
	assign found_o = found_q;
	assign abort_o = abort_q;
	assign bus_addr_o = req_q ? ad_q : issue_addr_w;
	assign result_o = res_q;
	assign psw_we_o = run_q && !ent_q && psw_we_q;
	assign ent_busy_o = ent_q;
	// The entry's END is its last step, once older work has left MEM and WB.
	assign exc_o = ent_q && uw_q[UW_END] && !older_i;
	necv810_ufp u_fp (
		.clk_i     (clk_i),
		.reset_i   (reset_i),
		.ce_i      (ce_i),
		.start_i   (dispatch_w),
		.run_i     (work_w),
		.op_i      (op_w),
		.cond_i    (cond_w),
		.k_i       (k_w[9:0]),
		.a_i       (a_v),
		.lz_i      ((bs_w == UBS_SCAN) ? rev32_fn(smatch_w) : a_v),
		.big_i     (t0_q[31:24] != 8'd0),
		.r1_i      (r1_i),
		.r2_i      (r2_i),
		.alt_i     (alt_q),
		.inexact_i (inexact_w),
		.pk_o      (fp_pk_w),
		.sc_o      (fp_sc_w),
		.sg_o      (fp_sg_w),
		.cond_o    (fp_cond_w),
		.zcls_o    (fp_zcls_w),
		.fpr_o     (fp_fpr_w),
		.fud_o     (fp_fud_w),
		.lz_o      (fp_lz_w)
	);

	// FP result flags at END, as the END word's imm says to form them. A
	// compare takes the last SUB's signed less (S ^ OV) for CY and S.
	wire [1:0]  fm_w = uw_q[UW_END] ? imm_w[1:0] : FM_NONE;
	wire [2:0]  fx_w = uw_q[UW_END] ? imm_w[4:2] : FX_NONE;
	wire        rz_w = (res_q[30:0] == 31'd0);
	wire        lt_w = fl_q[1] ^ fl_q[2];
	reg  [3:0]  cc_v;                // {CY, OV, S, Z}
	reg  [3:0]  ccm_v;
	always @* begin
		case (fm_w)
			FM_FP:   begin cc_v = {res_q[31] && !rz_w, 1'b0, res_q[31] && !rz_w, rz_w}; ccm_v = 4'b1111; end
			FM_INT:  begin cc_v = {1'b0, 1'b0, res_q[31], res_q == 32'd0};             ccm_v = 4'b0111; end
			FM_CMP:  begin cc_v = {lt_w, 1'b0, lt_w, fl_q[0]};                         ccm_v = 4'b1111; end
			default: begin cc_v = 4'd0;                                                 ccm_v = 4'd0;    end
		endcase
	end
	wire [9:0]  fsticky_w = {fx_w == FX_FRO, fx_w == FX_FIV, fx_w == FX_FZD, fx_w == FX_FOV,
		fp_fud_w, fp_fpr_w, 4'd0};
	reg  [15:0] fcode_v;
	always @* begin
		case (fx_w)
			FX_FRO:  fcode_v = 16'hff60;
			FX_FOV:  fcode_v = 16'hff64;
			FX_FZD:  fcode_v = 16'hff68;
			FX_FIV:  fcode_v = 16'hff70;
			default: fcode_v = 16'd0;
		endcase
	end

	assign fp_we_o = r2w_q;
	assign fp_flags_o = fsticky_w | {6'd0, cc_v};
	assign fp_fmask_o = fsticky_w | {6'd0, ccm_v};
	assign fp_exc_o = fx_w != FX_NONE;
	assign fp_code_o = fcode_v;
	assign flags_o = fl_q;
	assign bus_req_o = issue_w || req_q;
	assign bus_write_o = req_q ? wr_q : issue_wr_w;
	assign bus_wdata_o = req_q ? wd_q : t1_q;
	assign lock_o = lock_q;
endmodule
