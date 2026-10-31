// Copyright (c) 2026 Jamie Blanks

`timescale 1ns/1ps

// VIP object unit: draws one OBJ world into one 8-row strip.
//
//   CPU writes --> JY copy (1024 x 8)
//                     |
//   scanner: JY of each entry, end index down --> object queue --> drawer
//            (one per clock, tested a clock later)                   |
//                                                                    v
//   drawer:  words 0,1,3 -> setup -> char rows -> row queue -> strip buffer writes
//
// The scanner walks the group's OAM range on every strip and queues the entries
// that reach the strip. The drawer reads their words 0, 1 and 3 and one character
// row per visible row, then writes each row per eye (left, then right). Lower
// indexes are drawn later and land on top. Scanning and drawing overlap, so a strip
// costs about the longer of the two.
//
// Hardware scans one entry per clock whatever DRAM refresh and the CPU are doing,
// so the scanner reads its own copy of every entry's JY, kept up to date from the
// writes that land in DRAM. Only the drawer uses the DRAM port.
//
// A port grants the next read in the clock its previous data returns, so reads
// are presented before the data of the one in flight comes back.

module vip_obj
#(
	// A drawn object costs ~45 clocks plus 2 a visible row; its reads, the pipeline
	// and the last row's writes make ~11. This is the rest: internal work that leaves
	// VRAM free (fitted). Setup lasts at least one clock.
	parameter [7:0]   SETUP_CLOCKS   = 8'd34,
	// Extra when the object's top row lies above the strip (fitted).
	parameter [3:0]   CONT_CLOCKS    = 4'd5,
	// Shortest world; a small group's scan hides behind it (fitted).
	parameter [7:0]   MIN_CLOCKS     = 8'd20,
	// Visible entries the scanner may run ahead of the drawer, as a power of two.
	parameter integer QUEUE_BITS     = 2
)
(
	input  wire         clk_i,
	input  wire         reset_i,
	input  wire         ce_i,
	input  wire         abort_i,           // XPRST: drop the world and go idle, sampled on CE

	input  wire         cmd_valid_i,       // One clock, taken while idle
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [4:0]   cmd_world_i,
	input  wire [255:0] cmd_attr_i,        // Object worlds use no attributes beyond BGM
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire [4:0]   cmd_strip_i,
	input  wire [1:0]   cmd_obj_group_i,
	input  wire         cmd_obj_wrap_i,    // Group counter wrapped from 0 back to 3 before this world
	output reg          done_o,            // One clock, idle again

	input  wire [9:0]   spt0_i,
	input  wire [9:0]   spt1_i,
	input  wire [9:0]   spt2_i,
	input  wire [9:0]   spt3_i,
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [7:0]   jplt0_i,           // Bits 1:0 belong to transparent index 0
	input  wire [7:0]   jplt1_i,
	input  wire [7:0]   jplt2_i,
	input  wire [7:0]   jplt3_i,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire [2:0]   parallax_scale_i,  // MiSTer option, applied to JP

	// A byte written to the low half of an OAM word 2, in the clock it lands in DRAM
	input  wire         jy_we_i,
	input  wire [9:0]   jy_widx_i,
	input  wire [7:0]   jy_wdata_i,

	// DRAM read port (OAM)
	output reg          dram_req_o,
	output wire         dram_we_o,
	output reg  [15:0]  dram_addr_o,
	output wire [1:0]   dram_be_o,
	output wire [15:0]  dram_wdata_o,
	input  wire         dram_gnt_i,
	input  wire         dram_rvalid_i,
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [15:0]  dram_rdata_i,
	/* verilator lint_on UNUSEDSIGNAL */

	// VRAM read port (characters)
	output reg          vram_req_o,
	output wire         vram_we_o,
	output reg  [15:0]  vram_addr_o,
	output wire [1:0]   vram_be_o,
	output wire [15:0]  vram_wdata_o,
	input  wire         vram_gnt_i,
	input  wire         vram_rvalid_i,
	input  wire [15:0]  vram_rdata_i,

	// Strip buffer write, already clipped: pixel i (bits 2i+1:2i) goes to column x+i
	// where opaque[i] is set. x is 0..383.
	output wire         sb_valid_o,
	output wire         sb_eye_o,          // 0 left, 1 right
	output wire [2:0]   sb_row_o,
	output wire [9:0]   sb_x_o,
	output wire [15:0]  sb_pixels_o,
	output wire [7:0]   sb_opaque_o,
	input  wire         sb_ready_i
);

	localparam integer QD = 1 << QUEUE_BITS;
	localparam [QUEUE_BITS+1:0] QD_W = QD[QUEUE_BITS+1:0];

	localparam [1:0]
		D_IDLE  = 2'd0,
		D_WORD  = 2'd1,
		D_SETUP = 2'd2,
		D_ROWS  = 2'd3;

	// DRAM read kinds
	localparam [1:0]
		K_W0 = 2'd1,
		K_W1 = 2'd2,
		K_W3 = 2'd3;

	// OAM sits at DRAM halfword 0xF000, four halfwords per object.
	localparam [3:0] OAM_PAGE = 4'hf;

	reg        run_q;
	reg [4:0]  strip_q;

	// Scanner
	reg        scan_more_q;       // JY reads left to issue
	reg [7:0]  min_q;             // Clocks left of the shortest world
	reg [9:0]  scan_idx_q;        // Next entry to read
	reg [9:0]  first_q;           // Group start index, the last entry scanned
	reg        jy_rd_q;           // A JY read is presented; tested next clock
	reg [9:0]  jy_idx_q;

	// DRAM request and the read in flight
	reg [1:0]  dram_kind_q;
	reg        dram_pend_q;
	reg [1:0]  pend_kind_q;

	// Object queue: {index, dy, top above strip}
	reg [13:0]           oq_q [0:QD-1];
	reg [QUEUE_BITS-1:0] oq_wr_q;
	reg [QUEUE_BITS-1:0] oq_rd_q;
	reg [QUEUE_BITS:0]   oq_cnt_q;

	// Drawer
	reg [1:0]  d_state_q;
	reg [11:0] wait_q;
	reg [9:0]  d_idx_q;
	reg [1:0]  wreq_q;            // OAM words requested for the current object
	reg        vram_pend_q;
	reg [2:0]  rq_row_q;          // Strip row of the next character request
	reg        rq_more_q;
	reg [2:0]  row_q;             // Strip row of the next character data
	reg [2:0]  row_last_q;
	reg [2:0]  dy_q;              // Object top minus strip top, low bits
	reg        cont_q;            // Top row lies above the strip
	reg        rows_in_q;         // Every character row has arrived
	reg [9:0]  jx_q;
	reg [9:0]  jp_q;
	reg [1:0]  jon_q;             // {JRON, JLON}
	reg [1:0]  pal_q;
	reg        hflip_q;
	reg        vflip_q;
	reg [10:0] jca_q;
	reg        on_l_q,   on_r_q;
	reg [8:0]  x_l_q,    x_r_q;   // First written column
	reg [2:0]  skip_l_q, skip_r_q; // Leading pixels left of column 0
	reg [7:0]  keep_l_q, keep_r_q; // Pixels left of column 384

	// Row queue; entry 0 is the head. Each entry owes up to one write per eye.
	reg [1:0]  q_cnt_q;
	reg [1:0]  q_pend_q [0:2];    // {right, left}
	reg [2:0]  q_row_q  [0:2];
	reg [17:0] q_x_q    [0:2];    // {right, left}
	reg [31:0] q_px_q   [0:2];
	reg [15:0] q_op_q   [0:2];

	assign dram_we_o    = 1'b0;
	assign dram_be_o    = 2'b11;
	assign dram_wdata_o = 16'd0;
	assign vram_we_o    = 1'b0;
	assign vram_be_o    = 2'b11;
	assign vram_wdata_o = 16'd0;

	wire dram_data_w = dram_rvalid_i && dram_pend_q;
	wire char_data_w = vram_rvalid_i && vram_pend_q;
	wire dram_free_w = !dram_req_o || dram_gnt_i;
	wire vram_free_w = !vram_req_o || vram_gnt_i;

	// Group g runs from SPT(g-1)+1 (0 for group 0) up to SPTg, cyclic over 1024 entries.
	reg [9:0] grp_end_w;
	reg [9:0] grp_first_w;
	always @* begin
		case (cmd_obj_group_i)
			2'd3:    begin grp_end_w = spt3_i; grp_first_w = spt2_i + 10'd1; end
			2'd2:    begin grp_end_w = spt2_i; grp_first_w = spt1_i + 10'd1; end
			2'd1:    begin grp_end_w = spt1_i; grp_first_w = spt0_i + 10'd1; end
			default: begin grp_end_w = spt0_i; grp_first_w = 10'd0; end
		endcase
	end
	// Once the group counter has wrapped, group 3 starts at entry 1023, where the
	// scan of group 0 left off, instead of SPT3, and draws the entries above SPT3 too.
	wire wrap_scan_w = cmd_obj_wrap_i;

	// ---- Scanner ----------------------------------------------------------------------

	// The read port follows jy_idx_q, so its data is ready by the next CE.
	wire [7:0] jy_w;
	cache_ram_dp #(
		.ADDR_WIDTH (10),
		.DATA_WIDTH (8)
	) u_jy (
		.clk_i     (clk_i),
		.addr_a_i  (jy_widx_i),
		.wren_a_i  (jy_we_i),
		.wdata_a_i (jy_wdata_i),
		.q_a_o     (),
		.addr_b_i  (jy_idx_q),
		.wren_b_i  (1'b0),
		.wdata_b_i (8'd0),
		.q_b_o     (jy_w)
	);

	// JY E0-FF reads as -32..-1, so E0-F8 never reaches a strip.
	wire signed [9:0] obj_top_w = $signed({{2{&jy_w[7:5]}}, jy_w});
	wire signed [9:0] dy_w      = obj_top_w - $signed({2'b00, strip_q, 3'b000});
	wire              jy_hit_w  = (dy_w >= -10'sd7) && (dy_w <= 10'sd7);
	wire              oq_push_w = jy_rd_q && jy_hit_w;

	wire scan_last_w = (scan_idx_q == first_q);

	// ---- Drawer -----------------------------------------------------------------------

	wire        last_row_w = (d_state_q == D_ROWS) && char_data_w && (row_q == row_last_q);
	wire        obj_end_w;
	wire [13:0] oq_head_w = oq_q[oq_rd_q];
	wire        oq_pop_w  = ((d_state_q == D_IDLE) || obj_end_w) && (oq_cnt_q != 0);

	wire [QUEUE_BITS:0] oq_cnt_d_w = oq_cnt_q - {{QUEUE_BITS{1'b0}}, oq_pop_w} +
		{{QUEUE_BITS{1'b0}}, oq_push_w};

	// The drawer's word reads, and the next JY while the object queue has room for it.
	wire [1:0]  wreq_d_w     = oq_pop_w ? 2'd0 : wreq_q;
	wire [9:0]  word_idx_w   = oq_pop_w ? oq_head_w[13:4] : d_idx_q;
	wire [1:0]  word_sel_w   = (wreq_d_w == 2'd2) ? 2'd3 : wreq_d_w;
	wire        word_issue_w = dram_free_w && (oq_pop_w || (d_state_q == D_WORD)) && (wreq_d_w != 2'd3);
	wire        scan_issue_w = run_q && scan_more_q && ({1'b0, oq_cnt_d_w} < QD_W);

	wire [11:0] setup_total_w = {4'd0, SETUP_CLOCKS} + (cont_q ? {8'd0, CONT_CLOCKS} : 12'd0);

	// Eye positions JX - JP and JX + JP.
	wire signed [9:0]  jp_scaled_w;
	vip_stereo_scale_signed #(
		.WIDTH (10)
	) u_jp_scale (
		.value_i (jp_q),
		.scale_i (parallax_scale_i),
		.value_o (jp_scaled_w)
	);

	wire signed [11:0] jx_ext_w = $signed({{2{jx_q[9]}}, jx_q});
	wire signed [11:0] jp_ext_w = $signed({{2{jp_scaled_w[9]}}, jp_scaled_w});
	wire signed [11:0] x_l_w    = jx_ext_w - jp_ext_w;
	wire signed [11:0] x_r_w    = jx_ext_w + jp_ext_w;

	// Columns 376..383 share x[8:3] = 47; pixels past 383 are dropped.
	wire       vis_l_w  = (x_l_w >= -12'sd7) && (x_l_w <= 12'sd383);
	wire       vis_r_w  = (x_r_w >= -12'sd7) && (x_r_w <= 12'sd383);
	wire [7:0] keep_l_w = (!x_l_w[11] && (x_l_w[8:3] == 6'd47)) ? (8'hff >> x_l_w[2:0]) : 8'hff;
	wire [7:0] keep_r_w = (!x_r_w[11] && (x_r_w[8:3] == 6'd47)) ? (8'hff >> x_r_w[2:0]) : 8'hff;

	// Character rows: source row = strip row - dy, flipped by VFLIP.
	wire [2:0]  src_row_w  = (rq_row_q - dy_q) ^ {3{vflip_q}};
	wire [15:0] chr_addr_w = {jca_q[10:9], 2'b11, jca_q[8:0], src_row_w};

	// Character row decode: HFLIP, then JPLT. Pixel i sits in bits 2i+1:2i, leftmost first.
	reg [7:2]  jplt_w;
	reg [1:0]  raw_t;
	reg [7:0]  chr_nz_w;
	reg [15:0] chr_px_w;
	integer    pix_t;
	always @* begin
		case (pal_q)
			2'd0:    jplt_w = jplt0_i[7:2];
			2'd1:    jplt_w = jplt1_i[7:2];
			2'd2:    jplt_w = jplt2_i[7:2];
			default: jplt_w = jplt3_i[7:2];
		endcase
		raw_t = 2'd0;
		for (pix_t = 0; pix_t < 8; pix_t = pix_t + 1) begin
			raw_t = hflip_q ? vram_rdata_i[(14 - 2 * pix_t) +: 2] : vram_rdata_i[(2 * pix_t) +: 2];
			chr_nz_w[pix_t] = (raw_t != 2'd0);
			case (raw_t)
				2'd1:    chr_px_w[(2 * pix_t) +: 2] = jplt_w[3:2];
				2'd2:    chr_px_w[(2 * pix_t) +: 2] = jplt_w[5:4];
				2'd3:    chr_px_w[(2 * pix_t) +: 2] = jplt_w[7:6];
				default: chr_px_w[(2 * pix_t) +: 2] = 2'd0;
			endcase
		end
	end

	function automatic [15:0] spread_fn(input [7:0] mask);
		integer i;
		begin
			for (i = 0; i < 8; i = i + 1)
				spread_fn[(2 * i) +: 2] = {2{mask[i]}};
		end
	endfunction

	// Shift a row left of column 0 down to x = 0, then trim past column 383.
	wire [7:0]  op_l_w     = (chr_nz_w >> skip_l_q) & keep_l_q & {8{on_l_q}};
	wire [7:0]  op_r_w     = (chr_nz_w >> skip_r_q) & keep_r_q & {8{on_r_q}};
	wire [15:0] px_l_w     = (chr_px_w >> {skip_l_q, 1'b0}) & spread_fn(op_l_w);
	wire [15:0] px_r_w     = (chr_px_w >> {skip_r_q, 1'b0}) & spread_fn(op_r_w);
	// Each enabled eye writes every visible row, opaque or not, on screen or not.
	wire [1:0]  new_pend_w = jon_q;
	wire        q_push_w   = char_data_w && (new_pend_w != 2'b00);

	// Strip buffer side of the row queue.
	wire        head_left_w = q_pend_q[0][0];
	assign sb_valid_o  = (q_cnt_q != 2'd0);
	assign sb_eye_o    = !head_left_w;
	assign sb_row_o    = q_row_q[0];
	assign sb_x_o      = {1'b0, head_left_w ? q_x_q[0][8:0] : q_x_q[0][17:9]};
	assign sb_pixels_o = head_left_w ? q_px_q[0][15:0] : q_px_q[0][31:16];
	assign sb_opaque_o = head_left_w ? q_op_q[0][7:0]  : q_op_q[0][15:8];

	wire       sb_fire_w = sb_valid_o && sb_ready_i;
	wire       q_pop_w   = sb_fire_w && (q_pend_q[0] != 2'b11);
	wire [1:0] q_cnt_d_w = q_cnt_q - {1'b0, q_pop_w} + {1'b0, q_push_w};
	wire [1:0] q_slot_w  = q_cnt_q - {1'b0, q_pop_w};

	// An object ends once its last row's writes have gone; the next one's reads wait.
	assign obj_end_w = (d_state_q == D_ROWS) && (rows_in_q || last_row_w) && (q_cnt_d_w == 2'd0);

	// A character read is issued only when its row is sure of a row queue entry.
	wire       vram_pend_d_w = (vram_req_o && vram_gnt_i) || (vram_pend_q && !char_data_w);
	wire [2:0] q_used_w      = {1'b0, q_cnt_d_w} + {2'b00, vram_pend_d_w};
	wire       rows_go_w     = ((d_state_q == D_SETUP) && (wait_q < 12'd2)) || (d_state_q == D_ROWS);
	wire       row_issue_w   = rows_go_w && rq_more_q && vram_free_w && (q_used_w < 3'd3);

	// Finished: everything scanned, queued objects drawn, rows written.
	wire       fin_w = run_q && !scan_more_q && !jy_rd_q && (oq_cnt_q == 0) &&
		(d_state_q == D_IDLE) && (q_cnt_q == 2'd0) && (min_q == 8'd0);

	integer e_t;

	always @(posedge clk_i) begin
		if (reset_i) begin
			run_q       <= 1'b0;
			done_o      <= 1'b0;
			strip_q     <= 5'd0;
			scan_more_q <= 1'b0;
			min_q       <= 8'd0;
			scan_idx_q  <= 10'd0;
			first_q     <= 10'd0;
			jy_rd_q     <= 1'b0;
			jy_idx_q    <= 10'd0;
			dram_req_o  <= 1'b0;
			dram_addr_o <= 16'd0;
			dram_kind_q <= K_W0;
			dram_pend_q <= 1'b0;
			pend_kind_q <= K_W0;
			oq_wr_q     <= {QUEUE_BITS{1'b0}};
			oq_rd_q     <= {QUEUE_BITS{1'b0}};
			oq_cnt_q    <= {(QUEUE_BITS + 1){1'b0}};
			for (e_t = 0; e_t < QD; e_t = e_t + 1)
				oq_q[e_t] <= 14'd0;
			d_state_q   <= D_IDLE;
			wait_q      <= 12'd0;
			d_idx_q     <= 10'd0;
			wreq_q      <= 2'd0;
			vram_req_o  <= 1'b0;
			vram_addr_o <= 16'd0;
			vram_pend_q <= 1'b0;
			rq_row_q    <= 3'd0;
			rq_more_q   <= 1'b0;
			row_q       <= 3'd0;
			row_last_q  <= 3'd0;
			dy_q        <= 3'd0;
			cont_q      <= 1'b0;
			rows_in_q   <= 1'b0;
			jx_q        <= 10'd0;
			jp_q        <= 10'd0;
			jon_q       <= 2'd0;
			pal_q       <= 2'd0;
			hflip_q     <= 1'b0;
			vflip_q     <= 1'b0;
			jca_q       <= 11'd0;
			on_l_q      <= 1'b0;
			on_r_q      <= 1'b0;
			x_l_q       <= 9'd0;
			x_r_q       <= 9'd0;
			skip_l_q    <= 3'd0;
			skip_r_q    <= 3'd0;
			keep_l_q    <= 8'd0;
			keep_r_q    <= 8'd0;
			q_cnt_q     <= 2'd0;
			for (e_t = 0; e_t < 3; e_t = e_t + 1) begin
				q_pend_q[e_t] <= 2'd0;
				q_row_q[e_t]  <= 3'd0;
				q_x_q[e_t]    <= 18'd0;
				q_px_q[e_t]   <= 32'd0;
				q_op_q[e_t]   <= 16'd0;
			end
		end else if (ce_i) begin
			done_o <= 1'b0;

			if (abort_i) begin
				// Responses still due are ignored: nothing is marked in flight any more.
				run_q       <= 1'b0;
				scan_more_q <= 1'b0;
				min_q       <= 8'd0;
				jy_rd_q     <= 1'b0;
				dram_req_o  <= 1'b0;
				dram_pend_q <= 1'b0;
				oq_cnt_q    <= {(QUEUE_BITS + 1){1'b0}};
				oq_rd_q     <= oq_wr_q;
				d_state_q   <= D_IDLE;
				vram_req_o  <= 1'b0;
				vram_pend_q <= 1'b0;
				rq_more_q   <= 1'b0;
				q_cnt_q     <= 2'd0;
			end else begin
				if (!run_q && cmd_valid_i) begin
					run_q       <= 1'b1;
					strip_q     <= cmd_strip_i;
					first_q     <= grp_first_w;
					scan_idx_q  <= wrap_scan_w ? 10'h3ff : grp_end_w;
					scan_more_q <= 1'b1;
					min_q       <= MIN_CLOCKS;
				end else if (min_q != 8'd0) begin
					min_q <= min_q - 8'd1;
				end
				if (fin_w) begin
					run_q  <= 1'b0;
					done_o <= 1'b1;
				end

				// DRAM port: data first, then a grant taken in the same clock.
				if (dram_data_w)
					dram_pend_q <= 1'b0;
				if (dram_req_o && dram_gnt_i) begin
					dram_req_o  <= 1'b0;
					dram_pend_q <= 1'b1;
					pend_kind_q <= dram_kind_q;
				end

				if (oq_pop_w)
					wreq_q <= 2'd0;
				if (word_issue_w) begin
					dram_req_o  <= 1'b1;
					dram_kind_q <= K_W0 + wreq_d_w;
					dram_addr_o <= {OAM_PAGE, word_idx_w, word_sel_w};
					wreq_q      <= wreq_d_w + 2'd1;
				end

				jy_rd_q <= scan_issue_w;
				if (scan_issue_w) begin
					jy_idx_q    <= scan_idx_q;
					scan_idx_q  <= scan_idx_q - 10'd1;
					scan_more_q <= !scan_last_w;
				end

				// Object queue
				if (oq_push_w) begin
					oq_q[oq_wr_q] <= {jy_idx_q, dy_w[2:0], dy_w[9]};
					oq_wr_q       <= oq_wr_q + 1'b1;
				end
				if (oq_pop_w)
					oq_rd_q <= oq_rd_q + 1'b1;
				oq_cnt_q <= oq_cnt_d_w;

				// Drawer: words come back in the order asked.
				if (dram_data_w) begin
					case (pend_kind_q)
						K_W0: jx_q <= dram_rdata_i[9:0];
						K_W1: begin
							jp_q  <= dram_rdata_i[9:0];
							jon_q <= {dram_rdata_i[14], dram_rdata_i[15]};
						end
						K_W3: begin
							pal_q     <= dram_rdata_i[15:14];
							hflip_q   <= dram_rdata_i[13];
							vflip_q   <= dram_rdata_i[12];
							jca_q     <= dram_rdata_i[10:0];
							on_l_q    <= jon_q[0] && vis_l_w;
							on_r_q    <= jon_q[1] && vis_r_w;
							x_l_q     <= (x_l_w[11] || !vis_l_w) ? 9'd0 : x_l_w[8:0];
							x_r_q     <= (x_r_w[11] || !vis_r_w) ? 9'd0 : x_r_w[8:0];
							skip_l_q  <= x_l_w[11] ? (3'd0 - x_l_w[2:0]) : 3'd0;
							skip_r_q  <= x_r_w[11] ? (3'd0 - x_r_w[2:0]) : 3'd0;
							keep_l_q  <= keep_l_w;
							keep_r_q  <= keep_r_w;
							rq_more_q <= 1'b1;
							d_state_q <= D_SETUP;
							wait_q    <= setup_total_w;
						end
						default: ;
					endcase
				end

				case (d_state_q)
					D_SETUP: begin
						if (wait_q < 12'd2)
							d_state_q <= D_ROWS;
						else
							wait_q <= wait_q - 12'd1;
					end
					D_ROWS: begin
						if (obj_end_w)
							d_state_q <= D_IDLE;
					end
					default: ;
				endcase

				if (last_row_w) begin
					rows_in_q <= 1'b1;
				end
				if (oq_pop_w) begin
					rows_in_q  <= 1'b0;
					d_idx_q    <= oq_head_w[13:4];
					dy_q       <= oq_head_w[3:1];
					cont_q     <= oq_head_w[0];
					row_q      <= oq_head_w[0] ? 3'd0 : oq_head_w[3:1];
					rq_row_q   <= oq_head_w[0] ? 3'd0 : oq_head_w[3:1];
					row_last_q <= oq_head_w[0] ? (oq_head_w[3:1] - 3'd1) : 3'd7;
					d_state_q  <= D_WORD;
				end

				// VRAM port
				if (char_data_w && !oq_pop_w)
					row_q <= row_q + 3'd1;
				if (char_data_w)
					vram_pend_q <= 1'b0;
				if (vram_req_o && vram_gnt_i) begin
					vram_req_o  <= 1'b0;
					vram_pend_q <= 1'b1;
				end
				if (row_issue_w) begin
					vram_req_o  <= 1'b1;
					vram_addr_o <= chr_addr_w;
					rq_row_q    <= rq_row_q + 3'd1;
					rq_more_q   <= (rq_row_q != row_last_q);
				end

				// Row queue: pop when the head's last write goes, push a decoded row.
				if (q_pop_w) begin
					for (e_t = 0; e_t < 2; e_t = e_t + 1) begin
						q_pend_q[e_t] <= q_pend_q[e_t + 1];
						q_row_q[e_t]  <= q_row_q[e_t + 1];
						q_x_q[e_t]    <= q_x_q[e_t + 1];
						q_px_q[e_t]   <= q_px_q[e_t + 1];
						q_op_q[e_t]   <= q_op_q[e_t + 1];
					end
				end else if (sb_fire_w) begin
					q_pend_q[0] <= 2'b10;
				end
				if (q_push_w) begin
					q_pend_q[q_slot_w] <= new_pend_w;
					q_row_q[q_slot_w]  <= row_q;
					q_x_q[q_slot_w]    <= {x_r_q, x_l_q};
					q_px_q[q_slot_w]   <= {px_r_w, px_l_w};
					q_op_q[q_slot_w]   <= {op_r_w, op_l_w};
				end
				q_cnt_q <= q_cnt_d_w;
			end
		end
	end

endmodule
