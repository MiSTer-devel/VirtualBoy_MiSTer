// Copyright (c) 2026 Jamie Blanks

`timescale 1ns/1ps

// VIP background unit: draws one normal, H-bias or affine world into one
// 8-row strip, both eyes in one pass.
//
//   cmd -> setup -> per load or row: [parameter reads] -> geometry
//       -> read steps -> decode -> 2-entry write queue -> strip buffer
//
//   normal  per tile-row load (one or two a strip), across the union of both
//           eyes' source spans: per tile a cell step, then one CHAR per row,
//           shared by both eyes
//   H-bias  per row: HOFS L, HOFS R, then per tile of the row's union a cell
//           step and one CHAR, shared by both eyes
//   affine  per row: MX MP MY DX DY, MP x DX/DY by shift and add, then per pixel
//           CELL L | CHAR L + CELL R | CHAR R + next CELL L
//
// A tile's last character read also reads the next tile's map cell, so one
// cell read is in flight at a time and each tile still takes its cell step.
//
// A read step holds at most one DRAM and one VRAM read. The next step is
// requested in the clock the previous step's last data arrives, the first clock
// the port can grant it, so drawing time is the reads' slots plus the fixed
// setup clocks of the states below.
//
// A tile's cell is taken CELL_STEP_CLOCKS after the previous one once it has
// arrived.

module vip_bg
#(
	// Clocks per map cell.
	parameter integer CELL_STEP_CLOCKS   = 2,
	// Per normal or H-bias world in each strip (fitted).
	parameter integer WORLD_EXTRA_CLOCKS = 1,
	// Map address and coordinate setup, once per normal tile-row load and once
	// per H-bias row; it leaves VRAM free (fitted).
	parameter integer BG_ROW_SETUP       = 85,
	// Per affine row, beyond its parameter reads, multiply and pixels (fitted).
	parameter integer AFFINE_ROW_EXTRA   = 35
)
(
	input  wire         clk_i,
	input  wire         reset_i,
	input  wire         ce_i,
	input  wire         abort_i,           // XPRST: drop the world and go idle, sampled on CE

	input  wire         cmd_valid_i,       // One clock, taken while idle
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [4:0]   cmd_world_i,
	input  wire [255:0] cmd_attr_i,        // Words 0-15 as read, GP and MP already scaled
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire [4:0]   cmd_strip_i,
	output reg          done_o,            // One clock, idle again

	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [7:0]   gplt0_i,           // Bits 1:0 belong to transparent index 0
	input  wire [7:0]   gplt1_i,
	input  wire [7:0]   gplt2_i,
	input  wire [7:0]   gplt3_i,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire [2:0]   parallax_scale_i,  // MiSTer option, applied to per-row HOFS and affine MP

	// DRAM read port (maps, parameter tables)
	output wire         dram_req_o,
	output wire         dram_we_o,
	output wire [15:0]  dram_addr_o,
	output wire [1:0]   dram_be_o,
	output wire [15:0]  dram_wdata_o,
	input  wire         dram_gnt_i,
	input  wire         dram_rvalid_i,
	input  wire [15:0]  dram_rdata_i,

	// VRAM read port (characters)
	output wire         vram_req_o,
	output wire         vram_we_o,
	output wire [15:0]  vram_addr_o,
	output wire [1:0]   vram_be_o,
	output wire [15:0]  vram_wdata_o,
	input  wire         vram_gnt_i,
	input  wire         vram_rvalid_i,
	input  wire [15:0]  vram_rdata_i,

	// Strip buffer write
	output wire         sb_valid_o,
	output wire         sb_eye_o,          // 0 left, 1 right
	output wire [2:0]   sb_row_o,
	output wire [9:0]   sb_x_o,            // Signed x of pixel 0, always -7..383
	output wire [15:0]  sb_pixels_o,       // Pixel i in bits 2i+1:2i
	output wire [7:0]   sb_opaque_o,
	input  wire         sb_ready_i
);

	localparam [3:0]
		S_IDLE   = 4'd0,
		S_SETUP1 = 4'd1,   // window rows, eye offsets, width
		S_SETUP2 = 4'd2,   // screen clip offsets, normal source union
		S_ROW    = 4'd3,   // source row, load size, parameter row address
		S_PARAM  = 4'd4,   // H-bias and affine parameter reads
		S_HGEOM1 = 4'd5,   // H-bias: parallax scale of HOFS
		S_HGEOM2 = 4'd6,   // H-bias: eye source starts
		S_HGEOM3 = 4'd7,   // H-bias: union and tile range
		S_HGEOM4 = 4'd8,   // H-bias: clip spans
		S_AINIT  = 4'd9,   // affine: seeds and clip spans
		S_AMUL   = 4'd10,  // affine: 16 shift-add steps of |MP| x DX/DY
		S_TILES  = 4'd11,  // normal and H-bias character reads
		S_PIXELS = 4'd12,  // affine pixel reads
		S_FINISH = 4'd13,  // wait for the write queue to drain
		S_BURST  = 4'd14;  // normal and H-bias map cell step

	localparam [1:0]
		DK_NONE  = 2'd0,
		DK_CELL  = 2'd1,
		DK_PARAM = 2'd2;

	localparam integer CELL_GAP_I = CELL_STEP_CLOCKS - 1;
	localparam [3:0] CELL_GAP   = CELL_GAP_I[3:0];
	localparam [7:0] WORLD_PAD  = WORLD_EXTRA_CLOCKS[7:0];
	localparam [7:0] ROW_SETUP  = BG_ROW_SETUP[7:0];
	localparam [7:0] AFFINE_PAD = AFFINE_ROW_EXTRA[7:0];

	reg [3:0]         state_q;

	// Attributes, held for the whole command.
	reg [1:0]         en_q;          // bit 0 LON, bit 1 RON
	reg               hb_q;
	reg               af_q;
	reg [1:0]         scx_q;
	reg [1:0]         scy_q;
	reg [1:0]         col_log_q;     // log2 of map columns used by the map index
	reg [3:0]         mapbase_q;     // BGMAP_BASE rounded to the map count
	reg               over_q;
	reg [15:0]        ovp_q;
	reg signed [9:0]  gx_q;
	reg signed [9:0]  gp_q;
	reg signed [15:0] gy_q;
	reg signed [12:0] mx_q;
	reg signed [14:0] mp_q;
	reg signed [12:0] my_q;
	reg [12:0]        wraw_q;
	reg [15:0]        hraw_q;
	reg [15:0]        pb_q;
	reg [4:0]         strip_q;

	// Per command geometry.
	reg [8:0]         y_q;           // current screen row
	reg [3:0]         rows_left_q;
	reg [12:0]        width_q;
	reg signed [17:0] d_q [0:1];     // destination start GX-GP / GX+GP
	reg signed [17:0] sb_q [0:1];    // source start MX-MP / MX+MP
	reg signed [17:0] lo_off_q [0:1];
	reg signed [17:0] hi_off_q [0:1];
	reg signed [17:0] a0_q [0:1];    // first tile to eye source start
	reg signed [17:0] tile0_q;
	reg [13:0]        count0_q;

	// Clip spans: pixel i of the current tile or pixel group is drawn when
	// lo <= i < hi; base is the destination x of pixel 0.
	reg signed [17:0] lo_q [0:1];
	reg signed [17:0] hi_q [0:1];
	reg signed [17:0] base_q [0:1];

	// Load or row.
	reg [7:0]         pad_q;
	reg signed [16:0] sy_q;
	reg [3:0]         load_rows_q;
	reg [15:0]        pa_q;
	reg [2:0]         pidx_q;
	reg signed [17:0] tile_x_q;
	reg [13:0]        tiles_left_q;  // tiles whose cells are not fetched yet
	reg               first_tile_q;
	reg [2:0]         row_ord_q;

	// Map cells of the tile being drawn and of the next one.
	reg [15:0]        cbuf_q [0:1];
	reg               ch_n_q;        // the step has a tile
	reg               bi_q;          // its cell is read
	reg               wi_q;          // cell buffer entry of the cell in flight
	reg               ct_q;          // its character reads are issued
	reg [3:0]         cgap_q;        // clocks before the next cell may be taken
	reg               ar_q;          // its cell arrived
	reg               pc_q;          // its cell is taken by the pace
	reg               pf_q;          // the next tile's cell is read ahead
	reg               pf_ok_q;       // and has arrived
	reg               slot_q;        // cell buffer entry of the tile being drawn

	// H-bias row.
	reg signed [12:0] hofs_l_q;
	reg signed [12:0] hofs_r_q;
	reg signed [17:0] hl_q;
	reg signed [17:0] hr_q;
	reg signed [17:0] s_q [0:1];

	// Affine row.
	reg [15:0]        amx_q;
	reg [15:0]        amp_q;
	reg [15:0]        amy_q;
	reg [15:0]        adx_q;
	reg [15:0]        ady_q;
	reg [25:0]        ax_q [0:1];    // Q17.9 source x per eye
	reg [25:0]        ay_q [0:1];
	reg [25:0]        tx_q;
	reg [25:0]        ty_q;
	reg [15:0]        mag_q;
	reg               off_eye_q;     // eye that gets |MP| x DX/DY added
	reg [3:0]         mul_cnt_q;
	reg [10:0]        npix_q;
	reg [11:0]        k_q;           // step: cell k, char k-1

	// Read ports. pend: requested, not yet granted. fly: granted, data due.
	reg               dpend_q;
	reg               dfly_q;
	reg [15:0]        daddr_q;
	reg [1:0]         dkind_q;
	reg [2:0]         dpidx_q;
	reg               vpend_q;
	reg               vfly_q;
	reg [15:0]        vaddr_q;
	reg               vchar_q;

	// Last cell and the affine context of the cell read last launched.
	reg [15:0]        cell_q;
	reg               deye_q;
	reg [2:0]         dsx_q;
	reg [2:0]         dsy_q;
	reg [2:0]         dpx_q;
	reg               dlast_q;

	// Context of the character read in flight.
	reg [2:0]         vrow_q;
	reg [1:0]         vpal_q;
	reg               vhflip_q;
	reg               veye_q;
	reg [2:0]         vsel_q;
	reg [2:0]         vpx_q;
	reg               vpush_q;

	// Affine 8-pixel packers.
	reg [15:0]        pkl_pix_q;
	reg [7:0]         pkl_nz_q;
	reg [15:0]        pkr_pix_q;
	reg [7:0]         pkr_nz_q;

	// Write queue: {row, en[1:0], R: x, pix, opq, L: x, pix, opq}.
	localparam integer QW = 3 + 2 + 2 * 34;
	reg [QW-1:0]      q0_q;
	reg [QW-1:0]      q1_q;
	reg               qv0_q;
	reg               qv1_q;
	reg               ldone_q;       // left half of the head entry written

	// ------------------------------------------------------------------
	// Small helpers.

	// plt is GPLTn bits 7:2, the colours of indexes 1-3.
	function automatic [1:0] pal_map(input [1:0] raw, input [5:0] plt);
		case (raw)
			2'd0:    pal_map = 2'd0;
			2'd1:    pal_map = plt[1:0];
			2'd2:    pal_map = plt[3:2];
			default: pal_map = plt[5:4];
		endcase
	endfunction

	function automatic [7:0] span_mask(input signed [17:0] lo, input signed [17:0] hi);
		integer i;
		reg signed [17:0] p;
		begin
			for (i = 0; i < 8; i = i + 1) begin
				p = $signed({15'd0, i[2:0]});
				span_mask[i] = (lo <= p) && (hi > p);
			end
		end
	endfunction

	/* verilator lint_off UNUSEDSIGNAL */
	function automatic [15:0] char_addr(input [15:0] cw, input [2:0] row);
		char_addr = {cw[10:9], 2'b11, cw[8:0], row ^ {3{cw[12]}}};
	endfunction
	/* verilator lint_on UNUSEDSIGNAL */

	function automatic signed [17:0] min18(input signed [17:0] a, input signed [17:0] b);
		min18 = (a < b) ? a : b;
	endfunction

	function automatic signed [17:0] max18(input signed [17:0] a, input signed [17:0] b);
		max18 = (a > b) ? a : b;
	endfunction

	// ------------------------------------------------------------------
	// Command decode helpers (from the attribute words, used at accept).

	/* verilator lint_off UNUSEDSIGNAL */
	wire [15:0] cw0_w  = cmd_attr_i[15:0];
	/* verilator lint_on UNUSEDSIGNAL */
	wire [1:0]  cscx_w = cw0_w[11:10];
	wire [1:0]  cscy_w = cw0_w[9:8];
	wire [2:0]  cnt_log_sum_w = {1'b0, cscx_w} + {1'b0, cscy_w};
	wire [1:0]  cnt_log_w = (cnt_log_sum_w > 3'd3) ? 2'd3 : cnt_log_sum_w[1:0];
	wire [1:0]  ccol_lim_w = 2'd3 - cscy_w;
	wire [1:0]  ccol_log_w = (cscx_w < ccol_lim_w) ? cscx_w : ccol_lim_w;
	reg  [3:0]  cmapbase_t;

	always @* begin
		case (cnt_log_w)
			2'd0:    cmapbase_t = cw0_w[3:0];
			2'd1:    cmapbase_t = {cw0_w[3:1], 1'b0};
			2'd2:    cmapbase_t = {cw0_w[3:2], 2'b00};
			default: cmapbase_t = {cw0_w[3], 3'b000};
		endcase
	end

	// ------------------------------------------------------------------
	// Setup arithmetic.

	wire signed [17:0] strip_y_w = $signed({10'd0, strip_q, 3'b000});
	wire signed [16:0] sem_h_w   = $signed({hraw_q[15], hraw_q}) + 17'sd1;
	wire [15:0]        vis_h_w   = (sem_h_w < 17'sd8) ? 16'd8 : sem_h_w[15:0];
	wire signed [17:0] height_w  = af_q ? {sem_h_w[16], sem_h_w} : $signed({2'b00, vis_h_w});
	wire signed [17:0] gy18_w    = {{2{gy_q[15]}}, gy_q};
	wire signed [17:0] end_y_w   = gy18_w + height_w;
	wire signed [17:0] y0_w      = max18(strip_y_w, gy18_w);
	wire signed [17:0] y1_w      = min18(strip_y_w + 18'sd8, end_y_w);
	wire signed [17:0] rows_w    = y1_w - y0_w;
	wire signed [17:0] gx18_w    = {{8{gx_q[9]}}, gx_q};
	wire signed [17:0] gp18_w    = {{8{gp_q[9]}}, gp_q};
	wire signed [17:0] mx18_w    = {{5{mx_q[12]}}, mx_q};
	wire signed [17:0] mp18_w    = {{3{mp_q[14]}}, mp_q};
	wire signed [17:0] width18_w = $signed({5'd0, width_q});

	// Source union of both eyes and its tile range.
	/* verilator lint_off UNUSEDSIGNAL */
	wire signed [17:0] un_src_l_w = hb_q ? s_q[0] : sb_q[0];
	wire signed [17:0] un_src_r_w = hb_q ? s_q[1] : sb_q[1];
	wire signed [17:0] un_lo_w    = min18(un_src_l_w, un_src_r_w);
	wire signed [17:0] un_hi_w    = max18(un_src_l_w, un_src_r_w) + width18_w;
	wire signed [17:0] un_t0_w    = {un_lo_w[17:3], 3'b000};
	wire signed [17:0] un_hi_c_w  = un_hi_w + 18'sd7;
	wire signed [17:0] un_tiles_w = {{3{un_hi_c_w[17]}}, un_hi_c_w[17:3]} - {{3{un_lo_w[17]}}, un_lo_w[17:3]};
	/* verilator lint_on UNUSEDSIGNAL */
	wire [13:0]        un_count_w = un_tiles_w[17] ? 14'd0 : un_tiles_w[13:0];

	// Source row of the current screen row, and the row's place in the window.
	wire signed [16:0] row_sy_w = {{4{my_q[12]}}, my_q} + $signed({8'd0, y_q}) - {gy_q[15], gy_q};
	/* verilator lint_off UNUSEDSIGNAL */
	wire [15:0]        local_w  = {7'd0, y_q} - gy_q;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [3:0]         to_tile_end_w = 4'd8 - {1'b0, row_sy_w[2:0]};
	wire [3:0]         normal_rows_w = (rows_left_q < to_tile_end_w) ? rows_left_q : to_tile_end_w;

	// H-bias parallax: scale each eye's HOFS around their mean.
	wire signed [17:0] hofs_l18_w = {{5{hofs_l_q[12]}}, hofs_l_q};
	wire signed [17:0] hofs_r18_w = {{5{hofs_r_q[12]}}, hofs_r_q};
	/* verilator lint_off UNUSEDSIGNAL */
	wire signed [18:0] hofs_sum_w = {hofs_l18_w[17], hofs_l18_w} + {hofs_r18_w[17], hofs_r18_w};
	/* verilator lint_on UNUSEDSIGNAL */
	wire signed [17:0] hofs_mid_w = hofs_sum_w[18:1];
	wire signed [17:0] hofs_dl_w  = hofs_l18_w - hofs_mid_w;
	wire signed [17:0] hofs_dr_w  = hofs_r18_w - hofs_mid_w;
	wire signed [17:0] hofs_dl_s_w;
	wire signed [17:0] hofs_dr_s_w;
	wire signed [15:0] amp_s_w;

	vip_stereo_scale_signed #(.WIDTH(18)) u_hofs_l_scale (
		.value_i (hofs_dl_w),
		.scale_i (parallax_scale_i),
		.value_o (hofs_dl_s_w)
	);

	vip_stereo_scale_signed #(.WIDTH(18)) u_hofs_r_scale (
		.value_i (hofs_dr_w),
		.scale_i (parallax_scale_i),
		.value_o (hofs_dr_s_w)
	);

	vip_stereo_scale_signed #(.WIDTH(16)) u_amp_scale (
		.value_i (amp_q),
		.scale_i (parallax_scale_i),
		.value_o (amp_s_w)
	);

	wire [15:0] amp_mag_w = amp_s_w[15] ? (~amp_s_w + 16'd1) : amp_s_w;

	// ------------------------------------------------------------------
	// Map cell address: wrap or OVER per map size, then map index.

	wire        aeye_w = k_q[0];
	wire [25:0] aax_w  = aeye_w ? ax_q[1] : ax_q[0];
	wire [25:0] aay_w  = aeye_w ? ay_q[1] : ay_q[0];
	wire [16:0] csx_w  = af_q ? {{4{aax_w[21]}}, aax_w[21:9]} : tile_x_q[16:0];
	wire [16:0] csy_w  = af_q ? {{4{aay_w[21]}}, aay_w[21:9]} : sy_q;

	reg         oob_x_t;
	reg         oob_y_t;
	/* verilator lint_off UNUSEDSIGNAL */
	reg [11:0]  wx_t;
	reg [11:0]  wy_t;
	reg [2:0]   emx_t;
	reg [5:0]   roff_t;
	/* verilator lint_on UNUSEDSIGNAL */
	reg [3:0]   map_idx_t;
	reg [15:0]  cell_addr_t;

	always @* begin
		case (scx_q)
			2'd0: begin
				oob_x_t = csx_w[16] || (|csx_w[15:9]);
				wx_t    = {3'd0, csx_w[8:0]};
			end
			2'd1: begin
				oob_x_t = csx_w[16] || (|csx_w[15:10]);
				wx_t    = {2'd0, csx_w[9:0]};
			end
			2'd2: begin
				oob_x_t = csx_w[16] || (|csx_w[15:11]);
				wx_t    = {1'd0, csx_w[10:0]};
			end
			default: begin
				oob_x_t = csx_w[16] || (|csx_w[15:12]);
				wx_t    = csx_w[11:0];
			end
		endcase

		case (scy_q)
			2'd0: begin
				oob_y_t = csy_w[16] || (|csy_w[15:9]);
				wy_t    = {3'd0, csy_w[8:0]};
			end
			2'd1: begin
				oob_y_t = csy_w[16] || (|csy_w[15:10]);
				wy_t    = {2'd0, csy_w[9:0]};
			end
			2'd2: begin
				oob_y_t = csy_w[16] || (|csy_w[15:11]);
				wy_t    = {1'd0, csy_w[10:0]};
			end
			default: begin
				oob_y_t = csy_w[16] || (|csy_w[15:12]);
				wy_t    = csy_w[11:0];
			end
		endcase

		case (col_log_q)
			2'd0: begin
				emx_t  = 3'd0;
				roff_t = {3'd0, wy_t[11:9]};
			end
			2'd1: begin
				emx_t  = {2'd0, wx_t[9]};
				roff_t = {2'd0, wy_t[11:9], 1'b0};
			end
			2'd2: begin
				emx_t  = {1'b0, wx_t[10:9]};
				roff_t = {1'b0, wy_t[11:9], 2'b00};
			end
			default: begin
				emx_t  = wx_t[11:9];
				roff_t = {wy_t[11:9], 3'b000};
			end
		endcase

		map_idx_t   = mapbase_q + roff_t[3:0] + {1'b0, emx_t};
		cell_addr_t = (over_q && (oob_x_t || oob_y_t)) ? ovp_q : {map_idx_t, wy_t[8:3], wx_t[8:3]};
	end

	// ------------------------------------------------------------------
	// Read steps.

	wire        dret_w     = dram_rvalid_i && dfly_q;
	wire        vret_w     = vram_rvalid_i && vfly_q && vchar_q;
	wire        cell_ret_w = dret_w && (dkind_q == DK_CELL);
	// The cell arriving this clock feeds its character read without a register.
	wire [15:0] cell_w     = cell_ret_w ? dram_rdata_i : cell_q;
	// Normal and H-bias characters take their cell from the buffer.
	wire [15:0] ccell_w    = af_q ? cell_w : cbuf_q[slot_q];
	// The pace takes an arrived cell, or one arriving this clock.
	wire        take_w     = (state_q == S_BURST) && (ar_q || cell_ret_w) &&
		(cgap_q == 4'd0) && ch_n_q && !pc_q;
	wire        dram_free_w = !dpend_q && (!dfly_q || dram_rvalid_i);
	wire        vram_free_w = !vpend_q && (!vfly_q || vram_rvalid_i);
	wire        step_free_w = dram_free_w && vram_free_w;

	wire [11:0] two_w_w    = {npix_q, 1'b0};
	wire [2:0]  nparam_w   = af_q ? 3'd5 : 3'd2;
	wire [2:0]  tile_row_w = sy_q[2:0] + row_ord_q;
	// A tile's last row also reads the next tile's map cell, into the other entry.
	wire        pf_now_w   = ch_n_q && !pf_q && (tiles_left_q != 14'd0) &&
		({1'b0, row_ord_q} == load_rows_q - 4'd1);

	reg         nx_valid_t;
	reg         nx_d_t;
	reg [15:0]  nx_daddr_t;
	reg [1:0]   nx_dkind_t;
	reg         nx_v_t;
	reg [15:0]  nx_vaddr_t;
	reg         nx_vpush_t;

	always @* begin
		nx_valid_t = 1'b0;
		nx_d_t     = 1'b0;
		nx_daddr_t = cell_addr_t;
		nx_dkind_t = DK_CELL;
		nx_v_t     = 1'b0;
		nx_vaddr_t = char_addr(ccell_w, tile_row_w);
		nx_vpush_t = 1'b1;
		case (state_q)
			S_PARAM: begin
				if (pidx_q != nparam_w) begin
					nx_valid_t = 1'b1;
					nx_d_t     = 1'b1;
					nx_dkind_t = DK_PARAM;
					nx_daddr_t = pa_q | {13'd0, pidx_q};
				end
			end
			S_BURST: begin
				if (bi_q != ch_n_q) begin
					nx_valid_t = 1'b1;
					nx_d_t     = 1'b1;
				end
			end
			S_TILES: begin
				if (ct_q != ch_n_q) begin
					nx_valid_t = 1'b1;
					nx_v_t     = 1'b1;
					nx_d_t     = pf_now_w;
				end
			end
			S_PIXELS: begin
				if (k_q <= two_w_w) begin
					nx_valid_t = 1'b1;
					nx_d_t     = k_q != two_w_w;
					nx_v_t     = k_q != 12'd0;
					nx_vaddr_t = char_addr(cell_w, dsy_q);
					nx_vpush_t = deye_q && ((dpx_q == 3'd7) || dlast_q);
				end
			end
			default: begin
			end
		endcase
	end

	// A step whose character read writes the queue waits until the queue will
	// have room when that read returns.
	wire       vpush_now_w = vret_w && vpush_q;
	wire       pop_w;
	wire [1:0] q_count_w   = {1'b0, qv0_q} + {1'b0, qv1_q};
	wire [1:0] q_after_w   = q_count_w + {1'b0, vpush_now_w} - {1'b0, pop_w};
	wire       credit_w    = !(nx_v_t && nx_vpush_t) || (q_after_w < 2'd2);
	// A map cell read does not wait for a character read in flight, nor a
	// character read for the next tile's cell read.
	wire       cell_free_w = (state_q == S_BURST) && dram_free_w;
	wire       row_free_w  = (state_q == S_TILES) && vram_free_w && (!nx_d_t || dram_free_w);
	wire       launch_w    = !abort_i && (step_free_w || cell_free_w || row_free_w) && nx_valid_t && credit_w;

	assign dram_req_o   = dpend_q || (launch_w && nx_d_t);
	assign dram_addr_o  = dpend_q ? daddr_q : nx_daddr_t;
	assign dram_we_o    = 1'b0;
	assign dram_be_o    = 2'b11;
	assign dram_wdata_o = 16'd0;
	assign vram_req_o   = vpend_q || (launch_w && nx_v_t);
	assign vram_addr_o  = vpend_q ? vaddr_q : nx_vaddr_t;
	assign vram_we_o    = 1'b0;
	assign vram_be_o    = 2'b11;
	assign vram_wdata_o = 16'd0;

	// ------------------------------------------------------------------
	// Character decode and write queue entry.

	reg [5:0]         plt_t;
	reg [15:0]        row_pix_t;
	reg [7:0]         row_nz_t;
	reg [1:0]         araw_t;
	reg [1:0]         apix_t;
	reg [15:0]        lpk_pix_t;
	reg [7:0]         lpk_nz_t;
	reg [15:0]        rpk_pix_t;
	reg [7:0]         rpk_nz_t;
	reg [15:0]        pix_t [0:1];
	reg [7:0]         opq_t [0:1];
	reg [1:0]         ent_en_t;
	reg               push_t;
	integer           pi_t;

	always @* begin
		case (vpal_q)
			2'd0:    plt_t = gplt0_i[7:2];
			2'd1:    plt_t = gplt1_i[7:2];
			2'd2:    plt_t = gplt2_i[7:2];
			default: plt_t = gplt3_i[7:2];
		endcase

		for (pi_t = 0; pi_t < 8; pi_t = pi_t + 1) begin
			araw_t = vhflip_q ? vram_rdata_i[2 * (7 - pi_t) +: 2] : vram_rdata_i[2 * pi_t +: 2];
			row_pix_t[2 * pi_t +: 2] = pal_map(araw_t, plt_t);
			row_nz_t[pi_t]           = araw_t != 2'd0;
		end

		// Affine: one pixel into its eye's packer.
		araw_t    = vram_rdata_i[{vsel_q, 1'b0} +: 2];
		apix_t    = pal_map(araw_t, plt_t);
		lpk_pix_t = pkl_pix_q;
		lpk_nz_t  = pkl_nz_q;
		rpk_pix_t = pkr_pix_q;
		rpk_nz_t  = pkr_nz_q;
		if (veye_q) begin
			rpk_pix_t[{vpx_q, 1'b0} +: 2] = apix_t;
			rpk_nz_t[vpx_q]               = araw_t != 2'd0;
		end else begin
			lpk_pix_t[{vpx_q, 1'b0} +: 2] = apix_t;
			lpk_nz_t[vpx_q]               = araw_t != 2'd0;
		end

		pix_t[0] = af_q ? lpk_pix_t : row_pix_t;
		pix_t[1] = af_q ? rpk_pix_t : row_pix_t;
		opq_t[0] = span_mask(lo_q[0], hi_q[0]) & (af_q ? lpk_nz_t : row_nz_t) & {8{en_q[0]}};
		opq_t[1] = span_mask(lo_q[1], hi_q[1]) & (af_q ? rpk_nz_t : row_nz_t) & {8{en_q[1]}};
		ent_en_t = {opq_t[1] != 8'd0, opq_t[0] != 8'd0};
		push_t   = vret_w && (!af_q || vpush_q) && (ent_en_t != 2'b00);
	end

	wire [QW-1:0] entry_w = {vrow_q, ent_en_t,
		base_q[1][9:0], pix_t[1], opq_t[1],
		base_q[0][9:0], pix_t[0], opq_t[0]};

	// ------------------------------------------------------------------
	// Sequencer, read ports and returns.

	integer e_t;

	always @(posedge clk_i) begin
		if (reset_i) begin
			state_q      <= S_IDLE;
			done_o       <= 1'b0;
			dpend_q      <= 1'b0;
			dfly_q       <= 1'b0;
			daddr_q      <= 16'd0;
			dkind_q      <= DK_NONE;
			dpidx_q      <= 3'd0;
			vpend_q      <= 1'b0;
			vfly_q       <= 1'b0;
			vaddr_q      <= 16'd0;
			vchar_q      <= 1'b0;
			en_q         <= 2'b00;
			hb_q         <= 1'b0;
			af_q         <= 1'b0;
			pad_q        <= 8'd0;
			first_tile_q <= 1'b1;
			tiles_left_q <= 14'd0;
			ch_n_q       <= 1'b0;
			bi_q         <= 1'b0;
			ct_q         <= 1'b0;
			cgap_q       <= 4'd0;
			ar_q         <= 1'b0;
			pc_q         <= 1'b0;
			pf_q         <= 1'b0;
			pf_ok_q      <= 1'b0;
			slot_q       <= 1'b0;
			pidx_q       <= 3'd0;
			k_q          <= 12'd0;
			rows_left_q  <= 4'd0;
		end else if (ce_i) begin
			done_o <= 1'b0;
			if (cgap_q != 4'd0) begin
				cgap_q <= cgap_q - 4'd1;
			end

			// Port state. A grant in the clock a read returns starts the next one.
			if (dram_gnt_i) begin
				dfly_q <= 1'b1;
			end else if (dram_rvalid_i) begin
				dfly_q <= 1'b0;
			end
			if (vram_gnt_i) begin
				vfly_q <= 1'b1;
			end else if (vram_rvalid_i) begin
				vfly_q <= 1'b0;
			end
			if (dram_gnt_i) begin
				dpend_q <= 1'b0;
			end
			if (vram_gnt_i) begin
				vpend_q <= 1'b0;
			end

			// Returns.
			if (dret_w) begin
				if (dkind_q == DK_CELL) begin
					cell_q <= dram_rdata_i;
				end
				if (dkind_q == DK_PARAM) begin
					if (af_q) begin
						case (dpidx_q)
							3'd0:    amx_q <= dram_rdata_i;
							3'd1:    amp_q <= dram_rdata_i;
							3'd2:    amy_q <= dram_rdata_i;
							3'd3:    adx_q <= dram_rdata_i;
							default: ady_q <= dram_rdata_i;
						endcase
					end else if (dpidx_q[0]) begin
						hofs_r_q <= dram_rdata_i[12:0];
					end else begin
						hofs_l_q <= dram_rdata_i[12:0];
					end
				end
			end
			if (vret_w && af_q) begin
				pkl_pix_q <= lpk_pix_t;
				pkl_nz_q  <= lpk_nz_t;
				pkr_pix_q <= rpk_pix_t;
				pkr_nz_q  <= rpk_nz_t;
				if (vpush_q) begin
					for (e_t = 0; e_t < 2; e_t = e_t + 1) begin
						lo_q[e_t]   <= lo_q[e_t] - 18'sd8;
						hi_q[e_t]   <= hi_q[e_t] - 18'sd8;
						base_q[e_t] <= base_q[e_t] + 18'sd8;
					end
				end
			end

			// Launch the next step.
			if (launch_w) begin
				if (nx_d_t) begin
					dpend_q <= !dram_gnt_i;
					daddr_q <= nx_daddr_t;
					dkind_q <= nx_dkind_t;
					dpidx_q <= pidx_q;
				end
				if (nx_v_t) begin
					vpend_q  <= !vram_gnt_i;
					vaddr_q  <= nx_vaddr_t;
					vchar_q  <= 1'b1;
					vpush_q  <= nx_vpush_t;
					vpal_q   <= ccell_w[15:14];
					vhflip_q <= ccell_w[13];
					vrow_q   <= y_q[2:0] + row_ord_q;
					veye_q   <= deye_q;
					vsel_q   <= dsx_q ^ {3{cell_w[13]}};
					vpx_q    <= dpx_q;
				end
			end

			case (state_q)
				S_IDLE: begin
					if (cmd_valid_i) begin
						en_q      <= {cw0_w[14], cw0_w[15]};
						hb_q      <= cw0_w[13:12] == 2'd1;
						af_q      <= cw0_w[13:12] == 2'd2;
						scx_q     <= cscx_w;
						scy_q     <= cscy_w;
						col_log_q <= ccol_log_w;
						mapbase_q <= cmapbase_t;
						over_q    <= cw0_w[7];
						gx_q      <= cmd_attr_i[16 +: 10];
						gp_q      <= cmd_attr_i[32 +: 10];
						gy_q      <= cmd_attr_i[48 +: 16];
						mx_q      <= cmd_attr_i[64 +: 13];
						mp_q      <= cmd_attr_i[80 +: 15];
						my_q      <= cmd_attr_i[96 +: 13];
						wraw_q    <= cmd_attr_i[112 +: 13];
						hraw_q    <= cmd_attr_i[128 +: 16];
						pb_q      <= cmd_attr_i[144 +: 16];
						ovp_q     <= cmd_attr_i[160 +: 16];
						strip_q   <= cmd_strip_i;
						state_q   <= S_SETUP1;
					end
				end

				S_SETUP1: begin
					y_q         <= y0_w[8:0];
					rows_left_q <= (rows_w > 18'sd0) ? rows_w[3:0] : 4'd0;
					d_q[0]      <= gx18_w - gp18_w;
					d_q[1]      <= gx18_w + gp18_w;
					sb_q[0]     <= mx18_w - mp18_w;
					sb_q[1]     <= mx18_w + mp18_w;
					if (af_q) begin
						width_q <= {2'd0, {1'b0, wraw_q[9:0]} + 11'd1};
						npix_q  <= {1'b0, wraw_q[9:0]} + 11'd1;
					end else begin
						width_q <= wraw_q[12] ? 13'd0 : ({1'b0, wraw_q[11:0]} + 13'd1);
					end
					pad_q   <= af_q ? 8'd0 : WORLD_PAD;
					state_q <= S_SETUP2;
				end

				S_SETUP2: begin
					for (e_t = 0; e_t < 2; e_t = e_t + 1) begin
						lo_off_q[e_t] <= d_q[e_t][17] ? (18'sd0 - d_q[e_t]) : 18'sd0;
						hi_off_q[e_t] <= min18(width18_w, 18'sd384 - d_q[e_t]);
						a0_q[e_t]     <= af_q ? 18'sd0 : (sb_q[e_t] - un_t0_w);
					end
					tile0_q  <= un_t0_w;
					count0_q <= un_count_w;
					if (pad_q != 8'd0) begin
						pad_q <= pad_q - 8'd1;
					end else if (rows_left_q == 4'd0) begin
						state_q <= S_FINISH;
					end else begin
						pad_q   <= af_q ? AFFINE_PAD : ROW_SETUP;
						state_q <= S_ROW;
					end
				end

				S_ROW: begin
					sy_q         <= row_sy_w;
					load_rows_q  <= (af_q || hb_q) ? 4'd1 : normal_rows_w;
					pa_q         <= pb_q + (af_q ? {local_w[12:0], 3'b000} : {local_w[14:0], 1'b0});
					pidx_q       <= 3'd0;
					tile_x_q     <= tile0_q;
					tiles_left_q <= count0_q;
					ch_n_q       <= count0_q != 14'd0;
					slot_q       <= 1'b0;
					bi_q         <= 1'b0;
					ar_q         <= 1'b0;
					pc_q         <= 1'b0;
					ct_q         <= 1'b0;
					first_tile_q <= 1'b1;
					row_ord_q    <= 3'd0;
					for (e_t = 0; e_t < 2; e_t = e_t + 1) begin
						lo_q[e_t]   <= a0_q[e_t] + lo_off_q[e_t];
						hi_q[e_t]   <= a0_q[e_t] + hi_off_q[e_t];
						base_q[e_t] <= d_q[e_t] - a0_q[e_t];
					end
					if (pad_q != 8'd0) begin
						pad_q <= pad_q - 8'd1;
					end else begin
						state_q <= (af_q || hb_q) ? S_PARAM : S_BURST;
					end
				end

				S_PARAM: begin
					if (launch_w) begin
						pidx_q <= pidx_q + 3'd1;
					end
					if ((pidx_q == nparam_w) && step_free_w) begin
						state_q <= af_q ? S_AINIT : S_HGEOM1;
					end
				end

				S_HGEOM1: begin
					hl_q    <= hofs_mid_w + hofs_dl_s_w;
					hr_q    <= hofs_mid_w + hofs_dr_s_w;
					state_q <= S_HGEOM2;
				end

				S_HGEOM2: begin
					s_q[0]  <= sb_q[0] + hl_q;
					s_q[1]  <= sb_q[1] + hr_q;
					state_q <= S_HGEOM3;
				end

				S_HGEOM3: begin
					tile_x_q     <= un_t0_w;
					tiles_left_q <= un_count_w;
					a0_q[0]      <= s_q[0] - un_t0_w;
					a0_q[1]      <= s_q[1] - un_t0_w;
					state_q      <= S_HGEOM4;
				end

				S_HGEOM4: begin
					for (e_t = 0; e_t < 2; e_t = e_t + 1) begin
						lo_q[e_t]   <= a0_q[e_t] + lo_off_q[e_t];
						hi_q[e_t]   <= a0_q[e_t] + hi_off_q[e_t];
						base_q[e_t] <= d_q[e_t] - a0_q[e_t];
					end
					ch_n_q  <= tiles_left_q != 14'd0;
					state_q <= S_BURST;
				end

				S_AINIT: begin
					// Q13.3 MX/MY to Q17.9; both eyes start at the seed.
					for (e_t = 0; e_t < 2; e_t = e_t + 1) begin
						ax_q[e_t]   <= {{4{amx_q[15]}}, amx_q, 6'b000000};
						ay_q[e_t]   <= {{4{amy_q[15]}}, amy_q, 6'b000000};
						lo_q[e_t]   <= lo_off_q[e_t];
						hi_q[e_t]   <= hi_off_q[e_t];
						base_q[e_t] <= d_q[e_t];
					end
					tx_q      <= {{10{adx_q[15]}}, adx_q};
					ty_q      <= {{10{ady_q[15]}}, ady_q};
					mag_q     <= amp_mag_w;
					// Negative MP offsets the left eye, otherwise the right.
					off_eye_q <= !amp_s_w[15];
					mul_cnt_q <= 4'd0;
					k_q       <= 12'd0;
					state_q   <= S_AMUL;
				end

				S_AMUL: begin
					if (mag_q[0]) begin
						ax_q[off_eye_q] <= ax_q[off_eye_q] + tx_q;
						ay_q[off_eye_q] <= ay_q[off_eye_q] + ty_q;
					end
					tx_q      <= {tx_q[24:0], 1'b0};
					ty_q      <= {ty_q[24:0], 1'b0};
					mag_q     <= {1'b0, mag_q[15:1]};
					mul_cnt_q <= mul_cnt_q + 4'd1;
					if (mul_cnt_q == 4'd15) begin
						state_q <= S_PIXELS;
					end
				end

				S_BURST: begin
					if (launch_w) begin
						tile_x_q     <= tile_x_q + 18'sd8;
						tiles_left_q <= tiles_left_q - 14'd1;
						bi_q         <= 1'b1;
						wi_q         <= slot_q;
					end
					if (cell_ret_w) begin
						ar_q <= 1'b1;
					end
					if (take_w) begin
						pc_q   <= 1'b1;
						cgap_q <= CELL_GAP;
					end
					// The characters start with the last cell's step.
					if ((pc_q == ch_n_q) || take_w) begin
						ct_q      <= 1'b0;
						row_ord_q <= 3'd0;
						state_q   <= S_TILES;
					end
				end

				S_TILES: begin
					if (launch_w && pf_now_w) begin
						tile_x_q     <= tile_x_q + 18'sd8;
						tiles_left_q <= tiles_left_q - 14'd1;
						wi_q         <= !slot_q;
						pf_q         <= 1'b1;
					end
					if (cell_ret_w) begin
						pf_ok_q <= 1'b1;
					end
					if (launch_w) begin
						// The clip spans move on with the first character of each tile.
						first_tile_q <= 1'b0;
						if ((row_ord_q == 3'd0) && !first_tile_q) begin
							for (e_t = 0; e_t < 2; e_t = e_t + 1) begin
								lo_q[e_t]   <= lo_q[e_t] - 18'sd8;
								hi_q[e_t]   <= hi_q[e_t] - 18'sd8;
								base_q[e_t] <= base_q[e_t] + 18'sd8;
							end
						end
						if ({1'b0, row_ord_q} == load_rows_q - 4'd1) begin
							row_ord_q <= 3'd0;
							ct_q      <= 1'b1;
						end else begin
							row_ord_q <= row_ord_q + 3'd1;
						end
					end
					if ((ct_q == ch_n_q) && pf_q) begin
						// The cell read ahead still takes its 2-clock step.
						ch_n_q  <= 1'b1;
						bi_q    <= 1'b1;
						ar_q    <= pf_ok_q || cell_ret_w;
						pc_q    <= 1'b0;
						cgap_q  <= CELL_GAP;
						pf_q    <= 1'b0;
						pf_ok_q <= 1'b0;
						slot_q  <= !slot_q;
						state_q <= S_BURST;
					end
					if ((ct_q == ch_n_q) && !pf_q && step_free_w && (tiles_left_q == 14'd0)) begin
						y_q         <= y_q + {5'd0, load_rows_q};
						rows_left_q <= rows_left_q - load_rows_q;
						if (rows_left_q == load_rows_q) begin
							state_q <= S_FINISH;
						end else begin
							pad_q   <= ROW_SETUP;
							state_q <= S_ROW;
						end
					end
				end

				S_PIXELS: begin
					if (launch_w) begin
						k_q <= k_q + 12'd1;
						if (nx_d_t) begin
							deye_q  <= aeye_w;
							dsx_q   <= aax_w[11:9];
							dsy_q   <= aay_w[11:9];
							dpx_q   <= k_q[3:1];
							dlast_q <= k_q[11:1] == (npix_q - 11'd1);
							ax_q[aeye_w] <= aax_w + {{10{adx_q[15]}}, adx_q};
							ay_q[aeye_w] <= aay_w + {{10{ady_q[15]}}, ady_q};
						end
					end
					if ((k_q == two_w_w + 12'd1) && step_free_w) begin
						y_q         <= y_q + 9'd1;
						rows_left_q <= rows_left_q - 4'd1;
						if (rows_left_q == 4'd1) begin
							state_q <= S_FINISH;
						end else begin
							pad_q   <= AFFINE_PAD;
							state_q <= S_ROW;
						end
					end
				end

				S_FINISH: begin
					if (!qv0_q && !qv1_q && !push_t) begin
						done_o  <= 1'b1;
						state_q <= S_IDLE;
					end
				end

				default: begin
					state_q <= S_IDLE;
				end
			endcase

			// XPRST: requests not yet granted are withdrawn; reads in flight
			// still return but nothing uses them.
			if (abort_i) begin
				state_q <= S_IDLE;
				done_o  <= 1'b0;
				dpend_q <= 1'b0;
				vpend_q <= 1'b0;
				dkind_q <= DK_NONE;
				vchar_q <= 1'b0;
				pf_q    <= 1'b0;
				pf_ok_q <= 1'b0;
			end
		end
	end

	// Cell buffer write.
	always @(posedge clk_i) begin
		if (ce_i && dret_w && (dkind_q == DK_CELL) && !af_q) begin
			cbuf_q[wi_q] <= dram_rdata_i;
		end
	end

	// ------------------------------------------------------------------
	// Write queue and strip buffer writes.

	wire        head_en_r_w = q0_q[QW-4];
	wire        head_en_l_w = q0_q[QW-5];
	wire        head_eye_w  = !head_en_l_w || ldone_q;
	wire [33:0] head_half_w = head_eye_w ? q0_q[67:34] : q0_q[33:0];

	assign pop_w       = qv0_q && sb_ready_i && (head_eye_w || !head_en_r_w);
	assign sb_valid_o  = qv0_q;
	assign sb_eye_o    = head_eye_w;
	assign sb_row_o    = q0_q[QW-1 -: 3];
	assign sb_x_o      = head_half_w[33:24];
	assign sb_pixels_o = head_half_w[23:8];
	assign sb_opaque_o = head_half_w[7:0];

	always @(posedge clk_i) begin
		if (reset_i) begin
			qv0_q   <= 1'b0;
			qv1_q   <= 1'b0;
			ldone_q <= 1'b0;
		end else if (ce_i) begin
			if (abort_i) begin
				qv0_q   <= 1'b0;
				qv1_q   <= 1'b0;
				ldone_q <= 1'b0;
			end else begin
				if (qv0_q && sb_ready_i) begin
					ldone_q <= !pop_w;
				end
				case ({push_t, pop_w})
					2'b10: begin
						if (!qv0_q) begin
							q0_q  <= entry_w;
							qv0_q <= 1'b1;
						end else begin
							q1_q  <= entry_w;
							qv1_q <= 1'b1;
						end
					end
					2'b01: begin
						q0_q  <= q1_q;
						qv0_q <= qv1_q;
						qv1_q <= 1'b0;
					end
					2'b11: begin
						if (qv1_q) begin
							q0_q <= q1_q;
							q1_q <= entry_w;
						end else begin
							q0_q <= entry_w;
						end
					end
					default: begin
					end
				endcase
			end
		end
	end

endmodule
