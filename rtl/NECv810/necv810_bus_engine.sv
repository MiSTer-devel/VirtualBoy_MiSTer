// Copyright (c) 2026 Jamie Blanks

// NEC V810 bus unit.
//
//   posted stores -> staging register --+
//   fetch, data and system requests ----+-> request -> cycle registers, OPW -> A, BE, ST, R/W, MRQ, D out
//                                                           |
//   READY, SZRQ -> phi1 sample ----------------------> sequencer T1, T2, T2... -> BCYST, busy
//   D in -> read latch (OPRL + pins) -> read data
//
// Every cycle is a T1 with BCYST, then a T2 that repeats while READY is low.
// READY and SZRQ are taken at phi1, the falling edge in the middle of T2;
// read data at the CE edge that ends the last T2. The unit is busy until that
// last T2, so the next request starts at the edge that ends it.
//
// At a cycle boundary a due store goes first, then a data or system cycle,
// then a fetch. A data, store or system cycle started from an idle bus shows
// its T1 from phi1 of the clock it is requested in, as its request is known
// by then; it enters T2 at the CE. Any other cycle takes a full T1 clock.
//
// SIZ16B selects fixed 16-bit mode; SZRQ low sizes a word onto D15:0. A sized
// word runs as a second full cycle, low half first, with its own T1 and
// BCYST. Upper-half narrow writes also drive the low lanes. IO_MRQ = 1 drops
// MRQ for IN/OUT cycles.
module necv810_bus_engine #(
	parameter IO_MRQ = 0
) (
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        clk_en_i,
	input  wire        phi1_i,
	input  wire        hldrq_i,
	input  wire        lock_i,
	output wire        hldak_o,

	input  wire        launch_i,
	input  wire [31:0] launch_addr_i,
	input  wire        launch_fetch_i,
	input  wire        launch_fetch_branch_i,
	input  wire        launch_write_i,
	input  wire        launch_io_i,
	// 01 machine fault, 11 HALT, 00 ordinary.
	input  wire [1:0]  launch_sys_i,
	input  wire        launch_addr_error_i,
	input  wire [1:0]  launch_size_i,
	input  wire [31:0] launch_wdata_i,
	input  wire        release_halt_i,
	// Stores stage here as they leave MEM, beside any launch.
	input  wire        push_i,
	input  wire [31:0] push_addr_i,
	input  wire        push_io_i,
	input  wire        push_addr_error_i,
	input  wire [1:0]  push_size_i,
	input  wire [31:0] push_wdata_i,

	input  wire        fixed_16_i,
	input  wire        szrq_i,
	input  wire        ready_i,
	input  wire [31:0] din_i,
	input  wire [31:1] idle_addr_i,

	output wire        posted_ready_o,
	output wire        launch_accept_o,
	output wire        first_beat_o,
	output wire        complete_o,
	output wire [31:0] read_data_o,

	output wire        active_o,
	output wire        active_fetch_o,
	output wire        active_posted_o,
	output wire        posted_pending_o,
	output wire        posted_due_o,
	output wire        split_pending_o,    // the word on the bus has a second cycle to come

	output wire [31:1] a_o,
	output wire [31:0] dout_o,
	output wire        dout_oe_o,
	output wire [3:0]  be_o,
	output wire [1:0]  st_o,
	output wire        da_o,
	output wire        mrq_o,
	output wire        rw_o,
	output wire        bcyst_o,
	output wire        adrs_err_n_o,
	output wire        cycle_tag_o,
	output wire        beat_t1_o
);

	localparam [1:0] SIZE_B = 2'b00;
	localparam [1:0] SIZE_W = 2'b10;

	// ---------------------------------------------------------------
	// Falling-edge inputs; READY and SZRQ qualify the current T2.
	// ---------------------------------------------------------------
	reg        ready_s_q;
	reg        szrq_s_q;
	reg        hldrq_s_q;
	always @(posedge clk_i or posedge reset_i) begin
		if (reset_i) begin
			ready_s_q <= 1'b0;
			szrq_s_q <= 1'b1;
		end else if (phi1_i) begin
			ready_s_q <= ready_i;
			szrq_s_q <= szrq_i;
		end
	end
	always @(posedge clk_i) begin
		if (reset_i || phi1_i) hldrq_s_q <= hldrq_i;
	end

	// ---------------------------------------------------------------
	// Cycle registers and sequencer.
	// ---------------------------------------------------------------
	reg        active_q;
	reg        t1_q;               // this clock is the cycle's T1
	reg [31:0] addr_q;
	reg        fetch_q;
	reg        fetch_branch_q;
	reg        write_q;
	reg        posted_q;
	reg        io_q;
	reg [1:0]  sys_q;              // a HALT acknowledge keeps its status through idle
	reg        addr_error_q;
	reg [1:0]  size_q;
	reg        phase_q;            // the second cycle of a sized word
	reg [31:0] wdata_q;            // OPW, the write register
	reg [15:0] oprl_q;             // low half of a sized word, for the read latch
	reg        cycle_tag_q;
	reg        hold_q;
	reg        hold_resume_q;
	reg [31:1] hold_a_q;
	reg [3:0]  hold_be_q;
	reg [1:0]  hold_st_q;
	reg        hold_dout_oe_q;

	wire       low_lanes_w = fixed_16_i || !szrq_s_q;
	wire       split_first_w = (size_q == SIZE_W) && !phase_q && low_lanes_w;
	// The CE edge that ends a T2 with READY takes the data.
	wire       accepted_w = clk_en_i && active_q && !t1_q && ready_s_q;
	wire       complete_w = accepted_w && !split_first_w;
	// Busy until the request's last T2; the next request starts as it ends.
	wire       busy_w = active_q && !(!t1_q && ready_s_q && !split_first_w);

	// ---------------------------------------------------------------
	// Staging register: one posted store, due as it enters. It moves into
	// OPW as its cycle starts. The next store may stage once the first
	// halfword of the store on the bus is written: all of a byte or
	// halfword, the low half of a word.
	// ---------------------------------------------------------------
	reg        wb0_q;
	reg [31:0] wb0_addr_q;
	reg        wb0_io_q;
	reg        wb0_err_q;
	reg [1:0]  wb0_size_q;
	reg [31:0] wb0_data_q;

	wire       wb_due_w = wb0_q;
	wire       store_first_w = active_q && posted_q && !phase_q && !(!t1_q && ready_s_q);
	wire       posted_ready_w = !wb0_q && !store_first_w;
	wire       push_accept_w = clk_en_i && push_i && posted_ready_w;

	// The request for the next cycle. A fetch waits only for a due store;
	// other cycles wait for the buffer to drain, which keeps memory order.
	wire        bus_owned_w = !hold_q && (!hldrq_s_q || lock_i);
	wire        launch_ready_w = bus_owned_w && !busy_w &&
		(launch_fetch_i ? !wb_due_w : (!wb0_q && !push_i));
	wire        launch_accept_w = clk_en_i && launch_i && launch_ready_w;
	wire        req_w = bus_owned_w && (wb_due_w || (launch_i && launch_ready_w));
	wire        req_store_w = wb_due_w;
	wire        req_fetch_w = !req_store_w && launch_fetch_i;
	wire        req_write_w = req_store_w || launch_write_i;
	wire [31:0] req_addr_w = req_store_w ? wb0_addr_q : launch_addr_i;
	wire [1:0]  req_size_w = req_store_w ? wb0_size_q : launch_size_i;
	wire        req_io_w = req_store_w ? wb0_io_q : launch_io_i;
	wire        req_err_w = req_store_w ? wb0_err_q : launch_addr_error_i;
	wire [1:0]  req_sys_w = req_store_w ? 2'b00 : launch_sys_i;
	wire [31:0] req_wdata_w = req_store_w ? wb0_data_q : launch_wdata_i;
	wire        start_w = clk_en_i && !busy_w && req_w;

	// ---------------------------------------------------------------
	// Idle-start T1. From phi1 of an idle clock the pins show the data, store
	// or system cycle the CE will start, with the next cycle tag. DA waits for
	// T2, so no write starts early. The request comes from registered state
	// and has settled by phi1.
	// ---------------------------------------------------------------
	wire        early_go_w = !active_q && req_w && !req_fetch_w;
	reg         early_q;
	reg  [31:1] early_a_q;
	reg  [3:0]  early_be_q;
	reg  [1:0]  early_st_q;
	reg         early_mrq_q;
	reg         early_rw_q;
	reg         early_err_q;
	always @(posedge clk_i or posedge reset_i) begin
		if (reset_i) begin
			early_q <= 1'b0;
			early_a_q <= 31'd0;
			early_be_q <= 4'b0000;
			early_st_q <= 2'b00;
			early_mrq_q <= 1'b0;
			early_rw_q <= 1'b1;
			early_err_q <= 1'b0;
		end else if (clk_en_i) begin
			early_q <= 1'b0;
		end else if (phi1_i) begin
			// The sampled hold request wins over an idle-start T1 at this edge.
			early_q <= early_go_w && (!hldrq_i || lock_i);
			if (early_go_w && (!hldrq_i || lock_i)) begin
				early_a_q <= {req_addr_w[31:2], req_addr_w[1] && fixed_16_i};
				early_be_q <= byte_enable_fn(fixed_16_i, req_size_w, 1'b0, req_addr_w[1:0]);
				early_st_q <= (req_sys_w != 2'b00) ? req_sys_w : 2'b10;
				early_mrq_q <= (req_sys_w == 2'b00) && !(IO_MRQ && req_io_w);
				early_rw_q <= !req_write_w;
				early_err_q <= req_err_w;
			end
		end
	end
	// The cycle that starts is the one shown; it goes straight to T2.
	wire        early_hit_w = early_q && start_w && !req_fetch_w;

	// Finish the current word, then leave a TI before granting the bus.
	// Sampling during reset keeps an acknowledged hold across reset release.
	always @(posedge clk_i) begin
		if (reset_i) begin
			hold_q <= hldrq_i;
			hold_resume_q <= 1'b0;
			hold_a_q <= 31'd0;
			hold_be_q <= 4'd0;
			hold_st_q <= 2'd0;
			hold_dout_oe_q <= 1'b0;
		end else if (clk_en_i) begin
			hold_resume_q <= 1'b0;
			if (hold_q) begin
				if (!hldrq_s_q) begin
					hold_q <= 1'b0;
					hold_resume_q <= 1'b1;
				end
			end else if (hldrq_s_q && !lock_i && !active_q && !early_q) begin
				hold_q <= 1'b1;
				hold_a_q <= a_o;
				hold_be_q <= be_o;
				hold_st_q <= st_o;
				hold_dout_oe_q <= dout_oe_o;
			end
		end
	end
	assign hldak_o = hold_q;

	always @(posedge clk_i or posedge reset_i) begin
		if (reset_i) begin
			active_q <= 1'b0;
			t1_q <= 1'b0;
			addr_q <= 32'd0;
			fetch_q <= 1'b1;
			fetch_branch_q <= 1'b1;
			write_q <= 1'b0;
			posted_q <= 1'b0;
			io_q <= 1'b0;
			sys_q <= 2'b00;
			addr_error_q <= 1'b0;
			size_q <= 2'b01;
			phase_q <= 1'b0;
			wdata_q <= 32'd0;
			oprl_q <= 16'd0;
			cycle_tag_q <= 1'b0;
		end else if (clk_en_i) begin
			t1_q <= 1'b0;
			if (release_halt_i && !active_q && (sys_q == 2'b11)) sys_q <= 2'b00;
			if (accepted_w && split_first_w) begin
				// The low half is latched; the high half runs as its own cycle.
				if (!write_q) oprl_q <= read_halfword_fn(din_i, low_lanes_w, addr_q[1]);
				phase_q <= 1'b1;
				t1_q <= 1'b1;
				cycle_tag_q <= ~cycle_tag_q;
			end else if (start_w) begin
				active_q <= 1'b1;
				t1_q <= !early_hit_w;
				addr_q <= req_addr_w;
				fetch_q <= req_fetch_w;
				fetch_branch_q <= !req_store_w && launch_fetch_branch_i;
				write_q <= req_write_w;
				posted_q <= req_store_w;
				io_q <= req_io_w;
				sys_q <= req_sys_w;
				addr_error_q <= req_err_w;
				size_q <= req_size_w;
				phase_q <= 1'b0;
				wdata_q <= req_wdata_w;
				// A shown T1 that another cycle replaced keeps its tag for itself.
				if (!(early_q && !early_hit_w)) cycle_tag_q <= ~cycle_tag_q;
			end else if (complete_w) begin
				active_q <= 1'b0;
			end
		end
	end

	always @(posedge clk_i or posedge reset_i) begin
		if (reset_i) begin
			wb0_q <= 1'b0;
			wb0_addr_q <= 32'd0;
			wb0_io_q <= 1'b0;
			wb0_err_q <= 1'b0;
			wb0_size_q <= 2'b01;
			wb0_data_q <= 32'd0;
		end else if (clk_en_i) begin
			if (start_w && req_store_w) wb0_q <= 1'b0;
			if (push_accept_w) begin
				wb0_q <= 1'b1;
				wb0_addr_q <= push_addr_i;
				wb0_io_q <= push_io_i;
				wb0_err_q <= push_addr_error_i;
				wb0_size_q <= push_size_i;
				wb0_data_q <= push_wdata_i;
			end
		end
	end

	assign posted_ready_o = posted_ready_w;
	assign launch_accept_o = launch_accept_w;
	assign first_beat_o = accepted_w && split_first_w;
	assign complete_o = complete_w;
	assign read_data_o = read_data_fn(din_i, oprl_q, low_lanes_w, size_q, phase_q, addr_q[1:0]);

	assign active_o = active_q;
	assign active_fetch_o = fetch_q;
	assign active_posted_o = posted_q;
	assign posted_pending_o = wb0_q;
	assign posted_due_o = wb_due_w;
	assign split_pending_o = active_q && split_first_w;

	// ---------------------------------------------------------------
	// Pins. A sized word's second cycle raises A1 (32-bit mode keeps A1 low
	// otherwise). A HALT acknowledge holds its address and status after it ends.
	// ---------------------------------------------------------------
	wire        shown_w = active_q || (sys_q != 2'b00);
	// Restore the saved pin levels through TI, until the next T1.
	wire        hold_pins_w = hold_q || hold_resume_q;
	wire [31:1] cycle_a_w = {addr_q[31:2], phase_q || (addr_q[1] && fixed_16_i)};
	assign a_o = early_q ? early_a_q : hold_pins_w ? hold_a_q : shown_w ? cycle_a_w : idle_addr_i;
	assign be_o = early_q ? early_be_q : hold_pins_w ? hold_be_q :
		shown_w ? byte_enable_fn(fixed_16_i, size_q, phase_q, addr_q[1:0]) : 4'b0000;
	assign st_o = early_q ? early_st_q : hold_pins_w ? hold_st_q :
		(sys_q != 2'b00) ? sys_q : !active_q ? 2'b00 : fetch_q ?
		(fetch_branch_q ? 2'b01 : 2'b11) : 2'b10;
	assign da_o = active_q && !fetch_q;
	assign mrq_o = early_q ? early_mrq_q : (active_q && (sys_q == 2'b00) && !(IO_MRQ && io_q));
	assign rw_o = early_q ? early_rw_q : !write_q;
	assign bcyst_o = early_q || (active_q && t1_q);
	assign adrs_err_n_o = early_q ? !early_err_q : !(active_q && addr_error_q);
	assign cycle_tag_o = early_q ? !cycle_tag_q : cycle_tag_q;
	assign beat_t1_o = bcyst_o;
	assign dout_o = write_data_fn(wdata_q, fixed_16_i, size_q, phase_q, addr_q[1:0]);
	assign dout_oe_o = hold_pins_w ? hold_dout_oe_q : shown_w && write_q;

`ifndef SYNTHESIS
	// synthesis translate_off
	always @(posedge clk_i) begin
		if (!reset_i && early_hit_w &&
			(({req_addr_w[31:2], req_addr_w[1] && fixed_16_i} != early_a_q) || (!req_write_w != early_rw_q))) begin
			$display("FAIL idle T1 launch %h differs from the one shown %h", req_addr_w, {early_a_q, 1'b0});
			$fatal(1);
		end
		if (!reset_i && accepted_w && split_first_w && addr_q[1]) begin
			$display("FAIL sized word at %h is not word aligned", addr_q);
			$fatal(1);
		end
	end
	// Idle starts: shown from phi1 and started as shown, shown but none
	// started, shown but replaced, and started without being shown (full T1).
	integer early_hit_n, early_drop_n, early_swap_n, early_late_n;
	initial begin
		early_hit_n = 0;
		early_drop_n = 0;
		early_swap_n = 0;
		early_late_n = 0;
	end
	always @(posedge clk_i) begin
		if (!reset_i && clk_en_i) begin
			if (early_hit_w)
				early_hit_n <= early_hit_n + 1;
			else if (early_q && start_w)
				early_swap_n <= early_swap_n + 1;
			else if (early_q)
				early_drop_n <= early_drop_n + 1;
			else if (!active_q && start_w && !req_fetch_w)
				early_late_n <= early_late_n + 1;
		end
	end
	final if (early_hit_n + early_drop_n + early_swap_n + early_late_n != 0)
		$display("%m idle T1: %0d shown, %0d dropped, %0d replaced, %0d not shown",
			early_hit_n, early_drop_n, early_swap_n, early_late_n);
	// synthesis translate_on
`endif

	function [3:0] byte_enable_fn;
		input fixed_16;
		input [1:0] size;
		input phase;
		input [1:0] addr_low;
		begin
			if (fixed_16) begin
				byte_enable_fn = (size == SIZE_B) ?
					(addr_low[0] ? 4'b1101 : 4'b1110) : 4'b1100;
			end else if (phase) begin
				byte_enable_fn = 4'b0011;
			end else begin
				case (size)
					SIZE_B: begin
						case (addr_low)
							2'b00: byte_enable_fn = 4'b1110;
							2'b01: byte_enable_fn = 4'b1101;
							2'b10: byte_enable_fn = 4'b1011;
							default: byte_enable_fn = 4'b0111;
						endcase
					end
					2'b01: byte_enable_fn = addr_low[1] ? 4'b0011 : 4'b1100;
					default: byte_enable_fn = 4'b0000;
				endcase
			end
		end
	endfunction

	// OPW onto the pins: the high half goes to D31:16 or D15:0, the low half to D15:0.
	function [31:0] write_data_fn;
		input [31:0] data;
		input fixed_16;
		input [1:0] size;
		input phase;
		input [1:0] addr_low;
		begin
			if (fixed_16) begin
				case (size)
					// Fixed-16 byte stores still place all source bits 15:0 on the bus.
					SIZE_B: write_data_fn = addr_low[0] ?
						{16'd0, data[7:0], 8'd0} : {16'd0, data[15:0]};
					SIZE_W: write_data_fn = phase ?
						{16'd0, data[31:16]} : {16'd0, data[15:0]};
					default: write_data_fn = {16'd0, data[15:0]};
				endcase
			end else if (phase) begin
				write_data_fn = {data[31:16], data[31:16]};
			end else begin
				case (size)
					SIZE_B: begin
						case (addr_low)
							2'b00: write_data_fn = {24'd0, data[7:0]};
							2'b01: write_data_fn = {16'd0, data[7:0], 8'd0};
							2'b10: write_data_fn = {8'd0, data[7:0], 8'd0, data[7:0]};
							default: write_data_fn = {data[7:0], 8'd0, data[7:0], 8'd0};
						endcase
					end
					2'b01: write_data_fn = addr_low[1] ?
						{data[15:0], data[15:0]} : {16'd0, data[15:0]};
					default: write_data_fn = data;
				endcase
			end
		end
	endfunction

	function [15:0] read_halfword_fn;
		input [31:0] data;
		input low_lanes;
		input addr_bit1;
		begin
			read_halfword_fn = (low_lanes || !addr_bit1) ? data[15:0] : data[31:16];
		end
	endfunction

	// The read latch onto the internal bus: OPRL and the high half from the
	// pins, with bytes and halfwords aligned to bit 0.
	function [31:0] read_data_fn;
		input [31:0] data;
		input [15:0] oprl;
		input low_lanes;
		input [1:0] size;
		input phase;
		input [1:0] addr_low;
		reg [7:0] byte_v;
		begin
			byte_v = 8'd0;
			if (low_lanes) begin
				byte_v = addr_low[0] ? data[15:8] : data[7:0];
			end else begin
				case (addr_low)
					2'b00: byte_v = data[7:0];
					2'b01: byte_v = data[15:8];
					2'b10: byte_v = data[23:16];
					default: byte_v = data[31:24];
				endcase
			end
			case (size)
				SIZE_B: read_data_fn = {24'd0, byte_v};
				SIZE_W: read_data_fn = phase ? {data[15:0], oprl} : data;
				default: read_data_fn = {16'd0, read_halfword_fn(data, low_lanes, addr_low[1])};
			endcase
		end
	endfunction

endmodule
