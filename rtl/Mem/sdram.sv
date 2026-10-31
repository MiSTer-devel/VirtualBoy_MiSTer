`timescale 1ns/1ps
`default_nettype none

// MiSTer SDR SDRAM controller: three 16-bit busy/request ports, closed page.
// Derived from Marty_MiSTer rtl/mem/sdram.sv (sha256 08ac2694...): its pin
// layer, JEDEC init, DDIO clock forward and capture select, registered wait
// compares and asynchronous reset are kept. The scheduler is replaced:
//
// - A request is turned into ACTIVE on the first edge it is seen while the
//   controller is idle, so a client that presents an address at a fixed time
//   gets its data at a fixed time.
// - Every access is one 16-bit beat with auto-precharge; no row stays open.
// - Refresh comes only from rising edges of `refresh`, counted as debt and
//   paid in idle slots ahead of p2 and behind p0/p1.
//
// Client contract: `req` is a level. Hold it until `busy` rises, which is the
// acknowledgement; a request that misses its window is simply not taken.
// `ready` falls on acceptance and rises with `dout` valid, or on the WRITE
// edge for a write. Each port has its own read register. Priority p0, p1,
// refresh, p2. `addr` is a byte address: BA = addr[25:24], row = addr[23:11],
// column = addr[10:1]. `be` enables the low/high byte of a write.
//
// Read timing at 80 MHz, CL2, falling capture (DQ_CAPTURE_PIPELINE 2):
//   ACTIVE at A, READ at A+tRCD, the read register at READ+CL+2.
//
// MiSTer boards have no DQM pins, so DQML/DQMH alias SDRAM_A[11]/A[12].
module sdram #(
	parameter int unsigned CLK_FREQ_HZ = 80_000_000,
	parameter int unsigned SDRAM_TIMING_GRADE = 7,
	// Cycles from the input cell's capture edge to the edge that consumes the
	// beat. Re-measure after a clock rate change; see CAS_READ_CYCLES.
	parameter int unsigned DQ_CAPTURE_PIPELINE = 2,
	// Which state-clock edge the input cell samples DQ on. The chip drives a
	// beat tAC after its own edge (the falling state-clock edge) and holds it
	// tOH past the next one. Measured: 85.9 and 128.9 MHz falling, 114.5 MHz
	// rising (Marty); 80 MHz falling (RC5).
	parameter bit DQ_CAPTURE_RISING = 1'b0
) (
	input  wire        clk,
	// clk's rate, phase-shifted early; drives only the forwarded SDRAM_CLK.
	input  wire        clk_pin,
	input  wire        reset,

	input  wire        refresh,

	input  wire        p0_req,
	input  wire        p0_we,
	input  wire [25:0] p0_addr,
	input  wire [15:0] p0_din,
	input  wire [1:0]  p0_be,
	output logic [15:0] p0_dout,
	output wire        p0_busy,
	output logic       p0_ready,

	input  wire        p1_req,
	input  wire        p1_we,
	input  wire [25:0] p1_addr,
	input  wire [15:0] p1_din,
	input  wire [1:0]  p1_be,
	output logic [15:0] p1_dout,
	output wire        p1_busy,
	output logic       p1_ready,

	input  wire        p2_req,
	input  wire        p2_we,
	input  wire [25:0] p2_addr,
	input  wire [15:0] p2_din,
	input  wire [1:0]  p2_be,
	output logic [15:0] p2_dout,
	output wire        p2_busy,
	output logic       p2_ready,

	output wire        init_done,
	// No access, refresh or close timing in progress and no refresh owed.
	output wire        idle,

	output wire        SDRAM_CLK,
	output logic       SDRAM_CKE,
	output logic [12:0] SDRAM_A,
	output logic [1:0] SDRAM_BA,
	inout  wire [15:0] SDRAM_DQ,
	output wire        SDRAM_DQML,
	output wire        SDRAM_DQMH,
	output logic       SDRAM_nCS,
	output logic       SDRAM_nCAS,
	output logic       SDRAM_nRAS,
	output logic       SDRAM_nWE
);

	localparam logic [3:0] CMD_DESELECT      = 4'b1111;
	localparam logic [3:0] CMD_NOP           = 4'b0111;
	localparam logic [3:0] CMD_ACTIVE        = 4'b0011;
	localparam logic [3:0] CMD_READ          = 4'b0101;
	localparam logic [3:0] CMD_WRITE         = 4'b0100;
	localparam logic [3:0] CMD_PRECHARGE     = 4'b0010;
	localparam logic [3:0] CMD_AUTO_REFRESH  = 4'b0001;
	localparam logic [3:0] CMD_LOAD_MODE     = 4'b0000;

	function automatic [15:0] cycles_for_ns;
		input [31:0] freq_hz;
		input [31:0] ns;
		reg [63:0] num;
		reg [63:0] cycles;
		begin
			num = ({32'd0, freq_hz} * {32'd0, ns}) + 64'd999_999_999;
			cycles = num / 64'd1_000_000_000;
			if (cycles < 64'd1) cycles_for_ns = 16'd1;
			else cycles_for_ns = cycles[15:0];
		end
	endfunction

	function automatic [15:0] max16;
		input [15:0] a;
		input [15:0] b;
		begin
			if (a > b) max16 = a;
			else max16 = b;
		end
	endfunction

	// AS4C32M16SB supports CL2 through 100 MHz and CL3 above.
	localparam logic [2:0] CAS_LATENCY = (CLK_FREQ_HZ <= 100_000_000) ? 3'd2 : 3'd3;
	// Burst length 1, single-location writes.
	localparam logic [12:0] MODE_REG = {3'b000, 1'b1, 2'b00, CAS_LATENCY, 1'b0, 3'b000};
	// READ registered at T0; the chip samples it half a period later (SDRAM_CLK
	// is ~clk) and drives the beat CL-1 cycles on. The input cell samples it on
	// a falling clk edge, realigns it on the next rising edge, and the state
	// machine reads it at T0+CL+2 with falling capture.
	localparam logic [15:0] CAS_READ_CYCLES =
		{13'd0, CAS_LATENCY} + DQ_CAPTURE_PIPELINE[15:0];

	// AS4C32M16SB timing in ns. Grade 6 is -6TIN; anything else uses -7.
	localparam logic TIMING_GRADE_6 = (SDRAM_TIMING_GRADE == 6);
	localparam logic [31:0] T_RCD_NS = TIMING_GRADE_6 ? 32'd18 : 32'd21;
	localparam logic [31:0] T_RP_NS  = TIMING_GRADE_6 ? 32'd18 : 32'd21;
	localparam logic [31:0] T_RC_NS  = TIMING_GRADE_6 ? 32'd60 : 32'd63;
	localparam logic [31:0] T_RFC_NS = TIMING_GRADE_6 ? 32'd60 : 32'd63;
	localparam logic [31:0] T_WR_NS  = TIMING_GRADE_6 ? 32'd12 : 32'd14;
	localparam logic [31:0] T_MRD_NS = TIMING_GRADE_6 ? 32'd12 : 32'd14;
	localparam logic [31:0] T_RAS_NS = 32'd42;

`ifndef SYNTHESIS
	initial begin
		if ((CAS_LATENCY == 3'd2) && (CLK_FREQ_HZ > 100_000_000)) begin
			$error("sdram: CL2 is not valid above 100 MHz");
		end
	end
`endif

	localparam logic [15:0] T_POWER_WAIT = cycles_for_ns(CLK_FREQ_HZ[31:0], 32'd200_000);
	localparam logic [15:0] T_RCD = cycles_for_ns(CLK_FREQ_HZ[31:0], T_RCD_NS);
	localparam logic [15:0] T_RP  = cycles_for_ns(CLK_FREQ_HZ[31:0], T_RP_NS);
	localparam logic [15:0] T_RC  = cycles_for_ns(CLK_FREQ_HZ[31:0], T_RC_NS);
	localparam logic [15:0] T_RFC = cycles_for_ns(CLK_FREQ_HZ[31:0], T_RFC_NS);
	localparam logic [15:0] T_WR  = cycles_for_ns(CLK_FREQ_HZ[31:0], T_WR_NS);
	localparam logic [15:0] T_MRD = cycles_for_ns(CLK_FREQ_HZ[31:0], T_MRD_NS);
	localparam logic [15:0] T_RAS = cycles_for_ns(CLK_FREQ_HZ[31:0], T_RAS_NS);
	// ACTIVE to the next ACTIVE or REFRESH of the same access: the row cycle,
	// the READ auto-precharge (tRAS then tRP) and the WRITE auto-precharge
	// (one more clock, tWR then tRP).
	localparam logic [15:0] T_ACT_GAP = max16(max16(T_RC, T_RAS + T_RP),
		T_RCD + 16'd1 + T_WR + T_RP);

	// The power-up interval has its own counter; every other wait fits a
	// narrow one, which keeps the zero tests in front of the pins small.
	localparam int unsigned WAIT_W = 5;
	localparam logic [WAIT_W-1:0] W_RCD = T_RCD[WAIT_W-1:0] - 1'd1;
	localparam logic [WAIT_W-1:0] W_RP  = T_RP[WAIT_W-1:0] - 1'd1;
	localparam logic [WAIT_W-1:0] W_RFC = T_RFC[WAIT_W-1:0] - 1'd1;
	localparam logic [WAIT_W-1:0] W_MRD = T_MRD[WAIT_W-1:0] - 1'd1;
	localparam logic [WAIT_W-1:0] W_CAS = CAS_READ_CYCLES[WAIT_W-1:0] - 1'd1;
	localparam logic [WAIT_W-1:0] W_GAP = T_ACT_GAP[WAIT_W-1:0] - 1'd1;

`ifndef SYNTHESIS
	initial begin
		if (max16(max16(T_ACT_GAP, T_RFC), CAS_READ_CYCLES) >= (16'd1 << WAIT_W)) begin
			$error("sdram: WAIT_W is too narrow");
		end
	end
`endif

	typedef enum logic [3:0] {
		ST_POWER_WAIT,
		ST_NOP_WAIT,
		ST_INIT_TRP,
		ST_INIT_RFC1,
		ST_INIT_RFC2,
		ST_INIT_MRD,
		ST_IDLE,
		ST_RCD,
		ST_READ_WAIT,
		ST_REFRESH_WAIT
	} state_e;

	state_e state_q;
	logic init_done_q;
	logic [15:0] init_wait_q;
	logic [WAIT_W-1:0] wait_q;
	logic wait_zero_q;
	// Counts the ACTIVE-to-ACTIVE gap beside the command waits; the next
	// ACTIVE or REFRESH waits for it in ST_IDLE.
	logic [WAIT_W-1:0] gap_q;
	logic gap_zero_q;

	logic [1:0] active_port_q;
	logic active_we_q;
	logic [9:0] active_col_q;
	logic [15:0] active_din_q;
	logic [1:0] active_be_q;

	logic p0_busy_q;
	logic p1_busy_q;
	logic p2_busy_q;

	logic [2:0] refresh_debt_q;
	logic refresh_old_q;

	logic [15:0] dq_out_q;
	logic dq_oe_q;
	wire [15:0] dq_in;
	wire [15:0] dq_aligned_w;

	// A new access or refresh may start on this edge.
	wire slot_free = init_done_q && (state_q == ST_IDLE) && gap_zero_q;
	wire refresh_owed = refresh_debt_q != 3'd0;
	wire p0_accept_i = slot_free && !p0_busy_q;
	wire p1_accept_i = slot_free && !p1_busy_q && !p0_req;
	wire p2_accept_i = slot_free && !p2_busy_q && !p0_req && !p1_req && !refresh_owed;
	wire p0_take = p0_req && p0_accept_i;
	wire p1_take = p1_req && p1_accept_i;
	wire p2_take = p2_req && p2_accept_i;
	wire refresh_take = slot_free && refresh_owed && !p0_req && !p1_req;
	wire refresh_edge = refresh && !refresh_old_q;

	// The first-sight ACTIVE needs the live request's row and bank.
	wire [25:0] take_addr = p0_req ? p0_addr : p1_req ? p1_addr : p2_addr;

	assign p0_busy = p0_busy_q || !p0_accept_i;
	assign p1_busy = p1_busy_q || !p1_accept_i;
	assign p2_busy = p2_busy_q || !p2_accept_i;
	assign init_done = init_done_q;
	assign idle = slot_free && !refresh_owed;

	// The output DDIO cell drives an inverted copy of clk_pin to the pad. The
	// input DDIO cell samples DQ on both state-clock edges; DQ_CAPTURE_RISING
	// picks which half feeds the controller.
	wire [15:0] dq_capture_h_w, dq_capture_l_w;
	assign dq_aligned_w = DQ_CAPTURE_RISING ? dq_capture_h_w : dq_capture_l_w;
`ifdef SYNTHESIS
	altddio_out #(
		.extend_oe_disable("OFF"),
		.intended_device_family("Cyclone V"),
		.invert_output("OFF"),
		.lpm_hint("UNUSED"),
		.lpm_type("altddio_out"),
		.oe_reg("UNREGISTERED"),
		.power_up_high("OFF"),
		.width(1)
	) u_sdram_clock (
		.datain_h(1'b0),
		.datain_l(1'b1),
		.outclock(clk_pin),
		.dataout(SDRAM_CLK),
		.aclr(1'b0),
		.aset(1'b0),
		.oe(1'b1),
		.outclocken(1'b1),
		.sclr(1'b0),
		.sset(1'b0)
	);

	altddio_in #(
		.intended_device_family("Cyclone V"),
		.invert_input_clocks("OFF"),
		.lpm_hint("UNUSED"),
		.lpm_type("altddio_in"),
		.power_up_high("OFF"),
		.width(16)
	) u_sdram_dq_capture (
		.datain(dq_in),
		.inclock(clk),
		.inclocken(1'b1),
		.aclr(1'b0),
		.aset(1'b0),
		.sclr(1'b0),
		.sset(1'b0),
		.dataout_h(dq_capture_h_w),
		.dataout_l(dq_capture_l_w)
	);
`else
	// Simulation only, matching the Cyclone V cell: the falling-edge sample is
	// realigned to the next rising edge, as the high half already is.
	reg [15:0] dq_fall_q = 16'd0, dq_capture_l_q = 16'd0, dq_capture_h_q = 16'd0;

	assign SDRAM_CLK = ~clk_pin;
	assign dq_capture_l_w = dq_capture_l_q;
	assign dq_capture_h_w = dq_capture_h_q;

	always @(negedge clk) dq_fall_q <= dq_in;
	always @(posedge clk) begin
		dq_capture_l_q <= dq_fall_q;
		dq_capture_h_q <= dq_in;
	end
`endif

	assign SDRAM_DQML = SDRAM_A[11];
	assign SDRAM_DQMH = SDRAM_A[12];
	assign dq_in = SDRAM_DQ;
	assign SDRAM_DQ = dq_oe_q ? dq_out_q : 16'hZZZZ;

	// Reset is asynchronous on purpose: Cyclone V SDRAM output and
	// output-enable registers cannot pack into the IOE with a synchronous clear.
	always_ff @(posedge clk or posedge reset) begin
		if (reset) begin
			state_q <= ST_POWER_WAIT;
			init_done_q <= 1'b0;
			init_wait_q <= T_POWER_WAIT - 16'd1;
			wait_q <= {WAIT_W{1'b0}};
			wait_zero_q <= 1'b1;
			gap_q <= {WAIT_W{1'b0}};
			gap_zero_q <= 1'b1;
			active_port_q <= 2'd0;
			active_we_q <= 1'b0;
			active_col_q <= 10'd0;
			active_din_q <= 16'd0;
			active_be_q <= 2'b00;
			p0_busy_q <= 1'b1;
			p1_busy_q <= 1'b1;
			p2_busy_q <= 1'b1;
			p0_ready <= 1'b1;
			p1_ready <= 1'b1;
			p2_ready <= 1'b1;
			p0_dout <= 16'd0;
			p1_dout <= 16'd0;
			p2_dout <= 16'd0;
			refresh_debt_q <= 3'd0;
			refresh_old_q <= 1'b0;
			SDRAM_CKE <= 1'b0;
			SDRAM_A <= 13'd0;
			SDRAM_BA <= 2'd0;
			{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_DESELECT;
			dq_out_q <= 16'd0;
			dq_oe_q <= 1'b0;
		end else begin
			{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= init_done_q ? CMD_NOP : CMD_DESELECT;
			dq_oe_q <= 1'b0;
			refresh_old_q <= refresh;

			if (!wait_zero_q) begin
				wait_q <= wait_q - 1'd1;
				wait_zero_q <= (wait_q == {{(WAIT_W-1){1'b0}}, 1'b1});
			end
			if (!gap_zero_q) begin
				gap_q <= gap_q - 1'd1;
				gap_zero_q <= (gap_q == {{(WAIT_W-1){1'b0}}, 1'b1});
			end

			// Refresh debt saturates; one is paid per REFRESH.
			if (refresh_edge && !refresh_take) begin
				if (refresh_debt_q != 3'd7) refresh_debt_q <= refresh_debt_q + 3'd1;
			end else if (!refresh_edge && refresh_take) begin
				refresh_debt_q <= refresh_debt_q - 3'd1;
			end

			unique case (state_q)
				// JEDEC init: CKE low for 200 us, a NOP, PRECHARGE ALL, two
				// refreshes, LOAD MODE.
				ST_POWER_WAIT: begin
					if (init_wait_q != 16'd0) begin
						init_wait_q <= init_wait_q - 16'd1;
					end else begin
						SDRAM_CKE <= 1'b1;
						state_q <= ST_NOP_WAIT;
					end
				end

				ST_NOP_WAIT: begin
					{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_PRECHARGE;
					SDRAM_A <= 13'b0_0100_0000_0000;
					state_q <= ST_INIT_TRP;
					wait_q <= W_RP;
					wait_zero_q <= (W_RP == {WAIT_W{1'b0}});
				end

				ST_INIT_TRP: begin
					if (wait_zero_q) begin
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AUTO_REFRESH;
						state_q <= ST_INIT_RFC1;
						wait_q <= W_RFC;
						wait_zero_q <= (W_RFC == {WAIT_W{1'b0}});
					end
				end

				ST_INIT_RFC1: begin
					if (wait_zero_q) begin
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AUTO_REFRESH;
						state_q <= ST_INIT_RFC2;
						wait_q <= W_RFC;
						wait_zero_q <= (W_RFC == {WAIT_W{1'b0}});
					end
				end

				ST_INIT_RFC2: begin
					if (wait_zero_q) begin
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_LOAD_MODE;
						SDRAM_A <= MODE_REG;
						SDRAM_BA <= 2'd0;
						state_q <= ST_INIT_MRD;
						wait_q <= W_MRD;
						wait_zero_q <= (W_MRD == {WAIT_W{1'b0}});
					end
				end

				ST_INIT_MRD: begin
					if (wait_zero_q) begin
						init_done_q <= 1'b1;
						p0_busy_q <= 1'b0;
						p1_busy_q <= 1'b0;
						p2_busy_q <= 1'b0;
						state_q <= ST_IDLE;
					end
				end

				ST_IDLE: begin
					// The chip reads A and BA only with ACTIVE, so they can
					// follow the candidate request on every idle edge.
					SDRAM_A <= take_addr[23:11];
					SDRAM_BA <= take_addr[25:24];
					if (p0_take || p1_take || p2_take) begin
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_ACTIVE;
						active_col_q <= take_addr[10:1];
						if (p0_take) begin
							active_port_q <= 2'd0;
							active_we_q <= p0_we;
							active_din_q <= p0_din;
							active_be_q <= p0_be;
							p0_busy_q <= 1'b1;
							p0_ready <= 1'b0;
						end else if (p1_take) begin
							active_port_q <= 2'd1;
							active_we_q <= p1_we;
							active_din_q <= p1_din;
							active_be_q <= p1_be;
							p1_busy_q <= 1'b1;
							p1_ready <= 1'b0;
						end else begin
							active_port_q <= 2'd2;
							active_we_q <= p2_we;
							active_din_q <= p2_din;
							active_be_q <= p2_be;
							p2_busy_q <= 1'b1;
							p2_ready <= 1'b0;
						end
						state_q <= ST_RCD;
						wait_q <= W_RCD;
						wait_zero_q <= (W_RCD == {WAIT_W{1'b0}});
						gap_q <= W_GAP;
						gap_zero_q <= (W_GAP == {WAIT_W{1'b0}});
					end else if (refresh_take) begin
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AUTO_REFRESH;
						state_q <= ST_REFRESH_WAIT;
						wait_q <= W_RFC;
						wait_zero_q <= (W_RFC == {WAIT_W{1'b0}});
					end
				end

				// READ or WRITE with A10 set, so the bank closes by itself.
				ST_RCD: begin
					if (wait_zero_q) begin
						SDRAM_A <= {active_we_q ? ~active_be_q : 2'b00, 1'b1, active_col_q};
						if (active_we_q) begin
							{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_WRITE;
							dq_out_q <= active_din_q;
							dq_oe_q <= 1'b1;
							unique case (active_port_q)
								2'd0: begin p0_busy_q <= 1'b0; p0_ready <= 1'b1; end
								2'd1: begin p1_busy_q <= 1'b0; p1_ready <= 1'b1; end
								default: begin p2_busy_q <= 1'b0; p2_ready <= 1'b1; end
							endcase
							state_q <= ST_IDLE;
						end else begin
							{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_READ;
							state_q <= ST_READ_WAIT;
							wait_q <= W_CAS;
							wait_zero_q <= (W_CAS == {WAIT_W{1'b0}});
						end
					end
				end

				ST_READ_WAIT: begin
					if (wait_zero_q) begin
						unique case (active_port_q)
							2'd0: begin p0_dout <= dq_aligned_w; p0_busy_q <= 1'b0; p0_ready <= 1'b1; end
							2'd1: begin p1_dout <= dq_aligned_w; p1_busy_q <= 1'b0; p1_ready <= 1'b1; end
							default: begin p2_dout <= dq_aligned_w; p2_busy_q <= 1'b0; p2_ready <= 1'b1; end
						endcase
						state_q <= ST_IDLE;
					end
				end

				ST_REFRESH_WAIT: begin
					if (wait_zero_q) begin
						state_q <= ST_IDLE;
					end
				end

				default: begin
					state_q <= ST_POWER_WAIT;
					init_done_q <= 1'b0;
					init_wait_q <= T_POWER_WAIT - 16'd1;
					p0_busy_q <= 1'b1;
					p1_busy_q <= 1'b1;
					p2_busy_q <= 1'b1;
				end
			endcase
		end
	end

endmodule

`default_nettype wire
