// Copyright (c) 2026 Jamie Blanks

`timescale 1ns/1ps

// Mirror servo model. The servo, not the VIP, makes FCLK and the eye sync
// pulses, so the display frame is the servo period.
//
//   count 0          LEFT_SYNC      FCLK_HIGH      RIGHT_SYNC     FRAME_CLOCKS-1
//         | FCLK rise | L_SYNC       | FCLK fall    | R_SYNC       |
//
// Pulses are high for one CE period. Sync pulses start once SCANRDY is set;
// with spin-up enabled that takes SPINUP_CLOCKS after reset while FCLK runs.
module vip_scanner
#(
	parameter integer FRAME_CLOCKS  = 399353,
	parameter integer FCLK_HIGH     = 199808,
	parameter integer LEFT_SYNC     = 57966,
	parameter integer RIGHT_SYNC    = 250594,
	parameter integer SPINUP_CLOCKS = 81000000,
	parameter integer START_COUNT   = 0,        // frame count after reset; 0 starts with FCLK rising
	parameter [5:0]   SS_WORD       = 6'd24
)
(
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        ce_i,
	input  wire        spinup_en_i,      // model the mirror spin-up after power-on

	output wire        fclk_o,
	output wire        frame_start_o,    // FCLK rise
	output wire        l_sync_o,
	output wire        r_sync_o,
	output wire        scanrdy_o,
	output wire        scanerr_o,

	input  wire [5:0]  ss_addr_i,
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [63:0] ss_wdata_i,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        ss_we_i,
	output wire [63:0] ss_rdata_o
);
	localparam integer FRAME_LAST_I = FRAME_CLOCKS - 1;
	localparam integer SPIN_LAST_I  = SPINUP_CLOCKS - 1;
	localparam [18:0]  FRAME_LAST   = FRAME_LAST_I[18:0];
	localparam [18:0]  FCLK_FALL    = FCLK_HIGH[18:0];
	localparam [18:0]  L_AT         = LEFT_SYNC[18:0];
	localparam [18:0]  R_AT         = RIGHT_SYNC[18:0];
	localparam [18:0]  COUNT_RESET  = START_COUNT[18:0];
	localparam [26:0]  SPIN_LAST    = SPIN_LAST_I[26:0];

	reg [18:0] count_q;
	reg [26:0] spin_q;
	reg        ready_q;

	assign fclk_o        = (count_q < FCLK_FALL);
	assign frame_start_o = (count_q == 19'd0);
	assign l_sync_o      = ready_q && (count_q == L_AT);
	assign r_sync_o      = ready_q && (count_q == R_AT);
	assign scanrdy_o     = ready_q;
	// The modelled servo keeps lock once ready.
	assign scanerr_o     = 1'b0;
	assign ss_rdata_o    = (ss_addr_i == SS_WORD) ? {17'd0, ready_q, spin_q, count_q} : 64'd0;

	always @(posedge clk_i) begin
		if (reset_i) begin
			count_q <= COUNT_RESET;
			spin_q  <= 27'd0;
			ready_q <= !spinup_en_i;
		end else if (ss_we_i) begin
			if (ss_addr_i == SS_WORD) begin
				count_q <= ss_wdata_i[18:0];
				spin_q  <= ss_wdata_i[45:19];
				ready_q <= ss_wdata_i[46];
			end
		end else if (ce_i) begin
			count_q <= (count_q == FRAME_LAST) ? 19'd0 : count_q + 19'd1;
			if (!ready_q) begin
				if (!spinup_en_i || (spin_q == SPIN_LAST)) begin
					ready_q <= 1'b1;
				end else begin
					spin_q <= spin_q + 27'd1;
				end
			end
		end
	end

endmodule
