// Copyright (c) 2026 Jamie Blanks

// NEC V810 register file: one dual-port RAM shared by phase.
//
//   CE edge:   port A writes the result entering WB, port B r30 beside it
//              (MUL/DIV), or a savestate byte
//   phi1 edge: ports A and B read the two operands of the instruction in ID
//
// A value written as its instruction enters WB is read in that same clock,
// as the chip writes in phi1 and reads in phi2. A port with no write to come
// reads at both edges, so its data holds; debug reads take port A while
// halted, savestates port B while the core is paused.
module necv810_regfile
(
	input  wire        clk_i,
	input  wire        ce_i,

	input  wire [4:0]  read_addr_a_i,
	input  wire [4:0]  read_addr_b_i,
	output wire [31:0] read_data_a_o,
	output wire [31:0] read_data_b_o,

	input  wire        write_sel_a_i,    // a write may land at this CE edge: port A takes its address
	input  wire        write_a_i,
	input  wire [4:0]  write_addr_a_i,
	input  wire [31:0] write_data_a_i,
	input  wire        write_sel_b_i,    // and r30, on port B
	input  wire        write_b_i,
	input  wire [31:0] write_data_b_i,

	input  wire        savestate_active_i,
	input  wire [6:0]  savestate_addr_i,
	input  wire        savestate_rden_i,
	input  wire        savestate_wren_i,
	input  wire [7:0]  savestate_wdata_i,
	output reg  [7:0]  savestate_rdata_o
);
	wire [31:0] q_a_w;
	wire [31:0] q_b_w;
	reg  [4:0]  rd_a_q;
	reg  [4:0]  rd_b_q;

	wire [4:0]  ss_word_w = savestate_addr_i[6:2];
	wire        wsel_a_w = ce_i && write_sel_a_i;
	wire        wsel_b_w = ce_i && write_sel_b_i;
	wire [4:0]  addr_a_w = wsel_a_w ? write_addr_a_i : read_addr_a_i;
	wire [4:0]  addr_b_w = savestate_active_i ? ss_word_w : wsel_b_w ? 5'd30 : read_addr_b_i;

	// A savestate byte goes into the word port B read the clock before.
	reg  [31:0] ss_merged_v;
	always @* begin
		ss_merged_v = q_b_w;
		ss_merged_v[{savestate_addr_i[1:0], 3'b000} +: 8] = savestate_wdata_i;
		savestate_rdata_o = savestate_rden_i ? q_b_w[{savestate_addr_i[1:0], 3'b000} +: 8] : 8'd0;
	end

	// r0 reads as zero whatever the RAM holds.
	always @(posedge clk_i) begin
		if (!wsel_a_w) rd_a_q <= read_addr_a_i;
		if (!wsel_b_w) rd_b_q <= read_addr_b_i;
	end
	assign read_data_a_o = (rd_a_q == 5'd0) ? 32'd0 : q_a_w;
	assign read_data_b_o = (rd_b_q == 5'd0) ? 32'd0 : q_b_w;

	// NEC leaves r1-r31 undefined after reset, so the RAM does not reset.
	cache_ram_dp
	#(
		.ADDR_WIDTH(5),
		.DATA_WIDTH(32)
	)
	u_ram
	(
		.clk_i(clk_i),
		.addr_a_i(addr_a_w),
		.wren_a_i(wsel_a_w && write_a_i && (write_addr_a_i != 5'd0)),
		.wdata_a_i(write_data_a_i),
		.q_a_o(q_a_w),
		.addr_b_i(addr_b_w),
		.wren_b_i(savestate_active_i ? savestate_wren_i : (wsel_b_w && write_b_i)),
		.wdata_b_i(savestate_active_i ? ss_merged_v : write_data_b_i),
		.q_b_o(q_b_w)
	);

// synthesis translate_off
`ifndef SYNTHESIS
	task sim_set_gpr_task;
		input [4:0]  write_addr;
		input [31:0] write_data;
		begin
			if (write_addr != 5'd0) u_ram.mem_q[write_addr] = write_data;
		end
	endtask

	function [31:0] sim_read_gpr_fn;
		input [4:0] read_addr;
		begin
			sim_read_gpr_fn = (read_addr == 5'd0) ? 32'd0 : u_ram.mem_q[read_addr];
		end
	endfunction
`endif
// synthesis translate_on

endmodule
