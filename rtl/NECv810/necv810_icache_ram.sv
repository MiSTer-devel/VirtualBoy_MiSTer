// Copyright (c) 2026 Jamie Blanks

// Instruction cache data and tag RAMs: 128 lines on one port shared by phase (writes come only
// on CE edges, fetch reads on phi1 edges), and byte access for savestates.

module NECv810_icache_data_ram
(
	input  wire        clk_i,
	input  wire [6:0]  read_addr_i,
	input  wire [6:0]  write_addr_i,
	input  wire        write_en_i,
	input  wire [63:0] write_data_i,
	output wire [63:0] q_o,
	input  wire        savestate_active_i,
	input  wire [9:0]  savestate_addr_i,
	input  wire        savestate_rden_i,
	input  wire        savestate_wren_i,
	input  wire [7:0]  savestate_wdata_i,
	output reg  [7:0]  savestate_rdata_o
);
	wire [63:0] ram_q_w;
	reg [63:0] savestate_merged_data_v;

	assign q_o = ram_q_w;

	// Byte-lane select within the addressed 64-bit cache line.
	always @* begin
		savestate_rdata_o = savestate_rden_i ?
			ram_q_w[{savestate_addr_i[2:0], 3'b000} +: 8] : 8'd0;
		savestate_merged_data_v = ram_q_w;
		savestate_merged_data_v[{savestate_addr_i[2:0], 3'b000} +: 8] =
			savestate_wdata_i;
	end

	cache_ram
	#(
		.ADDR_WIDTH(7),
		.DATA_WIDTH(64)
	)
	u_icache_data
	(
		.clk_i(clk_i),
		.addr_i(savestate_active_i ? savestate_addr_i[9:3] : write_en_i ? write_addr_i : read_addr_i),
		.wren_i(savestate_active_i ? savestate_wren_i : write_en_i),
		.wdata_i(savestate_active_i ? savestate_merged_data_v : write_data_i),
		.q_o(ram_q_w)
	);

endmodule

module NECv810_icache_tag_ram
(
	input  wire        clk_i,
	input  wire [6:0]  read_addr_i,
	input  wire [6:0]  write_addr_i,
	input  wire        write_en_i,
	input  wire [31:0] write_data_i,
	output wire [31:0] q_o,
	input  wire        savestate_active_i,
	input  wire [8:0]  savestate_addr_i,
	input  wire        savestate_rden_i,
	input  wire        savestate_wren_i,
	input  wire [7:0]  savestate_wdata_i,
	output reg  [7:0]  savestate_rdata_o
);
	wire [31:0] ram_q_w;
	reg [31:0] savestate_merged_data_v;

	assign q_o = ram_q_w;

	// Byte-lane select within the addressed 32-bit tag word.
	always @* begin
		savestate_rdata_o = savestate_rden_i ?
			ram_q_w[{savestate_addr_i[1:0], 3'b000} +: 8] : 8'd0;
		savestate_merged_data_v = ram_q_w;
		savestate_merged_data_v[{savestate_addr_i[1:0], 3'b000} +: 8] =
			savestate_wdata_i;
	end

	cache_ram
	#(
		.ADDR_WIDTH(7),
		.DATA_WIDTH(32)
	)
	u_icache_tag
	(
		.clk_i(clk_i),
		.addr_i(savestate_active_i ? savestate_addr_i[8:2] : write_en_i ? write_addr_i : read_addr_i),
		.wren_i(savestate_active_i ? savestate_wren_i : write_en_i),
		.wdata_i(savestate_active_i ? savestate_merged_data_v : write_data_i),
		.q_o(ram_q_w)
	);

endmodule
