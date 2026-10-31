// Aligned instruction words share the registered bus engine with data accesses.
// Words queue from the one decode reads, four in all. A full queue stops the
// fetch until decode runs out of code; a word on the bus always completes.
module necv810_fetch_stream #(
	// Fitted: a redirect stops the next sequential word only within this many
	// clocks of the in-flight word's start. Branch-to-fetch latency is fixed in CPU clocks.
	parameter [2:0] REDIRECT_WINDOW = 3'd2
) (
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        ce_i,
	input  wire        enable_i,
	input  wire [31:1] want_pc_i,
	input  wire        redirect_i,
	input  wire        data_wait_i,
	input  wire        data_ahead_i,
	// Decode takes a RETI or TRAP: one more word past its own, then the stream stops.
	input  wire        reti_take_i,
	input  wire        launch_accept_i,
	input  wire        complete_i,
	input  wire [31:0] read_data_i,
	output wire        busy_o,
	output wire        ready_o,
	output wire [31:0] iw_o,
	output reg         request_o,
	output reg  [31:0] address_o,
	output reg         branch_o
);
	// The word of the instruction offered to decode, and three queued behind it.
	localparam integer DEPTH = 4;
	reg        q_valid_q [0:DEPTH-1];
	reg [29:0] q_addr_q [0:DEPTH-1];
	reg [31:0] q_data_q [0:DEPTH-1];
	reg        active_q;
	reg [29:0] addr_q;
	reg        waste_q;
	reg [2:0]  word_age_q;
	// The word on the bus restarted a stopped stream. The word after it is
	// already committed: a redirect drops it rather than stopping it.
	reg        restart_q;
	// The next sequential word: a word address and whether it is one on, so the
	// increment is worked from registers rather than after the launch mux.
	reg [29:0] next_base_q;
	reg        next_inc_q;
	wire [29:0] next_addr_w = next_base_q + {29'd0, next_inc_q};
	reg        next_set_q;
	reg        next_waste_q;
	reg        next_branch_q;
	reg        target_valid_q;
	reg [29:0] target_addr_q;
	reg        stopped_q;
	// Room for one more word behind the one on the bus, as of the last clock:
	// a word that completes starts the next only if that room was there.
	reg        room_q;
	// A load reading ahead took the bus as a word completed with room behind
	// it: the next word is asked for once the bus is free.
	reg        lost_q;
	// The word on the bus is that lost word. Unless decode came to wait for it,
	// a redirect as it completes goes straight to the target.
	reg        reask_q;
	reg        reask_need_q;
	// A RETI or TRAP is in decode, and a word past its own has started.
	reg        reti_q;
	reg        reti_hold_q;

	wire [29:0] want_word_w = want_pc_i[31:2];
	wire [29:0] want_next_w = want_word_w + 30'd1;
	reg         hit0_v, hit1_v;
	reg [31:0]  word0_v, word1_v;
	reg [DEPTH-1:0] keep_v;
	reg [2:0]   count_v;
	reg [29:0]  behind_v;
	integer     i;
	always @* begin
		hit0_v = 1'b0;
		hit1_v = 1'b0;
		word0_v = 32'd0;
		word1_v = 32'd0;
		count_v = 3'd0;
		keep_v = {DEPTH{1'b0}};
		behind_v = 30'd0;
		for (i = 0; i < DEPTH; i = i + 1) begin
			behind_v = want_word_w - q_addr_q[i];
			keep_v[i] = q_valid_q[i] && !((behind_v != 30'd0) && (behind_v < 30'd8));
			count_v = count_v + {2'd0, keep_v[i]};
			if (q_valid_q[i] && (q_addr_q[i] == want_word_w)) begin
				hit0_v = 1'b1;
				word0_v = q_data_q[i];
			end
			if (q_valid_q[i] && (q_addr_q[i] == want_next_w)) begin
				hit1_v = 1'b1;
				word1_v = q_data_q[i];
			end
		end
	end
	// First slot not kept: where an arriving word goes.
	wire [1:0]  free_w = !keep_v[0] ? 2'd0 : !keep_v[1] ? 2'd1 : !keep_v[2] ? 2'd2 : 2'd3;
	wire [15:0] half0_w = want_pc_i[1] ? word0_v[31:16] : word0_v[15:0];
	wire [15:0] half1_w = want_pc_i[1] ? word1_v[15:0] : word0_v[31:16];
	wire        long_w = (half0_w[15:13] == 3'b101) || (half0_w[15:13] == 3'b110) ||
		(half0_w[15:13] == 3'b111);
	wire        need_next_w = long_w && want_pc_i[1];
	wire [29:0] need_word_w = (hit0_v && need_next_w) ? want_next_w : want_word_w;
	wire        starve_w = !(hit0_v && (!need_next_w || hit1_v)) &&
		!(active_q && !waste_q && (addr_q == need_word_w));
	wire        wait_this_w = active_q && (addr_q == need_word_w) &&
		!(hit0_v && (!need_next_w || hit1_v));
	wire        done_w = active_q && complete_i;
	wire        bus_free_w = !active_q || (done_w && reask_q && !reask_need_q);
	wire        in_time_w = (word_age_q <= REDIRECT_WINDOW) && !restart_q;
	wire        push_w = done_w && !waste_q && !redirect_i;
	wire        start_ok_w = enable_i && (!active_q || done_w) && !data_wait_i;
	// Past a RETI's word only the next one is fetched.
	wire        seq_hold_w = reti_hold_q;
	assign busy_o = active_q;
	assign ready_o = enable_i && !redirect_i && hit0_v && (!need_next_w || hit1_v);
	assign iw_o = {half0_w, long_w ? half1_w : 16'd0};
	wire        launch_w = request_o && launch_accept_i;

	localparam [1:0] REQ_NEXT = 2'd0;
	localparam [1:0] REQ_TARGET = 2'd1;
	localparam [1:0] REQ_NEED = 2'd2;
	localparam [1:0] REQ_REDIRECT = 2'd3;
	reg [1:0] request_kind_v;
	reg       request_waste_v;
	always @* begin
		request_o = 1'b0;
		address_o = {next_addr_w, 2'b00};
		branch_o = next_branch_q;
		request_waste_v = 1'b0;
		request_kind_v = REQ_NEXT;
		if (start_ok_w) begin
			if (redirect_i) begin
				if (bus_free_w) begin
					request_o = 1'b1;
					address_o = {want_word_w, 2'b00};
					branch_o = 1'b1;
					request_kind_v = REQ_REDIRECT;
				end else if (active_q && (done_w || !in_time_w)) begin
					request_o = 1'b1;
					request_waste_v = 1'b1;
					branch_o = 1'b0;
				end
			end else if (next_set_q) begin
				request_o = 1'b1;
				request_waste_v = next_waste_q;
			end else if (!next_set_q && target_valid_q) begin
				request_o = 1'b1;
				address_o = {target_addr_q, 2'b00};
				branch_o = 1'b1;
				request_kind_v = REQ_TARGET;
			end else if (!next_set_q && done_w && !stopped_q && room_q && !seq_hold_w) begin
				request_o = 1'b1;
				branch_o = 1'b0;
			end else if (!next_set_q && !target_valid_q && lost_q && !active_q && !seq_hold_w) begin
				request_o = 1'b1;
				branch_o = 1'b0;
			end else if (!next_set_q && !target_valid_q && starve_w && !seq_hold_w) begin
				request_o = 1'b1;
				address_o = {need_word_w, 2'b00};
				branch_o = 1'b0;
				request_kind_v = REQ_NEED;
			end
		end
	end

	always @(posedge clk_i or posedge reset_i) begin
		if (reset_i) begin
			for (i = 0; i < DEPTH; i = i + 1) begin
				q_valid_q[i] <= 1'b0;
				q_addr_q[i] <= 30'd0;
				q_data_q[i] <= 32'd0;
			end
			active_q <= 1'b0;
			addr_q <= 30'd0;
			waste_q <= 1'b0;
			word_age_q <= 3'd0;
			restart_q <= 1'b0;
			next_base_q <= 30'd0;
			next_inc_q <= 1'b0;
			next_set_q <= 1'b0;
			next_waste_q <= 1'b0;
			next_branch_q <= 1'b0;
			target_valid_q <= 1'b0;
			target_addr_q <= 30'd0;
			stopped_q <= 1'b1;
			room_q <= 1'b0;
			lost_q <= 1'b0;
			reask_q <= 1'b0;
			reask_need_q <= 1'b0;
			reti_q <= 1'b0;
			reti_hold_q <= 1'b0;
		end else if (ce_i) begin
			if (wait_this_w && !redirect_i) reask_need_q <= 1'b1;
			if (!enable_i || redirect_i) begin
				reti_q <= 1'b0;
				reti_hold_q <= 1'b0;
			end else if (reti_take_i) begin
				reti_q <= 1'b1;
				reti_hold_q <= (launch_w ? address_o[31:2] : addr_q) != want_word_w;
			end else if (reti_q && launch_w) begin
				reti_hold_q <= 1'b1;
			end
			room_q <= count_v < 3'd3;
			if (active_q && (word_age_q != 3'd7)) word_age_q <= word_age_q + 3'd1;
			for (i = 0; i < DEPTH; i = i + 1) q_valid_q[i] <= keep_v[i];
			if (done_w) active_q <= 1'b0;
			if (!enable_i) begin
				for (i = 0; i < DEPTH; i = i + 1) q_valid_q[i] <= 1'b0;
				if (active_q) waste_q <= 1'b1;
				next_set_q <= 1'b0;
				target_valid_q <= 1'b0;
				stopped_q <= 1'b1;
				lost_q <= 1'b0;
			end else if (redirect_i) begin
				for (i = 0; i < DEPTH; i = i + 1) q_valid_q[i] <= 1'b0;
				stopped_q <= 1'b0;
				lost_q <= 1'b0;
				if (active_q) waste_q <= 1'b1;
				if (bus_free_w) begin
					target_valid_q <= 1'b1;
					target_addr_q <= want_word_w;
				end else if (done_w || !in_time_w) begin
					next_set_q <= 1'b1;
					next_waste_q <= 1'b1;
					next_branch_q <= 1'b0;
					target_valid_q <= 1'b1;
					target_addr_q <= want_word_w;
				end else begin
					next_base_q <= want_word_w;
					next_inc_q <= 1'b0;
					next_set_q <= 1'b1;
					next_waste_q <= 1'b0;
					next_branch_q <= 1'b1;
					target_valid_q <= 1'b0;
				end
			end else begin
				if (push_w) begin
					q_valid_q[free_w] <= 1'b1;
					q_addr_q[free_w] <= addr_q;
					q_data_q[free_w] <= read_data_i;
				end
				if (done_w && data_ahead_i && room_q && !stopped_q && !next_set_q) lost_q <= 1'b1;
				if ((!active_q || done_w) && data_wait_i) stopped_q <= 1'b1;
				else if (done_w && !request_o) stopped_q <= 1'b1;
			end
			if (request_o && launch_accept_i) begin
				active_q <= 1'b1;
				addr_q <= address_o[31:2];
				waste_q <= request_waste_v;
				word_age_q <= 3'd0;
				restart_q <= (request_kind_v == REQ_NEED);
				next_base_q <= address_o[31:2];
				next_inc_q <= 1'b1;
				next_set_q <= 1'b0;
				next_waste_q <= 1'b0;
				next_branch_q <= 1'b0;
				if ((request_kind_v == REQ_TARGET) || (request_kind_v == REQ_REDIRECT))
					target_valid_q <= 1'b0;
				if ((request_kind_v == REQ_NEED) || lost_q) stopped_q <= 1'b0;
				reask_q <= lost_q && !redirect_i;
				reask_need_q <= 1'b0;
				lost_q <= 1'b0;
			end
		end
	end
endmodule
