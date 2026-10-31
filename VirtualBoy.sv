//============================================================================
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

module emu
(
	`include "sys/emu_ports.vh"
);

	localparam integer SAVE_BLOCKS = 32768;   // 16 MiB / 512 B
	localparam [23:0] CART_SRAM_STANDARD_MASK   = 24'h007fff;
	localparam [23:0] CART_SRAM_LARGE_MASK      = 24'hffffff;
	localparam [31:0] CART_SRAM_STANDARD_BLOCKS = 32'd64;
	localparam [31:0] CART_SRAM_LARGE_BLOCKS    = 32'd32768;
	// The MiSTer header stores the payload length in 32-bit words. Each slot is
	// 32 MiB; four slots occupy 0x32000000-0x39ffffff, above the 16 MiB SRAM
	// shadow at 0x30000000-0x30ffffff.
	// Scalar and non-cartridge bulk state occupy 82,634 DWORDs. Cartridge SRAM is
	// appended at its active mirrored size rather than at the 16 MiB ceiling.
	localparam [31:0]  VIRTUALBOY_SAVESTATE_FIXED_DWORDS = 32'd82634;
	localparam integer VIRTUALBOY_SAVESTATE_SLOT_SHIFT   = 23;
	localparam integer VIRTUALBOY_SAVESTATE_BASE         = 32'h00800000;
	localparam integer VIRTUALBOY_SS_INTERNAL_WORDS      = 113;

	// Round a mounted image size up to a power-of-two address mask, saturating
	// at the full 16 MiB window for zero or oversized images.
	function automatic [23:0] cart_sram_size_mask_fn;
		input [63:0] size_bytes_v;
		reg [23:0] mask_v;
		begin
			if ((size_bytes_v == 64'd0) || |size_bytes_v[63:24]) begin
				cart_sram_size_mask_fn = 24'hffffff;
			end else begin
				mask_v = size_bytes_v[23:0] - 24'd1;
				mask_v = mask_v | (mask_v >> 1);
				mask_v = mask_v | (mask_v >> 2);
				mask_v = mask_v | (mask_v >> 4);
				mask_v = mask_v | (mask_v >> 8);
				mask_v = mask_v | (mask_v >> 16);
				cart_sram_size_mask_fn = mask_v;
			end
		end
	endfunction

	///////////////////////   SAVESTATE PLUMBING   ///////////////////

	wire        savestate_menu_enabled;
	wire [1:0]  savestate_slot;
	wire [7:0]  savestate_info;
	wire        savestate_info_req;
	wire        savestate_status_update;
	wire        savestate_ui_save;
	wire        savestate_ui_load;
	wire        savestate_savestate;
	wire        savestate_loadstate;
	wire        savestate_busy;
	wire        saving_savestate;
	wire        loading_savestate;
	wire        sleep_savestate;
	wire        savestate_reset_ss;
	wire        savestate_restore_begin;
	wire        savestate_pause_ready;
	wire        savestate_core_pause_ready;
	wire        savestate_pause_active_q;
	wire        core_execution_enable;
	wire        video_source_enable;
	wire        video_frame_wrap;
	wire        public_video_vsync;
	integer     savestate_address;
	wire [63:0] savestate_ddr_dout;
	wire [63:0] savestate_ddr_din;
	wire [27:2] savestate_ddr_addr;
	wire [7:0]  savestate_ddr_be;
	wire        savestate_ddr_rnw;
	wire        savestate_ddr_req;
	wire        savestate_ddr_ack;
	wire [63:0] savestate_state_wdata;
	wire [9:0]  savestate_state_addr;
	wire        savestate_state_wren;
	wire [63:0] savestate_state_rdata;
	wire [24:0] savestate_mem_addr;
	wire        savestate_mem_rden;
	wire        savestate_mem_wren;
	wire [7:0]  savestate_mem_wdata;
	wire [7:0]  savestate_ram_read_data;
	wire        savestate_ram_ready;
	wire [7:0]  savestate_core_mem_rdata;
	wire [2:0]  savestate_mem_type;
	wire        savestate_ram_active = saving_savestate || loading_savestate;
	wire        savestate_cart_sram_active = savestate_ram_active &&
		(savestate_mem_type == 3'd3);
	wire        savestate_core_mem_active = savestate_ram_active &&
		(savestate_mem_type != 3'd3);

	// The cartridge RAM initializer and the savestate walker share the core's
	// single byte-wide savestate memory port; the savestate side has priority.
	wire        cart_ram_clear_busy;
	wire [2:0]  cart_ram_clear_mem_type;
	wire [24:0] cart_ram_clear_mem_addr;
	wire        cart_ram_clear_mem_wren;
	wire [7:0]  cart_ram_clear_mem_wdata;
	wire        cart_ram_clear_mem_ready;
	wire        cart_ram_clear_core_active = cart_ram_clear_busy;
	wire        core_mem_active = savestate_core_mem_active || cart_ram_clear_core_active;
	wire [2:0]  core_mem_type = savestate_core_mem_active ?
		savestate_mem_type : cart_ram_clear_mem_type;
	wire [24:0] core_mem_addr = savestate_core_mem_active ?
		savestate_mem_addr : cart_ram_clear_mem_addr;
	wire        core_mem_rden = savestate_core_mem_active && savestate_mem_rden;
	wire        core_mem_wren = savestate_core_mem_active ?
		savestate_mem_wren : (cart_ram_clear_mem_wren && cart_ram_clear_core_active);
	wire [7:0]  core_mem_wdata = savestate_core_mem_active ?
		savestate_mem_wdata : cart_ram_clear_mem_wdata;

	///////// Default values for ports not used in this core /////////

	assign ADC_BUS  = 'Z;
	assign USER_OUT = '1;
	assign {UART_RTS, UART_TXD, UART_DTR} = 0;
	assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
	`ifdef MISTER_DUAL_SDRAM
	assign SDRAM2_CLK  = 1'bZ;
	assign SDRAM2_A    = 'Z;
	assign SDRAM2_BA   = 'Z;
	assign SDRAM2_DQ   = 'Z;
	assign SDRAM2_nCS  = 1'bZ;
	assign SDRAM2_nCAS = 1'bZ;
	assign SDRAM2_nRAS = 1'bZ;
	assign SDRAM2_nWE  = 1'bZ;
	`endif

	assign VGA_F1      = 0;
	assign VGA_SCALER  = 0;
	assign VGA_DISABLE = 0;
	// The public raster remains live and emits black during reset/load/pause.
	// Freezing the framework output would hide that deliberate fallback path.
	assign HDMI_FREEZE    = 1'b0;
	assign HDMI_BLACKOUT  = 0;
	assign HDMI_BOB_DEINT = 0;

	assign LED_DISK   = 0;
	assign LED_POWER  = 0;
	assign BUTTONS[1] = 1'b0;

	assign AUDIO_S = 1'b1;

	`ifdef MISTER_FB
	assign FB_EN          = 1'b0;
	assign FB_FORMAT      = 5'd0;
	assign FB_WIDTH       = 12'd0;
	assign FB_HEIGHT      = 12'd0;
	assign FB_BASE        = 32'd0;
	assign FB_STRIDE      = 14'd0;
	assign FB_FORCE_BLANK = 1'b0;
	`ifdef MISTER_FB_PALETTE
	assign FB_PAL_CLK  = 1'b0;
	assign FB_PAL_ADDR = 8'd0;
	assign FB_PAL_DOUT = 24'd0;
	assign FB_PAL_WR   = 1'b0;
	`endif
	`endif

	///////////////////////   CLOCKS   ///////////////////////////////

	// clk_sys is a 40 MHz PLL output, twice the 20 MHz Virtual Boy clock.
	wire clk_sys;
	// clk_ram is the 80 MHz PLL output used by the SDRAM controller, 2x clk_sys
	// from the same PLL: every clk_sys edge lands on a clk_ram edge, so the
	// cartridge path crosses with plain registers. The controller creates its
	// physical inverted SDRAM pin clock internally.
	wire clk_ram;
	// clk_ram 1.875 ns early: it forwards SDRAM_CLK, so read data reaches the
	// capture edge with margin in every corner.
	wire clk_ram_pin;
	wire pll_locked;

	pll pll (
		.refclk   (CLK_50M),
		.rst      (1'b0),
		.outclk_0 (clk_sys), // 40 MHz
		.outclk_1 (clk_ram), // 80 MHz (2x clk_sys)
		.outclk_2 (clk_ram_pin), // 80 MHz, -1.875 ns
		.locked   (pll_locked)
	);

	reg ce_div = 1'b0;
	always @(posedge clk_sys) begin
		if (!pll_locked)
			ce_div <= 1'b0;
		else
			ce_div <= ~ce_div;
	end

	wire ce_20m = ~ce_div;

	assign CLK_VIDEO = clk_sys;

	///////////////////////   HPS/OSD   //////////////////////////////

	wire [127:0] status;
	wire  [1:0]  buttons;
	wire         forced_scandoubler;
	wire [31:0]  joystick_0;
	wire [15:0]  joystick_0_rumble;
	wire [10:0]  ps2_key;
	wire         ioctl_download;
	wire         ioctl_wr;
	wire [26:0]  ioctl_addr;
	wire [15:0]  ioctl_dout;
	wire [15:0]  ioctl_index;
	wire         ioctl_wait;
	wire         rom_ioctl_wait;
	wire         tas_ioctl_wait;

	wire  [0:0]  img_mounted;
	wire         img_readonly;
	wire [63:0]  img_size;

	wire [31:0]  sd_lba;
	wire         sd_rd;
	wire         sd_wr;
	wire         sd_ack;
	wire [12:0]  sd_buff_addr;
	wire [15:0]  sd_buff_dout;
	wire [15:0]  sd_buff_din;
	wire         sd_buff_wr;

	// Keep file routing visibly constrained to the documented six slot bits.
	wire [5:0] file_index = ioctl_index[5:0];

	localparam [2:0] COLOR_MODE_PALETTE = 3'd2;
	localparam [2:0] COLOR_MODE_CUSTOM  = 3'd6;

	wire [2:0] color_mode      = status[5:3];
	wire [2:0] left_eye_color  = status[14:12];
	wire [2:0] right_eye_color = status[19:17];
	// Active-low so the all-zero power-on/menu state selects true black.
	wire       palette_true_black          = !status[41];
	wire       serial_rumble_enable        = !status[28];
	wire       video_60hz_compat_requested = status[29];
	wire       crop_216_enabled            = status[31];
	wire [3:0] crop_offset_choice          = status[35:32];
	wire [1:0] video_scale                 = status[37:36];
	wire       large_sram_enabled          = status[40];
	wire       presentation_hlg_enabled    = status[47];
	reg        crop_216_available;

	wire [15:0] status_menumask = {
		8'd0,
		savestate_menu_enabled,
		4'd0,
		crop_216_available,
		(color_mode != COLOR_MODE_PALETTE),
		(color_mode != COLOR_MODE_CUSTOM)
	};

	`ifndef TOPLEVEL_SIM
	wire [31:0] sd_lba_bus      [0:0];
	wire  [5:0] sd_blk_cnt_bus  [0:0];
	wire [15:0] sd_buff_din_bus [0:0];

	assign sd_lba_bus[0]      = sd_lba;
	assign sd_blk_cnt_bus[0]  = 6'd0;
	assign sd_buff_din_bus[0] = sd_buff_din;
	`endif

	`include "build_id.v"
	localparam CONF_STR = {
		"VirtualBoy;SS32000000:2000000;",
		"FS1,VB ,Load ROM;",
		"-;",
		"C,Cheats;",
		"-;",
		"O[46:45],Savestate Slot,1,2,3,4;",
		"d7rA,Save State(Alt+F1-F4);",
		"d7rB,Restore State(F1-F4);",
		"-;",
		"P1,Audio & Video;",
		"P1-;",
		"P1O[5:3],Color Mode,Natural,Multicolor,Palette,Anaglyph,ColorCode,TriOviz,Custom,Pure Red;",
		"H1P1FC3,GBP,Load Palette;",
		"H1P1O[41],Use True Black,On,Off;",
		"H0P1O[14:12],L Color,Red,Magenta,Blue,Cyan,Green,Yellow,White;",
		"H0P1O[19:17],R Color,Red,Magenta,Blue,Cyan,Green,Yellow,White;",
		"P1O[27:25],Stereo Scale,100%,75%,62.5%,50%,37.5%,25%,12.5%,0%;",
		"P1O[8:7],Eye Drawn,Left,Right,Both,Side by Side;",
		"P1-;",
		"P1O[29],Video Timing,Native 50 Hz,Buffered 60 Hz;",
		"P1O[2:1],Aspect Ratio,Original,Full Screen,[ARC1],[ARC2];",
		"d2P1O[31],Vertical Crop,Disabled,216p(5x);",
		"d2P1O[35:32],Crop Offset,0,2,4,6,8,10,-12,-10,-8,-6,-4,-2;",
		"P1O[37:36],Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
		"P1O[39:38],Audio Mix,None,25%,50%,100%;",
		"P1O[47],Brightness Map,SDR,HDR HLG;",
		"P2,Advanced;",
		"P2-;",
		"P2F4,VBT,Load Brightness Table;",
		"P2-;",
		"P2O[40],Enable Large SRAM,Off,On;",
		"P2O[28],Serial Port,Rumble,None;",
		"P2O[30],Mirror Spin-up Delay,Off,On;",
		"P2O[44],Savestates to SD Card,On,Off;",
		"P2-;",
		"P2R9,Load Backup RAM;",
		"P2RA,Save Backup RAM;",
		"P2OB,Autosave,On,Off;",
		"P2F5,TAS,Load TAS;",
		"-;",
		"R0,Reset;",
		"J1,A,B,L,R,Select,Start,R.Right,R.Left,R.Down,R.Up,Save State;",
		"v,15;",
		"V,v",`BUILD_DATE
	};

	hps_io #(
		.CONF_STR(CONF_STR),
		.WIDE(1),
		.VDNUM(1)
	) hps_io (
		.clk_sys            (clk_sys),
		.HPS_BUS            (HPS_BUS),
		.EXT_BUS            (),
		.gamma_bus          (),

		.buttons            (buttons),
		.status             (status),
		.status_menumask    (status_menumask),
		.status_in          ({status[127:47], savestate_slot, status[44:0]}),
		.status_set         (savestate_status_update),

		.forced_scandoubler (forced_scandoubler),
		.direct_video       (),
		.video_rotated      (1'b0),
		.new_vmode          (1'b0),

		.joystick_0         (joystick_0),
		.joystick_0_rumble  (joystick_0_rumble),
		.joystick_1_rumble  (16'd0),
		.joystick_2_rumble  (16'd0),
		.joystick_3_rumble  (16'd0),
		.joystick_4_rumble  (16'd0),
		.joystick_5_rumble  (16'd0),
		.ps2_key            (ps2_key),

		.ioctl_download     (ioctl_download),
		.ioctl_index        (ioctl_index),
		.ioctl_wr           (ioctl_wr),
		.ioctl_addr         (ioctl_addr),
		.ioctl_dout         (ioctl_dout),
		.ioctl_wait         (ioctl_wait),

		.img_mounted        (img_mounted),
		.img_readonly       (img_readonly),
		.img_size           (img_size),

	`ifdef TOPLEVEL_SIM
		.sd_lba             (sd_lba),
		.sd_blk_cnt         (6'd0),
	`else
		.sd_lba             (sd_lba_bus),
		.sd_blk_cnt         (sd_blk_cnt_bus),
	`endif
		.sd_rd              (sd_rd),
		.sd_wr              (sd_wr),
		.sd_ack             (sd_ack),
		.sd_buff_addr       (sd_buff_addr),
		.sd_buff_dout       (sd_buff_dout),
	`ifdef TOPLEVEL_SIM
		.sd_buff_din        (sd_buff_din),
	`else
		.sd_buff_din        (sd_buff_din_bus),
	`endif
		.sd_buff_wr         (sd_buff_wr),
		.info_req           (savestate_info_req),
		.info               (savestate_info)
	);

	///////////////////////   CORE CONTROL   /////////////////////////

	wire rom_download              = ioctl_download && (file_index == 6'd1 || file_index == 6'd0);
	wire palette_download          = ioctl_download && (file_index == 6'd3);
	wire brightness_table_download = ioctl_download && (file_index == 6'd4);
	wire tas_download              = ioctl_download && (file_index == 6'd5);
	wire cheat_download            = ioctl_download && (ioctl_index == 16'd255);

	wire [95:0] loaded_palette_rgb;
	wire        brightness_table_write;
	wire [9:0]  brightness_table_addr;
	wire [15:0] brightness_table_data;
	wire        brightness_table_legacy_sdr;
	wire        cheat_clear;
	wire [128:0] cheat_code;

	wire [15:0] tas_pad_word;
	wire        tas_pad_override;
	wire        tas_hold_reset;
	wire        tas_load_start;
	wire        tas_fault;
	wire        tas_pad_latch;
	wire [27:1] tas_ddr_addr;
	wire [63:0] tas_ddr_dout;
	wire [63:0] tas_ddr_din;
	wire        tas_ddr_req;
	wire        tas_ddr_rnw;
	wire [7:0]  tas_ddr_be;
	wire        tas_ddr_ready;

	assign ioctl_wait = rom_ioctl_wait | tas_ioctl_wait;

	vb_cheat_loader u_cheat_loader (
		.clk_sys_i        (clk_sys),
		.cheat_download_i (cheat_download),
		.rom_download_i   (rom_download),
		.ioctl_wr_i       (ioctl_wr),
		.ioctl_addr_i     (ioctl_addr[3:0]),
		.ioctl_dout_i     (ioctl_dout),
		.clear_o          (cheat_clear),
		.code_o           (cheat_code)
	);

	vb_gbp_palette_loader u_gbp_palette_loader (
		.clk_i         (clk_sys),
		.reset_i       (~pll_locked),
		.download_i    (palette_download),
		.write_i       (ioctl_wr),
		.addr_i        (ioctl_addr),
		.data_i        (ioctl_dout),
		.palette_rgb_o (loaded_palette_rgb)
	);

	vb_vbt_brightness_loader u_vbt_brightness_loader (
		.clk_i              (clk_sys),
		.reset_i            (~pll_locked),
		.download_i         (brightness_table_download),
		.write_i            (ioctl_wr),
		.addr_i             (ioctl_addr),
		.data_i             (ioctl_dout),
		.table_write_o      (brightness_table_write),
		.table_addr_o       (brightness_table_addr),
		.table_data_o       (brightness_table_data),
		.table_legacy_sdr_o (brightness_table_legacy_sdr)
	);

	reg old_rom_download = 1'b0;

	wire reset_button          = RESET | buttons[1] | status[0];
	wire save_load_req         = status[9];
	wire save_save_req         = status[10];
	wire save_autosave_disable = status[11];

	wire new_rom_pulse = rom_download && !old_rom_download;
	wire rom_loaded;
	wire startup_osd_button;

	wire save_slot_busy;
	wire save_slot_load_active;
	wire save_pending;
	wire save_busy;
	wire save_pause_busy;
	wire save_transfer_allowed;

	reg         cart_sram_init_busy = 1'b0;
	reg  [22:0] cart_sram_init_addr_q = 23'd0;
	reg  [23:0] cart_sram_image_addr_mask_q = CART_SRAM_LARGE_MASK;
	reg         cart_sram_mask_pending_q = 1'b0;
	reg         old_img_mounted_q = 1'b0;

	wire [23:0] cart_sram_addr_mask_w = cart_sram_image_addr_mask_q &
		(large_sram_enabled ? CART_SRAM_LARGE_MASK : CART_SRAM_STANDARD_MASK);
	// Exact 32 KiB saves use the HyperFlash32 programmer layout: one packed byte
	// per x8 SRAM cell. Other sizes retain the established MiSTer save-container
	// layout, but CPU accesses still observe only the physical D7..D0 SRAM lane.
	wire        cart_sram_packed_x8_w = cart_sram_addr_mask_w == 24'h007fff;
	wire [23:0] cart_sram_cpu_addr_mask_w = cart_sram_packed_x8_w ?
		{cart_sram_addr_mask_w[22:0], 1'b1} : cart_sram_addr_mask_w;
	wire [31:0] cart_sram_block_limit_w = large_sram_enabled ?
		CART_SRAM_LARGE_BLOCKS : CART_SRAM_STANDARD_BLOCKS;
	wire [24:0] savestate_cart_sram_size_raw_w =
		{1'b0, cart_sram_addr_mask_w} + 25'd1;
	wire [24:0] savestate_cart_sram_size_w =
		(savestate_cart_sram_size_raw_w < 25'd8) ?
			25'd8 : savestate_cart_sram_size_raw_w;
	wire [31:0] savestate_payload_dwords_w =
		VIRTUALBOY_SAVESTATE_FIXED_DWORDS +
		{9'd0, savestate_cart_sram_size_w[24:2]};

	// Latch the mounted backup image size once the mount pulse completes.
	always @(posedge clk_sys) begin
		if (!pll_locked) begin
			cart_sram_image_addr_mask_q <= CART_SRAM_LARGE_MASK;
			cart_sram_mask_pending_q <= 1'b0;
			old_img_mounted_q <= 1'b0;
		end else begin
			old_img_mounted_q <= img_mounted[0];
			if (new_rom_pulse) begin
				cart_sram_image_addr_mask_q <= CART_SRAM_LARGE_MASK;
				cart_sram_mask_pending_q <= 1'b0;
			end
			if (img_mounted[0] && !old_img_mounted_q) begin
				cart_sram_mask_pending_q <= 1'b1;
			end
			if (cart_sram_mask_pending_q && !img_mounted[0] &&
				(img_size != 64'd0)) begin
				cart_sram_image_addr_mask_q <= cart_sram_size_mask_fn(img_size);
				cart_sram_mask_pending_q <= 1'b0;
			end
		end
	end

	// Match the established MiSTer core convention: BUTTONS[0] simulates the
	// physical OSD button. If startup remains idle without a ROM for one second,
	// hold it long enough for the HPS side to observe one deliberate press.
	vb_startup_osd_button u_startup_osd_button (
		.clk_i          (clk_sys),
		.ce_i           (ce_20m),
		.reset_i        (~pll_locked | RESET),
		.rom_download_i (rom_download),
		.rom_loaded_i   (rom_loaded),
		.osd_button_o   (startup_osd_button)
	);

	assign BUTTONS[0] = startup_osd_button;

	// MiSTer sends an index-255 transfer whenever cheats are enabled or disabled.
	// Reload the table live; tying that transfer to reset would reboot the game.
	reg  cart_cpu_run_q = 1'b0;
	wire savestate_control_reset = reset_button | ~pll_locked | rom_download |
		~rom_loaded | cart_ram_clear_busy | tas_hold_reset;
	wire logic_reset = reset_button | ~pll_locked | new_rom_pulse |
		savestate_reset_ss | cart_ram_clear_busy | tas_hold_reset;
	wire core_reset = savestate_control_reset |
		savestate_reset_ss;
	wire core_ce = ce_20m & core_execution_enable &
		~savestate_pause_active_q & cart_cpu_run_q;

	// While the CPU is stopped the cart SDRAM refreshes on a timer and serves
	// HPS traffic. The CPU restarts only once that has finished, so its first
	// cartridge access always finds the SDRAM idle.
	wire cart_sdram_quiet_w;
	wire cart_cpu_stopped_w = core_reset | ~core_execution_enable |
		savestate_pause_active_q;
	always @(posedge clk_sys) begin
		if (!pll_locked) cart_cpu_run_q <= 1'b0;
		else cart_cpu_run_q <= !cart_cpu_stopped_w &&
			(cart_cpu_run_q || cart_sdram_quiet_w);
	end

	assign LED_USER = rom_download | palette_download |
		brightness_table_download | tas_download | cheat_download |
		save_busy | save_pending |
		savestate_busy | cart_ram_clear_busy | tas_hold_reset | tas_fault;
	assign save_busy = save_slot_busy | cart_sram_init_busy |
		cart_ram_clear_busy;
	assign savestate_menu_enabled = rom_loaded && !rom_download && !save_busy &&
		!tas_pad_override;

	// Zero the volatile memories only for a new cartridge. Savestate restore has
	// explicit cancellation and port priority, so its reset-before-restore pulse
	// cannot start or overlap this initializer.
	vb_cart_ram_clear u_cart_ram_clear (
		.clk_i       (clk_sys),
		.reset_i     (~pll_locked),
		.start_i     (new_rom_pulse | tas_load_start),
		.cancel_i    (loading_savestate | savestate_restore_begin),
		.mem_ready_i (cart_ram_clear_mem_ready),
		.busy_o      (cart_ram_clear_busy),
		.mem_type_o  (cart_ram_clear_mem_type),
		.mem_addr_o  (cart_ram_clear_mem_addr),
		.mem_wren_o  (cart_ram_clear_mem_wren),
		.mem_wdata_o (cart_ram_clear_mem_wdata)
	);

	`ifndef SYNTHESIS
	always @(posedge clk_sys) begin
		if (cart_ram_clear_busy && savestate_core_mem_active) begin
			$error("cartridge RAM clear overlapped savestate memory access");
		end
	end
	`endif

	///////////////////////   EXTERNAL MEMORY   //////////////////////

	wire [23:1] ext_a;
	wire [15:0] ext_dout;
	wire [3:0]  ext_be;
	wire        ext_da;
	wire        ext_mrq;
	wire        ext_rw;
	wire        cart_ram_cs;
	wire        cart_rom_cs;
	wire        ext_cycle_tag;
	wire [23:0] ext_addr = {ext_a, 1'b0};
	wire [15:0] ext_din;
	wire        cpu_bus_mrq;
	wire        cpu_bus_ready;
	wire [2:0]  cpu_bus_region;
	wire        cpu_bus_rw;

	// Cartridge SRAM is canonical in external SDRAM. A byte-enabled DDR3 mirror
	// is maintained only so HPS backup reads never compete with fixed-edge CPU
	// traffic. One 64-bit DDR beat contains four consecutive SDRAM halfwords.
	function automatic [63:0] shadow_write_data_fn;
		input [1:0] halfword_index_v;
		input [15:0] data_v;
		begin
			case (halfword_index_v)
				2'd0: shadow_write_data_fn = {48'd0, data_v};
				2'd1: shadow_write_data_fn = {32'd0, data_v, 16'd0};
				2'd2: shadow_write_data_fn = {16'd0, data_v, 32'd0};
				default: shadow_write_data_fn = {data_v, 48'd0};
			endcase
		end
	endfunction

	function automatic [7:0] shadow_write_be_fn;
		input [1:0] halfword_index_v;
		input [1:0] byte_enable_v;
		begin
			case (halfword_index_v)
				2'd0: shadow_write_be_fn = {6'd0, byte_enable_v};
				2'd1: shadow_write_be_fn = {4'd0, byte_enable_v, 2'd0};
				2'd2: shadow_write_be_fn = {2'd0, byte_enable_v, 4'd0};
				default: shadow_write_be_fn = {byte_enable_v, 6'd0};
			endcase
		end
	endfunction

	// Background P1 traffic uses a one-entry bundled-data toggle contract. CPU
	// traffic is attached directly through the restored cartridge wrappers.
	localparam [2:0] P1_SOURCE_NONE      = 3'd0;
	localparam [2:0] P1_SOURCE_INIT      = 3'd1;
	localparam [2:0] P1_SOURCE_SAVE_LOAD = 3'd2;
	localparam [2:0] P1_SOURCE_SAVESTATE = 3'd3;

	reg        p1_sys_busy_q = 1'b0;
	reg [2:0]  p1_source_q = P1_SOURCE_NONE;
	reg        p1_request_write_sys_q = 1'b0;
	reg [1:0]  p1_request_be_sys_q = 2'b00;
	reg [25:0] p1_request_addr_sys_q = 26'd0;
	reg [15:0] p1_request_data_sys_q = 16'd0;
	reg        p1_sdram_done_q = 1'b0;
	reg        p1_shadow_done_q = 1'b0;
	reg        p1_cpu_dirty_pulse_q = 1'b0;
	reg        savestate_sram_ready_q = 1'b0;
	reg [7:0]  savestate_sram_data_q = 8'hff;
	reg        cpu_shadow_dirty_q = 1'b0;

	wire        rom_core_req_w = ext_mrq && cart_rom_cs && ext_rw;
	wire [15:0] rom_cpu_data_w;
	wire        rom_cpu_valid_w;
	wire [15:0] sram_cpu_data_w;
	wire        sram_cpu_valid_w;
	wire        sram_cpu_sdram_done_w;
	wire        sram_cpu_access_done_w;
	wire        p1_background_ready_w;
	wire        p1_background_done_w;
	wire [15:0] p1_response_data_w;
	wire        sdram_foreground_idle_w;
	wire        cart_refresh_window_w;

	wire p1_shadow_ack_pulse_w;
	wire p1_sdram_complete_w = p1_sdram_done_q || p1_background_done_w;
	wire p1_shadow_complete_w = !p1_request_write_sys_q ||
		p1_shadow_done_q || p1_shadow_ack_pulse_w;
	wire p1_transaction_complete_w = p1_sys_busy_q &&
		p1_sdram_complete_w && p1_shadow_complete_w;

	// Staging FSM between the HPS 512-byte block buffer and the P1/DDR paths.
	localparam [3:0] SAVE_STAGE_IDLE         = 4'd0;
	localparam [3:0] SAVE_STAGE_LOAD_RECEIVE = 4'd1;
	localparam [3:0] SAVE_STAGE_LOAD_FETCH   = 4'd2;
	localparam [3:0] SAVE_STAGE_LOAD_DRAIN   = 4'd3;
	localparam [3:0] SAVE_STAGE_SAVE_DDR     = 4'd4;
	localparam [3:0] SAVE_STAGE_SAVE_UNPACK  = 4'd5;
	localparam [3:0] SAVE_STAGE_SAVE_HPS     = 4'd6;

	reg [3:0]  save_stage_state_q = SAVE_STAGE_IDLE;
	reg        save_stage_ack_seen_q = 1'b0;
	reg [7:0]  save_stage_word_q = 8'd0;
	reg [5:0]  save_stage_beat_q = 6'd0;
	reg [1:0]  save_stage_unpack_q = 2'd0;
	reg [63:0] save_stage_ddr_data_q = 64'd0;

	// Keep the save-side LBA named: its low 15 bits span all 32,768 blocks.
	wire [31:0] save_sd_lba;
	wire [15:0] save_stage_buffer_q_a;
	wire [15:0] save_stage_buffer_q_b;
	wire [7:0]  save_stage_buffer_addr_b =
		(save_stage_state_q == SAVE_STAGE_SAVE_UNPACK) ?
			{save_stage_beat_q, save_stage_unpack_q} : save_stage_word_q;
	wire save_stage_buffer_wren_b =
		save_stage_state_q == SAVE_STAGE_SAVE_UNPACK;
	reg [15:0] save_stage_buffer_wdata_b;

	wire save_stage_p1_req_w = save_stage_state_q == SAVE_STAGE_LOAD_DRAIN;
	wire [23:0] save_stage_p1_addr_w =
		{save_sd_lba[14:0], save_stage_word_q, 1'b0};
	wire save_stage_ddr_req_w = save_stage_state_q == SAVE_STAGE_SAVE_DDR;
	wire [23:0] save_stage_ddr_addr_w =
		{save_sd_lba[14:0], save_stage_beat_q, 3'b000};

	wire save_slot_sd_rd;
	wire save_slot_sd_wr;
	wire save_slot_sd_ack;

	localparam [1:0] SHADOW_SOURCE_NONE  = 2'd0;
	localparam [1:0] SHADOW_SOURCE_CPU   = 2'd1;
	localparam [1:0] SHADOW_SOURCE_P1    = 2'd2;
	localparam [1:0] SHADOW_SOURCE_STAGE = 2'd3;

	reg         shadow_ddr_req_q = 1'b0;
	reg  [27:1] shadow_ddr_addr_q = 27'd0;
	reg  [63:0] shadow_ddr_din_q = 64'd0;
	reg         shadow_ddr_rnw_q = 1'b0;
	reg   [7:0] shadow_ddr_be_q = 8'd0;
	wire [63:0] shadow_ddr_dout_w;
	wire        shadow_ddr_ack_w;
	reg         shadow_ddr_busy_q = 1'b0;
	reg   [1:0] shadow_ddr_source_q = SHADOW_SOURCE_NONE;
	reg         shadow_ddr_p1_seen_q = 1'b0;
	reg         shadow_ddr_stage_seen_q = 1'b0;
	reg         save_stage_ddr_ack_pulse_q = 1'b0;
	reg         cpu_shadow_seen_valid_q = 1'b0;
	reg         cpu_shadow_seen_tag_q = 1'b0;
	// CPU SRAM writes wait here for DDR ch1; eight entries of {addr, data, be}.
	reg  [40:0] cpu_shadow_fifo_q [0:7];
	reg   [2:0] cpu_shadow_wr_q = 3'd0;
	reg   [2:0] cpu_shadow_rd_q = 3'd0;
	reg   [3:0] cpu_shadow_count_q = 4'd0;
	wire        cpu_shadow_full_w = cpu_shadow_count_q == 4'd8;
	reg         cpu_shadow_hold_q = 1'b0;
	wire [40:0] cpu_shadow_head_w = cpu_shadow_fifo_q[cpu_shadow_rd_q];

	assign sd_lba = save_sd_lba;

	vb_save_slot #(
		.MAX_BLOCKS(SAVE_BLOCKS)
	) u_save_slot (
		.clk_sys          (clk_sys),
		.invalidate_pulse (new_rom_pulse),
		.mount_pulse      (img_mounted[0]),
		.mount_readonly   (img_readonly),
		.mount_size       (img_size),
		.block_limit      (cart_sram_block_limit_w),
		.load_req         (save_load_req),
		.save_req         (save_save_req),
		.autosave_disable (save_autosave_disable),
		.osd_status       (OSD_STATUS),
		.dirty_pulse      (p1_cpu_dirty_pulse_q),
		.transfer_allowed (save_transfer_allowed),
		.mounted_writable (),
		.pending          (save_pending),
		.busy             (save_slot_busy),
		.load_active      (save_slot_load_active),
		.sd_lba           (save_sd_lba),
		.sd_rd            (save_slot_sd_rd),
		.sd_wr            (save_slot_sd_wr),
		.sd_ack           (save_slot_sd_ack)
	);

	assign sd_rd = ((save_stage_state_q == SAVE_STAGE_IDLE) && save_slot_sd_rd) ||
		((save_stage_state_q == SAVE_STAGE_LOAD_RECEIVE) &&
		 !save_stage_ack_seen_q);
	assign sd_wr = (save_stage_state_q == SAVE_STAGE_SAVE_HPS) &&
		save_slot_sd_wr && !save_stage_ack_seen_q;
	assign save_slot_sd_ack =
		((save_stage_state_q == SAVE_STAGE_LOAD_FETCH) ||
		 (save_stage_state_q == SAVE_STAGE_LOAD_DRAIN)) ? 1'b1 :
		((save_stage_state_q == SAVE_STAGE_LOAD_RECEIVE) &&
		 save_stage_ack_seen_q) ? 1'b1 : sd_ack;
	assign sd_buff_din = save_stage_buffer_q_a;

	always @(*) begin
		case (save_stage_unpack_q)
			2'd0: save_stage_buffer_wdata_b = save_stage_ddr_data_q[15:0];
			2'd1: save_stage_buffer_wdata_b = save_stage_ddr_data_q[31:16];
			2'd2: save_stage_buffer_wdata_b = save_stage_ddr_data_q[47:32];
			default: save_stage_buffer_wdata_b = save_stage_ddr_data_q[63:48];
		endcase
	end

	cache_ram_dp #(
		.ADDR_WIDTH(8),
		.DATA_WIDTH(16)
	) save_stage_buffer (
		.clk_i     (clk_sys),
		.addr_a_i  (sd_buff_addr[7:0]),
		.wren_a_i  (sd_buff_wr &&
			(save_stage_state_q == SAVE_STAGE_LOAD_RECEIVE)),
		.wdata_a_i (sd_buff_dout),
		.q_a_o     (save_stage_buffer_q_a),
		.addr_b_i  (save_stage_buffer_addr_b),
		.wren_b_i  (save_stage_buffer_wren_b),
		.wdata_b_i (save_stage_buffer_wdata_b),
		.q_b_o     (save_stage_buffer_q_b)
	);

	always @(posedge clk_sys) begin
		if (!pll_locked) begin
			old_rom_download <= 1'b0;
		end else begin
			old_rom_download <= rom_download;
		end
	end

	wire [1:0] cpu_sram_byte_enable_w = {
		(ext_be == 4'b1100) || (ext_be == 4'b1101),
		(ext_be == 4'b1100) || (ext_be == 4'b1110)
	};
	wire savestate_sram_req_raw_w = savestate_cart_sram_active &&
		(savestate_mem_rden || savestate_mem_wren);
	wire [1:0] savestate_sram_be_w = savestate_mem_addr[0] ? 2'b10 : 2'b01;
	wire [23:0] savestate_sram_addr_masked_w =
		savestate_mem_addr[23:0] & cart_sram_addr_mask_w;
	wire [23:0] save_stage_sram_addr_masked_w =
		save_stage_p1_addr_w & cart_sram_addr_mask_w;
	wire [23:0] cpu_sram_addr_masked_w =
		ext_addr & cart_sram_cpu_addr_mask_w;
	wire [23:0] cpu_sram_backing_addr_w = cart_sram_packed_x8_w ?
		{1'b0, cpu_sram_addr_masked_w[23:2], 1'b0} :
		{cpu_sram_addr_masked_w[23:1], 1'b0};
	wire cpu_sram_backing_lane_w = cart_sram_packed_x8_w &&
		cpu_sram_addr_masked_w[1];
	wire p1_cpu_read_req_w = ext_mrq && cart_ram_cs && ext_rw;
	wire p1_cpu_write_req_w = ext_mrq && cart_ram_cs && ext_da &&
		!ext_rw && cpu_sram_byte_enable_w[0];
	wire p1_cpu_req_w = p1_cpu_read_req_w || p1_cpu_write_req_w;
	wire [25:0] sram_cpu_addr_w = {2'b01, cpu_sram_backing_addr_w};
	wire [15:0] sram_cpu_write_data_w = cpu_sram_backing_lane_w ?
		{ext_dout[7:0], 8'd0} : {8'd0, ext_dout[7:0]};
	wire [1:0] sram_cpu_be_w = cpu_sram_backing_lane_w ? 2'b10 : 2'b01;
	wire cpu_shadow_match_w = cpu_shadow_seen_valid_q &&
		(cpu_shadow_seen_tag_q == ext_cycle_tag);
	wire cpu_shadow_push_w = p1_cpu_write_req_w && !cpu_shadow_match_w &&
		!cpu_shadow_full_w;
	wire cpu_shadow_pop_w;
	wire p1_shadow_req_w = p1_sys_busy_q && p1_request_write_sys_q &&
		!p1_shadow_done_q;
	assign p1_shadow_ack_pulse_w = shadow_ddr_busy_q && shadow_ddr_ack_w &&
		(shadow_ddr_source_q == SHADOW_SOURCE_P1);
	assign sram_cpu_access_done_w = sram_cpu_sdram_done_w && cpu_shadow_match_w;
	wire p1_cpu_read_ready_w = p1_cpu_read_req_w && sram_cpu_valid_w;
	wire rom_cpu_resp_ready_w = rom_core_req_w && rom_cpu_valid_w;
	// READY comes from the console wait counter. The only hold is a CPU SRAM
	// write that finds the shadow queue full, which takes a DDR3 stall long
	// enough to back up eight writes; it is a register, so READY never sees the
	// cart logic.
	wire cart_access_ready_w = !cpu_shadow_hold_q;

	assign ext_din = cart_rom_cs ? rom_cpu_data_w :
		cart_ram_cs ? sram_cpu_data_w : 16'hffff;

	`ifndef SYNTHESIS
	// READY does not wait for the SDRAM. A cartridge cycle is accepted on the
	// CE edge after which its tag changes or MRQ drops, so its data must have
	// been there just before that edge.
	wire cart_cycle_w = rom_core_req_w || p1_cpu_read_req_w || p1_cpu_write_req_w;
	wire cart_cycle_ok_w = rom_core_req_w ? rom_cpu_resp_ready_w :
		p1_cpu_read_req_w ? p1_cpu_read_ready_w : sram_cpu_access_done_w;
	reg     cart_check_active_q = 1'b0;
	reg     cart_check_tag_q = 1'b0;
	reg     cart_check_ok_q = 1'b0;
	reg     cart_check_rom_q = 1'b0;
	integer cart_check_age = 0;
	integer cart_check_age_q = 0;
	always @(posedge clk_sys) begin
		if (!cart_cycle_w || (ext_cycle_tag != cart_check_tag_q) || !cart_check_active_q)
			cart_check_age = 1;
		else
			cart_check_age = cart_check_age + 1;
		if (pll_locked && core_ce && !core_reset) begin
			if (cart_check_active_q && !cart_check_ok_q &&
				(!cart_cycle_w || (ext_cycle_tag != cart_check_tag_q))) begin
				$error("%0s cycle accepted without its SDRAM data, %0d ns after the address",
					cart_check_rom_q ? "ROM" : "SRAM", cart_check_age_q * 25);
			end
			cart_check_active_q <= cart_cycle_w;
			cart_check_tag_q <= ext_cycle_tag;
			cart_check_ok_q <= cart_cycle_ok_w;
			cart_check_rom_q <= rom_core_req_w;
			cart_check_age_q <= cart_check_age;
		end
	end
	`endif

	always @(posedge clk_sys) begin
		if (!pll_locked) begin
			p1_sys_busy_q <= 1'b0;
			p1_source_q <= P1_SOURCE_NONE;
			p1_request_write_sys_q <= 1'b0;
			p1_request_be_sys_q <= 2'b00;
			p1_request_addr_sys_q <= 26'd0;
			p1_request_data_sys_q <= 16'd0;
			p1_sdram_done_q <= 1'b0;
			p1_shadow_done_q <= 1'b0;
			p1_cpu_dirty_pulse_q <= 1'b0;
			savestate_sram_ready_q <= 1'b0;
			savestate_sram_data_q <= 8'hff;
			cart_sram_init_busy <= 1'b0;
			cart_sram_init_addr_q <= 23'd0;
		end else begin
			p1_cpu_dirty_pulse_q <= 1'b0;
			if (p1_background_done_w) begin
				p1_sdram_done_q <= 1'b1;
			end
			if (p1_shadow_ack_pulse_w) begin
				p1_shadow_done_q <= 1'b1;
			end

			// A save started from the dirty pulse must see every queued write
			// in the shadow, so the pulse waits for the queue to drain.
			if (cpu_shadow_dirty_q && (cpu_shadow_count_q == 4'd0) &&
				!(shadow_ddr_busy_q &&
				  (shadow_ddr_source_q == SHADOW_SOURCE_CPU))) begin
				cpu_shadow_dirty_q <= 1'b0;
				p1_cpu_dirty_pulse_q <= 1'b1;
			end
			if (cpu_shadow_push_w) cpu_shadow_dirty_q <= 1'b1;

			if (savestate_sram_ready_q &&
				(!savestate_sram_req_raw_w ||
				 ({2'b01, savestate_sram_addr_masked_w[23:1], 1'b0} !=
				  p1_request_addr_sys_q) ||
				 (savestate_sram_be_w != p1_request_be_sys_q))) begin
				savestate_sram_ready_q <= 1'b0;
			end

			if (new_rom_pulse) begin
				cart_sram_init_busy <= 1'b1;
				cart_sram_init_addr_q <= 23'd0;
				savestate_sram_ready_q <= 1'b0;
			end else if (p1_transaction_complete_w) begin
				p1_sys_busy_q <= 1'b0;
				p1_source_q <= P1_SOURCE_NONE;
				case (p1_source_q)
					P1_SOURCE_INIT: begin
						if (cart_sram_init_addr_q >=
							cart_sram_addr_mask_w[23:1]) begin
							cart_sram_init_busy <= 1'b0;
						end else begin
							cart_sram_init_addr_q <=
								cart_sram_init_addr_q + 23'd1;
						end
					end
					P1_SOURCE_SAVE_LOAD: begin
					end
					P1_SOURCE_SAVESTATE: begin
						savestate_sram_data_q <= p1_request_be_sys_q[1] ?
							p1_response_data_w[15:8] :
							p1_response_data_w[7:0];
						savestate_sram_ready_q <= 1'b1;
					end
					default: begin
					end
				endcase
			end else if (!p1_sys_busy_q && p1_background_ready_w) begin
				p1_sdram_done_q <= 1'b0;
				p1_shadow_done_q <= 1'b0;
				if (savestate_sram_req_raw_w && !savestate_sram_ready_q) begin
					p1_sys_busy_q <= 1'b1;
					p1_source_q <= P1_SOURCE_SAVESTATE;
					p1_request_write_sys_q <= savestate_mem_wren;
					p1_request_be_sys_q <= savestate_sram_be_w;
					p1_request_addr_sys_q <=
						{2'b01, savestate_sram_addr_masked_w[23:1], 1'b0};
					p1_request_data_sys_q <= savestate_mem_addr[0] ?
						{savestate_mem_wdata, 8'd0} :
						{8'd0, savestate_mem_wdata};
				end else if (save_stage_p1_req_w) begin
					p1_sys_busy_q <= 1'b1;
					p1_source_q <= P1_SOURCE_SAVE_LOAD;
					p1_request_write_sys_q <= 1'b1;
					p1_request_be_sys_q <= 2'b11;
					p1_request_addr_sys_q <=
						{2'b01, save_stage_sram_addr_masked_w};
					p1_request_data_sys_q <= save_stage_buffer_q_b;
				end else if (cart_sram_init_busy) begin
					p1_sys_busy_q <= 1'b1;
					p1_source_q <= P1_SOURCE_INIT;
					p1_request_write_sys_q <= 1'b1;
					p1_request_be_sys_q <= 2'b11;
					p1_request_addr_sys_q <=
						{2'b01, cart_sram_init_addr_q, 1'b0};
					p1_request_data_sys_q <= 16'hffff;
				end
			end
		end
	end

	assign cpu_shadow_pop_w = !shadow_ddr_busy_q && (cpu_shadow_count_q != 4'd0);

	// Serialize byte-enabled shadow maintenance and save prefetches on DDR ch2.
	// CPU/background writes have priority over a new save read. A dirty pulse is
	// emitted only after both canonical SDRAM and its shadow have committed, so a
	// save started from that pulse cannot observe an older value.
	always @(posedge clk_sys) begin
		if (!pll_locked) begin
			shadow_ddr_req_q <= 1'b0;
			shadow_ddr_addr_q <= 27'd0;
			shadow_ddr_din_q <= 64'd0;
			shadow_ddr_rnw_q <= 1'b0;
			shadow_ddr_be_q <= 8'd0;
			shadow_ddr_busy_q <= 1'b0;
			shadow_ddr_source_q <= SHADOW_SOURCE_NONE;
			shadow_ddr_p1_seen_q <= 1'b0;
			shadow_ddr_stage_seen_q <= 1'b0;
			save_stage_ddr_ack_pulse_q <= 1'b0;
			cpu_shadow_seen_valid_q <= 1'b0;
			cpu_shadow_seen_tag_q <= 1'b0;
			cpu_shadow_wr_q <= 3'd0;
			cpu_shadow_rd_q <= 3'd0;
			cpu_shadow_count_q <= 4'd0;
			cpu_shadow_hold_q <= 1'b0;
		end else begin
			shadow_ddr_req_q <= 1'b0;
			save_stage_ddr_ack_pulse_q <= 1'b0;

			if (!p1_shadow_req_w) begin
				shadow_ddr_p1_seen_q <= 1'b0;
			end
			if (!save_stage_ddr_req_w) begin
				shadow_ddr_stage_seen_q <= 1'b0;
			end
			if (!p1_cpu_write_req_w) begin
				cpu_shadow_seen_valid_q <= 1'b0;
			end
			cpu_shadow_hold_q <= p1_cpu_write_req_w && !cpu_shadow_match_w &&
				cpu_shadow_full_w;
			if (cpu_shadow_push_w) begin
				cpu_shadow_fifo_q[cpu_shadow_wr_q] <= {sram_cpu_addr_w[23:1],
					sram_cpu_write_data_w, sram_cpu_be_w};
				cpu_shadow_wr_q <= cpu_shadow_wr_q + 3'd1;
				cpu_shadow_seen_valid_q <= 1'b1;
				cpu_shadow_seen_tag_q <= ext_cycle_tag;
			end
			if (cpu_shadow_pop_w) cpu_shadow_rd_q <= cpu_shadow_rd_q + 3'd1;
			cpu_shadow_count_q <= cpu_shadow_count_q +
				{3'd0, cpu_shadow_push_w} - {3'd0, cpu_shadow_pop_w};

			if (shadow_ddr_busy_q) begin
				if (shadow_ddr_ack_w) begin
					shadow_ddr_busy_q <= 1'b0;
					case (shadow_ddr_source_q)
						SHADOW_SOURCE_STAGE: begin
							save_stage_ddr_ack_pulse_q <= 1'b1;
						end
						default: begin
						end
					endcase
					shadow_ddr_source_q <= SHADOW_SOURCE_NONE;
				end
			end else if (cpu_shadow_count_q != 4'd0) begin
				shadow_ddr_busy_q <= 1'b1;
				shadow_ddr_source_q <= SHADOW_SOURCE_CPU;
				shadow_ddr_req_q <= 1'b1;
				shadow_ddr_addr_q <= {4'd0, cpu_shadow_head_w[40:18]};
				shadow_ddr_din_q <= shadow_write_data_fn(
					cpu_shadow_head_w[19:18], cpu_shadow_head_w[17:2]);
				shadow_ddr_rnw_q <= 1'b0;
				shadow_ddr_be_q <= shadow_write_be_fn(
					cpu_shadow_head_w[19:18], cpu_shadow_head_w[1:0]);
			end else if (p1_shadow_req_w && !shadow_ddr_p1_seen_q) begin
				shadow_ddr_busy_q <= 1'b1;
				shadow_ddr_source_q <= SHADOW_SOURCE_P1;
				shadow_ddr_p1_seen_q <= 1'b1;
				shadow_ddr_req_q <= 1'b1;
				shadow_ddr_addr_q <= {4'd0, p1_request_addr_sys_q[23:1]};
				shadow_ddr_din_q <= shadow_write_data_fn(
					p1_request_addr_sys_q[2:1], p1_request_data_sys_q);
				shadow_ddr_rnw_q <= 1'b0;
				shadow_ddr_be_q <= shadow_write_be_fn(
					p1_request_addr_sys_q[2:1], p1_request_be_sys_q);
			end else if (save_stage_ddr_req_w &&
				!shadow_ddr_stage_seen_q) begin
				shadow_ddr_busy_q <= 1'b1;
				shadow_ddr_source_q <= SHADOW_SOURCE_STAGE;
				shadow_ddr_stage_seen_q <= 1'b1;
				shadow_ddr_req_q <= 1'b1;
				shadow_ddr_addr_q <= {4'd0, save_stage_ddr_addr_w[23:1]};
				shadow_ddr_din_q <= 64'd0;
				shadow_ddr_rnw_q <= 1'b1;
				shadow_ddr_be_q <= 8'hff;
			end
		end
	end

	always @(posedge clk_sys) begin
		if (!pll_locked || new_rom_pulse) begin
			save_stage_state_q <= SAVE_STAGE_IDLE;
			save_stage_ack_seen_q <= 1'b0;
			save_stage_word_q <= 8'd0;
			save_stage_beat_q <= 6'd0;
			save_stage_unpack_q <= 2'd0;
			save_stage_ddr_data_q <= 64'd0;
		end else begin
			case (save_stage_state_q)
				SAVE_STAGE_IDLE: begin
					save_stage_ack_seen_q <= 1'b0;
					if (save_slot_sd_rd) begin
						save_stage_state_q <= SAVE_STAGE_LOAD_RECEIVE;
					end else if (save_slot_sd_wr) begin
						save_stage_beat_q <= 6'd0;
						save_stage_state_q <= SAVE_STAGE_SAVE_DDR;
					end
				end

				SAVE_STAGE_LOAD_RECEIVE: begin
					if (sd_ack) begin
						save_stage_ack_seen_q <= 1'b1;
					end else if (save_stage_ack_seen_q) begin
						save_stage_word_q <= 8'd0;
						save_stage_state_q <= SAVE_STAGE_LOAD_FETCH;
					end
				end

				SAVE_STAGE_LOAD_FETCH: begin
					// Port B is synchronous. Present the next word address for one
					// complete clk_sys cycle before allowing P1 to capture q_b.
					save_stage_state_q <= SAVE_STAGE_LOAD_DRAIN;
				end

				SAVE_STAGE_LOAD_DRAIN: begin
					if (p1_transaction_complete_w &&
						(p1_source_q == P1_SOURCE_SAVE_LOAD)) begin
						if (save_stage_word_q == 8'hff) begin
							save_stage_state_q <= SAVE_STAGE_IDLE;
							save_stage_ack_seen_q <= 1'b0;
						end else begin
							save_stage_word_q <= save_stage_word_q + 8'd1;
							save_stage_state_q <= SAVE_STAGE_LOAD_FETCH;
						end
					end
				end

				SAVE_STAGE_SAVE_DDR: begin
					if (save_stage_ddr_ack_pulse_q) begin
						save_stage_ddr_data_q <= shadow_ddr_dout_w;
						save_stage_unpack_q <= 2'd0;
						save_stage_state_q <= SAVE_STAGE_SAVE_UNPACK;
					end
				end

				SAVE_STAGE_SAVE_UNPACK: begin
					if (save_stage_unpack_q == 2'd3) begin
						if (save_stage_beat_q == 6'd63) begin
							save_stage_ack_seen_q <= 1'b0;
							save_stage_state_q <= SAVE_STAGE_SAVE_HPS;
						end else begin
							save_stage_beat_q <= save_stage_beat_q + 6'd1;
							save_stage_state_q <= SAVE_STAGE_SAVE_DDR;
						end
					end else begin
						save_stage_unpack_q <= save_stage_unpack_q + 2'd1;
					end
				end

				SAVE_STAGE_SAVE_HPS: begin
					if (sd_ack) begin
						save_stage_ack_seen_q <= 1'b1;
					end else if (save_stage_ack_seen_q) begin
						save_stage_ack_seen_q <= 1'b0;
						save_stage_state_q <= SAVE_STAGE_IDLE;
					end
				end

				default: begin
					save_stage_state_q <= SAVE_STAGE_IDLE;
				end
			endcase
		end
	end

	assign savestate_ram_read_data =
		savestate_cart_sram_active ? savestate_sram_data_q :
		savestate_core_mem_rdata;
	assign savestate_ram_ready =
		savestate_cart_sram_active ? savestate_sram_ready_q : 1'b1;
	assign cart_ram_clear_mem_ready = 1'b1;

	///////////////////////   MEMORY INSTANCES   /////////////////////

	vb_cart_rom u_cart (
		.clk_sys             (clk_sys),
		.clk_ram             (clk_ram),
		.clk_ram_pin         (clk_ram_pin),
		.reset_i             (!pll_locked),
		.rom_download_i      (rom_download),
		.ioctl_wr_i          (ioctl_wr),
		.ioctl_addr_i        (ioctl_addr),
		.ioctl_dout_i        (ioctl_dout),
		.ioctl_wait_o        (rom_ioctl_wait),
		.rom_loaded_o        (rom_loaded),
		.req_valid_i         (rom_core_req_w),
		.req_tag_i           (ext_cycle_tag),
		.req_addr_i          (ext_addr),
		.resp_data_o         (rom_cpu_data_w),
		.resp_ready_o        (rom_cpu_valid_w),
		.sram_req_i          (p1_cpu_req_w),
		.sram_we_i           (p1_cpu_write_req_w),
		.sram_addr_i         (sram_cpu_addr_w),
		.sram_write_data_i   (sram_cpu_write_data_w),
		.sram_be_i           (sram_cpu_be_w),
		.sram_read_lane_i    (cpu_sram_backing_lane_w),
		.sram_tag_i          (ext_cycle_tag),
		.sram_read_data_o    (sram_cpu_data_w),
		.sram_read_valid_o   (sram_cpu_valid_w),
		.sram_access_done_o  (sram_cpu_sdram_done_w),
		.sram_bg_req_i       (p1_sys_busy_q),
		.sram_bg_we_i        (p1_request_write_sys_q),
		.sram_bg_addr_i      (p1_request_addr_sys_q),
		.sram_bg_write_data_i(p1_request_data_sys_q),
		.sram_bg_be_i        (p1_request_be_sys_q),
		.sram_bg_ready_o     (p1_background_ready_w),
		.sram_bg_done_o      (p1_background_done_w),
		.sram_bg_read_data_o (p1_response_data_w),
		.foreground_idle_o   (sdram_foreground_idle_w),
		.refresh_window_i    (cart_refresh_window_w),
		.cpu_stopped_i       (cart_cpu_stopped_w),
		.quiet_o             (cart_sdram_quiet_w),
		.SDRAM_CLK           (SDRAM_CLK),
		.SDRAM_CKE           (SDRAM_CKE),
		.SDRAM_A             (SDRAM_A),
		.SDRAM_BA            (SDRAM_BA),
		.SDRAM_DQ            (SDRAM_DQ),
		.SDRAM_DQML          (SDRAM_DQML),
		.SDRAM_DQMH          (SDRAM_DQMH),
		.SDRAM_nCS           (SDRAM_nCS),
		.SDRAM_nCAS          (SDRAM_nCAS),
		.SDRAM_nRAS          (SDRAM_nRAS),
		.SDRAM_nWE           (SDRAM_nWE)
	);

	assign DDRAM_CLK = clk_sys;

	///////////////////////   SAVESTATES   ///////////////////////////

	wire [27:1] shared_aux_ddr_addr;
	wire [63:0] shared_aux_ddr_dout;
	wire [63:0] shared_aux_ddr_din;
	wire        shared_aux_ddr_req;
	wire        shared_aux_ddr_rnw;
	wire [7:0]  shared_aux_ddr_be;
	wire        shared_aux_ddr_ready;

	vb_tas_player u_tas_player (
		.clk_i          (clk_sys),
		.reset_i        (~pll_locked),
		.cancel_i       (reset_button | rom_download | savestate_restore_begin),
		.download_i     (tas_download),
		.write_i        (ioctl_wr),
		.addr_i         (ioctl_addr),
		.data_i         (ioctl_dout),
		.ioctl_wait_o   (tas_ioctl_wait),
		.pad_latch_i    (tas_pad_latch),
		.pad_word_o     (tas_pad_word),
		.pad_override_o (tas_pad_override),
		.hold_reset_o   (tas_hold_reset),
		.load_start_o   (tas_load_start),
		.loaded_o       (),
		.complete_o     (),
		.load_error_o   (),
		.fault_o        (tas_fault),
		.ddr_addr_o     (tas_ddr_addr),
		.ddr_dout_i     (tas_ddr_dout),
		.ddr_din_o      (tas_ddr_din),
		.ddr_req_o      (tas_ddr_req),
		.ddr_rnw_o      (tas_ddr_rnw),
		.ddr_be_o       (tas_ddr_be),
		.ddr_ready_i    (tas_ddr_ready)
	);

	vb_ddr_channel_arbiter u_aux_ddr_arbiter (
		.clk_i          (clk_sys),
		.reset_i        (~pll_locked),
		.ch1_addr_i     (shadow_ddr_addr_q),
		.ch1_dout_o     (shadow_ddr_dout_w),
		.ch1_din_i      (shadow_ddr_din_q),
		.ch1_req_i      (shadow_ddr_req_q),
		.ch1_rnw_i      (shadow_ddr_rnw_q),
		.ch1_be_i       (shadow_ddr_be_q),
		.ch1_ready_o    (shadow_ddr_ack_w),
		.ch2_addr_i     (tas_ddr_addr),
		.ch2_dout_o     (tas_ddr_dout),
		.ch2_din_i      (tas_ddr_din),
		.ch2_req_i      (tas_ddr_req),
		.ch2_rnw_i      (tas_ddr_rnw),
		.ch2_be_i       (tas_ddr_be),
		.ch2_ready_o    (tas_ddr_ready),
		.shared_addr_o  (shared_aux_ddr_addr),
		.shared_dout_i  (shared_aux_ddr_dout),
		.shared_din_o   (shared_aux_ddr_din),
		.shared_req_o   (shared_aux_ddr_req),
		.shared_rnw_o   (shared_aux_ddr_rnw),
		.shared_be_o    (shared_aux_ddr_be),
		.shared_ready_i (shared_aux_ddr_ready)
	);

	ddram u_savestate_ddram (
		.DDRAM_CLK        (clk_sys),
		.DDRAM_BUSY       (DDRAM_BUSY),
		.DDRAM_BURSTCNT   (DDRAM_BURSTCNT),
		.DDRAM_ADDR       (DDRAM_ADDR),
		.DDRAM_DOUT       (DDRAM_DOUT),
		.DDRAM_DOUT_READY (DDRAM_DOUT_READY),
		.DDRAM_RD         (DDRAM_RD),
		.DDRAM_DIN        (DDRAM_DIN),
		.DDRAM_BE         (DDRAM_BE),
		.DDRAM_WE         (DDRAM_WE),
		.ch1_addr         ({savestate_ddr_addr, 1'b0}),
		.ch1_dout         (savestate_ddr_dout),
		.ch1_din          (savestate_ddr_din),
		.ch1_req          (savestate_ddr_req),
		.ch1_rnw          (savestate_ddr_rnw),
		.ch1_be           (savestate_ddr_be),
		.ch1_ready        (savestate_ddr_ack),
		.ch2_addr         (shared_aux_ddr_addr),
		.ch2_dout         (shared_aux_ddr_dout),
		.ch2_din          (shared_aux_ddr_din),
		.ch2_req          (shared_aux_ddr_req),
		.ch2_rnw          (shared_aux_ddr_rnw),
		.ch2_be           (shared_aux_ddr_be),
		.ch2_ready        (shared_aux_ddr_ready)
	);

	savestate_ui #(
		.INFO_TIMEOUT_BITS(25)
	) u_savestate_ui (
		.clk           (clk_sys),
		.ps2_key       (ps2_key),
		.allow_ss      (savestate_menu_enabled),
		.joySS         (joystick_0[23]),
		.joyRight      (joystick_0[0]),
		.joyLeft       (joystick_0[1]),
		.joyDown       (joystick_0[2]),
		.joyUp         (joystick_0[3]),
		.joyStart      (joystick_0[9]),
		.joySaveState  (joystick_0[14]),
		.status_slot   (status[46:45]),
		.OSD_saveload  (status[43:42]),
		.ss_save       (savestate_ui_save),
		.ss_load       (savestate_ui_load),
		.ss_info_req   (savestate_info_req),
		.ss_info       (savestate_info),
		.statusUpdate  (savestate_status_update),
		.selected_slot (savestate_slot)
	);

	statemanager #(
		.Softmap_SaveState_ADDR(VIRTUALBOY_SAVESTATE_BASE),
		.Softmap_Rewind_ADDR(0),
		.SAVESTATE_SHIFT(VIRTUALBOY_SAVESTATE_SLOT_SHIFT)
	) u_statemanager (
		.clk               (clk_sys),
		.reset             (savestate_control_reset),
		.rewind_on         (1'b0),
		.rewind_active     (1'b0),
		.savestate_number  ({30'd0, savestate_slot}),
		.save              (savestate_ui_save),
		.load              (savestate_ui_load),
		.sleep_rewind      (),
		.vsync             (public_video_vsync),
		.request_savestate (savestate_savestate),
		.request_loadstate (savestate_loadstate),
		.request_address   (savestate_address),
		.request_busy      (savestate_busy)
	);

	savestates #(
		.STATESIZE_PARAM(4276938),
		.SETTLECOUNT_PARAM(16),
		.INTERNALSCOUNT_PARAM(VIRTUALBOY_SS_INTERNAL_WORDS),
		.SAVETYPESCOUNT_PARAM(6),
		.SAVETYPE0_OFFSET(25'd0),
		.SAVETYPE1_OFFSET(25'd0),
		.SAVETYPE2_OFFSET(25'd0),
		.SAVETYPE3_OFFSET(25'd0),
		.SAVETYPE4_OFFSET(25'd0),
		.SAVETYPE5_OFFSET(25'd0),
		.SAVETYPE0_SIZE(65536),
		.SAVETYPE1_SIZE(131072),
		.SAVETYPE2_SIZE(131072),
		.SAVETYPE3_SIZE(16777216),
		.SAVETYPE4_SIZE(288),
		.SAVETYPE5_SIZE(1664)
	) u_savestates (
		.clk                   (clk_sys),
		.reset_in              (savestate_control_reset),
		.reset_ss              (savestate_reset_ss),
		.reset_delay           (),
		.restore_begin         (savestate_restore_begin),
		.load_done             (),
		.increaseSSHeaderCount (!status[44]),
		.save                  (savestate_savestate),
		.load                  (savestate_loadstate),
		.state_size_i          (savestate_payload_dwords_w),
		.savetype3_size_i      (savestate_cart_sram_size_w),
		.savestate_address     (savestate_address),
		.savestate_busy        (savestate_busy),
		.paused                (savestate_pause_active_q),
		.BUS_Din               (savestate_state_wdata),
		.BUS_Adr               (savestate_state_addr),
		.BUS_wren              (savestate_state_wren),
		.BUS_rst               (),
		.BUS_Dout              (savestate_state_rdata),
		.loading_savestate     (loading_savestate),
		.saving_savestate      (saving_savestate),
		.sleep_savestate       (sleep_savestate),
		.Save_RAMAddr          (savestate_mem_addr),
		.Save_RAMRdEn          (savestate_mem_rden),
		.Save_RAMWrEn          (savestate_mem_wren),
		.Save_RAMWriteData     (savestate_mem_wdata),
		.Save_RAMReadData      (savestate_ram_read_data),
		.Save_RAMReady         (savestate_ram_ready),
		.Save_RAMType          (savestate_mem_type),
		.bus_out_Din           (savestate_ddr_din),
		.bus_out_Dout          (savestate_ddr_dout),
		.bus_out_Adr           (savestate_ddr_addr),
		.bus_out_rnw           (savestate_ddr_rnw),
		.bus_out_ena           (savestate_ddr_req),
		.bus_out_be            (savestate_ddr_be),
		.bus_out_done          (savestate_ddr_ack)
	);

	assign savestate_pause_ready = savestate_core_pause_ready &&
		!p1_sys_busy_q && sdram_foreground_idle_w && !ext_mrq;

	// DDR ch2 can copy the SRAM shadow to HPS while gameplay continues. Loads
	// modify canonical SDRAM, so only that direction requests a frame stop.
	assign save_pause_busy = save_slot_load_active | cart_sram_init_busy |
		cart_ram_clear_busy;
	assign save_transfer_allowed = (cpu_shadow_count_q == 4'd0) &&
		!cart_sram_init_busy &&
		!cart_ram_clear_busy && !sleep_savestate && !savestate_busy &&
		(!save_slot_load_active ||
		 (!core_execution_enable && !video_source_enable));

	vb_video_source_coordinator u_video_source_coordinator (
		.clk_i             (clk_sys),
		.reset_i           (savestate_control_reset),
		.core_reset_i      (core_reset),
		.execution_block_i (save_pause_busy),
		.pause_req_i       (sleep_savestate),
		.pause_ready_i     (savestate_pause_ready),
		.frame_wrap_i      (video_frame_wrap),
		.pause_active_o    (savestate_pause_active_q),
		.core_enable_o     (core_execution_enable),
		.source_enable_o   (video_source_enable)
	);

	///////////////////////   CORE   /////////////////////////////////

	wire [1:0] video_raw_left_raw;
	wire [1:0] video_raw_right_raw;
	wire [7:0] video_luma_left_raw;
	wire [7:0] video_luma_right_raw;
	wire       video_hblank;
	wire       video_vblank;

	wire [15:0] pad_buttons = {
		joystick_0[12], // RD
		joystick_0[11], // RL
		joystick_0[8],  // Select
		joystick_0[9],  // Start
		joystick_0[3],  // LU
		joystick_0[2],  // LD
		joystick_0[1],  // LL
		joystick_0[0],  // LR
		joystick_0[10], // RR
		joystick_0[13], // RU
		joystick_0[6],  // L
		joystick_0[7],  // R
		joystick_0[5],  // B
		joystick_0[4],  // A
		2'b00
	};
	wire [15:0] core_pad_buttons = tas_pad_override ? tas_pad_word : pad_buttons;

	localparam [1:0] EYE_DRAW_LEFT         = 2'd0;
	localparam [1:0] EYE_DRAW_RIGHT        = 2'd1;
	localparam [1:0] EYE_DRAW_SIDE_BY_SIDE = 2'd3;

	wire [2:0] stereo_scale = status[27:25];
	wire [1:0] eye_draw     = status[8:7];
	wire       side_by_side_requested =
		(eye_draw == EYE_DRAW_SIDE_BY_SIDE);
	wire       side_by_side;
	wire       video_60hz_compat;
	wire       draw_left_eye  = (eye_draw != EYE_DRAW_RIGHT);
	wire       draw_right_eye = (eye_draw != EYE_DRAW_LEFT);
	wire       colorizer_dual_eye_draw = draw_left_eye && draw_right_eye &&
		!side_by_side;
	// Native mode supplies 1,280 normal or 2,560 side-by-side source steps per
	// line and is exactly 20 ms. Compatibility mode supplies 1,270 or 2,540 steps
	// on a fixed 262-line progressive raster. Neither mode changes the native CE.
	wire       source_video_ce;
	wire       source_video_ce_core = source_video_ce &&
		video_source_enable && !savestate_pause_active_q;

	assign VGA_SL = 2'd0;
	assign AUDIO_MIX = status[39:38];

	wire [15:0] vsu_audio_l;
	wire [15:0] vsu_audio_r;
	wire        vsu_audio_sample;

	virtualboy #(
		// The VIP-facing producer runs fast enough to service its synchronous
		// framebuffer reads: 1,280 clocks at 20 MHz in normal mode and 2,560
		// clocks at 40 MHz for side-by-side. A one-line Block RAM bridge below
		// converts those lines to the public 10/20 MHz CRT raster. Both producer and
		// public enables pause together only in terminal horizontal blanking.
		// Presentation is strictly progressive in both 312-line native and
		// 262-line NTSC-compatibility timing.
		// Native framebuffer epochs are decoupled by the generation-owned
		// presentation stores.
		.VIDEO_H_VISIBLE(384),
		.VIDEO_H_TOTAL(1280),
		.VIDEO_H_SYNC_BEG(1056),
		.VIDEO_H_SYNC_END(1152),
		.VIDEO_V_VISIBLE(224),
		.VIDEO_V_TOTAL(312),
		.VIDEO_V_SYNC_BEG(289),
		.VIDEO_V_SYNC_END(292),
		.VIDEO_FLAT_OUTPUT(1'b1)
	) u_virtualboy (
		.clk_i                         (clk_sys),
		.reset_i                       (core_reset),
		.ce_i                          (core_ce),
		.video_ce_i                    (source_video_ce_core),
		.savestate_pause_req_i         (sleep_savestate),
		.savestate_pause_drain_ready_o (),
		.savestate_pause_ready_o       (savestate_core_pause_ready),
		.savestate_restore_i           (savestate_restore_begin),
		.savestate_state_addr_i        (savestate_state_addr[6:0]),
		.savestate_state_wdata_i       (savestate_state_wdata),
		.savestate_state_wren_i        (savestate_state_wren),
		.savestate_state_rdata_o       (savestate_state_rdata),
		.savestate_mem_active_i        (core_mem_active),
		.savestate_mem_type_i          (core_mem_type),
		.savestate_mem_addr_i          (core_mem_addr),
		.savestate_mem_rden_i          (core_mem_rden),
		.savestate_mem_wren_i          (core_mem_wren),
		.savestate_mem_wdata_i         (core_mem_wdata),
		.savestate_mem_rdata_o         (savestate_core_mem_rdata),
		.cheat_clear_i                 (cheat_clear),
		.cheat_code_i                  (cheat_code),

		.pad_buttons_i                 (core_pad_buttons),
		.cart_irq_i                    (1'b0),
		.rumble_enable_i               (serial_rumble_enable),
		.link_comcnt_i                 (1'b1),
		.link_clk_i                    (1'b0),
		.link_rx_i                     (1'b0),
		.link_sync_i                   (1'b0),
		.flat_dual_eye_i               (draw_right_eye),
		.side_by_side_i                (side_by_side),
		.compat_60hz_i                 (video_60hz_compat),
		.parallax_scale_i              (stereo_scale),
		.scanner_spinup_i              (status[30]),
		.presentation_hlg_i            (presentation_hlg_enabled),
		.brightness_table_legacy_sdr_i (brightness_table_legacy_sdr),
		.brightness_table_download_i   (brightness_table_download),
		.brightness_table_write_i      (brightness_table_write),
		.brightness_table_addr_i       (brightness_table_addr),
		.brightness_table_data_i       (brightness_table_data),

		.pad_latch_o                   (tas_pad_latch),
		.pad_clock_o                   (),
		.link_comcnt_o                 (),
		.link_clk_o                    (),
		.link_tx_o                     (),
		.link_sync_o                   (),
		.rumble_o                      (joystick_0_rumble),

		.ext_a_o                       (ext_a),
		.ext_dout_o                    (ext_dout),
		.ext_dout_oe_o                 (),
		.ext_be_o                      (ext_be),
		.ext_st_o                      (),
		.ext_da_o                      (ext_da),
		.ext_mrq_o                     (ext_mrq),
		.ext_rw_o                      (ext_rw),
		.wram_cs_o                     (),
		.cart_exp_cs_o                 (),
		.cart_ram_cs_o                 (cart_ram_cs),
		.cart_rom_cs_o                 (cart_rom_cs),
		.ext_cycle_tag_o               (ext_cycle_tag),
		.ext_din_i                     (ext_din),
		.ext_ready_i                   (cart_access_ready_w),
		.cart_refresh_window_o         (cart_refresh_window_w),

		.video_raw_l_o                 (video_raw_left_raw),
		.video_raw_r_o                 (video_raw_right_raw),
		.video_luma_l_o                (video_luma_left_raw),
		.video_luma_r_o                (video_luma_right_raw),
		.video_hblank_o                (video_hblank),
		.video_vblank_o                (video_vblank),
		.video_hsync_o                 (),
		.video_vsync_o                 (),
		.audio_l_o                     (vsu_audio_l),
		.audio_r_o                     (vsu_audio_r),
		.audio_sample_valid_o          (vsu_audio_sample),
		.timer_tick_o                  (),
		.timer_zero_o                  (),
		.timer_irq_o                   (),
		.cpu_bus_mrq_o                 (cpu_bus_mrq),
		.cpu_bus_ready_o               (cpu_bus_ready),
		.cpu_bus_region_o              (cpu_bus_region),
		.cpu_bus_da_o                  (),
		.cpu_bus_rw_o                  (cpu_bus_rw),
		.cpu_bus_st_o                  (),
		.cpu_bus_bcyst_o               (),

		.dbg_reg_addr_i                (5'd0),
		.dbg_reg_data_o                (),
		.dbg_pc_o                      (),
		.dbg_psw_o                     (),
		.trace_valid_o                 (),
		.trace_pc_o                    (),
		.trace_insn_o                  (),
		.cpu_halted_o                  (),
		.cpu_illegal_o                 ()
	);

	///////////////////////   VIDEO / AUDIO   ////////////////////////

	// Console RC coupling, then a band-limited move to 48 kHz on CLK_AUDIO.
	wire               audio_hp_sample;
	wire signed [17:0] audio_hp_l;
	wire signed [17:0] audio_hp_r;

	vsu_rc_highpass u_audio_highpass (
		.clk_i    (clk_sys),
		.sample_i (vsu_audio_sample),
		.left_i   (vsu_audio_l[15:6]),
		.right_i  (vsu_audio_r[15:6]),
		.sample_o (audio_hp_sample),
		.left_o   (audio_hp_l),
		.right_o  (audio_hp_r)
	);

	audio_resampler u_audio_resampler (
		.src_clk_i    (clk_sys),
		.src_sample_i (audio_hp_sample),
		.src_left_i   (audio_hp_l),
		.src_right_i  (audio_hp_r),
		.clk_i        (CLK_AUDIO),
		.left_o       (AUDIO_L),
		.right_o      (AUDIO_R)
	);

	wire [1:0] ar = status[2:1];
	wire video_active = video_source_enable && !core_reset &&
		!savestate_pause_active_q;
	reg [4:0] crop_offset_lines;

	always @(posedge CLK_VIDEO) begin
		crop_216_available <= (HDMI_WIDTH == 12'd1920) &&
			(HDMI_HEIGHT == 12'd1080) && !forced_scandoubler &&
			(video_scale == 2'd0);
		crop_offset_lines <= (crop_offset_choice < 4'd6) ?
			{crop_offset_choice, 1'b0} :
			({crop_offset_choice, 1'b0} - 5'd24);
	end

	// These direct aliases are production-boundary seams checked by the VIP
	// integration policy. Keep their names even though the current mapping is 1:1.
	wire [7:0] video_luma_left_aligned = video_luma_left_raw;
	wire video_hblank_aligned = video_hblank;
	wire video_vblank_aligned = video_vblank;
	wire video_hblank_final = video_hblank_aligned;
	wire video_vblank_final = video_vblank_aligned;

	wire [7:0] video_luma_left = draw_left_eye ? video_luma_left_aligned : 8'd0;
	wire [7:0] video_luma_right = draw_right_eye ? video_luma_right_raw : 8'd0;

	wire [11:0] video_arx_base = side_by_side ? 12'd16 :
		((ar == 2'd0) ? 12'd12 : {10'd0, ar - 2'd1});
	wire [11:0] video_ary_base = side_by_side ? 12'd9 :
		((ar == 2'd0) ? 12'd7 : 12'd0);

	wire [7:0] vid_red;
	wire [7:0] vid_green;
	wire [7:0] vid_blue;
	wire public_video_de;
	wire video_visible = video_active && !(video_hblank_final | video_vblank_final);

	vb_video_colorizer u_video_colorizer (
		.video_active_i  (video_visible),
		.color_mode_i    (color_mode),
		.dual_eye_draw_i (colorizer_dual_eye_draw),
		.left_color_i    (left_eye_color),
		.right_color_i   (right_eye_color),
		.left_raw_i      (video_raw_left_raw),
		.right_raw_i     (video_raw_right_raw),
		.left_luma_i     (video_luma_left),
		.right_luma_i    (video_luma_right),
		.palette_rgb_i   (loaded_palette_rgb),
		.true_black_i    (palette_true_black),
		.red_o           (vid_red),
		.green_o         (vid_green),
		.blue_o          (vid_blue)
	);

	vb_video_output u_video_output (
		.clk_i              (clk_sys),
		.reset_i            (~pll_locked),
		.side_by_side_req_i (side_by_side_requested),
		.compat_60hz_req_i  (video_60hz_compat_requested),
		.source_ce_i        (source_video_ce_core),
		.source_present_i   (video_active),
		.source_red_i       (vid_red),
		.source_green_i     (vid_green),
		.source_blue_i      (vid_blue),
		.source_hblank_i    (video_hblank_final),
		.source_vblank_i    (video_vblank_final),
		.side_by_side_o     (side_by_side),
		.compat_60hz_o      (video_60hz_compat),
		.frame_wrap_o       (video_frame_wrap),
		.pixel_ce_o         (CE_PIXEL),
		.source_timing_ce_o (source_video_ce),
		.red_o              (VGA_R),
		.green_o            (VGA_G),
		.blue_o             (VGA_B),
		.hsync_o            (VGA_HS),
		.vsync_o            (public_video_vsync),
		.de_o               (public_video_de)
	);

	video_freak u_video_freak (
		.CLK_VIDEO   (CLK_VIDEO),
		.CE_PIXEL    (CE_PIXEL),
		.VGA_VS      (public_video_vsync),
		.HDMI_WIDTH  (HDMI_WIDTH),
		.HDMI_HEIGHT (HDMI_HEIGHT),
		.VGA_DE      (VGA_DE),
		.VIDEO_ARX   (VIDEO_ARX),
		.VIDEO_ARY   (VIDEO_ARY),
		.VGA_DE_IN   (public_video_de),
		.ARX         (video_arx_base),
		.ARY         (video_ary_base),
		.CROP_SIZE   ((crop_216_available && crop_216_enabled) ? 12'd216 : 12'd0),
		.CROP_OFF    (crop_offset_lines),
		.SCALE       ({1'b0, video_scale})
	);

	assign VGA_VS = public_video_vsync;

endmodule

// Accepts VBT1 (256-byte legacy SDR) and VBT2 (1024-byte SDR + HLG) brightness
// table files and streams validated payload words to the VIP.
module vb_vbt_brightness_loader (
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        download_i,
	input  wire        write_i,
	input  wire [26:0] addr_i,
	input  wire [15:0] data_i,
	output wire        table_write_o,
	output wire [9:0]  table_addr_o,
	output wire [15:0] table_data_o,
	output wire        table_legacy_sdr_o
);

	reg vbt_magic_0_q;
	reg vbt1_header_valid_q;
	reg vbt2_header_valid_q;
	reg legacy_sdr_q;

	wire [10:0] payload_word_addr_w = addr_i[11:1] - 11'd2;
	wire vbt1_payload_addr_valid_w = !addr_i[0] &&
		(addr_i >= 27'd4) && (addr_i <= 27'd258);
	wire vbt2_payload_addr_valid_w = !addr_i[0] &&
		(addr_i >= 27'd4) && (addr_i <= 27'd2050);
	wire payload_addr_valid_w =
		(vbt1_header_valid_q && vbt1_payload_addr_valid_w) ||
		(vbt2_header_valid_q && vbt2_payload_addr_valid_w);

	// VBT1 carries 256 legacy SDR bytes. VBT2 carries complete 1024-byte
	// SDR and HLG banks. WIDE mode presents "VB" as 16'h4256, with "T1"
	// as 16'h3154 and "T2" as 16'h3254.
	always @(posedge clk_i) begin
		if (reset_i || !download_i) begin
			vbt_magic_0_q <= 1'b0;
			vbt1_header_valid_q <= 1'b0;
			vbt2_header_valid_q <= 1'b0;
			if (reset_i) begin
				legacy_sdr_q <= 1'b0;
			end
		end else if (write_i) begin
			case (addr_i)
				27'd0: begin
					vbt_magic_0_q <= data_i == 16'h4256;
					vbt1_header_valid_q <= 1'b0;
					vbt2_header_valid_q <= 1'b0;
				end
				27'd2: begin
					vbt1_header_valid_q <= vbt_magic_0_q &&
						(data_i == 16'h3154);
					vbt2_header_valid_q <= vbt_magic_0_q &&
						(data_i == 16'h3254);
					if (vbt_magic_0_q && (data_i == 16'h3154)) begin
						legacy_sdr_q <= 1'b1;
					end else if (vbt_magic_0_q &&
						(data_i == 16'h3254)) begin
						legacy_sdr_q <= 1'b0;
					end
				end
				default: begin
				end
			endcase
		end
	end

	assign table_write_o = download_i && write_i && payload_addr_valid_w;
	assign table_addr_o = payload_word_addr_w[9:0];
	assign table_data_o = data_i;
	assign table_legacy_sdr_o = legacy_sdr_q;

endmodule

module vb_gbp_palette_loader #(
	// Standard GBP payload order is four light-to-dark RGB888 entries.
	parameter [95:0] DEFAULT_PALETTE_RGB = 96'he30000_950000_560000_000000
) (
	input  wire        clk_i,
	input  wire        reset_i,
	input  wire        download_i,
	input  wire        write_i,
	input  wire [26:0] addr_i,
	input  wire [15:0] data_i,
	output reg  [95:0] palette_rgb_o
);

	always @(posedge clk_i) begin
		if (reset_i) begin
			palette_rgb_o <= DEFAULT_PALETTE_RGB;
		end else if (download_i && write_i) begin
			// hps_io WIDE mode presents two consecutive file bytes per write,
			// with the lower-addressed byte in data_i[7:0]. Update the live
			// palette at each valid address so visibility does not depend on an
			// exact transfer count or on detecting the end of the download.
			case (addr_i)
				27'd0: begin
					palette_rgb_o[95:88] <= data_i[7:0];
					palette_rgb_o[87:80] <= data_i[15:8];
				end
				27'd2: begin
					palette_rgb_o[79:72] <= data_i[7:0];
					palette_rgb_o[71:64] <= data_i[15:8];
				end
				27'd4: begin
					palette_rgb_o[63:56] <= data_i[7:0];
					palette_rgb_o[55:48] <= data_i[15:8];
				end
				27'd6: begin
					palette_rgb_o[47:40] <= data_i[7:0];
					palette_rgb_o[39:32] <= data_i[15:8];
				end
				27'd8: begin
					palette_rgb_o[31:24] <= data_i[7:0];
					palette_rgb_o[23:16] <= data_i[15:8];
				end
				27'd10: begin
					palette_rgb_o[15:8] <= data_i[7:0];
					palette_rgb_o[7:0] <= data_i[15:8];
				end
				default: begin end
			endcase
		end
	end

endmodule

// Maps the two raw eye channels onto the selected output color mode.
module vb_video_colorizer (
	input  wire        video_active_i,
	input  wire [2:0]  color_mode_i,
	input  wire        dual_eye_draw_i,
	input  wire [2:0]  left_color_i,
	input  wire [2:0]  right_color_i,
	input  wire [1:0]  left_raw_i,
	input  wire [1:0]  right_raw_i,
	input  wire [7:0]  left_luma_i,
	input  wire [7:0]  right_luma_i,
	input  wire [95:0] palette_rgb_i,
	input  wire        true_black_i,
	output reg  [7:0]  red_o,
	output reg  [7:0]  green_o,
	output reg  [7:0]  blue_o
);

	localparam [2:0] COLOR_MODE_NATURAL    = 3'd0;
	localparam [2:0] COLOR_MODE_MULTICOLOR = 3'd1;
	localparam [2:0] COLOR_MODE_PALETTE    = 3'd2;
	localparam [2:0] COLOR_MODE_ANAGLYPH   = 3'd3;
	localparam [2:0] COLOR_MODE_COLORCODE  = 3'd4;
	localparam [2:0] COLOR_MODE_TRIOVIZ    = 3'd5;
	localparam [2:0] COLOR_MODE_CUSTOM     = 3'd6;
	localparam [2:0] COLOR_MODE_PURE_RED   = 3'd7;

	function automatic [23:0] custom_eye_rgb_fn;
		input [2:0] color_i;
		input [7:0] luma_i;
		begin
			case (color_i)
				3'd0: custom_eye_rgb_fn = {luma_i, 8'd0, 8'd0};
				3'd1: custom_eye_rgb_fn = {luma_i, 8'd0, luma_i};
				3'd2: custom_eye_rgb_fn = {8'd0, 8'd0, luma_i};
				3'd3: custom_eye_rgb_fn = {8'd0, luma_i, luma_i};
				3'd4: custom_eye_rgb_fn = {8'd0, luma_i, 8'd0};
				3'd5: custom_eye_rgb_fn = {luma_i, luma_i, 8'd0};
				3'd6: custom_eye_rgb_fn = {luma_i, luma_i, luma_i};
				default: custom_eye_rgb_fn = {luma_i, 8'd0, 8'd0};
			endcase
		end
	endfunction

	function automatic [7:0] palette_blend_channel_fn;
		input [7:0] base_i;
		input [7:0] target_i;
		input [7:0] luma_i;
		reg [7:0] delta_v;
		reg [8:0] luma_plus_one_v;
		reg [15:0] product_v;
		reg [7:0] result_v;
		begin
			// delta * (luma + 1) >> 8 has exact 0/full endpoints and avoids
			// synthesizable division while bounding each channel to one 8x9 multiply.
			luma_plus_one_v = {1'b0, luma_i} + 9'd1;
			if (target_i >= base_i) begin
				delta_v = target_i - base_i;
				product_v = delta_v * luma_plus_one_v;
				result_v = base_i + product_v[15:8];
			end else begin
				delta_v = base_i - target_i;
				product_v = delta_v * luma_plus_one_v;
				result_v = base_i - product_v[15:8];
			end
			palette_blend_channel_fn = result_v;
		end
	endfunction

	wire left_luma_wins = (left_luma_i > right_luma_i);
	wire [7:0] combined_luma = left_luma_wins ? left_luma_i : right_luma_i;
	wire [1:0] combined_raw = left_luma_wins ? left_raw_i : right_raw_i;
	reg [23:0] palette_target_rgb;
	wire [23:0] palette_base_rgb = true_black_i ? 24'd0 : palette_rgb_i[23:0];

	// GBP is light-to-dark while VIP raw classes are dark-to-light.
	always @(*) begin
		case (combined_raw)
			2'd0: palette_target_rgb = palette_rgb_i[23:0];
			2'd1: palette_target_rgb = palette_rgb_i[47:24];
			2'd2: palette_target_rgb = palette_rgb_i[71:48];
			default: palette_target_rgb = palette_rgb_i[95:72];
		endcase
	end

	wire [7:0] palette_red = palette_blend_channel_fn(
		palette_base_rgb[23:16], palette_target_rgb[23:16], combined_luma
	);
	wire [7:0] palette_green = palette_blend_channel_fn(
		palette_base_rgb[15:8], palette_target_rgb[15:8], combined_luma
	);
	wire [7:0] palette_blue = palette_blend_channel_fn(
		palette_base_rgb[7:0], palette_target_rgb[7:0], combined_luma
	);

	wire [7:0] natural_blue;

	vb_natural_red_tint u_natural_tint (
		.red_i  (combined_luma),
		.blue_o (natural_blue)
	);

	wire [7:0] multicolor_red;
	wire [7:0] multicolor_green;
	wire [7:0] multicolor_blue;

	vb_multicolor_luma_to_rgb u_multicolor (
		.raw_pixel_i (combined_raw),
		.luma_i      (combined_luma),
		.red_o       (multicolor_red),
		.green_o     (multicolor_green),
		.blue_o      (multicolor_blue)
	);

	// The refactored VIP publishes semantic left and right carriers directly.
	// Keep that ownership through every built-in stereo transform: the left eye
	// drives the red/left filter and the right eye drives the cyan/blue filter.
	wire [7:0] stereo_left_luma = left_luma_i;
	wire [7:0] stereo_right_luma = right_luma_i;
	wire [15:0] colorcode_left_red_sum =
		({8'd0, stereo_left_luma} << 7) +
		({8'd0, stereo_left_luma} << 5) +
		({8'd0, stereo_left_luma} << 4) +
		({8'd0, stereo_left_luma} << 2);
	wire [15:0] colorcode_left_green_sum =
		({8'd0, stereo_left_luma} << 7) +
		({8'd0, stereo_left_luma} << 4) +
		({8'd0, stereo_left_luma} << 3) +
		({8'd0, stereo_left_luma} << 1) +
		{8'd0, stereo_left_luma};
	wire [15:0] anaglyph_right_green_sum =
		({8'd0, stereo_right_luma} << 7) +
		({8'd0, stereo_right_luma} << 6) +
		({8'd0, stereo_right_luma} << 2) +
		({8'd0, stereo_right_luma} << 1);
	wire [15:0] anaglyph_right_blue_sum =
		({8'd0, stereo_right_luma} << 7) +
		({8'd0, stereo_right_luma} << 6) +
		({8'd0, stereo_right_luma} << 5) +
		({8'd0, stereo_right_luma} << 4);

	wire [7:0] trio_left_r;
	wire [7:0] trio_left_g;
	wire [7:0] trio_left_b;
	wire [7:0] trio_right_r;
	wire [7:0] trio_right_g;
	wire [7:0] trio_right_b;

	trioviz_left_luma_to_rgb u_trioviz_left (
		.luma (stereo_left_luma),
		.r    (trio_left_r),
		.g    (trio_left_g),
		.b    (trio_left_b)
	);

	trioviz_right_luma_to_rgb u_trioviz_right (
		.luma (stereo_right_luma),
		.r    (trio_right_r),
		.g    (trio_right_g),
		.b    (trio_right_b)
	);

	wire [23:0] custom_left_rgb = custom_eye_rgb_fn(left_color_i, left_luma_i);
	wire [23:0] custom_right_rgb = custom_eye_rgb_fn(right_color_i, right_luma_i);
	wire [7:0] custom_red =
		(custom_left_rgb[23:16] > custom_right_rgb[23:16]) ?
		custom_left_rgb[23:16] : custom_right_rgb[23:16];
	wire [7:0] custom_green =
		(custom_left_rgb[15:8] > custom_right_rgb[15:8]) ?
		custom_left_rgb[15:8] : custom_right_rgb[15:8];
	wire [7:0] custom_blue =
		(custom_left_rgb[7:0] > custom_right_rgb[7:0]) ?
		custom_left_rgb[7:0] : custom_right_rgb[7:0];

	always @(*) begin
		red_o = 8'd0;
		green_o = 8'd0;
		blue_o = 8'd0;

		if (video_active_i) begin
			case (color_mode_i)
				COLOR_MODE_MULTICOLOR: begin
					red_o = multicolor_red;
					green_o = multicolor_green;
					blue_o = multicolor_blue;
				end
				COLOR_MODE_PALETTE: begin
					red_o = palette_red;
					green_o = palette_green;
					blue_o = palette_blue;
				end
				COLOR_MODE_ANAGLYPH: begin
					if (dual_eye_draw_i) begin
						red_o = stereo_left_luma;
						green_o = anaglyph_right_green_sum[15:8];
						blue_o = anaglyph_right_blue_sum[15:8];
					end else begin
						red_o = combined_luma;
					end
				end
				COLOR_MODE_COLORCODE: begin
					if (dual_eye_draw_i) begin
						red_o = colorcode_left_red_sum[15:8];
						green_o = colorcode_left_green_sum[15:8];
						blue_o = stereo_right_luma;
					end else begin
						red_o = combined_luma;
					end
				end
				COLOR_MODE_TRIOVIZ: begin
					if (dual_eye_draw_i) begin
						red_o = (trio_left_r > trio_right_r) ? trio_left_r : trio_right_r;
						green_o = (trio_left_g > trio_right_g) ? trio_left_g : trio_right_g;
						blue_o = (trio_left_b > trio_right_b) ? trio_left_b : trio_right_b;
					end else begin
						red_o = combined_luma;
					end
				end
				COLOR_MODE_CUSTOM: begin
					red_o = custom_red;
					green_o = custom_green;
					blue_o = custom_blue;
				end
				COLOR_MODE_NATURAL: begin
					red_o = combined_luma;
					blue_o = natural_blue;
				end
				COLOR_MODE_PURE_RED: begin
					red_o = combined_luma;
				end
				default: begin
					red_o = combined_luma;
				end
			endcase
		end
	end

endmodule

module vb_multicolor_luma_to_rgb (
	input  wire [1:0] raw_pixel_i,
	input  wire [7:0] luma_i,
	output reg  [7:0] red_o,
	output reg  [7:0] green_o,
	output reg  [7:0] blue_o
);

	// Four fixed Red Viper-style hue families. The native luma remains the
	// intensity source, and all channel ratios are wiring-only binary fractions.
	always @(*) begin
		case (raw_pixel_i)
			2'd0: begin
				red_o = 8'd0;
				green_o = 8'd0;
				blue_o = 8'd0;
			end
			2'd1: begin
				red_o = {2'b00, luma_i[7:2]};
				green_o = {4'b0000, luma_i[7:4]};
				blue_o = luma_i;
			end
			2'd2: begin
				red_o = luma_i;
				green_o = {1'b0, luma_i[7:1]};
				blue_o = 8'd0;
			end
			default: begin
				red_o = luma_i;
				green_o = luma_i;
				blue_o = {3'b000, luma_i[7:3]};
			end
		endcase
	end

endmodule

// Faint magenta tint: blue carries 1.97% of red's linear light, with both
// channels sRGB encoded. Full red 255 pairs with blue 38.
module vb_natural_red_tint (
	input  wire [7:0] red_i,
	output reg  [7:0] blue_o
);

	always @(*) begin
		case (red_i)
			8'd0, 8'd1, 8'd2, 8'd3, 8'd4, 8'd5, 8'd6, 8'd7, 8'd8, 8'd9, 8'd10, 8'd11,
			8'd12, 8'd13, 8'd14, 8'd15, 8'd16, 8'd17, 8'd18, 8'd19, 8'd20, 8'd21: blue_o = 8'd0;
			8'd22, 8'd23, 8'd24, 8'd25, 8'd26, 8'd27, 8'd28, 8'd29, 8'd30, 8'd31,
			8'd32, 8'd33, 8'd34, 8'd35, 8'd36, 8'd37, 8'd38, 8'd39, 8'd40, 8'd41: blue_o = 8'd1;
			8'd42, 8'd43, 8'd44, 8'd45, 8'd46, 8'd47, 8'd48, 8'd49, 8'd50, 8'd51,
			8'd52, 8'd53, 8'd54, 8'd55: blue_o = 8'd2;
			8'd56, 8'd57, 8'd58, 8'd59, 8'd60, 8'd61, 8'd62, 8'd63, 8'd64, 8'd65: blue_o = 8'd3;
			8'd66, 8'd67, 8'd68, 8'd69, 8'd70, 8'd71, 8'd72, 8'd73, 8'd74: blue_o = 8'd4;
			8'd75, 8'd76, 8'd77, 8'd78, 8'd79, 8'd80, 8'd81, 8'd82: blue_o = 8'd5;
			8'd83, 8'd84, 8'd85, 8'd86, 8'd87, 8'd88, 8'd89: blue_o = 8'd6;
			8'd90, 8'd91, 8'd92, 8'd93, 8'd94, 8'd95: blue_o = 8'd7;
			8'd96, 8'd97, 8'd98, 8'd99, 8'd100, 8'd101: blue_o = 8'd8;
			8'd102, 8'd103, 8'd104, 8'd105, 8'd106: blue_o = 8'd9;
			8'd107, 8'd108, 8'd109, 8'd110, 8'd111, 8'd112: blue_o = 8'd10;
			8'd113, 8'd114, 8'd115, 8'd116, 8'd117: blue_o = 8'd11;
			8'd118, 8'd119, 8'd120, 8'd121, 8'd122: blue_o = 8'd12;
			8'd123, 8'd124, 8'd125, 8'd126, 8'd127: blue_o = 8'd13;
			8'd128, 8'd129, 8'd130, 8'd131, 8'd132: blue_o = 8'd14;
			8'd133, 8'd134, 8'd135, 8'd136, 8'd137: blue_o = 8'd15;
			8'd138, 8'd139, 8'd140, 8'd141, 8'd142: blue_o = 8'd16;
			8'd143, 8'd144, 8'd145, 8'd146, 8'd147: blue_o = 8'd17;
			8'd148, 8'd149, 8'd150, 8'd151, 8'd152, 8'd153: blue_o = 8'd18;
			8'd154, 8'd155, 8'd156, 8'd157, 8'd158: blue_o = 8'd19;
			8'd159, 8'd160, 8'd161, 8'd162, 8'd163: blue_o = 8'd20;
			8'd164, 8'd165, 8'd166, 8'd167, 8'd168: blue_o = 8'd21;
			8'd169, 8'd170, 8'd171, 8'd172, 8'd173: blue_o = 8'd22;
			8'd174, 8'd175, 8'd176, 8'd177, 8'd178: blue_o = 8'd23;
			8'd179, 8'd180, 8'd181, 8'd182, 8'd183: blue_o = 8'd24;
			8'd184, 8'd185, 8'd186, 8'd187, 8'd188, 8'd189: blue_o = 8'd25;
			8'd190, 8'd191, 8'd192, 8'd193, 8'd194: blue_o = 8'd26;
			8'd195, 8'd196, 8'd197, 8'd198, 8'd199: blue_o = 8'd27;
			8'd200, 8'd201, 8'd202, 8'd203, 8'd204: blue_o = 8'd28;
			8'd205, 8'd206, 8'd207, 8'd208, 8'd209: blue_o = 8'd29;
			8'd210, 8'd211, 8'd212, 8'd213, 8'd214: blue_o = 8'd30;
			8'd215, 8'd216, 8'd217, 8'd218, 8'd219: blue_o = 8'd31;
			8'd220, 8'd221, 8'd222, 8'd223, 8'd224, 8'd225: blue_o = 8'd32;
			8'd226, 8'd227, 8'd228, 8'd229, 8'd230: blue_o = 8'd33;
			8'd231, 8'd232, 8'd233, 8'd234, 8'd235: blue_o = 8'd34;
			8'd236, 8'd237, 8'd238, 8'd239, 8'd240: blue_o = 8'd35;
			8'd241, 8'd242, 8'd243, 8'd244, 8'd245: blue_o = 8'd36;
			8'd246, 8'd247, 8'd248, 8'd249, 8'd250: blue_o = 8'd37;
			default: blue_o = 8'd38;
		endcase
	end

endmodule

module trioviz_left_luma_to_rgb (
	input  wire [7:0] luma,
	output wire [7:0] r,
	output wire [7:0] g,
	output wire [7:0] b
);

	wire [15:0] r_sum =
		({8'd0, luma} << 7) +
		({8'd0, luma} << 5) +
		({8'd0, luma} << 4) +
		({8'd0, luma} << 3) +
		({8'd0, luma} << 2) +
		{8'd0, luma};
	wire [15:0] g_sum =
		({8'd0, luma} << 4) +
		({8'd0, luma} << 2) +
		({8'd0, luma} << 1) +
		{8'd0, luma};
	wire [15:0] b_sum =
		({8'd0, luma} << 7) +
		({8'd0, luma} << 5) +
		({8'd0, luma} << 3) +
		({8'd0, luma} << 1) +
		{8'd0, luma};

	assign r = r_sum[15:8];
	assign g = g_sum[15:8];
	assign b = b_sum[15:8];

endmodule

module trioviz_right_luma_to_rgb (
	input  wire [7:0] luma,
	output wire [7:0] r,
	output wire [7:0] g,
	output wire [7:0] b
);

	wire [15:0] rb_sum =
		({8'd0, luma} << 6) +
		({8'd0, luma} << 3) +
		{8'd0, luma};

	assign r = rb_sum[15:8];
	assign g = luma;
	assign b = rb_sum[15:8];

endmodule
