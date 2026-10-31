# SDRAM pin timing for the inverted forwarded clock and the DDIO DQ capture.
# Paired with rtl/Mem/sdram.sv (derived from Marty's controller).
#
# Integration requirements:
# - Read this file AFTER the core SDC that runs `derive_pll_clocks`, so the
#   controller clock already exists when the generated pin clock is derived.
#   `files.qip` lists it after VirtualBoy.sdc for that reason.
# - Set `sdram_ctrl_clk_pin` below to the PLL output pin driving the controller.
#   That single line is the only core-specific coupling in this file.

set sdram_ctrl_clk_pin {emu|pll|pll_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}
# The early copy of the controller clock that forwards SDRAM_CLK.
set sdram_pin_clk_pin {emu|pll|pll_inst|altera_pll_i|general[2].gpll~PLL_OUTPUT_COUNTER|divclk}

# Which half of the input DDIO cell the controller reads; must match the
# DQ_CAPTURE_RISING parameter (0 at 80 MHz CL2).
set sdram_dq_capture_rising 0
# The beat launched on a pin-clock edge is taken by the falling-edge cell
# register on the second internal falling edge after it: the first comes only
# 1.875 ns later, since the pin clock runs early.
set sdram_dq_capture_edges 2

set sdram_ctrl_clk [get_clocks $sdram_ctrl_clk_pin]
if {[get_collection_size $sdram_ctrl_clk] == 0} {
	post_message -type error "sdram.sdc: no controller clock matched the configured pin"
}

# SDRAM_CLK is an inverted copy of the early controller clock forwarded
# through the output DDIO cell (altddio_out, datain_h=0 / datain_l=1).
# Modeling it as a generated clock ON the pin accounts for the forward path's
# insertion delay, about 4.1 ns more than the controller's own clock tree.
# The 1.875 ns PLL phase takes most of that back, which centres read data on
# the capture edge; the commands still have the rest of the half period.
create_generated_clock -name sdram_clk_pin -invert \
	-source [get_pins $sdram_pin_clk_pin] \
	[get_ports {SDRAM_CLK}]

if {[get_collection_size [get_clocks sdram_clk_pin]] == 0} {
	post_message -type error "sdram.sdc: no SDRAM_CLK generated pin clock matched"
}

set sdram_out_ports [get_ports {SDRAM_CKE SDRAM_A[*] SDRAM_BA[*] SDRAM_DQML SDRAM_DQMH SDRAM_nCS SDRAM_nCAS SDRAM_nRAS SDRAM_nWE SDRAM_DQ[*]}]

# Outputs launch on the controller clock rising edge and are sampled by the
# SDRAM on the SDRAM_CLK edge a half period later. The inversion lives in the
# generated clock, so no -clock_fall qualifier is used here.
set_output_delay -clock sdram_clk_pin -max 1.500 $sdram_out_ports
set_output_delay -clock sdram_clk_pin -min -0.800 $sdram_out_ports

# DQ is launched by the SDRAM tAC after an SDRAM_CLK edge (6.0 ns max at CL2
# for the -7 part, 2.5 ns tOH), so the forwarded pin clock is the reference.
set_input_delay -clock sdram_clk_pin -max 6.000 [get_ports {SDRAM_DQ[*]}]
set_input_delay -clock sdram_clk_pin -min 2.500 [get_ports {SDRAM_DQ[*]}]

# The hold check stays on the falling edge in between, the one that would
# take a beat launched a period earlier.
set dq_capture_regs [get_registers {*u_sdram_dq_capture*~DFFLO}]
if {[get_collection_size $dq_capture_regs] != 16} {
	post_message -type error "sdram.sdc: expected 16 falling-edge DQ capture registers"
} else {
	set_multicycle_path -setup -end $sdram_dq_capture_edges -from [get_ports {SDRAM_DQ[*]}] -to $dq_capture_regs
}

# sdram.sv reads one half of the input DDIO cell; the other register still
# exists inside the hard cell, so exclude only that unused half.
set dq_unused_regs [get_registers [expr {$sdram_dq_capture_rising ? "*u_sdram_dq_capture*dataout_l*" : "*u_sdram_dq_capture*dataout_h*"}]]
if {[get_collection_size $dq_unused_regs] == 0} {
	set dq_unused_regs [get_registers [expr {$sdram_dq_capture_rising ? "*u_sdram_dq_capture*~DFFLO" : "*u_sdram_dq_capture*~DFFHI"}]]
}
if {[get_collection_size $dq_unused_regs] > 0} {
	set_false_path -to $dq_unused_regs
}
