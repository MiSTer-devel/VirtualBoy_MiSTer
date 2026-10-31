derive_pll_clocks
derive_clock_uncertainty

# Core-specific constraints.

# clk_ram (80 MHz) is 2x clk_sys (40 MHz) from one PLL, so every clk_sys edge
# is a clk_ram edge. The cartridge path crosses with plain related-clock
# paths: CPU bus registers feed the controller's first-sight ACTIVE on the
# next clk_ram edge, and the controller's read registers feed 40 MHz latches.
# No crossing needs an exception.

# SDRAM pin timing (the generated SDRAM_CLK pin clock, command/address/write
# launch, and DDIO DQ capture) lives with the controller in rtl/Mem/sdram.sdc,
# which files.qip reads after this file. Only the wrapper-to-core crossings
# below belong to the integration.


# CPU clock-enable multicycle.
# The V810 runs on a clock enable: core_ce is ~ce_div (a clk_sys toggle) ANDed
# only with run/pause gates, so it never fires on two clk_sys edges in a row.
# A CPU register written only on core_ce holds for at least two clk_sys
# periods, and one read only on core_ce samples once per two, so a path
# between two such registers has two clk_sys periods. -hold 1 keeps the hold
# check on the launch edge.
# Left at one period:
# - registers also written on phi1 (the clk_sys after core_ce) or on every
#   clk_sys: READY/SZRQ sampling and the idle-T1 pin registers in the bus unit,
#   the multiplier digit registers, the register file read registers;
# - block RAMs, and the debug retire pulse;
# - every path that starts or ends outside the CPU (devices, cartridge,
#   savestate, hps_io).
# Savestate restore writes CPU registers between core_ce edges, but only while
# savestate_pause_active_q holds core_ce off; the pause is released from a
# register at the next frame start, well after the last write.
set cpu_ce_regs [get_registers -nowarn {*|NECv810:u_cpu|*}]
set cpu_fast_regs [get_registers -nowarn {
	*|NECv810:u_cpu|necv810_regfile:u_gpr|*
	*|NECv810:u_cpu|necv810_integer_engine:u_int|addend_q*
	*|NECv810:u_cpu|necv810_integer_engine:u_int|multiplicand3_q*
	*|NECv810:u_cpu|necv810_bus_engine:u_bus|ready_s_q*
	*|NECv810:u_cpu|necv810_bus_engine:u_bus|szrq_s_q*
	*|NECv810:u_cpu|necv810_bus_engine:u_bus|hldrq_s_q*
	*|NECv810:u_cpu|hold_data_q*
	*|NECv810:u_cpu|necv810_bus_engine:u_bus|early_*
	*|NECv810:u_cpu|trace_valid_o*
	*|NECv810:u_cpu|*altsyncram*
}]
set cpu_ce_regs [remove_from_collection $cpu_ce_regs $cpu_fast_regs]

if {[get_collection_size $cpu_ce_regs] == 0} {
	post_message -type error "No CPU clock-enable registers matched"
} else {
	post_message -type info "CPU clock-enable multicycle: [get_collection_size $cpu_ce_regs] registers, [get_collection_size $cpu_fast_regs] left at one period"
	set_multicycle_path -setup 2 -from $cpu_ce_regs -to $cpu_ce_regs
	set_multicycle_path -hold 1 -from $cpu_ce_regs -to $cpu_ce_regs
}
