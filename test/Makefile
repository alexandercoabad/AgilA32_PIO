# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info

# defaults
SIM ?= icarus
FST ?= -fst # Use more efficient FST format
TOPLEVEL_LANG ?= verilog
SRC_DIR = $(PWD)/../src
PROJECT_SOURCES = tt_um_agila32.v rv32i_core.v mem.v qspi_shared_engine.v pio.v pio_sm.v pio_fifo.v

ifneq ($(GATES),yes)

# RTL simulation:
SIM_BUILD				= sim_build/rtl
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))

else

# Gate level simulation:
SIM_BUILD				= sim_build/gl
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DSIM
# PDK / PDK_ROOT are set by the GL-test action from the GDS run's pdk.json, so this works
# for both ihp-sg13g2 (libs sg13g2_*) and ihp-sg13cmos5l (libs sg13cmos5l_*).
PDK ?= ihp-sg13g2
PDK_LIB = $(patsubst ihp-%,%,$(PDK))
VERILOG_SOURCES += $(PDK_ROOT)/$(PDK)/libs.ref/$(PDK_LIB)_io/verilog/$(PDK_LIB)_io.v
VERILOG_SOURCES += $(PDK_ROOT)/$(PDK)/libs.ref/$(PDK_LIB)_stdcell/verilog/$(PDK_LIB)_stdcell.v

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

endif

# Allow sharing configuration between design and testbench via `include`:
COMPILE_ARGS 		+= -I$(SRC_DIR)

# Include the testbench sources:
VERILOG_SOURCES += $(PWD)/tb.v
TOPLEVEL = tb

# List test modules to run, separated by commas and without the .py suffix:
COCOTB_TEST_MODULES = test

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim

# ---------------------------------------------------------------------
# Standalone (non-cocotb) Icarus testbenches -- cover the QSPI
# external-memory addition, the reprogrammable boot ROM, the
# FLASH_MODE handoff, FLASH_PAGE bank-switched flash execution
# (including the real ST7789 driver), the Timer/PWM peripheral, the
# EBREAK/halted-status PIN_MUX mode, the boot ROM's timeout ->
# automatic flash fallback, the variable SPI clock divider, and the
# generic SPI peripheral (CS2). See test/README.md's "Additional
# standalone testbenches" section for what each one does.
# ---------------------------------------------------------------------
.PHONY: standalone-tests
standalone-tests:
	iverilog -g2012 -o /tmp/tt_tb_qspi_engine.vvp $(SRC_DIR)/qspi_shared_engine.v tb_qspi_engine.v
	vvp /tmp/tt_tb_qspi_engine.vvp | tee /tmp/tt_out_qspi_engine.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_qspi_clkdiv.vvp $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_qspi_clkdiv.v
	vvp /tmp/tt_tb_qspi_clkdiv.vvp | tee /tmp/tt_out_qspi_clkdiv.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_spi_periph.vvp $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v tb_spi_periph.v
	vvp /tmp/tt_tb_spi_periph.vvp | tee /tmp/tt_out_spi_periph.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_mem_ext.vvp $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_mem_ext.v
	vvp /tmp/tt_tb_mem_ext.vvp | tee /tmp/tt_out_mem_ext.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_core_ext.vvp $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/qspi_shared_engine.v mem_extmem_test.v spi_ram_model.v tb_core_ext.v
	vvp /tmp/tt_tb_core_ext.vvp | tee /tmp/tt_out_core_ext.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_check.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_check.v
	vvp /tmp/tt_tb_check.vvp | tee /tmp/tt_out_check.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_flash_handoff.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_flash_handoff.v
	vvp /tmp/tt_tb_flash_handoff.vvp | tee /tmp/tt_out_flash_handoff.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_flash_paging.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_flash_paging.v
	vvp /tmp/tt_tb_flash_paging.vvp | tee /tmp/tt_out_flash_paging.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_st7789_driver.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_st7789_driver.v
	vvp /tmp/tt_tb_st7789_driver.vvp | tee /tmp/tt_out_st7789_driver.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_ps2_reader.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_ps2_reader.v
	vvp /tmp/tt_tb_ps2_reader.vvp | tee /tmp/tt_out_ps2_reader.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_ps2_ascii.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_ps2_ascii.v
	vvp /tmp/tt_tb_ps2_ascii.vvp | tee /tmp/tt_out_ps2_ascii.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_alu_test.vvp $(SRC_DIR)/rv32i_core.v alu_test_mem.v tb_alu_test.v
	vvp /tmp/tt_tb_alu_test.vvp | tee /tmp/tt_out_alu_test.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_timer_pwm.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v tb_timer_pwm.v
	vvp /tmp/tt_tb_timer_pwm.vvp | tee /tmp/tt_out_timer_pwm.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_ebreak_halt.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_ebreak_halt.v
	vvp /tmp/tt_tb_ebreak_halt.vvp | tee /tmp/tt_out_ebreak_halt.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_boot_timeout.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_boot_timeout.v
	vvp /tmp/tt_tb_boot_timeout.vvp | tee /tmp/tt_out_boot_timeout.log
	python3 ../tools/pioasm.py ../pio/uart_tx.pio --format hex > /tmp/pio_uart_tx.hex
	python3 ../tools/pioasm.py ../pio/uart_rx.pio --origin 4 --format hex > /tmp/pio_uart_rx.hex
	python3 ../tools/pioasm.py ../pio/spi_master.pio --format hex > /tmp/pio_spi.hex
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_pio_isa.vvp $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v tb_pio_isa.v
	vvp /tmp/tt_tb_pio_isa.vvp | tee /tmp/tt_out_pio_isa.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_pio_uart.vvp $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v tb_pio_uart.v
	vvp /tmp/tt_tb_pio_uart.vvp | tee /tmp/tt_out_pio_uart.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_pio_spi.vvp $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v tb_pio_spi.v
	vvp /tmp/tt_tb_pio_spi.vvp | tee /tmp/tt_out_pio_spi.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_pio_cpu_uart.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_pio_cpu_uart.v
	vvp /tmp/tt_tb_pio_cpu_uart.vvp | tee /tmp/tt_out_pio_cpu_uart.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_pio_cpu_i2c.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_pio_cpu_i2c.v
	vvp /tmp/tt_tb_pio_cpu_i2c.vvp | tee /tmp/tt_out_pio_cpu_i2c.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_pio_cpu_multi.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_pio_cpu_multi.v
	vvp /tmp/tt_tb_pio_cpu_multi.vvp | tee /tmp/tt_out_pio_cpu_multi.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_pio_cpu_spi4.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_pio_cpu_spi4.v
	vvp /tmp/tt_tb_pio_cpu_spi4.vvp | tee /tmp/tt_out_pio_cpu_spi4.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_pio_cpu_jtag.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_pio_cpu_jtag.v
	vvp /tmp/tt_tb_pio_cpu_jtag.vvp | tee /tmp/tt_out_pio_cpu_jtag.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_pio_cpu_usb.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_pio_cpu_usb.v
	vvp /tmp/tt_tb_pio_cpu_usb.vvp | tee /tmp/tt_out_pio_cpu_usb.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_pio_cpu_i2c_slave.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_pio_cpu_i2c_slave.v
	vvp /tmp/tt_tb_pio_cpu_i2c_slave.vvp | tee /tmp/tt_out_pio_cpu_i2c_slave.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_pio_cpu_ps2.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_pio_cpu_ps2.v
	vvp /tmp/tt_tb_pio_cpu_ps2.vvp | tee /tmp/tt_out_pio_cpu_ps2.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_pio_cpu_ws2812.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_pio_cpu_ws2812.v
	vvp /tmp/tt_tb_pio_cpu_ws2812.vvp | tee /tmp/tt_out_pio_cpu_ws2812.log
	iverilog -g2012 -I $(SRC_DIR) -o /tmp/tt_tb_pio_cpu_onewire.vvp $(SRC_DIR)/tt_um_agila32.v $(SRC_DIR)/pio.v $(SRC_DIR)/pio_sm.v $(SRC_DIR)/pio_fifo.v $(SRC_DIR)/rv32i_core.v $(SRC_DIR)/mem.v $(SRC_DIR)/qspi_shared_engine.v spi_ram_model.v tb_pio_cpu_onewire.v
	vvp /tmp/tt_tb_pio_cpu_onewire.vvp | tee /tmp/tt_out_pio_cpu_onewire.log
	$(MAKE) -f Makefile.proto
	@! grep -l "FAIL" /tmp/tt_out_*.log
