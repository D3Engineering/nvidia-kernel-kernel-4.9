# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the fpga framework and fpga manager drivers.
#

# Core FPGA Manager Framework
obj-$(CONFIG_FPGA)			+= fpga-mgr.o

# FPGA Manager Drivers
obj-$(CONFIG_FPGA_MGR_MACHXO2_SPI)	+= machxo2-spi.o
obj-$(CONFIG_FPGA_MGR_MACHXO2_I2C)	+= machxo2-i2c.o
obj-$(CONFIG_FPGA_MGR_SOCFPGA)		+= socfpga.o
obj-$(CONFIG_FPGA_MGR_ZYNQ_FPGA)	+= zynq-fpga.o

# FPGA Bridge Drivers
obj-$(CONFIG_FPGA_BRIDGE)		+= fpga-bridge.o

# High Level Interfaces
obj-$(CONFIG_FPGA_REGION)		+= fpga-region.o
obj-$(CONFIG_OF_FPGA_REGION)		+= of-fpga-region.o
