SHELL := /bin/bash

# --- Xilinx installation info ----
XILINX_ROOT    ?= /opt/Xilinx
XILINX_VERSION ?= 2024.2
export XILINX_ROOT
export XILINX_VERSION

# ---- Project name, passed from CLI ----
PROJECT ?= unset

ifeq ($(PROJECT),unset)
$(error PROJECT not set, please specify the name of the target project)
endif

# Plnx project for boot creation
PLNX_PROJECT ?=

# ---- Project specific, change them for your board ----
NJOBS      ?= 8
PART_NAME  ?= xczu7ev-ffvc1156-2-e
BOARD_NAME ?= xilinx.com:zcu104:part0:1.1
BOARD_DTS  ?= zcu104-revc

# ---- Build/Script Dirs ----
PROJECT_ROOT ?= $(CURDIR)
SCRIPTS_DIR ?= $(PROJECT_ROOT)/scripts
BUILD_DIR := $(PROJECT_ROOT)/build/$(PROJECT)
MB_DIR := $(PROJECT_ROOT)/microblaze

# ---- Run commands, using scripts that setup the env ----
RUN_VITIS  := $(SCRIPTS_DIR)/env/run_vitis.sh
RUN_VIVADO := $(SCRIPTS_DIR)/env/run_vivado.sh
RUN_XSCT   := $(SCRIPTS_DIR)/env/run_xsct.sh


.PHONY: new project synth impl bit xsa overlay

default: help

help:
	@echo vivgit main interface, available commands:

# -- shell
new:
	cp $(SCRIPTS_DIR)/bootstrap/templates/zynq_ultra.tcl bd/$(PROJECT).tcl
	sed -i 's/base_zynq/$(PROJECT)/g' bd/$(PROJECT).tcl

# -- vivado
project:
	PROJECT=$(PROJECT) PROJECT_ROOT=$(PROJECT_ROOT) PART_NAME=$(PART_NAME) BOARD_NAME=$(BOARD_NAME) \
	$(RUN_VIVADO) $(SCRIPTS_DIR)/vivado/create_project.tcl

synth: project
	PROJECT=$(PROJECT) PROJECT_ROOT=$(PROJECT_ROOT) \
	$(RUN_VIVADO) $(SCRIPTS_DIR)/vivado/run_synth_impl.tcl

update:
	PROJECT=$(PROJECT) PROJECT_ROOT=$(PROJECT_ROOT) \
	$(RUN_VIVADO) $(SCRIPTS_DIR)/vivado/update_bd.tcl

# -- xsct
sdt:
	PROJECT=$(PROJECT) PROJECT_ROOT=$(PROJECT_ROOT) BOARD_DTS=$(BOARD_DTS) \
	$(RUN_XSCT) $(SCRIPTS_DIR)/xsct/generate_sdt.tcl

boot:
	PROJECT=$(PROJECT) PROJECT_ROOT=$(PROJECT_ROOT) PLNX_PROJECT=$(PLNX_PROJECT) \
	$(RUN_XSCT) $(SCRIPTS_DIR)/xsct/generate_boot.tcl

# -- vitis
bsp:
	PROJECT=$(PROJECT) PROJECT_ROOT=$(PROJECT_ROOT) \
	$(RUN_VITIS) $(SCRIPTS_DIR)/vitis/generate_bsp.py

# -- others
overlay: sdt
	PROJECT=$(PROJECT) PROJECT_ROOT=$(PROJECT_ROOT) \
	python $(SCRIPTS_DIR)/helpers/generate_overlay.py

dtbo:
	dtc -@ -I dts -O dtb -o $(BUILD_DIR)/$(PROJECT).dtbo $(BUILD_DIR)/$(PROJECT)_overlay.dtsi
# Note: dtbo does not have overlay as a dependency since the user might want to modify it themself

# -- microblaze
mb:
	@mkdir $(MB_DIR)/$(MB_PROJECT)
	@cp $(SCRIPTS_DIR)/bootstrap/Makefile.microblaze $(MB_DIR)/$(MB_PROJECT)/Makefile

elf-mb:
	PROJECT=$(PROJECT) PROJECT_ROOT=$(PROJECT_ROOT) MB_PROJECT=$(MB_PROJECT) \
	$(MAKE) -C $(MB_DIR)/$(MB_PROJECT)

clean-mb:
	PROJECT=$(PROJECT) PROJECT_ROOT=$(PROJECT_ROOT) MB_PROJECT=$(MB_PROJECT) \
	$(MAKE) -C $(MB_DIR)/$(MB_PROJECT) clean
