# SPDX-License-Identifier: GPL-2.0-or-later
#
# Host-side model tests for src/selfboot.c (test.c): where the firmware image, the boot
# arguments and the boot disk go in guest memory, what is checked of the two files, the boot
# arguments and the device tree the firmware is handed; and for src/selfboot_run.c
# (run_test.c): reading the two files, setting up the hypervisor and entering the firmware.
# The real files, compiled for the build machine against stand-in headers, with device trees
# that the tests make.
#
#   make        build them and run them; then check_adt.py reads the device tree test.c wrote
#               with the parser of the proxy client (needs the Python of the proxy client)
#   make run    only the one for src/selfboot_run.c

CC := gcc
PYTHON ?= $(HOME)/src/m1n1-venv/bin/python
SRC := ../../src
CFLAGS := -O2 -g -Wall -Werror -I. -Ibuild

.PHONY: all run clean

all: build/model run
	build/model build/adt-out.bin
	$(PYTHON) check_adt.py build/adt-out.bin

run: build/run-model
	build/run-model

# Copies, so that their #include "..." lines find the stand-ins in this directory
build/selfboot.c: $(SRC)/selfboot.c
	@mkdir -p build
	cp $< $@

build/selfboot.h: $(SRC)/selfboot.h
	@mkdir -p build
	cp $< $@

build/adt.h: $(SRC)/adt.h
	@mkdir -p build
	cp $< $@

build/xnuboot.h: $(SRC)/xnuboot.h
	@mkdir -p build
	cp $< $@

build/model: test.c model.c $(wildcard *.h) build/selfboot.c build/selfboot.h build/adt.h build/xnuboot.h
	$(CC) $(CFLAGS) -o $@ test.c model.c build/selfboot.c

build/selfboot_run.c: $(SRC)/selfboot_run.c
	@mkdir -p build
	cp $< $@

build/mcc.h: $(SRC)/mcc.h
	@mkdir -p build
	cp $< $@

build/arm_cpu_regs.h: $(SRC)/arm_cpu_regs.h
	@mkdir -p build
	cp $< $@

# selfboot_run.c includes "../build/build_cfg.h": from build/, that is this one
build/build_cfg.h:
	@mkdir -p build
	echo "#define CHAINLOADING" > $@

build/selfboot_files.c: $(SRC)/selfboot_files.c
	@mkdir -p build
	cp $< $@

RUN_COPIES := build/selfboot_files.c build/selfboot.c build/selfboot_run.c build/selfboot.h build/adt.h build/xnuboot.h
RUN_COPIES += build/mcc.h build/arm_cpu_regs.h build/build_cfg.h

build/run-model: run_test.c model.c $(wildcard *.h) $(RUN_COPIES)
	$(CC) $(CFLAGS) -o $@ run_test.c model.c build/selfboot.c build/selfboot_files.c build/selfboot_run.c

clean:
	rm -rf build
