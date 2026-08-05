# SPDX-FileCopyrightText: 2026 Maciej Wilczyński <m@lupin.pl>
# SPDX-License-Identifier: AGPL-3.0-or-later
# Host test suite for pcan_msg / isotp / uds.
#
# Compiles the REAL firmware sources from ../../main against the minimal
# ESP-IDF stub headers in stubs/, so the tests cannot drift from the code they
# cover. No ESP-IDF, no toolchain, no test framework required:
#
#     make            build and run
#     make clean
#     make SAN=0      build without the sanitizers
#
# This directory is invisible to the ESP-IDF build: main/CMakeLists.txt lists
# its SRCS explicitly and idf.py only scans main/ and components/.

CC      ?= cc
MAIN    := ../../main
BUILD   := build
BIN     := $(BUILD)/run_tests

WARN    := -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-align \
           -Wstrict-prototypes -Wmissing-prototypes -Wno-unused-parameter

# gnu99, not c99: the fuzz harness places its buffers against an mmap'd guard
# page, and strict-ISO mode hides mmap/MAP_ANON behind feature-test macros that
# differ between glibc and Darwin. The firmware itself is still built as C99 by
# ESP-IDF; this only affects the test build.
CFLAGS  := -std=gnu99 -O1 -g $(WARN) -I. -I$(MAIN) -Istubs
LDFLAGS :=

# Sanitizers, if the toolchain can both build and RUN them (sanprobe.sh checks
# the second part too — some hosts ship an ASan runtime that hangs on start).
SAN ?= 1
ifeq ($(SAN),1)
SAN_FLAGS := $(shell [ -f $(BUILD)/sanflags ] || { mkdir -p $(BUILD); \
                CC='$(CC)' /bin/sh sanprobe.sh > $(BUILD)/sanflags; }; \
                cat $(BUILD)/sanflags)
ifneq ($(strip $(SAN_FLAGS)),)
CFLAGS  += $(SAN_FLAGS) -fno-omit-frame-pointer -fno-sanitize-recover=undefined
LDFLAGS += $(SAN_FLAGS)
endif
endif

# The real modules under test, plus the harness and stubs.
SRCS := \
    $(MAIN)/pcan_msg.c \
    $(MAIN)/pcan_time.c \
    $(MAIN)/isotp.c \
    $(MAIN)/uds.c \
    stubs/esp_stubs.c \
    test_main.c \
    test_pcan_msg.c \
    test_isotp.c \
    test_uds.c

OBJS := $(addprefix $(BUILD)/,$(notdir $(SRCS:.c=.o)))
vpath %.c . stubs $(MAIN)

.PHONY: all test clean
all: test

test: $(BIN)
	@echo "sanitizers: $(if $(strip $(SAN_FLAGS)),$(SAN_FLAGS),none)"
	@echo
	$(BIN)

$(BIN): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
