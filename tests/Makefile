# Makefile for BMS-HETI host-side unit tests
#
# Builds each test suite as a standalone executable and runs them all.
# No STM32 toolchain required; uses the host gcc.
#
# Usage:
#   make          - build and run all tests
#   make build    - build only
#   make run      - run already-built tests
#   make clean    - remove build artefacts and symlinks

CC      := gcc
CFLAGS  := -std=c99 -Wall -Wextra -g -O0 -lm
SRC_DIR := ..
BUILD   := build

# Source files that are compiled into every test binary
COMMON_SRC := \
    $(SRC_DIR)/bateryparamter.c \
    $(SRC_DIR)/CC.c             \
    $(SRC_DIR)/AUKF.c           \
    $(SRC_DIR)/SOCEstimator.c

# Test binaries
TESTS := \
    $(BUILD)/test_cc            \
    $(BUILD)/test_battery_params \
    $(BUILD)/test_aukf          \
    $(BUILD)/test_soc_estimator

# ── default target ────────────────────────────────────────────────────
.PHONY: all build run clean

all: build run

build: symlinks $(TESTS)

# ── symlinks ──────────────────────────────────────────────────────────
# Source files #include lowercase names (cc.h, aukf.h, socestimator.h)
# but the actual files have mixed case.  Create symlinks so gcc finds them.
.PHONY: symlinks
symlinks:
	@mkdir -p $(BUILD)
	@[ -f $(SRC_DIR)/cc.h           ] || ln -sf CC.h           $(SRC_DIR)/cc.h
	@[ -f $(SRC_DIR)/aukf.h         ] || ln -sf AUKF.h         $(SRC_DIR)/aukf.h
	@[ -f $(SRC_DIR)/socestimator.h ] || ln -sf SOCEstimator.h $(SRC_DIR)/socestimator.h
	@[ -f $(SRC_DIR)/bateryparameter.h ] || ln -sf Bateryparameter.h $(SRC_DIR)/bateryparameter.h

# ── individual test binaries ──────────────────────────────────────────
$(BUILD)/test_cc: test_cc.c $(SRC_DIR)/CC.c | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -I. $^ -o $@ -lm

$(BUILD)/test_battery_params: test_battery_params.c $(SRC_DIR)/bateryparamter.c | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -I. $^ -o $@ -lm

$(BUILD)/test_aukf: test_aukf.c $(COMMON_SRC) | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -I. $^ -o $@ -lm

$(BUILD)/test_soc_estimator: test_soc_estimator.c $(COMMON_SRC) | $(BUILD)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -I. $^ -o $@ -lm

$(BUILD):
	mkdir -p $(BUILD)

# ── run ──────────────────────────────────────────────────────────────
run: $(TESTS)
	@echo ""
	@FAILED=0; \
	for t in $(TESTS); do \
	    echo "Running $$t ..."; \
	    $$t; \
	    if [ $$? -ne 0 ]; then FAILED=$$((FAILED+1)); fi; \
	    echo ""; \
	done; \
	if [ $$FAILED -eq 0 ]; then \
	    echo "All test suites passed."; \
	else \
	    echo "$$FAILED test suite(s) FAILED."; \
	    exit 1; \
	fi

# ── clean ─────────────────────────────────────────────────────────────
clean:
	rm -rf $(BUILD)
	rm -f $(SRC_DIR)/cc.h \
	      $(SRC_DIR)/aukf.h \
	      $(SRC_DIR)/socestimator.h \
	      $(SRC_DIR)/bateryparameter.h
