# =============================================================================
# BMS-HETI Unit Test Makefile
#
# Usage:
#   make          – build all test binaries
#   make run      – build and run all tests
#   make clean    – remove build artefacts and compat symlinks
#
# The source files contain #include directives with lowercase/mixed-case names
# that don't match the actual filenames on a case-sensitive filesystem.
# The 'compat/' directory provides symlinks with the expected names so that
# GCC can resolve them via -I./compat without touching the source tree.
# =============================================================================

CC      = gcc
CFLAGS  = -Wall -Wextra -g -I.. -I./compat
LDFLAGS = -lm

ROOT  = ..
SRCS_CC    = $(ROOT)/CC.c
SRCS_BP    = $(ROOT)/bateryparamter.c
SRCS_AUKF  = $(ROOT)/AUKF.c
SRCS_SOC   = $(ROOT)/SOCEstimator.c

TESTS = test_cc test_battery_params test_aukf test_soc_estimator

.PHONY: all run clean compat

# ── Default target ────────────────────────────────────────────────────────────
all: compat $(TESTS)

# ── Compatibility symlinks ────────────────────────────────────────────────────
# Source files use lowercase/different-case includes; resolve them here.
compat:
	@mkdir -p compat
	@ln -sf ../../CC.h            compat/cc.h
	@ln -sf ../../AUKF.h          compat/aukf.h
	@ln -sf ../../SOCEstimator.h  compat/socestimator.h
	@ln -sf ../../Bateryparameter.h compat/bateryparameter.h

# ── Test binaries ─────────────────────────────────────────────────────────────
test_cc: test_cc.c $(SRCS_CC) test_framework.h
	$(CC) $(CFLAGS) -o $@ test_cc.c $(SRCS_CC) $(LDFLAGS)

test_battery_params: test_battery_params.c $(SRCS_BP) test_framework.h
	$(CC) $(CFLAGS) -o $@ test_battery_params.c $(SRCS_BP) $(LDFLAGS)

test_aukf: test_aukf.c $(SRCS_AUKF) $(SRCS_BP) test_framework.h
	$(CC) $(CFLAGS) -o $@ test_aukf.c $(SRCS_AUKF) $(SRCS_BP) $(LDFLAGS)

test_soc_estimator: test_soc_estimator.c $(SRCS_SOC) $(SRCS_CC) $(SRCS_AUKF) $(SRCS_BP) test_framework.h
	$(CC) $(CFLAGS) -o $@ test_soc_estimator.c $(SRCS_SOC) $(SRCS_CC) $(SRCS_AUKF) $(SRCS_BP) $(LDFLAGS)

# ── Run all tests ─────────────────────────────────────────────────────────────
run: all
	@echo ""
	@echo "Running all test suites..."
	@echo "──────────────────────────────────────────────────"
	@./test_cc
	@./test_battery_params
	@./test_aukf
	@./test_soc_estimator
	@echo "──────────────────────────────────────────────────"
	@echo "All suites passed."

# ── Clean ─────────────────────────────────────────────────────────────────────
clean:
	rm -f $(TESTS)
	rm -rf compat
