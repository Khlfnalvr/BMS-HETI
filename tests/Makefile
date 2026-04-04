# ============================================================
# Makefile for BMS-HETI unit tests
#
# Usage:
#   make          – build and run all tests
#   make build    – build only
#   make test     – run existing binary
#   make clean    – remove build artefacts and symlinks
#
# Note: SOCEstimator.c uses lowercase includes ("socestimator.h",
# "bateryparameter.h"). Two symlinks are created so that the
# case-sensitive Linux linker finds the real headers.
# ============================================================

CC      = gcc
CFLAGS  = -Wall -Wextra -g -I.. -I.
LDFLAGS = -lm

# Source files under test (from parent directory)
SRC_OBJS = \
    ../CC.c \
    ../AUKF.c \
    ../bateryparamter.c \
    ../SOCEstimator.c

# Test source files
TEST_SRCS = \
    test_battery_params.c \
    test_cc.c \
    test_aukf.c \
    test_soc_estimator.c \
    test_main.c

TARGET = test_runner

# Symlinks that fix case-sensitive include mismatches
SYMLINKS = socestimator.h bateryparameter.h cc.h aukf.h

# ── Targets ─────────────────────────────────────────────────

.PHONY: all build test clean

all: build test

build: $(SYMLINKS) $(TARGET)

$(TARGET): $(SRC_OBJS) $(TEST_SRCS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test: $(TARGET)
	./$(TARGET)

# Create lowercase symlinks so case-sensitive includes resolve
socestimator.h:
	ln -sf ../SOCEstimator.h socestimator.h

bateryparameter.h:
	ln -sf ../Bateryparameter.h bateryparameter.h

cc.h:
	ln -sf ../CC.h cc.h

aukf.h:
	ln -sf ../AUKF.h aukf.h

clean:
	rm -f $(TARGET) $(SYMLINKS)
