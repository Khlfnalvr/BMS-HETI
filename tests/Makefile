# ============================================================
# Makefile for BMS-HETI unit tests
# Usage:
#   make          — build and run all tests
#   make build    — compile only
#   make run      — run the existing binary
#   make clean    — remove build artefacts
# ============================================================

CC      = gcc
CFLAGS  = -Wall -Wextra -std=c99 -g \
          -I.. \
          -DUNIT_TEST
LDFLAGS = -lm

SRC_DIR = ..
OBJ_DIR = obj

# Source files under test (no Main.c — it depends on STM32 HAL)
SRC = \
    $(SRC_DIR)/CC.c \
    $(SRC_DIR)/bateryparamter.c \
    $(SRC_DIR)/AUKF.c \
    $(SRC_DIR)/SOCEstimator.c

# Test sources
TEST_SRC = \
    unity.c \
    test_cc.c \
    test_battery_params.c \
    test_aukf.c \
    test_soc_estimator.c \
    test_main.c

ALL_SRC  = $(SRC) $(TEST_SRC)
ALL_OBJ  = $(patsubst %.c, $(OBJ_DIR)/%.o, $(notdir $(ALL_SRC)))

TARGET = $(OBJ_DIR)/test_runner

# ---- Rules --------------------------------------------------

.PHONY: all build run clean

all: build run

build: $(OBJ_DIR) $(TARGET)

run: $(TARGET)
	@echo ""
	./$(TARGET)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(TARGET): $(ALL_OBJ)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

# Compile sources from the parent directory
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Compile test sources from the current directory
$(OBJ_DIR)/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR)
