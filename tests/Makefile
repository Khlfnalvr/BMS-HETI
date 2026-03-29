# Makefile for BMS-HETI unit tests
# Run from the tests/ directory: make && make run

CC      = gcc
CFLAGS  = -Wall -Wextra -std=c99 -I.. -lm
SRC_DIR = ..

# Source files (host-compilable modules only — no STM32 HAL)
SRCS_CC   = $(SRC_DIR)/CC.c
SRCS_BP   = $(SRC_DIR)/bateryparamter.c
SRCS_AUKF = $(SRC_DIR)/AUKF.c $(SRC_DIR)/bateryparamter.c
SRCS_EST  = $(SRC_DIR)/SOCEstimator.c $(SRC_DIR)/AUKF.c \
            $(SRC_DIR)/CC.c $(SRC_DIR)/bateryparamter.c

TESTS = test_cc test_battery_params test_aukf test_soc_estimator

.PHONY: all run clean

all: fix-case $(TESTS)

# Source files use lowercase includes but some headers have uppercase names.
# Create symlinks to satisfy the case-sensitive Linux filesystem.
fix-case:
	@cd .. && \
	for pair in "CC.h:cc.h" "AUKF.h:aukf.h" "SOCEstimator.h:socestimator.h" \
	            "Bateryparameter.h:bateryparameter.h"; do \
		src=$$(echo $$pair | cut -d: -f1); \
		dst=$$(echo $$pair | cut -d: -f2); \
		if [ ! -f $$dst ]; then \
			ln -sf $$src $$dst; \
			echo "Created symlink: $$dst -> $$src"; \
		fi; \
	done

test_cc: test_cc.c $(SRCS_CC) test_framework.h
	$(CC) $(CFLAGS) -o $@ test_cc.c $(SRCS_CC) -lm

test_battery_params: test_battery_params.c $(SRCS_BP) test_framework.h
	$(CC) $(CFLAGS) -o $@ test_battery_params.c $(SRCS_BP) -lm

test_aukf: test_aukf.c $(SRCS_AUKF) test_framework.h
	$(CC) $(CFLAGS) -o $@ test_aukf.c $(SRCS_AUKF) -lm

test_soc_estimator: test_soc_estimator.c $(SRCS_EST) test_framework.h
	$(CC) $(CFLAGS) -o $@ test_soc_estimator.c $(SRCS_EST) -lm

run: all
	@echo ""
	@echo "##################################################"
	@echo "# Running: test_cc"
	@echo "##################################################"
	@./test_cc; echo "Exit: $$?"
	@echo ""
	@echo "##################################################"
	@echo "# Running: test_battery_params"
	@echo "##################################################"
	@./test_battery_params; echo "Exit: $$?"
	@echo ""
	@echo "##################################################"
	@echo "# Running: test_aukf"
	@echo "##################################################"
	@./test_aukf; echo "Exit: $$?"
	@echo ""
	@echo "##################################################"
	@echo "# Running: test_soc_estimator"
	@echo "##################################################"
	@./test_soc_estimator; echo "Exit: $$?"

clean:
	rm -f $(TESTS)
	@cd .. && for dst in cc.h aukf.h socestimator.h bateryparameter.h; do \
		if [ -L $$dst ]; then \
			rm $$dst; \
			echo "Removed symlink: $$dst"; \
		fi; \
	done
