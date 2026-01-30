# Makefile for BMS-HETI Project
# Supports building both main program and test program

CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c99 -I.
LDFLAGS = -lm

# Source files for the library
LIB_SOURCES = bateryparamter.c CC.c AUKF.c SOCEstimator.c
LIB_OBJECTS = $(LIB_SOURCES:.c=.o)

# Executables
MAIN_TARGET = bms_main
TEST_TARGET = bms_test

# Default target
all: $(MAIN_TARGET) $(TEST_TARGET)

# Build main program
$(MAIN_TARGET): Main.o $(LIB_OBJECTS)
	@echo "Building main program..."
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "Main program built successfully: $(MAIN_TARGET)"

# Build test program
$(TEST_TARGET): TestSoCEstimator.o $(LIB_OBJECTS)
	@echo "Building test program..."
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "Test program built successfully: $(TEST_TARGET)"

# Compile source files to object files
%.o: %.c
	@echo "Compiling $<..."
	$(CC) $(CFLAGS) -c $< -o $@

# Create test_data directory if it doesn't exist
test_data:
	@mkdir -p test_data

# Run the test program
test: $(TEST_TARGET) test_data
	@echo ""
	@echo "Running SoC Estimator Test..."
	@echo "=============================="
	./$(TEST_TARGET)

# Run the main program
run: $(MAIN_TARGET)
	@echo ""
	@echo "Running Main Program..."
	@echo "======================="
	./$(MAIN_TARGET)

# Clean build files
clean:
	@echo "Cleaning build files..."
	rm -f *.o $(MAIN_TARGET) $(TEST_TARGET)
	@echo "Clean complete"

# Clean everything including test results
clean-all: clean
	@echo "Cleaning test results..."
	rm -f test_data/test_results.csv
	@echo "Clean all complete"

# Display help
help:
	@echo "BMS-HETI Makefile"
	@echo "================="
	@echo ""
	@echo "Available targets:"
	@echo "  all        - Build both main and test programs (default)"
	@echo "  test       - Build and run test program"
	@echo "  run        - Build and run main program"
	@echo "  clean      - Remove compiled files"
	@echo "  clean-all  - Remove compiled files and test results"
	@echo "  help       - Display this help message"
	@echo ""
	@echo "Examples:"
	@echo "  make           # Build everything"
	@echo "  make test      # Run the test program"
	@echo "  make run       # Run the main program"
	@echo "  make clean     # Clean build files"

# Dependencies
Main.o: Main.c main.h SOCEstimator.h Bateryparameter.h
TestSoCEstimator.o: TestSoCEstimator.c SOCEstimator.h Bateryparameter.h CC.h AUKF.h
SOCEstimator.o: SOCEstimator.c SOCEstimator.h CC.h AUKF.h Bateryparameter.h
CC.o: CC.c CC.h
AUKF.o: AUKF.c AUKF.h Bateryparameter.h
bateryparamter.o: bateryparamter.c Bateryparameter.h

.PHONY: all test run clean clean-all help test_data
