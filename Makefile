# Makefile
CROSS_COMPILE = ~/buildroot/output/host/bin/aarch64-buildroot-linux-gnu-
CC = $(CROSS_COMPILE)gcc
# switch to gnu99 from c99 to fix usleep warning
CFLAGS = -Wall -Wextra -std=gnu99 -O2
# Flags for the linker
LDFLAGS = -lgpiod -lm

# Source files for the library
LIB_SOURCES = tmc_uart_rpi.c tmc2209.c common.c tmc_gpio.c tmc_motion.c log.c
LIB_OBJECTS = $(LIB_SOURCES:.c=.o)

# Test executables
TEST_TARGETS = tests/tmc2209_test tests/test_s_curve_calculations tests/test_s_curve_motion tests/test_uart_timing tests/test_uart_timing_detailed tests/msync_benchmark tests/read_stepcount

# tests/test_motor_movement

.PHONY: all clean test_uart test_motor test_s_curve test_s_curve_motion

# Default target builds all tests
all: $(TEST_TARGETS)

# Test 1: UART connection test
tests/tmc2209_test: test_uart_connection.c $(LIB_OBJECTS)
	$(CC) $(CFLAGS) -o $@ $< $(LIB_OBJECTS) $(LDFLAGS)

# Test 2: Motor movement test
#tests/test_motor_movement: test_motor_movement.c $(LIB_OBJECTS)
#	$(CC) $(CFLAGS) -o $@ $< $(LIB_OBJECTS) $(LDFLAGS)

# Test 3: S-curve calculations test
tests/test_s_curve_calculations: test_s_curve_calculations.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

# Test 4: S-curve motion test
tests/test_s_curve_motion: test_s_curve_motion.c $(LIB_OBJECTS)
	$(CC) $(CFLAGS) -o $@ $< $(LIB_OBJECTS) $(LDFLAGS)

# Test 5: UART timing test
tests/test_uart_timing: test_uart_timing.c $(LIB_OBJECTS)
	$(CC) $(CFLAGS) -o $@ $< $(LIB_OBJECTS) $(LDFLAGS)

# Test 6: Detailed UART timing test
tests/test_uart_timing_detailed: test_uart_timing_detailed.c $(LIB_OBJECTS)
	$(CC) $(CFLAGS) -o $@ $< $(LIB_OBJECTS) $(LDFLAGS)

# Test 7: msync benchmark test
tests/msync_benchmark: msync_benchmark.c
	$(CC) $(CFLAGS) -o $@ $< -lrt

# Test 8: read stepcount utility
tests/read_stepcount: read_stepcount.c
	$(CC) $(CFLAGS) -o $@ $< -lrt



# Individual test targets
test_uart: tests/tmc2209_test

# test_motor: tests/test_motor_movement

test_s_curve: tests/test_s_curve_calculations

test_s_curve_motion: tests/test_s_curve_motion

test_uart_timing: tests/test_uart_timing

test_uart_timing_detailed: tests/test_uart_timing_detailed

test_msync_benchmark: tests/msync_benchmark

test_read_stepcount: tests/read_stepcount



# Compile library objects
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(LIB_OBJECTS) $(TEST_TARGETS)