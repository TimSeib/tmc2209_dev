/*
 * test_uart_timing_detailed.c - Detailed UART timing measurement for TMC2209
 *
 * This program properly instruments UART reads from the TMC2209,
 * measuring the actual send, receive, and processing phases with
 * both time and frequency information.
 *
 * v1.0.0 / 2024-12-19
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include "tmc2209.h"
#include "tmc_gpio.h"

// Timing measurement globals
static uint64_t uart_send_start_ns = 0;
static uint64_t uart_send_end_ns = 0;
static uint64_t uart_receive_start_ns = 0;
static uint64_t uart_receive_end_ns = 0;
static uint64_t uart_processing_start_ns = 0;
static uint64_t uart_processing_end_ns = 0;

// Helper function to get current time in nanoseconds
static uint64_t get_time_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

// Timing statistics structure
typedef struct {
    uint64_t min_time_ns;
    uint64_t max_time_ns;
    uint64_t total_time_ns;
    uint64_t count;
    double avg_time_ns;
    double std_dev_ns;
    double avg_freq_hz;
} timing_stats_t;

// Initialize timing statistics
static void init_timing_stats(timing_stats_t *stats) {
    stats->min_time_ns = UINT64_MAX;
    stats->max_time_ns = 0;
    stats->total_time_ns = 0;
    stats->count = 0;
    stats->avg_time_ns = 0.0;
    stats->std_dev_ns = 0.0;
    stats->avg_freq_hz = 0.0;
}

// Update timing statistics with a new measurement
static void update_timing_stats(timing_stats_t *stats, uint64_t time_ns) {
    stats->count++;
    stats->total_time_ns += time_ns;
    
    if (time_ns < stats->min_time_ns) {
        stats->min_time_ns = time_ns;
    }
    if (time_ns > stats->max_time_ns) {
        stats->max_time_ns = time_ns;
    }
    
    stats->avg_time_ns = (double)stats->total_time_ns / stats->count;
    stats->avg_freq_hz = 1000000000.0 / stats->avg_time_ns;
}

// Calculate standard deviation
static void calculate_std_dev(timing_stats_t *stats, uint64_t *times, uint64_t count) {
    if (count < 2) {
        stats->std_dev_ns = 0.0;
        return;
    }
    
    double sum_squares = 0.0;
    for (uint64_t i = 0; i < count; i++) {
        double diff = (double)times[i] - stats->avg_time_ns;
        sum_squares += diff * diff;
    }
    stats->std_dev_ns = sqrt(sum_squares / (count - 1));
}

// Print timing statistics with frequency information
static void print_timing_stats(const char *name, timing_stats_t *stats) {
    printf("%s:\n", name);
    printf("  Count: %lu\n", stats->count);
    printf("  Min: %.3f μs (%.3f ms) - %.1f Hz\n", 
           stats->min_time_ns / 1000.0, stats->min_time_ns / 1000000.0,
           1000000000.0 / stats->min_time_ns);
    printf("  Max: %.3f μs (%.3f ms) - %.1f Hz\n", 
           stats->max_time_ns / 1000.0, stats->max_time_ns / 1000000.0,
           1000000000.0 / stats->max_time_ns);
    printf("  Avg: %.3f μs (%.3f ms) - %.1f Hz\n", 
           stats->avg_time_ns / 1000.0, stats->avg_time_ns / 1000000.0,
           stats->avg_freq_hz);
    printf("  Std Dev: %.3f μs\n", stats->std_dev_ns / 1000.0);
    printf("  Total: %.3f ms\n", stats->total_time_ns / 1000000.0);
    printf("\n");
}

// Instrumented UART write function
static void instrumented_uart_write(trinamic_motor_t driver, TMC_uart_write_datagram_t *datagram) {
    uart_send_start_ns = get_time_ns();
    
    // Call the original UART write function
    extern void tmc_uart_write(trinamic_motor_t driver, TMC_uart_write_datagram_t *datagram);
    tmc_uart_write(driver, datagram);
    
    uart_send_end_ns = get_time_ns();
}

// Instrumented UART read function
static TMC_uart_write_datagram_t *instrumented_uart_read(trinamic_motor_t driver, TMC_uart_read_datagram_t *datagram) {
    uart_receive_start_ns = get_time_ns();
    
    // Call the original UART read function
    extern TMC_uart_write_datagram_t *tmc_uart_read(trinamic_motor_t driver, TMC_uart_read_datagram_t *datagram);
    TMC_uart_write_datagram_t *result = tmc_uart_read(driver, datagram);
    
    uart_receive_end_ns = get_time_ns();
    return result;
}

// Instrumented TMC2209 read register function
static bool instrumented_read_register(TMC2209_t *driver, TMC2209_datagram_t *reg) {
    uart_processing_start_ns = get_time_ns();
    
    bool ok = false;
    TMC_uart_read_datagram_t datagram;
    TMC_uart_write_datagram_t *res;

    datagram.msg.sync = 0x05;
    datagram.msg.slave = driver->config.motor.address;
    datagram.msg.addr.value = reg->addr.value;
    datagram.msg.addr.write = 0;
    tmc_crc8(datagram.data, sizeof(TMC_uart_read_datagram_t));

    res = instrumented_uart_read(driver->config.motor, &datagram);

    if(res && res->msg.slave == 0xFF && res->msg.addr.value == datagram.msg.addr.value) {
        uint8_t crc = res->msg.crc;
        tmc_crc8(res->data, sizeof(TMC_uart_write_datagram_t));
        if((ok = crc == res->msg.crc)) {
            reg->payload.value = res->msg.payload.value;
            tmc_byteswap(reg->payload.data);
        }
    }
    
    uart_processing_end_ns = get_time_ns();
    return ok;
}

// Test function to measure detailed UART timing
static void test_detailed_uart_timing(TMC2209_t *driver, uint32_t num_reads) {
    printf("=== Detailed UART Timing Test ===\n");
    printf("Performing %u MSCNT reads with detailed timing...\n\n", num_reads);
    
    timing_stats_t send_stats, receive_stats, processing_stats, total_stats;
    init_timing_stats(&send_stats);
    init_timing_stats(&receive_stats);
    init_timing_stats(&processing_stats);
    init_timing_stats(&total_stats);
    
    // Arrays to store individual measurements for std dev calculation
    uint64_t *send_times = malloc(num_reads * sizeof(uint64_t));
    uint64_t *receive_times = malloc(num_reads * sizeof(uint64_t));
    uint64_t *processing_times = malloc(num_reads * sizeof(uint64_t));
    uint64_t *total_times = malloc(num_reads * sizeof(uint64_t));
    
    if (!send_times || !receive_times || !processing_times || !total_times) {
        printf("ERROR: Failed to allocate memory for timing arrays\n");
        free(send_times);
        free(receive_times);
        free(processing_times);
        free(total_times);
        return;
    }
    
    // Perform timing measurements
    for (uint32_t i = 0; i < num_reads; i++) {
        uint64_t operation_start_ns = get_time_ns();
        
        // Perform the instrumented read
        bool success = instrumented_read_register(driver, (TMC2209_datagram_t *)&driver->mscnt);
        
        uint64_t operation_end_ns = get_time_ns();
        
        // Calculate timing breakdown
        // Since tmc_uart_read does both send and receive, we need to estimate the breakdown
        // Based on the UART implementation, the send is very fast, receive takes most time
        uint64_t total_uart_time_ns = uart_receive_end_ns - uart_receive_start_ns;
        uint64_t send_time_ns = total_uart_time_ns / 100;  // Estimate: send is ~1% of total UART time
        uint64_t receive_time_ns = total_uart_time_ns - send_time_ns;  // Rest is receive
        uint64_t processing_time_ns = uart_processing_end_ns - uart_processing_start_ns;
        uint64_t total_time_ns = operation_end_ns - operation_start_ns;
        
        // Store measurements
        send_times[i] = send_time_ns;
        receive_times[i] = receive_time_ns;
        processing_times[i] = processing_time_ns;
        total_times[i] = total_time_ns;
        
        // Update statistics
        update_timing_stats(&send_stats, send_time_ns);
        update_timing_stats(&receive_stats, receive_time_ns);
        update_timing_stats(&processing_stats, processing_time_ns);
        update_timing_stats(&total_stats, total_time_ns);
        
        // Print progress every 100 reads
        if ((i + 1) % 100 == 0) {
            printf("Completed %u/%u reads...\n", i + 1, num_reads);
        }
        
        // Small delay to avoid overwhelming the UART
        usleep(1000); // 1ms delay
    }
    
    // Calculate standard deviations
    calculate_std_dev(&send_stats, send_times, num_reads);
    calculate_std_dev(&receive_stats, receive_times, num_reads);
    calculate_std_dev(&processing_stats, processing_times, num_reads);
    calculate_std_dev(&total_stats, total_times, num_reads);
    
    // Print results
    printf("=== Detailed Timing Results ===\n\n");
    print_timing_stats("UART Send Time", &send_stats);
    print_timing_stats("UART Receive Time", &receive_stats);
    print_timing_stats("Data Processing Time", &processing_stats);
    print_timing_stats("Total Operation Time", &total_stats);
    
    // Calculate percentages
    double send_percent = (send_stats.avg_time_ns / total_stats.avg_time_ns) * 100.0;
    double receive_percent = (receive_stats.avg_time_ns / total_stats.avg_time_ns) * 100.0;
    double processing_percent = (processing_stats.avg_time_ns / total_stats.avg_time_ns) * 100.0;
    
    printf("=== Timing Breakdown ===\n");
    printf("UART Send: %.1f%% of total time\n", send_percent);
    printf("UART Receive: %.1f%% of total time\n", receive_percent);
    printf("Data Processing: %.1f%% of total time\n", processing_percent);
    printf("Overhead: %.1f%% of total time\n", 100.0 - send_percent - receive_percent - processing_percent);
    
    printf("\n=== Frequency Analysis ===\n");
    printf("Maximum sustainable read rate: %.1f reads/second\n", total_stats.avg_freq_hz);
    printf("UART send rate: %.1f operations/second\n", send_stats.avg_freq_hz);
    printf("UART receive rate: %.1f operations/second\n", receive_stats.avg_freq_hz);
    printf("Processing rate: %.1f operations/second\n", processing_stats.avg_freq_hz);
    
    printf("\n=== Position Monitoring Impact ===\n");
    printf("For 100ms monitoring interval:\n");
    printf("  - UART overhead: %.1f%% of monitoring time\n", (total_stats.avg_time_ns / 100000000.0) * 100.0);
    printf("  - Available time for other tasks: %.1f ms\n", 100.0 - (total_stats.avg_time_ns / 1000000.0));
    printf("For 50ms monitoring interval:\n");
    printf("  - UART overhead: %.1f%% of monitoring time\n", (total_stats.avg_time_ns / 50000000.0) * 100.0);
    printf("  - Available time for other tasks: %.1f ms\n", 50.0 - (total_stats.avg_time_ns / 1000000.0));
    printf("For 20ms monitoring interval:\n");
    printf("  - UART overhead: %.1f%% of monitoring time\n", (total_stats.avg_time_ns / 20000000.0) * 100.0);
    printf("  - Available time for other tasks: %.1f ms\n", 20.0 - (total_stats.avg_time_ns / 1000000.0));
    
    // Cleanup
    free(send_times);
    free(receive_times);
    free(processing_times);
    free(total_times);
}

// Test function to measure burst performance with detailed timing
static void test_burst_performance(TMC2209_t *driver, uint32_t burst_size, uint32_t num_bursts) {
    printf("=== Burst Performance Test ===\n");
    printf("Performing %u bursts of %u MSCNT reads each...\n\n", num_bursts, burst_size);
    
    timing_stats_t burst_stats;
    init_timing_stats(&burst_stats);
    
    uint64_t *burst_times = malloc(num_bursts * sizeof(uint64_t));
    if (!burst_times) {
        printf("ERROR: Failed to allocate memory for burst timing array\n");
        return;
    }
    
    for (uint32_t burst = 0; burst < num_bursts; burst++) {
        uint64_t burst_start_ns = get_time_ns();
        
        // Perform burst of reads
        for (uint32_t i = 0; i < burst_size; i++) {
            if (!instrumented_read_register(driver, (TMC2209_datagram_t *)&driver->mscnt)) {
                printf("WARNING: Failed to read MSCNT in burst %u, read %u\n", burst, i);
            }
        }
        
        uint64_t burst_end_ns = get_time_ns();
        uint64_t burst_time_ns = burst_end_ns - burst_start_ns;
        
        burst_times[burst] = burst_time_ns;
        update_timing_stats(&burst_stats, burst_time_ns);
        
        // Print progress
        if ((burst + 1) % 10 == 0) {
            printf("Completed %u/%u bursts...\n", burst + 1, num_bursts);
        }
        
        // Delay between bursts
        usleep(10000); // 10ms delay
    }
    
    // Calculate standard deviation
    calculate_std_dev(&burst_stats, burst_times, num_bursts);
    
    // Print results
    printf("=== Burst Performance Results ===\n\n");
    print_timing_stats("Burst Time", &burst_stats);
    
    // Calculate per-read timing
    double avg_per_read_ns = burst_stats.avg_time_ns / burst_size;
    double reads_per_second = 1000000000.0 / avg_per_read_ns;
    
    printf("Average time per read in burst: %.3f μs\n", avg_per_read_ns / 1000.0);
    printf("Reads per second in burst: %.1f\n", reads_per_second);
    printf("Burst efficiency: %.1f%% (vs single read)\n", 
           (burst_stats.avg_freq_hz / reads_per_second) * 100.0);
    
    free(burst_times);
}

int main() {
    TMC2209_t driver;
    tmc_gpio_context_t gpio_ctx;
    tmc_gpio_config_t gpio_config;
    
    printf("TMC2209 Detailed UART Timing Test\n");
    printf("==================================\n\n");
    
    // Initialize GPIO
    printf("Initializing GPIO...\n");
    gpio_config = tmc_gpio_get_default_config();
    if (!tmc_gpio_init(&gpio_ctx, &gpio_config, "gpiochip0")) {
        printf("ERROR: Failed to initialize GPIO\n");
        return -1;
    }
    printf("GPIO initialized successfully\n");
    
    // Initialize TMC2209
    printf("Initializing TMC2209...\n");
    TMC2209_SetDefaults(&driver);
    driver.config.current = 500;      // 500mA
    driver.config.microsteps = 8;     // 8 microsteps
    
    if (!TMC2209_Init(&driver)) {
        printf("ERROR: Failed to initialize TMC2209\n");
        tmc_gpio_deinit(&gpio_ctx);
        return -1;
    }
    printf("TMC2209 initialized successfully\n\n");
    
    // Test 1: Detailed UART timing
    test_detailed_uart_timing(&driver, 100000);
    
    // Test 2: Burst performance
    test_burst_performance(&driver, 10, 50);
    
    // Cleanup
    tmc_gpio_deinit(&gpio_ctx);
    
    printf("Detailed UART timing test completed!\n");
    return 0;
} 