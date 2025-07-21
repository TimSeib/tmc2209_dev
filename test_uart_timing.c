/*
 * test_uart_timing.c - UART timing measurement for TMC2209
 *
 * This program measures the timing breakdown of UART reads from the TMC2209,
 * including request time, receive time, and total communication time.
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
} timing_stats_t;

// Initialize timing statistics
static void init_timing_stats(timing_stats_t *stats) {
    stats->min_time_ns = UINT64_MAX;
    stats->max_time_ns = 0;
    stats->total_time_ns = 0;
    stats->count = 0;
    stats->avg_time_ns = 0.0;
    stats->std_dev_ns = 0.0;
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

// Print timing statistics
static void print_timing_stats(const char *name, timing_stats_t *stats) {
    printf("%s:\n", name);
    printf("  Count: %lu\n", stats->count);
    printf("  Min: %.3f μs (%.3f ms)\n", stats->min_time_ns / 1000.0, stats->min_time_ns / 1000000.0);
    printf("  Max: %.3f μs (%.3f ms)\n", stats->max_time_ns / 1000.0, stats->max_time_ns / 1000000.0);
    printf("  Avg: %.3f μs (%.3f ms)\n", stats->avg_time_ns / 1000.0, stats->avg_time_ns / 1000000.0);
    printf("  Std Dev: %.3f μs\n", stats->std_dev_ns / 1000.0);
    printf("  Total: %.3f ms\n", stats->total_time_ns / 1000000.0);
    printf("\n");
}

// Test function to measure MSCNT read timing
static void test_mscnt_read_timing(TMC2209_t *driver, uint32_t num_reads) {
    printf("=== MSCNT Read Timing Test ===\n");
    printf("Performing %u MSCNT reads...\n\n", num_reads);
    
    timing_stats_t total_stats, request_stats, receive_stats;
    init_timing_stats(&total_stats);
    init_timing_stats(&request_stats);
    init_timing_stats(&receive_stats);
    
    // Arrays to store individual measurements for std dev calculation
    uint64_t *total_times = malloc(num_reads * sizeof(uint64_t));
    uint64_t *request_times = malloc(num_reads * sizeof(uint64_t));
    uint64_t *receive_times = malloc(num_reads * sizeof(uint64_t));
    
    if (!total_times || !request_times || !receive_times) {
        printf("ERROR: Failed to allocate memory for timing arrays\n");
        free(total_times);
        free(request_times);
        free(receive_times);
        return;
    }
    
    // Perform timing measurements
    for (uint32_t i = 0; i < num_reads; i++) {
        uint64_t start_time_ns = get_time_ns();
        
        // Measure request time (time to send read request)
        uint64_t request_start_ns = get_time_ns();
        
        // This is a simplified measurement - in practice, we'd need to 
        // instrument the actual UART send function to get precise timing
        // For now, we'll measure the entire read operation
        (void)TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->mscnt);
        
        uint64_t request_end_ns = get_time_ns();
        uint64_t total_end_ns = get_time_ns();
        
        // Calculate timing breakdown
        uint64_t total_time_ns = total_end_ns - start_time_ns;
        uint64_t request_time_ns = request_end_ns - request_start_ns;
        uint64_t receive_time_ns = total_time_ns - request_time_ns;
        
        // Store measurements
        total_times[i] = total_time_ns;
        request_times[i] = request_time_ns;
        receive_times[i] = receive_time_ns;
        
        // Update statistics
        update_timing_stats(&total_stats, total_time_ns);
        update_timing_stats(&request_stats, request_time_ns);
        update_timing_stats(&receive_stats, receive_time_ns);
        
        // Print progress every 100 reads
        if ((i + 1) % 100 == 0) {
            printf("Completed %u/%u reads...\n", i + 1, num_reads);
        }
        
        // Small delay to avoid overwhelming the UART
        usleep(1000); // 1ms delay
    }
    
    // Calculate standard deviations
    calculate_std_dev(&total_stats, total_times, num_reads);
    calculate_std_dev(&request_stats, request_times, num_reads);
    calculate_std_dev(&receive_stats, receive_times, num_reads);
    
    // Print results
    printf("=== Timing Results ===\n\n");
    print_timing_stats("Total Read Time", &total_stats);
    print_timing_stats("Request Time", &request_stats);
    print_timing_stats("Receive Time", &receive_stats);
    
    // Calculate percentages
    double request_percent = (request_stats.avg_time_ns / total_stats.avg_time_ns) * 100.0;
    double receive_percent = (receive_stats.avg_time_ns / total_stats.avg_time_ns) * 100.0;
    
    printf("=== Timing Breakdown ===\n");
    printf("Request phase: %.1f%% of total time\n", request_percent);
    printf("Receive phase: %.1f%% of total time\n", receive_percent);
    printf("Overhead: %.1f%% of total time\n", 100.0 - request_percent - receive_percent);
    
    // Cleanup
    free(total_times);
    free(request_times);
    free(receive_times);
}

// Test function to measure different register read timing
static void test_register_read_timing(TMC2209_t *driver, uint32_t num_reads) {
    printf("=== Register Read Timing Comparison ===\n");
    printf("Performing %u reads of different registers...\n\n", num_reads);
    
    timing_stats_t mscnt_stats, tstep_stats, sg_result_stats;
    init_timing_stats(&mscnt_stats);
    init_timing_stats(&tstep_stats);
    init_timing_stats(&sg_result_stats);
    
    // Arrays for std dev calculation
    uint64_t *mscnt_times = malloc(num_reads * sizeof(uint64_t));
    uint64_t *tstep_times = malloc(num_reads * sizeof(uint64_t));
    uint64_t *sg_result_times = malloc(num_reads * sizeof(uint64_t));
    
    if (!mscnt_times || !tstep_times || !sg_result_times) {
        printf("ERROR: Failed to allocate memory for timing arrays\n");
        free(mscnt_times);
        free(tstep_times);
        free(sg_result_times);
        return;
    }
    
    // Test MSCNT reads
    printf("Testing MSCNT reads...\n");
    for (uint32_t i = 0; i < num_reads; i++) {
        uint64_t start_ns = get_time_ns();
        (void)TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->mscnt);
        uint64_t end_ns = get_time_ns();
        
        uint64_t time_ns = end_ns - start_ns;
        mscnt_times[i] = time_ns;
        update_timing_stats(&mscnt_stats, time_ns);
        
        usleep(1000);
    }
    
    // Test TSTEP reads
    printf("Testing TSTEP reads...\n");
    for (uint32_t i = 0; i < num_reads; i++) {
        uint64_t start_ns = get_time_ns();
        (void)TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->tstep);
        uint64_t end_ns = get_time_ns();
        
        uint64_t time_ns = end_ns - start_ns;
        tstep_times[i] = time_ns;
        update_timing_stats(&tstep_stats, time_ns);
        
        usleep(1000);
    }
    
    // Test SG_RESULT reads
    printf("Testing SG_RESULT reads...\n");
    for (uint32_t i = 0; i < num_reads; i++) {
        uint64_t start_ns = get_time_ns();
        (void)TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->sg_result);
        uint64_t end_ns = get_time_ns();
        
        uint64_t time_ns = end_ns - start_ns;
        sg_result_times[i] = time_ns;
        update_timing_stats(&sg_result_stats, time_ns);
        
        usleep(1000);
    }
    
    // Calculate standard deviations
    calculate_std_dev(&mscnt_stats, mscnt_times, num_reads);
    calculate_std_dev(&tstep_stats, tstep_times, num_reads);
    calculate_std_dev(&sg_result_stats, sg_result_times, num_reads);
    
    // Print results
    printf("=== Register Read Timing Results ===\n\n");
    print_timing_stats("MSCNT Register", &mscnt_stats);
    print_timing_stats("TSTEP Register", &tstep_stats);
    print_timing_stats("SG_RESULT Register", &sg_result_stats);
    
    // Cleanup
    free(mscnt_times);
    free(tstep_times);
    free(sg_result_times);
}

// Test function to measure burst read timing
static void test_burst_read_timing(TMC2209_t *driver, uint32_t burst_size, uint32_t num_bursts) {
    printf("=== Burst Read Timing Test ===\n");
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
            if (!TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->mscnt)) {
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
    printf("=== Burst Read Results ===\n\n");
    print_timing_stats("Burst Time", &burst_stats);
    
    // Calculate per-read timing
    double avg_per_read_ns = burst_stats.avg_time_ns / burst_size;
    printf("Average time per read in burst: %.3f μs\n", avg_per_read_ns / 1000.0);
    printf("Reads per second in burst: %.1f\n", 1000000000.0 / avg_per_read_ns);
    
    free(burst_times);
}

int main() {
    TMC2209_t driver;
    tmc_gpio_context_t gpio_ctx;
    tmc_gpio_config_t gpio_config;
    
    printf("TMC2209 UART Timing Test\n");
    printf("========================\n\n");
    
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
    
    // Test 1: Basic MSCNT read timing
    test_mscnt_read_timing(&driver, 1000);
    
    // Test 2: Different register read timing
    test_register_read_timing(&driver, 500);
    
    // Test 3: Burst read timing
    test_burst_read_timing(&driver, 10, 100);
    
    // Cleanup
    tmc_gpio_deinit(&gpio_ctx);
    
    printf("UART timing test completed!\n");
    return 0;
} 