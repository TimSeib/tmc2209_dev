/*
 * msync_benchmark.c - Benchmark script to test msync write performance
 *
 * This script tests the write speed of msync when writing values up to 100,000
 * and measures the time taken for different sync strategies.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <string.h>

// Get current timestamp in microseconds
static uint64_t get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

int main() {
    printf("=== MSCNT msync Write Performance Benchmark ===\n\n");
    
    // Test parameters
    const int32_t test_value = 100000;  // Single value to write
    const int num_operations = 1000;    // Number of times to repeat the operation
    const int num_runs = 5;
    
    // Test different sync strategies
    const char* strategies[] = {"MS_SYNC", "MS_ASYNC", "No sync"};
    const int sync_flags[] = {MS_SYNC, MS_ASYNC, -1}; // -1 means no sync
    
    for (int strategy = 0; strategy < 3; strategy++) {
        printf("Testing strategy: %s\n", strategies[strategy]);
        printf("----------------------------------------\n");
        
        double total_time = 0.0;
        double total_ops = 0.0;
        
        for (int run = 0; run < num_runs; run++) {
            // Create memory-mapped file
            int fd = open("benchmark.dat", O_RDWR | O_CREAT | O_TRUNC, 0644);
            if (fd == -1) {
                perror("Failed to open benchmark.dat");
                return -1;
            }
            
            // Set file size
            if (ftruncate(fd, sizeof(int32_t)) == -1) {
                perror("Failed to set file size");
                close(fd);
                return -1;
            }
            
            // Memory map the file
            volatile int32_t *value_ptr = mmap(NULL, sizeof(int32_t), 
                                              PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            if (value_ptr == MAP_FAILED) {
                perror("Failed to mmap benchmark.dat");
                close(fd);
                return -1;
            }
            
            // Initialize to zero
            *value_ptr = 0;
            
            // Benchmark: write the same value multiple times
            uint64_t start_time = get_time_us();
            
            for (int op = 0; op < num_operations; op++) {
                *value_ptr = test_value;
                
                if (sync_flags[strategy] != -1) {
                    msync((void*)value_ptr, sizeof(int32_t), sync_flags[strategy]);
                }
            }
            
            uint64_t end_time = get_time_us();
            double elapsed_ms = (double)(end_time - start_time) / 1000.0;
            
            printf("Run %d: %.2f ms for %d operations (%.2f ops/sec)\n", 
                   run + 1, elapsed_ms, num_operations, (double)num_operations / (elapsed_ms / 1000.0));
            
            total_time += elapsed_ms;
            total_ops += num_operations;
            
            // Cleanup
            munmap((void*)value_ptr, sizeof(int32_t));
            close(fd);
            
            // Small delay between runs
            usleep(100000); // 100ms
        }
        
        // Calculate averages
        double avg_time = total_time / num_runs;
        double avg_ops = total_ops / num_runs;
        double avg_ops_per_sec = avg_ops / (avg_time / 1000.0);
        
        printf("Average: %.2f ms for %.0f operations (%.2f ops/sec)\n", 
               avg_time, avg_ops, avg_ops_per_sec);
        printf("Average time per operation: %.3f μs\n\n", avg_time * 1000.0 / avg_ops);
    }
    
    // Test with different sync frequencies
    printf("Testing sync frequency impact (MS_ASYNC):\n");
    printf("----------------------------------------\n");
    
    int sync_intervals[] = {1, 10, 100, 1000, 10000};
    
    for (int interval_idx = 0; interval_idx < 5; interval_idx++) {
        int sync_interval = sync_intervals[interval_idx];
        
        // Create memory-mapped file
        int fd = open("benchmark.dat", O_RDWR | O_CREAT | O_TRUNC, 0644);
        if (fd == -1) {
            perror("Failed to open benchmark.dat");
            return -1;
        }
        
        if (ftruncate(fd, sizeof(int32_t)) == -1) {
            perror("Failed to set file size");
            close(fd);
            return -1;
        }
        
        volatile int32_t *value_ptr = mmap(NULL, sizeof(int32_t), 
                                          PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (value_ptr == MAP_FAILED) {
            perror("Failed to mmap benchmark.dat");
            close(fd);
            return -1;
        }
        
        *value_ptr = 0;
        
        uint64_t start_time = get_time_us();
        
        for (int op = 0; op < num_operations; op++) {
            *value_ptr = test_value;
            
            if (op % sync_interval == 0) {
                msync((void*)value_ptr, sizeof(int32_t), MS_ASYNC);
            }
        }
        
        uint64_t end_time = get_time_us();
        double elapsed_ms = (double)(end_time - start_time) / 1000.0;
        
        printf("Sync every %d ops: %.2f ms (%.2f ops/sec)\n", 
               sync_interval, elapsed_ms, (double)num_operations / (elapsed_ms / 1000.0));
        
        munmap((void*)value_ptr, sizeof(int32_t));
        close(fd);
        
        usleep(100000); // 100ms delay
    }
    
    printf("\nBenchmark completed!\n");
    return 0;
} 