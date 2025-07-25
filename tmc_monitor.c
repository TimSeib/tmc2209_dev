/*
 * tmc_monitor.c - Position monitoring implementation for TMC2209
 *
 * This file implements position monitoring using MSCNT register polling
 * for real-time position tracking and error detection.
 *
 * v1.0.0 / 2024-12-19
 */

#include "tmc_monitor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>
#include "log.h"

// Global variable to track total MSCNT delta for entire movement
int32_t g_total_mscnt_delta = 0;

// ============================================================================
// INTERNAL FUNCTIONS
// ============================================================================
/**
 * @brief Get current timestamp in microseconds
 * 
 * @return Current timestamp
 */
static uint64_t get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/**
 * @brief Position monitoring thread function - microstep-based polling
 * 
 * @param arg Position monitor structure pointer
 * @return NULL
 */
static void* position_monitor_thread_function(void *arg) {
    tmc_position_monitor_t *monitor = (tmc_position_monitor_t*)arg;
    
    if (!monitor || !monitor->driver) {
        log_error("Invalid position monitor in thread");
        return NULL;
    }
    
    // Initialize memory mapping for step counting
    int fd = open("stepcount.dat", O_RDWR | O_CREAT, 0644);
    struct stat st;
    fstat(fd, &st);
    bool file_was_empty = (st.st_size == 0);
    if (fd == -1) {
        log_error("Failed to open stepcount.dat file");
        return NULL;
    }
    
    // Ensure file has proper size for mapping
    if (ftruncate(fd, sizeof(int32_t)) == -1) {
        log_error("Failed to set file size for mapping");
        close(fd);
        return NULL;
    }
    
    volatile int32_t *step_ptr = mmap(NULL, sizeof(int32_t), 
                                    PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (step_ptr == MAP_FAILED) {
        log_error("Failed to mmap stepcount.dat file");
        close(fd);
        return NULL;
    }

    // Only initialize if this is a brand new file
    if (file_was_empty) {
        *step_ptr = 0;
        msync((void*)step_ptr, sizeof(int32_t), MS_ASYNC);
        log_info("Initialized new step counter to 0");
    } else {
        log_info("Restored step counter to %d", *step_ptr);
    }
    
    log_debug("Position monitoring thread started (timed polling)");
    
    uint64_t last_poll_time = get_time_us();
    uint32_t last_step_count = 0;
    bool first_poll = true;
    
    while (monitor->monitoring_active) {
        // Check if driver is still valid
        if (!monitor->driver) {
            log_warn("TMC2209 driver became invalid, stopping monitor");
            break;
        }
        
        uint64_t current_time = get_time_us();
        uint32_t current_step_count = 0;
        bool should_poll = false;
        
        // Get current step count under mutex protection
        if (pthread_mutex_lock(&monitor->data_mutex) == 0) {
            current_step_count = monitor->last_step_count;
            pthread_mutex_unlock(&monitor->data_mutex);
        }
        
        // Poll if:
        // 1. We're at a full step boundary (step count is multiple of microstep_resolution)
        // 2. At least 1ms has passed since last poll (to avoid excessive UART traffic)
        // 3. Either this is the first poll OR we've moved to a new step boundary
        if (current_step_count > 0 && 
            (current_step_count % monitor->poll_interval_steps) == 0 &&
            (current_time - last_poll_time) >= 1000) { // 1ms minimum interval
            
            // For first poll, always do it. For subsequent polls, check if we've moved to a new boundary
            if (first_poll || current_step_count != last_step_count) {
                should_poll = true;
                last_step_count = current_step_count;
                last_poll_time = current_time;
            }
        }
        
        if (should_poll) {
            // Read MSCNT at microstep boundary
            if (TMC2209_ReadRegister(monitor->driver, (TMC2209_datagram_t *)&monitor->driver->mscnt)) {
                uint16_t actual_mscnt = monitor->driver->mscnt.reg.mscnt;
                
                // NOW read the current step count (after MSCNT read)
                uint32_t current_step_count_at_mscnt_read = 0;
                if (pthread_mutex_lock(&monitor->data_mutex) == 0) {
                    current_step_count_at_mscnt_read = monitor->last_step_count;
                    pthread_mutex_unlock(&monitor->data_mutex);
                }

                // on first poll do not increment g_total_mscnt_delta wait until first full step
                if(first_poll){
                    log_trace("initial_mscnt=%d", monitor->initial_mscnt);
                    log_trace("actual_mscnt=%d", actual_mscnt);
                    int32_t mscnt_diff_actual = (int32_t)actual_mscnt - (int32_t)monitor->initial_mscnt;
                    // Handle MSCNT wrap-around properly
                    if (mscnt_diff_actual > 512) {
                        mscnt_diff_actual -= 1024;
                    } else if (mscnt_diff_actual < -512) {
                        mscnt_diff_actual += 1024;
                    }
                    g_total_mscnt_delta += mscnt_diff_actual;
                    log_trace("g_total_mscnt_delta=%d", g_total_mscnt_delta);
                    *step_ptr += mscnt_diff_actual;
                
                    // Sync to disk every step
                    msync((void*)step_ptr, sizeof(int32_t), MS_SYNC);
                    first_poll = false;
                }else{
                    int32_t mscnt_diff_actual = (int32_t)actual_mscnt - (int32_t)monitor->last_mscnt;
                    // Handle MSCNT wrap-around properly
                    if (mscnt_diff_actual > 512) {
                        mscnt_diff_actual -= 1024;
                    } else if (mscnt_diff_actual < -512) {
                        mscnt_diff_actual += 1024;
                    }
                    g_total_mscnt_delta += mscnt_diff_actual;
                    log_trace("g_total_mscnt_delta=%d", g_total_mscnt_delta);
                    *step_ptr += mscnt_diff_actual;
                
                    // Sync to disk every step
                    msync((void*)step_ptr, sizeof(int32_t), MS_SYNC);
                }
                
                // Use the step count that was current when MSCNT was read
                uint16_t expected_mscnt_absolute = 0;
                
                // Calculate expected MSCNT based on software step count from initial position
                // The software step count represents the number of steps moved from the initial position
                uint16_t expected_mscnt_relative = calculate_expected_mscnt(current_step_count_at_mscnt_read, monitor->microstep_resolution, monitor->direction);
                
                // For both directions, we add the relative MSCNT change to the initial position
                // The calculate_expected_mscnt function already handles the direction
                if (monitor->direction) { // counter-clockwise
                    expected_mscnt_absolute = (monitor->initial_mscnt + expected_mscnt_relative) % 1024;
                } else { // clockwise
                    // For clockwise, we subtract the change since MSCNT decreases
                    int32_t signed_mscnt = (int32_t)monitor->initial_mscnt - (int32_t)expected_mscnt_relative;
                    // Handle wrap-around for negative values
                    while (signed_mscnt < 0) {
                        signed_mscnt += 1024;
                    }
                    expected_mscnt_absolute = (uint16_t)signed_mscnt;
                }
                
                // Calculate instantaneous error with wrap-around handling (relative to initial)
                int16_t difference = (int16_t)actual_mscnt - (int16_t)expected_mscnt_absolute;
                if (difference > 512) {
                    difference -= 1024;
                } else if (difference < -512) {
                    difference += 1024;
                }
                
                // Calculate step error accumulation: compare expected vs actual step progress
                float step_error = 0.0f;
                if (monitor->error_check_count > 0) {
                    // Calculate expected step difference since last check
                    int32_t expected_step_diff = (int32_t)current_step_count - (int32_t)monitor->last_expected_step_count;
                    
                    // Calculate actual step difference based on MSCNT change
                    int32_t mscnt_diff = (int32_t)actual_mscnt - (int32_t)monitor->last_expected_mscnt;
                    
                    // Handle MSCNT wrap-around properly
                    if (mscnt_diff > 512) {
                        mscnt_diff -= 1024;
                    } else if (mscnt_diff < -512) {
                        mscnt_diff += 1024;
                    }

                    // Convert MSCNT difference to step difference
                    // MSCNT increments by 256 per full step
                    // Calculate precise microstep difference without rounding
                    float mscnt_full_step_diff_float = (float)mscnt_diff / 256.0f;
                    float actual_step_diff_float = mscnt_full_step_diff_float * (float)monitor->microstep_resolution;
                    
                    // Step error = expected step difference - actual step difference (in microsteps)
                    log_trace("expected_step_diff=%f, actual_step_diff_float=%f", (float)expected_step_diff, actual_step_diff_float);
                    step_error = (float)expected_step_diff - fabs(actual_step_diff_float);
                    
                    // Calculate actual position offset from initial position
                    uint32_t total_steps_moved = current_step_count - monitor->initial_step_count;
                    uint16_t expected_mscnt_for_total_movement = calculate_expected_mscnt(total_steps_moved, monitor->microstep_resolution, monitor->direction);
                    
                    // For both directions, we add the relative MSCNT change to the initial position
                    // The calculate_expected_mscnt function already handles the direction
                    uint16_t expected_final_mscnt;
                    // For counter-clockwise, we subtract the change since MSCNT decreases
                    int32_t signed_mscnt = (int32_t)monitor->initial_mscnt - (int32_t)expected_mscnt_for_total_movement;
                    // Handle wrap-around for negative values
                    while (signed_mscnt < 0) {
                        signed_mscnt += 1024;
                    }
                    expected_final_mscnt = (uint16_t)signed_mscnt;
                    
                    // Calculate actual position offset in microsteps
                    int16_t mscnt_position_offset = (int16_t)actual_mscnt - (int16_t)expected_final_mscnt;
                    if (mscnt_position_offset > 512) {
                        mscnt_position_offset -= 1024;
                    } else if (mscnt_position_offset < -512) {
                        mscnt_position_offset += 1024;
                    }
                    float position_offset_microsteps = (float)mscnt_position_offset * (float)monitor->microstep_resolution / 256.0f;
                    
                    // Debug the step error calculation and position offset
                    log_trace("step_diff=%d, mscnt_diff=%d, mscnt_full_step_diff_float=%.3f, actual_step_diff_float=%.3f, step_error=%.3f",
                           expected_step_diff, mscnt_diff, mscnt_full_step_diff_float, actual_step_diff_float, step_error);
                    log_debug("POSITION_OFFSET: total_steps_moved=%u, expected_final_mscnt=%u, actual_mscnt=%u, mscnt_offset=%d, position_offset_microsteps=%.3f",
                           total_steps_moved, expected_final_mscnt, actual_mscnt, mscnt_position_offset, position_offset_microsteps);
                    log_debug("last_expected_mscnt=%u, actual_mscnt=%u",
                           monitor->last_expected_mscnt, actual_mscnt);
                }
                
                // Update status under mutex protection
                if (pthread_mutex_lock(&monitor->data_mutex) == 0) {
                    monitor->last_mscnt = actual_mscnt;
                    monitor->position_error = difference;
                    monitor->position_valid = (abs(difference) <= 512); // Allow ±2 full steps (512 MSCNT units)
                    monitor->last_update_time_us = get_time_us();
                    
                    // Update error accumulation tracking
                    monitor->last_expected_step_count = current_step_count;
                    monitor->last_expected_mscnt = expected_mscnt_absolute;
                    monitor->total_step_error += step_error;
                    monitor->error_check_count++;
                    monitor->average_step_error = (float)monitor->total_step_error / monitor->error_check_count;
                    
                    pthread_mutex_unlock(&monitor->data_mutex);
                }
                
                // Debug output for position checks with error accumulation
                log_trace("Position check at step %u (full step %u): MSCNT=%u (expected=%u), position_error=%d MSCNT units, step_error=%.3f microsteps", 
                       current_step_count, current_step_count / monitor->microstep_resolution, 
                       actual_mscnt, expected_mscnt_absolute, difference, step_error);
                
                // Add position offset debugging if we have step error data
                if (monitor->error_check_count > 0) {
                    uint32_t total_steps_moved = current_step_count - monitor->initial_step_count;
                    uint16_t expected_mscnt_for_total_movement = calculate_expected_mscnt(total_steps_moved, monitor->microstep_resolution, monitor->direction);
                    
                    // For both directions, we add the relative MSCNT change to the initial position
                    // The calculate_expected_mscnt function already handles the direction
                    uint16_t expected_final_mscnt;
                    // For counter-clockwise, we subtract the change since MSCNT decreases
                    int32_t signed_mscnt = (int32_t)monitor->initial_mscnt - (int32_t)expected_mscnt_for_total_movement;
                    // Handle wrap-around for negative values
                    while (signed_mscnt < 0) {
                        signed_mscnt += 1024;
                    }
                    expected_final_mscnt = (uint16_t)signed_mscnt;
                    
                    int16_t mscnt_position_offset = (int16_t)actual_mscnt - (int16_t)expected_final_mscnt;
                    if (mscnt_position_offset > 512) {
                        mscnt_position_offset -= 1024;
                    } else if (mscnt_position_offset < -512) {
                        mscnt_position_offset += 1024;
                    }
                    float position_offset_microsteps = (float)mscnt_position_offset * (float)monitor->microstep_resolution / 256.0f;
                    
                    log_trace("Position accuracy: %.3f microsteps offset from expected position", position_offset_microsteps);
                }
                
                log_trace("step_count=%u, expected_mscnt_absolute=%u, initial_mscnt=%u",
                       current_step_count_at_mscnt_read, expected_mscnt_absolute, monitor->initial_mscnt);
            } else {
                log_warn("Failed to read MSCNT register at step %u", current_step_count);
            }
        }
        usleep(500); // 100μs sleep - increased to reduce UART conflicts
    }
    if(TMC2209_ReadRegister(monitor->driver, (TMC2209_datagram_t *)&monitor->driver->mscnt)){
        uint16_t actual_mscnt = monitor->driver->mscnt.reg.mscnt;
        int32_t mscnt_diff_actual = (int32_t)actual_mscnt - (int32_t)monitor->last_mscnt;
        // Handle MSCNT wrap-around properly
        if (mscnt_diff_actual > 512) {
            mscnt_diff_actual -= 1024;
        } else if (mscnt_diff_actual < -512) {
            mscnt_diff_actual += 1024;
        }
        log_trace("g_total_mscnt_delta before FINAL=%d", g_total_mscnt_delta);
        g_total_mscnt_delta += mscnt_diff_actual;
        log_trace("g_total_mscnt_delta FINAL=%d", g_total_mscnt_delta);
        *step_ptr += mscnt_diff_actual;
                
        // Sync to disk
        msync((void*)step_ptr, sizeof(int32_t), MS_ASYNC);
    }else{
        log_error("Failed to read MSCNT register at end of motion");
    }
    
    // Clean up memory mapping
    if (step_ptr != MAP_FAILED) {
        msync((void*)step_ptr, sizeof(uint32_t), MS_SYNC); // Final sync
        munmap((void*)step_ptr, sizeof(uint32_t));
    }
    if (fd != -1) {
        close(fd);
    }
    
    log_debug("Position monitoring thread stopped");
    return NULL;
}

// ============================================================================
// POSITION MONITORING FUNCTIONS
// ============================================================================

bool tmc_position_monitor_init(tmc_position_monitor_t *monitor, TMC2209_t *driver, uint16_t microstep_resolution, bool direction) {
    if (!monitor || !driver) {
        log_error("Invalid parameters for position monitor init");
        return false;
    }
    
    memset(monitor, 0, sizeof(tmc_position_monitor_t));
    monitor->driver = driver;
    monitor->microstep_resolution = microstep_resolution;
    monitor->direction = direction;
    monitor->monitoring_active = false;
    monitor->position_valid = false;
    monitor->position_error = 0;
    monitor->last_step_count = 0;
    monitor->last_mscnt = 0;
    monitor->last_update_time_us = 0;
    monitor->last_step_count_update_time_us = 0;
    
    // Initialize polling configuration
    monitor->poll_interval_steps = microstep_resolution;  // Poll every full step (microstep_resolution microsteps)
    
    // Initialize error accumulation tracking
    monitor->last_expected_step_count = 0;
    monitor->last_expected_mscnt = 0;
    monitor->total_step_error = 0;
    monitor->error_check_count = 0;
    monitor->average_step_error = 0.0f;
    
    // Initialize initial position tracking
    monitor->initial_mscnt = 0;
    monitor->initial_step_count = 0;
    
    if (pthread_mutex_init(&monitor->data_mutex, NULL) != 0) {
        log_error("Failed to initialize position monitor mutex");
        return false;
    }
    
    log_debug("Position monitor initialized successfully (polling every %u microsteps)", monitor->poll_interval_steps);
    return true;
}

void tmc_position_monitor_deinit(tmc_position_monitor_t *monitor) {
    if (!monitor) {
        return;
    }
    
    // Stop monitoring
    tmc_position_monitor_stop(monitor);
    
    // Clean up mutex
    pthread_mutex_destroy(&monitor->data_mutex);
    
    log_debug("Position monitor deinitialized");
}

bool tmc_position_monitor_start(tmc_position_monitor_t *monitor) {
    if (!monitor || !monitor->driver) {
        log_error("Invalid position monitor or driver for start");
        return false;
    }

    // Stop any existing monitoring
    tmc_position_monitor_stop(monitor);

    // Synchronize initial step count with actual motor position (MSCNT)
    if (TMC2209_ReadRegister(monitor->driver, (TMC2209_datagram_t *)&monitor->driver->mscnt)) {
        uint16_t mscnt = monitor->driver->mscnt.reg.mscnt;
        
        // Use the actual MSCNT as the initial position reference
        // The motor is at this MSCNT position, so we start from step 0
        uint32_t step_count = 0; // Start from step 0, use MSCNT as absolute position reference
        
        if (pthread_mutex_lock(&monitor->data_mutex) == 0) {
            // Store initial position for relative error calculation
            monitor->initial_step_count = step_count;
            monitor->initial_mscnt = mscnt;
            
            // Initialize error tracking with current values
            monitor->last_expected_step_count = step_count;
            monitor->last_expected_mscnt = mscnt;
            
            pthread_mutex_unlock(&monitor->data_mutex);
        }
        log_debug("Position monitor synchronized: MSCNT=%u, calculated step count=%u, polling every %u microsteps", 
               mscnt, step_count, monitor->poll_interval_steps);
        log_debug("Initial MSCNT=%u corresponds to step %u (microstep resolution=%u)", 
               mscnt, step_count, monitor->microstep_resolution);
        log_debug("Motion direction: %s", monitor->direction ? "clockwise" : "counter-clockwise");
        
        // Calculate what the expected MSCNT should be for step 0
        uint16_t expected_mscnt_at_zero = calculate_expected_mscnt(0, monitor->microstep_resolution, monitor->direction);
        log_debug("Expected MSCNT at step 0: %u, Offset: %d", 
               expected_mscnt_at_zero, (int16_t)mscnt - (int16_t)expected_mscnt_at_zero);
        
        // Test MSCNT calculation for first few steps
        for (int i = 1; i <= 3; i++) {
            uint16_t test_mscnt = calculate_expected_mscnt(i * monitor->microstep_resolution, monitor->microstep_resolution, monitor->direction);
            log_debug("Expected MSCNT at step %u: %u", i * monitor->microstep_resolution, test_mscnt);
        }
    } else {
        log_warn("Failed to read MSCNT register for initial synchronization");
    }

    // Start monitoring
    monitor->monitoring_active = true;

    // Create monitoring thread
    int result = pthread_create(&monitor->monitor_thread, NULL, position_monitor_thread_function, monitor);
    if (result != 0) {
        log_error("Failed to create position monitoring thread (error %d)", result);
        monitor->monitoring_active = false;
        return false;
    }

    log_debug("Position monitoring started successfully");
    return true;
}

void tmc_position_monitor_stop(tmc_position_monitor_t *monitor) {
    if (!monitor) {
        return;
    }
    
    // Signal thread to exit
    monitor->monitoring_active = false;
    
    // Wait for thread to finish (with timeout)
    if (monitor->monitor_thread) {
        void *thread_result;
        int result = pthread_join(monitor->monitor_thread, &thread_result);
        if (result != 0) {
            log_warn("Failed to join position monitoring thread (error %d)", result);
        }
        monitor->monitor_thread = 0;
    }
    
    log_debug("Position monitoring stopped");
}

void tmc_position_monitor_update_step_count(tmc_position_monitor_t *monitor, uint32_t step_count) {
    if (!monitor) {
        return;
    }
    
    if (pthread_mutex_lock(&monitor->data_mutex) == 0) {
        monitor->last_step_count = step_count;
        monitor->last_step_count_update_time_us = get_time_us();
        pthread_mutex_unlock(&monitor->data_mutex);
    }
}

bool tmc_position_monitor_get_status(tmc_position_monitor_t *monitor, uint32_t *step_count, uint16_t *mscnt, int16_t *error, uint64_t *last_update_time) {
    if (!monitor || !step_count || !mscnt || !error || !last_update_time) {
        return false;
    }
    
    if (pthread_mutex_lock(&monitor->data_mutex) == 0) {
        *step_count = monitor->last_step_count;
        *mscnt = monitor->last_mscnt;
        *error = monitor->position_error;
        *last_update_time = monitor->last_update_time_us;
        bool valid = monitor->position_valid;
        pthread_mutex_unlock(&monitor->data_mutex);
        return valid;
    }
    
    return false;
}

bool tmc_position_monitor_get_error_stats(tmc_position_monitor_t *monitor, int32_t *total_error, uint32_t *check_count, float *average_error) {
    if (!monitor || !total_error || !check_count || !average_error) {
        return false;
    }
    
    if (pthread_mutex_lock(&monitor->data_mutex) == 0) {
        *total_error = monitor->total_step_error;
        *check_count = monitor->error_check_count;
        *average_error = monitor->average_step_error;
        pthread_mutex_unlock(&monitor->data_mutex);
        return true;
    }
    
    return false;
}

// ============================================================================
// MSCNT POSITION TRACKING FUNCTIONS
// ============================================================================
 
uint16_t calculate_expected_mscnt(uint32_t step_count, uint16_t microstep_resolution, bool direction) {
    // Suppress unused parameter warning
    (void)direction;
    
    // MSCNT changes by 256 per FULL STEP, not per microstep
    // Convert microsteps to full steps first using floating point for accuracy
    float full_steps = (float)step_count / (float)microstep_resolution;
    uint32_t total_mscnt = (uint32_t)(full_steps * 256.0f);
    
    // Apply wrap-around (MSCNT is 10-bit, so 0-1023)
    total_mscnt = total_mscnt % 1024;

    int32_t signed_mscnt = -(int32_t)total_mscnt;
    // Handle wrap-around for negative values
    while (signed_mscnt < 0) {
        signed_mscnt += 1024;
    }
    total_mscnt = (uint32_t)signed_mscnt;
 

    // For clockwise (direction = true), we keep the positive change as-is
    
    return (uint16_t)total_mscnt;
}

bool check_mscnt_position(TMC2209_t *driver, uint32_t step_count, uint16_t microstep_resolution) {
    if (!driver) {
        return false;
    }
    
    // Calculate expected MSCNT value (assume clockwise for this function)
    uint16_t expected_mscnt = calculate_expected_mscnt(step_count, microstep_resolution, true);
    
    // Read actual MSCNT from TMC2209
    if (!TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->mscnt)) {
        log_error("Failed to read MSCNT register");
        return false;
    }
    
    uint16_t actual_mscnt = driver->mscnt.reg.mscnt;
    
    // Compare actual vs expected (allow small tolerance for timing differences)
    int16_t difference = (int16_t)actual_mscnt - (int16_t)expected_mscnt;
    
    // Handle wrap-around in comparison
    if (difference > 512) {
        difference -= 1024;
    } else if (difference < -512) {
        difference += 1024;
    }
    
    // Check if difference is within tolerance (allow ±2 microsteps)
    uint16_t tolerance = 2 * (256 / microstep_resolution);
    bool position_valid = (abs(difference) <= tolerance);
    
    // Print position check results
    log_debug("  Position Check: Software=%u steps, MSCNT=%u (expected=%u) %s", 
           step_count, actual_mscnt, expected_mscnt, 
           position_valid ? "OK" : "ERROR");
    
    if (!position_valid) {
        log_warn("Position discrepancy detected! Difference: %d microsteps", difference);
    }
    
    return position_valid;
} 