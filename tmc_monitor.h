/*
 * tmc_monitor.h - Position monitoring for TMC2209
 *
 * This file provides position monitoring using MSCNT register polling
 * for real-time position tracking and error detection.
 *
 * v1.0.0 / 2024-12-19
 */

#ifndef _TMC_MONITOR_H_
#define _TMC_MONITOR_H_

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <time.h>
#include "tmc2209.h"

// ============================================================================
// POSITION MONITORING STRUCTURES
// ============================================================================

/**
 * @brief Position monitoring structure for non-blocking MSCNT tracking
 */
typedef struct {
    TMC2209_t *driver;
    uint16_t microstep_resolution;
    bool monitoring_active;
    pthread_t monitor_thread;
    pthread_mutex_t data_mutex;
    uint32_t last_step_count;
    uint16_t last_mscnt;
    bool position_valid;
    int16_t position_error;
    uint64_t last_update_time_us;
    uint64_t last_step_count_update_time_us;  // Timestamp when step count was last updated
    
    // Polling configuration
    uint32_t poll_interval_steps;     // How many steps to wait before next poll (microstep_resolution)
    
    // Error accumulation tracking
    uint32_t last_expected_step_count;  // Expected step count at last poll
    uint16_t last_expected_mscnt;       // Expected MSCNT at last poll
    float total_step_error;           // Cumulative step error
    uint32_t error_check_count;         // Number of error checks performed
    float average_step_error;           // Average error per check
    
    // Initial position offset for relative error calculation
    uint16_t initial_mscnt;             // Initial MSCNT value at start
    uint32_t initial_step_count;        // Initial step count at start
    
    // Motion direction for MSCNT calculation
    bool direction;                     // true = clockwise, false = counter-clockwise
} tmc_position_monitor_t;

// ============================================================================
// POSITION MONITORING FUNCTION DECLARATIONS
// ============================================================================

// Position monitoring function declarations
bool tmc_position_monitor_init(tmc_position_monitor_t *monitor, TMC2209_t *driver, uint16_t microstep_resolution, bool direction);
void tmc_position_monitor_deinit(tmc_position_monitor_t *monitor);
bool tmc_position_monitor_start(tmc_position_monitor_t *monitor);
void tmc_position_monitor_stop(tmc_position_monitor_t *monitor);
void tmc_position_monitor_update_step_count(tmc_position_monitor_t *monitor, uint32_t step_count);
bool tmc_position_monitor_get_status(tmc_position_monitor_t *monitor, uint32_t *step_count, uint16_t *mscnt, int16_t *error, uint64_t *last_update_time);
bool tmc_position_monitor_get_error_stats(tmc_position_monitor_t *monitor, int32_t *total_error, uint32_t *check_count, float *average_error);

// ============================================================================
// MSCNT POSITION TRACKING FUNCTION DECLARATIONS
// ============================================================================

// MSCNT position tracking functions
uint16_t calculate_expected_mscnt(uint32_t step_count, uint16_t microstep_resolution, bool direction);
bool check_mscnt_position(TMC2209_t *driver, uint32_t step_count, uint16_t microstep_resolution);

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Global variable for MSCNT delta tracking
extern int32_t g_total_mscnt_delta;

#endif // _TMC_MONITOR_H_ 