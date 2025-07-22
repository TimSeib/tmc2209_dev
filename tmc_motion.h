/*
 * tmc_motion.h - S-curve motion control for TMC2209
 *
 * This file provides S-curve motion control using timer interrupts
 * for precise step generation. Takes RPM and revolutions as input.
 *
 * v1.0.0 / 2024-12-19
 */

#ifndef _TMC_MOTION_H_
#define _TMC_MOTION_H_

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <time.h>
#include "tmc_gpio.h"
#include "tmc2209.h"

// ============================================================================
// CONSTANTS
// ============================================================================

#define STEPS_PER_REVOLUTION    200     // Full steps per revolution
#define SECONDS_PER_MINUTE      60.0f
#define MIN_TIMER_PERIOD_US     10      // Minimum timer period (10μs)
#define MAX_TIMER_PERIOD_US     1000000 // Maximum timer period (1 second)

// S-curve phases
typedef enum {
    PHASE_ACCEL_INCREASE = 0,   // Increasing acceleration (jerk positive)
    PHASE_ACCEL_CONSTANT,       // Constant acceleration
    PHASE_ACCEL_DECREASE,       // Decreasing acceleration (jerk negative)
    PHASE_UNIFORM,              // Constant speed
    PHASE_DECEL_INCREASE,       // Increasing deceleration (jerk negative)
    PHASE_DECEL_CONSTANT,       // Constant deceleration
    PHASE_DECEL_DECREASE,       // Decreasing deceleration (jerk positive)
    PHASE_COMPLETE              // Motion finished
} s_curve_phase_t;

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
    int32_t total_step_error;           // Cumulative step error
    uint32_t error_check_count;         // Number of error checks performed
    float average_step_error;           // Average error per check
    
    // Initial position offset for relative error calculation
    uint16_t initial_mscnt;             // Initial MSCNT value at start
    uint32_t initial_step_count;        // Initial step count at start
} tmc_position_monitor_t;

// ============================================================================
// S-CURVE MOTION STRUCTURE
// ============================================================================

typedef struct {
   // GPIO and driver context
   tmc_gpio_context_t *gpio_ctx;
   TMC2209_t *tmc_driver;
   
   // Input parameters
   float target_angle_degrees;
   float max_speed_rpm;
   float max_accel_hz_per_sec;
   float jerk_rate_hz_per_sec2;
   float start_speed_hz;
   float start_accel_hz_per_sec;
   uint32_t gear_ratio;
   bool direction;
   uint16_t microstep_resolution;
   
   // Calculated parameters
   uint32_t steps_per_rev;
   uint32_t total_steps;
   float max_speed_hz;
   float max_accel_hz_per_sec2;
   float jerk_rate_hz_per_sec3;
   
   // S-curve phase boundaries
   uint32_t accel_phase_steps;
   uint32_t constant_phase_steps;
   uint32_t decel_phase_steps;
   
   // Sub-phase boundaries for detailed tracking
   uint32_t accel_jerk_increase_steps;
   uint32_t accel_constant_steps;
   uint32_t accel_jerk_decrease_steps;
   uint32_t decel_jerk_increase_steps;
   uint32_t decel_constant_steps;
   uint32_t decel_jerk_decrease_steps;
   
   // Motion state
   uint32_t current_step;
   uint32_t steps_completed;
   s_curve_phase_t current_phase;
   float current_speed_hz;
   float current_accel_hz_per_sec;
   uint32_t current_timer_period_us;
   
   // Threading
   pthread_t motion_thread;
   bool thread_running;
   bool thread_should_exit;
   bool motion_active;
   bool motion_complete;
   
   // Timing
   uint64_t start_time_us;
   uint64_t last_step_time_us;
   uint64_t next_step_time_us;
   
   // Parameter validation
   bool parameters_valid;
   char validation_message[256];
   
   // Position monitoring
   tmc_position_monitor_t position_monitor;
   
} tmc_motion_s_curve_t;
 
 // ============================================================================
 // POSITION MONITORING STRUCTURES AND FUNCTIONS
 // ============================================================================

// Position monitoring function declarations
bool tmc_position_monitor_init(tmc_position_monitor_t *monitor, TMC2209_t *driver, uint16_t microstep_resolution);
void tmc_position_monitor_deinit(tmc_position_monitor_t *monitor);
bool tmc_position_monitor_start(tmc_position_monitor_t *monitor);
void tmc_position_monitor_stop(tmc_position_monitor_t *monitor);
void tmc_position_monitor_update_step_count(tmc_position_monitor_t *monitor, uint32_t step_count);
bool tmc_position_monitor_get_status(tmc_position_monitor_t *monitor, uint32_t *step_count, uint16_t *mscnt, int16_t *error, uint64_t *last_update_time);
bool tmc_position_monitor_get_error_stats(tmc_position_monitor_t *monitor, int32_t *total_error, uint32_t *check_count, float *average_error);
 
 // ============================================================================
 // FUNCTION DECLARATIONS
 // ============================================================================
 
 // S-curve motion control functions
 bool tmc_motion_s_curve_init(tmc_motion_s_curve_t *motion, tmc_gpio_context_t *gpio_ctx, TMC2209_t *tmc_driver);
 void tmc_motion_s_curve_deinit(tmc_motion_s_curve_t *motion);
 bool tmc_motion_s_curve_start(tmc_motion_s_curve_t *motion,
                              float target_angle_degrees,
                              float max_speed_rpm,
                              float max_accel_hz_per_sec,
                              float jerk_rate_hz_per_sec2,
                              float start_speed_hz,
                              float start_accel_hz_per_sec,
                              uint32_t gear_ratio,
                              bool direction,
                              uint16_t microstep_resolution);
 bool tmc_motion_s_curve_stop(tmc_motion_s_curve_t *motion);
 bool tmc_motion_s_curve_is_complete(tmc_motion_s_curve_t *motion);
 float tmc_motion_s_curve_get_progress(tmc_motion_s_curve_t *motion);
 const char* tmc_motion_s_curve_get_status(tmc_motion_s_curve_t *motion);
 bool tmc_motion_s_curve_wait_for_completion(tmc_motion_s_curve_t *motion, uint32_t timeout_ms);
 
 // S-curve calculation functions
 bool validate_s_curve_parameters(tmc_motion_s_curve_t *motion);
 bool calculate_s_curve_profile(tmc_motion_s_curve_t *motion);
 float calculate_speed_at_step(tmc_motion_s_curve_t *motion, uint32_t step_position);
 uint32_t speed_to_timer_period(float speed_hz);
 
 // MSCNT position tracking functions
 uint16_t calculate_expected_mscnt(uint32_t step_count, uint16_t microstep_resolution);
 bool check_mscnt_position(TMC2209_t *driver, uint32_t step_count, uint16_t microstep_resolution);
 
 #endif // _TMC_MOTION_SIMPLE_H_