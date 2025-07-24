/*
 * test_s_curve_calculations.c - S-curve parameter validation and calculation test
 *
 * This program validates input parameters for S-curve motion profiles and
 * performs all mathematical calculations to verify the approach before
 * implementing motor movement.
 *
 * v1.0.0 / 2024-12-19
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

// ============================================================================
// CONSTANTS AND DEFINITIONS
// ============================================================================

#define STEPS_PER_REVOLUTION    200     // Full steps per revolution
#define SECONDS_PER_MINUTE      60.0f
#define PI                      3.14159265359f

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

// S-curve motion profile structure
typedef struct {
    // Input parameters
    float target_angle_degrees;     // Target angle in degrees (output shaft)
    float max_speed_rpm;            // Maximum speed in RPM (input shaft)
    float max_accel_hz_per_sec;     // Maximum acceleration in Hz/sec (input shaft)
    float jerk_rate_hz_per_sec2;    // Jerk rate in Hz/sec² (input shaft)
    float start_speed_hz;           // Starting speed in Hz (input shaft)
    float start_accel_hz_per_sec;   // Starting acceleration in Hz/sec (input shaft)
    float gear_ratio;            // Gear reduction ratio
    uint16_t microstep_resolution;  // Microstep resolution (1, 2, 4, 8, 16, 32, 64, 128, 256)
    
    // Calculated parameters (converted to Hz-based units)
    uint32_t total_steps;           // Total steps for the move (N)
    uint32_t steps_per_rev;         // Steps per revolution (calculated)
    float max_speed_hz;             // Maximum speed in Hz (calculated from RPM)
    float max_accel_hz_per_sec2;    // Maximum acceleration in Hz/sec² (calculated)
    float jerk_rate_hz_per_sec3;    // Jerk rate in Hz/sec³ (calculated)
    
    // Phase boundaries (in steps)
    uint32_t accel_phase_steps;     // Steps in acceleration phase (n1)
    uint32_t constant_phase_steps;  // Steps in constant speed phase (n2)
    uint32_t decel_phase_steps;     // Steps in deceleration phase (n3)
    
    // Timing parameters (used to estimate motion time)
    float total_time_seconds;       // Total motion time
    float accel_time_seconds;       // Time in acceleration phase
    float constant_time_seconds;    // Time in constant speed phase
    float decel_time_seconds;       // Time in deceleration phase
    
    // Validation flags
    bool parameters_valid;          // Whether all parameters are reasonable
    char validation_message[256];   // Validation error message buffer
} s_curve_profile_t;

// ============================================================================
// PARAMETER VALIDATION FUNCTIONS
// ============================================================================

/**
 * @brief Validate input parameters for S-curve motion
 * 
 * @param profile Profile structure to validate
 * @return true if parameters are valid, false otherwise
 */
bool validate_s_curve_parameters(s_curve_profile_t *profile) {
    if (!profile) {
        return false;
    }
    
    // Reset validation
    profile->parameters_valid = true;
    strcpy(profile->validation_message, "Parameters are valid");
    
    // Check target angle
    if (profile->target_angle_degrees <= 0.0f || profile->target_angle_degrees > 3600.0f) {
        profile->parameters_valid = false;
        strcpy(profile->validation_message, "Target angle must be between 0.1 and 3600 degrees");
        return false;
    }
    
    // Check maximum speed
    if (profile->max_speed_rpm <= 0.0f || profile->max_speed_rpm > 10000.0f) {
        profile->parameters_valid = false;
        strcpy(profile->validation_message, "Maximum speed must be between 0.1 and 10000 RPM");
        return false;
    }
    
    // Check maximum acceleration
    if (profile->max_accel_hz_per_sec <= 0.0f || profile->max_accel_hz_per_sec > 100000.0f) {
        profile->parameters_valid = false;
        strcpy(profile->validation_message, "Maximum acceleration must be between 0.1 and 100000 Hz/sec");
        return false;
    }
    
    // Check jerk rate
    if (profile->jerk_rate_hz_per_sec2 <= 0.0f || profile->jerk_rate_hz_per_sec2 > 1000000.0f) {
        profile->parameters_valid = false;
        strcpy(profile->validation_message, "Jerk rate must be between 0.1 and 1000000 Hz/sec²");
        return false;
    }
    
    // Check starting speed
    if (profile->start_speed_hz < 0.0f || profile->start_speed_hz > 10000.0f) {
        profile->parameters_valid = false;
        strcpy(profile->validation_message, "Starting speed must be between 0 and 10000 Hz");
        return false;
    }
    
    // Check starting acceleration
    if (profile->start_accel_hz_per_sec < 0.0f || profile->start_accel_hz_per_sec > profile->max_accel_hz_per_sec) {
        profile->parameters_valid = false;
        strcpy(profile->validation_message, "Starting acceleration must be between 0 and max acceleration");
        return false;
    }
    
    // Check gear ratio
    if (profile->gear_ratio == 0 || profile->gear_ratio > 10000) {
        profile->parameters_valid = false;
        strcpy(profile->validation_message, "Gear ratio must be between 1 and 10000");
        return false;
    }
    
    // Check microstep resolution
    uint16_t valid_microsteps[] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
    bool valid_microstep = false;
    size_t num_microsteps = sizeof(valid_microsteps)/sizeof(valid_microsteps[0]);
    for (size_t i = 0; i < num_microsteps; i++) {
        if (profile->microstep_resolution == valid_microsteps[i]) {
            valid_microstep = true;
            break;
        }
    }
    if (!valid_microstep) {
        profile->parameters_valid = false;
        strcpy(profile->validation_message, "Microstep resolution must be 1, 2, 4, 8, 16, 32, 64, 128, or 256");
        return false;
    }
    
    // Check physical constraints
    float output_speed_rpm = profile->max_speed_rpm / profile->gear_ratio;
    if (output_speed_rpm > 1000.0f) {
        profile->parameters_valid = false;
        strcpy(profile->validation_message, "Output shaft speed too high (>1000 RPM)");
        return false;
    }
    
    // Check if motion is feasible (step-based validation)
    // Calculate if we can reach max speed with given acceleration and jerk
    float max_speed_hz = (profile->max_speed_rpm * profile->steps_per_rev) / SECONDS_PER_MINUTE;
    float max_accel_per_step = profile->max_accel_hz_per_sec;
    float jerk_per_step2 = profile->jerk_rate_hz_per_sec2;
    
    // Calculate steps needed to reach max acceleration
    uint32_t jerk_phase_steps = (uint32_t)(max_accel_per_step / jerk_per_step2); // equation for n1/n2
    
    // Calculate speed after jerk phase (starting from initial speed and acceleration)
    float speed_after_jerk = profile->start_speed_hz + 
                            (profile->start_accel_hz_per_sec * jerk_phase_steps) +
                            (jerk_per_step2 * jerk_phase_steps * jerk_phase_steps) / 2.0f;
    
    // Check if we can reach max speed
    if (speed_after_jerk >= max_speed_hz) {
        // We can reach max speed in jerk phase alone
        profile->parameters_valid = true;
    } else {
        // Need constant acceleration phase
        float speed_increase_needed = max_speed_hz - speed_after_jerk;
        uint32_t constant_accel_steps = (uint32_t)(speed_increase_needed / max_accel_per_step);
        
        // Check if total acceleration steps are reasonable (not too many)
        uint32_t total_accel_steps = jerk_phase_steps + constant_accel_steps;
        if (total_accel_steps > 100000) {
            profile->parameters_valid = false;
            strcpy(profile->validation_message, "Too many acceleration steps (>100k) - increase acceleration or jerk rate");
            return false;
        }
    }
    
    return true;
}

// ============================================================================
// CALCULATION FUNCTIONS
// ============================================================================

/**
 * @brief Calculate S-curve profile parameters
 * 
 * @param profile Profile structure to calculate
 * @return true on success, false on failure
 */
bool calculate_s_curve_profile(s_curve_profile_t *profile) {
    if (!profile || !profile->parameters_valid) {
        return false;
    }
    
    // Calculate steps per revolution
    profile->steps_per_rev = STEPS_PER_REVOLUTION * profile->microstep_resolution;
    
    // Calculate total steps needed
    float target_revolutions = profile->target_angle_degrees / 360.0f;
    float input_revolutions = target_revolutions * profile->gear_ratio;
    profile->total_steps = (uint32_t)(input_revolutions * profile->steps_per_rev);
    
    // Calculate S-curve parameters using Hz-based units
    // Convert RPM to Hz for step frequency calculations
    
    // Maximum speed in Hz (step frequency)
    profile->max_speed_hz = (profile->max_speed_rpm * profile->steps_per_rev) / SECONDS_PER_MINUTE;
    
    // Maximum acceleration in Hz/sec (step frequency acceleration)
    profile->max_accel_hz_per_sec2 = profile->max_accel_hz_per_sec;
    
    // Jerk rate in Hz/sec² (step frequency jerk)
    profile->jerk_rate_hz_per_sec3 = profile->jerk_rate_hz_per_sec2;
    
    // Calculate step-based S-curve phase boundaries
    // Using step-based acceleration and jerk rates
    
    // Convert acceleration and jerk to per-step units
    float max_accel_per_step = profile->max_accel_hz_per_sec2;  // Hz/step
    float jerk_per_step2 = profile->jerk_rate_hz_per_sec3;      // Hz/step²
    
    // Phase 1: Jerk phase - acceleration increases linearly from 0 to max_accel
    // Steps needed to reach max acceleration: max_accel / jerk_rate
    uint32_t jerk_phase_steps = (uint32_t)(max_accel_per_step / jerk_per_step2); // n1
    
    // Phase 2: Constant acceleration phase
    // Calculate how many steps needed to reach max speed from current speed
    
    // Speed after jerk phase (starting from initial conditions)
    float speed_after_jerk = profile->start_speed_hz + 
                            (profile->start_accel_hz_per_sec * jerk_phase_steps) +
                            (jerk_per_step2 * jerk_phase_steps * jerk_phase_steps) / 2.0f;
    
    // Steps needed in constant acceleration phase to reach max speed
    float speed_increase_needed = profile->max_speed_hz - speed_after_jerk;
    uint32_t constant_accel_steps = (uint32_t)(speed_increase_needed / max_accel_per_step);
    
    // Total acceleration steps
    uint32_t total_accel_steps = jerk_phase_steps + constant_accel_steps;
    
    // Check if we need a constant speed phase
    if (2 * total_accel_steps >= profile->total_steps) {
        // Triangular profile - no constant speed phase
        profile->constant_phase_steps = 0;
        profile->accel_phase_steps = profile->total_steps / 2;
        profile->decel_phase_steps = profile->total_steps / 2;
        
        // Recalculate for triangular profile
        uint32_t triangular_accel_steps = profile->total_steps / 2;
        
        // For triangular profile, we need to solve:
        // triangular_accel_steps = jerk_phase_steps + constant_accel_steps
        // where jerk_phase_steps = max_accel / jerk_rate
        // and constant_accel_steps = (max_speed - speed_after_jerk) / max_accel
        
        // This is a quadratic equation that needs to be solved
        // For now, use approximation
        profile->accel_phase_steps = triangular_accel_steps;
        profile->decel_phase_steps = triangular_accel_steps;
        
    } else {
        // Trapezoidal profile - includes constant speed phase
        profile->accel_phase_steps = total_accel_steps;
        profile->decel_phase_steps = total_accel_steps;
        profile->constant_phase_steps = profile->total_steps - (2 * total_accel_steps);
    }
    
    // Calculate estimated times (for display only)
    profile->accel_time_seconds = (float)profile->accel_phase_steps / profile->max_speed_hz;
    profile->decel_time_seconds = (float)profile->decel_phase_steps / profile->max_speed_hz;
    profile->constant_time_seconds = (float)profile->constant_phase_steps / profile->max_speed_hz;
    profile->total_time_seconds = profile->accel_time_seconds + profile->constant_time_seconds + profile->decel_time_seconds;
    
    return true;
}

/**
 * @brief Calculate speed at a specific step position (step-based)
 * 
 * @param profile Profile structure
 * @param step_position Current step position
 * @return Speed in Hz
 */
float calculate_speed_at_step(s_curve_profile_t *profile, uint32_t step_position) {
    if (!profile || step_position >= profile->total_steps) {
        return 0.0f;
    }
    
    float current_speed = profile->start_speed_hz; // Starting speed in Hz
    float current_accel = profile->start_accel_hz_per_sec; // Starting acceleration in Hz/step
    float max_accel_per_step = profile->max_accel_hz_per_sec2;
    float jerk_per_step2 = profile->jerk_rate_hz_per_sec3;
    
    // Determine which phase we're in
    if (step_position < profile->accel_phase_steps) {
        // Acceleration phase
        uint32_t steps_in_phase = step_position;
        
        // Calculate jerk phase steps
        uint32_t jerk_phase_steps = (uint32_t)(max_accel_per_step / jerk_per_step2);
        
        if (steps_in_phase < jerk_phase_steps) {
            // Phase 1: Jerk phase - acceleration increases linearly from start_accel to max_accel
            // UNUSED:
            // float accel_at_step = current_accel + jerk_per_step2 * steps_in_phase;
            float speed = current_speed + (current_accel * steps_in_phase) + (jerk_per_step2 * steps_in_phase * steps_in_phase) / 2.0f;
            return (speed > profile->max_speed_hz) ? profile->max_speed_hz : speed;
        } else {
            // Phase 2: Constant acceleration phase
            uint32_t steps_in_constant_accel = steps_in_phase - jerk_phase_steps;
            float speed_after_jerk = current_speed + 
                                   (current_accel * jerk_phase_steps) + 
                                   (jerk_per_step2 * jerk_phase_steps * jerk_phase_steps) / 2.0f;
            float speed = speed_after_jerk + max_accel_per_step * steps_in_constant_accel;
            return (speed > profile->max_speed_hz) ? profile->max_speed_hz : speed;
        }
        
    } else if (step_position < profile->accel_phase_steps + profile->constant_phase_steps) {
        // Constant speed phase
        return profile->max_speed_hz;
        
    } else {
        // Deceleration phase (mirror of acceleration)
        uint32_t steps_from_end = profile->total_steps - step_position;
        
        // Calculate jerk phase steps
        uint32_t jerk_phase_steps = (uint32_t)(max_accel_per_step / jerk_per_step2);
        
        if (steps_from_end < jerk_phase_steps) {
            // Final jerk phase - acceleration decreases linearly
            float speed = (jerk_per_step2 * steps_from_end * steps_from_end) / 2.0f;
            return (speed > profile->max_speed_hz) ? profile->max_speed_hz : speed;
        } else {
            // Constant deceleration phase
            uint32_t steps_in_constant_decel = steps_from_end - jerk_phase_steps;
            float speed_after_jerk = (jerk_per_step2 * jerk_phase_steps * jerk_phase_steps) / 2.0f;
            float speed = speed_after_jerk + max_accel_per_step * steps_in_constant_decel;
            return (speed > profile->max_speed_hz) ? profile->max_speed_hz : speed;
        }
    }
}

/**
 * @brief Calculate timer period from speed
 * 
 * @param speed_hz Speed in Hz (step frequency)
 * @return Timer period in microseconds
 */
uint32_t speed_to_timer_period(float speed_hz) {
    if (speed_hz <= 0.0f) {
        return 1000000; // 1 second if stopped
    }
    
    uint32_t period_us = (uint32_t)(1000000.0f / speed_hz);
    
    // Ensure minimum period for hardware constraints
    if (period_us < 10) {
        period_us = 10; // 10μs minimum
    }
    
    return period_us;
}

// ============================================================================
// INPUT AND DISPLAY FUNCTIONS
// ============================================================================

/**
 * @brief Get user input for S-curve parameters
 * 
 * @param profile Profile structure to fill
 */
void get_user_input(s_curve_profile_t *profile) {
    printf("\n=== S-Curve Motion Profile Calculator ===\n\n");
    
    printf("Enter target angle (degrees, output shaft): ");
    scanf("%f", &profile->target_angle_degrees);
    
    printf("Enter maximum speed (RPM, input shaft): ");
    scanf("%f", &profile->max_speed_rpm);
    
    printf("Enter maximum acceleration (Hz/sec, input shaft): ");
    scanf("%f", &profile->max_accel_hz_per_sec);
    
    printf("Enter jerk rate (Hz/sec², input shaft): ");
    scanf("%f", &profile->jerk_rate_hz_per_sec2);
    
    printf("Enter starting speed (Hz, input shaft): ");
    scanf("%f", &profile->start_speed_hz);
    
    printf("Enter starting acceleration (Hz/sec, input shaft): ");
    scanf("%f", &profile->start_accel_hz_per_sec);
    
    printf("Enter gear ratio (e.g., 100 for 100:1 reduction): ");
    scanf("%f", &profile->gear_ratio);
    
    printf("Enter microstep resolution (1, 2, 4, 8, 16, 32, 64, 128, 256): ");
    scanf("%hu", &profile->microstep_resolution);
}

/**
 * @brief Display S-curve profile results
 * 
 * @param profile Profile structure to display
 */
void display_profile_results(s_curve_profile_t *profile) {
    printf("\n=== S-Curve Profile Results ===\n\n");
    
    if (!profile->parameters_valid) {
        printf("❌ VALIDATION FAILED: %s\n\n", profile->validation_message);
        return;
    }
    
    printf("✅ PARAMETERS VALIDATED\n\n");
    
    printf("Input Parameters:\n");
    printf("- Target angle: %.2f degrees (output shaft)\n", profile->target_angle_degrees);
    printf("- Maximum speed: %.2f RPM (input shaft)\n", profile->max_speed_rpm);
    printf("- Maximum acceleration: %.2f Hz/sec (input shaft)\n", profile->max_accel_hz_per_sec);
    printf("- Jerk rate: %.2f Hz/sec² (input shaft)\n", profile->jerk_rate_hz_per_sec2);
    printf("- Starting speed: %.2f Hz (input shaft)\n", profile->start_speed_hz);
    printf("- Starting acceleration: %.2f Hz/sec (input shaft)\n", profile->start_accel_hz_per_sec);
    printf("- Gear ratio: %f:1\n", profile->gear_ratio);
    printf("- Microstep resolution: 1/%u\n", profile->microstep_resolution);
    
    printf("\nCalculated Parameters:\n");
    printf("- Steps per revolution: %u\n", profile->steps_per_rev);
    printf("- Total steps: %u\n", profile->total_steps);
    printf("- Output shaft speed: %.2f RPM\n", profile->max_speed_rpm / profile->gear_ratio);
    
    printf("\nPhase Breakdown:\n");
    printf("- Acceleration phase: %u steps (%.3f sec)\n", 
           profile->accel_phase_steps, profile->accel_time_seconds);
    printf("- Constant speed phase: %u steps (%.3f sec)\n", 
           profile->constant_phase_steps, profile->constant_time_seconds);
    printf("- Deceleration phase: %u steps (%.3f sec)\n", 
           profile->decel_phase_steps, profile->decel_time_seconds);
    printf("- Total motion time: %.3f seconds\n", profile->total_time_seconds);
    
    printf("\nSpeed Analysis:\n");
    printf("- Maximum speed: %.2f Hz (step frequency)\n", profile->max_speed_hz);
    printf("- Minimum timer period: %u μs\n", speed_to_timer_period(profile->max_speed_hz));
    
    // Show speed at key points
    printf("\nSpeed at Key Points:\n");
    if (profile->accel_phase_steps > 0) {
        float speed_25pct = calculate_speed_at_step(profile, profile->accel_phase_steps / 4);
        float speed_50pct = calculate_speed_at_step(profile, profile->accel_phase_steps / 2);
        float speed_75pct = calculate_speed_at_step(profile, 3 * profile->accel_phase_steps / 4);
        printf("- 25%% through acceleration: %.2f Hz\n", speed_25pct);
        printf("- 50%% through acceleration: %.2f Hz\n", speed_50pct);
        printf("- 75%% through acceleration: %.2f Hz\n", speed_75pct);
    }
    
    if (profile->constant_phase_steps > 0) {
        float speed_constant = calculate_speed_at_step(profile, profile->accel_phase_steps + profile->constant_phase_steps / 2);
        printf("- Mid constant speed: %.2f Hz\n", speed_constant);
    }
    
    printf("\nProfile Type: %s\n", 
           (profile->constant_phase_steps > 0) ? "Trapezoidal (with constant speed)" : "Triangular (no constant speed)");
    
    // Show detailed step breakdown for acceleration phases
    printf("\nDetailed Step Breakdown:\n");
    float max_accel_per_step = profile->max_accel_hz_per_sec2;
    float jerk_per_step2 = profile->jerk_rate_hz_per_sec3;
    uint32_t jerk_phase_steps = (uint32_t)(max_accel_per_step / jerk_per_step2);
    uint32_t constant_accel_steps = profile->accel_phase_steps - jerk_phase_steps;
    
    printf("- Jerk phase (accel 0→max): %u steps\n", jerk_phase_steps);
    printf("- Constant acceleration phase: %u steps\n", constant_accel_steps);
    printf("- Total acceleration steps: %u steps\n", profile->accel_phase_steps);
    printf("- Constant speed phase: %u steps\n", profile->constant_phase_steps);
    printf("- Total deceleration steps: %u steps (same as acceleration)\n", profile->decel_phase_steps);
}

/**
 * @brief Interactive speed calculation
 * 
 * @param profile Profile structure
 */
void interactive_speed_calculation(s_curve_profile_t *profile) {
    if (!profile->parameters_valid) {
        return;
    }
    
    printf("\n=== Interactive Speed Calculator ===\n");
    printf("Enter step positions to calculate speed (or 0 to exit):\n");
    
    while (1) {
        uint32_t step_pos;
        printf("\nStep position (0-%u): ", profile->total_steps);
        scanf("%u", &step_pos);
        
        if (step_pos == 0) {
            break;
        }
        
        if (step_pos > profile->total_steps) {
            printf("❌ Step position exceeds total steps!\n");
            continue;
        }
        
        float speed = calculate_speed_at_step(profile, step_pos);
        uint32_t timer_period = speed_to_timer_period(speed);
        float progress = (float)step_pos / profile->total_steps * 100.0f;
        
        printf("Step %u (%.1f%%): %.2f Hz, timer period: %u μs\n", 
               step_pos, progress, speed, timer_period);
    }
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================

int main() {
    s_curve_profile_t profile = {0};
    
    // Get user input
    get_user_input(&profile);
    
    // Validate parameters
    if (!validate_s_curve_parameters(&profile)) {
        printf("\n❌ Parameter validation failed: %s\n", profile.validation_message);
        return 1;
    }
    
    // Calculate S-curve profile
    if (!calculate_s_curve_profile(&profile)) {
        printf("\n❌ Profile calculation failed!\n");
        return 1;
    }
    
    // Display results
    display_profile_results(&profile);
    
    // Interactive speed calculation
    interactive_speed_calculation(&profile);
    
    printf("\n=== Calculation Complete ===\n");
    printf("If the calculations look correct, you can proceed with motor implementation.\n");
    
    // Answer the specific question about the given parameters
    printf("\n=== Specific Parameter Analysis ===\n");
    printf("For your parameters:\n");
    printf("- Max acceleration: 2 Hz/step\n");
    printf("- Jerk rate: 0.005 Hz/step²\n");
    printf("- Starting velocity: 100 Hz\n");
    printf("- Starting acceleration: 0 Hz/step\n");
    printf("- Max velocity: 3600 Hz\n");
    
    float max_accel = 2.0f;
    float jerk_rate = 0.005f;
    float start_speed = 100.0f;
    float start_accel = 0.0f;
    float max_speed = 3600.0f;
    
    // Calculate steps for each phase
    uint32_t jerk_steps = (uint32_t)(max_accel / jerk_rate);  // Steps to reach max acceleration
    float speed_after_jerk = start_speed + (start_accel * jerk_steps) + (jerk_rate * jerk_steps * jerk_steps) / 2.0f;
    uint32_t constant_accel_steps = (uint32_t)((max_speed - speed_after_jerk) / max_accel);
    uint32_t total_accel_steps = jerk_steps + constant_accel_steps;
    
    printf("\nStep breakdown:\n");
    printf("- Jerk phase (accel %.1f→%.1f Hz/step): %u steps\n", start_accel, max_accel, jerk_steps);
    printf("- Constant acceleration phase: %u steps\n", constant_accel_steps);
    printf("- Total acceleration steps: %u steps\n", total_accel_steps);
    printf("- Constant speed phase: depends on total distance\n");
    printf("- Deceleration steps: %u steps (same as acceleration)\n", total_accel_steps);
    
    return 0;
} 