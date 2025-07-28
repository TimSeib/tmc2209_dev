/*
 * test_s_curve_motion.c - S-curve motion control test
 *
 * This program tests the integrated S-curve motion control by taking
 * S-curve parameters as input and executing real-time motion with
 * dynamic speed changes based on step position.
 *
 * v1.0.0 / 2024-12-19
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include "tmc2209.h"
#include "tmc_gpio.h"
#include "tmc_motion.h"
#include "log.h"

// Helper function to get current time (copied from motion.c)
static uint64_t get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

int main() {
    TMC2209_t driver;
    tmc_gpio_context_t gpio_ctx;
    tmc_gpio_config_t gpio_config;
    tmc_motion_s_curve_t motion;
    int log_level;

    /*
    log_trace(const char *fmt, ...);
    log_debug(const char *fmt, ...);
    log_info(const char *fmt, ...);
    log_warn(const char *fmt, ...);
    log_error(const char *fmt, ...);
    log_fatal(const char *fmt, ...);
    */
    
    printf("Enter log level (0, Trace, 1, Debug, 2, Info, 3, Warn, 4, Error, 5, Fatal): ");
    if (scanf("%d", &log_level) != 1) {
        printf("ERROR: Invalid log level\n");
        return -1;
    }
    log_set_level(log_level);
    
    log_info("TMC2209 S-Curve Motion Test");
    log_info("===========================");
    
    // Initialize GPIO

    log_info("Initializing GPIO...");
    gpio_config = tmc_gpio_get_default_config();
    if (!tmc_gpio_init(&gpio_ctx, &gpio_config, "gpiochip0")) {
        log_error("Failed to initialize GPIO");
        return -1;
    }
    log_info("GPIO initialized successfully");
    
    // Initialize TMC2209 with defaults
    log_info("Initializing TMC2209...");
    TMC2209_SetDefaults(&driver);
    driver.config.current = 600;      // 500mA
    driver.config.microsteps = 8;     // change microsteps here (32 max for this S-curve)
    
    if (!TMC2209_Init(&driver)) {
        log_error("Failed to initialize TMC2209");
        return -1;
    }
    log_info("TMC2209 initialized successfully");
    
    // Set motor current
    log_info("Setting motor current to 600mA...");
    TMC2209_SetCurrent(&driver, 600, 50);
    
    // Verify current was set
    uint16_t actual_current = TMC2209_GetCurrent(&driver, TMCCurrent_Actual);
    log_info("Motor current: %d mA", actual_current);
    
         // Initialize S-curve motion control
     log_info("Initializing S-curve motion control...");
     if (!tmc_motion_s_curve_init(&motion, &gpio_ctx, &driver)) {
         log_error("Failed to initialize S-curve motion control");
         return -1;
     }
    
    // Get user input for S-curve parameters
    float target_angle_degrees, max_speed_rpm, max_accel_hz_per_sec, jerk_rate_hz_per_sec2;
    float start_speed_hz, start_accel_hz_per_sec;
    float gear_ratio;
    bool direction;
    char dir_input;
    
    printf("=== S-Curve Motion Parameters ===\n\n");
    
    printf("Enter target angle (degrees, output shaft): ");
    if (scanf("%f", &target_angle_degrees) != 1) {
        log_error("Invalid target angle");
        return -1;
    }
    
    printf("Enter maximum speed (RPM, input shaft): ");
    if (scanf("%f", &max_speed_rpm) != 1) {
        log_error("Invalid maximum speed");
        return -1;
    }
    
    printf("Enter maximum acceleration (Hz/sec, input shaft): ");
    if (scanf("%f", &max_accel_hz_per_sec) != 1) {
        log_error("Invalid maximum acceleration");
        return -1;
    }
    
    printf("Enter jerk rate (Hz/sec², input shaft): ");
    if (scanf("%f", &jerk_rate_hz_per_sec2) != 1) {
        log_error("Invalid jerk rate");
        return -1;
    }
    
    printf("Enter starting speed (Hz, input shaft): ");
    if (scanf("%f", &start_speed_hz) != 1) {
        log_error("Invalid starting speed");
        return -1;
    }
    
    printf("Enter starting acceleration (Hz/sec, input shaft): ");
    if (scanf("%f", &start_accel_hz_per_sec) != 1) {
        log_error("Invalid starting acceleration");
        return -1;
    }
    
    printf("Enter gear ratio (e.g., 100 for 100:1 reduction): ");
    if (scanf("%f", &gear_ratio) != 1) {
        log_error("Invalid gear ratio");
        return -1;
    }
    
    printf("Enter direction (c = clockwise, w = counter-clockwise): ");
    if (scanf(" %c", &dir_input) != 1) {
        log_error("Invalid direction");
        return -1;
    }
    direction = (dir_input == 'w' || dir_input == 'W');
    
    log_info("=== Motion Parameters Summary ===");
    log_info("- Target angle: %.2f degrees (output shaft)", target_angle_degrees);
    log_info("- Maximum speed: %.2f RPM (input shaft)", max_speed_rpm);
    log_info("- Maximum acceleration: %.2f Hz/sec (input shaft)", max_accel_hz_per_sec);
    log_info("- Jerk rate: %.3f Hz/sec² (input shaft)", jerk_rate_hz_per_sec2);
    log_info("- Starting speed: %.2f Hz (input shaft)", start_speed_hz);
    log_info("- Starting acceleration: %.2f Hz/sec (input shaft)", start_accel_hz_per_sec);
    log_info("- Gear ratio: %f:1", gear_ratio);
    log_info("- Direction: %s", direction ? "Clockwise" : "Counter-clockwise");
    log_info("- Microstep resolution: 1/%d", driver.config.microsteps);
    
    // Calculate expected parameters for display
    uint32_t steps_per_rev = 200 * driver.config.microsteps;
    float target_revolutions = target_angle_degrees / 360.0f;
    float input_revolutions = target_revolutions * gear_ratio;
    uint32_t total_steps = (uint32_t)(input_revolutions * steps_per_rev);
    float max_speed_hz = (max_speed_rpm * steps_per_rev) / 60.0f;
    float output_speed_rpm = max_speed_rpm / gear_ratio;
    
    log_info("=== Calculated Parameters ===");
    log_info("- Steps per revolution: %u", steps_per_rev);
    log_info("- Total steps: %u", total_steps);
    log_info("- Output shaft speed: %.2f RPM", output_speed_rpm);
    log_info("- Maximum step frequency: %.2f Hz", max_speed_hz);
    log_info("- Minimum timer period: %u μs", speed_to_timer_period(max_speed_hz));
    log_info("- Position monitoring: Every %u microsteps (every full step)", driver.config.microsteps);
    
    // Start S-curve motion
    log_info("Starting S-curve motion...");
    if (!tmc_motion_s_curve_start(&motion, 
                                 target_angle_degrees,
                                 max_speed_rpm,
                                 max_accel_hz_per_sec,
                                 jerk_rate_hz_per_sec2,
                                 start_speed_hz,
                                 start_accel_hz_per_sec,
                                 gear_ratio,
                                 direction,
                                 driver.config.microsteps)) {
        log_error("Failed to start S-curve motion");
        return -1;
    }
    
    // Monitor motion progress
    printf("\nMonitoring motion progress...\n");
    printf("Position checks will occur every %u microsteps (every full step)\n", driver.config.microsteps);
    printf("Press Ctrl+C to stop motion\n\n");
    
    uint32_t last_reported_step = 0;
    uint64_t start_time = get_time_us();
    
    while (!tmc_motion_s_curve_is_complete(&motion)) {
        usleep(100000); // 100ms sleep
        
        // Report progress every 1000 steps or every 5 seconds
        uint64_t current_time = get_time_us();
        uint32_t steps_completed = motion.steps_completed;
        float progress = tmc_motion_s_curve_get_progress(&motion) * 100.0f;
        const char* status = tmc_motion_s_curve_get_status(&motion);
        
        if (steps_completed - last_reported_step >= 1000 || 
            (current_time - start_time) >= 5000000) { // 5 seconds
            
            log_info("Progress: %.1f%% (%u/%u steps) - %s", 
                   progress, steps_completed, motion.total_steps, status);
            
            if (motion.motion_active) {
                log_info("  Current speed: %.2f Hz, Timer: %u μs, Phase: %s",
                       motion.current_speed_hz, motion.current_timer_period_us, status);
            }
            
            // Show position monitoring status if available
            if (motion.position_monitor.driver) {
                uint32_t monitor_step_count;
                uint16_t monitor_mscnt;
                int16_t monitor_error;
                uint64_t last_update_time;
                
                if (tmc_position_monitor_get_status(&motion.position_monitor, 
                                                           &monitor_step_count, &monitor_mscnt, 
                                                           &monitor_error, &last_update_time)) {
                    uint64_t current_time = get_time_us();
                    uint64_t age_ms = (current_time - last_update_time) / 1000;
                    log_info("  Position: Step=%u, MSCNT=%u, Error=%d MSCNT units, Age=%lu ms", 
                                   monitor_step_count, monitor_mscnt, monitor_error, age_ms);
                            
                            // Show position accuracy
                            int32_t total_error;
                            uint32_t check_count;
                            float average_error;
                            if (tmc_position_monitor_get_error_stats(&motion.position_monitor, 
                                                                    &total_error, &check_count, &average_error)) {
                                log_info("  Position accuracy: %.3f microsteps average offset", average_error);
                            }
                } else {
                    log_info("  Position: No data available yet");
                }
            }
            
            last_reported_step = steps_completed;
            start_time = current_time;
        }
    }
    
    // Motion complete
    log_info("SUCCESS: S-curve motion completed!");
    
    // Print final status
    log_info("=== Final Status ===");
    log_info("- Status: %s", tmc_motion_s_curve_get_status(&motion));
    log_info("- Progress: %.1f%%", tmc_motion_s_curve_get_progress(&motion) * 100.0f);
    log_info("- Steps completed: %u", motion.steps_completed);
    log_info("- Total steps: %u", motion.total_steps);
    
    // Show final position status
    if (motion.position_monitor.driver) {
        uint32_t monitor_step_count;
        uint16_t monitor_mscnt;
        int16_t monitor_error;
        uint64_t last_update_time;
        
        log_info("=== Final Position Status ===");
        if (tmc_position_monitor_get_status(&motion.position_monitor, 
                                           &monitor_step_count, &monitor_mscnt, 
                                           &monitor_error, &last_update_time)) {
            uint64_t current_time = get_time_us();
            uint64_t age_ms = (current_time - last_update_time) / 1000;
            log_info("- Final step count: %u", monitor_step_count);
            log_info("- Final MSCNT value: %u", monitor_mscnt);
            log_info("- Final position error: %d MSCNT units (%.3f microsteps)", 
                   monitor_error, (float)monitor_error * (float)motion.position_monitor.microstep_resolution / 256.0f);
            log_info("- Last update age: %lu ms", age_ms);
            log_info("- Position valid: %s", motion.position_monitor.position_valid ? "Yes" : "No");
            
            // Show position accuracy summary
            int32_t total_error;
            uint32_t check_count;
            float average_error;
            if (tmc_position_monitor_get_error_stats(&motion.position_monitor, 
                                                    &total_error, &check_count, &average_error)) {
                log_info("=== Position Accuracy Summary ===");
                log_info("- Number of position checks: %u", check_count);
                log_info("- Average position offset: %.3f microsteps", average_error);
                log_info("- Position consistency: %s", 
                       (fabs(average_error) < 0.1f) ? "Excellent" : 
                       (fabs(average_error) < 0.5f) ? "Good" : 
                       (fabs(average_error) < 1.0f) ? "Acceptable" : "Poor");
            }
        } else {
            log_info("- Position data: Not available");
        }
    }
    
    // Calculate and display timing information
    uint64_t end_time = get_time_us();
    float total_time_seconds = (float)(end_time - motion.start_time_us) / 1000000.0f;
    float average_speed_hz = (float)motion.steps_completed / total_time_seconds;
    
    log_info("=== Timing Information ===");
    log_info("- Total motion time: %.3f seconds", total_time_seconds);
    log_info("- Average step frequency: %.2f Hz", average_speed_hz);
    log_info("- Average speed: %.2f RPM", (average_speed_hz * 60.0f) / (steps_per_rev * gear_ratio));
    
    // Cleanup
    tmc_motion_s_curve_deinit(&motion);
    tmc_gpio_deinit(&gpio_ctx);
    // Add MSCNT delta summary
    log_info("=== MSCNT Delta Summary ===");
    log_info("Total MSCNT delta for entire movement: %d MSCNT units", g_total_mscnt_delta);
    log_info("Total MSCNT delta in full steps: %.3f full steps", (float)g_total_mscnt_delta / 256.0f);
    log_info("Total MSCNT delta in microsteps: %.3f microsteps", (float)g_total_mscnt_delta * driver.config.microsteps / 256.0f);
    
    printf("\nS-curve motion test completed successfully!\n");
    
    return 0;
} 