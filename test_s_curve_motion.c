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
#include "tmc2209.h"
#include "tmc_gpio.h"
#include "tmc_motion.h"

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

    printf("TMC2209 S-Curve Motion Test\n");
    printf("===========================\n\n");
    
    // Initialize GPIO
    printf("Initializing GPIO...\n");
    gpio_config = tmc_gpio_get_default_config();
    if (!tmc_gpio_init(&gpio_ctx, &gpio_config, "gpiochip0")) {
        printf("ERROR: Failed to initialize GPIO\n");
        return -1;
    }
    printf("GPIO initialized successfully\n");
    
    // Initialize TMC2209 with defaults
    printf("Initializing TMC2209...\n");
    TMC2209_SetDefaults(&driver);
    driver.config.current = 500;      // 500mA
    driver.config.microsteps = 8;     // change microsteps here
    
    if (!TMC2209_Init(&driver)) {
        printf("ERROR: Failed to initialize TMC2209\n");
        return -1;
    }
    printf("TMC2209 initialized successfully\n");
    
    // Set motor current
    printf("Setting motor current to 500mA...\n");
    TMC2209_SetCurrent(&driver, 600, 50);
    
    // Verify current was set
    uint16_t actual_current = TMC2209_GetCurrent(&driver, TMCCurrent_Actual);
    printf("Motor current: %d mA\n\n", actual_current);
    
         // Initialize S-curve motion control
     printf("Initializing S-curve motion control...\n");
     if (!tmc_motion_s_curve_init(&motion, &gpio_ctx, &driver)) {
         printf("ERROR: Failed to initialize S-curve motion control\n");
         return -1;
     }
    
    // Get user input for S-curve parameters
    float target_angle_degrees, max_speed_rpm, max_accel_hz_per_sec, jerk_rate_hz_per_sec2;
    float start_speed_hz, start_accel_hz_per_sec;
    uint32_t gear_ratio;
    bool direction;
    char dir_input;
    
    printf("=== S-Curve Motion Parameters ===\n\n");
    
    printf("Enter target angle (degrees, output shaft): ");
    if (scanf("%f", &target_angle_degrees) != 1) {
        printf("ERROR: Invalid target angle\n");
        return -1;
    }
    
    printf("Enter maximum speed (RPM, input shaft): ");
    if (scanf("%f", &max_speed_rpm) != 1) {
        printf("ERROR: Invalid maximum speed\n");
        return -1;
    }
    
    printf("Enter maximum acceleration (Hz/sec, input shaft): ");
    if (scanf("%f", &max_accel_hz_per_sec) != 1) {
        printf("ERROR: Invalid maximum acceleration\n");
        return -1;
    }
    
    printf("Enter jerk rate (Hz/sec², input shaft): ");
    if (scanf("%f", &jerk_rate_hz_per_sec2) != 1) {
        printf("ERROR: Invalid jerk rate\n");
        return -1;
    }
    
    printf("Enter starting speed (Hz, input shaft): ");
    if (scanf("%f", &start_speed_hz) != 1) {
        printf("ERROR: Invalid starting speed\n");
        return -1;
    }
    
    printf("Enter starting acceleration (Hz/sec, input shaft): ");
    if (scanf("%f", &start_accel_hz_per_sec) != 1) {
        printf("ERROR: Invalid starting acceleration\n");
        return -1;
    }
    
    printf("Enter gear ratio (e.g., 100 for 100:1 reduction): ");
    if (scanf("%u", &gear_ratio) != 1) {
        printf("ERROR: Invalid gear ratio\n");
        return -1;
    }
    
    printf("Enter direction (c = clockwise, w = counter-clockwise): ");
    if (scanf(" %c", &dir_input) != 1) {
        printf("ERROR: Invalid direction\n");
        return -1;
    }
    direction = (dir_input == 'c' || dir_input == 'C');
    
    printf("\n=== Motion Parameters Summary ===\n");
    printf("- Target angle: %.2f degrees (output shaft)\n", target_angle_degrees);
    printf("- Maximum speed: %.2f RPM (input shaft)\n", max_speed_rpm);
    printf("- Maximum acceleration: %.2f Hz/sec (input shaft)\n", max_accel_hz_per_sec);
    printf("- Jerk rate: %.3f Hz/sec² (input shaft)\n", jerk_rate_hz_per_sec2);
    printf("- Starting speed: %.2f Hz (input shaft)\n", start_speed_hz);
    printf("- Starting acceleration: %.2f Hz/sec (input shaft)\n", start_accel_hz_per_sec);
    printf("- Gear ratio: %u:1\n", gear_ratio);
    printf("- Direction: %s\n", direction ? "Clockwise" : "Counter-clockwise");
    printf("- Microstep resolution: 1/%d\n", driver.config.microsteps);
    
    // Calculate expected parameters for display
    uint32_t steps_per_rev = 200 * driver.config.microsteps;
    float target_revolutions = target_angle_degrees / 360.0f;
    float input_revolutions = target_revolutions * gear_ratio;
    uint32_t total_steps = (uint32_t)(input_revolutions * steps_per_rev);
    float max_speed_hz = (max_speed_rpm * steps_per_rev) / 60.0f;
    float output_speed_rpm = max_speed_rpm / gear_ratio;
    
    printf("\n=== Calculated Parameters ===\n");
    printf("- Steps per revolution: %u\n", steps_per_rev);
    printf("- Total steps: %u\n", total_steps);
    printf("- Output shaft speed: %.2f RPM\n", output_speed_rpm);
    printf("- Maximum step frequency: %.2f Hz\n", max_speed_hz);
    printf("- Minimum timer period: %u μs\n", speed_to_timer_period(max_speed_hz));
    printf("- Position monitoring: Every %u microsteps (every full step)\n", driver.config.microsteps);
    
    // Start S-curve motion
    printf("\nStarting S-curve motion...\n");
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
        printf("ERROR: Failed to start S-curve motion\n");
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
            
            printf("Progress: %.1f%% (%u/%u steps) - %s\n", 
                   progress, steps_completed, motion.total_steps, status);
            
            if (motion.motion_active) {
                printf("  Current speed: %.2f Hz, Timer: %u μs, Phase: %s\n",
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
                    printf("  Position: Step=%u, MSCNT=%u, Error=%d, Age=%lu ms %s\n", 
                           monitor_step_count, monitor_mscnt, monitor_error, age_ms,
                           motion.position_monitor.position_valid ? "✓" : "✗");
                    
                    // Show error accumulation statistics
                    int32_t total_error;
                    uint32_t check_count;
                    float average_error;
                    if (tmc_position_monitor_get_error_stats(&motion.position_monitor, 
                                                            &total_error, &check_count, &average_error)) {
                        printf("  Error Stats: Total=%d, Checks=%u, Avg=%.2f steps\n", 
                               total_error, check_count, average_error);
                    }
                } else {
                    printf("  Position: No data available yet\n");
                }
            }
            
            last_reported_step = steps_completed;
            start_time = current_time;
        }
    }
    
    // Motion complete
    printf("\nSUCCESS: S-curve motion completed!\n");
    
    // Print final status
    printf("\n=== Final Status ===\n");
    printf("- Status: %s\n", tmc_motion_s_curve_get_status(&motion));
    printf("- Progress: %.1f%%\n", tmc_motion_s_curve_get_progress(&motion) * 100.0f);
    printf("- Steps completed: %u\n", motion.steps_completed);
    printf("- Total steps: %u\n", motion.total_steps);
    
    // Show final position status
    if (motion.position_monitor.driver) {
        uint32_t monitor_step_count;
        uint16_t monitor_mscnt;
        int16_t monitor_error;
        uint64_t last_update_time;
        
        printf("\n=== Final Position Status ===\n");
        if (tmc_position_monitor_get_status(&motion.position_monitor, 
                                           &monitor_step_count, &monitor_mscnt, 
                                           &monitor_error, &last_update_time)) {
            uint64_t current_time = get_time_us();
            uint64_t age_ms = (current_time - last_update_time) / 1000;
            printf("- Final step count: %u\n", monitor_step_count);
            printf("- Final MSCNT value: %u\n", monitor_mscnt);
            printf("- Position error: %d microsteps\n", monitor_error);
            printf("- Last update age: %lu ms\n", age_ms);
            printf("- Position valid: %s\n", motion.position_monitor.position_valid ? "Yes ✓" : "No ✗");
            
            // Show final error accumulation statistics
            int32_t total_error;
            uint32_t check_count;
            float average_error;
            if (tmc_position_monitor_get_error_stats(&motion.position_monitor, 
                                                    &total_error, &check_count, &average_error)) {
                printf("\n=== Error Accumulation Analysis ===\n");
                printf("- Total accumulated error: %d steps\n", total_error);
                printf("- Number of position checks: %u\n", check_count);
                printf("- Average error per check: %.2f steps\n", average_error);
                printf("- Error rate: %.3f steps per full step\n", average_error);
                
                if (check_count > 0) {
                    float error_percentage = (float)abs(total_error) / check_count * 100.0f;
                    printf("- Error percentage: %.2f%%\n", error_percentage);
                    
                    if (total_error == 0) {
                        printf("- Result: Perfect synchronization ✓\n");
                    } else if (abs(total_error) <= check_count / 10) {
                        printf("- Result: Good synchronization ✓\n");
                    } else if (abs(total_error) <= check_count / 2) {
                        printf("- Result: Moderate drift ⚠\n");
                    } else {
                        printf("- Result: Significant drift ✗\n");
                    }
                }
            }
        } else {
            printf("- Position data: Not available\n");
        }
    }
    
    // Calculate and display timing information
    uint64_t end_time = get_time_us();
    float total_time_seconds = (float)(end_time - motion.start_time_us) / 1000000.0f;
    float average_speed_hz = (float)motion.steps_completed / total_time_seconds;
    
    printf("\n=== Timing Information ===\n");
    printf("- Total motion time: %.3f seconds\n", total_time_seconds);
    printf("- Average step frequency: %.2f Hz\n", average_speed_hz);
    printf("- Average speed: %.2f RPM\n", (average_speed_hz * 60.0f) / (steps_per_rev * gear_ratio));
    
    // Cleanup
    tmc_motion_s_curve_deinit(&motion);
    tmc_gpio_deinit(&gpio_ctx);
    
    printf("\nS-curve motion test completed successfully!\n");
    
    return 0;
} 