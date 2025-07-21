/*
 * tmc_motion.c - Simple interrupt-based motion control implementation
 *
 * This file implements simple constant-speed motion control using timer interrupts
 * for precise step generation. Takes RPM and revolutions as input.
 *
 * v1.0.0 / 2024-12-19
 */

 #include "tmc_motion.h"
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <unistd.h>
 #include <errno.h>
 #include <math.h>
 
 // ============================================================================
 // INTERNAL FUNCTIONS
 // ============================================================================
 
 static uint32_t calculate_step_count_from_mscnt(uint16_t mscnt, uint16_t microstep_resolution);
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
  * @brief S-curve motion execution thread function
  * 
  * @param arg S-curve motion control structure pointer
  * @return NULL
  */
 static void* s_curve_motion_thread_function(void *arg) {

    //cast the generic pointer to the tmc_motion_s_curve_t structure
     tmc_motion_s_curve_t *motion = (tmc_motion_s_curve_t*)arg;
     
     if (!motion || !motion->gpio_ctx) {
         return NULL;
     }
     
     // Set direction
     tmc_gpio_write(motion->gpio_ctx, TMC_GPIO_DIR_PIN, 
                    motion->direction ? TMC_GPIO_HIGH : TMC_GPIO_LOW);
     
     // Enable motor
     tmc_gpio_enable_driver(motion->gpio_ctx, true);
     
     // Initialize timing and motion state
     motion->start_time_us = get_time_us();
     motion->last_step_time_us = motion->start_time_us;
     motion->next_step_time_us = motion->start_time_us;
     motion->current_step = 0;
     motion->steps_completed = 0;
     motion->current_phase = PHASE_ACCEL_INCREASE;
     motion->current_speed_hz = motion->start_speed_hz;
     motion->current_accel_hz_per_sec = motion->start_accel_hz_per_sec;
     motion->current_timer_period_us = speed_to_timer_period(motion->current_speed_hz);
     
     printf("Starting S-curve motion: %u steps\n", motion->total_steps);
     printf("Initial speed: %.2f Hz, timer period: %u μs\n", 
            motion->current_speed_hz, motion->current_timer_period_us);
     
     // Main S-curve motion loop
     while (!motion->thread_should_exit && motion->steps_completed < motion->total_steps) {
         uint64_t current_time = get_time_us();
         
         // Check if it's time for the next step
         if (current_time >= motion->next_step_time_us) {
             // Generate step pulse
             tmc_gpio_write(motion->gpio_ctx, TMC_GPIO_STEP_PIN, TMC_GPIO_HIGH);
             usleep(2); // 2μs pulse width
             tmc_gpio_write(motion->gpio_ctx, TMC_GPIO_STEP_PIN, TMC_GPIO_LOW);
             
             // Update step counters
             motion->steps_completed++;
             motion->current_step++;
             
             // Update position monitor with current step count
             if (motion->position_monitor.driver) {
                 tmc_position_monitor_update_step_count(&motion->position_monitor, motion->steps_completed);
                 
                 // Check if we should read MSCNT at this step (every full step boundary)
                 if (motion->steps_completed > 0 && (motion->steps_completed % motion->position_monitor.poll_interval_steps) == 0) {
                     // Read MSCNT directly in the step generation thread
                     if (TMC2209_ReadRegister(motion->position_monitor.driver, (TMC2209_datagram_t *)&motion->position_monitor.driver->mscnt)) {
                         uint16_t actual_mscnt = motion->position_monitor.driver->mscnt.reg.mscnt;
                         uint16_t expected_mscnt = calculate_expected_mscnt(motion->steps_completed, motion->position_monitor.microstep_resolution);
                         
                         // Calculate instantaneous error with wrap-around handling
                         int16_t difference = (int16_t)actual_mscnt - (int16_t)expected_mscnt;
                         if (difference > 512) {
                             difference -= 1024;
                         } else if (difference < -512) {
                             difference += 1024;
                         }
                         
                                                  // Calculate step error accumulation
                         int32_t step_error = 0;
                         if (motion->position_monitor.error_check_count > 0) {
                             // Calculate expected step difference
                             int32_t expected_step_diff = (int32_t)motion->steps_completed - (int32_t)motion->position_monitor.last_expected_step_count;
                             
                             // Calculate MSCNT difference with wrap-around handling
                             int32_t mscnt_diff = (int32_t)actual_mscnt - (int32_t)motion->position_monitor.last_expected_mscnt;
                             if (mscnt_diff > 512) {
                                 mscnt_diff -= 1024;
                             } else if (mscnt_diff < -512) {
                                 mscnt_diff += 1024;
                             }
                             
                             // Convert MSCNT difference to step difference
                             // For 8 microstep resolution: 256/8 = 32 MSCNT per microstep
                             uint32_t mscnt_increment = 256 / motion->position_monitor.microstep_resolution;
                             
                             // Calculate how many steps the MSCNT change represents
                             int32_t mscnt_step_diff = 0;
                             if (mscnt_diff != 0) {
                                 // Handle the case where MSCNT difference doesn't align perfectly with step boundaries
                                 mscnt_step_diff = mscnt_diff / mscnt_increment;
                                 
                                 // Check for remainder to handle partial steps
                                 int32_t remainder = mscnt_diff % mscnt_increment;
                                 if (abs(remainder) > mscnt_increment / 2) {
                                     // Round to nearest step
                                     if (remainder > 0) {
                                         mscnt_step_diff++;
                                     } else {
                                         mscnt_step_diff--;
                                     }
                                 }
                             }
                             
                             // Calculate step error: (expected_step_diff) - (mscnt_step_diff)
                             step_error = expected_step_diff - mscnt_step_diff;
                             
                             // Debug the calculation
                             printf("DEBUG: step_diff=%d, mscnt_diff=%d, mscnt_increment=%u, mscnt_step_diff=%d, step_error=%d\n",
                                    expected_step_diff, mscnt_diff, mscnt_increment, mscnt_step_diff, step_error);
                             
                             // Check if motor is moving the expected amount
                             if (motion->position_monitor.error_check_count <= 3) {
                                 printf("  Motor movement check: Expected %d steps, MSCNT changed by %d (should be %d per step)\n",
                                        expected_step_diff, mscnt_diff, expected_step_diff * mscnt_increment);
                                 
                                 // Check if the movement is consistent
                                 if (abs(mscnt_diff - (expected_step_diff * mscnt_increment)) <= mscnt_increment) {
                                     printf("  ✓ Motor movement is consistent (within 1 microstep)\n");
                                 } else {
                                     printf("  ✗ Motor movement is inconsistent\n");
                                 }
                             }
                         }
                         
                         // Update position monitor status
                         if (pthread_mutex_lock(&motion->position_monitor.data_mutex) == 0) {
                             motion->position_monitor.last_mscnt = actual_mscnt;
                             motion->position_monitor.position_error = difference;
                             motion->position_monitor.position_valid = (abs(difference) <= 2 * (256 / motion->position_monitor.microstep_resolution));
                             motion->position_monitor.last_update_time_us = get_time_us();
                             
                             // Update error accumulation tracking
                             motion->position_monitor.last_expected_step_count = motion->steps_completed;
                             motion->position_monitor.last_expected_mscnt = expected_mscnt;
                             motion->position_monitor.total_step_error += step_error;
                             motion->position_monitor.error_check_count++;
                             motion->position_monitor.average_step_error = (float)motion->position_monitor.total_step_error / motion->position_monitor.error_check_count;
                             
                             pthread_mutex_unlock(&motion->position_monitor.data_mutex);
                         }
                         
                         // Debug output for position checks
                         printf("Position check at step %u (full step %u): MSCNT=%u (expected=%u), error=%d, step_error=%d, total_error=%d, avg_error=%.2f %s\n", 
                                motion->steps_completed, motion->steps_completed / motion->position_monitor.microstep_resolution, 
                                actual_mscnt, expected_mscnt, difference, step_error, 
                                motion->position_monitor.total_step_error, motion->position_monitor.average_step_error,
                                motion->position_monitor.position_valid ? "✓" : "✗");
                         
                         // Additional debug info for first few checks
                         if (motion->position_monitor.error_check_count <= 3) {
                             printf("  Initial MSCNT: %u, Initial step: %u\n", 
                                    motion->position_monitor.last_expected_mscnt, motion->position_monitor.last_expected_step_count);
                         }
                     }
                 }
             }
             
             // Calculate new speed and timer period for next step
             float new_speed = calculate_speed_at_step(motion, motion->steps_completed);
             uint32_t new_timer_period = speed_to_timer_period(new_speed);
             
             // Update motion state
             motion->current_speed_hz = new_speed;
             motion->current_timer_period_us = new_timer_period;
             
             // Determine current detailed phase
             if (motion->steps_completed < motion->accel_phase_steps) {
                 // Acceleration phase - determine sub-phase
                 uint32_t steps_in_accel = motion->steps_completed;
                 if (steps_in_accel < motion->accel_jerk_increase_steps) {
                     motion->current_phase = PHASE_ACCEL_INCREASE;
                 } else if (steps_in_accel < motion->accel_jerk_increase_steps + motion->accel_constant_steps) {
                     motion->current_phase = PHASE_ACCEL_CONSTANT;
                 } else {
                     motion->current_phase = PHASE_ACCEL_DECREASE;
                 }
             } else if (motion->steps_completed < motion->accel_phase_steps + motion->constant_phase_steps) {
                 motion->current_phase = PHASE_UNIFORM;
             } else {
                 // Deceleration phase - determine sub-phase
                 uint32_t steps_in_decel = motion->steps_completed - (motion->accel_phase_steps + motion->constant_phase_steps);
                 if (steps_in_decel < motion->decel_jerk_increase_steps) {
                     motion->current_phase = PHASE_DECEL_INCREASE;
                 } else if (steps_in_decel < motion->decel_jerk_increase_steps + motion->decel_constant_steps) {
                     motion->current_phase = PHASE_DECEL_CONSTANT;
                 } else {
                     motion->current_phase = PHASE_DECEL_DECREASE;
                 }
             }
             
             // Calculate next step time using current timer period
             motion->next_step_time_us = motion->last_step_time_us + motion->current_timer_period_us;
             motion->last_step_time_us = motion->next_step_time_us;
             
             // Debug output every 1000 steps
             if (motion->steps_completed % 1000 == 0) {
                 float progress = (float)motion->steps_completed / motion->total_steps * 100.0f;
                 const char* phase_name = tmc_motion_s_curve_get_status(motion);
                 printf("Step %u/%u (%.1f%%): %.2f Hz, %u μs, Phase: %s\n", 
                        motion->steps_completed, motion->total_steps, progress,
                        motion->current_speed_hz, motion->current_timer_period_us, phase_name);
                 
                 // Get position monitoring status
                 uint32_t monitor_step_count = 0;
                 uint16_t monitor_mscnt = 0;
                 int16_t monitor_error = 0;
                 uint64_t last_update_time = 0;
                 bool position_valid = false;
                 
                 if (motion->position_monitor.driver) {
                     position_valid = tmc_position_monitor_get_status(&motion->position_monitor, 
                                                                      &monitor_step_count, &monitor_mscnt, 
                                                                      &monitor_error, &last_update_time);
                     
                     if (last_update_time > 0) {
                         uint64_t current_time = get_time_us();
                         uint64_t age_ms = (current_time - last_update_time) / 1000;
                         printf("  Position: Step=%u, MSCNT=%u, Error=%d, Age=%lu ms %s\n", 
                                monitor_step_count, monitor_mscnt, monitor_error, age_ms,
                                position_valid ? "✓" : "✗");
                     }
                 }
             }
         } else {
             // Wait a bit before checking again
             usleep(10); // 10μs delay
         }
     }
     
     // Motion complete
     motion->motion_complete = true;
     motion->motion_active = false;
     motion->current_phase = PHASE_COMPLETE;
     
     // Disable motor
     tmc_gpio_enable_driver(motion->gpio_ctx, false);
     
     printf("S-curve motion complete: %u steps executed\n", motion->steps_completed);
     
     return NULL;
 }
 
 // ============================================================================
 // PUBLIC FUNCTION IMPLEMENTATIONS - S-CURVE MOTION
 // ============================================================================
 
 bool tmc_motion_s_curve_init(tmc_motion_s_curve_t *motion, tmc_gpio_context_t *gpio_ctx, TMC2209_t *tmc_driver) {
     if (!motion || !gpio_ctx || !tmc_driver) {
         return false;
     }
     
     // Initialize motion structure
     memset(motion, 0, sizeof(tmc_motion_s_curve_t));
     motion->gpio_ctx = gpio_ctx;
     motion->tmc_driver = tmc_driver;
     
     // Initialize position monitor
     if (!tmc_position_monitor_init(&motion->position_monitor, tmc_driver, 0)) {
         printf("WARNING: Failed to initialize position monitor - continuing without position tracking\n");
         // Set a flag to disable position monitoring
         motion->position_monitor.driver = NULL;
     }
     
     printf("S-curve motion control initialized with MSCNT position tracking\n");
     return true;
 }
 
 void tmc_motion_s_curve_deinit(tmc_motion_s_curve_t *motion) {
     if (!motion) {
         return;
     }
     
     // Stop any running motion
     tmc_motion_s_curve_stop(motion);
     
     // Deinitialize position monitor
     if (motion->position_monitor.driver) {
         tmc_position_monitor_deinit(&motion->position_monitor);
     }
     
     // Clear structure
     memset(motion, 0, sizeof(tmc_motion_s_curve_t));
     
     printf("S-curve motion control deinitialized\n");
 }
 
 bool tmc_motion_s_curve_start(tmc_motion_s_curve_t *motion,
                              float target_angle_degrees,
                              float max_speed_rpm,
                              float max_accel_hz_per_sec,
                              float jerk_rate_hz_per_sec2,
                              float start_speed_hz,
                              float start_accel_hz_per_sec,
                              uint32_t gear_ratio,
                              bool direction,
                              uint16_t microstep_resolution) {
     if (!motion) {
         return false;
     }
     
     // Stop any existing motion
     tmc_motion_s_curve_stop(motion);
     
     // Set input parameters
     motion->target_angle_degrees = target_angle_degrees;
     motion->max_speed_rpm = max_speed_rpm;
     motion->max_accel_hz_per_sec = max_accel_hz_per_sec;
     motion->jerk_rate_hz_per_sec2 = jerk_rate_hz_per_sec2;
     motion->start_speed_hz = start_speed_hz;
     motion->start_accel_hz_per_sec = start_accel_hz_per_sec;
     motion->gear_ratio = gear_ratio;
     motion->direction = direction;
     motion->microstep_resolution = microstep_resolution;
     
     // Validate parameters
     if (!validate_s_curve_parameters(motion)) {
         printf("ERROR: S-curve parameter validation failed: %s\n", motion->validation_message);
         return false;
     }
     
     // Calculate S-curve profile
     if (!calculate_s_curve_profile(motion)) {
         printf("ERROR: S-curve profile calculation failed\n");
         return false;
     }
     
     // Reset motion state
     motion->current_step = 0;
     motion->steps_completed = 0;
     motion->motion_active = true;
     motion->motion_complete = false;
     motion->thread_should_exit = false;
     motion->current_phase = PHASE_ACCEL_INCREASE;
     
     // Update position monitor with microstep resolution (no separate thread needed)
     motion->position_monitor.microstep_resolution = microstep_resolution;
     if (motion->position_monitor.driver) {
         // Initialize polling interval for step-based monitoring
         motion->position_monitor.poll_interval_steps = microstep_resolution;
         printf("Position monitoring enabled: polling every %u microsteps (every full step)\n", microstep_resolution);
     }
     
     // Create motion thread
     if (pthread_create(&motion->motion_thread, NULL, s_curve_motion_thread_function, motion) != 0) {
         printf("ERROR: Failed to create S-curve motion thread\n");
         motion->motion_active = false;
         if (motion->position_monitor.driver) {
             tmc_position_monitor_stop(&motion->position_monitor);
         }
         return false;
     }
     
     motion->thread_running = true;
     
     printf("S-curve motion started: %.2f degrees, %.2f RPM max, %s direction\n",
            target_angle_degrees, max_speed_rpm, direction ? "clockwise" : "counter-clockwise");
     
     return true;
 }
 
 bool tmc_motion_s_curve_stop(tmc_motion_s_curve_t *motion) {
     if (!motion) {
         return false;
     }
     
     // Signal thread to exit
     motion->thread_should_exit = true;
     motion->motion_active = false;
     
     // Wait for thread to finish
     if (motion->thread_running) {
         pthread_join(motion->motion_thread, NULL);
         motion->thread_running = false;
     }
     
     // Position monitoring is handled directly in step generation thread
     // No separate thread to stop
     
     // Disable motor
     if (motion->gpio_ctx) {
         tmc_gpio_enable_driver(motion->gpio_ctx, false);
     }
     
     printf("S-curve motion stopped\n");
     return true;
 }
 
 bool tmc_motion_s_curve_is_complete(tmc_motion_s_curve_t *motion) {
     if (!motion) {
         return false;
     }
     return motion->motion_complete;
 }
 
 float tmc_motion_s_curve_get_progress(tmc_motion_s_curve_t *motion) {
     if (!motion || motion->total_steps == 0) {
         return 0.0f;
     }
     return (float)motion->steps_completed / motion->total_steps;
 }
 
 const char* tmc_motion_s_curve_get_status(tmc_motion_s_curve_t *motion) {
     if (!motion) {
         return "Invalid";
     }
     
     if (motion->motion_complete) {
         return "Complete";
     } else if (motion->motion_active) {
         switch (motion->current_phase) {
             case PHASE_ACCEL_INCREASE: return "Accel Jerk Increase";
             case PHASE_ACCEL_CONSTANT: return "Accel Constant";
             case PHASE_ACCEL_DECREASE: return "Accel Jerk Decrease";
             case PHASE_UNIFORM: return "Constant Speed";
             case PHASE_DECEL_INCREASE: return "Decel Jerk Increase";
             case PHASE_DECEL_CONSTANT: return "Decel Constant";
             case PHASE_DECEL_DECREASE: return "Decel Jerk Decrease";
             default: return "Running";
         }
     } else {
         return "Stopped";
     }
 }
 
 //unused but could be useful
 bool tmc_motion_s_curve_wait_for_completion(tmc_motion_s_curve_t *motion, uint32_t timeout_ms) {
     if (!motion) {
         return false;
     }
     
     uint64_t start_time = get_time_us();
     uint64_t timeout_us = (uint64_t)timeout_ms * 1000ULL;
     
     while (!motion->motion_complete && !motion->thread_should_exit) {
         usleep(1000); // 1ms sleep
         
         if (timeout_ms > 0 && (get_time_us() - start_time) > timeout_us) {
             return false; // Timeout
         }
     }
     
     return motion->motion_complete;
 }
 
 // ============================================================================
 // S-CURVE CALCULATION FUNCTIONS
 // ============================================================================
 
 bool validate_s_curve_parameters(tmc_motion_s_curve_t *motion) {
     if (!motion) {
         return false;
     }
     
     // Reset validation
     motion->parameters_valid = true;
     strcpy(motion->validation_message, "Parameters are valid");
     
     // Check target angle
     if (motion->target_angle_degrees <= 0.0f || motion->target_angle_degrees > 360.0f) {
         motion->parameters_valid = false;
         strcpy(motion->validation_message, "Target angle must be between 0.1 and 360 degrees");
         return false;
     }
     
     // Check maximum speed
     if (motion->max_speed_rpm <= 0.0f || motion->max_speed_rpm > 270.0f) {
         motion->parameters_valid = false;
         strcpy(motion->validation_message, "Maximum speed must be between 0.1 and 10000 RPM");
         return false;
     }
     
     // Check maximum acceleration
     if (motion->max_accel_hz_per_sec <= 0.0f || motion->max_accel_hz_per_sec > 10.0f) {
         motion->parameters_valid = false;
         strcpy(motion->validation_message, "Maximum acceleration must be between 0.1 and 10 Hz/sec");
         return false;
     }
     
     // Check jerk rate
     if (motion->jerk_rate_hz_per_sec2 <= 0.0001f || motion->jerk_rate_hz_per_sec2 > 1.0f) {
         motion->parameters_valid = false;
         strcpy(motion->validation_message, "Jerk rate must be between 0.0001 and 1 Hz/sec²");
         return false;
     }
     
     // Check starting speed
     if (motion->start_speed_hz < 10.0f || motion->start_speed_hz > 500.0f) {
         motion->parameters_valid = false;
         strcpy(motion->validation_message, "Starting speed must be between 10 and 500 Hz");
         return false;
     }
     
     // Check starting acceleration
     if (motion->start_accel_hz_per_sec < 0.0f || motion->start_accel_hz_per_sec > motion->max_accel_hz_per_sec) {
         motion->parameters_valid = false;
         strcpy(motion->validation_message, "Starting acceleration must be between 0 and max acceleration");
         return false;
     }
     
     // Check gear ratio
     if (motion->gear_ratio == 0 || motion->gear_ratio > 100) {
         motion->parameters_valid = false;
         strcpy(motion->validation_message, "Gear ratio must be between 1 and 100");
         return false;
     }
     
     // Check microstep resolution
     uint16_t valid_microsteps[] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
     bool valid_microstep = false;
     size_t num_microsteps = sizeof(valid_microsteps)/sizeof(valid_microsteps[0]);
     for (size_t i = 0; i < num_microsteps; i++) {
         if (motion->microstep_resolution == valid_microsteps[i]) {
             valid_microstep = true;
             break;
         }
     }
     if (!valid_microstep) {
         motion->parameters_valid = false;
         strcpy(motion->validation_message, "Microstep resolution must be 1, 2, 4, 8, 16, 32, 64, 128, or 256");
         return false;
     }
     
     return true;
 }
 
 bool calculate_s_curve_profile(tmc_motion_s_curve_t *motion) {
     if (!motion || !motion->parameters_valid) {
         return false;
     }
     
     // Calculate steps per revolution
     motion->steps_per_rev = STEPS_PER_REVOLUTION * motion->microstep_resolution;
     
     // Calculate total steps needed
     float target_revolutions = motion->target_angle_degrees / 360.0f;
     float input_revolutions = target_revolutions * motion->gear_ratio;
     motion->total_steps = (uint32_t)(input_revolutions * motion->steps_per_rev);
     
     // Calculate S-curve parameters using Hz-based units
     motion->max_speed_hz = (motion->max_speed_rpm * motion->steps_per_rev) / SECONDS_PER_MINUTE;
     motion->max_accel_hz_per_sec2 = motion->max_accel_hz_per_sec;
     motion->jerk_rate_hz_per_sec3 = motion->jerk_rate_hz_per_sec2;
     
     // Calculate step-based S-curve phase boundaries
     float max_accel_per_step = motion->max_accel_hz_per_sec2;
     float jerk_per_step2 = motion->jerk_rate_hz_per_sec3;
     
     // Phase 1: Jerk phase - acceleration increases linearly from 0 to max_accel
     uint32_t jerk_phase_steps = (uint32_t)(max_accel_per_step / jerk_per_step2);
     
     // Speed after jerk phase
     float speed_after_jerk = motion->start_speed_hz + 
                            (motion->start_accel_hz_per_sec * jerk_phase_steps) +
                            (jerk_per_step2 * jerk_phase_steps * jerk_phase_steps) / 2.0f;
     
     // Phase 2: Constant acceleration phase - steps needed to reach max speed
     float speed_increase_needed = motion->max_speed_hz - speed_after_jerk;
     uint32_t constant_accel_steps = (uint32_t)(speed_increase_needed / max_accel_per_step);
     
     // Phase 3: Jerk phase - acceleration decreases linearly from max_accel to 0
     // This phase takes the same number of steps as the first jerk phase
     uint32_t jerk_decrease_steps = jerk_phase_steps;
     
     // Total acceleration steps (all 3 phases)
     uint32_t total_accel_steps = jerk_phase_steps + constant_accel_steps + jerk_decrease_steps;
     
     // Store sub-phase boundaries for detailed phase tracking
     motion->accel_jerk_increase_steps = jerk_phase_steps;
     motion->accel_constant_steps = constant_accel_steps;
     motion->accel_jerk_decrease_steps = jerk_decrease_steps;
     
     // Check if we need a constant speed phase
     if (2 * total_accel_steps >= motion->total_steps) {
         // Triangular profile - no constant speed phase
         motion->constant_phase_steps = 0;
         motion->accel_phase_steps = motion->total_steps / 2;
         motion->decel_phase_steps = motion->total_steps / 2;
         
         // For triangular profile, scale sub-phases proportionally
         float scale_factor = (float)motion->accel_phase_steps / total_accel_steps;
         motion->accel_jerk_increase_steps = (uint32_t)(jerk_phase_steps * scale_factor);
         motion->accel_constant_steps = (uint32_t)(constant_accel_steps * scale_factor);
         motion->accel_jerk_decrease_steps = motion->accel_phase_steps - motion->accel_jerk_increase_steps - motion->accel_constant_steps;
     } else {
         // Trapezoidal profile - includes constant speed phase
         motion->accel_phase_steps = total_accel_steps;
         motion->decel_phase_steps = total_accel_steps;
         motion->constant_phase_steps = motion->total_steps - (2 * total_accel_steps);
     }
     
     // Deceleration sub-phases mirror acceleration sub-phases
     motion->decel_jerk_increase_steps = motion->accel_jerk_increase_steps;
     motion->decel_constant_steps = motion->accel_constant_steps;
     motion->decel_jerk_decrease_steps = motion->accel_jerk_decrease_steps;
     
     // Debug output
     printf("S-curve profile calculation:\n");
     printf("  Total steps: %u\n", motion->total_steps);
     printf("  Max speed: %.2f Hz\n", motion->max_speed_hz);
     printf("  Max acceleration: %.2f Hz/step\n", max_accel_per_step);
     printf("  Jerk rate: %.3f Hz/step²\n", jerk_per_step2);
     printf("  Jerk phase steps (increase): %u\n", jerk_phase_steps);
     printf("  Speed after jerk: %.2f Hz\n", speed_after_jerk);
     printf("  Constant accel steps: %u\n", constant_accel_steps);
     printf("  Jerk phase steps (decrease): %u\n", jerk_decrease_steps);
     printf("  Total accel steps: %u\n", total_accel_steps);
     printf("  Acceleration phase: %u steps\n", motion->accel_phase_steps);
     printf("  Constant speed phase: %u steps\n", motion->constant_phase_steps);
     printf("  Deceleration phase: %u steps\n", motion->decel_phase_steps);
     printf("\n  Sub-phase breakdown:\n");
     printf("    Accel jerk increase: %u steps\n", motion->accel_jerk_increase_steps);
     printf("    Accel constant: %u steps\n", motion->accel_constant_steps);
     printf("    Accel jerk decrease: %u steps\n", motion->accel_jerk_decrease_steps);
     printf("    Decel jerk increase: %u steps\n", motion->decel_jerk_increase_steps);
     printf("    Decel constant: %u steps\n", motion->decel_constant_steps);
     printf("    Decel jerk decrease: %u steps\n", motion->decel_jerk_decrease_steps);
     
     return true;
 }
 
 float calculate_speed_at_step(tmc_motion_s_curve_t *motion, uint32_t step_position) {
     if (!motion || step_position >= motion->total_steps) {
         return 0.0f;
     }
     
     float current_speed = motion->start_speed_hz;
     float current_accel = motion->start_accel_hz_per_sec;
     float max_accel_per_step = motion->max_accel_hz_per_sec2;
     float jerk_per_step2 = motion->jerk_rate_hz_per_sec3;
     
     // Determine which phase we're in
     if (step_position < motion->accel_phase_steps) {
         // Acceleration phase - has 3 sub-phases
         uint32_t steps_in_phase = step_position;
         uint32_t jerk_phase_steps = (uint32_t)(max_accel_per_step / jerk_per_step2);
         uint32_t constant_accel_steps = (uint32_t)((motion->max_speed_hz - current_speed - (current_accel * jerk_phase_steps) - (jerk_per_step2 * jerk_phase_steps * jerk_phase_steps) / 2.0f) / max_accel_per_step);
         
         if (steps_in_phase < jerk_phase_steps) {
             // Phase 1: Jerk phase - acceleration increases linearly from start_accel to max_accel
             // UNUSED:
             // float accel_at_step = current_accel + jerk_per_step2 * steps_in_phase;
             float speed = current_speed + (current_accel * steps_in_phase) + (jerk_per_step2 * steps_in_phase * steps_in_phase) / 2.0f;
             return (speed > motion->max_speed_hz) ? motion->max_speed_hz : speed;
         } else if (steps_in_phase < jerk_phase_steps + constant_accel_steps) {
             // Phase 2: Constant acceleration phase - acceleration = max_accel
             uint32_t steps_in_constant_accel = steps_in_phase - jerk_phase_steps;
             float speed_after_jerk = current_speed + 
                                    (current_accel * jerk_phase_steps) + 
                                    (jerk_per_step2 * jerk_phase_steps * jerk_phase_steps) / 2.0f;
             float speed = speed_after_jerk + max_accel_per_step * steps_in_constant_accel;
             return (speed > motion->max_speed_hz) ? motion->max_speed_hz : speed;
         } else {
             // Phase 3: Jerk phase - acceleration decreases linearly from max_accel to 0
             uint32_t steps_in_jerk_decrease = steps_in_phase - jerk_phase_steps - constant_accel_steps;
             float speed_after_constant_accel = current_speed + 
                                              (current_accel * jerk_phase_steps) + 
                                              (jerk_per_step2 * jerk_phase_steps * jerk_phase_steps) / 2.0f +
                                              max_accel_per_step * constant_accel_steps;
             // Current acceleration decreases: a = max_accel - jerk * steps
             float current_accel_at_step = max_accel_per_step - jerk_per_step2 * steps_in_jerk_decrease;
             // Speed increases with decreasing acceleration: v = v_prev + a_avg * steps
             // Average acceleration over this step: (max_accel + current_accel) / 2
             float avg_accel = (max_accel_per_step + current_accel_at_step) / 2.0f;
             float speed = speed_after_constant_accel + avg_accel * steps_in_jerk_decrease;
             return (speed > motion->max_speed_hz) ? motion->max_speed_hz : speed;
         }
         
     } else if (step_position < motion->accel_phase_steps + motion->constant_phase_steps) {
         // Constant speed phase
         return motion->max_speed_hz;
         
     } else {
         // Deceleration phase (mirror of acceleration) - has 3 sub-phases
         uint32_t steps_in_decel = step_position - (motion->accel_phase_steps + motion->constant_phase_steps);
         uint32_t jerk_phase_steps = (uint32_t)(max_accel_per_step / jerk_per_step2);
         uint32_t constant_accel_steps = (uint32_t)((motion->max_speed_hz - current_speed - (current_accel * jerk_phase_steps) - (jerk_per_step2 * jerk_phase_steps * jerk_phase_steps) / 2.0f) / max_accel_per_step);
         
         if (steps_in_decel < jerk_phase_steps) {
             // Phase 1: Jerk phase - acceleration increases linearly from 0 to max_accel (negative)
             float speed = motion->max_speed_hz - (jerk_per_step2 * steps_in_decel * steps_in_decel) / 2.0f;
             return (speed > 0.0f) ? speed : 0.0f;
         } else if (steps_in_decel < jerk_phase_steps + constant_accel_steps) {
             // Phase 2: Constant deceleration phase - acceleration = -max_accel
             uint32_t steps_in_constant_decel = steps_in_decel - jerk_phase_steps;
             float speed_after_jerk = motion->max_speed_hz - (jerk_per_step2 * jerk_phase_steps * jerk_phase_steps) / 2.0f;
             float speed = speed_after_jerk - max_accel_per_step * steps_in_constant_decel;
             return (speed > 0.0f) ? speed : 0.0f;
         } else {
             // Phase 3: Jerk phase - acceleration decreases linearly from -max_accel to 0
             uint32_t steps_in_jerk_decrease = steps_in_decel - jerk_phase_steps - constant_accel_steps;
             float speed_after_constant_decel = motion->max_speed_hz - 
                                              (jerk_per_step2 * jerk_phase_steps * jerk_phase_steps) / 2.0f -
                                              max_accel_per_step * constant_accel_steps;
             // Current acceleration decreases: a = -max_accel + jerk * steps
             float current_accel_at_step = -max_accel_per_step + jerk_per_step2 * steps_in_jerk_decrease;
             // Speed decreases with decreasing acceleration: v = v_prev + a_avg * steps
             // Average acceleration over this step: (-max_accel + current_accel) / 2
             float avg_accel = (-max_accel_per_step + current_accel_at_step) / 2.0f;
             float speed = speed_after_constant_decel + avg_accel * steps_in_jerk_decrease;
             return (speed > 0.0f) ? speed : 0.0f;
         }
     }
 }
 
 uint32_t speed_to_timer_period(float speed_hz) {
     if (speed_hz <= 0.0f) {
         return MAX_TIMER_PERIOD_US;
     }
     
     uint32_t period_us = (uint32_t)(1000000.0f / speed_hz);
     
     // Clamp to valid range
     if (period_us < MIN_TIMER_PERIOD_US) {
         period_us = MIN_TIMER_PERIOD_US;
     } else if (period_us > MAX_TIMER_PERIOD_US) {
         period_us = MAX_TIMER_PERIOD_US;
     }
     
     return period_us;
 }
 
 // ============================================================================
 // POSITION MONITORING FUNCTIONS
 // ============================================================================

/**
 * @brief Position monitoring thread function - microstep-based polling
 * 
 * @param arg Position monitor structure pointer
 * @return NULL
 */
static void* position_monitor_thread_function(void *arg) {
    tmc_position_monitor_t *monitor = (tmc_position_monitor_t*)arg;
    
    if (!monitor || !monitor->driver) {
        printf("ERROR: Invalid position monitor in thread\n");
        return NULL;
    }
    
    printf("Position monitoring thread started (microstep-based polling)\n");
    
    while (monitor->monitoring_active) {
        // Check if driver is still valid
        if (!monitor->driver) {
            printf("WARNING: TMC2209 driver became invalid, stopping monitor\n");
            break;
        }
        
        bool should_poll = false;
        uint32_t current_step_count = 0;
        
        // Check if we should poll based on microstep boundaries
        if (pthread_mutex_lock(&monitor->data_mutex) == 0) {
            current_step_count = monitor->last_step_count;
            
            // Poll if we've completed enough steps since last poll
            // Only poll when we reach a full step boundary (step count is multiple of microstep_resolution)
            if (current_step_count > 0 && (current_step_count % monitor->poll_interval_steps) == 0) {
                // Check if we haven't already polled at this step
                if (current_step_count != monitor->last_polled_step_count) {
                    should_poll = true;
                    monitor->last_polled_step_count = current_step_count;
                }
            }
            
            pthread_mutex_unlock(&monitor->data_mutex);
        }
        
        if (should_poll) {
            // Read MSCNT at microstep boundary
            if (TMC2209_ReadRegister(monitor->driver, (TMC2209_datagram_t *)&monitor->driver->mscnt)) {
                uint16_t actual_mscnt = monitor->driver->mscnt.reg.mscnt;
                uint16_t expected_mscnt = calculate_expected_mscnt(current_step_count, monitor->microstep_resolution);
                
                // Calculate instantaneous error with wrap-around handling
                int16_t difference = (int16_t)actual_mscnt - (int16_t)expected_mscnt;
                if (difference > 512) {
                    difference -= 1024;
                } else if (difference < -512) {
                    difference += 1024;
                }
                
                // Calculate step error accumulation: (expected_now - expected_last) - (mscnt_now - mscnt_last)
                int32_t step_error = 0;
                if (monitor->error_check_count > 0) {
                    // Calculate expected step difference
                    int32_t expected_step_diff = (int32_t)current_step_count - (int32_t)monitor->last_expected_step_count;
                    
                    // Calculate MSCNT difference with wrap-around handling
                    int32_t mscnt_diff = (int32_t)actual_mscnt - (int32_t)monitor->last_expected_mscnt;
                    if (mscnt_diff > 512) {
                        mscnt_diff -= 1024;
                    } else if (mscnt_diff < -512) {
                        mscnt_diff += 1024;
                    }
                    
                    // Convert MSCNT difference to step difference
                    uint32_t mscnt_increment = 256 / monitor->microstep_resolution;
                    int32_t mscnt_step_diff = mscnt_diff / mscnt_increment;
                    
                    // Calculate step error: (expected_step_diff) - (mscnt_step_diff)
                    step_error = expected_step_diff - mscnt_step_diff;
                }
                
                // Update status under mutex protection
                if (pthread_mutex_lock(&monitor->data_mutex) == 0) {
                    monitor->last_mscnt = actual_mscnt;
                    monitor->position_error = difference;
                    monitor->position_valid = (abs(difference) <= 2 * (256 / monitor->microstep_resolution));
                    monitor->last_update_time_us = get_time_us();
                    
                    // Update error accumulation tracking
                    monitor->last_expected_step_count = current_step_count;
                    monitor->last_expected_mscnt = expected_mscnt;
                    monitor->total_step_error += step_error;
                    monitor->error_check_count++;
                    monitor->average_step_error = (float)monitor->total_step_error / monitor->error_check_count;
                    
                    pthread_mutex_unlock(&monitor->data_mutex);
                }
                
                // Debug output for position checks with error accumulation
                printf("Position check at step %u (full step %u): MSCNT=%u (expected=%u), error=%d, step_error=%d, total_error=%d, avg_error=%.2f %s\n", 
                       current_step_count, current_step_count / monitor->microstep_resolution, 
                       actual_mscnt, expected_mscnt, difference, step_error, 
                       monitor->total_step_error, monitor->average_step_error,
                       monitor->position_valid ? "✓" : "✗");
            } else {
                printf("WARNING: Failed to read MSCNT register at step %u\n", current_step_count);
            }
        }
        
        // No sleep - pure step-driven polling
        // The thread will wake up when step count is updated by the motion thread
    }
    
    printf("Position monitoring thread stopped\n");
    return NULL;
}

bool tmc_position_monitor_init(tmc_position_monitor_t *monitor, TMC2209_t *driver, uint16_t microstep_resolution) {
    if (!monitor || !driver) {
        printf("ERROR: Invalid parameters for position monitor init\n");
        return false;
    }
    
    memset(monitor, 0, sizeof(tmc_position_monitor_t));
    monitor->driver = driver;
    monitor->microstep_resolution = microstep_resolution;
    monitor->monitoring_active = false;
    monitor->position_valid = false;
    monitor->position_error = 0;
    monitor->last_step_count = 0;
    monitor->last_mscnt = 0;
    monitor->last_update_time_us = 0;
    monitor->last_step_count_update_time_us = 0;
    
    // Initialize microstep-based polling
    monitor->last_polled_step_count = 0;
    monitor->steps_since_last_poll = 0;
    monitor->poll_interval_steps = microstep_resolution;  // Poll every full step (microstep_resolution microsteps)
    
    // Initialize error accumulation tracking
    monitor->last_expected_step_count = 0;
    monitor->last_expected_mscnt = 0;
    monitor->total_step_error = 0;
    monitor->error_check_count = 0;
    monitor->average_step_error = 0.0f;
    
    if (pthread_mutex_init(&monitor->data_mutex, NULL) != 0) {
        printf("ERROR: Failed to initialize position monitor mutex\n");
        return false;
    }
    
    printf("Position monitor initialized successfully (polling every %u microsteps)\n", monitor->poll_interval_steps);
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
    
    printf("Position monitor deinitialized\n");
}

bool tmc_position_monitor_start(tmc_position_monitor_t *monitor) {
    if (!monitor || !monitor->driver) {
        printf("ERROR: Invalid position monitor or driver for start\n");
        return false;
    }

    // Stop any existing monitoring
    tmc_position_monitor_stop(monitor);

    // Synchronize initial step count with MSCNT
    if (TMC2209_ReadRegister(monitor->driver, (TMC2209_datagram_t *)&monitor->driver->mscnt)) {
        uint16_t mscnt = monitor->driver->mscnt.reg.mscnt;
        uint32_t step_count = calculate_step_count_from_mscnt(mscnt, monitor->microstep_resolution);
        if (pthread_mutex_lock(&monitor->data_mutex) == 0) {
            monitor->last_step_count = step_count;
            monitor->last_step_count_update_time_us = get_time_us();
            monitor->last_polled_step_count = step_count;  // Initialize polling counter
            monitor->steps_since_last_poll = 0;
            
            // Initialize error tracking with current values
            monitor->last_expected_step_count = step_count;
            monitor->last_expected_mscnt = mscnt;
            
            pthread_mutex_unlock(&monitor->data_mutex);
        }
        printf("Position monitor synchronized: MSCNT=%u, initial step count=%u, polling every %u microsteps\n", 
               mscnt, step_count, monitor->poll_interval_steps);
        printf("DEBUG: Initial MSCNT=%u corresponds to step %u (microstep resolution=%u)\n", 
               mscnt, step_count, monitor->microstep_resolution);
        
        // Calculate what the expected MSCNT should be for step 0
        uint16_t expected_mscnt_at_zero = calculate_expected_mscnt(0, monitor->microstep_resolution);
        printf("DEBUG: Expected MSCNT at step 0: %u, Offset: %d\n", 
               expected_mscnt_at_zero, (int16_t)mscnt - (int16_t)expected_mscnt_at_zero);
    } else {
        printf("WARNING: Failed to read MSCNT register for initial synchronization\n");
    }

    // Start monitoring
    monitor->monitoring_active = true;

    // Create monitoring thread
    int result = pthread_create(&monitor->monitor_thread, NULL, position_monitor_thread_function, monitor);
    if (result != 0) {
        printf("ERROR: Failed to create position monitoring thread (error %d)\n", result);
        monitor->monitoring_active = false;
        return false;
    }

    printf("Position monitoring started successfully\n");
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
            printf("WARNING: Failed to join position monitoring thread (error %d)\n", result);
        }
        monitor->monitor_thread = 0;
    }
    
    printf("Position monitoring stopped\n");
}

void tmc_position_monitor_update_step_count(tmc_position_monitor_t *monitor, uint32_t step_count) {
    if (!monitor) {
        return;
    }
    
    if (pthread_mutex_lock(&monitor->data_mutex) == 0) {
        monitor->last_step_count = step_count;
        monitor->last_step_count_update_time_us = get_time_us();
        
        // Track steps since last poll for microstep-based polling
        if (step_count >= monitor->last_polled_step_count) {
            monitor->steps_since_last_poll = step_count - monitor->last_polled_step_count;
        } else {
            // Handle step count wraparound (unlikely but possible)
            monitor->steps_since_last_poll = step_count;
        }
        
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
 
 uint16_t calculate_expected_mscnt(uint32_t step_count, uint16_t microstep_resolution) {
     // MSCNT increments by 256/microstep_resolution for each step
     // MSCNT wraps around from 1023 to 0
     uint32_t mscnt_increment = 256 / microstep_resolution;
     uint32_t total_mscnt = step_count * mscnt_increment;
     
     // Apply wrap-around (MSCNT is 10-bit, so 0-1023)
     return (uint16_t)(total_mscnt % 1024);
 }

// Helper function to calculate step count from MSCNT
static uint32_t calculate_step_count_from_mscnt(uint16_t mscnt, uint16_t microstep_resolution) {
    // MSCNT increments by 256/microstep_resolution for each step
    uint32_t mscnt_increment = 256 / microstep_resolution;
    return mscnt / mscnt_increment;
}
 
 bool check_mscnt_position(TMC2209_t *driver, uint32_t step_count, uint16_t microstep_resolution) {
     if (!driver) {
         return false;
     }
     
     // Calculate expected MSCNT value
     uint16_t expected_mscnt = calculate_expected_mscnt(step_count, microstep_resolution);
     
     // Read actual MSCNT from TMC2209
     if (!TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->mscnt)) {
         printf("ERROR: Failed to read MSCNT register\n");
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
     printf("  Position Check: Software=%u steps, MSCNT=%u (expected=%u) %s\n", 
            step_count, actual_mscnt, expected_mscnt, 
            position_valid ? "✓" : "✗");
     
     if (!position_valid) {
         printf("    WARNING: Position discrepancy detected! Difference: %d microsteps\n", difference);
     }
     
     return position_valid;
 }