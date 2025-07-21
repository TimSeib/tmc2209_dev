/*
 * tmc_motion_simple.c - Simple interrupt-based motion control implementation
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
  * @brief Motion execution thread function
  * 
  * @param arg Motion control structure pointer
  * @return NULL
  */
 static void* motion_thread_function(void *arg) {
     tmc_motion_simple_t *motion = (tmc_motion_simple_t*)arg;
     
     if (!motion || !motion->gpio_ctx) {
         return NULL;
     }
     
     // Set direction
     tmc_gpio_write(motion->gpio_ctx, TMC_GPIO_DIR_PIN, 
                    motion->direction ? TMC_GPIO_HIGH : TMC_GPIO_LOW);
     
     // Enable motor
     tmc_gpio_enable_driver(motion->gpio_ctx, true);
     
     // Initialize timing
     motion->start_time_us = get_time_us();
     motion->last_step_time_us = motion->start_time_us;
     motion->next_step_time_us = motion->start_time_us;
     
     printf("Starting motion: %u steps at %.2f Hz (%.2f RPM)\n", 
            motion->total_steps, motion->step_frequency_hz, motion->speed_rpm);
     
     // Main motion loop
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
             
             // Calculate next step time
             motion->next_step_time_us = motion->start_time_us + 
                                        (motion->steps_completed * motion->timer_period_us);
             
             // Debug output every 1000 steps
             if (motion->steps_completed % 1000 == 0) {
                 float progress = (float)motion->steps_completed / motion->total_steps * 100.0f;
                 printf("Step %u/%u (%.1f%%)\n", 
                        motion->steps_completed, motion->total_steps, progress);
             }
         } else {
             // Wait a bit before checking again
             usleep(10); // 10μs delay
         }
     }
     
     // Motion complete
     motion->motion_complete = true;
     motion->motion_active = false;
     
     // Disable motor
     tmc_gpio_enable_driver(motion->gpio_ctx, false);
     
     printf("Motion complete: %u steps executed\n", motion->steps_completed);
     
     return NULL;
 }
 
 /**
  * @brief S-curve motion execution thread function
  * 
  * @param arg S-curve motion control structure pointer
  * @return NULL
  */
 static void* s_curve_motion_thread_function(void *arg) {
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
             
             // Calculate new speed and timer period for next step
             float new_speed = calculate_speed_at_step(motion, motion->steps_completed);
             uint32_t new_timer_period = speed_to_timer_period(new_speed);
             
             // Update motion state
             motion->current_speed_hz = new_speed;
             motion->current_timer_period_us = new_timer_period;
             
             // Determine current phase
             if (motion->steps_completed < motion->accel_phase_steps) {
                 motion->current_phase = PHASE_ACCEL_INCREASE;
             } else if (motion->steps_completed < motion->accel_phase_steps + motion->constant_phase_steps) {
                 motion->current_phase = PHASE_UNIFORM;
             } else {
                 motion->current_phase = PHASE_DECEL_INCREASE;
             }
             
             // Calculate next step time using current timer period
             motion->next_step_time_us = motion->last_step_time_us + motion->current_timer_period_us;
             motion->last_step_time_us = motion->next_step_time_us;
             
             // Debug output every 1000 steps
             if (motion->steps_completed % 1000 == 0) {
                 float progress = (float)motion->steps_completed / motion->total_steps * 100.0f;
                 printf("Step %u/%u (%.1f%%): %.2f Hz, %u μs, Phase: %d\n", 
                        motion->steps_completed, motion->total_steps, progress,
                        motion->current_speed_hz, motion->current_timer_period_us, motion->current_phase);
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
 // PUBLIC FUNCTION IMPLEMENTATIONS - SIMPLE MOTION
 // ============================================================================
 
 bool tmc_motion_simple_init(tmc_motion_simple_t *motion, tmc_gpio_context_t *gpio_ctx) {
     if (!motion || !gpio_ctx) {
         return false;
     }
     
     // Initialize motion structure
     memset(motion, 0, sizeof(tmc_motion_simple_t));
     motion->gpio_ctx = gpio_ctx;
     
     printf("Simple motion control initialized\n");
     return true;
 }
 
 void tmc_motion_simple_deinit(tmc_motion_simple_t *motion) {
     if (!motion) {
         return;
     }
     
     // Stop any running motion
     tmc_motion_simple_stop(motion);
     
     // Clear structure
     memset(motion, 0, sizeof(tmc_motion_simple_t));
     
     printf("Simple motion control deinitialized\n");
 }
 
 bool tmc_motion_simple_start(tmc_motion_simple_t *motion,
                             float speed_rpm,
                             float revolutions,
                             bool direction,
                             uint16_t microstep_resolution) {
     if (!motion || speed_rpm <= 0.0f || revolutions <= 0.0f) {
         return false;
     }
     
     // Stop any existing motion
     tmc_motion_simple_stop(motion);
     
     // Set input parameters
     motion->speed_rpm = speed_rpm;
     motion->revolutions = revolutions;
     motion->direction = direction;
     motion->microstep_resolution = microstep_resolution;
     
     // Calculate derived parameters
     motion->steps_per_rev = STEPS_PER_REVOLUTION * microstep_resolution;
     motion->total_steps = tmc_motion_simple_revolutions_to_steps(revolutions, microstep_resolution);
     motion->step_frequency_hz = tmc_motion_simple_rpm_to_hz(speed_rpm, microstep_resolution);
     motion->timer_period_us = tmc_motion_simple_hz_to_timer_period(motion->step_frequency_hz);
     
     // Validate timer period
     if (motion->timer_period_us < MIN_TIMER_PERIOD_US || 
         motion->timer_period_us > MAX_TIMER_PERIOD_US) {
         printf("ERROR: Invalid timer period: %u μs\n", motion->timer_period_us);
         return false;
     }
     
     // Reset motion state
     motion->current_step = 0;
     motion->steps_completed = 0;
     motion->motion_active = true;
     motion->motion_complete = false;
     motion->thread_should_exit = false;
     
     // Create motion thread
     if (pthread_create(&motion->motion_thread, NULL, motion_thread_function, motion) != 0) {
         printf("ERROR: Failed to create motion thread\n");
         motion->motion_active = false;
         return false;
     }
     
     motion->thread_running = true;
     
     printf("Motion started: %.2f RPM, %.2f revolutions, %s direction\n",
            speed_rpm, revolutions, direction ? "clockwise" : "counter-clockwise");
     
     return true;
 }
 
 bool tmc_motion_simple_stop(tmc_motion_simple_t *motion) {
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
     
     // Disable motor
     if (motion->gpio_ctx) {
         tmc_gpio_enable_driver(motion->gpio_ctx, false);
     }
     
     printf("Motion stopped\n");
     return true;
 }
 
 bool tmc_motion_simple_is_complete(tmc_motion_simple_t *motion) {
     if (!motion) {
         return false;
     }
     return motion->motion_complete;
 }
 
 float tmc_motion_simple_get_progress(tmc_motion_simple_t *motion) {
     if (!motion || motion->total_steps == 0) {
         return 0.0f;
     }
     return (float)motion->steps_completed / motion->total_steps;
 }
 
 const char* tmc_motion_simple_get_status(tmc_motion_simple_t *motion) {
     if (!motion) {
         return "Invalid";
     }
     
     if (motion->motion_complete) {
         return "Complete";
     } else if (motion->motion_active) {
         return "Running";
     } else {
         return "Stopped";
     }
 }
 
 bool tmc_motion_simple_wait_for_completion(tmc_motion_simple_t *motion, uint32_t timeout_ms) {
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
 
 float tmc_motion_simple_rpm_to_hz(float rpm, uint16_t microstep_resolution) {
     if (rpm <= 0.0f || microstep_resolution == 0) {
         return 0.0f;
     }
     
     // Convert RPM to steps per second
     // RPM * (steps/rev) / (60 sec/min) = steps/sec
     return (rpm * STEPS_PER_REVOLUTION * microstep_resolution) / SECONDS_PER_MINUTE;
 }
 
 uint32_t tmc_motion_simple_hz_to_timer_period(float frequency_hz) {
     if (frequency_hz <= 0.0f) {
         return MAX_TIMER_PERIOD_US;
     }
     
     uint32_t period_us = (uint32_t)(1000000.0f / frequency_hz);
     
     // Clamp to valid range
     if (period_us < MIN_TIMER_PERIOD_US) {
         period_us = MIN_TIMER_PERIOD_US;
     } else if (period_us > MAX_TIMER_PERIOD_US) {
         period_us = MAX_TIMER_PERIOD_US;
     }
     
     return period_us;
 }
 
 uint32_t tmc_motion_simple_revolutions_to_steps(float revolutions, uint16_t microstep_resolution) {
     if (revolutions <= 0.0f || microstep_resolution == 0) {
         return 0;
     }
     
     return (uint32_t)(revolutions * STEPS_PER_REVOLUTION * microstep_resolution);
 }
 
 // ============================================================================
 // PUBLIC FUNCTION IMPLEMENTATIONS - S-CURVE MOTION
 // ============================================================================
 
 bool tmc_motion_s_curve_init(tmc_motion_s_curve_t *motion, tmc_gpio_context_t *gpio_ctx) {
     if (!motion || !gpio_ctx) {
         return false;
     }
     
     // Initialize motion structure
     memset(motion, 0, sizeof(tmc_motion_s_curve_t));
     motion->gpio_ctx = gpio_ctx;
     
     printf("S-curve motion control initialized\n");
     return true;
 }
 
 void tmc_motion_s_curve_deinit(tmc_motion_s_curve_t *motion) {
     if (!motion) {
         return;
     }
     
     // Stop any running motion
     tmc_motion_s_curve_stop(motion);
     
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
     
     // Create motion thread
     if (pthread_create(&motion->motion_thread, NULL, s_curve_motion_thread_function, motion) != 0) {
         printf("ERROR: Failed to create S-curve motion thread\n");
         motion->motion_active = false;
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
             case PHASE_ACCEL_INCREASE: return "Accelerating (jerk)";
             case PHASE_ACCEL_CONSTANT: return "Accelerating (constant)";
             case PHASE_UNIFORM: return "Constant speed";
             case PHASE_DECEL_INCREASE: return "Decelerating (jerk)";
             case PHASE_DECEL_CONSTANT: return "Decelerating (constant)";
             default: return "Running";
         }
     } else {
         return "Stopped";
     }
 }
 
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
     if (motion->target_angle_degrees <= 0.0f || motion->target_angle_degrees > 3600.0f) {
         motion->parameters_valid = false;
         strcpy(motion->validation_message, "Target angle must be between 0.1 and 3600 degrees");
         return false;
     }
     
     // Check maximum speed
     if (motion->max_speed_rpm <= 0.0f || motion->max_speed_rpm > 10000.0f) {
         motion->parameters_valid = false;
         strcpy(motion->validation_message, "Maximum speed must be between 0.1 and 10000 RPM");
         return false;
     }
     
     // Check maximum acceleration
     if (motion->max_accel_hz_per_sec <= 0.0f || motion->max_accel_hz_per_sec > 100000.0f) {
         motion->parameters_valid = false;
         strcpy(motion->validation_message, "Maximum acceleration must be between 0.1 and 100000 Hz/sec");
         return false;
     }
     
     // Check jerk rate
     if (motion->jerk_rate_hz_per_sec2 <= 0.0f || motion->jerk_rate_hz_per_sec2 > 1000000.0f) {
         motion->parameters_valid = false;
         strcpy(motion->validation_message, "Jerk rate must be between 0.1 and 1000000 Hz/sec²");
         return false;
     }
     
     // Check starting speed
     if (motion->start_speed_hz < 0.0f || motion->start_speed_hz > 10000.0f) {
         motion->parameters_valid = false;
         strcpy(motion->validation_message, "Starting speed must be between 0 and 10000 Hz");
         return false;
     }
     
     // Check starting acceleration
     if (motion->start_accel_hz_per_sec < 0.0f || motion->start_accel_hz_per_sec > motion->max_accel_hz_per_sec) {
         motion->parameters_valid = false;
         strcpy(motion->validation_message, "Starting acceleration must be between 0 and max acceleration");
         return false;
     }
     
     // Check gear ratio
     if (motion->gear_ratio == 0 || motion->gear_ratio > 10000) {
         motion->parameters_valid = false;
         strcpy(motion->validation_message, "Gear ratio must be between 1 and 10000");
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
     
     // Check if we need a constant speed phase
     if (2 * total_accel_steps >= motion->total_steps) {
         // Triangular profile - no constant speed phase
         motion->constant_phase_steps = 0;
         motion->accel_phase_steps = motion->total_steps / 2;
         motion->decel_phase_steps = motion->total_steps / 2;
     } else {
         // Trapezoidal profile - includes constant speed phase
         motion->accel_phase_steps = total_accel_steps;
         motion->decel_phase_steps = total_accel_steps;
         motion->constant_phase_steps = motion->total_steps - (2 * total_accel_steps);
     }
     
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
             float accel_at_step = current_accel + jerk_per_step2 * steps_in_phase;
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