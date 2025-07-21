/*
 * test_simple_motion.c - Simple motion control test
 *
 * This program tests the simple interrupt-based motion control
 * by taking RPM and revolutions as input and executing the motion.
 *
 * v1.0.0 / 2024-12-19
 */
 
 #include <stdio.h>
 #include <stdlib.h>
 #include <unistd.h>
 #include "tmc2209.h"
 #include "tmc_gpio.h"
 #include "tmc_motion.h"
 
 int main() {
     TMC2209_t driver;
     tmc_gpio_context_t gpio_ctx;
     tmc_gpio_config_t gpio_config;
     tmc_motion_simple_t motion;
 
     printf("TMC2209 Simple Motion Test\n");
     printf("==========================\n\n");
     
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
     driver.config.microsteps = 4;     // 4 microsteps
     
     if (!TMC2209_Init(&driver)) {
         printf("ERROR: Failed to initialize TMC2209\n");
         return -1;
     }
     printf("TMC2209 initialized successfully\n");
     
     // Set motor current
     printf("Setting motor current to 500mA...\n");
     TMC2209_SetCurrent(&driver, 500, 50);
     
     // Verify current was set
     uint16_t actual_current = TMC2209_GetCurrent(&driver, TMCCurrent_Actual);
     printf("Motor current: %d mA\n\n", actual_current);
     
     // Initialize simple motion control
     printf("Initializing simple motion control...\n");
     if (!tmc_motion_simple_init(&motion, &gpio_ctx)) {
         printf("ERROR: Failed to initialize motion control\n");
         return -1;
     }
     
     // Get user input
     float speed_rpm, revolutions;
     bool direction;
     char dir_input;
     
     printf("Enter speed (RPM, input shaft): ");
     scanf("%f", &speed_rpm);
     
     printf("Enter number of revolutions: ");
     scanf("%f", &revolutions);
     
     printf("Enter direction (c = clockwise, w = counter-clockwise): ");
     scanf(" %c", &dir_input);
     direction = (dir_input == 'c' || dir_input == 'C');
     
     printf("\nMotion Parameters:\n");
     printf("- Speed: %.2f RPM (input shaft)\n", speed_rpm);
     printf("- Revolutions: %.2f\n", revolutions);
     printf("- Direction: %s\n", direction ? "Clockwise" : "Counter-clockwise");
     printf("- Microsteps: %d\n", driver.config.microsteps);
     
     // Calculate expected duration
     float step_frequency = tmc_motion_simple_rpm_to_hz(speed_rpm, driver.config.microsteps);
     uint32_t total_steps = tmc_motion_simple_revolutions_to_steps(revolutions, driver.config.microsteps);
     float duration_seconds = (float)total_steps / step_frequency;
     
     printf("- Step frequency: %.2f Hz\n", step_frequency);
     printf("- Total steps: %u\n", total_steps);
     printf("- Expected duration: %.2f seconds\n\n", duration_seconds);
     
     // Start motion
     printf("Starting motion...\n");
     if (!tmc_motion_simple_start(&motion, speed_rpm, revolutions, direction, driver.config.microsteps)) {
         printf("ERROR: Failed to start motion\n");
         return -1;
     }
     
     // Wait for completion
     printf("Waiting for motion to complete...\n");
     if (tmc_motion_simple_wait_for_completion(&motion, 0)) {
         printf("SUCCESS: Motion completed!\n");
     } else {
         printf("ERROR: Motion failed or timed out\n");
         return -1;
     }
     
     // Print final status
     printf("\nFinal Status:\n");
     printf("- Status: %s\n", tmc_motion_simple_get_status(&motion));
     printf("- Progress: %.1f%%\n", tmc_motion_simple_get_progress(&motion) * 100.0f);
     printf("- Steps completed: %u\n", motion.steps_completed);
     
     // Cleanup
     tmc_motion_simple_deinit(&motion);
     tmc_gpio_deinit(&gpio_ctx);
     
     printf("\nTest completed successfully!\n");
     
     return 0;
 }