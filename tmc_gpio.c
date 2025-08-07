/*
 * tmc_gpio.c - GPIO control implementation for TMC2209 stepper motor driver
 *
 * This file implements GPIO functionality using libgpiod for controlling
 * TMC2209 stepper motor driver pins.
 *
 * v1.0.0 / 2024-12-19
 */

 #include "tmc_gpio.h"
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <unistd.h>
 #include <errno.h>
 #include <pthread.h>
 #include <time.h>
 
 // ============================================================================
 // INTERNAL HELPER FUNCTIONS
 // ============================================================================
 
 /**
  * @brief Get GPIO line handle for a pin
  * 
  * @param ctx GPIO context structure
  * @param pin Pin number
  * @return GPIO line handle or NULL on error
  */
 static struct gpiod_line* get_line_for_pin(tmc_gpio_context_t *ctx, uint8_t pin) {
     if (!ctx || !ctx->initialized) {
         return NULL;
     }
     
     // Map pin numbers to line handles
     switch (pin) {
         case TMC_GPIO_ENABLE_PIN:
             return ctx->enable_line;
         case TMC_GPIO_STEP_PIN:
             return ctx->step_line;
         case TMC_GPIO_DIR_PIN:
             return ctx->dir_line;
         case TMC_GPIO_DIAG_PIN:
             return ctx->diag_line;
         case TMC_GPIO_INDEX_PIN:
             return ctx->index_line;
         default:
             return NULL;
     }
 }
 
 /**
  * @brief Microsecond delay function
  * 
  * @param us Microseconds to wait
  */
 static void delay_us(uint32_t us) {
     usleep(us);
 }
 
 /**
  * @brief Get current timestamp in microseconds
  * 
  * @return Current timestamp in microseconds
  */
 static uint64_t get_timestamp_us(void) {
     struct timespec ts;
     clock_gettime(CLOCK_MONOTONIC, &ts);
     return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
 }
 
 // ============================================================================
 // PUBLIC FUNCTION IMPLEMENTATIONS
 // ============================================================================
 
 bool tmc_gpio_init(tmc_gpio_context_t *ctx, const tmc_gpio_config_t *config, const char *chip_name) {
     if (!ctx || !config || !chip_name) {
         printf("ERROR: Invalid parameters\n");
         return false;
     }
     
     // Initialize context
     memset(ctx, 0, sizeof(tmc_gpio_context_t));
     
     // Open GPIO chip
     ctx->chip = gpiod_chip_open_by_name(chip_name);
     if (!ctx->chip) {
         fprintf(stderr, "Failed to open GPIO chip: %s\n", chip_name);
         return false;
     }
     
     // Get GPIO lines
     ctx->enable_line = gpiod_chip_get_line(ctx->chip, config->enable_pin);
     ctx->step_line = gpiod_chip_get_line(ctx->chip, config->step_pin);
     ctx->dir_line = gpiod_chip_get_line(ctx->chip, config->dir_pin);
     ctx->diag_line = gpiod_chip_get_line(ctx->chip, config->diag_pin);
     ctx->index_line = gpiod_chip_get_line(ctx->chip, config->index_pin);
     
     // Check if all lines were obtained successfully
     if (!ctx->enable_line || !ctx->step_line || !ctx->dir_line || 
         !ctx->diag_line || !ctx->index_line) {
         fprintf(stderr, "Failed to get one or more GPIO lines\n");
         tmc_gpio_deinit(ctx);
         return false;
     }
     
     // Configure output pins
     if (gpiod_line_request_output(ctx->enable_line, "tmc2209_enable", TMC_GPIO_HIGH) < 0 ||
         gpiod_line_request_output(ctx->step_line, "tmc2209_step", TMC_GPIO_LOW) < 0 ||
         gpiod_line_request_output(ctx->dir_line, "tmc2209_dir", TMC_GPIO_LOW) < 0) {
         fprintf(stderr, "Failed to configure output pins\n");
         tmc_gpio_deinit(ctx);
         return false;
     }

    // Configure input pins (INDEX and DIAG)
    if (gpiod_line_request_input(ctx->index_line, "tmc2209_index") < 0) {
        fprintf(stderr, "Failed to configure INDEX input pin\n");
        tmc_gpio_deinit(ctx);
        return false;
    }
    
    if (gpiod_line_request_input(ctx->diag_line, "tmc2209_diag") < 0) {
        fprintf(stderr, "Failed to configure DIAG input pin\n");
        tmc_gpio_deinit(ctx);
        return false;
    }
     
     // Set initial states
     gpiod_line_set_value(ctx->enable_line, TMC_GPIO_HIGH);  // Disable driver initially
     gpiod_line_set_value(ctx->step_line, TMC_GPIO_LOW);
     gpiod_line_set_value(ctx->dir_line, TMC_GPIO_LOW);
     
     ctx->initialized = true;
     printf("TMC2209 GPIO initialized successfully\n");
     
     return true;
 }
 
 void tmc_gpio_deinit(tmc_gpio_context_t *ctx) {
     if (!ctx) {
         return;
     }
     
     // Stop event monitoring if active
     if (ctx->event_monitoring_active) {
         tmc_gpio_stop_event_monitoring(ctx);
     }
     
     // Release GPIO lines
     if (ctx->enable_line) {
         gpiod_line_release(ctx->enable_line);
         ctx->enable_line = NULL;
     }
     if (ctx->step_line) {
         gpiod_line_release(ctx->step_line);
         ctx->step_line = NULL;
     }
     if (ctx->dir_line) {
         gpiod_line_release(ctx->dir_line);
         ctx->dir_line = NULL;
     }
     if (ctx->index_line) {
         gpiod_line_release(ctx->index_line);
         ctx->index_line = NULL;
     }
     if (ctx->diag_line) {
         gpiod_line_release(ctx->diag_line);
         ctx->diag_line = NULL;
     }
     
     // Close GPIO chip
     if (ctx->chip) {
         gpiod_chip_close(ctx->chip);
         ctx->chip = NULL;
     }
     
     ctx->initialized = false;
     printf("TMC2209 GPIO deinitialized\n");
 }
 
 bool tmc_gpio_write(tmc_gpio_context_t *ctx, uint8_t pin, uint8_t value) {
     struct gpiod_line *line = get_line_for_pin(ctx, pin);
     if (!line) {
         return false;
     }
     
     return gpiod_line_set_value(line, value) == 0;
 }
 
 int tmc_gpio_read(tmc_gpio_context_t *ctx, uint8_t pin) {
     struct gpiod_line *line = get_line_for_pin(ctx, pin);
     if (!line) {
         return -1;
     }
     
     return gpiod_line_get_value(line);
 }
 
 bool tmc_gpio_toggle(tmc_gpio_context_t *ctx, uint8_t pin) {
     int current_value = tmc_gpio_read(ctx, pin);
     if (current_value == -1) {
         return false;
     }
     
     return tmc_gpio_write(ctx, pin, current_value ? TMC_GPIO_LOW : TMC_GPIO_HIGH);
 }
 
 // ============================================================================
 // TMC2209 SPECIFIC GPIO FUNCTIONS
 // ============================================================================
 
 bool tmc_gpio_enable_driver(tmc_gpio_context_t *ctx, bool enable) {
     if (!ctx || !ctx->initialized) {
         return false;
     }
     
     // Enable pin is active low
     uint8_t value = enable ? TMC_GPIO_LOW : TMC_GPIO_HIGH;
     return gpiod_line_set_value(ctx->enable_line, value) == 0;
 }
 
 bool tmc_gpio_read_diagnostic(tmc_gpio_context_t *ctx) {
     if (!ctx || !ctx->initialized) {
         return false;
     }
     
     int value = gpiod_line_get_value(ctx->diag_line);
     return (value == TMC_GPIO_HIGH);
 }


 bool tmc_gpio_read_index(tmc_gpio_context_t *ctx) {
     if (!ctx || !ctx->initialized) {
         return false;
     }
     
     int value = gpiod_line_get_value(ctx->index_line);
     return (value == TMC_GPIO_HIGH);
 }

 // ============================================================================
 // GPIO INTERRUPT HANDLING FUNCTIONS
 // ============================================================================
 
 // Event monitoring thread function
 static void* gpio_event_monitoring_thread(void *arg) {
     tmc_gpio_context_t *ctx = (tmc_gpio_context_t*)arg;
     
     if (!ctx || !ctx->initialized) {
         printf("ERROR: Invalid GPIO context in event monitoring thread\n");
         return NULL;
     }
     
     printf("GPIO event monitoring thread started (DIAG pin only)\n");
     
     // Check if DIAG interrupt is enabled
     if (!ctx->diag_interrupt.enabled || !ctx->diag_interrupt.line) {
         printf("WARNING: DIAG interrupt not configured for event monitoring\n");
         return NULL;
     }
     
     while (!ctx->thread_should_exit) {
         struct gpiod_line_event event;
         struct timespec timeout = {0, 100000000}; // 100ms timeout
         
         // Wait for event on the DIAG line
         int ret = gpiod_line_event_wait(ctx->diag_line, &timeout);
         
         if (ret == 1) {
             // Event available, read it
             ret = gpiod_line_event_read(ctx->diag_line, &event);
             
             if (ret == 0) {
                 // Handle DIAG line interrupt
                 if (ctx->diag_interrupt.callback) {
                     bool rising_edge = (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE);
                     
                     // Update pulse statistics
                     uint64_t current_time = get_timestamp_us();
                     ctx->diag_interrupt.pulse_count++;
                     ctx->diag_interrupt.last_pulse_time = current_time;
                     
                     // Call user callback
                     ctx->diag_interrupt.callback(TMC_GPIO_DIAG_PIN, rising_edge);
                     
                     printf("DIAG pulse detected: %s edge, count: %u, time: %lu us\n",
                            rising_edge ? "RISING" : "FALLING",
                            ctx->diag_interrupt.pulse_count,
                            (unsigned long)current_time);
                 }
             } else if (ret == -1) {
                 // Error reading event
                 printf("ERROR: Error reading GPIO event: %s\n", strerror(errno));
                 break;
             }
         } else if (ret == -1) {
             // Error waiting for event
             if (errno != ETIMEDOUT) {
                 printf("ERROR: Error waiting for GPIO event: %s\n", strerror(errno));
                 break;
             }
             // Timeout is normal, continue loop
         }
         // ret == 0 means timeout, which is normal
     }
     
     printf("GPIO event monitoring thread exiting\n");
     return NULL;
 }
 
 bool tmc_gpio_setup_diag_interrupt(tmc_gpio_context_t *ctx, 
                                   tmc_gpio_interrupt_callback_t callback) {
     if (!ctx || !ctx->initialized || !callback) {
         printf("ERROR: Invalid parameters for DIAG interrupt setup\n");
         return false;
     }
     
     if (!ctx->diag_line) {
         printf("ERROR: DIAG line not available for interrupt setup\n");
         return false;
     }
     
     // Release the current DIAG line request
     gpiod_line_release(ctx->diag_line);
     
     // Configure the DIAG line for edge detection (both edges for StallGuard)
     if (gpiod_line_request_both_edges_events(ctx->diag_line, "tmc2209_diag_interrupt") < 0) {
         printf("ERROR: Failed to configure DIAG pin for edge detection\n");
         return false;
     }
     
     // Setup DIAG interrupt context
     ctx->diag_interrupt.line = ctx->diag_line;
     ctx->diag_interrupt.callback = callback;
     ctx->diag_interrupt.enabled = true;
     ctx->diag_interrupt.last_state = false;
     ctx->diag_interrupt.pulse_count = 0;
     ctx->diag_interrupt.last_pulse_time = 0;
     ctx->diag_interrupt.pin_offset = TMC_GPIO_DIAG_PIN;
     
     printf("DIAG interrupt setup complete\n");
     return true;
 }
 
 bool tmc_gpio_start_event_monitoring(tmc_gpio_context_t *ctx) {
     if (!ctx || !ctx->initialized) {
         printf("ERROR: Invalid GPIO context for event monitoring start\n");
         return false;
     }
     
     if (ctx->event_monitoring_active) {
         printf("WARNING: GPIO event monitoring already active\n");
         return true;
     }
     
     ctx->thread_should_exit = false;
     
     // Create event monitoring thread
     if (pthread_create(&ctx->event_thread, NULL, gpio_event_monitoring_thread, ctx) != 0) {
         printf("ERROR: Failed to create GPIO event monitoring thread\n");
         return false;
     }
     
     ctx->event_monitoring_active = true;
     printf("GPIO event monitoring started\n");
     return true;
 }
 
 void tmc_gpio_stop_event_monitoring(tmc_gpio_context_t *ctx) {
     if (!ctx || !ctx->event_monitoring_active) {
         return;
     }
     
     ctx->thread_should_exit = true;
     
     // Wait for thread to exit
     if (ctx->event_thread != 0) {
         pthread_join(ctx->event_thread, NULL);
         ctx->event_thread = 0;
     }
     
     ctx->event_monitoring_active = false;
     printf("GPIO event monitoring stopped\n");
 }
 
 bool tmc_gpio_is_event_monitoring_active(tmc_gpio_context_t *ctx) {
     if (!ctx) {
         return false;
     }
     return ctx->event_monitoring_active;
 }

 // ============================================================================
 // UTILITY FUNCTIONS
 // ============================================================================
 
 tmc_gpio_config_t tmc_gpio_get_default_config(void) {
     tmc_gpio_config_t config = {
         .enable_pin = TMC_GPIO_ENABLE_PIN,
         .step_pin = TMC_GPIO_STEP_PIN,
         .dir_pin = TMC_GPIO_DIR_PIN,
         .diag_pin = TMC_GPIO_DIAG_PIN,
         .index_pin = TMC_GPIO_INDEX_PIN
     };
     return config;
 }
 
 void tmc_gpio_print_status(tmc_gpio_context_t *ctx) {
     if (!ctx || !ctx->initialized) {
         printf("GPIO context not initialized\n");
         return;
     }
     
     printf("TMC2209 GPIO Status:\n");
     printf("  Enable:     %s\n", gpiod_line_get_value(ctx->enable_line) ? "HIGH" : "LOW");
     printf("  Step:       %s\n", gpiod_line_get_value(ctx->step_line) ? "HIGH" : "LOW");
     printf("  Direction:  %s\n", gpiod_line_get_value(ctx->dir_line) ? "HIGH" : "LOW");
     printf("  Diagnostic: %s\n", gpiod_line_get_value(ctx->diag_line) ? "HIGH" : "LOW");
     printf("  Index:      %s\n", gpiod_line_get_value(ctx->index_line) ? "HIGH" : "LOW");
     
     if (ctx->diag_interrupt.enabled) {
         printf("  DIAG Interrupt: Enabled (pulse count: %u)\n", ctx->diag_interrupt.pulse_count);
     }
 }
 
 void tmc_gpio_delay_us(uint32_t us) {
     delay_us(us);
 }