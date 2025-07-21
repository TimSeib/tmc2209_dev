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
 }
 
 void tmc_gpio_delay_us(uint32_t us) {
     delay_us(us);
 }