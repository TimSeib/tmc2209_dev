/*
 * tmc_gpio.h - GPIO control for TMC2209 stepper motor driver
 *
 * This file provides GPIO functionality using libgpiod for controlling
 * TMC2209 stepper motor driver pins including enable, step, direction,
 * and diagnostic pins.
 *
 * v1.0.0 / 2024-12-19
 */

 #ifndef _TMC_GPIO_H_
 #define _TMC_GPIO_H_
 
 #include <stdint.h>
 #include <stdbool.h>
 #include <gpiod.h>
 
 // ============================================================================
 // GPIO PIN DEFINITIONS
 // ============================================================================
 
 // GPIO pin assignments for TMC2209
 #define TMC_GPIO_ENABLE_PIN        21  // Enable pin (active low)
 #define TMC_GPIO_STEP_PIN          16  // Step input pin
 #define TMC_GPIO_DIR_PIN           20  // Direction input pin
 #define TMC_GPIO_DIAG_PIN          22  // Diagnostic pin (stall detection)
 #define TMC_GPIO_INDEX_PIN         26  // Index pin
 
 // GPIO pin states
 #define TMC_GPIO_LOW               0
 #define TMC_GPIO_HIGH              1
 
 // GPIO directions
 #define TMC_GPIO_INPUT             0
 #define TMC_GPIO_OUTPUT            1
 
 // Pull-up/down configurations
 #define TMC_GPIO_PULL_NONE         0
 #define TMC_GPIO_PULL_UP           1
 #define TMC_GPIO_PULL_DOWN         2

 // ============================================================================
 // INTERRUPT CALLBACK TYPES
 // ============================================================================

typedef void (*tmc_gpio_interrupt_callback_t)(uint8_t pin, bool rising_edge);

// ============================================================================
// INTERRUPT CONTEXT STRUCTURE
// ============================================================================

typedef struct {
    struct gpiod_line *line;                    // GPIO line handle
    tmc_gpio_interrupt_callback_t callback;     // User callback function
    bool enabled;                               // Interrupt enabled flag
    bool last_state;                            // Last known pin state
    uint32_t pulse_count;                       // Pulse counter
    uint64_t last_pulse_time;                   // Timestamp of last pulse
} tmc_gpio_interrupt_context_t;

 // ============================================================================
 // GPIO PIN CONFIGURATION STRUCTURE
 // ============================================================================
 
 typedef struct {
     uint8_t enable_pin;      // Enable pin number
     uint8_t step_pin;        // Step pin number
     uint8_t dir_pin;         // Direction pin number
     uint8_t diag_pin;        // Diagnostic pin
     uint8_t index_pin;       // Index pin
 } tmc_gpio_config_t;
 
 // ============================================================================
 // GPIO CONTEXT STRUCTURE
 // ============================================================================
 
 typedef struct {
     struct gpiod_chip *chip;         // GPIO chip handle
     struct gpiod_line *enable_line;  // Enable line handle
     struct gpiod_line *step_line;    // Step line handle
     struct gpiod_line *dir_line;     // Direction line handle
     struct gpiod_line *diag_line;    // Diagnostic line handle
     struct gpiod_line *index_line;   // Index line handle

     tmc_gpio_interrupt_context_t index_interrupt;   // INDEX pin interrupt
     tmc_gpio_interrupt_context_t diag_interrupt;    // DIAG pin interrupt

      // Event monitoring
     struct gpiod_line_bulk *event_lines;        // Lines to monitor for events
     bool event_monitoring_active;               // Event monitoring thread active
     pthread_t event_thread;                     // Event monitoring thread
     bool thread_should_exit;                    // Thread exit flag

     bool initialized;                // Initialization status
 } tmc_gpio_context_t;
 
 // ============================================================================
 // FUNCTION DECLARATIONS
 // ============================================================================
 
 /**
  * @brief Initialize GPIO system for TMC2209
  * 
  * @param ctx GPIO context structure
  * @param config GPIO pin configuration
  * @param chip_name GPIO chip name (e.g., "gpiochip0")
  * @return true on success, false on failure
  */
 bool tmc_gpio_init(tmc_gpio_context_t *ctx, const tmc_gpio_config_t *config, const char *chip_name);
 
 /**
  * @brief Deinitialize GPIO system
  * 
  * @param ctx GPIO context structure
  */
 void tmc_gpio_deinit(tmc_gpio_context_t *ctx);
 
 
 /**
  * @brief Write value to GPIO pin
  * 
  * @param ctx GPIO context structure
  * @param pin Pin number
  * @param value GPIO_LOW or GPIO_HIGH
  * @return true on success, false on failure
  */
 bool tmc_gpio_write(tmc_gpio_context_t *ctx, uint8_t pin, uint8_t value);
 
 /**
  * @brief Read value from GPIO pin
  * 
  * @param ctx GPIO context structure
  * @param pin Pin number
  * @return GPIO_LOW or GPIO_HIGH, -1 on error
  */
 int tmc_gpio_read(tmc_gpio_context_t *ctx, uint8_t pin);
 
 /**
  * @brief Toggle GPIO pin value
  * 
  * @param ctx GPIO context structure
  * @param pin Pin number
  * @return true on success, false on failure
  */
 bool tmc_gpio_toggle(tmc_gpio_context_t *ctx, uint8_t pin);
 
 // ============================================================================
 // TMC2209 SPECIFIC GPIO FUNCTIONS
 // ============================================================================
 
 /**
  * @brief Enable/disable TMC2209 driver
  * 
  * @param ctx GPIO context structure
  * @param enable true to enable, false to disable
  * @return true on success, false on failure
  */
 bool tmc_gpio_enable_driver(tmc_gpio_context_t *ctx, bool enable);
 
 /** 
  * @brief Read diagnostic pin state
  * 
  * @param ctx GPIO context structure
  * @return true if diagnostic condition detected, false otherwise
  */
 bool tmc_gpio_read_diagnostic(tmc_gpio_context_t *ctx);

 /** 
  * @brief Read index pin state
  * 
  * @param ctx GPIO context structure
  * @return true if index condition detected, false otherwise
  */
  bool tmc_gpio_read_index(tmc_gpio_context_t *ctx);

 // ============================================================================
 // UTILITY FUNCTIONS
 // ============================================================================
 
 /**
  * @brief Get default GPIO configuration
  * 
  * @return Default GPIO pin configuration
  */
 tmc_gpio_config_t tmc_gpio_get_default_config(void);
 
 /**
  * @brief Print GPIO pin states for debugging
  * 
  * @param ctx GPIO context structure
  */
 void tmc_gpio_print_status(tmc_gpio_context_t *ctx);
 
 /**
  * @brief Wait for specified number of microseconds
  * 
  * @param us Microseconds to wait
  */
 void tmc_gpio_delay_us(uint32_t us);
 
 #endif // _TMC_GPIO_H_