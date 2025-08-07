#ifndef _TMC_STALLGUARD_H_
#define _TMC_STALLGUARD_H_

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include "tmc2209.h"
#include "tmc_gpio.h"

// StallGuard callback function type
typedef void (*stallguard_callback_t)(void);

// StallGuard configuration structure
typedef struct {
    int diag_pin;                    // GPIO pin connected to DIAG output
    uint32_t threshold;              // StallGuard threshold (SGTHRS)
    uint32_t min_speed_threshold;    // Minimum speed for StallGuard (TCOOLTHRS)
    stallguard_callback_t callback;  // User callback function
} tmc_stallguard_config_t;

// Function declarations
bool tmc_stallguard_init(TMC2209_t *driver, tmc_stallguard_config_t *config);
void tmc_stallguard_deinit(TMC2209_t *driver);

bool tmc_stallguard_set_callback(TMC2209_t *driver, int diag_pin, uint32_t threshold, 
                                stallguard_callback_t callback, uint32_t min_speed);

// Setup StallGuard monitoring using existing GPIO infrastructure
bool tmc_stallguard_setup_monitoring(TMC2209_t *driver, tmc_gpio_context_t *gpio_ctx, 
                                    stallguard_callback_t callback);

uint32_t tmc_stallguard_get_result(TMC2209_t *driver);
bool tmc_stallguard_set_threshold(TMC2209_t *driver, uint32_t threshold);
bool tmc_stallguard_set_tcoolthrs_threshold(TMC2209_t *driver, uint32_t threshold);

// Check if StallGuard is triggered (for integration with motion control)
bool tmc_stallguard_check_triggered(TMC2209_t *driver, tmc_gpio_context_t *gpio_ctx);

// Reset the triggered state (call after handling a stall event)
void tmc_stallguard_reset_triggered(void);

// Debug function to check StallGuard configuration
void tmc_stallguard_debug_status(TMC2209_t *driver);

#endif // _TMC_STALLGUARD_H_ 