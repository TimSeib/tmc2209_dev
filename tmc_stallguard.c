#include "tmc_stallguard.h"
#include "log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <pthread.h>
#include <gpiod.h>
#include <errno.h>

// Global variables for StallGuard state
static volatile bool g_stallguard_triggered = false;
static stallguard_callback_t g_stallguard_callback = NULL;
static TMC2209_t *g_stallguard_driver = NULL;
static tmc_gpio_context_t *g_stallguard_gpio = NULL;

// StallGuard configuration
static tmc_stallguard_config_t g_stallguard_config = {0};

// Mutex for thread safety
static pthread_mutex_t g_stallguard_mutex = PTHREAD_MUTEX_INITIALIZER;

// GPIO interrupt callback for DIAG pin (integrated with existing GPIO infrastructure)
static void stallguard_gpio_callback(uint8_t pin, bool rising_edge) {
    (void)pin; // Suppress unused parameter warning
    
    if (rising_edge) {
        log_warn("StallGuard triggered! DIAG pin went HIGH");
        
        // Set the triggered flag
        pthread_mutex_lock(&g_stallguard_mutex);
        g_stallguard_triggered = true;
        pthread_mutex_unlock(&g_stallguard_mutex);
        
        // Call user callback if set
        if (g_stallguard_callback) {
            g_stallguard_callback();
        }
    }
}

bool tmc_stallguard_init(TMC2209_t *driver, tmc_stallguard_config_t *config) {
    if (!driver || !config) {
        log_error("Invalid parameters for StallGuard init");
        return false;
    }
    
    // Store configuration
    memcpy(&g_stallguard_config, config, sizeof(tmc_stallguard_config_t));
    g_stallguard_driver = driver;
    
    // Set initial registers
    if (!tmc_stallguard_set_threshold(driver, config->threshold)) {
        log_error("Failed to set StallGuard threshold");
        return false;
    }
    
    if (!tmc_stallguard_set_tcoolthrs_threshold(driver, config->min_speed_threshold)) {
        log_error("Failed to set minimum speed threshold");
        return false;
    }
    
    log_info("StallGuard initialized: DIAG pin=%d, threshold=%u, min_speed=%u", 
             config->diag_pin, config->threshold, config->min_speed_threshold);
    
    return true;
}

void tmc_stallguard_deinit(TMC2209_t *driver) {
    (void)driver; // Suppress unused parameter warning

    // Reset global variables
    g_stallguard_callback = NULL;
    g_stallguard_driver = NULL;
    g_stallguard_gpio = NULL;
    memset(&g_stallguard_config, 0, sizeof(tmc_stallguard_config_t));
    
    log_debug("StallGuard deinitialized");
}

bool tmc_stallguard_set_callback(TMC2209_t *driver, int diag_pin, uint32_t threshold, 
                                stallguard_callback_t callback, uint32_t min_speed) {
    if (!driver || !callback) {
        log_error("Invalid parameters for StallGuard callback setup");
        return false;
    }
    
    // Update configuration
    g_stallguard_config.diag_pin = diag_pin;
    g_stallguard_config.threshold = threshold;
    g_stallguard_config.min_speed_threshold = min_speed;
    g_stallguard_config.callback = callback;
    g_stallguard_driver = driver;
    
    // Set registers
    if (!tmc_stallguard_set_threshold(driver, threshold)) {
        return false;
    }
    
    if (!tmc_stallguard_set_tcoolthrs_threshold(driver, min_speed)) {
        return false;
    }
    
    log_info("StallGuard callback set: pin=%d, threshold=%u, min_speed=%u", 
             diag_pin, threshold, min_speed);
    
    return true;
}

// Setup StallGuard monitoring using existing GPIO infrastructure
bool tmc_stallguard_setup_monitoring(TMC2209_t *driver, tmc_gpio_context_t *gpio_ctx, 
                                    stallguard_callback_t callback) {
    if (!driver || !gpio_ctx || !callback) {
        log_error("Invalid parameters for StallGuard monitoring setup");
        return false;
    }
    
    // Store references
    g_stallguard_driver = driver;
    g_stallguard_gpio = gpio_ctx;
    g_stallguard_callback = callback;
    
         // Setup DIAG pin interrupt using existing GPIO infrastructure
     if (!tmc_gpio_setup_diag_interrupt(gpio_ctx, stallguard_gpio_callback)) {
         log_error("Failed to setup DIAG pin interrupt");
         return false;
     }
    
    // Start GPIO event monitoring
    if (!tmc_gpio_start_event_monitoring(gpio_ctx)) {
        log_error("Failed to start GPIO event monitoring");
        return false;
    }
    
    log_info("StallGuard monitoring setup complete using existing GPIO infrastructure");
    return true;
}

// Check if StallGuard is triggered (for integration with motion control)
bool tmc_stallguard_check_triggered(TMC2209_t *driver, tmc_gpio_context_t *gpio_ctx) {
    (void)driver; // Suppress unused parameter warning
    (void)gpio_ctx; // Suppress unused parameter warning
    
    pthread_mutex_lock(&g_stallguard_mutex);
    bool triggered = g_stallguard_triggered;
    pthread_mutex_unlock(&g_stallguard_mutex);
    
    return triggered;
}

// Reset the triggered state (call after handling a stall event)
void tmc_stallguard_reset_triggered(void) {
    pthread_mutex_lock(&g_stallguard_mutex);
    g_stallguard_triggered = false;
    pthread_mutex_unlock(&g_stallguard_mutex);
    log_debug("StallGuard triggered state reset");
}

uint32_t tmc_stallguard_get_result(TMC2209_t *driver) {
    if (!driver) {
        return 0;
    }
    
    TMC2209_sg_result_dgr_t sgresult;
    memset(&sgresult, 0, sizeof(TMC2209_sg_result_dgr_t));
    sgresult.addr.reg = TMC2209Reg_SG_RESULT;
    
    if (!TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&sgresult)) {
        log_error("Failed to read SGRESULT register");
        return 0;
    }
    
    return sgresult.reg.result;
}

bool tmc_stallguard_set_threshold(TMC2209_t *driver, uint32_t threshold) {
    if (!driver) {
        return false;
    }
    
    driver->sgthrs.addr.reg = TMC2209Reg_SGTHRS;
    driver->sgthrs.reg.threshold = threshold;

    if (!TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->sgthrs)) {
        log_error("Failed to write SGTHRS register");
        return false;
    }
    return true;
}

bool tmc_stallguard_set_tcoolthrs_threshold(TMC2209_t *driver, uint32_t threshold) {
    if (!driver) {
        return false;
    }
    
    // Use the driver's register structure (same as TMC2209 driver does)
    driver->tcoolthrs.addr.reg = TMC2209Reg_TCOOLTHRS;  // Ensure address is set
    driver->tcoolthrs.reg.tcoolthrs = threshold & 0xFFFFF;
    
    if (!TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->tcoolthrs)) {
        log_error("Failed to write TCOOLTHRS register");
        return false;
    }
    
    log_debug("TCOOLTHRS threshold set to %u", threshold);
    return true;
}

// Debug function to check StallGuard configuration and status
void tmc_stallguard_debug_status(TMC2209_t *driver) {
    if (!driver) {
        log_error("Invalid driver pointer for StallGuard debug");
        return;
    }
    
    log_info("=== StallGuard Debug Status ===");
    
    // Check GCONF (StealthChop mode)
    TMC2209_gconf_dgr_t gconf;
    memset(&gconf, 0, sizeof(TMC2209_gconf_dgr_t));
    gconf.addr.reg = TMC2209Reg_GCONF;
    
    if (TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&gconf)) {
        log_info("GCONF.en_spreadcycle = %u (%s)", 
                 gconf.reg.en_spreadcycle,
                 gconf.reg.en_spreadcycle ? "SpreadCycle" : "StealthChop");
    } else {
        log_error("Failed to read GCONF register");
    }
    
    // Check CHOPCONF (StallGuard enable)
    TMC2209_chopconf_dgr_t chopconf;
    memset(&chopconf, 0, sizeof(TMC2209_chopconf_dgr_t));
    chopconf.addr.reg = TMC2209Reg_CHOPCONF;
    
    if (TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&chopconf)) {
        log_info("CHOPCONF.diss2g = %u (%s)", 
                 chopconf.reg.diss2g,
                 chopconf.reg.diss2g ? "StallGuard DISABLED" : "StallGuard ENABLED");
    } else {
        log_error("Failed to read CHOPCONF register");
    }
    
    // Check SGTHRS (threshold)
    TMC2209_sgthrs_dgr_t sgthrs;
    memset(&sgthrs, 0, sizeof(TMC2209_sgthrs_dgr_t));
    sgthrs.addr.reg = TMC2209Reg_SGTHRS;
    
    if (TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&sgthrs)) {
        log_info("SGTHRS.threshold = %u (lower = more sensitive)", sgthrs.reg.threshold);
    } else {
        log_error("Failed to read SGTHRS register");
    }
    
    // Check TCOOLTHRS (minimum speed threshold)
    TMC2209_tcoolthrs_dgr_t tcoolthrs;
    memset(&tcoolthrs, 0, sizeof(TMC2209_tcoolthrs_dgr_t));
    tcoolthrs.addr.reg = TMC2209Reg_TCOOLTHRS;
    
    if (TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&tcoolthrs)) {
        log_info("TCOOLTHRS.tcoolthrs = %u", tcoolthrs.reg.tcoolthrs);
    } else {
        log_error("Failed to read TCOOLTHRS register");
    }
    
    // Check current SG_RESULT
    uint32_t sg_result = tmc_stallguard_get_result(driver);
    log_info("SG_RESULT.result = %u (lower = closer to stall)", sg_result);
    
    // Check DRV_STATUS for StealthChop mode
    TMC2209_drv_status_dgr_t drv_status;
    memset(&drv_status, 0, sizeof(TMC2209_drv_status_dgr_t));
    drv_status.addr.reg = TMC2209Reg_DRV_STATUS;
    
    if (TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&drv_status)) {
        log_info("DRV_STATUS.stealth = %u (%s)", 
                 drv_status.reg.stealth,
                 drv_status.reg.stealth ? "StealthChop ACTIVE" : "SpreadCycle ACTIVE");
        log_info("DRV_STATUS.stst = %u (%s)", 
                 drv_status.reg.stst,
                 drv_status.reg.stst ? "Motor STANDSTILL" : "Motor MOVING");
    } else {
        log_error("Failed to read DRV_STATUS register");
    }
    
    log_info("=== End StallGuard Debug ===");
} 