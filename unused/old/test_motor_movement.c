#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "tmc2209.h"
#include "tmc_gpio.h"

// Calculate VACTUAL value for given speed in μsteps/sec
int32_t calculate_vactual(float speed_usteps_per_sec) {
    // VACTUAL = speed / 0.715 (from TMC2209 datasheet)
    return (int32_t)(speed_usteps_per_sec / 0.715f);
}

// Calculate speed in μsteps/sec for one rotation in given time
float calculate_speed_for_rotation(int microsteps_per_rev, float duration_seconds) {
    return (float)microsteps_per_rev / duration_seconds;
}

/**
 * @brief Monitor motor position using MSCNT register
 * 
 * @param driver TMC2209 driver structure
 * @param duration_ms Duration to monitor in milliseconds
 * @return Number of full steps completed
 */
 uint32_t tmc_gpio_monitor_mscnt_steps(TMC2209_t *driver, uint32_t duration_ms) {
    if (!driver) {
        return 0;
    }
    
    uint32_t total_steps = 0;
    uint32_t elapsed_ms = 0;
    uint16_t last_mscnt = 0;
    bool first_read = true;
    
    printf("Monitoring MSCNT for %u ms...\n", duration_ms);
    
    while (elapsed_ms < duration_ms) {
        // Read MSCNT register
        if (TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->mscnt)) {
            uint16_t current_mscnt = driver->mscnt.reg.mscnt;
            
            if (!first_read) {
                // Calculate steps since last read
                int16_t step_diff;
                if (current_mscnt >= last_mscnt) {
                    step_diff = current_mscnt - last_mscnt;
                } else {
                    // Handle wrap-around from 1023 to 0
                    step_diff = (1024 - last_mscnt) + current_mscnt;
                }
                
                // Convert microsteps to full steps
                uint32_t full_steps = step_diff / driver->config.microsteps;
                total_steps += full_steps;
                
                if (full_steps > 0) {
                    printf("MSCNT: %u -> %u, diff: %d μsteps = %u full steps\n", 
                           last_mscnt, current_mscnt, step_diff, full_steps);
                }
            }
            
            last_mscnt = current_mscnt;
            first_read = false;
        }
        
        // detecting change in GPIO
        usleep(10000); // 10ms polling interval
        elapsed_ms += 10;
    }
    
    printf("MSCNT monitoring: %u full steps in %u ms\n", total_steps, elapsed_ms);
    return total_steps;
}

int main() {
    TMC2209_t driver;
    tmc_gpio_context_t gpio_ctx;
    tmc_gpio_config_t gpio_config;

      // Initialize GPIO with error checking
    printf("Initializing GPIO...\n");
    gpio_config = tmc_gpio_get_default_config();
    if (!tmc_gpio_init(&gpio_ctx, &gpio_config, "gpiochip0")) {
        printf("ERROR: Failed to initialize GPIO\n");
        return -1;
    }
    printf("SUCCESS: GPIO initialized\n");
    
    // Print initial GPIO status
    printf("Initial GPIO status:\n");
    tmc_gpio_print_status(&gpio_ctx);
    printf("\n");

    printf("TMC2209 Motor Movement Test (VACTUAL Register)\n");
    printf("==============================================\n\n");
    
    // Initialize driver with defaults
    TMC2209_SetDefaults(&driver);
    
    // Configure motor settings
    driver.config.motor.address = 0;  // Default address
    driver.config.current = 500;      // 500mA
    driver.config.microsteps = 4;     // 4 microsteps
    
    printf("Motor Configuration:\n");
    printf("- Current: %d mA\n", driver.config.current);
    printf("- Microsteps: %d\n", driver.config.microsteps);
    printf("- Full steps per revolution: 200\n");
    printf("- μsteps per revolution: %d\n", 200 * driver.config.microsteps);
    printf("\n");
    
    printf("Initializing TMC2209...\n");
    if (!TMC2209_Init(&driver)) {
        printf("ERROR: Failed to initialize TMC2209\n");
        return -1;
    }
    printf("SUCCESS: TMC2209 initialized\n\n");
    
    // Read initial register states
    printf("Reading initial register states:\n");
    
    if (TMC2209_ReadRegister(&driver, (TMC2209_datagram_t *)&driver.gconf)) {
        printf("GCONF: 0x%08X\n", driver.gconf.reg.value);
    }
    
    if (TMC2209_ReadRegister(&driver, (TMC2209_datagram_t *)&driver.ioin)) {
        printf("IOIN: 0x%08X (Version: 0x%02X)\n", 
               driver.ioin.reg.value, driver.ioin.reg.version);
    }
    
    printf("\n");
    
    // Calculate movement parameters
    // 
    int microsteps_per_rev = 600 * driver.config.microsteps;  // 800 μsteps
    float rotation_duration = 3.0f;  // 3 seconds for one rotation
    float speed_usteps_per_sec = calculate_speed_for_rotation(microsteps_per_rev, rotation_duration);
    int32_t vactual_value = calculate_vactual(speed_usteps_per_sec);
    
    printf("Movement Parameters:\n");
    printf("- μsteps per revolution: %d\n", microsteps_per_rev);
    printf("- Rotation duration: %.1f seconds\n", rotation_duration);
    printf("- Speed: %.1f μsteps/sec\n", speed_usteps_per_sec);
    printf("- VACTUAL value: %d\n", vactual_value);
    printf("\n");

    // Enable motor
    printf("Enabling motor...\n");
    tmc_gpio_enable_driver(&gpio_ctx, true);
    printf("Motor enabled\n");

    // Test 1: Clockwise rotation
    printf("=== Test 1: Clockwise Rotation ===\n");
    printf("Setting VACTUAL to %d (clockwise)...\n", vactual_value);
    
    
    driver.vactual.reg.actual = vactual_value;
    if (TMC2209_WriteRegister(&driver, (TMC2209_datagram_t *)&driver.vactual)) {
        printf("VACTUAL set successfully\n");
    } else {
        printf("ERROR: Failed to set VACTUAL\n");
        return -1;
    }
    
    printf("Motor should now be rotating clockwise...\n");
    sleep((int)rotation_duration);
    float expected_rpm = (speed_usteps_per_sec * 60.0f) / (200.0f * driver.config.microsteps);
    printf("- Expected speed: %.2f RPM\n", expected_rpm);

    float measured_rpm_tstep = TMC2209_GetSpeedRPM(&driver);
    printf("- Measured speed from TSTEP: %.2f RPM\n", measured_rpm_tstep);
    
    printf("Stopping motor...\n");
    driver.vactual.reg.actual = 0;
    TMC2209_WriteRegister(&driver, (TMC2209_datagram_t *)&driver.vactual);
    sleep((int)rotation_duration);
    printf("Clockwise rotation completed.\n\n");
    
    // Wait a moment before next test
    sleep(2);
    
    // Test 2: Counter-clockwise rotation
    printf("=== Test 2: Counter-clockwise Rotation ===\n");
    printf("Setting VACTUAL to %d (counter-clockwise)...\n", -vactual_value);
    
    driver.vactual.reg.actual = -vactual_value;
    if (TMC2209_WriteRegister(&driver, (TMC2209_datagram_t *)&driver.vactual)) {
        printf("VACTUAL set successfully\n");
    } else {
        printf("ERROR: Failed to set VACTUAL\n");
        return -1;
    }
    
    printf("Motor should now be rotating counter-clockwise...\n");
    printf("Waiting %.1f seconds for one rotation...\n", rotation_duration);
    
    // Wait for rotation to complete
    sleep((int)rotation_duration);
    
    printf("Stopping motor...\n");
    driver.vactual.reg.actual = 0;
    TMC2209_WriteRegister(&driver, (TMC2209_datagram_t *)&driver.vactual);
    
    printf("Counter-clockwise rotation completed.\n\n");
    
    printf("Disabling motor...\n");
    tmc_gpio_enable_driver(&gpio_ctx, false);
    printf("Motor disabled\n");
    
    printf("\n=== Test Completed Successfully! ===\n");
    printf("The motor should have completed:\n");
    printf("1. One clockwise rotation (3 seconds)\n");
    printf("2. One counter-clockwise rotation (3 seconds)\n");
    printf("3. Returned to stopped state\n");
    
    printf("Closing TMC2209...\n");
    if (!TMC2209_Init(&driver)) {
        printf("ERROR: Failed to close TMC2209\n");
        return -1;
    }

    printf("Closing GPIO...\n");
    tmc_gpio_deinit(&gpio_ctx);
    printf("GPIO closed\n");

    return 0;
}