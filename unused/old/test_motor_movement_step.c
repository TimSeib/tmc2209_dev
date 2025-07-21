#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "tmc2209.h"
#include "tmc_gpio.h"

int main() {
    TMC2209_t driver;
    tmc_gpio_context_t gpio_ctx;
    tmc_gpio_config_t gpio_config;

    printf("TMC2209 Simple Step/DIR Test\n");
    printf("============================\n\n");
    
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
    
    // Enable motor
    printf("Enabling motor...\n");
    tmc_gpio_enable_driver(&gpio_ctx, true);
    
    // Test 1: 200 steps clockwise (1 full revolution)
    printf("Test 1: 200 steps clockwise (1 full revolution)\n");
    tmc_gpio_write(&gpio_ctx, TMC_GPIO_DIR_PIN, TMC_GPIO_HIGH);
    
    for (int i = 0; i < 800; i++) {
        tmc_gpio_write(&gpio_ctx, TMC_GPIO_STEP_PIN, TMC_GPIO_HIGH);
        usleep(1000);  // 1ms pulse
        tmc_gpio_write(&gpio_ctx, TMC_GPIO_STEP_PIN, TMC_GPIO_LOW);
        usleep(1000);  // 1ms between steps
    }
    printf("Test 1 complete\n\n");
    
    sleep(1);
    
    // Test 2: 200 steps counter-clockwise (1 full revolution back)
    printf("Test 2: 200 steps counter-clockwise (1 full revolution back)\n");
    tmc_gpio_write(&gpio_ctx, TMC_GPIO_DIR_PIN, TMC_GPIO_LOW);
    
    for (int i = 0; i < 800; i++) {
        tmc_gpio_write(&gpio_ctx, TMC_GPIO_STEP_PIN, TMC_GPIO_HIGH);
        usleep(1000);  // 1ms pulse
        tmc_gpio_write(&gpio_ctx, TMC_GPIO_STEP_PIN, TMC_GPIO_LOW);
        usleep(1000);  // 1ms between steps
    }
    printf("Test 2 complete\n\n");
    
    // Disable motor
    printf("Disabling motor...\n");
    tmc_gpio_enable_driver(&gpio_ctx, false);
    
    printf("Test completed successfully!\n");
    printf("Motor should have moved 1 full revolution clockwise, then 1 full revolution counter-clockwise\n");
    
    // Cleanup
    tmc_gpio_deinit(&gpio_ctx);
    
    return 0;
}