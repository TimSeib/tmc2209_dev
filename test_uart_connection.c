// File to test the UART connection

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "tmc2209.h"
#include "tmc_gpio.h"

static tmc_gpio_context_t g_gpio_ctx;

int main() {
    TMC2209_t driver;
    
    printf("TMC2209 UART Communication Test\n");
    printf("===============================\n\n");
    printf("Initializing GPIO...\n");
    tmc_gpio_config_t gpio_config = tmc_gpio_get_default_config();
    if (!tmc_gpio_init(&g_gpio_ctx, &gpio_config, "gpiochip0")) {
        printf("Failed to initialize GPIO\n");
        return -1;
    }
    tmc_gpio_enable_driver(&g_gpio_ctx, true);
    // Initialize driver with defaults
    TMC2209_SetDefaults(&driver);

    // Configure motor settings
    driver.config.motor.address = 0;  // Default address
    driver.config.current = 670;      // 500mA
    driver.config.microsteps = 4;     // 4 microsteps
    
    printf("Initializing TMC2209...\n");
    if (!TMC2209_Init(&driver)) {
        printf("ERROR: Failed to initialize TMC2209\n");
        return -1;
    }
    printf("SUCCESS: TMC2209 initialized\n\n");
    usleep(1000000); // wait for the driver to settle for 1 s

    // Test reading registers
    printf("Reading registers:\n");
    
    // Read GSTAT register
    if (TMC2209_ReadRegister(&driver, (TMC2209_datagram_t *)&driver.gstat)) {
        printf("GSTAT: 0x%08X\n", driver.gstat.reg.value);
    } else {
        printf("ERROR: Failed to read GSTAT\n");
    }
    
    // Read IFCNT register
    if (TMC2209_ReadRegister(&driver, (TMC2209_datagram_t *)&driver.ifcnt)) {
        printf("IFCNT: 0x%08X\n", driver.ifcnt.reg.value);
    } else {
        printf("ERROR: Failed to read IFCNT\n");
    }
    
    // Read IOIN register
    if (TMC2209_ReadRegister(&driver, (TMC2209_datagram_t *)&driver.ioin)) {
        printf("IOIN: 0x%08X (Version: 0x%02X)\n", 
               driver.ioin.reg.value, driver.ioin.reg.version);
    } else {
        printf("ERROR: Failed to read IOIN\n");
    }
    
    // Read GCONFregister
    if (TMC2209_ReadRegister(&driver, (TMC2209_datagram_t *)&driver.gconf)) {
        printf("GCONF: 0x%08X\n", driver.gconf.reg.value);
    } else {
        printf("ERROR: Failed to read GCONF\n");
    }

    // Write GCONF register
    driver.gconf.reg.index_otpw = 1;
    if(TMC2209_WriteRegister(&driver, (TMC2209_datagram_t *)&driver.gconf)) {
        printf("GCONF: 0x%08X\n", driver.gconf.reg.value);
    } else {
        printf("ERROR: Failed to write GCONF\n");
    }
    // Read GCONFregister
    if (TMC2209_ReadRegister(&driver, (TMC2209_datagram_t *)&driver.gconf)) {
        printf("GCONF: 0x%08X\n", driver.gconf.reg.value);
    } else {
        printf("ERROR: Failed to read GCONF\n");
    }
    
    // Read CHOPCONF register
    if (TMC2209_ReadRegister(&driver, (TMC2209_datagram_t *)&driver.chopconf)) {
        printf("CHOPCONF: 0x%08X\n", driver.chopconf.reg.value);
    } else {
        printf("ERROR: Failed to read CHOPCONF\n");
    }

    driver.tcoolthrs.reg.tcoolthrs = 1000;
    if(TMC2209_WriteRegister(&driver, (TMC2209_datagram_t *)&driver.tcoolthrs)) {
        printf("TCOOLTHRS: 0x%08X\n", driver.tcoolthrs.reg.value);
    } else {
        printf("ERROR: Failed to write TCOOLTHRS\n");
    }
    // Read TCOOLTHRS register
    if (TMC2209_ReadRegister(&driver, (TMC2209_datagram_t *)&driver.tcoolthrs)) {
        printf("TCOOLTHRS: 0x%08X\n", driver.tcoolthrs.reg.value);
    } else {
        printf("ERROR: Failed to read GCONF\n");
    }
    

    // Write SGTHRS register
    driver.sgthrs.reg.threshold = 100;
    if(TMC2209_WriteRegister(&driver, (TMC2209_datagram_t *)&driver.sgthrs)) {
        printf("SGTHRS: 0x%08X\n", driver.sgthrs.reg.threshold);
    } else {
        printf("ERROR: Failed to write SGTHRS\n");
    }
    // Read SGTHRS register
    if (TMC2209_ReadRegister(&driver, (TMC2209_datagram_t *)&driver.sgthrs)) {
        printf("SGTHRS: 0x%08X\n", driver.sgthrs.reg.threshold);
    } else {
        printf("ERROR: Failed to read GCONF\n");
    }

    // Read SG_RESULT register
    if (TMC2209_ReadRegister(&driver, (TMC2209_datagram_t *)&driver.sg_result)) {
        printf("SG_RESULT: 0x%08X\n", driver.sg_result.reg.value);
    } else {
        printf("ERROR: Failed to read GCONF\n");
    }

    driver.tpwmthrs.reg.tpwmthrs = 100;
    if(TMC2209_WriteRegister(&driver, (TMC2209_datagram_t *)&driver.tpwmthrs)) {
        printf("TPWMTHRS: 0x%08X\n", driver.tpwmthrs.reg.value);
    } else {
        printf("ERROR: Failed to write TPWMTHRS\n");
    }
    // read pwmthrs register
    if (TMC2209_ReadRegister(&driver, (TMC2209_datagram_t *)&driver.tpwmthrs)) {
        printf("TPWMTHRS: 0x%08X\n", driver.tpwmthrs.reg.value);
    } else {
        printf("ERROR: Failed to read TPWMTHRS\n");
    }

     // Read DRV_STATUS register
     if (TMC2209_ReadRegister(&driver, (TMC2209_datagram_t *)&driver.drv_status)) {
        printf("DRV_STATUS: 0x%08X\n", driver.drv_status.reg.value);
    } else {
        printf("ERROR: Failed to read GCONF\n");
    }
    if (TMC2209_ReadRegister(&driver, (TMC2209_datagram_t *)&driver.pwm_scale)) {
        printf("PWM_SCALE: 0x%08X\n", driver.pwm_scale.reg.value);
    } else {
        printf("ERROR: Failed to read GCONF\n");
    }


    printf("\nTest completed.\n");
    usleep(1000000);
    tmc_gpio_enable_driver(&g_gpio_ctx, false);
    return 0;
}