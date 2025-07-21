// File to test the UART connection

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "tmc2209.h"

int main() {
    TMC2209_t driver;
    
    printf("TMC2209 UART Communication Test\n");
    printf("===============================\n\n");
    
    // Initialize driver with defaults
    TMC2209_SetDefaults(&driver);
    
    // Configure motor settings
    driver.config.motor.address = 0;  // Default address
    driver.config.current = 500;      // 500mA
    driver.config.microsteps = 4;     // 4 microsteps
    
    printf("Initializing TMC2209...\n");
    if (!TMC2209_Init(&driver)) {
        printf("ERROR: Failed to initialize TMC2209\n");
        return -1;
    }
    printf("SUCCESS: TMC2209 initialized\n\n");
    
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
    
    printf("\nTest completed.\n");
    return 0;
}