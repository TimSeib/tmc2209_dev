# TMC2209 StallGuard Implementation

This document explains how to use the StallGuard functionality for stall detection and homing in your TMC2209 stepper motor driver project.

## Overview

StallGuard is a feature of the TMC2209 that can detect when a motor stalls (hits a mechanical limit or obstruction) by monitoring the motor load. This is useful for:

- **Homing**: Automatically finding mechanical end stops
- **Stall Detection**: Detecting when the motor hits an obstacle
- **Load Monitoring**: Monitoring motor load during operation
- **Safety**: Stopping motion when stalls are detected

## Features

- **Automatic Stall Detection**: Detects stalls using the DIAG pin
- **Configurable Thresholds**: Adjustable sensitivity for different applications
- **CoolStep Integration**: Automatic current adjustment based on load
- **Homing Support**: Automatic homing to mechanical limits
- **Real-time Monitoring**: Continuous monitoring during motion
- **Callback System**: User-defined callbacks for stall events

## Hardware Requirements

- TMC2209 stepper motor driver
- Raspberry Pi or compatible board
- GPIO connection to TMC2209 DIAG pin
- Motor with mechanical limits or load sensing capability

## Installation

1. **Install Dependencies**:
   ```bash
   sudo apt-get update
   sudo apt-get install -y libgpiod-dev
   ```

2. **Compile the Test Program**:
   ```bash
   make -f Makefile.stallguard
   ```

3. **Run the Test Program**:
   ```bash
   ./test_stallguard
   ```

## Usage

### Basic Setup

```c
#include "tmc_stallguard.h"

// Initialize StallGuard
tmc_stallguard_config_t config = {
    .diag_pin = TMC_GPIO_DIAG_PIN,  // GPIO pin connected to DIAG
    .threshold = 50,                 // StallGuard threshold (0-255)
    .min_speed_threshold = 100,      // Minimum speed for StallGuard
    .callback = my_stall_callback,   // Callback function
    .enabled = true
};

if (tmc_stallguard_init(&driver, &config)) {
    printf("StallGuard initialized successfully\n");
}
```

### StallGuard Callback

```c
void my_stall_callback(void) {
    printf("Stall detected!\n");
    // Stop motion, save position, etc.
}
```

### Threshold Tuning

The StallGuard threshold determines sensitivity:

- **Lower values (10-30)**: More sensitive, triggers on lighter loads
- **Higher values (70-100)**: Less sensitive, only triggers on heavy loads
- **Typical range (40-60)**: Good for most applications

Use the test program to find the optimal threshold for your setup:

```bash
./test_stallguard
# Select option 1 for threshold tuning
```

### CoolStep Configuration

CoolStep automatically adjusts motor current based on load:

```c
tmc_coolstep_config_t coolstep = {
    .semin = 4,    // Lower threshold for current increase
    .semax = 8,    // Upper threshold for current decrease
    .seup = 1,     // Current increment step
    .sedn = 3,     // Decrement rate
    .seimin = true // Scale down to 1/4 of IRun
};

tmc_stallguard_enable_coolstep(&driver, &coolstep);
```

### Homing with StallGuard

```c
// Home to mechanical limit
bool success = tmc_stallguard_do_homing(
    &driver,           // TMC2209 driver
    TMC_GPIO_DIAG_PIN, // DIAG pin
    5,                 // Max revolutions
    50,                // Threshold
    30.0f              // Speed in RPM
);

if (success) {
    printf("Homing completed successfully\n");
}
```

### Real-time Monitoring

```c
// Enable StallGuard monitoring
tmc_stallguard_enable(&driver, true);

// During motion, check for stalls
while (motion_active) {
    if (tmc_stallguard_check_triggered(&driver, &gpio_ctx)) {
        printf("Stall detected during motion\n");
        stop_motion();
        break;
    }
    
    // Get current StallGuard result
    uint32_t sg_result = tmc_stallguard_get_result(&driver);
    printf("Current load: %u\n", sg_result);
    
    usleep(100000); // 100ms
}
```

## Integration with Your Project

### In move_lightbar.c

The StallGuard functionality is already integrated into your `move_lightbar.c`:

1. **Initialization**: StallGuard is initialized during startup
2. **Monitoring**: StallGuard is checked during motion
3. **Recovery**: When stalls are detected, the system saves position and stops

### Key Functions

- `tmc_stallguard_init()`: Initialize StallGuard
- `tmc_stallguard_set_callback()`: Set stall detection callback
- `tmc_stallguard_enable()`: Enable/disable StallGuard
- `tmc_stallguard_get_result()`: Get current StallGuard result
- `tmc_stallguard_check_triggered()`: Check if stall is detected
- `tmc_stallguard_do_homing()`: Perform homing operation

## Configuration

### Environment Variables

You can configure StallGuard behavior using environment variables:

```bash
export STALLGUARD_THRESHOLD=50
export STALLGUARD_MIN_SPEED=100
export STALLGUARD_ENABLED=1
```

### Register Settings

StallGuard uses these TMC2209 registers:

- **SGTHRS (0x40)**: StallGuard threshold
- **SGRESULT (0x41)**: Current StallGuard result (read-only)
- **TCOOLTHRS (0x14)**: Minimum speed threshold
- **COOLCONF (0x42)**: CoolStep configuration
- **CHOPCONF (0x6C)**: StallGuard enable/disable

## Troubleshooting

### StallGuard Not Triggering

1. **Check DIAG Pin**: Ensure DIAG pin is properly connected
2. **Verify StealthChop**: StallGuard only works in StealthChop mode
3. **Adjust Threshold**: Try lower threshold values
4. **Check Speed**: Ensure motor speed is above minimum threshold

### False Triggers

1. **Increase Threshold**: Use higher threshold values
2. **Check Wiring**: Ensure no electrical interference
3. **Verify Load**: Ensure motor has sufficient load to detect

### Performance Issues

1. **Optimize Polling**: Reduce polling frequency if needed
2. **Use Callbacks**: Use interrupt-based detection instead of polling
3. **Adjust CoolStep**: Fine-tune CoolStep parameters

## Testing

Use the provided test program to verify StallGuard functionality:

```bash
# Build and run tests
make -f Makefile.stallguard
./test_stallguard

# Available test modes:
# 1. Threshold tuning - Find optimal threshold
# 2. Homing test - Test homing functionality
# 3. Real-time monitoring - Monitor StallGuard results
```

## Safety Considerations

- **Mechanical Limits**: Ensure motor can safely hit mechanical stops
- **Current Limits**: Monitor motor current to prevent damage
- **Emergency Stop**: Always have emergency stop capability
- **Position Recovery**: Save position when stalls are detected

## Examples

See the following files for complete examples:

- `test_stallguard.c`: Complete test program
- `move_lightbar.c`: Integration with your lightbar project
- `tmc_stallguard.c`: Full implementation

## Support

For issues or questions:

1. Check the troubleshooting section above
2. Review the TMC2209 datasheet for StallGuard details
3. Test with the provided test program
4. Verify hardware connections and configuration 