# Trinamic Stepper Motor Driver Library

## Overview

This library provides a comprehensive interface for Trinamic stepper motor drivers, with particular focus on the **TMC2209** silent stepper motor driver. The library is written in plain C and is processor-agnostic, requiring only a low-level communications layer to be implemented by the user.

## Supported Drivers

- **TMC2209** - Silent stepper driver with UART interface
- **TMC2130** - SPI stepper driver with StallGuard
- **TMC5160** - High-power stepper driver with SPI
- **TMC2660** - High-power stepper driver with SPI
- **TMC2240** - High-power stepper driver with SPI

## Key Features

### TMC2209 Specific Features
- **StealthChop Mode** - Silent operation with PWM current control
- **SpreadCycle Mode** - High-performance operation with chopper control
- **CoolStep** - Automatic current reduction for energy efficiency
- **StallGuard** - Stall detection for endstop-less homing
- **UART Interface** - Simple single-wire communication
- **256 Microsteps** - Ultra-smooth motion with interpolation

### Library Features
- **Processor Agnostic** - Works with any microcontroller
- **Modular Design** - Easy to extend for new drivers
- **Comprehensive Documentation** - Detailed comments based on datasheets
- **Configuration Management** - Easy setup and parameter adjustment
- **Error Detection** - Built-in communication verification

## File Structure

```
├── tmc2209.h          # TMC2209 register definitions and API
├── tmc2209.c          # TMC2209 implementation
├── tmc2209hal.h       # Hardware abstraction layer interface
├── tmc2209hal.c       # Hardware abstraction layer implementation
├── common.h           # Shared structures and constants
├── common.c           # Shared utility functions
├── tmchal.h           # Hardware abstraction layer base
├── tmc_i2c_interface.h # I2C interface definitions
├── tmc_interface.c    # Interface implementation
└── README.md          # This file
```

## Quick Start

### 1. Basic Initialization

```c
#include "tmc2209.h"

// Create driver instance
TMC2209_t driver;

// Set default configuration
TMC2209_SetDefaults(&driver);

// Configure motor parameters
driver.config.motor.address = 0;        // UART address
driver.config.current = 500;            // 500mA RMS
driver.config.microsteps = 16;          // 16 microsteps
driver.config.r_sense = 110;            // 110mOhm sense resistor

// Initialize the driver
if (!TMC2209_Init(&driver)) {
    // Handle initialization error
}
```

### 2. Current Control

```c
// Set motor current
TMC2209_SetCurrent(&driver, 750, 50);  // 750mA run, 50% hold

// Get current values
uint16_t run_current = TMC2209_GetCurrent(&driver, TMCCurrent_Actual);
uint16_t hold_current = TMC2209_GetCurrent(&driver, TMCCurrent_Hold);
```

### 3. Microstepping

```c
// Set microstep resolution
TMC2209_SetMicrosteps(&driver, TMC2209_Microsteps_256);

// Validate microstep setting
if (TMC2209_MicrostepsIsValid(128)) {
    // Valid microstep value
}
```

### 4. Threshold Configuration

```c
// Set StealthChop threshold (speed for switching modes)
TMC2209_SetTPWMTHRS(&driver, 100.0f, 80.0f);  // 100mm/s, 80 steps/mm

// Set CoolStep threshold
TMC2209_SetTCOOLTHRS(&driver, 50.0f, 80.0f);  // 50mm/s, 80 steps/mm
```

## Configuration Options

### Operating Modes
- **StealthChop** - Silent operation, best for low-speed applications
- **SpreadCycle** - High performance, best for high-speed applications
- **CoolStep** - Energy efficient, automatically reduces current

### Current Settings
- **Run Current** - Current during motion (0-2000mA typical)
- **Hold Current** - Current when stationary (0-100% of run current)
- **Hold Delay** - Time before switching to hold current

### Chopper Configuration
- **Off Time** - Chopper frequency (1-15, higher = lower frequency)
- **Hysteresis** - Current control precision
- **Blanking Time** - Noise filtering (16-54 clock cycles)

## Hardware Requirements

### TMC2209 Pin Connections
- **UART_TX** - Single-wire UART transmit
- **UART_RX** - Single-wire UART receive (optional)
- **ENN** - Enable pin (active low)
- **STEP** - Step input
- **DIR** - Direction input
- **VCC** - 5V supply
- **VM** - Motor supply voltage
- **GND** - Ground

### Sense Resistor
- **External** - 110mOhm recommended (default)
- **Internal** - 60mOhm (requires configuration change)

## Communication Interface

The library uses a UART-based communication protocol with the following characteristics:
- **Baud Rate** - 115200 (default)
- **Data Format** - 8N1
- **Protocol** - Single-wire UART with CRC8
- **Addressing** - Up to 256 devices on one bus

## Integration Examples

### grblHAL Integration
This library is used by [grblHAL](https://github.com/grblHAL) drivers. Examples of:
- Low-level communications layers
- Higher-level configuration/reporting layer
- Complete integration examples

### I2C Bridge
For systems with limited I/O capabilities, a [SPI <> I2C bridge](https://github.com/terjeio/Trinamic_TMC2130_I2C_SPI_Bridge) is available, implemented on a TI MSP430G2553 processor.

## API Reference

### Core Functions
- `TMC2209_Init()` - Initialize driver
- `TMC2209_SetDefaults()` - Set default configuration
- `TMC2209_SetCurrent()` - Configure motor current
- `TMC2209_GetCurrent()` - Read current settings
- `TMC2209_SetMicrosteps()` - Set microstep resolution
- `TMC2209_SetTPWMTHRS()` - Set StealthChop threshold
- `TMC2209_SetTCOOLTHRS()` - Set CoolStep threshold

### Register Access
- `TMC2209_WriteRegister()` - Write to driver register
- `TMC2209_ReadRegister()` - Read from driver register
- `TMC2209_GetRegPtr()` - Get pointer to shadow register

## Troubleshooting

### Common Issues
1. **No Communication** - Check UART configuration and wiring
2. **High Current** - Verify sense resistor value and current settings
3. **Motor Noise** - Adjust chopper settings or switch to StealthChop mode
4. **Stall Detection** - Configure StallGuard threshold properly

### Debug Features
- Interface counter verification
- CRC error detection
- Status register monitoring
- Temperature monitoring

## License

This library is provided under the BSD license. See the individual source files for license details.

## Contributing

Contributions are welcome! Please ensure:
- Code follows the existing style
- New features are properly documented
- Tests are included for new functionality

## Version History

- **v0.0.9** - Enhanced documentation, improved error handling
- **v0.0.8** - Added comprehensive register documentation
- **v0.0.7** - Improved current calculation accuracy
- **v0.0.6** - Added CoolStep configuration support
- **v0.0.5** - Initial TMC2209 support

---

*Last updated: 2024-11-17*
