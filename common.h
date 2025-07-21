/*
 * ============================================================================
 * COMMON.H - SHARED CODE FOR TRINAMIC DRIVERS
 * ============================================================================
 * 
 * This file contains shared structures, constants, and functions used across
 * different Trinamic stepper motor drivers (TMC2209, TMC2130, TMC5160, etc.).
 *
 * v0.0.9 / 2025-06-08
 */

/*

Copyright (c) 2021-2025, Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission..

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// GLOBAL CONSTANTS
// ============================================================================

#define TMC_N_MOTORS_MAX    6  // Maximum number of Trinamic drivers supported
#define TMC_THRESHOLD_MIN   0  // Minimum threshold value for velocity thresholds
#define TMC_THRESHOLD_MAX   ((1<<20) - 1)  // Maximum threshold value (20-bit)

// ============================================================================
// SHARED REGISTER STRUCTURES
// ============================================================================

/**
 * @brief CoolStep configuration register structure
 * 
 * This structure is used by drivers that support CoolStep current reduction
 * (TMC2209, TMC2130, TMC5160). CoolStep automatically reduces motor current
 * when the motor is running at constant speed to save energy.
 */
typedef union {
    uint32_t value;
    struct {
        uint32_t
        cs :5,      // Current scaling (0-31)
        seup: 2,    // Current increase steps (0-3: 1,2,4,8 steps)
        sedn: 2,    // Current decrease steps (0-3: 1,2,4,8 steps)
        semax: 4,   // Maximum current reduction (0-15 levels)
        semin: 4,   // Minimum current reduction (0-15 levels)
        seimin :1;  // Minimum current level (0=half, 1=quarter)
    };
} trinamic_coolconf_t;

/**
 * @brief Chopper configuration register structure
 * 
 * This structure defines the chopper behavior for stepper motor control.
 * The chopper controls the current regulation and microstepping behavior.
 * Different drivers may use different subsets of these fields.
 */
typedef union {
    uint32_t value;
    struct {
        uint32_t
        toff            :4,    // Off time: 1-15, 0=driver disable
        hstrt           :3,    // Hysteresis start: 0-7 (current control precision)
        hend            :4,    // Hysteresis end: 0-15 (current control precision)
        hdec            :2,    // Hysteresis decrement (some drivers only)
        rndtf           :1,    // Random toff: 0=constant, 1=random
        chm             :1,    // Chopper mode: 0=constant off time, 1=spread cycle
        tbl             :2,    // Blanking time: 0=16, 1=24, 2=36, 3=54 clocks
        tfd             :4,    // Fast decay time (some drivers only)
        intpol          :1;    // Step interpolation: 0=off, 1=on
    };
} trinamic_chopconf_t;

typedef uint32_t trinamic_drvconf_t;  // Driver configuration (driver-specific)

/**
 * @brief Configuration structure for Trinamic drivers
 * 
 * This structure groups the main configuration registers that are common
 * across different Trinamic drivers.
 */
typedef struct {
    trinamic_drvconf_t drvconf;       // Driver-specific configuration
    trinamic_coolconf_t coolconf;     // CoolStep configuration
    trinamic_chopconf_t chopconf;     // Chopper configuration
} trinamic_cfg_t;

/**
 * @brief Configuration parameters structure
 * 
 * This structure contains the voltage sense levels and configuration
 * capabilities/defaults for a specific Trinamic driver.
 */
typedef struct {
     float vsense[2];         // Voltage sense levels in mV [high_sensitivity, low_sensitivity]
     trinamic_cfg_t cap;      // Maximum capabilities of the driver
     trinamic_cfg_t dflt;     // Default configuration values
} trinamic_cfg_params_t;

// ============================================================================
// ENUMERATIONS
// ============================================================================

/**
 * @brief Trinamic driver types
 * 
 * Enumeration of supported Trinamic stepper motor drivers.
 * Each driver has different capabilities and register sets.
 */
typedef enum {
    TMC2209 = 0,    // Silent stepper driver with UART interface
    TMC2130,        // SPI stepper driver with StallGuard
    TMC5160,        // High-power stepper driver with SPI
    TMC2660,        // High-power stepper driver with SPI
    TMC2240,        // High-power stepper driver with SPI
    TMCNULL         // Null driver (for testing)
} trinamic_driver_t;

/**
 * @brief Operating modes for Trinamic drivers
 * 
 * Different operating modes provide different trade-offs between
 * performance, noise, and energy efficiency.
 */
typedef enum {
   TMCMode_StealthChop = 0,  // Silent operation with PWM current control
   TMCMode_CoolStep,         // Energy-efficient operation with current reduction
   TMCMode_StallGuard,       // High-performance mode with stall detection
} trinamic_mode_t;

/**
 * @brief Current measurement types
 * 
 * Different ways to specify or measure motor current.
 */
typedef enum {
   TMCCurrent_Min = 0,       // Minimum current (0mA)
   TMCCurrent_Max,           // Maximum current (driver limit)
   TMCCurrent_Actual,        // Current run current setting
   TMCCurrent_Hold,          // Current hold current setting
//   TMCCurrent_Momentary,   // Momentary current (not implemented)
} trinamic_current_t;

#pragma pack(push, 1)

typedef struct {
    uint8_t id;         // motor id
    uint8_t axis;       // axis index
    uint8_t address;    // UART address
    uint8_t seq;        // optional motor sequence number (for chained SPI drivers)
    void *cs_pin;       // optional CS pin data for the stepper driver
} trinamic_motor_t;

typedef struct {
    uint32_t f_clk;
    uint16_t microsteps;
    uint16_t r_sense;           // mOhm
    uint16_t current;           // mA
    uint8_t hold_current_pct;   // percent
    trinamic_mode_t mode;
    trinamic_motor_t motor;
    const trinamic_cfg_params_t *cfg_params;
} trinamic_config_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t
        idx   :7,
        write :1;
    };
} TMC_addr_t;

typedef union {
    uint32_t value;
    uint8_t data[4];
} TMC_payload_t;

typedef struct {
    TMC_addr_t addr;
    TMC_payload_t payload;
} TMC_spi_datagram_t;

typedef union {
    uint32_t value;
    uint8_t data[3];
    struct {
        uint32_t
        payload :20;
    };
} TMC_spi20_datagram_t;

typedef union {
    uint8_t data[8];
    struct {
        uint8_t sync;
        uint8_t slave;
        TMC_addr_t addr;
        TMC_payload_t payload;
        uint8_t crc;
    } msg;
} TMC_uart_write_datagram_t;

typedef union {
    uint8_t data[4];
    struct {
        uint8_t sync;
        uint8_t slave;
        TMC_addr_t addr;
        uint8_t crc;
    } msg;
} TMC_uart_read_datagram_t;

// Custom registers used by I2C <> SPI bridge
typedef enum {
    TMC_I2CReg_MON_STATE = 0x7D,
    TMC_I2CReg_ENABLE = 0x7E
} TMC_i2c_registers_t;

#pragma pack(pop)

typedef uint8_t TMC_spi_status_t;

bool tmc_microsteps_validate (uint16_t microsteps);
uint8_t tmc_microsteps_to_mres (uint16_t microsteps);
uint32_t tmc_calc_tstep (trinamic_config_t *config, float mm_sec, float steps_mm);
float tmc_calc_tstep_inv (trinamic_config_t *config, uint32_t tstep, float steps_mm);
void tmc_crc8 (uint8_t *datagram, uint8_t datagramLength);
void tmc_motors_set (uint8_t motors);
uint8_t tmc_motors_get (void);

static inline void tmc_byteswap (uint8_t data[4])
{
    uint8_t tmp;

    tmp = data[0];
    data[0] = data[3];
    data[3] = tmp;
    tmp = data[1];
    data[1] = data[2];
    data[2] = tmp;
}

extern TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram);
extern TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram);

extern TMC_spi20_datagram_t tmc_spi20_write (trinamic_motor_t driver, TMC_spi20_datagram_t *datagram);
extern TMC_spi20_datagram_t tmc_spi20_read (trinamic_motor_t driver, TMC_spi20_datagram_t *datagram);

extern void tmc_uart_write (trinamic_motor_t driver, TMC_uart_write_datagram_t *datagram);
extern TMC_uart_write_datagram_t *tmc_uart_read (trinamic_motor_t driver, TMC_uart_read_datagram_t *datagram);
