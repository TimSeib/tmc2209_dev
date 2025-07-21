/*
 * ============================================================================
 * TMCHAL.H - HARDWARE ABSTRACTION LAYER FOR TRINAMIC DRIVERS
 * ============================================================================
 * 
 * This file defines the Hardware Abstraction Layer (HAL) interface for
 * Trinamic stepper motor drivers. It provides a unified API that abstracts
 * the differences between different Trinamic driver types (TMC2209, TMC2130,
 * TMC5160, etc.) and provides a consistent interface for motor control.
 * 
 * The HAL uses function pointers to allow different driver implementations
 * to provide their specific functionality while maintaining a common interface.
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

#include "common.h"

// ============================================================================
// SHARED REGISTER STRUCTURES
// ============================================================================

/**
 * @brief Chopper configuration structure for HAL interface
 * 
 * This structure contains the essential chopper configuration parameters
 * that are common across different Trinamic drivers. It's used by the HAL
 * to provide a unified interface for chopper configuration.
 */
typedef struct {
    uint32_t
    toff      :4,    // Off time: 1-15, 0=driver disable
    hstrt     :3,    // Hysteresis start: 0-7
    hend      :4,    // Hysteresis end: 0-15
    tbl       :2,    // Blanking time: 0=16, 1=24, 2=36, 3=54 clocks
    mres      :4;    // Microstep resolution: 0=256, 1=128, 2=64, etc.
} TMC_chopconf_t;

/**
 * @brief Motor current settings structure for HAL interface
 * 
 * This structure contains the motor current configuration parameters
 * that are common across different Trinamic drivers.
 */
typedef union {
    uint32_t value;
    struct {
        uint32_t
        ihold      :5,    // Hold current (0-31)
        irun       :5,    // Run current (0-31)
        iholddelay :4;    // Hold delay (0-15)
    };
} TMC_ihold_irun_t;

/**
 * @brief Driver status structure for HAL interface
 * 
 * This structure contains the driver status information that is common
 * across different Trinamic drivers. It provides a unified way to access
 * driver status and error conditions.
 */
typedef union {
    uint32_t value;
    struct {
        uint32_t
        sg_result  :10,    // StallGuard result (0-1023)
        s2vsa      :1,     // Short to supply on phase A
        s2vsb      :1,     // Short to supply on phase B
        stealth    :1,     // StealthChop mode active
        fsactive   :1,     // Full step active
        cs_actual  :5,     // Actual CoolStep current reduction
        stallguard :1,     // StallGuard status
        ot         :1,     // Overtemperature shutdown
        otpw       :1,     // Overtemperature prewarning
        s2ga       :1,     // Short to ground on phase A
        s2gb       :1,     // Short to ground on phase B
        ola        :1,     // Open load on phase A
        olb        :1,     // Open load on phase B
        stst       :1,     // Standstill detected
        driver_error : 1;  // Driver error
    };
} TMC_drv_status_t;

/**
 * @brief CoolStep configuration structure for HAL interface
 * 
 * This structure contains CoolStep configuration parameters that are
 * common across different Trinamic drivers.
 */
typedef struct {
    uint8_t semin;    // CoolStep minimum current reduction (0-15)
    int8_t semax;     // CoolStep maximum current reduction (0-15)
    uint8_t sedn;     // CoolStep down step size (0-3)
} TMC_coolconf_t;

/**
 * @brief Chopper timing structure for HAL interface
 * 
 * This structure contains chopper timing parameters that are
 * common across different Trinamic drivers.
 */
typedef struct {
    uint8_t hstrt;    // Hysteresis start (0-7)
    int8_t hend;      // Hysteresis end (-3 to 12)
    uint8_t tbl;      // Blanking time (0-3)
    uint8_t toff;     // Off time (1-15)
} TMC_chopper_timing_t;

// ============================================================================
// HAL FUNCTION POINTER TYPES
// ============================================================================
// These typedefs define the function signatures for the HAL interface

/**
 * @brief Function pointer type for getting driver configuration
 * @param motor Motor index (0-5)
 * @return Pointer to driver configuration structure
 */
typedef trinamic_config_t *(*tmc_get_config)(uint8_t motor);

/**
 * @brief Function pointer type for microstep validation
 * @param motor Motor index (0-5)
 * @param microsteps Microstep value to validate
 * @return true if valid, false otherwise
 */
typedef bool (*tmc_microsteps_isvalid)(uint8_t motor, uint16_t microsteps);

/**
 * @brief Function pointer type for setting microsteps
 * @param motor Motor index (0-5)
 * @param microsteps Microstep resolution to set
 */
typedef void (*tmc_set_microsteps)(uint8_t motor, uint16_t microsteps);

/**
 * @brief Function pointer type for setting motor current
 * @param motor Motor index (0-5)
 * @param mA Run current in milliamps
 * @param hold_pct Hold current as percentage of run current
 */
typedef void (*tmc_set_current)(uint8_t motor, uint16_t mA, uint8_t hold_pct);

/**
 * @brief Function pointer type for getting motor current
 * @param motor Motor index (0-5)
 * @param type Type of current to get (min, max, actual, hold)
 * @return Current value in milliamps
 */
typedef uint16_t (*tmc_get_current)(uint8_t motor, trinamic_current_t type);
/**
 * @brief Function pointer type for getting chopper configuration
 * @param motor Motor index (0-5)
 * @return Chopper configuration structure
 */
typedef TMC_chopconf_t (*tmc_get_chopconf)(uint8_t motor);

/**
 * @brief Function pointer type for getting TSTEP value
 * @param motor Motor index (0-5)
 * @return TSTEP register value (step velocity)
 */
typedef uint32_t (*tmc_get_tstep)(uint8_t motor);

/**
 * @brief Function pointer type for getting driver status
 * @param motor Motor index (0-5)
 * @return Driver status structure
 */
typedef TMC_drv_status_t (*tmc_get_drv_status)(uint8_t motor);

/**
 * @brief Function pointer type for getting raw driver status
 * @param motor Motor index (0-5)
 * @return Raw driver status register value
 */
typedef uint32_t (*tmc_get_drv_status_raw)(uint8_t motor);

/**
 * @brief Function pointer type for setting CoolStep threshold
 * @param motor Motor index (0-5)
 * @param mm_sec Velocity threshold in mm/second
 * @param steps_mm Steps per mm (mechanical resolution)
 */
typedef void (*tmc_set_tcoolthrs)(uint8_t motor, float mm_sec, float steps_mm);
/**
 * @brief Function pointer type for setting CoolStep threshold (raw value)
 * @param motor Motor index (0-5)
 * @param value Raw TCOOLTHRS register value
 */
typedef void (*tmc_set_tcoolthrs_raw)(uint8_t motor, uint32_t value);

/**
 * @brief Function pointer type for setting StealthChop threshold
 * @param motor Motor index (0-5)
 * @param mm_sec Velocity threshold in mm/second
 * @param steps_mm Steps per mm (mechanical resolution)
 */
typedef void (*tmc_set_thigh)(uint8_t motor, float mm_sec, float steps_mm);

/**
 * @brief Function pointer type for setting StealthChop threshold (raw value)
 * @param motor Motor index (0-5)
 * @param value Raw THIGH register value
 */
typedef void (*tmc_set_thigh_raw)(uint8_t motor, uint32_t value);

/**
 * @brief Function pointer type for enabling StallGuard
 * @param motor Motor index (0-5)
 * @param feed_rate Feed rate in mm/min
 * @param steps_mm Steps per mm (mechanical resolution)
 * @param sensitivity StallGuard sensitivity (-64 to 63)
 */
typedef void (*tmc_stallguard_enable)(uint8_t motor, float feed_rate, float steps_mm, int16_t sensitivity);
/**
 * @brief Function pointer type for enabling StealthChop mode
 * @param motor Motor index (0-5)
 */
typedef void (*tmc_stealthchop_enable)(uint8_t motor);

/**
 * @brief Function pointer type for getting StallGuard result
 * @param motor Motor index (0-5)
 * @return StallGuard result value (0-1023)
 */
typedef uint32_t (*tmc_get_sg_result)(uint8_t motor);

/**
 * @brief Function pointer type for enabling CoolStep mode
 * @param motor Motor index (0-5)
 */
typedef void (*tmc_coolstep_enable)(uint8_t motor);

/**
 * @brief Function pointer type for getting StealthChop threshold (raw)
 * @param motor Motor index (0-5)
 * @return Raw TPWMTHRS register value
 */
typedef uint32_t (*tmc_get_tpwmthrs_raw)(uint8_t motor);
/**
 * @brief Function pointer type for getting StealthChop threshold
 * @param motor Motor index (0-5)
 * @param steps_mm Steps per mm (mechanical resolution)
 * @return StealthChop threshold in mm/second
 */
typedef float (*tmc_get_tpwmthrs)(uint8_t motor, float steps_mm);

/**
 * @brief Function pointer type for setting StealthChop threshold
 * @param motor Motor index (0-5)
 * @param mm_sec Velocity threshold in mm/second
 * @param steps_mm Steps per mm (mechanical resolution)
 */
typedef void (*tmc_set_tpwmthrs)(uint8_t motor, float mm_sec, float steps_mm);

/**
 * @brief Function pointer type for getting global current scaler
 * @param motor Motor index (0-5)
 * @return Global current scaler value
 */
typedef uint8_t (*tmc_get_global_scaler)(uint8_t motor);
/**
 * @brief Function pointer type for getting PWM mode status
 * @param motor Motor index (0-5)
 * @return true if PWM mode is enabled, false otherwise
 */
typedef bool (*tmc_get_en_pwm_mode)(uint8_t motor);

/**
 * @brief Function pointer type for getting current settings
 * @param motor Motor index (0-5)
 * @return Current settings structure (ihold, irun, iholddelay)
 */
typedef TMC_ihold_irun_t (*tmc_get_ihold_irun)(uint8_t motor);

/**
 * @brief Function pointer type for enabling/disabling StealthChop
 * @param motor Motor index (0-5)
 * @param on true to enable StealthChop, false for CoolStep
 */
typedef void (*tmc_stealthChop)(uint8_t motor, bool on);
typedef void (*tmc_sg_filter)(uint8_t motor, bool on);
typedef void (*tmc_sg_stall_value)(uint8_t motor, int16_t val);
typedef int16_t (*tmc_get_sg_stall_value)(uint8_t motor);
typedef uint8_t (*tmc_pwm_scale)(uint8_t motor);
typedef float (*tmc_get_temp)(uint8_t motor);
typedef bool (*tmc_vsense)(uint8_t motor);
typedef void (*tmc_coolconf)(uint8_t motor, trinamic_coolconf_t coolconf);
typedef void (*tmc_chopper_timing)(uint8_t motor, trinamic_chopconf_t timing);
typedef bool (*tmc_read_register)(uint8_t motor, uint8_t addr, uint32_t *val);
typedef bool (*tmc_write_register)(uint8_t motor, uint8_t addr, uint32_t val);
typedef void *(*tmc_get_register_addr)(uint8_t motor, uint8_t addr);

typedef struct {
    const char *name;
    trinamic_driver_t driver;
    uint8_t drvconf_address; // address of driver configuration register

    tmc_get_config get_config;

    tmc_microsteps_isvalid microsteps_isvalid;
    tmc_set_microsteps set_microsteps;
    tmc_set_current set_current;
    tmc_get_current get_current;
    tmc_get_chopconf get_chopconf;
    tmc_get_drv_status get_drv_status;
    tmc_get_drv_status_raw get_drv_status_raw;
    tmc_stallguard_enable stallguard_enable;
    tmc_coolstep_enable coolstep_enable;
    tmc_get_ihold_irun get_ihold_irun;

// The following functions are dependent on driver support and may be NULL
    tmc_get_global_scaler get_global_scaler;
    tmc_get_tstep get_tstep;
    tmc_set_tcoolthrs set_tcoolthrs;
    tmc_set_thigh set_thigh;
    tmc_set_tcoolthrs_raw set_tcoolthrs_raw;
    tmc_set_thigh_raw set_thigh_raw;
    tmc_get_sg_result get_sg_result;
    tmc_stealthchop_enable stealthchop_enable;
    tmc_get_tpwmthrs_raw get_tpwmthrs_raw;
    tmc_get_tpwmthrs get_tpwmthrs;
    tmc_set_tpwmthrs set_tpwmthrs;
    tmc_get_en_pwm_mode get_en_pwm_mode;
    tmc_stealthChop stealthChop;
    tmc_pwm_scale pwm_scale;
    tmc_get_temp get_temp;
// end of dependent fuctions

    tmc_sg_filter sg_filter;
    tmc_sg_stall_value sg_stall_value;
    tmc_get_sg_stall_value get_sg_stall_value;
    tmc_vsense vsense;
    tmc_coolconf coolconf;
    tmc_chopper_timing chopper_timing;
    tmc_get_register_addr get_register_addr;
    tmc_read_register read_register;
    tmc_write_register write_register;
} tmchal_t;

static const tmchal_t tmc_null_driver = {
    .driver = TMCNULL,
    .name = "null",
};
