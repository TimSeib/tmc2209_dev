/*
 * tmc2209.h - register and message (datagram) descriptors for Trinamic TMC2209 stepper driver
 *
 * v0.0.8 / 2024-11-16
 */

/*

Copyright (c) 2020-2024, Terje Io
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
specific prior written permission.

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

#ifndef _TRINAMIC2209_H_
#define _TRINAMIC2209_H_

#include <stdint.h>
#include <stdbool.h>

//#include "common.h"

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

typedef enum {
    TMC2209_Microsteps_1 = 1,
    TMC2209_Microsteps_2 = 2,
    TMC2209_Microsteps_4 = 4,
    TMC2209_Microsteps_8 = 8,
    TMC2209_Microsteps_16 = 16,
    TMC2209_Microsteps_32 = 32,
    TMC2209_Microsteps_64 = 64,
    TMC2209_Microsteps_128 = 128,
    TMC2209_Microsteps_256 = 256
} tmc2209_microsteps_t;

// ============================================================================
// TMC2209 DEFAULT CONFIGURATION VALUES
// ============================================================================
// These values are based on the TMC2209 datasheet and provide a good starting
// point for most applications. Adjust these based on your specific motor and
// application requirements.

// General Configuration
#define TMC2209_F_CLK               12000000UL              // Internal clock frequency: 12MHz (factory calibrated)
#define TMC2209_MODE                TMCMode_StealthChop     // Operating mode: StealthChop for silent operation
#define TMC2209_MICROSTEPS          TMC2209_Microsteps_4    // Microstepping: 4 microsteps per full step
#define TMC2209_R_SENSE             110                     // Sense resistor value in mOhm (external resistor)
#define TMC2209_CURRENT             500                     // Motor current in mA RMS (peak current = RMS * 1.414)
#define TMC2209_HOLD_CURRENT_PCT    50                      // Hold current as percentage of run current (0-100%)

// CHOPCONF (Chopper Configuration) Register Defaults
// These control the stepper motor chopper behavior for optimal performance
#define TMC2209_INTPOL              1   // Step interpolation: 0=off, 1=on (256 microsteps internally)
#define TMC2209_TOFF                3   // Off time: 1-15, 0=driver disable (controls chopper frequency)
#define TMC2209_TBL                 1   // Blanking time: 0=16, 1=24, 2=36, 3=54 clocks (noise filtering)
#define TMC2209_HSTRT               1   // Hysteresis start: 1-8 (current control precision)
#define TMC2209_HEND               -1   // Hysteresis end: -3 to 12 (current control precision)
#define TMC2209_HMAX               16   // Maximum hysteresis: HSTRT + HEND (must be <= 16)

#define TMC2209_IHOLDDELAY          10  // Hold delay: 0-15 (delay before switching to hold current)

// TPOWERDOWN (Power Down Time) Register
// Controls how long the driver waits before entering power down mode
#define TMC2209_TPOWERDOWN          20  // Power down delay: 0-255 * 2^18 clock cycles

// TPWMTHRS (StealthChop PWM Mode Threshold) Register
// Speed threshold for switching between StealthChop and SpreadCycle modes
#define TMC2209_TPWM_THRS           TMC_THRESHOLD_MIN   // Threshold: 0-2^20-1 (20 bits)

// PWMCONF (PWM Configuration) Register - StealthChop Mode Settings
// These control the PWM behavior in StealthChop mode for silent operation
#define TMC2209_PWM_FREQ            1   // PWM frequency: 0=1/1024, 1=2/683, 2=2/512, 3=2/410 of fCLK
#define TMC2209_PWM_AUTOGRAD        1   // Auto gradient: 0=manual, 1=automatic gradient adjustment
#define TMC2209_PWM_GRAD            14  // PWM gradient: 0-255 (controls PWM amplitude)
#define TMC2209_PWM_LIM             12  // PWM limit: 0-15 (maximum PWM amplitude)
#define TMC2209_PWM_REG             8   // PWM register: 1-15 (PWM amplitude scaling)
#define TMC2209_PWM_OFS             36  // PWM offset: 0-255 (PWM amplitude offset)

// TCOOLTHRS (CoolStep Threshold) Register
// Speed threshold for enabling CoolStep current reduction
#define TMC2209_COOLSTEP_THRS       TMC_THRESHOLD_MIN   // Threshold: 0-2^20-1 (20 bits)

// COOLCONF (CoolStep Configuration) Register - CoolStep Mode Settings
// These control the CoolStep current reduction feature for energy efficiency
#define TMC2209_SEMIN               5   // CoolStep minimum: 0=off, 1-15=minimum current reduction
#define TMC2209_SEUP                0   // CoolStep up: 0-3 (current increase steps: 1,2,4,8)
#define TMC2209_SEMAX               2   // CoolStep maximum: 0-15 (maximum current reduction)
#define TMC2209_SEDN                1   // CoolStep down: 0-3 (current decrease steps: 1,2,4,8)
#define TMC2209_SEIMIN              0   // CoolStep minimum current: 0=half current, 1=quarter current

// end of default values

#if TMC2209_MODE == 0   // StealthChop
#define TMC2209_PWM_AUTOSCALE 1
#define TMC2209_SPREADCYCLE   0
#elif TMC2209_MODE == 1 // CoolStep
#define TMC2209_PWM_AUTOSCALE 0
#define TMC2209_SPREADCYCLE   1
#else                   //StallGuard
#define TMC2209_PWM_AUTOSCALE 0
#define TMC2209_SPREADCYCLE   0
#endif

typedef uint8_t tmc2209_regaddr_t;

// ============================================================================
// TMC2209 REGISTER ADDRESSES
// ============================================================================
// These addresses correspond to the TMC2209 datasheet register map
enum tmc2209_regaddr_t {
    // System Configuration Registers (0x00-0x07)
    TMC2209Reg_GCONF        = 0x00,    // Global configuration flags
    TMC2209Reg_GSTAT        = 0x01,    // Global status flags (read-only)
    TMC2209Reg_IFCNT        = 0x02,    // Interface transmission counter
    TMC2209Reg_SLAVECONF    = 0x03,    // Slave configuration
    TMC2209Reg_OTP_PROG     = 0x04,    // OTP programming
    TMC2209Reg_OTP_READ     = 0x05,    // OTP read access
    TMC2209Reg_IOIN         = 0x06,    // Input pin states and version
    TMC2209Reg_FACTORY_CONF = 0x07,    // Factory configuration

    // Motor Control Registers (0x10-0x14, 0x22)
    TMC2209Reg_IHOLD_IRUN   = 0x10,    // Motor current settings (hold, run, delay)
    TMC2209Reg_TPOWERDOWN   = 0x11,    // Power down delay time
    TMC2209Reg_TSTEP        = 0x12,    // Actual step velocity (read-only)
    TMC2209Reg_TPWMTHRS     = 0x13,    // StealthChop PWM mode threshold
    TMC2209Reg_VACTUAL      = 0x22,    // Actual motor velocity (read-only)
    TMC2209Reg_TCOOLTHRS    = 0x14,    // CoolStep threshold
    TMC2209Reg_SGTHRS       = 0x40,    // StallGuard threshold
    TMC2209Reg_SG_RESULT    = 0x41,    // StallGuard result (read-only)
    TMC2209Reg_COOLCONF     = 0x42,    // CoolStep configuration

    // Chopper and Status Registers (0x6A-0x72)
    TMC2209Reg_MSCNT        = 0x6A,    // Microstep counter (read-only)
    TMC2209Reg_MSCURACT     = 0x6B,    // Actual microstep current (read-only)
    TMC2209Reg_CHOPCONF     = 0x6C,    // Chopper configuration
    TMC2209Reg_DRV_STATUS   = 0x6F,    // Driver status (read-only)
    TMC2209Reg_PWMCONF      = 0x70,    // PWM configuration
    TMC2209Reg_PWM_SCALE    = 0x71,    // PWM scale (read-only)
    TMC2209Reg_PWM_AUTO     = 0x72,    // PWM auto configuration (read-only)
    TMC2209Reg_LAST_ADDR    = TMC2209Reg_PWM_AUTO
};

typedef union {
    uint8_t value;
    struct {
        uint8_t
        reset_flag   :1,
        driver_error :1,
        sg2          :1,
        standstill   :1,
        unused       :4;
    };
} TMC2209_status_t;

// --- register definitions ---

// ============================================================================
// GCONF (Global Configuration) Register - Read/Write
// ============================================================================
// Controls global driver behavior and configuration options
typedef union {
    uint32_t value;
    struct {
        uint32_t
        I_scale_analog   :1,    // 0: Internal voltage reference, 1: External voltage reference
        internal_Rsense  :1,    // 0: External sense resistor, 1: Internal sense resistor
        en_spreadcycle   :1,    // 0: StealthChop mode, 1: SpreadCycle mode
        shaft            :1,    // 0: Normal direction, 1: Invert motor direction
        index_otpw       :1,    // 0: INDEX pin shows step, 1: INDEX pin shows overtemperature warning
        index_step       :1,    // 0: INDEX pin shows step, 1: INDEX pin shows microstep position
        pdn_disable      :1,    // 0: Power down enabled, 1: Power down disabled
        mstep_reg_select :1,    // 0: Microstep table from OTP, 1: Microstep table from registers
        multistep_filt   :1,    // 0: No filtering, 1: Enable step input filtering
        test_mode        :1,    // 0: Normal operation, 1: Test mode (for factory use)
        reserved         :22;   // Reserved bits (must be 0)
    };
} TMC2209_gconf_reg_t;

// ============================================================================
// GSTAT (Global Status) Register - Read + Clear on Read
// ============================================================================
// Provides global status information about the driver
typedef union {
    uint32_t value;
    struct {
        uint32_t
        reset    :1,    // 1: Driver has been reset since last read (clears on read)
        drv_err  :1,    // 1: Driver error occurred (clears on read)
        uv_cp    :1,    // 1: Undervoltage on charge pump detected (clears on read)
        reserved :29;   // Reserved bits (must be 0)
    };
} TMC2209_gstat_reg_t;

// ============================================================================
// IFCNT (Interface Transmission Counter) Register - Read Only
// ============================================================================
// Increments with each successful UART transmission for error detection
typedef union {
    uint32_t value;
    struct {
        uint32_t
        count    :8,    // Interface transmission counter (0-255, increments with each transmission)
        reserved :24;   // Reserved bits (must be 0)
    };
} TMC2209_ifcnt_reg_t;

// ============================================================================
// SLAVECONF (Slave Configuration) Register - Write Only
// ============================================================================
// Configures slave address and transmission settings
typedef union {
    uint32_t value;
    struct {
        uint32_t
        reserved0 :8,   // Reserved bits (must be 0)
        conf      :4,   // Slave configuration (0-15, typically 0)
        reserved1 :20;  // Reserved bits (must be 0)
    };
} TMC2209_slaveconf_reg_t;

// ============================================================================
// OTP_PROG (OTP Programming) Register - Write Only
// ============================================================================
// Used for programming One-Time Programmable memory (factory use only)
typedef union {
    uint32_t value;
    struct {
        uint32_t
        otpbit   :2,    // OTP bit address (0-3)
        otpbyte  :2,    // OTP byte address (0-3)
        otpmagic :28;   // OTP programming magic number (must be 0x5A5A5A5)
    };
} TMC2209_otp_prog_reg_t;

// ============================================================================
// OTP_READ (OTP Read Access) Register - Read Only
// ============================================================================
// Provides read access to One-Time Programmable memory contents
typedef union {
    uint32_t value;
    struct {
        uint32_t
        otp0_0_4 :5,    // OTP byte 0, bits 0-4
        otp0_5   :1,    // OTP byte 0, bit 5
        otp0_6   :1,    // OTP byte 0, bit 6
        otp0_7   :1,    // OTP byte 0, bit 7
        otp1_0_3 :4,    // OTP byte 1, bits 0-3
        otp1_4   :1,    // OTP byte 1, bit 4
        otp1_5_7 :3,    // OTP byte 1, bits 5-7
        otp2_0   :1,    // OTP byte 2, bit 0
        otp2_1   :1,    // OTP byte 2, bit 1
        otp2_2   :1,    // OTP byte 2, bit 2
        otp2_3_4 :2,    // OTP byte 2, bits 3-4
        otp2_5_6 :2,    // OTP byte 2, bits 5-6
        otp2_7   :1,    // OTP byte 2, bit 7
        reserved :8;    // Reserved bits (must be 0)
    };
} TMC2209_otp_read_reg_t;

// ============================================================================
// IOIN (Input Pin States and Version) Register - Read Only
// ============================================================================
// Provides the current state of input pins and chip version information
typedef union {
    uint32_t value;
    struct {
        uint32_t
        enn       :1,    // Enable pin state (0: enabled, 1: disabled)
        unused0   :1,    // Unused pin (reserved)
        ms1       :1,    // Microstep pin 1 state
        ms2       :1,    // Microstep pin 2 state
        diag      :1,    // Diagnostic pin state
        unused1   :1,    // Unused pin (reserved)
        pdn_uart  :1,    // Power down UART pin state
        step      :1,    // Step input pin state
        spread_en :1,    // Spread cycle enable pin state
        dir       :1,    // Direction input pin state
        reserved  :14,   // Reserved bits (must be 0)
        version   :8;    // Chip version (0x21 for TMC2209)
    };
} TMC2209_ioin_reg_t;

// ============================================================================
// FACTORY_CONF (Factory Configuration) Register - Read/Write
// ============================================================================
// Contains factory calibration settings (typically not modified by user)
typedef union {
    uint32_t value;
    struct {
        uint32_t
        fclktrim  :4,    // Factory clock trim (0-15, factory calibrated)
        reserved1 :3,    // Reserved bits (must be 0)
        ottrim    :2,    // Overtemperature trim (0-3, factory calibrated)
        reserved :23;    // Reserved bits (must be 0)
    };
} TMC2209_factory_conf_reg_t;

// ============================================================================
// IHOLD_IRUN (Motor Current Settings) Register - Read/Write
// ============================================================================
// Controls motor current levels for hold, run, and delay timing
typedef union {
    uint32_t value;
    struct {
        uint32_t
        ihold      :5,    // Hold current (0-31, 0=minimum, 31=maximum)
        reserved1  :3,    // Reserved bits (must be 0)
        irun       :5,    // Run current (0-31, 0=minimum, 31=maximum)
        reserved2  :3,    // Reserved bits (must be 0)
        iholddelay :4,    // Hold delay (0-15, delay before switching to hold current)
        reserved3  :12;   // Reserved bits (must be 0)
    };
} TMC2209_ihold_irun_reg_t;

// ============================================================================
// TPOWERDOWN (Power Down Time) Register - Write Only
// ============================================================================
// Controls the delay before entering power down mode
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tpowerdown :8,    // Power down delay: 0-255 * 2^18 clock cycles
        reserved   :24;   // Reserved bits (must be 0)
    };
} TMC2209_tpowerdown_reg_t;

// ============================================================================
// TSTEP (Actual Step Velocity) Register - Read Only
// ============================================================================
// Shows the current step velocity (inverse of step frequency)
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tstep    :20,    // Actual step velocity: 0-2^20-1 (lower = faster)
        reserved :12;    // Reserved bits (must be 0)
    };
} TMC2209_tstep_reg_t;

// ============================================================================
// TPWMTHRS (StealthChop PWM Mode Threshold) Register - Write Only
// ============================================================================
// Speed threshold for switching between StealthChop and SpreadCycle modes
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tpwmthrs :20,    // PWM threshold: 0-2^20-1 (lower = faster switching)
        reserved :12;    // Reserved bits (must be 0)
    };
} TMC2209_tpwmthrs_reg_t;

// ============================================================================
// TCOOLTHRS (CoolStep Threshold) Register - Write Only
// ============================================================================
// Speed threshold for enabling CoolStep current reduction
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tcoolthrs :20,   // CoolStep threshold: 0-2^20-1 (lower = faster activation)
        reserved  :12;   // Reserved bits (must be 0)
    };
} TMC2209_tcoolthrs_reg_t;

// ============================================================================
// VACTUAL (Actual Motor Velocity) Register - Write Only
// ============================================================================
// Sets the target motor velocity (used for velocity mode operation)
typedef union {
    uint32_t value;
    struct {
        uint32_t
        actual   :24,    // Target velocity: 0-2^24-1 (signed, positive = forward)
        reserved :8;     // Reserved bits (must be 0)
    };
} TMC2209_vactual_reg_t;

// ============================================================================
// SGTHRS (StallGuard Threshold) Register - Write Only
// ============================================================================
// Sets the threshold for StallGuard stall detection
typedef union {
    uint32_t value;
    struct {
        uint32_t
        threshold :8,    // StallGuard threshold: 0-255 (lower = more sensitive)
        reserved  :24;   // Reserved bits (must be 0)
    };
} TMC2209_sgthrs_reg_t;

// ============================================================================
// SG_RESULT (StallGuard Result) Register - Read Only
// ============================================================================
// Provides the current StallGuard stall detection result
typedef union {
    uint32_t value;
    struct {
        uint32_t
        result   :10,    // StallGuard result: 0-1023 (lower = closer to stall)
        reserved :22;    // Reserved bits (must be 0)
    };
} TMC2209_sg_result_reg_t;

// ============================================================================
// MSCNT (Microstep Counter) Register - Read Only
// ============================================================================
// Shows the current microstep position within a full step
typedef union {
    uint32_t value;
    struct {
        uint32_t
        mscnt    :10,    // Microstep counter: 0-1023 (position within full step)
        reserved :22;    // Reserved bits (must be 0)
    };
} TMC2209_mscnt_reg_t;

// ============================================================================
// MSCURACT (Actual Microstep Current) Register - Read Only
// ============================================================================
// Shows the actual current levels for both motor phases
typedef union {
    uint32_t value;
    struct {
        uint32_t
        cur_a     :9,    // Phase A current: 0-511 (actual current level)
        reserved1 :7,    // Reserved bits (must be 0)
        cur_b     :9,    // Phase B current: 0-511 (actual current level)
        reserved2 :7;    // Reserved bits (must be 0)
    };
} TMC2209_mscuract_reg_t;

// ============================================================================
// CHOPCONF (Chopper Configuration) Register - Read/Write
// ============================================================================
// Controls the stepper motor chopper behavior for optimal performance
typedef union {
    uint32_t value;
    struct {
        uint32_t
        toff      :4,    // Off time: 1-15, 0=driver disable (chopper frequency)
        hstrt     :3,    // Hysteresis start: 0-7 (current control precision)
        hend      :4,    // Hysteresis end: 0-15 (current control precision)
        reserved0 :4,    // Reserved bits (must be 0)
        tbl       :2,    // Blanking time: 0=16, 1=24, 2=36, 3=54 clocks
        vsense    :1,    // 0: High sensitivity, 1: Low sensitivity
        reserved1 :6,    // Reserved bits (must be 0)
        mres      :4,    // Microstep resolution: 0=256, 1=128, 2=64, etc.
        intpol    :1,    // 0: No interpolation, 1: 256 microstep interpolation
        dedge     :1,    // 0: Normal step, 1: Double edge step
        diss2g    :1,    // 0: Normal operation, 1: Disable StallGuard
        diss2vs   :1;    // 0: Normal operation, 1: Disable voltage sense
    };
} TMC2209_chopconf_reg_t;

// ============================================================================
// DRV_STATUS (Driver Status) Register - Read Only
// ============================================================================
// Provides comprehensive status information about the driver operation
typedef union {
    uint32_t value;
    struct {
        uint32_t
        otpw       :1,    // 1: Overtemperature prewarning (temperature > 120°C)
        ot         :1,    // 1: Overtemperature shutdown (temperature > 150°C)
        s2ga       :1,    // 1: Short to ground detected on phase A
        s2gb       :1,    // 1: Short to ground detected on phase B
        s2vsa      :1,    // 1: Short to supply detected on phase A
        s2vsb      :1,    // 1: Short to supply detected on phase B
        ola        :1,    // 1: Open load detected on phase A
        olb        :1,    // 1: Open load detected on phase B
        t120       :1,    // 1: Temperature > 120°C
        t143       :1,    // 1: Temperature > 143°C
        t150       :1,    // 1: Temperature > 150°C
        t157       :1,    // 1: Temperature > 157°C
        reserved1  :4,    // Reserved bits (must be 0)
        cs_actual  :5,    // Actual CoolStep current reduction (0-31)
        reserved2  :3,    // Reserved bits (must be 0)
        reserved3  :6,    // Reserved bits (must be 0)
        stealth    :1,    // 1: StealthChop mode active, 0: SpreadCycle mode
        stst       :1;    // 1: Motor standstill detected
    };
} TMC2209_drv_status_reg_t;

// ============================================================================
// COOLCONF (CoolStep Configuration) Register - Write Only
// ============================================================================
// Controls the CoolStep current reduction feature for energy efficiency
typedef union {
    uint32_t value;
    struct {
        uint32_t
        semin     :4,    // CoolStep minimum: 0=off, 1-15=minimum current reduction
        reserved1 :1,    // Reserved bit (must be 0)
        seup      :2,    // CoolStep up: 0-3 (current increase steps: 1,2,4,8)
        reserved2 :1,    // Reserved bit (must be 0)
        semax     :4,    // CoolStep maximum: 0-15 (maximum current reduction)
        reserved3 :1,    // Reserved bit (must be 0)
        sedn      :2,    // CoolStep down: 0-3 (current decrease steps: 1,2,4,8)
        seimin    :1,    // CoolStep minimum current: 0=half current, 1=quarter current
        reserved5 :16;   // Reserved bits (must be 0)
    };
} TMC2209_coolconf_reg_t;

// ============================================================================
// PWMCONF (PWM Configuration) Register - Write Only
// ============================================================================
// Controls the PWM behavior in StealthChop mode for silent operation
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_ofs       :8,    // PWM offset: 0-255 (PWM amplitude offset)
        pwm_grad      :8,    // PWM gradient: 0-255 (controls PWM amplitude)
        pwm_freq      :2,    // PWM frequency: 0=1/1024, 1=2/683, 2=2/512, 3=2/410 of fCLK
        pwm_autoscale :1,    // 0: Manual PWM scaling, 1: Automatic PWM scaling
        pwm_autograd  :1,    // 0: Manual gradient, 1: Automatic gradient adjustment
        freewheel     :2,    // Freewheeling mode: 0=normal, 1=freewheeling, 2=strong braking, 3=braking
        reserved      :2,    // Reserved bits (must be 0)
        pwm_reg       :4,    // PWM register: 1-15 (PWM amplitude scaling)
        pwm_lim       :4;    // PWM limit: 0-15 (maximum PWM amplitude)
    };
} TMC2209_pwmconf_reg_t;

// ============================================================================
// PWM_SCALE (PWM Scale) Register - Read Only
// ============================================================================
// Shows the current PWM scaling values for monitoring
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_scale_sum  :8,    // PWM scale sum: 0-255 (current PWM scaling)
        reserved1      :8,    // Reserved bits (must be 0)
        pwm_scale_auto :9,    // PWM scale auto: 0-511 (automatic PWM scaling)
        reserved2      :7;    // Reserved bits (must be 0)
    };
} TMC2209_pwm_scale_reg_t;

// ============================================================================
// PWM_AUTO (PWM Auto Configuration) Register - Read Only
// ============================================================================
// Shows the automatic PWM configuration values
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_ofs_auto  :8,    // PWM offset auto: 0-255 (automatic offset value)
        unused0       :8,    // Unused bits (must be 0)
        pwm_grad_auto :8,    // PWM gradient auto: 0-255 (automatic gradient value)
        unused1       :8;    // Unused bits (must be 0)
    };
} TMC2209_pwm_auto_ctrl_reg_t;

// --- end of register definitions ---

typedef union {
    tmc2209_regaddr_t reg;
    uint8_t value;
    struct {
        uint8_t
        idx   :7,
        write :1;
    };
} TMC2209_addr_t;

// --- datagrams ---

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_gconf_reg_t reg;
} TMC2209_gconf_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_gstat_reg_t reg;
} TMC2209_gstat_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_tpowerdown_reg_t reg;
} TMC2209_tpowerdown_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_ifcnt_reg_t reg;
} TMC2209_ifcnt_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_slaveconf_reg_t reg;
} TMC2209_slaveconf_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_otp_prog_reg_t reg;
} TMC2209_otp_prog_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_otp_read_reg_t reg;
} TMC2209_otp_read_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_ioin_reg_t reg;
} TMC2209_ioin_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_factory_conf_reg_t reg;
} TMC2209_factory_conf_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_ihold_irun_reg_t reg;
} TMC2209_ihold_irun_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_tstep_reg_t reg;
} TMC2209_tstep_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_tpwmthrs_reg_t reg;
} TMC2209_tpwmthrs_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_tcoolthrs_reg_t reg;
} TMC2209_tcoolthrs_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_vactual_reg_t reg;
} TMC2209_vactual_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_sgthrs_reg_t reg;
} TMC2209_sgthrs_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_sg_result_reg_t reg;
} TMC2209_sg_result_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_mscnt_reg_t reg;
} TMC2209_mscnt_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_mscuract_reg_t reg;
} TMC2209_mscuract_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_chopconf_reg_t reg;
} TMC2209_chopconf_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_drv_status_reg_t reg;
} TMC2209_drv_status_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_coolconf_reg_t reg;
} TMC2209_coolconf_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_pwmconf_reg_t reg;
} TMC2209_pwmconf_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_pwm_scale_reg_t reg;
} TMC2209_pwm_scale_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_pwm_auto_ctrl_reg_t reg;
} TMC2209_pwm_auto_ctrl_dgr_t;

// -- end of datagrams

typedef union {
    uint32_t value;
    uint8_t data[4];
    TMC2209_gconf_reg_t gconf;
    TMC2209_gstat_reg_t gstat;
    TMC2209_ifcnt_reg_t ifcnt;
    TMC2209_slaveconf_reg_t slaveconf;
    TMC2209_otp_prog_reg_t otp_prog;
    TMC2209_otp_read_reg_t otp_read;
    TMC2209_ioin_reg_t ioin;
    TMC2209_factory_conf_reg_t factory_conf;
    TMC2209_ihold_irun_reg_t ihold_irun;
    TMC2209_tpowerdown_reg_t tpowerdown;
    TMC2209_tstep_reg_t tstep;
    TMC2209_tpwmthrs_reg_t tpwmthrs;
    TMC2209_tcoolthrs_reg_t tcoolthrs;
    TMC2209_vactual_reg_t vactual;
    TMC2209_sgthrs_reg_t sgthrs;
    TMC2209_sg_result_reg_t sg_result;
    TMC2209_coolconf_reg_t coolconf;
    TMC2209_mscnt_reg_t mscnt;
    TMC2209_mscuract_reg_t mscuract;
    TMC2209_chopconf_reg_t chopconf;
    TMC2209_drv_status_reg_t drv_status;
    TMC2209_pwmconf_reg_t pwmconf;
    TMC2209_pwm_scale_reg_t pwm_scale;
    TMC2209_pwm_auto_ctrl_reg_t pwm_auto_ctrl;
} TMC2209_payload;

typedef struct {
     TMC2209_addr_t addr;
     TMC2209_payload payload;
} TMC2209_datagram_t;

typedef union {
    uint8_t data[8];
    struct {
        uint8_t sync;
        uint8_t slave;
        TMC2209_addr_t addr;
        TMC2209_payload payload;
        uint8_t crc;
    } msg;
} TMC2209_write_datagram_t;

typedef union {
    uint8_t data[4];
    struct {
        uint8_t sync;
        uint8_t slave;
        TMC2209_addr_t addr;
        uint8_t crc;
    } msg;
} TMC2209_read_datagram_t;

typedef struct {
    // driver registers
    TMC2209_gconf_dgr_t gconf;
    TMC2209_gstat_dgr_t gstat;
    TMC2209_ifcnt_dgr_t ifcnt;
    TMC2209_slaveconf_dgr_t slaveconf;
    TMC2209_otp_prog_dgr_t otp_prog;
    TMC2209_otp_read_dgr_t otp_read;
    TMC2209_ioin_dgr_t ioin;
    TMC2209_factory_conf_dgr_t factory_conf;
    TMC2209_ihold_irun_dgr_t ihold_irun;
    TMC2209_tpowerdown_dgr_t tpowerdown;
    TMC2209_tstep_dgr_t tstep;
    TMC2209_tpwmthrs_dgr_t tpwmthrs;
    TMC2209_tcoolthrs_dgr_t tcoolthrs;
    TMC2209_vactual_dgr_t vactual;
    TMC2209_sgthrs_dgr_t sgthrs;
    TMC2209_sg_result_dgr_t sg_result;
    TMC2209_coolconf_dgr_t coolconf;
    TMC2209_mscnt_dgr_t mscnt;
    TMC2209_mscuract_dgr_t mscuract;
    TMC2209_chopconf_dgr_t chopconf;
    TMC2209_drv_status_dgr_t drv_status;
    TMC2209_pwmconf_dgr_t pwmconf;
    TMC2209_pwm_scale_dgr_t pwm_scale;
    TMC2209_pwm_auto_ctrl_dgr_t pwm_auto;

    TMC2209_status_t driver_status;

    trinamic_config_t config;
} TMC2209_t;

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

bool TMC2209_Init(TMC2209_t *driver);
void TMC2209_SetDefaults (TMC2209_t *driver);
const trinamic_cfg_params_t *TMC2209_GetConfigDefaults (void);
void TMC2209_SetCurrent (TMC2209_t *driver, uint16_t mA, uint8_t hold_pct);
uint16_t TMC2209_GetCurrent (TMC2209_t *driver, trinamic_current_t type);
float TMC2209_GetTPWMTHRS (TMC2209_t *driver, float steps_mm);
void TMC2209_SetTPWMTHRS (TMC2209_t *driver, float mm_sec, float steps_mm);
bool TMC2209_MicrostepsIsValid (uint16_t usteps);
void TMC2209_SetMicrosteps(TMC2209_t *driver, tmc2209_microsteps_t usteps);
void TMC2209_SetTCOOLTHRS (TMC2209_t *driver, float mm_sec, float steps_mm);
void TMC2209_SetConstantOffTimeChopper(TMC2209_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator);
TMC2209_datagram_t *TMC2209_GetRegPtr (TMC2209_t *driver, tmc2209_regaddr_t reg);
bool TMC2209_WriteRegister (TMC2209_t *driver, TMC2209_datagram_t *reg);
bool TMC2209_ReadRegister (TMC2209_t *driver, TMC2209_datagram_t *reg);
float TMC2209_GetSpeedRPM(TMC2209_t *driver);

#endif
