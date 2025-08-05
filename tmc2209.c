/*
 * tmc2209.c - interface for Trinamic TMC2209 stepper driver
 *
 * v0.0.9 / 2024-11-17
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

/*
 * ============================================================================
 * TMC2209 IMPLEMENTATION
 * ============================================================================
 * 
 * This file implements the TMC2209 stepper motor driver interface.
 * The TMC2209 is a silent stepper motor driver with advanced features:
 * - StealthChop mode for silent operation
 * - SpreadCycle mode for high performance
 * - CoolStep current reduction for energy efficiency
 * - StallGuard stall detection
 * - UART interface for configuration
 * 
 * Reference for calculations:
 * https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC5130_TMC2209_TMC2100_Calculations.xlsx
 *
 */
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "tmc2209.h"

// ============================================================================
// TMC2209 CONFIGURATION PARAMETERS
// ============================================================================
// These parameters define the capabilities and default settings for the TMC2209
static const trinamic_cfg_params_t cfg_params = {
    // Voltage sense levels for current calculation (in mV)
    .vsense[0] = 325.0f,    // High sensitivity mode (vsense=0)
    .vsense[1] = 180.0f,    // Low sensitivity mode (vsense=1)

    .cap.drvconf = 0,       // Driver configuration capabilities

    // CoolStep configuration capabilities (maximum values)
    .cap.coolconf.seup = 0b11,      // Maximum current increase steps (8x)
    .cap.coolconf.sedn = 0b11,      // Maximum current decrease steps (8x)
    .cap.coolconf.semax = 0b1111,   // Maximum current reduction (15 levels)
    .cap.coolconf.semin = 0b1111,   // Maximum minimum current reduction (15 levels)
    .cap.coolconf.seimin = 1,       // Minimum current can be quarter

    // Chopper configuration capabilities (maximum values)
    .cap.chopconf.toff = 0b1111,    // Maximum off time (15)
    .cap.chopconf.hstrt = 0b111,    // Maximum hysteresis start (7)
    .cap.chopconf.hend = 0b1111,    // Maximum hysteresis end (15)
    .cap.chopconf.rndtf = 1,        // Random toff enabled
    .cap.chopconf.intpol = 1,       // Interpolation enabled
    .cap.chopconf.tbl = 0b11,       // Maximum blanking time (54 clocks)

    .dflt.drvconf = 0,      // Default driver configuration

    // Default CoolStep settings
    .dflt.coolconf.seup = TMC2209_SEUP,
    .dflt.coolconf.sedn = TMC2209_SEDN,
    .dflt.coolconf.semax = TMC2209_SEMAX,
    .dflt.coolconf.semin = TMC2209_SEMIN,
    .dflt.coolconf.seimin = TMC2209_SEIMIN,

    // Default chopper settings
    .dflt.chopconf.toff = TMC2209_TOFF,
    .dflt.chopconf.hstrt = TMC2209_HSTRT - 1,
    .dflt.chopconf.hend = TMC2209_HEND + 3,
    .dflt.chopconf.intpol = TMC2209_INTPOL,
    .dflt.chopconf.tbl = TMC2209_TBL
};

// ============================================================================
// TMC2209 DEFAULT CONFIGURATION
// ============================================================================
// This structure contains all the default register values for the TMC2209
static const TMC2209_t tmc2209_defaults = {
    // Basic configuration
    .config.f_clk = TMC2209_F_CLK,              // 12MHz internal clock
    .config.mode = TMC2209_MODE,                // StealthChop mode
    .config.r_sense = TMC2209_R_SENSE,          // 110mOhm sense resistor
    .config.current = TMC2209_CURRENT,          // 500mA RMS current
    .config.hold_current_pct = TMC2209_HOLD_CURRENT_PCT,  // 50% hold current
    .config.microsteps = TMC2209_MICROSTEPS,    // 4 microsteps

    // Register addresses and default values
    .gconf.addr.reg = TMC2209Reg_GCONF,
    .gconf.reg.en_spreadcycle = TMC2209_SPREADCYCLE,  // StealthChop mode
    .gstat.addr.reg = TMC2209Reg_GSTAT,
    .ifcnt.addr.reg = TMC2209Reg_IFCNT,
    .slaveconf.addr.reg = TMC2209Reg_SLAVECONF,
    .otp_prog.addr.reg = TMC2209Reg_OTP_PROG,
    .otp_read.addr.reg = TMC2209Reg_OTP_READ,
    .ioin.addr.reg = TMC2209Reg_IOIN,
    .factory_conf.addr.reg = TMC2209Reg_FACTORY_CONF,
    
    // Motor current settings
    .ihold_irun.addr.reg = TMC2209Reg_IHOLD_IRUN,
    .ihold_irun.reg.iholddelay = TMC2209_IHOLDDELAY,  // Hold delay timing
    
    // Power management
    .tpowerdown.addr.reg = TMC2209Reg_TPOWERDOWN,
    .tpowerdown.reg.tpowerdown = TMC2209_TPOWERDOWN,  // Power down delay
    
    // Velocity and threshold registers
    .tstep.addr.reg = TMC2209Reg_TSTEP,
    .tpwmthrs.addr.reg = TMC2209Reg_TPWMTHRS,    // StealthChop threshold
    .vactual.addr.reg = TMC2209Reg_VACTUAL,
    .tcoolthrs.addr.reg = TMC2209Reg_TCOOLTHRS,  // CoolStep threshold
    .tcoolthrs.reg.tcoolthrs = TMC2209_COOLSTEP_THRS,
    
    // StallGuard settings
    .sgthrs.addr.reg = TMC2209Reg_SGTHRS,
    .sg_result.addr.reg = TMC2209Reg_SG_RESULT,
    
    // CoolStep configuration
    .coolconf.addr.reg = TMC2209Reg_COOLCONF,
    .coolconf.reg.semin = TMC2209_SEMIN,         // Minimum current reduction
    .coolconf.reg.seup = TMC2209_SEUP,           // Current increase steps
    .coolconf.reg.semax = TMC2209_SEMAX,         // Maximum current reduction
    .coolconf.reg.sedn = TMC2209_SEDN,           // Current decrease steps
    .coolconf.reg.seimin = TMC2209_SEIMIN,       // Minimum current level
    
    // Microstep and current monitoring
    .mscnt.addr.reg = TMC2209Reg_MSCNT,
    .mscuract.addr.reg = TMC2209Reg_MSCURACT,
    
    // Chopper configuration
    .chopconf.addr.reg = TMC2209Reg_CHOPCONF,
    .chopconf.reg.tbl = TMC2209_TBL,             // Blanking time: 24 clocks
    .chopconf.reg.toff = TMC2209_TOFF,           // Off time: 3 (chopper frequency)
    .chopconf.reg.hstrt = TMC2209_HSTRT - 1,     // Hysteresis start: 0 (1-8 range)
    .chopconf.reg.hend = TMC2209_HEND + 3,       // Hysteresis end: 2 (-3 to 12 range)
    .chopconf.reg.intpol = TMC2209_INTPOL,       // Step interpolation enabled
    
    // Status and PWM configuration
    .drv_status.addr.reg = TMC2209Reg_DRV_STATUS,
    .pwmconf.addr.reg = TMC2209Reg_PWMCONF,
    .pwmconf.reg.pwm_lim = TMC2209_PWM_LIM,      // PWM amplitude limit
    .pwmconf.reg.pwm_reg = TMC2209_PWM_REG,      // PWM amplitude scaling
    .pwmconf.reg.pwm_autograd = TMC2209_PWM_AUTOGRAD,  // Auto gradient enabled
    .pwmconf.reg.pwm_freq = TMC2209_PWM_FREQ,    // PWM frequency: 2/683 of fCLK
    .pwmconf.reg.pwm_grad = TMC2209_PWM_GRAD,    // PWM gradient: 14
    .pwmconf.reg.pwm_ofs = TMC2209_PWM_OFS,      // PWM offset: 36
    .pwmconf.reg.pwm_autoscale = TMC2209_PWM_AUTOSCALE,  // Auto scaling enabled
    .pwm_scale.addr.reg = TMC2209Reg_PWM_SCALE,
    .pwm_auto.addr.reg = TMC2209Reg_PWM_AUTO
};

// ============================================================================
// INTERNAL FUNCTIONS
// ============================================================================

/**
 * @brief Calculate and set the RMS current values for the motor
 * @param driver Pointer to the TMC2209 driver structure
 * 
 * This function calculates the appropriate current scaling values based on:
 * - The configured RMS current (mA)
 * - The sense resistor value (mOhm)
 * - The voltage sense sensitivity
 * 
 * The calculation follows the TMC2209 datasheet formula:
 * Current = (CS + 1) / 32 * V_SENSE / (R_SENSE + 20mOhm) / sqrt(2) * 1000
 */
static void _set_rms_current (TMC2209_t *driver)
{
    // Calculate maximum voltage across sense resistor
    // V = I * R * sqrt(2) / 1000 (convert mA to A and mOhm to Ohm)
    float maxv = (((float)(driver->config.r_sense + 20)) * (float)(32UL * driver->config.current)) * 1.41421f / 1000.0f;

    // Calculate current scaling for high sensitivity mode (vsense=0)
    int8_t current_scaling = (int8_t)(maxv / cfg_params.vsense[0]) - 1;

    // If current scaling is too low (< 16), switch to low sensitivity mode (vsense=1)
    // and recalculate with the lower voltage sense level
    if ((driver->chopconf.reg.vsense = (current_scaling < 16)))
        current_scaling = (uint8_t)(maxv / cfg_params.vsense[1]) - 1;

    // Set run current (clamp to maximum of 31)
    driver->ihold_irun.reg.irun = current_scaling > 31 ? 31 : current_scaling;
    
    // Set hold current as percentage of run current
    driver->ihold_irun.reg.ihold = (driver->ihold_irun.reg.irun * driver->config.hold_current_pct) / 100;

    // TODO: Set CoolStep minimum current based on run current
    // driver->coolconf.reg.seimin = driver->ihold_irun.reg.irun >= 20;
}

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

/**
 * @brief Get the default configuration parameters for TMC2209
 * @return Pointer to the configuration parameters structure
 */
const trinamic_cfg_params_t *TMC2209_GetConfigDefaults (void)
{
    return &cfg_params;
}

/**
 * @brief Set default configuration values for the TMC2209 driver
 * @param driver Pointer to the TMC2209 driver structure
 * 
 * This function initializes the driver with all default values including:
 * - Register addresses
 * - Default register values
 * - Current calculations
 * - Microstep resolution
 */
void TMC2209_SetDefaults (TMC2209_t *driver)
{
    // Copy all default values from the template
    memcpy(driver, &tmc2209_defaults, sizeof(TMC2209_t));

    // Calculate and set the current values based on configuration
    //Internal function
    _set_rms_current(driver);

    // Set the microstep resolution based on the configured microsteps
    driver->chopconf.reg.mres = tmc_microsteps_to_mres(driver->config.microsteps);
}

/**
 * @brief Initialize the TMC2209 driver
 * @param driver Pointer to the TMC2209 driver structure
 * @return true if initialization successful, false otherwise
 * 
 * This function performs the complete initialization sequence:
 * 1. Clears status flags by reading/writing GSTAT register
 * 2. Configures global settings (GCONF register)
 * 3. Sets up chopper configuration
 * 4. Writes all configuration registers to the driver
 * 5. Verifies communication by checking interface counter
 */
bool TMC2209_Init (TMC2209_t *driver)
{
    // Step 1: Clear status flags by reading GSTAT register
    // If no response from driver, return error
    if(!TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->gstat)){
        return false;
    }
        

    // Write GSTAT to clear any pending flags
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->gstat);

    // Step 2: Read and configure global settings
    TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->gconf);
    driver->gconf.reg.pdn_disable = 1;        // Disable power down mode
    driver->gconf.reg.mstep_reg_select = 1;   // Use register microstep table
    driver->gconf.reg.I_scale_analog = 0;     // Use digital current scaling
    driver->gconf.reg.index_step = 1;         // Index step enabled
    driver->gconf.reg.index_otpw = 0;        // Index output power down disabled
    driver->tcoolthrs.reg.tcoolthrs = 1500; // set above TSTEP to enable stallguard
    driver->sgthrs.reg.threshold = 3;       // 2*SGTHRS >= SG_RESULT means stall

    // Note: These settings use factory defaults from OTP memory
    // driver->gconf.reg.internal_Rsense = 0;  // External sense resistor
    // driver->gconf.reg.en_spreadcycle = 0;   // StealthChop mode
    // driver->gconf.reg.multistep_filt = 1;   // Enable step filtering

    // Step 3: Read interface counter for verification
    TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->ifcnt);
    uint8_t ifcnt = driver->ifcnt.reg.count;

    // Step 4: Set microstep resolution
    driver->chopconf.reg.mres = tmc_microsteps_to_mres(driver->config.microsteps);

    // Step 5: Write all configuration registers to the driver
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->gconf);
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->tpowerdown);
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->pwmconf);
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->tpwmthrs);
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->tcoolthrs);
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->sgthrs);
    TMC2209_SetCurrent(driver, driver->config.current, driver->config.hold_current_pct);

    // Step 6: Verify communication by checking interface counter
    // Should have incremented by 8 (the number of write operations)
    TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->ifcnt);
    return (((uint8_t)driver->ifcnt.reg.count - ifcnt) & 0xFF) == 8;
}

uint16_t TMC2209_GetCurrent (TMC2209_t *driver, trinamic_current_t type)
{
    uint8_t cs;
    bool vsense;

    switch(type) {
        case TMCCurrent_Max:
            cs = 31;
            vsense = 0;
            break;
        case TMCCurrent_Actual:
            cs = driver->ihold_irun.reg.irun;
            vsense = driver->chopconf.reg.vsense;
            break;
        case TMCCurrent_Hold:
            cs = driver->ihold_irun.reg.ihold;
            vsense = driver->chopconf.reg.vsense;
            break;
        default: // TMCCurrent_Min:
            cs = 0;
            vsense = 1;
            break;
    }

    return (uint16_t)ceilf((float)(cs + 1) / 32.0f * cfg_params.vsense[vsense] / (float)(driver->config.r_sense + 20) / 1.41421f * 1000.0f);
}

// r_sense = mOhm, Vsense = mV, current = mA (RMS)
void TMC2209_SetCurrent (TMC2209_t *driver, uint16_t mA, uint8_t hold_pct)
{
    driver->config.current = mA;
    driver->config.hold_current_pct = hold_pct;

    //Internal function
    _set_rms_current(driver);

    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->chopconf);
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->ihold_irun);
}

float TMC2209_GetTPWMTHRS (TMC2209_t *driver, float steps_mm)
{
    return tmc_calc_tstep_inv(&driver->config, driver->tpwmthrs.reg.tpwmthrs, steps_mm);
}

void TMC2209_SetTPWMTHRS (TMC2209_t *driver, float mm_sec, float steps_mm)
{
    driver->tpwmthrs.reg.tpwmthrs = tmc_calc_tstep(&driver->config, mm_sec, steps_mm);
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->tpwmthrs);
}

/**
 * @brief Get motor speed in RPM from TSTEP register
 * 
 * @param driver TMC2209 driver structure
 * @return Motor speed in RPM
 */
 float TMC2209_GetSpeedRPM(TMC2209_t *driver) {
    if (!driver) {
        return 0.0f;
    }
    
    if (TMC2209_ReadRegister(driver, (TMC2209_datagram_t *)&driver->tstep)) {
        uint32_t tstep = driver->tstep.reg.tstep;
        
        if (tstep == 0) {
            return 0.0f;
        }
        
        // RPM = (f_clk * 60) / (tstep * 200 * microsteps)
        return (float)(driver->config.f_clk * 60) / 
               (float)(tstep * 200 * driver->config.microsteps);
    }
    
    return 0.0f;
}

void TMC2209_SetTCOOLTHRS (TMC2209_t *driver, float mm_sec, float steps_mm) // -> pwm threshold
{
    driver->tcoolthrs.reg.tcoolthrs = tmc_calc_tstep(&driver->config, mm_sec, steps_mm);
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->tcoolthrs);
}

// 1 - 256 in steps of 2^value is valid for TMC2209
bool TMC2209_MicrostepsIsValid (uint16_t usteps)
{
    return tmc_microsteps_validate(usteps);
}

void TMC2209_SetMicrosteps (TMC2209_t *driver, tmc2209_microsteps_t msteps)
{
    driver->chopconf.reg.mres = tmc_microsteps_to_mres(msteps);
    driver->config.microsteps = (tmc2209_microsteps_t)(1 << (8 - driver->chopconf.reg.mres));
// TODO: recalc and set hybrid threshold if enabled?
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->chopconf);
}

void TMC2209_SetConstantOffTimeChopper (TMC2209_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator)
{
    // Suppress unused parameter warning
    (void)use_current_comparator;

    //calculate the value acc to the clock cycles
    if (blank_time >= 54)
        blank_time = 3;
    else if (blank_time >= 36)
        blank_time = 2;
    else if (blank_time >= 24)
        blank_time = 1;
    else
        blank_time = 0;

    if (fast_decay_time > 15)
        fast_decay_time = 15;

    driver->chopconf.reg.tbl = blank_time;
    driver->chopconf.reg.toff = constant_off_time < 2 ? 2 : (constant_off_time > 15 ? 15 : constant_off_time);
    driver->chopconf.reg.hstrt = fast_decay_time & 0x7;
    driver->chopconf.reg.hend = (sine_wave_offset < -3 ? -3 : (sine_wave_offset > 12 ? 12 : sine_wave_offset)) + 3;
//!    driver->chopconf.reg.rndtf = !use_current_comparator;

    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->chopconf);
}

bool TMC2209_WriteRegister (TMC2209_t *driver, TMC2209_datagram_t *reg)
{
    TMC_uart_write_datagram_t datagram;

    datagram.msg.sync = 0x05;
    datagram.msg.slave = driver->config.motor.address;
    datagram.msg.addr.value = reg->addr.value;
    datagram.msg.addr.write = 1;
    datagram.msg.payload.value = reg->payload.value;

    tmc_byteswap(datagram.msg.payload.data);

    tmc_crc8(datagram.data, sizeof(TMC_uart_write_datagram_t));

    tmc_uart_write(driver->config.motor, &datagram);

// TODO: add check for ok'ed?

    return true;
}

bool TMC2209_ReadRegister (TMC2209_t *driver, TMC2209_datagram_t *reg)
{
    bool ok = false;
    TMC_uart_read_datagram_t datagram;
    TMC_uart_write_datagram_t *res;

    datagram.msg.sync = 0x05;
    datagram.msg.slave = driver->config.motor.address;
    datagram.msg.addr.value = reg->addr.value;
    datagram.msg.addr.write = 0;
    tmc_crc8(datagram.data, sizeof(TMC_uart_read_datagram_t));

    res = tmc_uart_read(driver->config.motor, &datagram);

    if(res->msg.slave == 0xFF && res->msg.addr.value == datagram.msg.addr.value) {
        uint8_t crc = res->msg.crc;
        tmc_crc8(res->data, sizeof(TMC_uart_write_datagram_t));
        //printf("TMC2209_ReadRegister_crc: %d\n", crc);
        //printf("TMC2209_ReadRegister_crc_res: %d\n", res->msg.crc);
        if((ok = crc == res->msg.crc)) {
            reg->payload.value = res->msg.payload.value;
            tmc_byteswap(reg->payload.data);
        }
    }
    
    //printf("TMC2209_ReadRegister_pass_y/n: %d\n", ok);
    return ok;
}

// Returns pointer to shadow register or NULL if not found
TMC2209_datagram_t *TMC2209_GetRegPtr (TMC2209_t *driver, tmc2209_regaddr_t reg)
{
    TMC2209_datagram_t *ptr = (TMC2209_datagram_t *)driver;

    while(ptr && ptr->addr.reg != reg) {
        ptr++;
        if(ptr->addr.reg == TMC2209Reg_LAST_ADDR && ptr->addr.reg != reg)
            ptr = NULL;
    }

    return ptr;
}

