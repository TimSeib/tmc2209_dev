/*
 * ============================================================================
 * TMC2209HAL.C - TMC2209 DRIVER HAL IMPLEMENTATION
 * ============================================================================
 *
 * This file implements the Hardware Abstraction Layer (HAL) for the TMC2209
 * stepper motor driver. It provides the glue logic between the generic HAL
 * interface (tmchal_t) and the TMC2209-specific register operations and
 * configuration routines. All functions here operate on the TMC2209_t driver
 * structure and translate generic HAL calls into TMC2209 register accesses.
 *
 * Each function is documented with its purpose, parameters, and relation to
 * the TMC2209 datasheet. This makes it easier for developers to understand
 * how to extend or adapt the driver for their own hardware.
 * ============================================================================
 */

/*

Copyright (c) 2021-2024, Terje Io
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

#include <stdlib.h>
#include <string.h>

#include "grbl/hal.h"

#include "tmc2209.h"
#include "tmchal.h"

static TMC2209_t *tmcdriver[6];

/**
 * @brief Get pointer to the configuration structure for a motor
 * @param motor Motor index (0-5)
 * @return Pointer to trinamic_config_t for the specified motor
 *
 * This function is used by the HAL to retrieve the configuration structure
 * for a given motor. The configuration contains microstep, current, and
 * clock settings used in velocity and current calculations.
 */
static trinamic_config_t *getConfig (uint8_t motor)
{
    return &tmcdriver[motor]->config;
}

/**
 * @brief Validate microsteps value for a motor
 * @param motor Motor index (0-5)
 * @param msteps Microsteps value to validate
 * @return true if valid, false otherwise
 *
 * This function checks if the provided microsteps value is within the
 * valid range for the TMC2209. It uses the internal TMC2209_microsteps_t
 * enumeration for validation.
 */
static bool isValidMicrosteps (uint8_t motor, uint16_t msteps)
{
    return tmc_microsteps_validate(msteps);
}

/**
 * @brief Set microsteps for a motor
 * @param motor Motor index (0-5)
 * @param msteps Microsteps value to set
 *
 * This function sets the microsteps for a given motor. It translates the
 * generic HAL microsteps value to the TMC2209's internal microsteps
 * enumeration and writes it to the TMC2209_microsteps register.
 */
static void setMicrosteps (uint8_t motor, uint16_t msteps)
{
   TMC2209_SetMicrosteps(tmcdriver[motor], (tmc2209_microsteps_t)msteps);
}

/**
 * @brief Set current for a motor
 * @param motor Motor index (0-5)
 * @param mA Current in milliamps
 * @param hold_pct Hold current percentage
 *
 * This function sets the current for a given motor. It writes the current
 * value (mA) and hold percentage to the TMC2209_ihold_irun register.
 */
static void setCurrent (uint8_t motor, uint16_t mA, uint8_t hold_pct)
{
    TMC2209_SetCurrent(tmcdriver[motor], mA, hold_pct);
}

/**
 * @brief Get current for a motor
 * @param motor Motor index (0-5)
 * @param type Type of current to get (e.g., I_RUN, I_HOLD)
 * @return Current value in microamperes
 *
 * This function reads the current value from the TMC2209_ihold_irun register.
 * The current type (I_RUN or I_HOLD) determines which register is read.
 */
static uint16_t getCurrent (uint8_t motor, trinamic_current_t type)
{
    return TMC2209_GetCurrent(tmcdriver[motor], type);
}

/**
 * @brief Get chopper configuration for a motor
 * @param motor Motor index (0-5)
 * @return TMC_chopconf_t structure containing chopper settings
 *
 * This function reads the chopper configuration from the TMC2209_chopconf
 * register and translates it into a generic TMC_chopconf_t structure.
 */
static TMC_chopconf_t getChopconf (uint8_t motor)
{
    TMC_chopconf_t chopconf;

    TMC2209_ReadRegister(tmcdriver[motor], (TMC2209_datagram_t *)&tmcdriver[motor]->chopconf);

    chopconf.mres = tmcdriver[motor]->chopconf.reg.mres;
    chopconf.toff = tmcdriver[motor]->chopconf.reg.toff;
    chopconf.tbl = tmcdriver[motor]->chopconf.reg.tbl;
    chopconf.hend = tmcdriver[motor]->chopconf.reg.hend;
    chopconf.hstrt = tmcdriver[motor]->chopconf.reg.hstrt;

    return chopconf;
}

/**
 * @brief Get stall guard result for a motor
 * @param motor Motor index (0-5)
 * @return Stall guard result value
 *
 * This function reads the stall guard result from the TMC2209_sg_result
 * register. The result indicates the current status of the stall guard.
 */
static uint32_t getStallGuardResult (uint8_t motor)
{
    TMC2209_ReadRegister(tmcdriver[motor], (TMC2209_datagram_t *)&tmcdriver[motor]->sg_result);

    return (uint32_t)tmcdriver[motor]->sg_result.reg.result;
}

/**
 * @brief Get driver status for a motor
 * @param motor Motor index (0-5)
 * @return TMC_drv_status_t structure containing driver status
 *
 * This function reads the driver status from the TMC2209_sg_result and
 * TMC2209_drv_status registers. It translates the raw register values into
 * a generic TMC_drv_status_t structure.
 */
static TMC_drv_status_t getDriverStatus (uint8_t motor)
{
    TMC_drv_status_t drv_status = {0};
    TMC2209_status_t status;

    TMC2209_ReadRegister(tmcdriver[motor], (TMC2209_datagram_t *)&tmcdriver[motor]->sg_result);
    status.value = TMC2209_ReadRegister(tmcdriver[motor], (TMC2209_datagram_t *)&tmcdriver[motor]->drv_status);

    drv_status.driver_error = status.driver_error;
    drv_status.sg_result = tmcdriver[motor]->sg_result.reg.result;
    drv_status.ot = tmcdriver[motor]->drv_status.reg.ot;
    drv_status.otpw = tmcdriver[motor]->drv_status.reg.otpw;
    drv_status.cs_actual = tmcdriver[motor]->drv_status.reg.cs_actual;
    drv_status.stst = tmcdriver[motor]->drv_status.reg.stst;
//    drv_status.fsactive = tmcdriver[motor]->drv_status.reg.fsactive;
    drv_status.ola = tmcdriver[motor]->drv_status.reg.ola;
    drv_status.olb = tmcdriver[motor]->drv_status.reg.olb;
    drv_status.s2ga = tmcdriver[motor]->drv_status.reg.s2ga;
    drv_status.s2gb = tmcdriver[motor]->drv_status.reg.s2gb;

    return drv_status;
}

/**
 * @brief Get Ihold and Irun for a motor
 * @param motor Motor index (0-5)
 * @return TMC_ihold_irun_t structure containing Ihold and Irun values
 *
 * This function reads the Ihold and Irun values from the TMC2209_ihold_irun
 * register. It translates the raw register values into a generic
 * TMC_ihold_irun_t structure.
 */
static TMC_ihold_irun_t getIholdIrun (uint8_t motor)
{
    TMC_ihold_irun_t ihold_irun;

    ihold_irun.ihold = tmcdriver[motor]->ihold_irun.reg.ihold;
    ihold_irun.irun = tmcdriver[motor]->ihold_irun.reg.irun;
    ihold_irun.iholddelay = tmcdriver[motor]->ihold_irun.reg.iholddelay;

    return ihold_irun;
}

/**
 * @brief Get raw driver status value for a motor
 * @param motor Motor index (0-5)
 * @return Raw driver status value
 *
 * This function reads the raw driver status value from the TMC2209_drv_status
 * register. The value is returned as a 32-bit unsigned integer.
 */
static uint32_t getDriverStatusRaw (uint8_t motor)
{
    TMC2209_ReadRegister(tmcdriver[motor], (TMC2209_datagram_t *)&tmcdriver[motor]->drv_status);

    return tmcdriver[motor]->drv_status.reg.value;
}

/**
 * @brief Get TStep for a motor
 * @param motor Motor index (0-5)
 * @return TStep value
 *
 * This function reads the TStep value from the TMC2209_tstep register.
 * The TStep value is used for velocity calculations.
 */
static uint32_t getTStep (uint8_t motor)
{
    TMC2209_ReadRegister(tmcdriver[motor], (TMC2209_datagram_t *)&tmcdriver[motor]->tstep);

    return (uint32_t)tmcdriver[motor]->tstep.reg.tstep;
}

/**
 * @brief Set TCOOLTHRS for a motor
 * @param motor Motor index (0-5)
 * @param mm_sec Feed rate in millimeters per second
 * @param steps_mm Steps per millimeter
 *
 * This function sets the TCOOLTHRS value for a given motor. It translates
 * the generic HAL parameters into the TMC2209's internal TCOOLTHRS register
 * value.
 */
static void setTCoolThrs (uint8_t motor, float mm_sec, float steps_mm)
{
    TMC2209_SetTCOOLTHRS(tmcdriver[motor], mm_sec, steps_mm);
}

/**
 * @brief Set TCOOLTHRS raw value for a motor
 * @param motor Motor index (0-5)
 * @param value Raw TCOOLTHRS value
 *
 * This function sets the raw TCOOLTHRS value for a given motor. It writes
 * the value to the TMC2209_tcoolthrs register.
 */
static void setTCoolThrsRaw (uint8_t motor, uint32_t value)
{
    tmcdriver[motor]->tcoolthrs.reg.tcoolthrs = value;
    TMC2209_WriteRegister(tmcdriver[motor], (TMC2209_datagram_t *)&tmcdriver[motor]->tcoolthrs);
}

/**
 * @brief Enable stall guard for a motor
 * @param motor Motor index (0-5)
 * @param feed_rate Feed rate in millimeters per second
 * @param steps_mm Steps per millimeter
 * @param sensitivity Stall guard sensitivity
 *
 * This function enables the stall guard for a given motor. It configures
 * the TMC2209 to use stealthChop mode, sets the PWM auto-scale, and
 * calculates the TCOOLTHRS value based on the feed rate and steps per
 * millimeter. It also sets the stall guard threshold.
 */
static void stallGuardEnable (uint8_t motor, float feed_rate, float steps_mm, int16_t sensitivity)
{
    TMC2209_t *driver = tmcdriver[motor];

    driver->gconf.reg.en_spreadcycle = false; // stealthChop on
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->gconf);

    driver->pwmconf.reg.pwm_autoscale = true;
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->pwmconf);

    TMC2209_SetTCOOLTHRS(driver, feed_rate / (60.0f * 1.5f), steps_mm);

    driver->sgthrs.reg.threshold = (uint8_t)sensitivity;
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->sgthrs);
}

/**
 * @brief Enable stealthChop mode for a motor
 * @param motor Motor index (0-5)
 *
 * This function enables stealthChop mode for a given motor. It configures
 * the TMC2209 to use stealthChop mode by setting en_spreadcycle to false
 * and pwm_autoscale to true. It also sets the TCOOLTHRS to 0.
 */
static void stealthChopEnable (uint8_t motor)
{
    TMC2209_t *driver = tmcdriver[motor];

    driver->gconf.reg.en_spreadcycle = false; // stealthChop on
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->gconf);

    driver->pwmconf.reg.pwm_autoscale = true;
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->pwmconf);

    setTCoolThrsRaw(motor, 0);
}

/**
 * @brief Enable coolStep mode for a motor
 * @param motor Motor index (0-5)
 *
 * This function enables coolStep mode for a given motor. It configures
 * the TMC2209 to use stealthChop mode by setting en_spreadcycle to true
 * and pwm_autoscale to false. It also sets the TCOOLTHRS to 0.
 */
static void coolStepEnable (uint8_t motor)
{
    TMC2209_t *driver = tmcdriver[motor];

    driver->gconf.reg.en_spreadcycle = true; // stealthChop off
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->gconf);

    driver->pwmconf.reg.pwm_autoscale = false;
    TMC2209_WriteRegister(driver, (TMC2209_datagram_t *)&driver->pwmconf);

    setTCoolThrsRaw(motor, 0);
}

/**
 * @brief Get TPWMTHRS for a motor
 * @param motor Motor index (0-5)
 * @param steps_mm Steps per millimeter
 * @return TPWMTHRS value
 *
 * This function calculates the TPWMTHRS value for a given motor based on
 * the feed rate and steps per millimeter. It uses the TMC2209's internal
 * TPWMTHRS calculation function.
 */
static float getTPWMThrs (uint8_t motor, float steps_mm)
{
    return TMC2209_GetTPWMTHRS(tmcdriver[motor], steps_mm);
}

/**
 * @brief Get raw TPWMTHRS value for a motor
 * @param motor Motor index (0-5)
 * @return Raw TPWMTHRS value
 *
 * This function reads the raw TPWMTHRS value from the TMC2209_tpwmthrs
 * register.
 */
static uint32_t getTPWMThrsRaw (uint8_t motor)
{
    return tmcdriver[motor]->tpwmthrs.reg.tpwmthrs;
}

/**
 * @brief Set TPWMTHRS for a motor
 * @param motor Motor index (0-5)
 * @param mm_sec Feed rate in millimeters per second
 * @param steps_mm Steps per millimeter
 *
 * This function sets the TPWMTHRS value for a given motor. It translates
 * the generic HAL parameters into the TMC2209's internal TPWMTHRS register
 * value.
 */
static void setTPWMThrs (uint8_t motor, float mm_sec, float steps_mm)
{
    TMC2209_SetTPWMTHRS(tmcdriver[motor], mm_sec, steps_mm);
}

/**
 * @brief Toggle stealthChop mode for a motor
 * @param motor Motor index (0-5)
 * @param on true to enable stealthChop, false to enable coolStep
 *
 * This function toggles the stealthChop mode for a given motor. It sets
 * the TMC2209's mode to stealthChop if on is true, and coolStep if on is
 * false. It then calls the appropriate enable function.
 */
static void stealthChop (uint8_t motor, bool on)
{
    tmcdriver[motor]->config.mode = on ? TMCMode_StealthChop : TMCMode_CoolStep;

    if(on)
        stealthChopEnable(motor);
    else
        coolStepEnable(motor);
}

/**
 * @brief Get stealthChop status for a motor
 * @param motor Motor index (0-5)
 * @return true if stealthChop is enabled, false otherwise
 *
 * This function checks if stealthChop is enabled by examining the TMC2209's
 * gconf and pwmconf registers.
 */
static bool stealthChopGet (uint8_t motor)
{
    return !tmcdriver[motor]->gconf.reg.en_spreadcycle && tmcdriver[motor]->pwmconf.reg.pwm_autoscale;
}

// coolconf

/**
 * @brief Set stall guard filter value
 * @param motor Motor index (0-5)
 * @param val true to enable filter, false to disable
 *
 * This function sets the stall guard filter value. It writes the value to
 * the TMC2209_coolconf register.
 */
static void sg_filter (uint8_t motor, bool val)
{
//    tmcdriver[motor]->sgthrs.reg.threshold = val;
//    TMC2209_WriteRegister(tmcdriver[motor], (TMC2209_datagram_t *)&tmcdriver[motor]->coolconf);
}

/**
 * @brief Set stall guard stall value
 * @param motor Motor index (0-5)
 * @param val Stall guard stall value
 *
 * This function sets the stall guard stall value. It writes the value to
 * the TMC2209_sgthrs register.
 */
static void sg_stall_value (uint8_t motor, int16_t val)
{
    tmcdriver[motor]->sgthrs.reg.threshold = (uint8_t)val;
    TMC2209_WriteRegister(tmcdriver[motor], (TMC2209_datagram_t *)&tmcdriver[motor]->sgthrs);
}

/**
 * @brief Get stall guard stall value
 * @param motor Motor index (0-5)
 * @return Stall guard stall value
 *
 * This function reads the stall guard stall value from the TMC2209_sgthrs
 * register.
 */
static int16_t get_sg_stall_value (uint8_t motor)
{
    return (int16_t)tmcdriver[motor]->sgthrs.reg.threshold;
}

/**
 * @brief Set coolconf for a motor
 * @param motor Motor index (0-5)
 * @param coolconf trinamic_coolconf_t structure containing coolconf settings
 *
 * This function sets the coolconf values for a given motor. It writes the
 * values to the TMC2209_coolconf register.
 */
static void coolconf (uint8_t motor, trinamic_coolconf_t coolconf)
{
    TMC2209_t *driver = tmcdriver[motor];

    driver->coolconf.reg.semin = coolconf.semin;
    driver->coolconf.reg.semax = coolconf.semax;
    driver->coolconf.reg.sedn = coolconf.sedn;
    driver->coolconf.reg.seimin = coolconf.seimin;
    driver->coolconf.reg.seup = coolconf.seup;
    TMC2209_WriteRegister(tmcdriver[motor], (TMC2209_datagram_t *)&driver->coolconf);
}

// chopconf

/**
 * @brief Set chopper timing for a motor
 * @param motor Motor index (0-5)
 * @param chopconf trinamic_chopconf_t structure containing chopper settings
 *
 * This function sets the chopper timing values for a given motor. It writes
 * the values to the TMC2209_chopconf register.
 */
static void chopper_timing (uint8_t motor, trinamic_chopconf_t chopconf)
{
    TMC2209_t *driver = tmcdriver[motor];

    driver->chopconf.reg.hstrt = chopconf.hstrt;
    driver->chopconf.reg.hend = chopconf.hend;
    driver->chopconf.reg.tbl = chopconf.tbl;
    driver->chopconf.reg.toff = chopconf.toff;
    TMC2209_WriteRegister(tmcdriver[motor], (TMC2209_datagram_t *)&driver->chopconf);
}

/**
 * @brief Get PWM scale for a motor
 * @param motor Motor index (0-5)
 * @return PWM scale value
 *
 * This function reads the PWM scale value from the TMC2209_pwm_scale
 * register.
 */
static uint8_t pwm_scale (uint8_t motor)
{
    TMC2209_ReadRegister(tmcdriver[motor], (TMC2209_datagram_t *)&tmcdriver[motor]->pwm_scale);

    return tmcdriver[motor]->pwm_scale.reg.pwm_scale_sum;
}

/**
 * @brief Check if VSENSE is enabled for a motor
 * @param motor Motor index (0-5)
 * @return true if VSENSE is enabled, false otherwise
 *
 * This function checks if VSENSE is enabled by reading the TMC2209_chopconf
 * register.
 */
static bool vsense (uint8_t motor)
{
    TMC2209_ReadRegister(tmcdriver[motor], (TMC2209_datagram_t *)&tmcdriver[motor]->chopconf);

    return tmcdriver[motor]->chopconf.reg.vsense;
}

/**
 * @brief Read a register from the TMC2209
 * @param motor Motor index (0-5)
 * @param addr Register address
 * @param val Pointer to store the read value
 * @return true on success, false on failure
 *
 * This function reads a single register from the TMC2209. It constructs
 * the TMC2209_datagram_t, reads the register, and stores the value.
 */
static bool read_register (uint8_t motor, uint8_t addr, uint32_t *val)
{
    TMC2209_datagram_t reg;
    reg.addr.reg = (tmc2209_regaddr_t)addr;
    reg.addr.write = 1;

    TMC2209_ReadRegister(tmcdriver[motor], &reg);

    *val = reg.payload.value;

    return true;
}

/**
 * @brief Write a register to the TMC2209
 * @param motor Motor index (0-5)
 * @param addr Register address
 * @param val Value to write
 * @return true on success, false on failure
 *
 * This function writes a single register to the TMC2209. It constructs
 * the TMC2209_datagram_t, sets the value, and writes the register.
 */
static bool write_register (uint8_t motor, uint8_t addr, uint32_t val)
{
    TMC2209_datagram_t reg;
    reg.addr.reg = (tmc2209_regaddr_t)addr;
    reg.addr.write = 0;
    reg.payload.value = val;

    TMC2209_WriteRegister(tmcdriver[motor], &reg);

    return true;
}

/**
 * @brief Get pointer to a specific register in the TMC2209 driver structure
 * @param motor Motor index (0-5)
 * @param addr Register address
 * @return Pointer to the register
 *
 * This function returns a pointer to a specific register in the TMC2209
 * driver structure based on the register address.
 */
static void *get_register_addr (uint8_t motor, uint8_t addr)
{
    return TMC2209_GetRegPtr(tmcdriver[motor], (tmc2209_regaddr_t)addr);
}

static const tmchal_t tmc_hal = {
    .driver = TMC2209,
    .name = "TMC2209",

    .get_config = getConfig,

    .microsteps_isvalid = isValidMicrosteps,
    .set_microsteps = setMicrosteps,
    .set_current = setCurrent,
    .get_current = getCurrent,
    .get_chopconf = getChopconf,
    .get_tstep = getTStep,
    .get_drv_status = getDriverStatus,
    .get_drv_status_raw = getDriverStatusRaw,
    .set_tcoolthrs = setTCoolThrs,
    .set_tcoolthrs_raw = setTCoolThrsRaw,
    .set_thigh = NULL,
    .set_thigh_raw = NULL,
    .stallguard_enable = stallGuardEnable,
    .stealthchop_enable = stealthChopEnable,
    .coolstep_enable = coolStepEnable,
    .get_sg_result = getStallGuardResult,
    .get_tpwmthrs = getTPWMThrs,
    .get_tpwmthrs_raw = getTPWMThrsRaw,
    .set_tpwmthrs = setTPWMThrs,
    .get_en_pwm_mode = stealthChopGet,
    .get_ihold_irun = getIholdIrun,

    .stealthChop = stealthChop,
    .sg_filter = sg_filter,
    .sg_stall_value = sg_stall_value,
    .get_sg_stall_value = get_sg_stall_value,
    .coolconf = coolconf,
    .vsense = vsense,
    .pwm_scale = pwm_scale,
    .chopper_timing = chopper_timing,

    .get_register_addr = get_register_addr,
    .read_register = read_register,
    .write_register = write_register
};

const tmchal_t *TMC2209_AddMotor (motor_map_t motor, uint8_t address, uint16_t current, uint8_t microsteps, uint8_t r_sense)
{
    bool ok = !!tmcdriver[motor.id];

    if(ok || (ok = (tmcdriver[motor.id] = malloc(sizeof(TMC2209_t))) != NULL)) {
        TMC2209_SetDefaults(tmcdriver[motor.id]);
        tmcdriver[motor.id]->config.motor.id = motor.id;
        tmcdriver[motor.id]->config.motor.address = address;
        tmcdriver[motor.id]->config.motor.axis = motor.axis;
        tmcdriver[motor.id]->config.current = current;
        tmcdriver[motor.id]->config.microsteps = microsteps;
        tmcdriver[motor.id]->config.r_sense = r_sense;
        tmcdriver[motor.id]->chopconf.reg.mres = tmc_microsteps_to_mres(microsteps);
    }

    if(ok && !(ok = TMC2209_Init(tmcdriver[motor.id]))) {
        free(tmcdriver[motor.id]);
        tmcdriver[motor.id] = NULL;
    }

    return ok ? &tmc_hal : NULL;
}
