/*
 * ============================================================================
 * COMMON.C - SHARED UTILITY FUNCTIONS FOR TRINAMIC DRIVERS
 * ============================================================================
 * 
 * This file contains utility functions that are shared across different
 * Trinamic stepper motor drivers. These functions provide common functionality
 * for microstep validation, current calculations, and communication protocols.
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

#include "common.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

static uint8_t tmc_motors = 0;  // Bitmap of configured motors (0-5)

// ============================================================================
// MICROSTEP VALIDATION AND CONVERSION
// ============================================================================

/**
 * @brief Validate microstep value for Trinamic drivers
 * @param microsteps Microstep value to validate (1-256)
 * @return true if valid, false otherwise
 * 
 * Trinamic drivers only support microstep values that are powers of 2:
 * 1, 2, 4, 8, 16, 32, 64, 128, 256
 * 
 * This function checks if the microstep value has exactly one bit set,
 * which indicates it's a valid power of 2.
 */
bool tmc_microsteps_validate (uint16_t microsteps)
{
    uint_fast8_t i = 8, count = 0;

    // Check if microsteps is within valid range (1-256)
    if(microsteps <= 256) do {
        if(microsteps & 0x01)  // Check if least significant bit is set
            count++;           // Count set bits
        microsteps >>= 1;      // Shift right to check next bit
    } while(i--);              // Check all 8 bits

    // Valid microstep values have exactly one bit set
    return count == 1;
}

/**
 * @brief Convert microstep value to MRES register value
 * @param microsteps Microstep value (1-256, must be power of 2)
 * @return MRES register value (0-8)
 * 
 * Converts microstep values to the MRES (Microstep Resolution) register value:
 * - 1 microstep   -> MRES = 8
 * - 2 microsteps  -> MRES = 7
 * - 4 microsteps  -> MRES = 6
 * - 8 microsteps  -> MRES = 5
 * - 16 microsteps -> MRES = 4
 * - 32 microsteps -> MRES = 3
 * - 64 microsteps -> MRES = 2
 * - 128 microsteps-> MRES = 1
 * - 256 microsteps-> MRES = 0
 * 
 * The formula is: MRES = 8 - log2(microsteps)
 */
uint8_t tmc_microsteps_to_mres (uint16_t microsteps)
{
    uint8_t value = 0;

    // Handle edge case: 0 microsteps defaults to 1
    microsteps = microsteps == 0 ? 1 : microsteps;

    // Count trailing zeros (equivalent to log2 of the lowest set bit)
    while((microsteps & 0x01) == 0) {
        value++;
        microsteps >>= 1;
    }

    // Convert to MRES value: 8 - log2(microsteps)
    return 8 - (value > 8 ? 8 : value);
}

// ============================================================================
// VELOCITY CALCULATIONS
// ============================================================================

/**
 * @brief Calculate TSTEP value from velocity
 * @param config Driver configuration structure
 * @param mm_sec Velocity in mm/second
 * @param steps_mm Steps per mm (mechanical resolution)
 * @return TSTEP register value (0-2^20-1)
 * 
 * TSTEP is the step velocity register used by Trinamic drivers.
 * The formula is: TSTEP = (microsteps * f_clk) / (256 * velocity * steps_mm)
 * 
 * Where:
 * - microsteps: Microstep resolution (1-256)
 * - f_clk: Internal clock frequency (typically 12MHz)
 * - velocity: Speed in mm/second
 * - steps_mm: Mechanical steps per mm
 * 
 * Lower TSTEP values = higher velocity
 */
uint32_t tmc_calc_tstep (trinamic_config_t *config, float mm_sec, float steps_mm)
{
    // Calculate denominator: 256 * velocity * steps_mm
    uint32_t den = (uint32_t)(256.0f * mm_sec * steps_mm);

    // Calculate TSTEP, avoid division by zero
    return den ? (config->microsteps * config->f_clk) / den : 0;
}

/**
 * @brief Calculate velocity from TSTEP value
 * @param config Driver configuration structure
 * @param tstep TSTEP register value (0-2^20-1)
 * @param steps_mm Steps per mm (mechanical resolution)
 * @return Velocity in mm/second
 * 
 * Inverse function of tmc_calc_tstep().
 * The formula is: velocity = (microsteps * f_clk) / (256 * TSTEP * steps_mm)
 * 
 * Returns 0.0 if TSTEP is 0 (invalid or stopped condition).
 */
float tmc_calc_tstep_inv (trinamic_config_t *config, uint32_t tstep, float steps_mm)
{
    // Avoid division by zero
    return tstep == 0 ? 0.0f : (float)(config->f_clk * config->microsteps) / (256.0f * (float)tstep * steps_mm);
}

// ============================================================================
// MOTOR MANAGEMENT
// ============================================================================

/**
 * @brief Set the bitmap of configured motors
 * @param motors Bitmap where each bit represents a motor (0-5)
 * 
 * This function sets which motors are configured in the system.
 * Each bit in the motors parameter represents a motor:
 * - Bit 0: Motor 0
 * - Bit 1: Motor 1
 * - Bit 2: Motor 2
 * - Bit 3: Motor 3
 * - Bit 4: Motor 4
 * - Bit 5: Motor 5
 */
void tmc_motors_set (uint8_t motors)
{
    tmc_motors = motors;
}

/**
 * @brief Get the bitmap of configured motors
 * @return Bitmap of configured motors
 * 
 * Returns the current bitmap of which motors are configured.
 * Each bit represents a motor (0-5).
 */
uint8_t tmc_motors_get (void)
{
    return tmc_motors;
}

// ============================================================================
// COMMUNICATION PROTOCOLS
// ============================================================================

/**
 * @brief Calculate CRC8 checksum for Trinamic UART communication
 * @param datagram Pointer to the datagram buffer
 * @param datagramLength Length of the datagram in bytes
 * 
 * This function calculates a CRC8 checksum for Trinamic UART communication.
 * The CRC is calculated using polynomial 0x07 (x^8 + x^2 + x + 1).
 * 
 * The CRC is stored in the last byte of the datagram.
 * 
 * Algorithm:
 * 1. Initialize CRC to 0
 * 2. For each byte in the datagram (except the last byte):
 *    - For each bit in the byte (LSB first):
 *      - If (CRC MSB XOR data bit) is 1, shift CRC left and XOR with 0x07
 *      - Otherwise, just shift CRC left
 * 3. Store the final CRC in the last byte
 * 
 * This ensures data integrity in UART communication with Trinamic drivers.
 */
void tmc_crc8 (uint8_t *datagram, uint8_t datagramLength)
{
    int i,j;
    uint8_t *crc = datagram + (datagramLength - 1); // CRC located in last byte of message
    uint8_t currentByte;
    
    // Initialize CRC to 0
    *crc = 0;
    
    // Process all bytes except the last one (which will contain the CRC)
    for (i = 0; i < (datagramLength - 1); i++) {
        currentByte = datagram[i];                  // Get current byte to process
        
        // Process each bit in the byte (LSB first)
        for (j = 0; j < 8; j++) {
            if ((*crc >> 7) ^ (currentByte & 0x01)) // XOR CRC MSB with data LSB
                *crc = (*crc << 1) ^ 0x07;          // Shift left and XOR with polynomial
            else
                *crc = (*crc << 1);                 // Just shift left
            currentByte = currentByte >> 1;         // Move to next bit
        }
    }
}
