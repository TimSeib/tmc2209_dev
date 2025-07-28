/*
    This file contains the process of moving the lightbar either out or in.
    It will take environment variables to determine the profile of the motion.
    Ie. the max speed, max acceleration, jerk, initial velocity, and gear ratio.
    It will use the contents of stepcount.dat to determine the cur position of the lightbar.
    If stepcount.dat is 0 then it moves 45 deg clockwise, if it is close to MAX_MSCNT then it moves 45 deg counter-clockwise.
    If it's in the middle of the motion then it will continue in the same direction, or default to close if that is not known.

*/


//take in environment variables

// DEFAULT_PROFILE
// MAX_SPEED_RPM = 15 (400 Hz)
// MAX_ACCELERATION_HZ_PER_SEC = 0.25
// JERK_RATE_HZ_PER_SEC2 = 0.001
// START_SPEED_HZ = 100
// START_ACCELERATION_HZ_PER_SEC = 0
// GEAR_RATIO = 99.548
// MICROSTEP_RESOLUTION = 8 (?)

// SLOW_PROFILE
// MAX_SPEED_RPM = 7.5 (200 Hz)
// MAX_ACCELERATION_HZ_PER_SEC = 0.25
// JERK_RATE_HZ_PER_SEC2 = 0.001
// START_SPEED_HZ = 100
// START_ACCELERATION_HZ_PER_SEC = 0
// GEAR_RATIO = 99.548
// MICROSTEP_RESOLUTION = 32 (?)

// DEFAULT_CLOSE_PROFILE (200 seconds)
// MAX_SPEED_RPM = 3.75 (100 Hz)
// GEAR_RATIO = 99.548
// MICROSTEP_RESOLUTION = 32 (?)


// get the current position of the lightbar, valid bit, and direction

// perform checks based on value of stepcount.dat

// if valid bit is not set then move to close slowly with SG (default_close)

// if stepcount.dat is 0 then move 45 deg out

// if stepcount.dat is close to MAX_MSCNT then move 45 deg in

// if stepcount.dat is in the middle of the motion then continue in the same direction

    // select the motion profile based on the value of stepcount.dat (0-39 deg default_profile, 5-2.5 slow_profile, <2.5 default_close)


// once s_curve profile is set, start motion

// set valid bit to 0

// wait for motion to complete

// motion complete, set valid bit to 1

// if any interrupt occurs during motion that can be handled gracefully, save context to file (MSCNT value, direction, etc.) and exit

// stallguard interrupt will handle contiuing motion, in case of all other interrupts (what are they?) the motion will be resumed on reboot

// Read and print all environment variables
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "log.h"

#define LOG_LEVEL_FATAL 5
#define LOG_LEVEL_ERROR 4
#define LOG_LEVEL_WARN 3
#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_DEBUG 1
#define LOG_LEVEL_TRACE 0

// Profile structures
typedef struct {
    char name[50];
    double max_speed_rpm;
    double max_acceleration_hz_per_sec;
    double jerk_rate_hz_per_sec2;
    double start_speed_hz;
    double start_acceleration_hz;
    double gear_ratio;
    int microstep_resolution;
} motion_profile_t;

void extract_and_print_profiles() {
    // Get environment variables
    const char *msres_fast = getenv("MSRES_FAST");
    const char *msres_slow = getenv("MSRES_SLOW");
    const char *max_speed_rpm_fast = getenv("MAX_SPEED_RPM_FAST");
    const char *max_speed_rpm_slow = getenv("MAX_SPEED_RPM_SLOW");
    const char *max_speed_rpm_close = getenv("MAX_SPEED_RPM_CLOSE");
    const char *max_acceleration = getenv("MAX_ACCELERATION_HZ_PER_SEC");
    const char *jerk_rate = getenv("JERK_RATE_HZ_PER_SEC2");
    const char *start_speed = getenv("START_SPEED_HZ");
    const char *start_acceleration = getenv("START_ACCELERATION_HZ");
    const char *gear_ratio = getenv("GEAR_RATIO");
    
    // Create profiles
    motion_profile_t default_profile = {
        .name = "DEFAULT_PROFILE",
        .max_speed_rpm = max_speed_rpm_fast ? atof(max_speed_rpm_fast) : 15.0,
        .max_acceleration_hz_per_sec = max_acceleration ? atof(max_acceleration) : 0.25,
        .jerk_rate_hz_per_sec2 = jerk_rate ? atof(jerk_rate) : 0.001,
        .start_speed_hz = start_speed ? atof(start_speed) : 100.0,
        .start_acceleration_hz = start_acceleration ? atof(start_acceleration) : 0.0,
        .gear_ratio = gear_ratio ? atof(gear_ratio) : 99.548,
        .microstep_resolution = msres_fast ? atoi(msres_fast) : 8
    };
    
    motion_profile_t slow_profile = {
        .name = "SLOW_PROFILE",
        .max_speed_rpm = max_speed_rpm_slow ? atof(max_speed_rpm_slow) : 7.5,
        .max_acceleration_hz_per_sec = max_acceleration ? atof(max_acceleration) : 0.25,
        .jerk_rate_hz_per_sec2 = jerk_rate ? atof(jerk_rate) : 0.001,
        .start_speed_hz = start_speed ? atof(start_speed) : 100.0,
        .start_acceleration_hz = start_acceleration ? atof(start_acceleration) : 0.0,
        .gear_ratio = gear_ratio ? atof(gear_ratio) : 99.548,
        .microstep_resolution = msres_slow ? atoi(msres_slow) : 32
    };
    
    motion_profile_t default_close_profile = {
        .name = "DEFAULT_CLOSE_PROFILE (200 seconds)",
        .max_speed_rpm = max_speed_rpm_close ? atof(max_speed_rpm_close) : 3.75,
        .max_acceleration_hz_per_sec = max_acceleration ? atof(max_acceleration) : 0.25,
        .jerk_rate_hz_per_sec2 = jerk_rate ? atof(jerk_rate) : 0.001,
        .start_speed_hz = start_speed ? atof(start_speed) : 100.0,
        .start_acceleration_hz = start_acceleration ? atof(start_acceleration) : 0.0,
        .gear_ratio = gear_ratio ? atof(gear_ratio) : 99.548,
        .microstep_resolution = msres_slow ? atoi(msres_slow) : 32
    };
    
    // Print profiles
    motion_profile_t profiles[] = {default_profile, slow_profile, default_close_profile};
    
    for (int i = 0; i < 3; i++) {
        log_debug("%s:", profiles[i].name);
        log_debug("MAX_SPEED_RPM = %.2f (%.0f Hz)", 
               profiles[i].max_speed_rpm, 
               profiles[i].max_speed_rpm * 26.67); // Convert RPM to Hz (15 RPM = 400 Hz)
        log_debug("MAX_ACCELERATION_HZ_PER_SEC = %.3f", profiles[i].max_acceleration_hz_per_sec);
        log_debug("JERK_RATE_HZ_PER_SEC2 = %.3f", profiles[i].jerk_rate_hz_per_sec2);
        log_debug("START_SPEED_HZ = %.1f", profiles[i].start_speed_hz);
        log_debug("START_ACCELERATION_HZ_PER_SEC = %.1f", profiles[i].start_acceleration_hz);
        log_debug("GEAR_RATIO = %.3f", profiles[i].gear_ratio);
        log_debug("MICROSTEP_RESOLUTION = %d", profiles[i].microstep_resolution);
    }
}

int main(int argc, char *argv[]) {
    // Set log level based on command line argument
    if (argc > 1) {
        char *endptr;
        int log_level = strtol(argv[1], &endptr, 10);
        if (*endptr != '\0' || log_level < LOG_LEVEL_TRACE || log_level > LOG_LEVEL_FATAL) {
            fprintf(stderr, "Invalid log level: %s\n", argv[1]);
            fprintf(stderr, "Valid log levels are:\n");
            fprintf(stderr, "  %d = TRACE\n", LOG_LEVEL_TRACE);
            fprintf(stderr, "  %d = DEBUG\n", LOG_LEVEL_DEBUG);
            fprintf(stderr, "  %d = INFO\n", LOG_LEVEL_INFO);
            fprintf(stderr, "  %d = WARN\n", LOG_LEVEL_WARN);
            fprintf(stderr, "  %d = ERROR\n", LOG_LEVEL_ERROR);
            fprintf(stderr, "  %d = FATAL\n", LOG_LEVEL_FATAL);
            fprintf(stderr, "Usage: %s [LOG_LEVEL]\n", argv[0]);
            return 1;
        }
        log_set_level(log_level);
    } else {
        // Default to info level if no argument provided
        log_set_level(LOG_LEVEL_INFO);
    }
    
    extract_and_print_profiles();
    return 0;
}