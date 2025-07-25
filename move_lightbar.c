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
// MICROSTEP_RESOLUTION = 8 (?)

// DEFAULT_CLOSE_PROFILE
// MAX_SPEED_RPM = 3.75 (100 Hz)
// GEAR_RATIO = 99.548
// MICROSTEP_RESOLUTION = 8 (?)


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








