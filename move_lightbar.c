/*
    This file contains the process of moving the lightbar either out or in.
    It will take environment variables to determine the profile of the motion.
    Ie. the max speed, max acceleration, jerk, initial velocity, and gear ratio.
    It will use the contents of stepcount.dat to determine the cur position of the lightbar.
    If stepcount.dat is 0 then it moves 45 deg clockwise, if it is close to MAX_MSCNT then it moves 45 deg counter-clockwise.
    If it's in the middle of the motion then it will continue in the same direction, or default to close if that is not known.

*/




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
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdint.h>
#include <signal.h>
#include <errno.h>
#include "tmc2209.h"
#include "log.h"
#include "tmc_gpio.h"
#include "tmc_motion.h"

// Global variables for signal handling
static volatile bool g_shutdown_requested = false;
static tmc_motion_s_curve_t *g_current_motion = NULL;
static TMC2209_t *g_current_driver = NULL;
static tmc_gpio_context_t *g_current_gpio = NULL;

// Callback function pointer for final actions
static void (*g_final_actions_callback)(void) = NULL;

// Max MSCNT
#define MAX_MSCNT 637087
#define MAX_MSCNT_THEORETICAL 637088

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

// ============================================================================
// SIGNAL HANDLING AND GRACEFUL SHUTDOWN
// ============================================================================

/**
 * @brief Signal handler for graceful shutdown
 * 
 * @param sig Signal number
 */
static void signal_handler(int sig) {
    log_info("Received signal %d (%s), initiating graceful shutdown...", sig, 
             sig == SIGINT ? "Interrupt" : 
             sig == SIGTERM ? "Terminate" : 
             sig == SIGQUIT ? "Quit" : "Unknown");
    
    if (g_shutdown_requested) {
        log_warn("Shutdown already in progress, forcing exit");
        exit(1);
    }
    
    g_shutdown_requested = true;
}

/**
 * @brief Save comprehensive motion state to file
 * 
 * @param current_mscnt Current MSCNT value
 * @param current_step_count Current step count
 * @param direction Current direction
 * @param valid_bit Whether position is valid
 * @param total_mscnt_delta Total MSCNT delta accumulated
 * @return true if save successful, false otherwise
 */
static bool save_motion_state(uint32_t current_mscnt, uint32_t current_step_count,
                            int direction, int valid_bit, int32_t total_mscnt_delta) {
    // Save final MSCNT delta to stepcount.dat (this is the accurate accumulated value)
    int fd = open("stepcount.dat", O_RDWR | O_CREAT, 0644);
    if (fd == -1) {
        log_error("Failed to open stepcount.dat for saving: %s", strerror(errno));
        return false;
    }
    
    // Ensure file has proper size
    if (ftruncate(fd, sizeof(int32_t)) == -1) {
        log_error("Failed to set file size: %s", strerror(errno));
        close(fd);
        return false;
    }
    
    // Memory map the file
    volatile int32_t *step_ptr = mmap(NULL, sizeof(int32_t), 
                                    PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (step_ptr == MAP_FAILED) {
        log_error("Failed to mmap stepcount.dat for saving: %s", strerror(errno));
        close(fd);
        return false;
    }
    
    // Save the final MSCNT delta (this is the accurate accumulated value)
    *step_ptr = current_mscnt;
    
    // Sync to disk
    if (msync((void*)step_ptr, sizeof(int32_t), MS_SYNC) == -1) {
        log_error("Failed to sync stepcount.dat: %s", strerror(errno));
    }
    
    // Cleanup
    munmap((void*)step_ptr, sizeof(int32_t));
    close(fd);
    
    // Save microstep count to microstepcount.dat
    int fd_micro = open("microstepcount.dat", O_RDWR | O_CREAT, 0644);
    if (fd_micro == -1) {
        log_error("Failed to open microstepcount.dat for saving: %s", strerror(errno));
        return false;
    }
    
    // Ensure file has proper size
    if (ftruncate(fd_micro, sizeof(int32_t)) == -1) {
        log_error("Failed to set microstepcount.dat file size: %s", strerror(errno));
        close(fd_micro);
        return false;
    }
    
    // Memory map the file
    volatile int32_t *micro_ptr = mmap(NULL, sizeof(int32_t), 
                                     PROT_READ | PROT_WRITE, MAP_SHARED, fd_micro, 0);
    if (micro_ptr == MAP_FAILED) {
        log_error("Failed to mmap microstepcount.dat for saving: %s", strerror(errno));
        close(fd_micro);
        return false;
    }
    
    // Save the microstep count
    *micro_ptr = (int32_t)current_step_count;
    
    // Sync to disk
    if (msync((void*)micro_ptr, sizeof(int32_t), MS_SYNC) == -1) {
        log_error("Failed to sync microstepcount.dat: %s", strerror(errno));
    }
    
    // Cleanup
    munmap((void*)micro_ptr, sizeof(int32_t));
    close(fd_micro);
    
    log_info("Saved motion state: MSCNT_Delta=%d (stepcount.dat), Microstep_Count=%u (microstepcount.dat)", 
            total_mscnt_delta, current_step_count);
    
    return true;
}

/**
 * @brief Perform graceful shutdown
 * 
 * @param current_mscnt Current MSCNT value
 * @param current_step_count Current step count
 * @param direction Current direction
 * @param valid_bit Whether position is valid
 * @param total_mscnt_delta Total MSCNT delta accumulated
 */
static void graceful_shutdown(uint32_t current_mscnt, uint32_t current_step_count, 
                            int direction, int valid_bit, int32_t total_mscnt_delta) {
    log_info("Performing graceful shutdown...");
    
    // Call user-defined final actions callback if set
    if (g_final_actions_callback) {
        log_info("Executing user-defined final actions...");
        g_final_actions_callback();
    }
    
    // Note: Motion has already been stopped by the caller
    
    // Disable motor
    if (g_current_gpio) {
        log_info("Disabling motor...");
        tmc_gpio_enable_driver(g_current_gpio, false);
    }
    
    // Save current position and comprehensive state
    log_info("Saving current position and comprehensive state...");
    if (!save_motion_state(current_mscnt, current_step_count, direction, valid_bit, total_mscnt_delta)) {
        log_error("Failed to save motion state during shutdown");
    }
    
    // Cleanup motion control
    if (g_current_motion) {
        log_info("Cleaning up motion control...");
        tmc_motion_s_curve_deinit(g_current_motion);
    }
    
    // Cleanup GPIO
    if (g_current_gpio) {
        log_info("Cleaning up GPIO...");
        tmc_gpio_deinit(g_current_gpio);
    }
    
    log_info("Graceful shutdown completed");
}

/**
 * @brief Setup signal handlers for graceful shutdown
 */
static void setup_signal_handlers(void) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = signal_handler;
    
    // Handle common termination signals
    sigaction(SIGINT, &sa, NULL);   // Ctrl+C
    sigaction(SIGTERM, &sa, NULL);  // Termination request
    sigaction(SIGHUP, &sa, NULL);   // Hangup
    sigaction(SIGQUIT, &sa, NULL);  // Quit
    
    // Handle other signals that should trigger graceful shutdown
    sigaction(SIGUSR1, &sa, NULL);  // User-defined signal 1
    sigaction(SIGUSR2, &sa, NULL);  // User-defined signal 2
    
    log_debug("Signal handlers installed for graceful shutdown");
}

/**
 * @brief Set callback function for final actions before shutdown
 * 
 * @param callback Function pointer to callback (can be NULL)
 */
void set_final_actions_callback(void (*callback)(void)) {
    g_final_actions_callback = callback;
    log_debug("Final actions callback %s", callback ? "set" : "cleared");
}

/**
 * @brief Save motor direction to file
 * 
 * @param direction 0 for clockwise, 1 for counter-clockwise
 * @return true if successful, false otherwise
 */
static bool save_motor_direction(int direction) {
    int fd = open("direction.dat", O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd == -1) {
        log_error("Failed to open direction.dat for writing: %s", strerror(errno));
        return false;
    }
    
    // Write direction as binary int32_t
    int32_t dir_value = direction;
    if (write(fd, &dir_value, sizeof(int32_t)) != sizeof(int32_t)) {
        log_error("Failed to write to direction.dat: %s", strerror(errno));
        close(fd);
        return false;
    }
    
    close(fd);
    log_info("Saved motor direction: %s (value: %d)", 
             direction ? "counter-clockwise" : "clockwise", direction);
    return true;
}

/**
 * @brief Read motor direction from file
 * 
 * @return 0 for clockwise, 1 for counter-clockwise, defaults to 0 if file not found
 */
static int read_motor_direction(void) {
    int fd = open("direction.dat", O_RDONLY);
    if (fd == -1) {
        log_warn("direction.dat not found, assuming clockwise direction");
        return 0; // Default to clockwise
    }
    
    // Check file size
    struct stat st;
    if (fstat(fd, &st) == -1) {
        log_warn("Failed to get direction.dat stats, assuming clockwise direction");
        close(fd);
        return 0;
    }
    
    if (st.st_size != sizeof(int32_t)) {
        log_warn("direction.dat size is %ld bytes, expected %zu bytes, assuming clockwise direction", 
                st.st_size, sizeof(int32_t));
        close(fd);
        return 0;
    }
    
    // Read direction value
    int32_t direction = 0;
    if (read(fd, &direction, sizeof(int32_t)) != sizeof(int32_t)) {
        log_warn("Failed to read direction.dat, assuming clockwise direction");
        close(fd);
        return 0;
    }
    
    close(fd);
    log_info("Read motor direction from file: %s (value: %d)", 
             direction ? "counter-clockwise" : "clockwise", direction);
    return direction;
}


void extract_profiles(motion_profile_t *default_profile, motion_profile_t *slow_profile, motion_profile_t *default_close_profile) {
    // Get environment variables
    const char *msres_default = getenv("MSRES_DEFAULT");
    const char *max_speed_rpm_fast = getenv("MAX_SPEED_RPM_FAST");
    const char *max_speed_rpm_slow = getenv("MAX_SPEED_RPM_SLOW");
    const char *max_speed_rpm_close = getenv("MAX_SPEED_RPM_CLOSE");
    const char *max_acceleration = getenv("MAX_ACCELERATION_HZ_PER_SEC");
    const char *jerk_rate = getenv("JERK_RATE_HZ_PER_SEC2");
    const char *start_speed = getenv("START_SPEED_HZ");
    const char *start_acceleration = getenv("START_ACCELERATION_HZ");
    const char *gear_ratio = getenv("GEAR_RATIO");
    
    // Create default profile
    strcpy(default_profile->name, "DEFAULT_PROFILE");
    default_profile->max_speed_rpm = max_speed_rpm_fast ? atof(max_speed_rpm_fast) : 15.0;
    default_profile->max_acceleration_hz_per_sec = max_acceleration ? atof(max_acceleration) : 0.25;
    default_profile->jerk_rate_hz_per_sec2 = jerk_rate ? atof(jerk_rate) : 0.001;
    default_profile->start_speed_hz = start_speed ? atof(start_speed) : 100.0;
    default_profile->start_acceleration_hz = start_acceleration ? atof(start_acceleration) : 0.0;
    default_profile->gear_ratio = gear_ratio ? atof(gear_ratio) : 99.548;
    default_profile->microstep_resolution = msres_default ? atoi(msres_default) : 8;
    
    // Create slow profile
    strcpy(slow_profile->name, "SLOW_PROFILE");
    slow_profile->max_speed_rpm = max_speed_rpm_slow ? atof(max_speed_rpm_slow) : 7.5;
    slow_profile->max_acceleration_hz_per_sec = max_acceleration ? atof(max_acceleration) : 0.25;
    slow_profile->jerk_rate_hz_per_sec2 = jerk_rate ? atof(jerk_rate) : 0.001;
    slow_profile->start_speed_hz = start_speed ? atof(start_speed) : 100.0;
    slow_profile->start_acceleration_hz = start_acceleration ? atof(start_acceleration) : 0.0;
    slow_profile->gear_ratio = gear_ratio ? atof(gear_ratio) : 99.548;
    slow_profile->microstep_resolution = msres_default ? atoi(msres_default) : 8;
    
    // Create default close profile
    strcpy(default_close_profile->name, "DEFAULT_CLOSE_PROFILE (200 seconds)");
    default_close_profile->max_speed_rpm = max_speed_rpm_close ? atof(max_speed_rpm_close) : 3.75;
    default_close_profile->max_acceleration_hz_per_sec = max_acceleration ? atof(max_acceleration) : 0.25;
    default_close_profile->jerk_rate_hz_per_sec2 = jerk_rate ? atof(jerk_rate) : 0.001;
    default_close_profile->start_speed_hz = start_speed ? atof(start_speed) : 100.0;
    default_close_profile->start_acceleration_hz = start_acceleration ? atof(start_acceleration) : 0.0;
    default_close_profile->gear_ratio = gear_ratio ? atof(gear_ratio) : 99.548;
    default_close_profile->microstep_resolution = msres_default ? atoi(msres_default) : 8;
    
    // Print profiles for debugging
    motion_profile_t profiles[] = {*default_profile, *slow_profile, *default_close_profile};
    
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

    // Setup signal handlers for graceful shutdown
    setup_signal_handlers();

    //take in environment variables
    // Initialize GPIO and TMC2209 driver
    TMC2209_t driver;
    tmc_gpio_context_t gpio_ctx;
    tmc_gpio_config_t gpio_config;
    tmc_motion_s_curve_t motion;

    // Assign global pointers for signal handler access
    g_current_driver = &driver;
    g_current_gpio = &gpio_ctx;
    g_current_motion = &motion;

    log_info("Initializing GPIO...");
    gpio_config = tmc_gpio_get_default_config();
    if (!tmc_gpio_init(&gpio_ctx, &gpio_config, "gpiochip0")) {
        log_error("Failed to initialize GPIO");
        return -1;
    }
    log_info("GPIO initialized successfully");

    log_info("Initializing TMC2209...");
    TMC2209_SetDefaults(&driver);
    driver.config.current = 600;      // 600mA
    driver.config.microsteps = 8;     // Default microstep resolution
    
    if (!TMC2209_Init(&driver)) {
        log_error("Failed to initialize TMC2209");
        return -1;
    }
    log_info("TMC2209 initialized successfully");

    // Set motor current
    log_info("Setting motor current to 600mA...");
    TMC2209_SetCurrent(&driver, 600, 50);

    // Initialize S-curve motion control
    log_info("Initializing S-curve motion control...");
    if (!tmc_motion_s_curve_init(&motion, &gpio_ctx, &driver)) {
        log_error("Failed to initialize S-curve motion control");
        return -1;
    }

    // get the current position of the lightbar (stepcount.dat)
    int32_t current_mscnt = 0;
    int valid_bit = 1; // Assume valid for now
    int direction = 0; // 0 = clockwise, 1 = counter-clockwise (will be read from hardware)
    
    // Read stepcount.dat file (contains binary int32_t MSCNT value)
    int fd = open("stepcount.dat", O_RDONLY);
    if (fd != -1) {
        // Check file size
        struct stat st;
        if (fstat(fd, &st) == -1) {
            log_warn("Failed to get file stats, using default MSCNT=0");
            current_mscnt = 0;
        } else if (st.st_size != sizeof(int32_t)) {
            log_warn("File size is %ld bytes, expected %zu bytes, using default MSCNT=0", 
                    st.st_size, sizeof(int32_t));
            current_mscnt = 0;
        } else {
            // Memory map the file
            volatile int32_t *value_ptr = mmap(NULL, sizeof(int32_t), 
                                              PROT_READ, MAP_SHARED, fd, 0);
            if (value_ptr == MAP_FAILED) {
                log_warn("Failed to mmap stepcount.dat, using default MSCNT=0");
                current_mscnt = 0;
            } else {
                // Read the value
                int32_t value = *value_ptr;
                current_mscnt = value;
                log_info("Read from stepcount.dat: MSCNT=%u (decimal: %d, hex: 0x%08x)", 
                        current_mscnt, value, value);
                
                // Cleanup
                munmap((void*)value_ptr, sizeof(int32_t));
            }
        }
        close(fd);
    } else {
        log_warn("stepcount.dat not found, using default MSCNT=0");
        current_mscnt = 0;
    }

    // Read current motor direction from hardware
    direction = read_motor_direction();

    // perform checks based on value of stepcount.dat
    float target_angle_degrees = 0.0f;
    motion_profile_t *selected_profile = NULL;
    
    // Define profile ranges based on MSCNT values
    const int32_t DEFAULT_PROFILE_MAX = 537984;  // 38 deg
    const int32_t DEFAULT_PROFILE_MIN = 99104;  // 7 deg
    const int32_t SLOW_PROFILE_MAX = 601696;  // 38 deg
    const int32_t SLOW_PROFILE_MIN = 35392;  // 38 deg
    
    // Add tolerance for MAX_MSCNT comparison (within 1000 MSCNT units)
    const int32_t MAX_MSCNT_TOLERANCE = 32;
    
    // Create profile instances
    motion_profile_t default_profile, slow_profile, default_close_profile;
    extract_profiles(&default_profile, &slow_profile, &default_close_profile); // This will populate the profiles
    
    if (!valid_bit) {
        // If valid bit is not set, move to close slowly with default_close profile
        log_info("Valid bit not set, moving to close position with DEFAULT_CLOSE_PROFILE");
        selected_profile = &default_close_profile;
        target_angle_degrees = 45.0f; // Move to fully closed position
        direction = 1; // Counter-clockwise to close

    // Logic for moving closed to open
    } else if (-32 <= current_mscnt && current_mscnt <= 32) {
        // If stepcount.dat is 0, move to fully open position (clockwise)
        log_info("Current position is closed, moving to fully open with DEFAULT_PROFILE");
        selected_profile = &default_profile;

        int total_steps = ((float)MAX_MSCNT_THEORETICAL / 256.0f) * selected_profile->microstep_resolution;
        log_debug("Total steps: %d", total_steps);
        float desired_output_revolutions = (float)total_steps / (selected_profile->gear_ratio * 200.0f * (float)selected_profile->microstep_resolution);
        target_angle_degrees = roundf(desired_output_revolutions * 360.0f);
        direction = 0; // Clockwise to open
        
        log_info("Calculated target angle: %.5f degrees (to fully open)", target_angle_degrees);

    // Logic for moving open to closed
    } else if (MAX_MSCNT - MAX_MSCNT_TOLERANCE <= current_mscnt && 
                current_mscnt <= MAX_MSCNT + MAX_MSCNT_TOLERANCE) {
        // If stepcount.dat is in default profile range, move to fully closed
        log_info("Current position is out, moving to fully closed with DEFAULT_PROFILE");
        selected_profile = &default_profile;

        // Calculate remaining angle to fully close
        // desired_output_revolutions = current_mscnt / (Gear_ratio × 200 × 256)
        int total_steps = ((float)current_mscnt / 256.0f) * selected_profile->microstep_resolution;
        float desired_output_revolutions = (float)total_steps / (selected_profile->gear_ratio * 200.0f * (float)selected_profile->microstep_resolution);
        target_angle_degrees = roundf(desired_output_revolutions * 360.0f);
        direction = 1; // counter-clockwise to close
        
        log_info("Calculated target angle: %.5f degrees (to fully close)", target_angle_degrees);

    // Logic for moving between open and closed > 39 degrees 
    } else if (abs(current_mscnt) >= DEFAULT_PROFILE_MIN && abs(current_mscnt) <= DEFAULT_PROFILE_MAX) {
        // If stepcount.dat is in default profile range, use saved motor direction from file
        log_info("Current position in default range in between 7 and 38 degrees, using DEFAULT_PROFILE");
        selected_profile = &default_profile;

        // Use the direction from saved file to determine target
        if (direction == 0) {
            // Saved direction shows clockwise (opening), continue to fully open
            float desired_output_revolutions = (float)(MAX_MSCNT - abs(current_mscnt)) / (selected_profile->gear_ratio * 200.0f * 256.0f);
            target_angle_degrees = desired_output_revolutions * 360.0f;
            log_info("Saved direction is clockwise (opening), continuing to fully open, target angle: %.2f degrees", target_angle_degrees);
        } else {
            // Saved direction shows counter-clockwise (closing), continue to fully close
            float desired_output_revolutions = (float)abs(current_mscnt) / (selected_profile->gear_ratio * 200.0f * 256.0f);
            target_angle_degrees = desired_output_revolutions * 360.0f;
            log_info("Saved direction is counter-clockwise (closing), continuing to fully close, target angle: %.2f degrees", target_angle_degrees);
        }

    // make sure to account for direction of the motor
    // Logic for moving between open and closed for sections less than 7 degrees
    } else if ((current_mscnt > SLOW_PROFILE_MIN  && current_mscnt < DEFAULT_PROFILE_MIN)){
        
        log_info("Current position in slow range in between 2.5-7 degrees");
        
        // Use the direction from saved file to determine target
        // If we are going clockwise, we are opening and should continue to 45*
        if (direction == 0) {
            selected_profile = &default_profile;
            // Saved direction shows clockwise (opening), continue to fully open
            float desired_output_revolutions = (float)(MAX_MSCNT - abs(current_mscnt)) / (selected_profile->gear_ratio * 200.0f * 256.0f);
            target_angle_degrees = desired_output_revolutions * 360.0f;
            log_info("Saved direction is clockwise (opening), continuing to fully open, target angle: %.2f degrees", target_angle_degrees);

        // If we are going counter-clockwise, we are closing and should continue to 0*
        } else {
            selected_profile = &slow_profile;
            // Saved direction shows counter-clockwise (closing), continue to fully close
            float desired_output_revolutions = (float)abs(current_mscnt) / (selected_profile->gear_ratio * 200.0f * 256.0f);
            target_angle_degrees = desired_output_revolutions * 360.0f;
            log_info("Saved direction is counter-clockwise (closing), continuing to fully close, target angle: %.2f degrees", target_angle_degrees);
        }
        
    // Logic for moving between open and closed for sections greater than 38 degrees
    } else if(current_mscnt < SLOW_PROFILE_MAX && current_mscnt > DEFAULT_PROFILE_MAX){

        log_info("Current position in slow range in between 38-42.5 degrees");

        // Use the direction from saved file to determine target

        // If we are going clockwise, we are opening and should continue to 45*
        if (direction == 0) {
            selected_profile = &slow_profile;
            // Saved direction shows clockwise (opening), continue to fully open
            float desired_output_revolutions = (float)(MAX_MSCNT - abs(current_mscnt)) / (selected_profile->gear_ratio * 200.0f * 256.0f);
            target_angle_degrees = desired_output_revolutions * 360.0f;
            log_info("Saved direction is clockwise (opening), continuing to fully open, target angle: %.2f degrees", target_angle_degrees);

        // If we are going counter-clockwise, we are closing and should continue to 0*
        } else {
            selected_profile = &default_profile;
            // Saved direction shows counter-clockwise (closing), continue to fully close
            float desired_output_revolutions = (float)abs(current_mscnt) / (selected_profile->gear_ratio * 200.0f * 256.0f);
            target_angle_degrees = desired_output_revolutions * 360.0f;
            log_info("Saved direction is counter-clockwise (closing), continuing to fully close, target angle: %.2f degrees", target_angle_degrees);
        }
    } else {
        // Default to close if position is unknown or too small
        log_warn("Unknown position, defaulting to close with DEFAULT_CLOSE_PROFILE");
        selected_profile = &default_close_profile;
        target_angle_degrees = 45.0f; // Move 45 degrees in closing direction (minimum valid angle)
        direction = 1; // Counter-clockwise to close
    }

    if (selected_profile == NULL) {
        log_error("No profile selected");
        return -1;
    }

    log_info("Selected profile: %s", selected_profile->name);
    log_info("Target angle: %.2f degrees", target_angle_degrees);
    log_info("Direction: %s (value: %d)", direction ? "counter-clockwise" : "clockwise", direction);
    log_info("Current MSCNT: %u, MAX_MSCNT: %u", current_mscnt, MAX_MSCNT);

    // Start S-curve motion (extract from selected profile)
    log_info("Starting S-curve motion...");
    
    // Save the direction we're about to use
    if (!save_motor_direction(direction)) {
        log_warn("Failed to save motor direction, continuing anyway");
    }
    
    if (!tmc_motion_s_curve_start(&motion, 
                                 target_angle_degrees,
                                 selected_profile->max_speed_rpm,
                                 selected_profile->max_acceleration_hz_per_sec,
                                 selected_profile->jerk_rate_hz_per_sec2,
                                 selected_profile->start_speed_hz,
                                 selected_profile->start_acceleration_hz,
                                 selected_profile->gear_ratio,
                                 (bool)direction, // Convert int to bool (0=clockwise, 1=counter-clockwise)
                                 selected_profile->microstep_resolution)) {
        log_error("Failed to start S-curve motion");
        return -1;
    }

    // Wait for motion to complete
    log_info("Waiting for motion to complete...");
    while (!tmc_motion_s_curve_is_complete(&motion)) {
        usleep(100000); // 100ms sleep
        // Check for shutdown request
        if (g_shutdown_requested) {
            log_info("Shutdown requested, exiting motion loop.");
            break;
        }
    }

    if (g_shutdown_requested) {
        log_info("Motion interrupted by shutdown request");
        
        // Stop motion first and wait for it to complete
        if (g_current_motion && g_current_motion->motion_active) {
            log_info("Stopping active motion...");
            tmc_motion_s_curve_stop(g_current_motion);
        }
        
        // Now get the final state AFTER motion has stopped
        uint32_t current_step_count = 0;
        if (g_current_motion) {
            current_step_count = g_current_motion->steps_completed;
            log_info("Final step count after motion stopped: %u", current_step_count);
        }
        
        // Get the accumulated MSCNT delta from the global variable in tmc_monitor.c
        extern int32_t g_total_mscnt_delta;
        int32_t total_mscnt_delta = g_total_mscnt_delta;
        log_info("Final MSCNT delta after motion stopped: %d", total_mscnt_delta);
        
        // Calculate the final MSCNT position by adding delta to initial position
        // The initial position was read from stepcount.dat as current_mscnt
        int32_t final_mscnt = 0;
        if (g_current_motion && g_current_motion->position_monitor.driver) {
            // Calculate final MSCNT: initial_position + delta
            // Since we started from current_mscnt and moved by total_mscnt_delta
            final_mscnt = ((int32_t)current_mscnt + total_mscnt_delta);
            log_info("Initial MSCNT: %d, Delta: %d, Final MSCNT: %u", 
                    current_mscnt, total_mscnt_delta, final_mscnt);
        } else {
            log_warn("Position monitor not available, using step count only");
            final_mscnt = ((int32_t)current_mscnt + total_mscnt_delta);
        }
        
        // Save current position and state
        int direction = g_current_motion ? g_current_motion->direction : 0;
        int valid_bit = 1; // Assume valid since we have step count
        
        // Save the current direction before shutdown
        if (!save_motor_direction(direction)) {
            log_warn("Failed to save motor direction during shutdown");
        }
        usleep(1000); // 1ms delay to ensure the final MSCNT reading is captured
        graceful_shutdown(final_mscnt, current_step_count, direction, valid_bit, total_mscnt_delta);
        return 1; // Exit with error code to indicate interrupted
    }else

    log_info("Motion completed successfully! Assume parked open or closed");

    // Cleanup
    tmc_motion_s_curve_deinit(&motion);
    tmc_gpio_deinit(&gpio_ctx);

    // Clear global pointers
    g_current_motion = NULL;
    g_current_driver = NULL;
    g_current_gpio = NULL;

    return 0;
}