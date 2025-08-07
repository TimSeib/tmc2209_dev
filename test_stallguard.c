/*
 * test_stallguard_simple.c - Simple StallGuard test with direct motor control
 *
 * This program provides direct motor control and real-time StallGuard monitoring
 * without the complexity of S-curve motion profiles.
 *
 * v1.0.0 / 2024-12-19
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <unistd.h>
 #include <signal.h>
 #include <pthread.h>
 #include <time.h>
 #include "tmc2209.h"
 #include "tmc_gpio.h"
 #include "tmc_stallguard.h"
 #include "log.h"
 
 // Global variables
 static volatile bool g_running = true;
 static volatile bool g_motor_running = false;
 static TMC2209_t g_driver;
 static tmc_gpio_context_t g_gpio_ctx;
 
 // Motor control thread variables
 static pthread_t g_motor_thread;
 static pthread_mutex_t g_motor_mutex = PTHREAD_MUTEX_INITIALIZER;
 static uint32_t g_target_speed_hz = 1000;  // Target speed in Hz
 static uint32_t g_step_delay_us = 500;     // Step delay in microseconds
 
 // StallGuard monitoring variables
 static volatile bool g_stall_detected = false;
 static volatile uint32_t g_stall_pulse_count = 0;
 
 // DIAG pin interrupt callback for StallGuard monitoring
 static void diag_stall_callback(uint8_t pin, bool rising_edge) {
     (void)pin; // Suppress unused parameter warning
     
     if (rising_edge) {
         g_stall_pulse_count++;
         printf("\n*** STALL DETECTED! DIAG pin went HIGH (pulse #%u) ***\n", g_stall_pulse_count);
         printf("Shutting down program immediately...\n");
         
         // Set stall detected flag
         g_stall_detected = true;
         
         // Trigger shutdown
         g_running = false;
     }
 }
 
 // Signal handler for graceful shutdown
 static void signal_handler(int sig) {
     (void)sig;
     g_running = false;
     printf("\nShutdown requested...\n");
     
     // Stop motor
     pthread_mutex_lock(&g_motor_mutex);
     g_motor_running = false;
     pthread_mutex_unlock(&g_motor_mutex);
     
     // Disable motor
     tmc_gpio_enable_driver(&g_gpio_ctx, false);
 }
 
 // Simple motor control thread
 static void* motor_control_thread(void* arg) {
     (void)arg;
     
     printf("Motor control thread started\n");
     
     while (g_running) {
         pthread_mutex_lock(&g_motor_mutex);
         bool should_run = g_motor_running;
         pthread_mutex_unlock(&g_motor_mutex);
         
         if (should_run) {
             // Generate step pulse
             tmc_gpio_write(&g_gpio_ctx, TMC_GPIO_STEP_PIN, TMC_GPIO_HIGH);
             usleep(1);  // 1us pulse width
             tmc_gpio_write(&g_gpio_ctx, TMC_GPIO_STEP_PIN, TMC_GPIO_LOW);
             
             // Wait for next step
             usleep(g_step_delay_us);
         } else {
             usleep(1000);  // 1ms when not running
         }
     }
     
     printf("Motor control thread stopped\n");
     return NULL;
 }
 
 // Function to set motor speed
 static void set_motor_speed(uint32_t speed_hz) {
     if (speed_hz == 0) {
         g_step_delay_us = 0;
     } else {
         g_step_delay_us = 1000000 / speed_hz;  // Convert Hz to microseconds
     }
     g_target_speed_hz = speed_hz;
     printf("Motor speed set to %u Hz (delay: %u us)\n", speed_hz, g_step_delay_us);
 }
 
 // Function to start motor
 static void start_motor(void) {
     pthread_mutex_lock(&g_motor_mutex);
     g_motor_running = true;
     pthread_mutex_unlock(&g_motor_mutex);
     printf("Motor started\n");
 }
 
 // Function to stop motor
 static void stop_motor(void) {
     pthread_mutex_lock(&g_motor_mutex);
     g_motor_running = false;
     pthread_mutex_unlock(&g_motor_mutex);
     printf("Motor stopped\n");
 }
 
 // Function to monitor StallGuard registers in real-time
 static void monitor_stallguard_registers(uint32_t duration_seconds) {
     printf("=== StallGuard Register Monitoring ===\n");
     printf("Monitoring for %u seconds...\n", duration_seconds);
     printf("Note: TSTEP=1048575 means standstill, SG_RESULT=0 when stopped\n");
     printf("      Lower SG_RESULT = closer to stall, DIAG pulses when SG_RESULT < SGTHRS\n");
     printf("      TSTEP counts down from 1048575, Speed = 16777216 / (TSTEP + 1)\n");
     printf("Time(s) | TSTEP | SG_RESULT | DIAG_Pinv |\n");
     printf("--------|-------|-----------|-----------|\n");
     
     uint64_t start_time = 0;
     struct timespec ts;
     clock_gettime(CLOCK_MONOTONIC, &ts);
     start_time = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
 
     while (g_running) {
         // Get current time
         clock_gettime(CLOCK_MONOTONIC, &ts);
         uint64_t current_time = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
         uint32_t elapsed_seconds = (uint32_t)((current_time - start_time) / 1000000ULL);
         
         if (elapsed_seconds >= duration_seconds) {
             break;
         }
         
         // Read TSTEP (actual step velocity)
         TMC2209_tstep_dgr_t tstep;
         memset(&tstep, 0, sizeof(TMC2209_tstep_dgr_t));
         tstep.addr.reg = TMC2209Reg_TSTEP;
         TMC2209_ReadRegister(&g_driver, (TMC2209_datagram_t *)&tstep);
         
         // Read SG_RESULT
         uint32_t sg_result = tmc_stallguard_get_result(&g_driver);
         
         // Read DIAG pin
         int diag_state = tmc_gpio_read(&g_gpio_ctx, TMC_GPIO_DIAG_PIN);
         
         printf("%7u | %5u | %9u | %8s |\n", 
                elapsed_seconds,
                tstep.reg.tstep,
                sg_result,
                diag_state ? "HIGH" : "LOW");
         
         usleep(100000);  // 100ms update rate
     }
     
     printf("Monitoring completed\n");
 }

 // Function to monitor StallGuard trigger and shutdown on DIAG high using GPIO events
 static void monitor_stallguard_trigger(uint32_t duration_seconds) {
     printf("=== StallGuard Trigger Monitoring (GPIO Event Mode) ===\n");
     printf("Monitoring for %u seconds or until DIAG pin goes HIGH...\n", duration_seconds);
     printf("Using GPIO event monitoring thread for real-time stall detection\n");
     printf("Program will shutdown immediately when DIAG pin goes HIGH (stall detected)\n");
     printf("Time(s) | TSTEP | SG_RESULT | DIAG_Pinv | Status\n");
     printf("--------|-------|-----------|-----------|--------\n");
     
     // Reset stall detection flags
     g_stall_detected = false;
     g_stall_pulse_count = 0;
     
     // Setup DIAG pin interrupt for StallGuard monitoring
     if (!tmc_gpio_setup_diag_interrupt(&g_gpio_ctx, diag_stall_callback)) {
         printf("ERROR: Failed to setup DIAG pin interrupt\n");
         return;
     }
     
     // Start GPIO event monitoring
     if (!tmc_gpio_start_event_monitoring(&g_gpio_ctx)) {
         printf("ERROR: Failed to start GPIO event monitoring\n");
         return;
     }
     
     printf("GPIO event monitoring started - waiting for stall detection...\n");
     
     uint64_t start_time = 0;
     struct timespec ts;
     clock_gettime(CLOCK_MONOTONIC, &ts);
     start_time = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
 
     while (g_running) {
         // Get current time
         clock_gettime(CLOCK_MONOTONIC, &ts);
         uint64_t current_time = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
         uint32_t elapsed_seconds = (uint32_t)((current_time - start_time) / 1000000ULL);
         
         if (elapsed_seconds >= duration_seconds) {
             printf("Monitoring completed - no stall detected\n");
             break;
         }
         
         // Check if stall was detected by the callback
         if (g_stall_detected) {
             printf("Stall detection confirmed by GPIO event monitoring\n");
             break;
         }
         
         // Read TSTEP (actual step velocity)
         TMC2209_tstep_dgr_t tstep;
         memset(&tstep, 0, sizeof(TMC2209_tstep_dgr_t));
         tstep.addr.reg = TMC2209Reg_TSTEP;
         TMC2209_ReadRegister(&g_driver, (TMC2209_datagram_t *)&tstep);
         
         // Read SG_RESULT
         uint32_t sg_result = tmc_stallguard_get_result(&g_driver);
         
         // Read DIAG pin (for display purposes only)
         int diag_state = tmc_gpio_read(&g_gpio_ctx, TMC_GPIO_DIAG_PIN);
         
         printf("%7u | %5u | %9u | %8s | %s\n", 
                elapsed_seconds,
                tstep.reg.tstep,
                sg_result,
                diag_state ? "HIGH" : "LOW",
                g_stall_detected ? "*** STALL DETECTED ***" : "OK");
         
         usleep(100000);  // 100ms update rate
     }
     
     // Stop GPIO event monitoring
     tmc_gpio_stop_event_monitoring(&g_gpio_ctx);
     printf("GPIO event monitoring stopped\n");
 }

 
 int main(int argc, char *argv[]) {
     (void)argc;  // Suppress unused parameter warning
     (void)argv;  // Suppress unused parameter warning
     
     // Setup signal handler
     signal(SIGINT, signal_handler);
     signal(SIGTERM, signal_handler);
     
     int log_level;
     printf("Enter log level (0=Trace, 1=Debug, 2=Info, 3=Warn, 4=Error, 5=Fatal): ");
     if (scanf("%d", &log_level) != 1) {
         printf("ERROR: Invalid log level\n");
         return -1;
     }
     log_set_level(log_level);
 
     printf("TMC2209 Simple StallGuard Test Program\n");
     printf("=====================================\n\n");
     
     // Initialize GPIO
     printf("Initializing GPIO...\n");
     tmc_gpio_config_t gpio_config = tmc_gpio_get_default_config();
     if (!tmc_gpio_init(&g_gpio_ctx, &gpio_config, "gpiochip0")) {
         printf("Failed to initialize GPIO\n");
         return -1;
     }
     
     // Initialize TMC2209
     printf("Initializing TMC2209...\n");
     TMC2209_SetDefaults(&g_driver);
     g_driver.config.current = 600;
     g_driver.config.microsteps = 8;
     
     if (!TMC2209_Init(&g_driver)) {
         printf("Failed to initialize TMC2209\n");
         tmc_gpio_deinit(&g_gpio_ctx);
         return -1;
     }
     // Set motor current
     TMC2209_SetCurrent(&g_driver, 600, 50);
     
     // Enable motor
     tmc_gpio_enable_driver(&g_gpio_ctx, true);
     
     // Debug: Check StallGuard configuration
     printf("Checking StallGuard configuration...\n");
     tmc_stallguard_debug_status(&g_driver);
     
     // Create motor control thread
     if (pthread_create(&g_motor_thread, NULL, motor_control_thread, NULL) != 0) {
         printf("Failed to create motor control thread\n");
         tmc_gpio_deinit(&g_gpio_ctx);
         return -1;
     }
     // Show menu
     printf("\nSelect test mode:\n");
     printf("1. Monitor StallGuard registers\n");
     printf("2. Monitor StallGuard trigger and shutdown on DIAG high\n");
     printf("3. Exit\n");
     printf("Enter choice (1-3): ");
     
     int choice;
     if (scanf("%d", &choice) != 1) {
         choice = 5;
     }
     
     printf("\n");
     
     switch (choice) {
         case 1: {
             uint32_t duration;
             uint32_t speed;
             printf("Enter monitoring duration (seconds): ");
             if (scanf("%u", &duration) != 1) {
                 duration = 30;
             }
             printf("Enter motor speed (Hz, 0=stopped): ");
             if (scanf("%u", &speed) != 1) {
                 speed = 500;
             }
             
             if (speed > 0) {
                 set_motor_speed(speed);
                 start_motor();
                 printf("Motor started at %u Hz for monitoring\n", speed);
             } else {
                 printf("Motor will remain stopped during monitoring\n");
             }
             
             monitor_stallguard_registers(duration);
             
             if (speed > 0) {
                 stop_motor();
             }
             break;
         }
         case 2: {
             uint32_t duration;
             uint32_t speed;
             printf("Enter monitoring duration (seconds): ");
             if (scanf("%u", &duration) != 1) {
                 duration = 30;
             }
             printf("Enter motor speed (Hz, 0=stopped): ");
             if (scanf("%u", &speed) != 1) {
                 speed = 500;
             }
             
             if (speed > 0) {
                 set_motor_speed(speed);
                 start_motor();
                 printf("Motor started at %u Hz for trigger monitoring\n", speed);
             } else {
                 printf("Motor will remain stopped during trigger monitoring\n");
             }
             
             monitor_stallguard_trigger(duration);
             
             if (speed > 0) {
                 stop_motor();
             }
             break;
         }
         case 3:
         default:
             printf("Exiting...\n");
             break;
     }
     
     // Check if shutdown was requested during test execution
     if (!g_running) {
         printf("Shutdown completed.\n");
     }
     
     // Cleanup
     printf("Cleaning up...\n");
     
     // Stop motor and wait for thread
     stop_motor();
     printf("trying to join\n");
     pthread_join(g_motor_thread, NULL);
     printf("motor thread joined\n");
     
     // Stop GPIO event monitoring if active
     if (tmc_gpio_is_event_monitoring_active(&g_gpio_ctx)) {
         printf("stopping GPIO event monitoring\n");
         tmc_gpio_stop_event_monitoring(&g_gpio_ctx);
     }
     
     // Disable StallGuard
     tmc_stallguard_deinit(&g_driver);
     printf("stallguard deinit\n");
     // Disable motor
     tmc_gpio_enable_driver(&g_gpio_ctx, false);
     tmc_gpio_deinit(&g_gpio_ctx);
     
     printf("Test completed.\n");
     return 0;
 } 