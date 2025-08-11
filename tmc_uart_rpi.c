// tmc_uart_rpi.c
//
// This file contains the implementation of the UART interface for the TMC2209 motor driver on our Raspberry Pi CM4
// It sets up the UART interface and provides the functions to write and read data to and from the TMC2209 motor driver.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <time.h>

// ioctl is designed to interact with device drivers, allowing applications to send commands
// and data to control hardware devices or access device-specific features. 
#include <sys/ioctl.h>
#include "tmc2209.h"


#define UART_DEVICE "/dev/ttyAMA5"  // UART5 on CM4 change to 3 for next board
#define UART_BAUDRATE B115200 // 115200 baud rate

static int uart_fd = -1;

// Initialize UART using termios library
static bool uart_init(void) {
    struct termios tty;
    
    // Open UART device
    uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_SYNC);
    if (uart_fd < 0) {
        perror("Failed to open UART device");
        return false;
    }
    
    // Configure UART settings
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(uart_fd, &tty) != 0) {
        perror("tcgetattr failed");
        close(uart_fd);
        return false;
    }
    
    // Set baud rate
    cfsetospeed(&tty, UART_BAUDRATE);
    cfsetispeed(&tty, UART_BAUDRATE);
    
    // Configure 8N1 (8 data bits, no parity, 1 stop bit)
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CREAD | CLOCAL;
    
    // Configure input modes
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Configure output modes
    tty.c_oflag &= ~OPOST;
    
    // Configure local modes
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Set timeouts
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;  // 1 second timeout
    
    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr failed");
        close(uart_fd);
        return false;
    }
    
    return true;
}

// Override weak UART functions
void tmc_uart_write(trinamic_motor_t driver, TMC_uart_write_datagram_t *datagram) {
    if (uart_fd < 0) {
        if (!uart_init()) {
            return;
        }
    }
    
    // Set the slave address from the driver configuration
    datagram->msg.slave = driver.address;
    
    // Flush input and output buffers before sending
    tcflush(uart_fd, TCIOFLUSH);
    
    // Send datagram
    ssize_t written = write(uart_fd, datagram->data, sizeof(TMC_uart_write_datagram_t));
    if (written != sizeof(TMC_uart_write_datagram_t)) {
        perror("UART write failed");
    }
    
    usleep(1000);
}

TMC_uart_write_datagram_t *tmc_uart_read(trinamic_motor_t driver, TMC_uart_read_datagram_t *datagram) {
    static TMC_uart_write_datagram_t response;
    uint8_t raw_response[12];  // TMC2209 sends 12 bytes
    
    if (uart_fd < 0) {
        if (!uart_init()) {
            fprintf(stderr, "tmc_uart_read: Failed to initialize UART\n");
            return NULL;
        }
    }
    
    // Set the slave address from the driver configuration
    datagram->msg.slave = driver.address;
    
    // Flush input and output buffers before sending command
    tcflush(uart_fd, TCIOFLUSH);
    
    // Send read command
    ssize_t written = write(uart_fd, datagram->data, sizeof(TMC_uart_read_datagram_t));
    if (written != sizeof(TMC_uart_read_datagram_t)) {
        fprintf(stderr, "tmc_uart_read: UART read command failed - expected %zu bytes, wrote %zd bytes\n", 
                sizeof(TMC_uart_read_datagram_t), written);
        if (written < 0) {
            perror("tmc_uart_read: write error");
        }
        return NULL;
    }
    
    // Adaptive polling: start aggressive, back off if needed
    int max_attempts = 400;  // 20ms total timeout
    int attempt = 0;
    ssize_t total_bytes_read = 0;
    clock_t start_time = clock();
    
    while (attempt < max_attempts && total_bytes_read < 12) {
        ssize_t bytes_read = read(uart_fd, &raw_response[total_bytes_read], 12 - total_bytes_read);
        
        if (bytes_read > 0) {
            total_bytes_read += bytes_read;
            if (total_bytes_read == 12) {
                break; // Got all bytes, exit early
            }
        } else if (bytes_read == 0) {
            // No data available yet - adaptive polling interval
            int poll_interval_us;
            if (attempt < 100) {
                poll_interval_us = 25;  // Aggressive: 25μs for first 2.5ms
            } else if (attempt < 200) {
                poll_interval_us = 50;  // Moderate: 50μs for next 5ms
            } else {
                poll_interval_us = 100; // Conservative: 100μs for remaining time
            }
            usleep(poll_interval_us);
        } else if (bytes_read < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Would block - use same adaptive polling
                int poll_interval_us;
                if (attempt < 100) {
                    poll_interval_us = 25;
                } else if (attempt < 200) {
                    poll_interval_us = 50;
                } else {
                    poll_interval_us = 100;
                }
                usleep(poll_interval_us);
            } else {
                fprintf(stderr, "tmc_uart_read: UART read error on attempt %d - errno: %d (%s)\n", 
                        attempt, errno, strerror(errno));
                perror("tmc_uart_read: read error");
                return NULL;
            }
        }
        
        attempt++;
    }
    
    clock_t end_time = clock();
    double elapsed_ms = ((double)(end_time - start_time)) / CLOCKS_PER_SEC * 1000.0;
    
    if (total_bytes_read != 12) {
        fprintf(stderr, "tmc_uart_read: TIMEOUT - Expected 12 bytes, got %zd after %d attempts (%.2f ms)\n", 
                total_bytes_read, attempt, elapsed_ms);
        
        // Log what we did receive
        if (total_bytes_read > 0) {
            fprintf(stderr, "tmc_uart_read: Partial response received (%zd bytes): ", total_bytes_read);
            for (int i = 0; i < total_bytes_read; i++) {
                fprintf(stderr, "%02X ", raw_response[i]);
            }
            fprintf(stderr, "\n");
        }
        
        // Check if we got any echo bytes (first 4 bytes should echo the command)
        if (total_bytes_read >= 4) {
            fprintf(stderr, "tmc_uart_read: Echo bytes: ");
            for (int i = 0; i < 4; i++) {
                fprintf(stderr, "%02X ", raw_response[i]);
            }
            fprintf(stderr, "\n");
        }
        
        return NULL;
    }
    
    // Flush any remaining data in the input buffer
    tcflush(uart_fd, TCIFLUSH);
    
    // Extract the actual 8-byte response from bytes 4-11
    // The TMC2209 sends: [echo of command (4 bytes)] + [actual response (8 bytes)]
    memcpy(response.data, &raw_response[4], 8);
    
    return &response;
}