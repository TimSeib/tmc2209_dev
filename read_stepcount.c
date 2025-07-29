/*
 * read_stepcount.c - Simple script to read int32 value from stepcount.dat
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

int main() {
    // Open the file
    int fd = open("stepcount.dat", O_RDONLY);
    if (fd == -1) {
        perror("Failed to open stepcount.dat");
        return -1;
    }
    
    // Check file size
    struct stat st;
    if (fstat(fd, &st) == -1) {
        perror("Failed to get file stats");
        close(fd);
        return -1;
    }
    
    if (st.st_size != sizeof(int32_t)) {
        printf("Warning: File size is %ld bytes, expected %zu bytes\n", 
               st.st_size, sizeof(int32_t));
    }
    
    // Memory map the file
    volatile int32_t *value_ptr = mmap(NULL, sizeof(int32_t), 
                                      PROT_READ, MAP_SHARED, fd, 0);
    if (value_ptr == MAP_FAILED) {
        perror("Failed to mmap stepcount.dat");
        close(fd);
        return -1;
    }
    // Read the value
    int32_t mscnt_value = *value_ptr;
    

    // Print the value in different formats
    printf("Value from stepcount.dat:\n");
    printf("  Decimal: %d\n", mscnt_value);
    printf("  Hexadecimal: 0x%08x\n", mscnt_value);
    printf("  Unsigned: %u\n", (unsigned int)mscnt_value);
    
    // Convert to more meaningful units
    printf("\nConverted units:\n");
    printf("  MSCNT units: %d\n", mscnt_value);
    printf("  Full steps: %.3f\n", (float)mscnt_value / 256.0f);
    printf("  Microsteps (8 mres): %.3f\n", (float)mscnt_value * 8.0f / 256.0f);
    
    // Cleanup
    munmap((void*)value_ptr, sizeof(int32_t));
    close(fd);
    
    return 0;
} 