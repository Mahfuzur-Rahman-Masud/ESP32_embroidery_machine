#include "linear_motor.h"
#include "driver/ledc.h"
#include "esp_vfs_fat.h" // Replace with spiffs/littlefs header based on your VFS mounting
#include "grbl/grbl.h"
#include <stdio.h>
#include <string.h>
#include "motor_control.h"

#ifdef LINEAR_MOTOR

#define CONFIG_FILE_PATH "/littlefs/l_motor.json"
// #define PWM_PIN_LINEAR_MOTOR GPIO_NUM_18  // Change to your designated safe pin
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_RESOLUTION LEDC_TIMER_10_BIT // 10-bit gives duty cycles from 0 to 1023

// Define the configuration structure
typedef struct {
    int16_t trim_pwm;
    int16_t trim_delay;
    int16_t idle_pwm;
    int16_t min_rpm;
    int16_t max_rpm;
    int16_t rpm[36];
    int16_t pwm[36];
} linear_motor_config_t;

static linear_motor_config_t config;

// Forward Declarations
static void load_or_create_json_config(void);
static void setup_hardware_pwm(uint32_t freq_hz);
static void write_hardware_pwm(int16_t duty);

static volatile int16_t rpm_set = 0; 

IRAM_ATTR void linear_motor_set_rpm_ramp(int16_t target_rpm, int16_t ramp)
{

    // Check if we need to ramp up
    if (target_rpm > rpm_set + ramp) {
        linear_motor_set_rpm(rpm_set + ramp);
    } 
    else  {
        linear_motor_set_rpm(target_rpm);
    } 

}

IRAM_ATTR int16_t get_rpm()
{
    return rpm_set;
}

void report_rpm()
{
    printf("RPM: %d\n", rpm_set);
}



IRAM_ATTR void linear_motor_set_rpm(int16_t target_rpm)
{

     rpm_set = target_rpm;

    int16_t target_pwm = config.idle_pwm;

    // Condition 1 & 2: Handle boundaries before entering the loop
    if (target_rpm <= 0) {
        target_pwm = config.idle_pwm;
    } else if (target_rpm < config.min_rpm) {
        target_pwm = config.pwm[0]; // Clamp to lowest configured step
    } else {
        for (int i = 1; i < 36; i++) {

            // Safety Trap: If an array entry is 0, we've hit uninitialized data.
            // Break out and default to safe maximum power.
            if (config.rpm[i] == 0) {
                target_pwm = config.pwm[i - 1]; // Max PWM array limit
                break;
            }

            // Exact Match Optimization: If target perfectly hits a data point
            if (config.rpm[i] == target_rpm) {
                target_pwm = config.pwm[i];
                break;
            }

            // Piecewise Interpolation: Target falls cleanly between index [i-1] and [i]
            if (target_rpm > config.rpm[i - 1] && target_rpm < config.rpm[i]) {
                int16_t r0 = config.rpm[i - 1]; // Lower bounding RPM
                int16_t r1 = config.rpm[i]; // Upper bounding RPM
                int16_t p0 = config.pwm[i - 1]; // Lower bounding PWM
                int16_t p1 = config.pwm[i]; // Upper bounding PWM

                // Fixed-point linear interpolation formula
                target_pwm = p0 + ((target_rpm - r0) * (p1 - p0)) / (r1 - r0);
                break;
            }
        }
    }

    // Output calculated PWM step directly to LEDC hardware registers
    write_hardware_pwm(target_pwm);
}

// Function to enforce trim calibration
void linear_motor_trim(void)
{
    write_hardware_pwm(config.trim_pwm);

    vTaskDelay(config.trim_delay);
    write_hardware_pwm(config.idle_pwm);
}

// Hardware-level abstraction wrapper for ESP32-S3
static void setup_hardware_pwm(uint32_t freq_hz)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz = freq_hz,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PWM_PIN_LINEAR_MOTOR,
        .duty = 0, // Start completely off
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

static void write_hardware_pwm(int16_t duty)
{
    // Enforce 10-bit safety constraints (0 to 1023 duty cycle)
    if (duty < 0)
        duty = 0;
    if (duty > 1023)
        duty = 1023;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);
}

static void load_or_create_json_config(void)
{
    // 1. Open the file pointer using grblHAL's custom VFS abstraction layer
    char* resolved_path = vfs_fixpath(CONFIG_FILE_PATH);
    vfs_file_t* f = vfs_open(resolved_path, "r");
    
    if (f == NULL) {
        printf("Linear motor: Config file is not found\n");
        // File doesn't exist! Populate default values structurally
        config.trim_pwm   = 100;
        config.trim_delay = 5;     // 5 milliseconds
        config.idle_pwm   = 200;
        config.min_rpm    = 200;
        config.max_rpm    = 1200;

        // Clear all data profiles to 0
        memset(config.rpm, 0, sizeof(config.rpm));
        memset(config.pwm, 0, sizeof(config.pwm));

        // Assign explicit constraints onto only the first two elements
        config.rpm[0] = config.min_rpm; // Index [0] = 200
        config.pwm[0] = 400;            // Random minimum baseline PWM output

        config.rpm[1] = config.max_rpm; // Index [1] = 1200
        config.pwm[1] = 1023;           // ESP32 Maximum hardware PWM (Assuming 10-bit resolution)

        // Save defaults using grblHAL VFS writing utilities
        printf("Linear motor: writing config file..\n");
        f = vfs_open(resolved_path, "w");
        if (f != NULL) {
            char write_buf[128];

            // Write standard config header blocks
            snprintf(write_buf, sizeof(write_buf), "{\n  \"trim_pwm\": %hd,\n  \"trim_delay\": %hd,\n  \"idle_pwm\": %hd,\n", 
                     config.trim_pwm, config.trim_delay, config.idle_pwm);
            vfs_puts(write_buf, f);

            snprintf(write_buf, sizeof(write_buf), "  \"min_rpm\": %hd,\n  \"max_rpm\": %hd,\n", 
                     config.min_rpm, config.max_rpm);
            vfs_puts(write_buf, f);
            
            // Build and stream the RPM array string block
            vfs_puts("  \"rpm\": [", f);
            for (int i = 0; i < 36; i++) {
                snprintf(write_buf, sizeof(write_buf), "%hd%s", config.rpm[i], i == 35 ? "" : ",");
                vfs_puts(write_buf, f);
            }
            vfs_puts("],\n", f);
                
            // Build and stream the PWM array string block
            vfs_puts("  \"pwm\": [", f);
            for (int i = 0; i < 36; i++) {
                snprintf(write_buf, sizeof(write_buf), "%hd%s", config.pwm[i], i == 35 ? "" : ",");
                vfs_puts(write_buf, f);
            }
            vfs_puts("]\n}", f);
                
            // Close the stream using the custom wrapper function
            vfs_close(f);
        }
    } else {
        printf("Linear motor: found the config file\n");
        // File exists! Read and parse the custom parameter inputs using grblHAL VFS read mechanisms
        char buf[2048];
        
        // grblHAL's vfs_read operates like standard fread
        size_t bytes_read = vfs_read(buf, 1, sizeof(buf) - 1, f);
        buf[bytes_read] = '\0';
        vfs_close(f); // Close immediately after pulling storage buffer to RAM

        char* ptr;
        if ((ptr = strstr(buf, "\"trim_pwm\"")))
            sscanf(ptr, "\"trim_pwm\": %hd", &config.trim_pwm);
        if ((ptr = strstr(buf, "\"trim_delay\"")))
            sscanf(ptr, "\"trim_delay\": %hd", &config.trim_delay);
        if ((ptr = strstr(buf, "\"idle_pwm\"")))
            sscanf(ptr, "\"idle_pwm\": %hd", &config.idle_pwm);
        if ((ptr = strstr(buf, "\"min_rpm\"")))
            sscanf(ptr, "\"min_rpm\": %hd", &config.min_rpm);
        if ((ptr = strstr(buf, "\"max_rpm\"")))
            sscanf(ptr, "\"max_rpm\": %hd", &config.max_rpm);

        // Parse array entries accurately
        char* rpm_ptr = strstr(buf, "\"rpm\"");
        if (rpm_ptr) {
            rpm_ptr = strchr(rpm_ptr, '[');
            if (rpm_ptr) {
                rpm_ptr++;
                for (int i = 0; i < 36; i++) {
                    config.rpm[i] = (int16_t)strtol(rpm_ptr, &rpm_ptr, 10);
                    while (*rpm_ptr == ',' || *rpm_ptr == ' ' || *rpm_ptr == '\n')
                        rpm_ptr++;
                }
            }
        }

        char* pwm_ptr = strstr(buf, "\"pwm\"");
        if (pwm_ptr) {
            pwm_ptr = strchr(pwm_ptr, '[');
            if (pwm_ptr) {
                pwm_ptr++;
                for (int i = 0; i < 36; i++) {
                    config.pwm[i] = (int16_t)strtol(pwm_ptr, &pwm_ptr, 10);
                    while (*pwm_ptr == ',' || *pwm_ptr == ' ' || *pwm_ptr == '\n')
                        pwm_ptr++;
                }
            }
        }
    }
}

// Initialize the plugin
void linear_motor_init(void)
{
    // 1. Load data from LittleFS or write defaults if file is missing
    load_or_create_json_config();

    // 2. Initialize Hardware PWM at 5kHz using ESP32-S3 LEDC
    setup_hardware_pwm(5000);

    // 3. Set to starting state (idle)
    linear_motor_set_rpm(0);

    printf("Linear motor initialized\n");

}

#endif