#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
// #include "driver/pulse_cnt.h"
#include "driver/pcnt.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "ADRC_MOTOR";

// --- Hardware Assignments ---
#define ENCODER_A_GPIO      6       // Connect to MT6701 OutA (Incremental pin)
#define MOTOR_PWM_GPIO      4       // Connect to Single-Direction Gate/Driver Pin

// --- Motor & Sensor Constants ---
#define ENCODER_EDGES_PER_REV  4096.0f  // MT6701 Max Quadrature/Edge Resolution
#define PWM_MAX_DUTY           8191    // 13-bit PWM depth (0 to 8191)

// --- ADRC Settings (Tuning Variables) ---
const float h = 0.001f;        // 1 ms (1000 Hz) loop interval
const float b0 = 400.0f;       // Control gain scaling factor (Adjust to system physical scale)
const float omega_o = 45.0f;   // Observer Bandwidth (Lowered to account for missing negative brake)
const float omega_c = 10.0f;   // Controller Bandwidth

// Calculated ADRC Gains
static float beta_1, beta_2, K_p;

// --- ADRC States & Targets ---
static float z1 = 0.0f;        // Speed observer estimate (Filtered RPM)
static float z2 = 0.0f;        // Disturbance observer estimate
static float u_cmd = 0.0f;     // Calculated control output command
static float target_rpm = 1200.0f; // Speed setpoint
static float current_rpm = 0.0f;

// --- Drivers and Hardware Handles ---
pcnt_unit_handle_t pcnt_unit = NULL;
esp_timer_handle_t adrc_timer = NULL;

// --- High-Speed Deterministic ADRC Execution Callback ---
static void adrc_loop_callback(void* arg) {
    int pulse_count = 0;
    // 1. Fetch count from Hardware Pulse Counter Peripheral and reset instantly
    pcnt_unit_get_count(pcnt_unit, &pulse_count);
    pcnt_unit_clear_count(pcnt_unit);

    // 2. Convert Raw Edge Ticks to Physical Velocity (RPM)
    // Formula: (ticks / interval_seconds) / ticks_per_rev * 60 seconds
    current_rpm = ((float)pulse_count / h) / ENCODER_EDGES_PER_REV * 60.0f;

    // 3. 1st-Order Extended State Observer (ESO) Execution
    float err_eso = z1 - current_rpm;
    z1 = z1 + h * (z2 + b0 * u_cmd - beta_1 * err_eso);
    z2 = z2 + h * (-beta_2 * err_eso);

    // 4. Control Law (Proportional Tracking Block)
    float u0 = K_p * (target_rpm - z1);
    u_cmd = (u0 - z2) / b0;

    // 5. Strict Unidirectional Clamping Constraints
    // Motor cannot reverse (No negative voltage/braking authority allowed)
    if (u_cmd > 1.0f)  u_cmd = 1.0f;   // Max 100% duty cycle
    if (u_cmd < 0.0f)  u_cmd = 0.0f;   // Lower limit bounded strictly at zero (coasting)

    // 6. Push Output Command straight to LEDC Hardware Registers
    uint32_t duty_register_val = (uint32_t)(u_cmd * (float)PWM_MAX_DUTY);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_register_val);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void app_main(void) {
    // Math Initialization
    beta_1 = 2.0f * omega_o;
    beta_2 = omega_o * omega_o;
    K_p = omega_c;

    ESP_LOGI(TAG, "Initializing Single-Direction ADRC Loop Hardware...");

    // --- Configure Unidirectional Hardware Pulse Counter (PCNT) ---
    pcnt_unit_config_t unit_config = {
        .high_limit = 30000,
        .low_limit = -10,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = ENCODER_A_GPIO,
        .level_gpio_num = -1, // No direction pin used, locked forward
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    // Configure channel to increment on both rising and falling edges for max accuracy
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_LIGHT_EDGE_ACTION_INCREASE, PCNT_LIGHT_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    // --- Configure LEDC (PWM Driver Engine) ---
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT, // 0 to 8191 dynamic resolution
        .freq_hz          = 25000,             // Ultra-sonic 25kHz switching frequency (removes motor whining noise)
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_PWM_GPIO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // --- Setup High-Priority Periodic Microsecond Hardware Timer ---
    const esp_timer_create_args_t adrc_timer_args = {
        .callback = &adrc_loop_callback,
        .name = "adrc_math_ticker"
    };
    ESP_ERROR_CHECK(esp_timer_create(&adrc_timer_args, &adrc_timer));
    // Trigger every 1000 microseconds (1 millisecond execution frequency)
    ESP_ERROR_CHECK(esp_timer_start_periodic(adrc_timer, 1000));

    // Low priority diagnostic task running on main core
    while (1) {
        printf("Set:%7.1f | Real:%7.1f | Duty:%5.1f%% | Disturbance_Est:%7.1f\n", 
               target_rpm, current_rpm, u_cmd * 100.0f, z2);
        vTaskDelay(pdMS_TO_TICKS(100)); // Print data telemetry frame every 100ms
    }
}
