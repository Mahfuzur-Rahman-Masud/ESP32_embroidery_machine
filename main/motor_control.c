#include "driver.h"
#include "driver/ledc.h"
#include "driver/pcnt.h" // Legacy v4.4 header structure
#include "esp_timer.h"
#include "grbl/spindle_control.h"
#include "grbl/grbl.h"
#include "custom/linear_motor.h"

#define SPINDLE_PWM_GPIO SPINDLE_PWM_PIN
#define ENCODER_A_GPIO SPINDLE_ENCODER_PIN


#define ENCODER_EDGES_PER_REV      (DEFAULT_SPINDLE_PPR * 2.0f)
#define PWM_MAX_DUTY 2047


static bool motor_control_enabled = false;
static const float h = 0.001f;

static float b0 = 3500.0f; 
static float omega_o = 20.0f; 
static float omega_c = 2.5f; 

static float beta_1, beta_2, K_p;

static float z1 = 0.0f;
static float z2 = 0.0f;
static float u_cmd = 0.0f;
static float target_rpm = 0.0f;
static float current_rpm = 0.0f;
static float raw_rpm = 0.0f;

esp_timer_handle_t adrc_timer_handle = NULL;

static uint16_t print_counter = 0;
static volatile bool flag_print_data = false;

esp_timer_handle_t logging_timer_handle = NULL;

volatile uint32_t rolling_pulse_accumulator = 0;
volatile uint32_t pulse_sample_count = 0;
static float smooth_display_rpm = 0.0f;

static int rpm_change = 0;

static spindle_ptrs_t* spindle;
static spindle_data_t spindle_data;
static spindle_state_t spindle_state;
static spindle_param_t spindle_param;


void motor_control_enable(bool enabled){
    motor_control_enabled = enabled;

    #if LINEAR_MOTOR
    if(enabled){
        linear_motor_enable(false);
    }
    

    #endif 
}

spindle_data_t* spindle_get_data(spindle_data_request_t request)
{
    spindle_data.ccw = false;
    spindle_data.rpm = smooth_display_rpm;
    spindle_data.rpm_programmed = target_rpm;

    return &spindle_data;
}

void IRAM_ATTR set_modifiers()
{

    // --- DYNAMIC PARAMETER MAP ---
    if (target_rpm <= 150.0f) {
        // Zone 1: Ultra-low crawl (S100) - Already working well
        b0 = 3500.0f;
        omega_o = 20.0f;
        omega_c = 2.5f;
    } else if (target_rpm > 150.0f && target_rpm <= 750.0f) {
        // Zone 2: Mid-low range (S300/S500) - Crushing the Phase Lag Wave
        // We drop omega_c to stop the controller from amplifying the filter delay
        b0 = 4500.0f; // Increased dampening
        omega_o = 15.0f; // Slow down observer slightly to ignore the wave
        omega_c = 1.2f; // Lowered stiffness to kill the rhythmic oscillation
    } else {
        // Zone 3: High speed (S1000/S3000) - Tight, crisp tracking
        b0 = 2000.0f; // Less dampening needed due to motor momentum
        omega_o = 35.0f; // Crisp observation
        omega_c = 5.0f; // Rigid control loop
    }

    // CRITICAL: Recalculate your execution properties immediately!
    beta_1 = 2.0f * omega_o;
    beta_2 = omega_o * omega_o;
    K_p = omega_c;
}



float getRPM()
{
    return current_rpm;
}

float getTargetRPM()
{
    return target_rpm;
}

void reportRPM()
{
    char log_buffer[64];

    int32_t display_rpm_int = (int32_t)(current_rpm + 0.5f);
    int32_t target_rpm_int = (int32_t)(target_rpm + 0.5f);
    snprintf(log_buffer, sizeof(log_buffer), "#RPM: %d / %d\n", (int)display_rpm_int, (int)target_rpm_int);
    hal.stream.write(log_buffer);
}

static void spindle_logging_ticker(void* arg)
{
    static int32_t last_printed_rpm = -1;
    static int32_t last_printed_target = -1;

  
    uint32_t captured_pulses = rolling_pulse_accumulator;
    uint32_t captured_samples = pulse_sample_count;


    rolling_pulse_accumulator = 0;
    pulse_sample_count = 0;

    if (captured_samples > 0 && target_rpm > 0.0f) {
        float total_time = (float)captured_samples * h;
        float calculated_avg_rpm = ((float)captured_pulses / total_time) / ENCODER_EDGES_PER_REV * 60.0f;
        smooth_display_rpm = (smooth_display_rpm * 0.3f) + (calculated_avg_rpm * 0.7f);

    } else {
        smooth_display_rpm = 0.0f;
    }


    int32_t display_rpm_int = (int32_t)(smooth_display_rpm + 0.5f);
    int32_t target_rpm_int = (int32_t)(target_rpm + 0.5f);


    if (display_rpm_int != last_printed_rpm || target_rpm_int != last_printed_target) {

        char log_buffer[64];
        snprintf(log_buffer, sizeof(log_buffer), "RPM: %d / %d\n", (int)display_rpm_int, (int)target_rpm_int);
        hal.stream.write(log_buffer);

        last_printed_rpm = display_rpm_int;
        last_printed_target = target_rpm_int;
    }

    if (spindle && spindle->param) {
        spindle->param->rpm_overridden = smooth_display_rpm;
    }else if(spindle){
        spindle->param = &spindle_param;
        spindle->param->rpm_overridden = smooth_display_rpm;
    }
}



// Deterministic 1kHz Execution block running via hardware timer
static void IRAM_ATTR adrc_spindle_ticker(void* arg)
{

    static bool spindle_on = false;

    if (target_rpm <= 0.0f && !spindle_on) {
        return;
    }

    int16_t pulse_count = 0;

    // v4.4 Legacy Counter Read and Clear API
    pcnt_get_counter_value(PCNT_UNIT_0, &pulse_count);
    pcnt_counter_clear(PCNT_UNIT_0);

    rolling_pulse_accumulator += pulse_count;
    pulse_sample_count++;

    // current_rpm = (current_rpm * 0.85f) + (raw_rpm * 0.15f);
    // Shift slightly to allow faster tracking of low pulse counts
    current_rpm = (current_rpm * 0.80f) + (raw_rpm * 0.20f);

    print_counter++;
    if (print_counter >= 500) { // 500 ticks @ 1kHz = 500ms
        print_counter = 0;
        flag_print_data = true; // Signal the background loop to print
    }


    // Unidirectional Speed Translation
    raw_rpm = ((float)pulse_count / h) / ENCODER_EDGES_PER_REV * 60.0f;

    if (target_rpm <= 0.0f && spindle_on) {
        z1 = 0.0f;
        z2 = 0.0f;
        u_cmd = 0.0f;
        pcnt_counter_clear(PCNT_UNIT_0);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        if (current_rpm < .001f) {
            raw_rpm = 0;
            current_rpm = 0;
            spindle_on = false;
        }

        return;
    }

    if (!spindle_on) {
        pcnt_counter_clear(PCNT_UNIT_0);
    }

    spindle_on = true;

    // static bool stable_modifier_needed;
    // bool is_falling_fast = (target_rpm < (raw_rpm - 75.0f));
    // if (is_falling_fast && rpm_change == 2 && !stable_modifier_needed) {
    //         // Aggressive deceleration settings for Zone 3
    //         b0 = 4000.0f;
    //         omega_o = 50.0f;  // Ultra-fast tracking of the falling edge
    //         omega_c = 10.0f;  // Force rigid downward correction

    //         beta_1 = 2.0f * omega_o;
    //         beta_2 = omega_o * omega_o;
    //         K_p = omega_c;

    //         stable_modifier_needed = true;

    // }else if(stable_modifier_needed){
    //     stable_modifier_needed = false;
    //     set_modifiers();
    // }

    // ESO Mathematics
    float err_eso = z1 - current_rpm;
    z1 = z1 + h * (z2 + b0 * u_cmd - beta_1 * err_eso);
    z2 = z2 + h * (-beta_2 * err_eso);

    // Control Mapping
    float u0 = K_p * (target_rpm - z1);
    u_cmd = (u0 - z2) / b0;

    // Strict Single-Direction Clamping Constraints
    if (u_cmd > 1.0f)
        u_cmd = 1.0f;
    if (u_cmd < 0.0f)
        u_cmd = 0.0f;

    // Inject directly into the LEDC hardware timer registers
    uint32_t duty_val = (uint32_t)(u_cmd * (float)PWM_MAX_DUTY);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_val);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void IRAM_ATTR set_adrc_spindle_speed(float rpm)
{

    if(rpm > 0 && !motor_control_enabled){
        motor_control_enable(true);
    }

    if (rpm < 0.0f)
        rpm = 0.0f;

    if (target_rpm == rpm) {
        rpm_change = 0;
        return;
    } else if (rpm > target_rpm) {
        rpm_change = 2;
    } else {
        rpm_change = 1;
    }

    target_rpm = rpm;

    spindle_state.on = rpm > 0;
    spindle_param.rpm_overridden = rpm;

    set_modifiers();
    // reportRPM();
}

void IRAM_ATTR set_adrc_spindle_speed_ramp(float rpm, float ramp)
{
    if (rpm > target_rpm + ramp) {
        rpm = target_rpm + ramp;
    }

    set_adrc_spindle_speed(rpm);
}

static void IRAM_ATTR mc_spindle_update_rpm(spindle_ptrs_t* spindle, float rpm)
{

    set_adrc_spindle_speed(rpm);
}

static void IRAM_ATTR mc_spindle_set_state(spindle_ptrs_t* spindle, spindle_state_t state, float rpm)
{
    mc_spindle_update_rpm(spindle, rpm);
}

static void IRAM_ATTR mc_on_spindle_programmed(spindle_ptrs_t* spindle, spindle_state_t state, float rpm, spindle_rpm_mode_t mode)
{
    mc_spindle_set_state(spindle, state, rpm);
}

static spindle_state_t spindle_get_state(spindle_ptrs_t* spindle)
{
    return spindle_state;
}

void init_adrc_spindle_control(void)
{
    // Math Matrix Calculations
    beta_1 = 2.0f * omega_o;
    beta_2 = omega_o * omega_o;
    K_p = omega_c;

    // v4.4 Legacy Hardware Pulse Counter (PCNT) Configuration
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = ENCODER_A_GPIO,
        .ctrl_gpio_num = -1, // No direction channel line needed
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_0,
        .pos_mode = PCNT_COUNT_INC, // Count up on rising edges
        .neg_mode = PCNT_COUNT_INC, // Count up on falling edges (doubles resolution accuracy)
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = 32000,
        .counter_l_lim = -10,
    };

    pcnt_unit_config(&pcnt_config);
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);

    // High-Speed 25kHz LEDC Configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_11_BIT,
        .freq_hz = 25000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .gpio_num = SPINDLE_PWM_GPIO,
        .duty = 0
    };
    ledc_channel_config(&ledc_channel);

    // High-Priority Hardware Interrupt Link
    const esp_timer_create_args_t timer_args = {
        .callback = &adrc_spindle_ticker,
        .name = "adrc_spindle_timer"
    };

    esp_timer_create(&timer_args, &adrc_timer_handle);
    esp_timer_start_periodic(adrc_timer_handle, 1000); // 1ms Loop Ticker

    // ==========================================
    // TIMER 2: Your new 2Hz Telemetry Logging Loop
    // ==========================================
    const esp_timer_create_args_t logging_timer_args = {
        .callback = &spindle_logging_ticker,
        .name = "spindle_logging_timer"
    };
    esp_timer_create(&logging_timer_args, &logging_timer_handle);
    esp_timer_start_periodic(logging_timer_handle, 500000);

    // grbl.on_spindle_programmed = mc_on_spindle_programmed;
    // spindle_id_t default_id = spindle_get_default();
    spindle = spindle_get_hal(0, SpindleHAL_Configured);

    // if (default_id >= 0) {
    //     ///
    // }

    if (spindle) {
        hal.stream.write("spindle found\n");
        spindle->set_state = mc_spindle_set_state;
        spindle->update_rpm = mc_spindle_update_rpm;
        spindle->rpm_max = RPM_MAX;
        spindle->rpm_min = RPM_MIN;
        spindle->get_data = spindle_get_data;
        spindle->cap.variable = true;

        spindle->get_state = spindle_get_state;
    } else {
        hal.stream.write("spindle not found\n");
    }
}


