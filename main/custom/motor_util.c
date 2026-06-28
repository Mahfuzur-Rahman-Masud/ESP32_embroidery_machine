#include "motor_util.h"
#include "driver.h"
#include "driver/pcnt.h" // Legacy v4.4 header structure
#include "grbl/grbl.h"




static float motor_util_target_rpm = 0.0f;
static float motor_util_display_rpm = 0.0f;
static bool motor_util_motor_enabled = false;
extern volatile uint32_t lm_pulses;
extern volatile uint32_t lm_samples;
extern volatile float lm_raw_rpm;
extern volatile float lm_current_rpm;
// static bool motor_util_motor_on = false;

esp_timer_handle_t motor_util_logging_timer_handle = NULL;


void motor_util_enable(bool on){
    printf("motor_util_enable: %u\n", on);
    motor_util_motor_enabled = on;
}

static void motor_util_logging_ticker(void* arg)
{
    static int32_t last_printed_rpm = -1;
    static int32_t last_printed_target = -1;

    if (!motor_util_motor_enabled) {
        return;
    }

    uint32_t captured_pulses = lm_pulses;

    uint32_t captured_samples = lm_samples;

    lm_pulses = 0;
    lm_samples = 0;

    if (captured_samples > 0) {
        float total_time = (float)captured_samples * SAMPLE_TIME_SEC;
        float calculated_avg_rpm = ((float)captured_pulses / total_time) / ENCODER_EDGES_PER_REV * 60.0f;
        motor_util_display_rpm = (motor_util_display_rpm * 0.3f) + (calculated_avg_rpm * 0.7f);
    } else {
        motor_util_display_rpm = 0.0f;
    }

    if(motor_util_target_rpm <= 0){
        motor_util_display_rpm = lm_raw_rpm;
    }

    int32_t display_rpm_int = (int32_t)(motor_util_display_rpm + 0.5f);
    int32_t motor_util_target_rpm_int = (int32_t)(motor_util_target_rpm + 0.5f);

    if (display_rpm_int != last_printed_rpm || motor_util_target_rpm_int != last_printed_target) {

        char log_buffer[36];
        snprintf(log_buffer, sizeof(log_buffer), "RPM: %d / %d\n", display_rpm_int, motor_util_target_rpm_int);
        hal.stream.write(log_buffer);

        last_printed_rpm = display_rpm_int;
        last_printed_target = motor_util_target_rpm_int;
    }
}

float motor_util_get_rpm(){
    return lm_current_rpm;
}

void motor_util_report_rpm()
{
    char log_buffer[36];
    snprintf(log_buffer, sizeof(log_buffer), "#RPM: %d / %d\n", (int)motor_util_display_rpm, (int)motor_util_target_rpm);
    hal.stream.write(log_buffer);
}

void motor_util_set_target(uint16_t rpm){
    motor_util_target_rpm = rpm;
}

void motor_util_init()
{


    const esp_timer_create_args_t logging_timer_args = {
        .callback = &motor_util_logging_ticker,
        .name = "linear_motor_logging_timer"
    };
    esp_timer_create(&logging_timer_args, &motor_util_logging_timer_handle);
    esp_timer_start_periodic(motor_util_logging_timer_handle, 500000); // 500ms loop
}
