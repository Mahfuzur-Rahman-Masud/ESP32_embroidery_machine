#include "linear_motor.h"
#include "driver/ledc.h"
#include "esp_vfs_fat.h" // Replace with spiffs/littlefs header based on your VFS mounting
#include "grbl/grbl.h"
#include <stdio.h>
#include <string.h>
#include "motor_control.h"
#include "motor_util.h"
#include "driver/pcnt.h" 

#ifdef LINEAR_MOTOR

#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_RESOLUTION LEDC_TIMER_10_BIT 
#define SYSTEM_MAX_UV   4720000U  
// #define SYSTEM_MAX_UV   3500000U  
#define PWM_BIT_RES     LEDC_RESOLUTION        
#define PWM_MAX_DUTY    ((1U << PWM_BIT_RES) - 1U) 


static inline uint32_t __attribute__((always_inline)) uv_to_pwm(uint32_t target_uv) {
    if (target_uv >= SYSTEM_MAX_UV) return PWM_MAX_DUTY;
    return (uint32_t)(((uint64_t)target_uv * PWM_MAX_DUTY) / SYSTEM_MAX_UV);
}



static bool linear_motor_enabled = false;
static bool linear_motor_on = false;
static volatile int16_t lm_target_rpm = 0; 
static volatile int32_t lm_uv = 0;
static void setup_hardware_pwm(uint32_t freq_hz);
static void write_hardware_pwm(int16_t duty);

volatile uint32_t lm_pulses = 0;
volatile uint32_t lm_samples = 0;
volatile float lm_raw_rpm = 0.0f;
volatile float lm_current_rpm = 0.0f;

// Persistent ALL-INTEGER PI Controller variables
static int32_t lm_pi_i_term = 0;       
static const int32_t lm_Kp = 40;     
// Ki is pre-multiplied by SAMPLE_TIME_SEC (50ms = 0.05) to eliminate decimal loops.
// Original Ki (45) * 0.05 = 2.25 -> Rounded to 2 for pure integer math
static const int32_t lm_Ki_scaled = 2; 
static const int32_t INTEGRAL_LIMIT = 1000; // Anti-windup cap


esp_timer_handle_t lm_pulse_timer_handle = NULL;

typedef struct {
    int16_t min_rpm;
    int16_t max_rpm;
    
    int16_t trim_delay;

    int32_t idle_uv;
    int32_t trim_uv;
    int32_t min_rpm_uv;
    int32_t min_rpm_max_uv;
    int32_t max_uv;
    int32_t volt_ramp_up;
    int32_t volt_ramp_down;

} linear_motor_config_t;

static linear_motor_config_t lm_config;



void linear_motor_enable(bool on){
    linear_motor_enabled = on;
    motor_util_enable(on);

    if(on){
        motor_control_enable(false);
    }
}


static void  write_hardware_pwm(int16_t duty)
{
    if (duty < 0)
        duty = 0;
    if (duty > 1023)
        duty = 1023;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL);
}

static void IRAM_ATTR linear_motor_set_uv(int32_t uv){
    lm_uv = uv;
    write_hardware_pwm(uv_to_pwm(uv));
}


void linear_motor_report_rpm(){
   motor_util_report_rpm();
}

IRAM_ATTR int16_t linear_motor_get_rpm()
{
    return (int16_t) motor_util_get_rpm();
}


IRAM_ATTR void linear_motor_set_rpm_ramp(int16_t target_rpm, int16_t ramp)
{
    if (target_rpm > lm_target_rpm + ramp) {
        linear_motor_set_rpm(lm_target_rpm + ramp);
    } 
    else  {
        linear_motor_set_rpm(target_rpm);
    } 
}


IRAM_ATTR void linear_motor_set_rpm(int16_t target_rpm)
{

    if(target_rpm > 0 && !linear_motor_enabled){
        linear_motor_enable(true);
    }

    if(target_rpm <= 0){
        target_rpm = 0;
    }else if (target_rpm > lm_config.max_rpm){
        target_rpm = lm_config.max_rpm;
    }

    if(target_rpm > 0){
        linear_motor_on = true;
    }

    lm_target_rpm = target_rpm;
    motor_util_set_target(target_rpm);
}


void linear_motor_trim(void)
{
    linear_motor_set_uv(lm_config.trim_uv);
    vTaskDelay(lm_config.trim_delay);
    linear_motor_set_uv(lm_config.idle_uv);
}



static void IRAM_ATTR take_sample(void* arg)
{
    if (!linear_motor_enabled) {
        return;
    }

    int16_t pulse_count = 0;

    pcnt_get_counter_value(PCNT_UNIT_0, &pulse_count);
    pcnt_counter_clear(PCNT_UNIT_0);
    int16_t target = lm_target_rpm;
    // if (target > 0 && target < lm_config.min_rpm){
    //     target = lm_config.min_rpm;
    // }

    lm_pulses += pulse_count;
    lm_samples++;
    lm_raw_rpm = ((float)pulse_count / SAMPLE_TIME_SEC) / ENCODER_EDGES_PER_REV * 60.0f;
    lm_current_rpm = (lm_current_rpm * 0.80f) + (lm_raw_rpm * 0.20f);

    if(target == 0){
        if(linear_motor_on){
            linear_motor_set_uv(lm_config.idle_uv);
            linear_motor_on = false;
        }

        return;
    }

    bool motor_moves = lm_current_rpm > lm_config.min_rpm /2;
    bool needs_pid = false;


    int32_t uv = lm_uv;
    if (motor_moves
        && lm_current_rpm  > target - 30 
        && lm_current_rpm < target + 30){
            // Deadband target reached - hold voltage steady
        // printf("SE1: [%u] rpm: %u / %u\n", uv, (int)lm_current_rpm, target);
        return;

    }else if (lm_uv < lm_config.min_rpm_uv)    {
        uv = (lm_config.min_rpm_uv + lm_config.min_rpm_max_uv) / 2;
        // printf("SeEX1\n");
        
    }else if (target > lm_config.min_rpm && lm_uv < lm_config.min_rpm_max_uv)    {
        uv = lm_config.min_rpm_max_uv;
        // printf("SeEX2\n");

    }else if (target >lm_config.min_rpm && uv < lm_config.min_rpm_max_uv){    
         uv = lm_config.min_rpm_max_uv;

    }else if(target <= lm_config.min_rpm ){
       
        if(!motor_moves && uv < lm_config.min_rpm_max_uv){
            uv += lm_config.volt_ramp_up;
            needs_pid = true;
        }else if (lm_current_rpm > lm_config.min_rpm + 30 && uv > lm_config.min_rpm_max_uv){
            uv -= lm_config.volt_ramp_down;
            needs_pid = true;

        }else if(!motor_moves){
            // printf("min-rpm max volt reached\n");
        }
        
        // printf("SE2: [%u] rpm: %u / %u\n", uv, (int)lm_current_rpm, target);

    }else if(uv < lm_config.min_rpm_max_uv) {
         uv = lm_config.min_rpm_max_uv;
        // printf("SE3: [%u] rpm: %u / %u\n", uv, (int)lm_current_rpm, target);

    }else if (lm_current_rpm < target){
        uv+= lm_config.volt_ramp_up;
        needs_pid = true;
        // printf("SE4: [%u] rpm: %u / %u\n", uv, (int)lm_current_rpm, target);

    }else {
        // current rpm > target
        if (lm_uv > lm_config.min_rpm_uv){
            uv-= lm_config.volt_ramp_down;
            needs_pid = true;
            // printf("SE5: [%u] rpm: %u / %u\n", uv, (int)lm_current_rpm, target);

        }else{
            // printf("SE6: [%u] rpm: %u / %u\n", uv, (int)lm_current_rpm, target);
            // printf("min rpm uv reached cannot reduce volt / rpm anymore\n");
        }
    }


    if(needs_pid){
        int32_t actual_target = target;
        if (actual_target > 0 && actual_target < lm_config.min_rpm){
            actual_target = lm_config.min_rpm;
        }

        int32_t error = (int32_t)actual_target - (int32_t)lm_current_rpm;
        lm_pi_i_term += error;
        if (lm_pi_i_term > INTEGRAL_LIMIT)  lm_pi_i_term = INTEGRAL_LIMIT;
        if (lm_pi_i_term < -INTEGRAL_LIMIT) lm_pi_i_term = -INTEGRAL_LIMIT;
        int32_t pi_output = (lm_Kp * error) + (lm_Ki_scaled * lm_pi_i_term);
        int32_t target_uv = uv + pi_output;

        if(target_uv < lm_config.min_rpm_uv && actual_target <= lm_config.min_rpm){
            target_uv = lm_config.min_rpm_uv;

        }else if (target_uv > lm_config.min_rpm_max_uv && actual_target > lm_config.min_rpm_max_uv){
            target_uv = lm_config.min_rpm_max_uv;

        }else if (target_uv < lm_config.idle_uv){
            target_uv = lm_config.idle_uv;
        }

        uv = target_uv;
    }

    linear_motor_set_uv(uv);
}


static void linear_motor_load_config(void){
    
    lm_config.min_rpm              = 200;
    lm_config.max_rpm              = 3000; 

    lm_config.trim_delay           = 5;    
    
    lm_config.idle_uv              = 1000000; // actual start at 400,000
    lm_config.trim_uv              = 250000;
    lm_config.min_rpm_uv           = 2000000; // actual 1500,000
    lm_config.min_rpm_max_uv       = 3300000; // break from minimum RPM
    lm_config.max_uv               = 4700000;
    lm_config.volt_ramp_up         = 1000;    // 1mv
    lm_config.volt_ramp_down       = lm_config.volt_ramp_up * 2; // 1v

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
        .duty = uv_to_pwm(400000), // Start at .4v 
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}




void linear_motor_init(void)
{
    
    
    linear_motor_load_config();
    setup_hardware_pwm(25000);
    linear_motor_set_rpm(0);

    
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = SPINDLE_PULSE_PIN,   // GPIO 10 connected to your Pulse wire
        .ctrl_gpio_num = -1,                   // Disables direction pin to prevent signal corruption
        .lctrl_mode = PCNT_MODE_KEEP,          // Ignored when ctrl_gpio is -1
        .hctrl_mode = PCNT_MODE_KEEP,          // Ignored when ctrl_gpio is -1
        .pos_mode = PCNT_COUNT_INC,            // Count up on rising edges
        .neg_mode = PCNT_COUNT_INC,            // Count up on falling edges (Enables X2 Doubling!)
        .counter_h_lim = 32000,                // Secure safety ceiling limits
        .counter_l_lim = -10,                  // Adjusted padding margin to stop roll-over bugs
        .unit = PCNT_UNIT_0,                   // Binds to Unit 0 (Matches your ISR query)
        .channel = PCNT_CHANNEL_0,
    };


    
    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(PCNT_UNIT_0, 100); 
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);


    const esp_timer_create_args_t timer_args = {
        .callback = &take_sample,
        .name = "linear_motor_timer"
    };
    esp_timer_create(&timer_args, &lm_pulse_timer_handle);
    esp_timer_start_periodic(lm_pulse_timer_handle, 50000); // 50,000µs = 50ms loop


    // motor_util_init();
    printf("Linear motor initialized\n");
}

#endif