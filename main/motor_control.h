#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the hardware peripherals (PCNT, LEDC) and starts the 1kHz ADRC timer loop.
 */
void init_adrc_spindle_control(void);

float getTargetRPM();
float getRPM();
void reportRPM();

/**
 * @brief Updates the target RPM setpoint for the ADRC controller.
 * @param rpm Desired motor speed in Revolutions Per Minute.
 */
void set_adrc_spindle_speed(float rpm);


void set_adrc_spindle_speed_ramp(float rpm, float ramp);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_H
