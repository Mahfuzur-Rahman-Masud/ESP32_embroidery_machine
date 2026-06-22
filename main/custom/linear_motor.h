#ifndef LINEAR_MOTOR_H
#define LINEAR_MOTOR_H

#include <stdint.h>
#include "driver.h"

#ifdef LINEAR_MOTOR

/**
 * @brief Initializes the linear motor plugin.
 * Loads configuration from LittleFS and sets up hardware PWM.
 */
void linear_motor_init(void);

/**
 * @brief Sets the motor target speed with piecewise linear interpolation mapping.
 * @param target_rpm The desired RPM for the spindle/motor.
 */
void linear_motor_set_rpm(int16_t target_rpm);


/**
 * @brief Sets the motor target speed with piecewise linear interpolation mapping.
 * @param target_rpm The desired RPM for the spindle/motor.
 * @param ramp for slow increase in rpm we use the ramp value
 */
void linear_motor_set_rpm_ramp(int16_t target_rpm, int16_t ramp);

int16_t get_rpm();
void report_rpm();

/**
 * @brief Forces the hardware PWM to output the preset trim calibration value.
 */
void linear_motor_trim(void);

#endif // LINEAR_MOTOR

#endif // LINEAR_MOTOR_H