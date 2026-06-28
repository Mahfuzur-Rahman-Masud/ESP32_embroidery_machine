#ifndef LINEAR_MOTOR_H
#define LINEAR_MOTOR_H

#include <stdint.h>
#include "driver.h"

#ifdef LINEAR_MOTOR



void linear_motor_init(void);
void linear_motor_enable(bool on);
void linear_motor_set_rpm(int16_t target_rpm);
void linear_motor_set_rpm_ramp(int16_t target_rpm, int16_t ramp);
int16_t linear_motor_get_rpm();
void linear_motor_report_rpm();
void linear_motor_trim(void);

#endif // LINEAR_MOTOR
#endif // LINEAR_MOTOR_H