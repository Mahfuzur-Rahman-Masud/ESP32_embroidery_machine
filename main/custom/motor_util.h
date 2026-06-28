#ifndef MOTOR_UTIL_H__
#define MOTOR_UTIL_H__
#include <stdint.h>
#include "driver.h"

#ifndef ENCODER_EDGES_PER_REV
#define ENCODER_EDGES_PER_REV       (DEFAULT_SPINDLE_PPR * 2.0f)
#endif

static const float SAMPLE_TIME_SEC = 0.05f;







void motor_util_init();
void motor_util_enable(bool on);
float motor_util_get_rpm();
void motor_util_report_rpm();
void motor_util_set_target(uint16_t rpm);
#endif //MOTOR_UTIL_H__