#ifndef __MAHONY_H__
#define __MAHONY_H__

#include <stdint.h>
#include "arm_math.h"
#include "quaternion.h"

// mahony filter struct
typedef struct
{
    Quaternion q;          // current orientation quaternion
    float32_t integral[3]; // integral error
    float32_t kp;
    float32_t ki;
    float32_t i_limit;
    float32_t sample_freq;
} MahonyFilter;

void mahony_init(MahonyFilter *filter,
                 float32_t kp, float32_t ki, float32_t i_limit, float32_t sample_freq);

void mahony_update(MahonyFilter *filter, float32_t gyro[3], float32_t accel[3]);

Quaternion *mahony_get_quaternion(MahonyFilter *filter);
void mahony_get_euler(MahonyFilter *filter, float32_t euler[3]);

// assume stabale, then get orientation form g
void get_orientation_from_g(Quaternion *q, float32_t euler[3], float32_t accel[3]);

#endif // __MAHONY_H__
