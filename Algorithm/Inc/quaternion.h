#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include <stdint.h>
#include "arm_math.h"

typedef struct
{
    float32_t q_w; // real part
    float32_t q_x; // imaginary i
    float32_t q_y; // imaginary j
    float32_t q_z; // imaginary k
} Quaternion;

// set and copy
void quat_set(Quaternion *q, float32_t w, float32_t x, float32_t y, float32_t z);
void quat_identity(Quaternion *q);
void quat_copy(Quaternion *dest, Quaternion *src);

// basic algebra
void quat_add(Quaternion *a, Quaternion *b, Quaternion *result);      // result = a + b
void quat_subtract(Quaternion *a, Quaternion *b, Quaternion *result); // result = a - b
void quat_multiply(Quaternion *a, Quaternion *b, Quaternion *result); // result = a \otimes b
void quat_scale(Quaternion *q, float32_t scalar, Quaternion *result); // result = scalar q

// vector character algebra
float32_t quat_norm(Quaternion *q);
void quat_normalize(Quaternion *q);
void quat_conjugate(Quaternion *q, Quaternion *result); // conjugation
void quat_inverse(Quaternion *q, Quaternion *result);   // conjugation + norm

// quaternion derivative operation
void quat_derivative(Quaternion *q, float32_t w[3], Quaternion *result);

// quaterion and vector rotation
void quat_from_axis_angle(float32_t axis[3], float32_t angle, Quaternion *q);
void quat_to_axis_angle(Quaternion *q, float32_t axis[3], float32_t *angle);
void quat_rotate_vector(Quaternion *q, float32_t vec[3], float32_t result[3]);

// quaternion and euler angles
// euler[0]: yaw, euler[1]: pitch, euler[2]: roll
void quat_from_euler(float32_t euler[3], Quaternion *q);
void quat_to_euler(Quaternion *q, float32_t euler[3]);

#endif // __QUATERNION_H__
