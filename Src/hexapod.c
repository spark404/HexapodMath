//
// Created by Hugo Trippaers on 01/01/2025.
//

#include "arm_math.h"
#include "additional_functions.h"

void project_point_on_circle(const float32_t radius, const float32_t origin[2], const float32_t direction[2], float32_t point[2]) {
    float32_t direction_magnitude = arm_vec_magnitude(direction, 2);

    if (direction_magnitude == 0) {
        arm_vec_copy_f32(origin, point, 2);
        return;
    }

    float32_t direction_normalized[2];
    arm_vec_normalize_f32(direction, direction_normalized, 2);

    float32_t clamped_direction_magnitude = fmaxf(direction_magnitude, radius);

    float32_t a[2];
    arm_vec_mult_scalar_f32(direction_normalized, clamped_direction_magnitude, a, 2);

    arm_vec_add(origin, a, point, 2);
}