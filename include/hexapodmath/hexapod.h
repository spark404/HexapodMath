//
// Created by Hugo Trippaers on 01/01/2025.
//

#ifndef HEXAPODMATH_HEXAPOD_H
#define HEXAPODMATH_HEXAPOD_H

#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif


void project_point_on_circle(float32_t radius, const float32_t origin[2], const float32_t direction[2], float32_t point[2]);
uint32_t almost_equal(float32_t srcA, float32_t srcB, float32_t epsilon);
void calculate_path(float32_t current[3], float32_t target[3], float32_t lift_height, float32_t lift_incline_factor, float32_t path[4][3]);
void interpolate(float32_t path[4][3], float32_t step_size, float32_t next[3]);
float32_t calculate_path_length(float32_t path[4][3]);

#ifdef __cplusplus
}
#endif


#endif //HEXAPODMATH_HEXAPOD_H
