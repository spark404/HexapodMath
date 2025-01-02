//
// Created by Hugo Trippaers on 01/01/2025.
//

#ifndef HEXAPODMATH_HEXAPOD_H
#define HEXAPODMATH_HEXAPOD_H

#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif


void project_point_on_circle(float32_t radius, const float32_t origin[3], const float32_t direction[3], float32_t point[3]);

#ifdef __cplusplus
}
#endif


#endif //HEXAPODMATH_HEXAPOD_H
