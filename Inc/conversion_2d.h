#ifndef CONVERSION_2D_H
#define CONVERSION_2D_H

#include <stdint.h>
#include <arm_math.h>

#ifdef __cplusplus
extern "C" {
#endif

void convert_2d_polar_to_cartesian(const float32_t *polar, float32_t *cartesian);

#ifdef __cplusplus
}
#endif

#endif /* CONVERSION_2D_H */
