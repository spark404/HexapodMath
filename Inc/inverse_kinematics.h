#ifndef HEXAPODMATH_INC_INVERSE_KINEMATICS_H_
#define HEXAPODMATH_INC_INVERSE_KINEMATICS_H_

#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

void inverse_kinematics(const float32_t *origin, const float32_t *tip, float32_t *angles);

#ifdef __cplusplus
}
#endif

#endif /* HEXAPODMATH_INC_INVERSE_KINEMATICS_H_ */
