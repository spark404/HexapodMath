/*
 * matrix_3d.h
 *
 *  Created on: Oct 5, 2024
 *      Author: hugo
 */

#ifndef HEXAPODMATH_INC_MATRIX_3D_H_
#define HEXAPODMATH_INC_MATRIX_3D_H_

#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

void matrix_3d_translation_matrix(arm_matrix_instance_f32 *T, float32_t translation[3]);
void matrix_3d_rotation_x_matrix(arm_matrix_instance_f32 *T, float32_t omega);
void matrix_3d_rotation_y_matrix(arm_matrix_instance_f32 *T, float32_t omega);
void matrix_3d_rotation_z_matrix(arm_matrix_instance_f32 *T, float32_t omega);

#ifdef __cplusplus
}
#endif

#endif /* HEXAPODMATH_INC_MATRIX_3D_H_ */
