//
// Created by Hugo Trippaers on 25/10/2024.
//

#ifndef HEXAPODTESTS_ADDITIONAL_FUNCTIONS_H
#define HEXAPODTESTS_ADDITIONAL_FUNCTIONS_H

#include "arm_math.h"

#define R2D(R) (R * 57.295779513082323)
#define D2R(D) (D * 0.017453292519943)

#define MATRIX(M,S) \
	arm_matrix_instance_f32 M; \
    float32_t p ## M ## Data[S*S];   \
    arm_mat_init_f32(&M, S, S, p ## M ## Data)

#define MATRIX4(M) \
    MATRIX(M, 4)

#ifdef __cplusplus
extern "C" {
#endif

void arm_vec_norm_f32(const float32_t *pSrc, uint32_t blockSize, float32_t *pResult);

void arm_vec_normalize_f32(const float32_t *pSrc, float32_t *pDst, uint32_t blockSize);

void arm_vec_skew_matrix(float32_t vector[3], arm_matrix_instance_f32 *S);

void arm_vec_cross(const float32_t *pSrcA, const float32_t *pSrcB, float32_t *pDst, uint32_t blocksize);

void arm_vec_add(const float32_t *pSrcA, const float32_t *pSrcB, float32_t *pDst, uint32_t blocksize);

void arm_vec_sub(const float32_t *pSrcA, const float32_t *pSrcB, float32_t *pDst, uint32_t blocksize);

void arm_vec_mult_scalar_f32(const float32_t *pSrc, float32_t multiplier, float32_t *pDst, uint32_t blocksize);

void arm_vec_add_scalar_f32(const float32_t *pSrc, float32_t addition, float32_t *pDst, uint32_t blocksize);

void arm_vec_copy_f32(const float32_t *pSrc, float32_t *pDst, uint32_t blocksize);

float32_t arm_vec_magnitude(const float32_t *pA, uint32_t blockSize);

void arm_mat_identity_f32(arm_matrix_instance_f32 *M);

void arm_mat_mult_scalar_f32(float32_t value, arm_matrix_instance_f32 *Msrc, arm_matrix_instance_f32 *Mdst);

void arm_mat_transformation_matrix_f32(arm_matrix_instance_f32 *Rsrc, float32_t tsrc[3], arm_matrix_instance_f32 *Tdst);

#ifdef __cplusplus
}
#endif

#endif //HEXAPODTESTS_ADDITIONAL_FUNCTIONS_H
