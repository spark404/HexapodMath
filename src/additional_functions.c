//
// Created by Hugo Trippaers on 25/10/2024.
//

#include "hexapodmath/additional_functions.h"
#include "assert.h"

// Function to compute the norm of a floating-point vector
void arm_vec_norm_f32(const float32_t *pSrc, uint32_t blockSize, float32_t *pResult) {
    float32_t sum = 0.0f;
    for (uint32_t i = 0; i < blockSize; i++) {
        sum += pSrc[i] * pSrc[i]; // Sum of squares
    }
    // Calculate the Euclidean norm
    float32_t norm;
    arm_sqrt_f32(sum, &norm);

    // Optionally store the result
    if (pResult != NULL) {
        *pResult = norm;
    }
}

// Function to normalize the vector
void arm_vec_normalize_f32(const float32_t *pSrc, float32_t *pDst, uint32_t blockSize) {
    float32_t norm;
    arm_vec_norm_f32(pSrc, blockSize, &norm);

    if (norm > 0.0f) {
        for (uint32_t i = 0; i < blockSize; i++) {
            pDst[i] = pSrc[i] / norm; // Normalize the vector
        }
    }
}

void arm_vec_skew_matrix_f32(float32_t vector[3], arm_matrix_instance_f32 *S) {
    assert(S->numCols == 3);
    assert(S->numRows == 3);

    S->pData[0 * 3 + 0] = 0;
    S->pData[0 * 3 + 1] = -vector[2];
    S->pData[0 * 3 + 2] = vector[1];

    S->pData[1 * 3 + 0] = vector[2];
    S->pData[1 * 3 + 1] = 0;
    S->pData[1 * 3 + 2] = -vector[0];

    S->pData[2 * 3 + 0] = -vector[1];
    S->pData[2 * 3 + 1] = vector[0];
    S->pData[2 * 3 + 2] = 0;
}

void arm_vec_cross_f32(const float32_t *pSrcA, const float32_t *pSrcB, float32_t *pDst, uint32_t blocksize)
{
    assert(blocksize == 3); // Not implemented for any other lenght
    pDst[0] = pSrcA[1] * pSrcB[2] - pSrcA[2] * pSrcB[1];
    pDst[1] = pSrcA[2] * pSrcB[0] - pSrcA[0] * pSrcB[2];
    pDst[2] = pSrcA[0] * pSrcB[1] - pSrcA[1] * pSrcB[0];
}

void arm_vec_add_f32(const float32_t *pSrcA, const float32_t *pSrcB, float32_t *pDst, uint32_t blocksize)
{
    for (uint32_t i = 0; i<blocksize; i++) {
        pDst[i] = pSrcA[i] + pSrcB[i];
    }
}

void arm_vec_sub_f32(const float32_t *pSrcA, const float32_t *pSrcB, float32_t *pDst, uint32_t blocksize)
{
    for (uint32_t i = 0; i<blocksize; i++) {
        pDst[i] = pSrcA[i] - pSrcB[i];
    }
}

void arm_vec_mult_scalar_f32(const float32_t *pSrc, const float32_t multiplier, float32_t *pDst, uint32_t blocksize) {
    for (uint32_t i = 0; i<blocksize; i++) {
        pDst[i] = pSrc[i] * multiplier;
    }
}

void arm_vec_copy_f32(const float32_t *pSrc, float32_t *pDst, uint32_t blocksize) {
    for (uint32_t i = 0; i<blocksize; i++) {
        pDst[i] = pSrc[i];
    }
}

void arm_vec_add_scalar_f32(const float32_t *pSrc, const float32_t addition, float32_t *pDst, uint32_t blocksize) {
    for (uint32_t i = 0; i<blocksize; i++) {
        pDst[i] = pSrc[i] + addition;
    }
}

float32_t arm_vec_magnitude_f32(const float32_t *pA, uint32_t blockSize) {
    float32_t accum=0.0f,tmp;

    while(blockSize > 0)
    {
        tmp = *pA++;
        accum += ARM_SQ(tmp);
        blockSize --;
    }
    arm_sqrt_f32(accum,&tmp);
    return(tmp);
}

void arm_mat_identity_f32(arm_matrix_instance_f32 *M) {
    assert(M->numRows == M->numCols);

    bzero(M->pData, sizeof(float32_t) * M->numCols * M->numRows);
    for (uint32_t i = 0; i<M->numRows; i++) {
        M->pData[i * M->numCols + i] = 1.;
    }
}

void arm_mat_mult_scalar_f32(const float32_t value, arm_matrix_instance_f32 *Msrc, arm_matrix_instance_f32 *Mdst) {
    assert(Msrc->numCols == Mdst->numCols);
    assert(Msrc->numRows == Mdst->numRows);

    for (uint32_t i = 0; i<Msrc->numCols; i++) {
        for (uint32_t j = 0; j<Msrc->numRows; j++) {
            Mdst->pData[i * Msrc->numCols + j] = Msrc->pData[i * Msrc->numCols + j] * value;
        }
    }
}

/// Create a transformation matrix from a rotation matrix and a translation
///
/// \param Rsrc 3x3 rotation matrix
/// \param tsrc transformation vector
/// \param Tdst [out] 4x4 transformation matrix
void arm_mat_transformation_matrix_f32(arm_matrix_instance_f32 *Rsrc, float32_t tsrc[3], arm_matrix_instance_f32 *Tdst) {
    assert(Rsrc->numCols == 3);
    assert(Rsrc->numRows == 3);

    assert(Tdst->numCols == 4);
    assert(Tdst->numRows == 4);

    // Start with the 4x4 identity matrix
    arm_mat_identity_f32(Tdst);

    // Copy the rotation matrix
    for (uint32_t i = 0; i<Rsrc->numRows; i++) {
        for (uint32_t j = 0; j < Rsrc->numCols; j++) {
            Tdst->pData[i * Tdst->numRows + j] = Rsrc->pData[i * Rsrc->numRows + j];
        }
    }

    // Copy the translation
    for (uint32_t i = 0; i < 3; i++) {
        Tdst->pData[i * Tdst->numRows + 3] = tsrc[i];
    }
}