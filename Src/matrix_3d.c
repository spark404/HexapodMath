#include <assert.h>
#include "additional_functions.h"

#include "matrix_3d.h"

typedef arm_matrix_instance_f32 *matrix;

void matrix_3d_translation_init(matrix T){
	// Initialize matrix T as transformation matrix
	for (int i = 0; i<4; i++) {
		for (int j = 0; j<4; j++) {
			T->pData[i * 4 + j] = i == j;
		}
	}
}

void matrix_3d_translation_matrix(matrix T, float32_t translation[3])
{
	matrix_3d_translation_init(T);

	// Apply the translation
	for (int i = 0; i<3; i++) {
		T->pData[i * 4 + 3] = translation[i];
	}
}

void matrix_3d_rotation_x_matrix(matrix Rx, float32_t omega)
{
	matrix_3d_translation_init(Rx);

	float32_t c = arm_cos_f32(omega);
	float32_t s = arm_sin_f32(omega);

	Rx->pData[1 * 4 + 1] = c;
	Rx->pData[1 * 4 + 2] = -s;
	Rx->pData[2 * 4 + 1] = s;
	Rx->pData[2 * 4 + 2] = c;
}

void matrix_3d_rotation_y_matrix(matrix Ry, float32_t omega)
{
	matrix_3d_translation_init(Ry);

	float32_t c = arm_cos_f32(omega);
	float32_t s = arm_sin_f32(omega);

	Ry->pData[0 * 4 + 0] = c;
	Ry->pData[0 * 4 + 2] = s;
	Ry->pData[2 * 4 + 0] = -s;
	Ry->pData[2 * 4 + 2] = c;
}

void matrix_3d_rotation_z_matrix(matrix Rz, float32_t omega)
{
	matrix_3d_translation_init(Rz);

	float32_t c = arm_cos_f32(omega);
	float32_t s = arm_sin_f32(omega);

	Rz->pData[0 * 4 + 0] = c;
	Rz->pData[0 * 4 + 1] = -s;
	Rz->pData[1 * 4 + 0] = s;
	Rz->pData[1 * 4 + 1] = c;
}

void matrix_3d_invert(arm_matrix_instance_f32 *Tsrc, arm_matrix_instance_f32 *Tdst) {
    assert(Tsrc->numRows == 4);
    assert(Tsrc->numCols == 4);
    assert(Tdst->numRows == 4);
    assert(Tdst->numCols == 4);

    MATRIX(R, 3);
    for (uint32_t i = 0; i<3; i++) {
        for (uint32_t j = 0; j<3; j++) {
            R.pData[i * 3 + j] = Tsrc->pData[i * 4 + j];
        }
    }

    float32_t t[3] = {Tsrc->pData[0*4 + 3], Tsrc->pData[1*4 + 3], Tsrc->pData[2*4 + 3]};

    MATRIX(R_T, 3);
    arm_mat_trans_f32(&R, &R_T);

    float32_t neg_Rt[3];
    for (int i = 0; i < 3; i++) {
        neg_Rt[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            neg_Rt[i] -= R_T.pData[i * 3 + j] * t[j];
        }
    }

    for (uint32_t i = 0; i<3; i++) {
        for (uint32_t j = 0; j<3; j++) {
            Tdst->pData[i * 4 + j] = R_T.pData[i * 3 + j];
        }
        Tdst->pData[i * 4 + 3] = neg_Rt[i];
    }

    Tdst->pData[3 * 4 + 0] = 0;
    Tdst->pData[3 * 4 + 1] = 0;
    Tdst->pData[3 * 4 + 2] = 0;
    Tdst->pData[3 * 4 + 3] = 1;
}

void matrix_3d_vec_transform(arm_matrix_instance_f32 *T, const float32_t pSrc[3], float32_t pDst[3]) {
    assert(T->numCols == 4);
    assert(T->numRows == 4);

    float32_t v[4] = { 0.0f, 0.0f, 0.0f, 1.0f};
    for (uint32_t i=0; i<3; i++) {
        v[i] = pSrc[i];
    }

    float32_t v_out[4];
    arm_mat_vec_mult_f32(T, v, v_out);

    for (uint32_t i=0; i<3; i++) {
        pDst[i] = v_out[i];
    }
}