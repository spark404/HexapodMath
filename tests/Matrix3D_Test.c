//
// Created by Hugo Trippaers on 01/01/2025.
//
#include "unity_fixture.h"

#include "hexapodmath/matrix_3d.h"
#include "hexapodmath/additional_functions.h"

TEST_GROUP(Matrix3D);

TEST_SETUP(Matrix3D)
{
}

TEST_TEAR_DOWN(Matrix3D)
{
}

TEST(Matrix3D, Matrix3D_Invert) {
    float32_t Tdata[4][4] = {
            {0.573576436351046f, 0.819152044288992f, 0, 51.621899999999997f},
            {-0.819152044288992f, 0.573576436351046f, 0, -73.723699999999994f},
            {0 ,0, 1, 0},
            {0,0,0,1}
    };
    arm_matrix_instance_f32 T;
    arm_mat_init_f32(&T, 4, 4, (float32_t *) Tdata);

    MATRIX4(actual);
    matrix_3d_invert(&T, &actual);

    float32_t expected[4*4] = { 0.5735764f, -0.8191520f, 0.f, -90.f,
                                0.8191520f, 0.5735764f, 0.f, 0.f,
                                0.f, 0.f, 1.f, 0.f,
                                0.f, 0.f, 0.f, 1.f};

    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.00001, expected, actual.pData, 4*4);
}

TEST(Matrix3D, Matrix3D_Invert2) {
    float32_t Tdata[4][4] = {
            {0.573576436351046f, 0.819152044288992f, 0, 51.621899999999997f},
            {-0.819152044288992f, 0.573576436351046f, 0, -73.723699999999994f},
            {0 ,0, 1, 0},
            {0,0,0,1}
    };
    arm_matrix_instance_f32 T;
    arm_mat_init_f32(&T, 4, 4, (float32_t *) Tdata);

    MATRIX4(actual);
    matrix_3d_invert(&T, &actual);

    MATRIX4(expected);
    arm_status res = arm_mat_inverse_f32(&T, &expected);
    TEST_ASSERT_EQUAL(res, ARM_MATH_SUCCESS);

    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.00001, expected.pData, actual.pData, 4*4);
}

