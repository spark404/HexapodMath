//
// Created by Hugo Trippaers on 25/10/2024.
//
#include "unity_fixture.h"

#include "conversion_2d.h"
#include "additional_functions.h"

TEST_GROUP(AdditionalFunctions);

TEST_SETUP(AdditionalFunctions)
{
}

TEST_TEAR_DOWN(AdditionalFunctions)
{
}

TEST(AdditionalFunctions, SkewMatrix) {
    float32_t vector[3] = {2, 3, 4};
    MATRIX(actual, 3);

    arm_vec_skew_matrix_f32(vector, &actual);

    float32_t expected[3*3] = { 0.f, -4.f, 3.f,
                                4.f, 0.f, -2.f,
                                -3.f, 2.f, 0.f};

    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.00001, expected, actual.pData, 3*3);
}

TEST(AdditionalFunctions, TransformationMatrix) {
    float32_t vector[3] = {10, 11, 12};

    arm_matrix_instance_f32 src;
    float32_t psrcData[3 * 3] = { 1, 2, 3, 4 , 5, 6, 7, 8, 9};
    arm_mat_init_f32(&src, 3, 3, psrcData);

    arm_matrix_instance_f32 dst;
    float32_t actual[4*4];
    arm_mat_init_f32(&dst, 4, 4, actual);

    arm_mat_transformation_matrix_f32(&src, vector, &dst);

    float32_t expected[4*4] = { 1.f, 2.f, 3.f, 10.f,
                                4.f, 5.f, 6.f, 11.f,
                                7.f, 8.f, 9.f, 12.f,
                                0.f, 0.f, 0.f, 1.f};

    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.00001, expected, actual, 4*4);
}

TEST(AdditionalFunctions, MatrixScalarMult) {
    float32_t scalar = 5;

    arm_matrix_instance_f32 src;
    float32_t psrcData[3 * 3] = { 1, 2, 3, 4 , 5, 6, 7, 8, 9};
    arm_mat_init_f32(&src, 3, 3, psrcData);

    float32_t expected[3*3] = { 5.f, 10.f, 15.f,
                                20.f,25.f, 30.f,
                                35.f, 40.f,45.f};

    arm_matrix_instance_f32 dst;
    float32_t actual[3*3];
    arm_mat_init_f32(&dst, 3, 3, actual);

    arm_mat_mult_scalar_f32(scalar, &src, &dst);

    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.00001, expected, actual, 3*3);
}

TEST(AdditionalFunctions, VectorCrossProduct) {
    float32_t a[3] = {1, 2, 3};
    float32_t b[3] = {3, 2, 1};

    float32_t actual[3];
    arm_vec_cross_f32(a, b, actual, 3);

    float32_t expected[3] = { -4, 8, -4};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.00001, expected, actual, 3);
}

TEST(AdditionalFunctions, VectorCrossProduct2) {
    float32_t a[3] = {0, 1, 0};
    float32_t b[3] = {0, 0, 24};

    float32_t actual[3];
    arm_vec_cross_f32(a, b, actual, 3);

    float32_t expected[3] = { 24, 0, 0};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.00001, expected, actual, 3);
}

TEST(AdditionalFunctions, VectorNorm) {
    float32_t a[3] = {1, 2, 3};

    float32_t actual;
    arm_vec_norm_f32(a, 3, &actual);

    TEST_ASSERT_EQUAL_FLOAT(3.741658, actual);
}

TEST(AdditionalFunctions, VectorNormalize) {
    float32_t a[3] = {1, 2, 3};

    float32_t actual[3];
    arm_vec_normalize_f32(a, actual, 3);

    float32_t norm;
    arm_vec_norm_f32(actual, 3, &norm);

    TEST_ASSERT_EQUAL_FLOAT(1, norm);
}

TEST(AdditionalFunctions, VectorNormalize2) {
    float32_t a[3] = {1, -2, 3};

    float32_t actual[3];
    arm_vec_normalize_f32(a, actual, 3);

    float32_t norm;
    arm_vec_norm_f32(actual, 3, &norm);

    TEST_ASSERT_EQUAL_FLOAT(1, norm);
}