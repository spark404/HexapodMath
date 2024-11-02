//
// Created by Hugo Trippaers on 26/10/2024.
//
#include "unity_fixture.h"

#include "forward_kinematics.h"
#include "additional_functions.h"

TEST_GROUP(ForwardKinematics);

TEST_SETUP(ForwardKinematics) {
}

TEST_TEAR_DOWN(ForwardKinematics) {
}

TEST(ForwardKinematics, ForwardKinematicsNoAngles) {
    float32_t angles[3] = {0.f, 0.f, 0.f};
    float32_t actual[3];

    forward_kinematics(angles, actual);

    TEST_ASSERT_FLOAT_WITHIN(0.0001, 274., actual[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0., actual[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0., actual[2]);
}

TEST(ForwardKinematics, ForwardKinematicsJoint390) {
    float32_t angles[3] = {0.f, 0.f, M_PI_2};
    float32_t actual[3];

    forward_kinematics(angles, actual);

    TEST_ASSERT_FLOAT_WITHIN(0.0001, 124., actual[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 0., actual[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, -150., actual[2]);
}

TEST(ForwardKinematics, Rodriques_0Angle) {
    MATRIX(T, 4);
    float32_t theta = 0;
    float32_t twist[6] = {0, 0, 1, 0, 0, 0};
    rodrigues_rotation_matrix(twist, theta, &T);

    float32_t expected[16] = {1, 0, 0, 0,
                              0, 1, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1};

    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.00001, expected, T.pData, 16);
}

TEST(ForwardKinematics, Rodriques_90Angle) {
    MATRIX(T, 4);
    float32_t theta = M_PI_2;
    float32_t twist[6] = {0, 0, 1, 0, 0, 0};
    rodrigues_rotation_matrix(twist, theta, &T);

    float32_t expected[16] = {0, -1, 0, 0,
                              1, 0, 0, 0,
                              0, 0, 1, 0,
                              0, 0, 0, 1};

    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.00001, expected, T.pData, 16);
}

TEST(ForwardKinematics, Rodriques_90Angle50Linear) {
    MATRIX(T, 4);
    float32_t theta = M_PI_2;
    float32_t twist[6] = {0, 1, 0, 0, 0, 50};
    rodrigues_rotation_matrix(twist, theta, &T);

    float32_t expected[16] = {0, 0, 1, 50,
                              0, 1, 0, 0,
                              -1, 0, 0, 50,
                              0, 0, 0, 1};

    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.00001, expected, T.pData, 16);
}