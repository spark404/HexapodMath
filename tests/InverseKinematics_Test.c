//
// Created by Hugo Trippaers on 01/11/2024.
//
#include "unity_fixture.h"

#include "hexapodmath/conversion_2d.h"
#include "hexapodmath/inverse_kinematics.h"
#include "hexapodmath/pose.h"
#include "hexapodmath/matrix_3d.h"
#include "hexapodmath/additional_functions.h"
#include "hexapodmath/forward_kinematics.h"

TEST_GROUP(InverseKinematics);

TEST_SETUP(InverseKinematics) {

}

TEST_TEAR_DOWN(InverseKinematics) {

}

TEST(InverseKinematics, InverseKinematics_Start) {
    float32_t origin[3] = {0, 0, 0};
    float32_t tip[3] = {124, 0, -150};

    float32_t expected[3] = {0.f, 0.f, M_PI_2};
    float32_t actual[3];

    inverse_kinematics(origin, tip, actual);

    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.0001, expected, actual, 3);
}

TEST(InverseKinematics, InverseKinematics_Extreme) {
    float32_t origin[3] = {0, 0, 0};
    float32_t tip[3] = {274, 0, 0};

    float32_t expected[3] = { 0, 0, 0};
    float32_t actual[3];

    inverse_kinematics(origin, tip, actual);

    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.0001, expected, actual, 3);
}

TEST(InverseKinematics, InverseKinematics_Realistic) {
    float32_t origin[3] = {0, 0, 0};
    float32_t tip[3] = {161.4718f, 0, -116.6255f};

    float32_t expected[3] = { 0, -0.2793f, 1.5708f};
    float32_t actual[3];

    inverse_kinematics(origin, tip, actual);

    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.0001, expected, actual, 3);
}

TEST(InverseKinematics, InverseKinematics_RealisticWithOmega) {
    float32_t origin[3] = {0, 0, 0};
    float32_t tip[3] = {160.8573f, 14.0732f, -116.6255f};

    float32_t expected[3] = { 0.0873f, -0.2793f, 1.5708f};
    float32_t actual[3];

    inverse_kinematics(origin, tip, actual);

    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.0001, expected, actual, 3);
}
