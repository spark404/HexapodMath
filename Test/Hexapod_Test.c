//
// Created by Hugo Trippaers on 01/01/2025.
//
#include "unity_fixture.h"

#include "hexapod.h"

TEST_GROUP(Hexapod);

TEST_SETUP(Hexapod)
{
}

TEST_TEAR_DOWN(Hexapod)
{
}

TEST(Hexapod, Hexapod_PointNoMovement) {
    float32_t radius = 25.f;
    float32_t origin[2] = { 1.f, 2.f};
    float32_t direction[2] = {0.f, 0.f};

    float32_t actual[2];
    project_point_on_circle(radius, origin, direction, actual);

    TEST_ASSERT_EQUAL_FLOAT_ARRAY(actual, origin, 2);
}

TEST(Hexapod, Hexapod_PointMovementInsideRadius) {
    float32_t radius = 25.f;
    float32_t origin[2] = { 0.f, -194.f};
    float32_t direction[2] = {10.f, 0.f};

    float32_t actual[2];
    project_point_on_circle(radius, origin, direction, actual);

    float32_t expected[2] = { 10.f, -194.f};
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected, actual, 2);
}

TEST(Hexapod, Hexapod_PointMovementOutsideRadius) {
    float32_t radius = 25.f;
    float32_t origin[2] = { 0, -194};
    float32_t direction[2] = {50.f, 0.f};

    float32_t actual[2];
    project_point_on_circle(radius, origin, direction, actual);

    float32_t expected[2] = { 25.f, -194.f};
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected, actual, 2);
}

TEST(Hexapod, Hexapod_PointMovementOutsideRadiusAtAngle) {
    float32_t radius = 25.f;
    float32_t origin[2] = { 0.f, -194.f};
    float32_t direction[2] = {16.f, 20.f};

    float32_t actual[2];
    project_point_on_circle(radius, origin, direction, actual);

    float32_t expected[2] = { 15.61738f, -174.4783f};
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected, actual, 2);
}
