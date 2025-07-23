//
// Created by Hugo Trippaers on 01/01/2025.
//
#include "unity_fixture.h"

#include "hexapodmath/hexapod.h"

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

    float32_t expected[2] = { 25.f, -194.f};
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

TEST(Hexapod, Interpolate_TestInterpolateMidsegment)
{
    // Path from (0,0,0) -> (1,0,0) -> (2,0,0) -> (3,0,0)
    float32_t path[4][3] = {{0,0,0}, {1,0,0}, {2,0,0}, {3,0,0}};
    float32_t next[3];
    // Try a step halfway along the first segment
    interpolate(path, 0.5f, next);
    float32_t expected[3] = {0.5f, 0.0f, 0.0f};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, expected, next, 3);
}

TEST(Hexapod, Interpolate_TestInterpolateExactSegmentBoundary)
{
    float32_t path[4][3] = {{0,0,0}, {2,0,0}, {4,0,0}, {6,0,0}};
    float32_t next[3];
    // Step size lands exactly at first segment end
    interpolate(path, 2.0f, next);
    float32_t expected[3] = {2.0f, 0.0f, 0.0f};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, expected, next, 3);
}

TEST(Hexapod, Interpolate_TestInterpolateIntoSecondSegment)
{
    float32_t path[4][3] = {{0,0,0}, {2,0,0}, {4,0,0}, {6,0,0}};
    float32_t next[3];
    // Step size lands inside second segment, 3 units from start
    interpolate(path, 3.0f, next);
    float32_t expected[3] = {3.0f, 0.0f, 0.0f};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, expected, next, 3);
}

TEST(Hexapod, Interpolate_TestInterpolatePastEndOfPath)
{
    float32_t path[4][3] = {{0,0,0}, {1,0,0}, {2,0,0}, {3,0,0}};
    float32_t next[3];
    // Step size greater than total path length, should snap to end
    interpolate(path, 10.0f, next);
    float32_t expected[3] = {3.0f, 0.0f, 0.0f};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, expected, next, 3);
}

TEST(Hexapod, Interpolate_TestInterpolateWithDuplicateWaypoints)
{
    // Last segment is non-zero, others are all at the same point
    float32_t path[4][3] = {
        {1,2,3},
        {1,2,3},
        {1,2,3},
        {2,4,6}
    };
    float32_t next[3];
    interpolate(path, 0.0f, next);
    float32_t expected0[3] = {1,2,3};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, expected0, next, 3);

    // Test with step size just before final segment (should interpolate into last segment)
    interpolate(path, 0.5f, next);
    float32_t expected1[3] = {1.133630f, 2.267261f, 3.400891f};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, expected1, next, 3);

    // Test with stepsize overshooting path, should land at last point
    interpolate(path, 20.0f, next);
    float32_t expected2[3] = {2,4,6};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-6, expected2, next, 3);
}

TEST(Hexapod, Interpolate_TestInterpolateRealworldCase)
{
    // Matches your FIXME example
    float32_t path[4][3] = {
        {92.2359467f, -175.293106f, -98.9864807f},
        {92.2359467f, -175.293106f, -98.9864807f},
        {92.2359467f, -175.293106f, -98.9864807f},
        {82.7437592f, -175.296265f, -100.0f}
    };
    float32_t step_size = 9.99557495f;
    float32_t next[3];
    interpolate(path, step_size, next);
    // Length of last segment = sqrt((92.236-82.744)^2 + (..)^2 + (..)^2) ≈ 9.9956, so should land exactly at the last point
    float32_t expected[3] = {82.7437592f, -175.296265f, -100.0f};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(1e-5, expected, next, 3);
}

