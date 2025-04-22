//
// Created by Hugo Trippaers on 01/01/2025.
//
#include "unity_fixture.h"

TEST_GROUP_RUNNER(Hexapod) {
    RUN_TEST_CASE(Hexapod, Hexapod_PointNoMovement)
    RUN_TEST_CASE(Hexapod, Hexapod_PointMovementInsideRadius)
    RUN_TEST_CASE(Hexapod, Hexapod_PointMovementOutsideRadius)
    RUN_TEST_CASE(Hexapod, Hexapod_PointMovementOutsideRadiusAtAngle)
}
