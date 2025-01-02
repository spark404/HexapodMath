//
// Created by Hugo Trippaers on 01/01/2025.
//
#include "unity_fixture.h"

TEST_GROUP_RUNNER(Matrix3D) {
    RUN_TEST_CASE(Matrix3D, Matrix3D_Invert)
    RUN_TEST_CASE(Matrix3D, Matrix3D_Invert2)
}
