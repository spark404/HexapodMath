//
// Created by Hugo Trippaers on 01/11/2024.
//
#include "unity_fixture.h"

TEST_GROUP_RUNNER(InverseKinematics) {
    RUN_TEST_CASE(InverseKinematics, InverseKinematics_Start)
    RUN_TEST_CASE(InverseKinematics, InverseKinematics_Extreme)
    RUN_TEST_CASE(InverseKinematics, InverseKinematics_Realistic)
    RUN_TEST_CASE(InverseKinematics, InverseKinematics_RealisticWithOmega)
}
