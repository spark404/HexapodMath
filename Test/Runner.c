//
// Created by Hugo Trippaers on 01/11/2024.
//

#include "unity_fixture.h"

static void runAllTests(void)
{
    RUN_TEST_GROUP(InverseKinematics)
    RUN_TEST_GROUP(AdditionalFunctions)
    RUN_TEST_GROUP(ForwardKinematics)
    RUN_TEST_GROUP(HexapodMath)
}

int main(int argc, const char *argv[]) {
    return UnityMain(argc, argv, runAllTests);
}