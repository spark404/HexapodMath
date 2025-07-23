//
// Created by Hugo Trippaers on 26/10/2024.
//

//
// Created by Hugo Trippaers on 25/10/2024.
//

#include "unity_fixture.h"

TEST_GROUP_RUNNER(ForwardKinematics)
{
    RUN_TEST_CASE(ForwardKinematics, ForwardKinematicsNoAngles)
    RUN_TEST_CASE(ForwardKinematics, ForwardKinematicsJoint390)

    RUN_TEST_CASE(ForwardKinematics, Rodriques_0Angle)
    RUN_TEST_CASE(ForwardKinematics, Rodriques_90Angle)
    RUN_TEST_CASE(ForwardKinematics, Rodriques_90Angle50Linear)
}
