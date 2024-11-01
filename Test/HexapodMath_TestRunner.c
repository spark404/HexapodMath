#include "unity_fixture.h"

TEST_GROUP_RUNNER(HexapodMath)
{
    RUN_TEST_CASE(HexapodMath, PolarToCartesian)
    RUN_TEST_CASE(HexapodMath, PolarToCartesianQ2)
    RUN_TEST_CASE(HexapodMath, PolarToCartesianQ3)

	RUN_TEST_CASE(HexapodMath, InverseKinematics_Start)
	RUN_TEST_CASE(HexapodMath, InverseKinematics_Extreme)
	RUN_TEST_CASE(HexapodMath, InverseKinematics_Realistic)

	RUN_TEST_CASE(HexapodMath, Matrix3d_Translation)
	RUN_TEST_CASE(HexapodMath, Matrix3d_RotationX)
	RUN_TEST_CASE(HexapodMath, Matrix3d_RotationY)
	RUN_TEST_CASE(HexapodMath, Matrix3d_RotationZ)

	RUN_TEST_CASE(HexapodMath, Pose_Set)
	RUN_TEST_CASE(HexapodMath, Pose_GetTransformationMatrix)
	RUN_TEST_CASE(HexapodMath, Pose_GetTransformationMatrixWithAngles)

    RUN_TEST_CASE(HexapodMath, ForwardKinematicsNoAngles)
    RUN_TEST_CASE(HexapodMath, ForwardKinematicsJoint390)

    RUN_TEST_CASE(HexapodMath, InverseKinematics_To_ForwardKinematics)
    RUN_TEST_CASE(HexapodMath, InverseKinematics_To_ForwardKinematics2)

    RUN_TEST_CASE(HexapodMath, VectorCalculation);
}
