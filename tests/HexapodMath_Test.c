#include "unity_fixture.h"

#include "hexapodmath/conversion_2d.h"
#include "hexapodmath/inverse_kinematics.h"
#include "hexapodmath/pose.h"
#include "hexapodmath/matrix_3d.h"
#include "hexapodmath/additional_functions.h"
#include "hexapodmath/forward_kinematics.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

TEST_GROUP(HexapodMath);

TEST_SETUP(HexapodMath)
{

}

TEST_TEAR_DOWN(HexapodMath)
{

}

TEST(HexapodMath, PolarToCartesian)
{
	float32_t polar[2] = { 90, D2R(55) };
	float32_t cartesian[2];

	convert_2d_polar_to_cartesian(polar, cartesian);

	TEST_ASSERT_FLOAT_WITHIN(0.001, 51.6219, cartesian[0]);
	TEST_ASSERT_FLOAT_WITHIN(0.001, 73.7237, cartesian[1]);
}

TEST(HexapodMath, PolarToCartesianQ2)
{
    float32_t polar[2] = { 90, D2R(-55) };
    float32_t cartesian[2];

    convert_2d_polar_to_cartesian(polar, cartesian);

    TEST_ASSERT_FLOAT_WITHIN(0.001, 51.6219, cartesian[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001, -73.7237, cartesian[1]);
}

TEST(HexapodMath, PolarToCartesianQ3)
{
    float32_t polar[2] = { 90, D2R(125) };
    float32_t cartesian[2];

    convert_2d_polar_to_cartesian(polar, cartesian);

    TEST_ASSERT_FLOAT_WITHIN(0.001, -51.6219, cartesian[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 73.7237, cartesian[1]);
}


TEST(HexapodMath, Pose_Set)
{
	struct pose pose;
	float32_t translation_actual[3];
	float32_t rotation_actual[3];

	pose_set(&pose, 1, 2, 3, 4, 5, 6);

	pose_get_translation(&pose, translation_actual);
	pose_get_rotation(&pose, rotation_actual);


	TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, translation_actual[0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 2, translation_actual[1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 3, translation_actual[2]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 4, rotation_actual[0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 5, rotation_actual[1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 6, rotation_actual[2]);
}

TEST(HexapodMath, Pose_GetTransformationMatrix)
{
	struct pose pose;

	arm_matrix_instance_f32 actual;
	float32_t pData[4*4];
	arm_mat_init_f32(&actual, 4, 4, pData);

	pose_set(&pose, 5, 3, -7, 0, 0, 0);

	pose_get_transformation(&pose, &actual);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, actual.pData[0 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[0 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[0 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 5, actual.pData[0 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[1 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, actual.pData[1 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[1 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 3, actual.pData[1 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[2 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[2 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, actual.pData[2 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, -7, actual.pData[2 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, actual.pData[3 * 4 + 3]);
}

TEST(HexapodMath, Pose_GetTransformationMatrixWithAngles)
{
	struct pose pose;

	arm_matrix_instance_f32 actual;
	float32_t pData[4*4];
	arm_mat_init_f32(&actual, 4, 4, pData);

	pose_set(&pose, 5, 3, -7, D2R(30.0), D2R(30.0), D2R(30.0));

	pose_get_transformation(&pose, &actual);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.7500000, actual.pData[0 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, -0.2165063, actual.pData[0 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.6250000, actual.pData[0 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 5, actual.pData[0 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.4330127, actual.pData[1 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.8750000, actual.pData[1 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, -0.2165063, actual.pData[1 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 3, actual.pData[1 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, -0.5000000, actual.pData[2 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.4330127, actual.pData[2 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.7500000, actual.pData[2 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, -7, actual.pData[2 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, actual.pData[3 * 4 + 3]);
}

TEST(HexapodMath, Matrix3d_Translation)
{
	arm_matrix_instance_f32 actual;
	float32_t pData[4*4];
	arm_mat_init_f32(&actual, 4, 4, pData);

	float32_t translation[] = { 3.0f, 5.0f, -7.0f};

	matrix_3d_translation_matrix(&actual, translation);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, actual.pData[0 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[0 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[0 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 3, actual.pData[0 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[1 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, actual.pData[1 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[1 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 5, actual.pData[1 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[2 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[2 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, actual.pData[2 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, -7, actual.pData[2 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, actual.pData[3 * 4 + 3]);
}

TEST(HexapodMath, Matrix3d_RotationX)
{
	arm_matrix_instance_f32 actual;
	float32_t pData[4*4];
	arm_mat_init_f32(&actual, 4, 4, pData);

	float32_t omega = 30.0f * (float32_t)M_PI / 180;

	matrix_3d_rotation_x_matrix(&actual, omega);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, actual.pData[0 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[0 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[0 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[0 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[1 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.866025, actual.pData[1 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, -0.5, actual.pData[1 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[1 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[2 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.5, actual.pData[2 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.866025, actual.pData[2 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[2 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, actual.pData[3 * 4 + 3]);
}

TEST(HexapodMath, Matrix3d_RotationY)
{
	arm_matrix_instance_f32 actual;
	float32_t pData[4*4];
	arm_mat_init_f32(&actual, 4, 4, pData);

	float32_t omega = 30.0 * M_PI / 180;

	matrix_3d_rotation_y_matrix(&actual, omega);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.866025, actual.pData[0 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[0 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.5, actual.pData[0 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[0 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[1 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, actual.pData[1 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[1 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[1 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, -0.5, actual.pData[2 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[2 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.866025, actual.pData[2 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[2 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, actual.pData[3 * 4 + 3]);
}

TEST(HexapodMath, Matrix3d_RotationZ)
{
	arm_matrix_instance_f32 actual;
	float32_t pData[4*4];
	arm_mat_init_f32(&actual, 4, 4, pData);

	float32_t omega = 30.0f * (float32_t)M_PI / 180;

	matrix_3d_rotation_z_matrix(&actual, omega);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.866025, actual.pData[0 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, -0.5, actual.pData[0 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[0 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[0 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.5, actual.pData[1 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0.866025, actual.pData[1 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[1 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[1 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[2 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[2 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, actual.pData[2 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[2 * 4 + 3]);

	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 0]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 1]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 0, actual.pData[3 * 4 + 2]);
	TEST_ASSERT_FLOAT_WITHIN(0.0001, 1, actual.pData[3 * 4 + 3]);
}

TEST(HexapodMath, VectorCalculation)
{
    // Reference position for the front right leg
    // standing on the ground with angles 0, 0, pi/2
    float32_t tipVector[4] = {122.7454f, -175.2986f, 0.f, 1};

    struct pose hexapod, body, coxa;
    pose_set(&hexapod, 0, 0, 150, 0, 0, 0);
    pose_set(&body, 0, 0, 0, 0, 0, 0);
    pose_set(&coxa, 51.6219f, -73.7237f, 0, 0, 0, D2R(-55));

    MATRIX4(Thexapod);
    MATRIX4(Tbody);
    MATRIX4(Tcoxa);

    pose_get_transformation(&hexapod, &Thexapod);
    pose_get_transformation(&body, &Tbody);
    pose_get_transformation(&coxa, &Tcoxa);

    MATRIX4(I1);
    arm_mat_mult_f32(&Thexapod, &Tbody, &I1);

    MATRIX4(T);
    arm_mat_mult_f32(&I1, &Tcoxa, &T);

    MATRIX4(Ti);
    arm_mat_inverse_f32(&T, &Ti);

    float32_t actual[4];
    float32_t expected[4] = { 124, 0, -150, 1};
    arm_mat_vec_mult_f32(&Ti, tipVector, actual);

    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.001, expected, actual, 4);
}

TEST(HexapodMath, InverseKinematics_To_ForwardKinematics)
{
    float32_t origin[3] = {0, 0, 0};
    float32_t tip[3] = {124, 0, -150};

    float32_t actual[3];

    inverse_kinematics(origin, tip, actual);

    float32_t expected[3] = {0.f, 0.f, 1.5708f};
    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.0001, expected, actual, 3);

    float32_t actual_coords[3];

    forward_kinematics(actual, actual_coords);

    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.0001, tip, actual_coords, 3);
}

TEST(HexapodMath, InverseKinematics_To_ForwardKinematics2)
{
    float32_t origin[3] = {0, 0, 0};
    float32_t angles[3] = {D2R(10), D2R(-25), D2R(80)};

    float32_t actual_coords[3];
    forward_kinematics(angles, actual_coords);

    float32_t actual[3];
    inverse_kinematics(origin, actual_coords, actual);

    TEST_ASSERT_FLOAT_ARRAY_WITHIN(0.00001, angles, actual, 3);
}
