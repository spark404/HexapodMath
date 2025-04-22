//
// Created by Hugo Trippaers on 20/10/2024.
//

#include "hexapodmath/forward_kinematics.h"

#include "hexapodmath/additional_functions.h"

void rodrigues_rotation_matrix(float32_t twist[6], float32_t theta, arm_matrix_instance_f32 *T) {
    MATRIX(omega_hat, 3);
    arm_vec_skew_matrix_f32(twist, &omega_hat);

    MATRIX(I, 3);
    arm_mat_identity_f32(&I);

    // e_omega_hat_theta = I + sin(theta) * omega_hat + (1 - cos (theta)) * omega_hat^2 ;
    float32_t sin_theta;
    float32_t cos_theta;
    arm_sin_cos_f32(R2D(theta), &sin_theta, &cos_theta);

    MATRIX(stoh, 3);
    arm_mat_mult_scalar_f32(sin_theta, &omega_hat, &stoh);

    MATRIX(omega_hat_sq, 3);
    arm_mat_mult_f32(&omega_hat, &omega_hat, &omega_hat_sq);

    MATRIX(ctohsq, 3);
    arm_mat_mult_scalar_f32(1 - cos_theta, &omega_hat_sq, &ctohsq);

    MATRIX(t1, 3);
    arm_mat_add_f32(&I, &stoh, &t1);

    MATRIX(e_omega_hat_theta, 3);
    arm_mat_add_f32(&t1, &ctohsq, &e_omega_hat_theta);

    // t = (I - e_omega_hat_theta)*(cross(w,v)) + w * transpose(w) * v * theta;
    arm_mat_sub_f32(&I, &e_omega_hat_theta, &t1);
    float32_t wxv[3];
    arm_vec_cross_f32(&twist[0], &twist[3], wxv, 3);

    float32_t v1[3];
    arm_mat_vec_mult_f32(&t1, wxv, v1);

    arm_matrix_instance_f32 w_column;
    arm_matrix_instance_f32 w_row;
    arm_mat_init_f32(&w_column, 3, 1, twist);
    arm_mat_init_f32(&w_row, 1, 3, twist);

    MATRIX(t3, 3);
    arm_mat_mult_f32(&w_column, &w_row, &t3);

    float32_t t4[3];
    arm_mat_vec_mult_f32(&t3, &twist[3], t4);

    float32_t t5[3];
    for (uint32_t i = 0; i < 3; i++) {
        t5[i] = t4[i] * theta;
    }

    float32_t t[3];
    arm_vec_add_f32(v1, t5, t, 3);

    arm_mat_transformation_matrix_f32(&e_omega_hat_theta, t, T);
}

void forward_kinematics(float32_t angles[3], float32_t coordinates[3]) {
    /* Basic configuration of the robot
     */
    float32_t l1 = 24.f; // mm
    float32_t l2 = 100.f; // mm
    float32_t l3 = 150.f; // mm

    /* Matrix M is the matrix describing the transformation of the end effector (the leg tip) in
     * the source frame S or {0} frame with all joint angles at their 0 position.
     * In this case the identity matrix with a linear transformation over the X axis of the total
     * length to the legs together.
     */
    MATRIX(M, 4);
    M.pData[0 * 4 + 0] = 1.f;
    M.pData[0 * 4 + 1] = 0.f;
    M.pData[0 * 4 + 2] = 0.f;
    M.pData[0 * 4 + 3] = l1 + l2 + l3;

    M.pData[1 * 4 + 0] = 0.f;
    M.pData[1 * 4 + 1] = 1.f;
    M.pData[1 * 4 + 2] = 0.f;
    M.pData[1 * 4 + 3] = 0.f;

    M.pData[2 * 4 + 0] = 0.f;
    M.pData[2 * 4 + 1] = 0.f;
    M.pData[2 * 4 + 2] = 1.f;
    M.pData[2 * 4 + 3] = 0.f;

    M.pData[3 * 4 + 0] = 0.f;
    M.pData[3 * 4 + 1] = 0.f;
    M.pData[3 * 4 + 2] = 0.f;
    M.pData[3 * 4 + 3] = 1.f;

    /* Joint 1: Rotation over the Z axis, no linear velocity on the point of origin
     */
    float32_t S1[6] = {0, 0, 1, 0, 0, 0};

    /* Joint 2: Rotation over the Y axis, linear velocity on the point of origin
     * of 1rad/s * l1 on the z axis
     */
    float32_t S2[6] = {0, 1, 0, 0, 0, l1};

    /* Joint 2: Rotation over the Y axis, linear velocity on the point of origin
     * of 1rad/s * (l1 + l2) on the z axis
    */
    float32_t S3[6] = {0, 1, 0, 0, 0, l1 + l2};

    MATRIX4(G1);
    rodrigues_rotation_matrix(S1, angles[0], &G1);

    MATRIX4(G2);
    rodrigues_rotation_matrix(S2, angles[1], &G2);

    MATRIX4(G3);
    rodrigues_rotation_matrix(S3, angles[2], &G3);

    MATRIX4(tmp1);
    MATRIX4(tmp2);
    MATRIX4(T);
    arm_mat_mult_f32(&G1, &G2, &tmp1);
    arm_mat_mult_f32(&tmp1, &G3, &tmp2);
    arm_mat_mult_f32(&tmp2, &M, &T);

    float32_t coordinates_vector[4];

    float32_t origin_vector[4] = {0, 0, 0, 1};
    arm_mat_vec_mult_f32(&T, origin_vector, coordinates_vector);

    coordinates[0] = coordinates_vector[0];
    coordinates[1] = coordinates_vector[1];
    coordinates[2] = coordinates_vector[2];
}



