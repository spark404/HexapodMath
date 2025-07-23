//
// Created by Hugo Trippaers on 20/10/2024.
//

#ifndef HEXAPOD_FORWARD_KINEMATICS_H
#define HEXAPOD_FORWARD_KINEMATICS_H

#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Given a set of joint angles and a robot configuration, calculate the location of
 * the tip with the base of the coxa as origin.
 *
 * @param angles [in] array of float32_t with angles for the joints
 * @param coordinates [out] array with the tip coordinates
 */
 void forward_kinematics(const float32_t angles[3], float32_t coordinates[3]);

 /**
 * @brief Computes the homogeneous transformation matrix for a screw motion using Rodrigues' rotation formula.
 *
 * This function generates a 4x4 homogeneous transformation matrix representing the rotation and translation
 * induced by moving along a screw axis (represented as a twist, [w, v]), for a distance (angle) theta.
 *
 * The function applies Rodrigues' formula to compute the rotation matrix component and combines it with the
 * translation component per twist-exponential coordinates.
 *
 * @param twist   6D float32 array. First 3 entries are the rotation axis (omega, w), last 3 are the translation axis (v).
 * @param theta   The scalar displacement along the screw axis (in radians).
 * @param T       Output. A 4x4 arm_matrix_instance_f32 which will contain the computed homogeneous transform.
 *
 * The computation is as follows:
 *   - Computes the skew-symmetric matrix of w ("omega_hat").
 *   - Calculates the rotation component using Rodrigues' rotation formula:
 *         R = I + sin(theta) * omega_hat + (1 - cos(theta)) * omega_hat^2
 *   - Computes the translation component:
 *         t = (I - R) * (w x v) + (w * w^T * v) * theta
 *   - Combines these into a 4x4 transformation matrix T:
 *         | R  t |
 *         | 0  1 |
 *
 * Assumes all matrices/vectors are float32. Uses helper matrix/vector functions for all math operations.
 *
 * @note This function is commonly used in robotics and computer vision for modeling rigid-body motions with exponential coordinates.
 */
void rodrigues_rotation_matrix(float32_t twist[6], float32_t theta, arm_matrix_instance_f32 *T);

#ifdef __cplusplus
}
#endif

#endif //HEXAPOD_FORWARD_KINEMATICS_H
