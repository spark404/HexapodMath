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
 void forward_kinematics(float32_t angles[3], float32_t coordinates[3]);

void rodrigues_rotation_matrix(float32_t twist[6], float32_t theta, arm_matrix_instance_f32 *T);

#ifdef __cplusplus
}
#endif

#endif //HEXAPOD_FORWARD_KINEMATICS_H
