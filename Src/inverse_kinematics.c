#include "inverse_kinematics.h"

#include "additional_functions.h"

void inverse_kinematics(const float32_t *origin, const float32_t *tip, float32_t *angles)
{
    // TODO This should come from a robot definition file
    float32_t l1 = 24; // mm
    float32_t l2 = 100; // mm
    float32_t l3 = 150; // mm

    float32_t distance = arm_euclidean_distance_f32(origin, tip, 3);
    if (distance > (l1 + l2 + l3)) {
        // TODO we should return an error
        return;
    }

	float32_t delta_x = tip[0] - origin[0];
	float32_t delta_y = tip[1] - origin[1];
	float32_t delta_z = tip[2] - origin[2];

	float32_t l2_sq = l2 * l2;
	float32_t l3_sq = l3 * l3;

	/* We can calculate the hip (tibia) angle using the Z and X
	 * difference, giving us a right sides triangle to work with.
	 */
	float32_t omega;
	arm_atan2_f32(delta_y, delta_x, &omega);

	/* The remainder of the calculations should exclude the coxa length.
	 * So we define a new origin for the calculations using the omega angle
	 * and the coxa length. As the Y coordinate is 0 we can skip that part of
	 * the formula in the rotation.
	 */
    float32_t coxa_cos;
    float32_t coxa_sin;
    arm_sin_cos_f32(R2D(omega), &coxa_sin, &coxa_cos);

	float32_t femur_origin[3];
	femur_origin[0] = origin[0] + l1 * coxa_cos;
	femur_origin[1] = origin[1] + l1 * coxa_sin;
	femur_origin[2] = origin[2];

    delta_x = tip[0] - femur_origin[0];
    delta_y = tip[1] - femur_origin[1];
    delta_z = tip[2] - femur_origin[2];

    /* We know three things
     *  - the distance from origin to target (pythagoras), side c
     *  - the height difference between origin and target, side a
     *  - the corner opposite to the distance is 90 deg
     */
	float32_t c = arm_euclidean_distance_f32(femur_origin, tip, 3);

	// using the distance we can use the pythagoras theorem to compute the remaining side b
	float32_t w;
	arm_sqrt_f32(c * c - delta_z * delta_z, &w);

	float32_t alpha_accent;
    arm_atan2_f32(delta_z, w, &alpha_accent);

	/* Use the law of cosines to find the remaining angles
	 * as we know all sides, but none of the angles
	 * c**2 = a**2 + b**2 - 2ab * cos C
	 */
    float32_t c_sq = c * c;
	float32_t alpha = acosf((l2_sq + c_sq - l3_sq) / (2 * l2 * c));
	float32_t beta = acosf((l3_sq + l2_sq - c_sq) / (2 * l2 * l3));

	angles[0] = omega; // Joint 1 body <-> coxa
	angles[1] = -(alpha + alpha_accent); // Joint 2 coxa <-> femur
	angles[2] = (float32_t)M_PI - beta;  // Joint 3 femur <-> tibia
}
