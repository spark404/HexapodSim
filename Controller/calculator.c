//
// Created by Hugo Trippaers on 20/07/2025.
//

#include "arm_math.h"
#include "hexapodmath/additional_functions.h"

#include "calculator.h"


/// Calculates the next motion step given a current and target position using the supplied time interval
///
/// @param current The current position
/// @param target The target that should be reached
/// @param delta_t_s The time interval in seconds (or fractions there of)
/// @param next  [out] The next intermediate target given the time interval
/// @param distance_remaining [out] The Euclidean distance remaining to the target position
void calculate_motion_step(motion_param_t *motion, float32_t current[3], float32_t target[3], float32_t delta_t_s, float32_t next[3], float32_t *distance_remaining) {
    float32_t direction[2];
    float32_t unit_direction[2];
    arm_vec_sub_f32(target, current, direction, 2);
    arm_vec_normalize_f32(direction, unit_direction, 2);

    const float32_t remaining_distance_2d = arm_euclidean_distance_f32(current, target, 2);
    const float32_t remaining_duration_s = remaining_distance_2d / motion->velocity;

    float32_t movement[2] = {0, 0};
    float32_t max_movement = motion->velocity * delta_t_s;
    if (remaining_distance_2d < max_movement) {
        max_movement = remaining_distance_2d;
    }
    arm_vec_mult_scalar_f32(unit_direction, max_movement, movement, 2);

    // Compute list distance and duration
    const float32_t lift_distance = target[2] - current[2] + motion->lift_height;
    const float32_t lift_duration_s = fabsf(lift_distance) / motion->lift_velocity;
    const float32_t lower_duration_s = motion->lift_height / motion->lift_velocity;

    // If double the lift_duration is greater than the total
    // time to move the leg we need to recalculate the lift height
    float32_t calculated_lift_height = motion->lift_height;
    if ((2 *lift_duration_s ) > remaining_duration_s) {
        calculated_lift_height = calculated_lift_height * (remaining_duration_s / (2 * lift_duration_s));
    }

    // If the time taken to lower the leg is greater than the remaining
    // movement we need to adjust the lift_height
    if (lower_duration_s > remaining_duration_s) {
        calculated_lift_height = motion->lift_height * (remaining_duration_s / lower_duration_s);
    }

    const float32_t target_height = target[2] + calculated_lift_height;

    float32_t z = current[2];
    const float32_t max_z_movement = fminf(motion->lift_velocity * delta_t_s, fabsf(target_height - current[2]));

    if (current[2] < target_height) {
        z = fminf(current[2] + max_z_movement, target[2] + calculated_lift_height);
    } else if (current[2] > target_height) {
        z = current[2] - max_z_movement;
    }

    next[0] = current[0] + movement[0];
    next[1] = current[1] + movement[1];
    next[2] = z;
    *distance_remaining = arm_euclidean_distance_f32(current, target, 3);
}
