//
// Created by Hugo Trippaers on 11/01/2025.
//

#include "Standup.h"

#include "hexapodmath/additional_functions.h"
#include "hexapodmath/forward_kinematics.h"
#include "hexapodmath/inverse_kinematics.h"
#include "hexapodmath/pose.h"
#include "hexapodmath/matrix_3d.h"

void printCoordinate(const float coord[3]) {
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "Coordinate: ("
              << coord[0] << ", "
              << coord[1] << ", "
              << coord[2] << ")"
              << std::endl;
}

void Standup::init() {

}

void Standup::calculate(const Robot &robot, std::array<LegState, 6> &state, const float32_t *movement_vector, float32_t delta_t_ms) {
    for (int i=0; i<6; i++) {

        auto &leg = state[i];

        // Prepare transformations for this leg
        MATRIX4(Tcoxa);
        pose_get_transformation(&leg.coxa_joint_in_body_frame, &Tcoxa);
        MATRIX4(Tcoxa_inv);
        matrix_3d_invert(&Tcoxa, &Tcoxa_inv);

        // Presets
        float32_t p_target[3] = {
            robot.leg[i].tip_starting_position[0],
            robot.leg[i].tip_starting_position[1],
            -100
        };
        float32_t origin[3] = {0, 0, 0};

        // State variables
        float32_t p_current_coxa[3] = {0.0, 0.0, 0.0};
        float32_t p_current[3] = {0.0, 0.0, 0.0};
        float32_t p_next[3] = {0.0, 0.0, 0.0};
        float32_t p_next_coxa[3] = {0.0, 0.0, 0.0};
        float32_t a_next[3] = {0.0, 0.0, 0.0};

        arm_vec_copy_f32(leg.actual_joint_angles, p_current, 3);

        // Actual angles to current world position
        forward_kinematics(p_current, p_current_coxa);
        matrix_3d_vec_transform(&Tcoxa, p_current_coxa, p_current);

        // Calculate the next step
        calculate_motion_step(p_current, p_target, p_next, delta_t_ms);
        matrix_3d_vec_transform(&Tcoxa_inv, p_next, p_next_coxa);
        inverse_kinematics(origin, p_next_coxa, a_next);

        // Copy the joint angles for the next step
        arm_vec_copy_f32(a_next, leg.joint_angles, 3);
    }
}

void Standup::update(std::array<LegState, 6> &state) {

}

void Standup::calculate_motion_step(float32_t current[3], float32_t target[3], float32_t next[3], float32_t delta_t_s) {
    float32_t direction[2];
    float32_t unit_direction[2];
    arm_vec_sub_f32(target, current, direction, 2);
    arm_vec_normalize_f32(direction, unit_direction, 2);

    float32_t remaining_distance = arm_euclidean_distance_f32(current, target, 2);
    float32_t remaining_duration_s = remaining_distance / _velocity;
    printf("Remaining XY distance is %f\n", remaining_distance);

    float32_t movement[2] = {0, 0};
    float32_t max_movement = _velocity * delta_t_s;
    if (remaining_distance < max_movement) {
        max_movement = remaining_distance;
    }
    arm_vec_mult_scalar_f32(unit_direction, max_movement, movement, 2);

    // Compute list distance and duration
    float32_t lift_distance = target[2] - current[2] + _lift_height;
    float32_t lift_duration_s = fabsf(lift_distance) / _lift_velocity;
    float32_t lower_duration_s = _lift_height / _lift_velocity;

    // If double the lift_duration is greater than the total
    // time to move the leg we need to recalculate the lift height
    float32_t calculated_lift_height = _lift_height;
    if ((2 *lift_duration_s ) > remaining_duration_s) {
        calculated_lift_height = calculated_lift_height * (remaining_duration_s / (2 * lift_duration_s));
    }

    // If the time taken to lower the leg is great than the remaining
    // movement we need to adjust the lift_height
    if (lower_duration_s > remaining_duration_s) {
        calculated_lift_height = _lift_height * (remaining_duration_s / lower_duration_s);
    }

    float32_t target_height = target[2] + calculated_lift_height;
    printf("Target height is %f\n", target_height);

    float32_t z = current[2];
    float32_t max_z_movement = fminf(_lift_velocity * delta_t_s, fabsf(target_height - current[2]));

    if (current[2] < target_height) {
        z = fminf(current[2] + max_z_movement, target[2] + calculated_lift_height);
    } else if (current[2] > target_height) {
        z = current[2] - max_z_movement;
    }
    printf("Z-heigth is %f\n", current[2]);
    printf("Next height is %f (max movement %f)\n", z, max_z_movement);

    next[0] = current[0] + movement[0];
    next[1] = current[1] + movement[1];
    next[2] = z;
}
