//
// Created by Hugo Trippaers on 11/01/2025.
//

#include "Standup.h"

#include "additional_functions.h"
#include "forward_kinematics.h"
#include "inverse_kinematics.h"
#include "hexapod.h"
#include "pose.h"
#include "matrix_3d.h"

void printCoordinate(const float coord[3]) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Coordinate: ("
              << coord[0] << ", "
              << coord[1] << ", "
              << coord[2] << ")"
              << std::endl;
}

void Standup::init() {

}

void Standup::calculate(const Robot &robot, std::array<LegState, 6> &state, const float32_t *movement_vector, float32_t delta_t_ms) const {
    for (int i=0; i<6; i++) {

        auto &leg = state[i];

        float32_t p_current_coxa[3];
        forward_kinematics(leg.actual_joint_angles, p_current_coxa);

        float32_t p_current[3];

        MATRIX4(Tcoxa);
        pose_get_transformation(&leg.coxa_joint_in_body_frame, &Tcoxa);

        MATRIX4(Tcoxa_inv);
        matrix_3d_invert(&Tcoxa, &Tcoxa_inv);

        matrix_3d_vec_transform(&Tcoxa, p_current_coxa, p_current);

        float32_t p_target[3] = {
                robot.leg[i].tip_starting_position[0],
                robot.leg[i].tip_starting_position[1],
                0
        };

        float32_t direction[2];
        float32_t unit_direction[2];
        arm_vec_sub_f32(p_target, p_current, direction, 2);
        arm_vec_normalize_f32(direction, unit_direction, 2);

        float32_t movement[2];
        arm_vec_mult_scalar_f32(unit_direction, _velocity * delta_t_ms, movement, 2);

        float32_t remaining_distance = arm_euclidean_distance_f32(p_current, p_target, 2);
        float32_t remaining_duration_s = remaining_distance / _velocity;

        // Compute list distance and duration
        float32_t lift_distance = p_target[2] - p_current[2] + _lift_height;
        float32_t lift_duration_s = abs(lift_distance) / _lift_velocity;
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

        float32_t target_height = p_target[2] + calculated_lift_height;

        float32_t z = p_current[2];
        if (p_current[2] < target_height) {
            z = std::min(p_current[2] + _lift_velocity * delta_t_ms, p_current[2] + calculated_lift_height * delta_t_ms);
        } else if (p_current[2] > target_height) {
            z = p_current[2] - _lift_velocity * delta_t_ms;
        }

        float32_t next[3] = {p_current[0] + movement[0], p_current[1] + movement[1], z};

        float32_t next_in_coxa[3];
        matrix_3d_vec_transform(&Tcoxa_inv, next, next_in_coxa);

        float32_t origin[3] = {0, 0, 0};
        arm_vec_copy_f32(leg.joint_angles, leg.prev_joint_angles, 3);
        inverse_kinematics(origin, next_in_coxa, leg.joint_angles);
    }
}

void Standup::update(std::array<LegState, 6> &state) {

}