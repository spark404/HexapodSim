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

void Standup::init() {

}

void Standup::calculate(const Robot &robot, std::array<LegState, 6> &state, const float32_t *movement_vector) const {
    for (int i=0; i<6; i++) {

        auto &leg = state[i];

        float32_t p_current_coxa[3];
        forward_kinematics(leg.prev_joint_angles, p_current_coxa);

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

        float32_t path[4][3];
        calculate_path(p_current, p_target, _lift_height, _lift_incline_factor, path);

        float32_t path_length = calculate_path_length(path);
        float32_t step_length = path_length / (path_length / 0.05f);

        float32_t next[3];
        interpolate(path, step_length, next);

        float32_t next_in_coxa[3];
        matrix_3d_vec_transform(&Tcoxa_inv, next, next_in_coxa);

        float32_t origin[3] = {0, 0, 0};
        arm_vec_copy_f32(leg.joint_angles, leg.prev_joint_angles, 3);
        inverse_kinematics(origin, next_in_coxa, leg.joint_angles);
    }
}

void Standup::update(std::array<LegState, 6> &state) {

}