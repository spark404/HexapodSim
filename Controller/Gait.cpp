//
// Created by Hugo Trippaers on 02/11/2024.
//

#include "arm_math.h"

#include "Gait.h"
#include "Robot.h"
#include "hexapodmath/hexapod.h"

void Gait::init() {

}

void Gait::update(std::array<LegState, 6> &state) {
    uint32_t lifted_leg = state[0].grounded ? 1 : 0;
    float32_t remaining_travel = arm_euclidean_distance_f32(state[lifted_leg].tip_target, state[lifted_leg].tip_interpolated_target, 3);
    if (almost_equal(remaining_travel, 0, 0.05)) {
        // Close enough to the destination after the next update, swap legs
        for (int i=0; i<6; i++) {
            state[i].grounded = !state[i].grounded;
        }
    }
}

void Gait::calculate(const Robot &robot, std::array<LegState, 6> &state, const float32_t movement_vector[3], float32_t delta_t_ms) {
    if (movement_vector[0] == 0 && movement_vector[1] == 0) {
        // No movement in the X-Y plane, ground all the legs
        for (auto & i : state) {
            if (i.tip_position_in_world_frame.translation[2] <= 0) {
                i.grounded = true;
            }
        }
        return;
    }

    bool reinit = true;
    for (auto & i : state) {
        if (!i.grounded) {
            reinit = false;
        }
    }

    if (reinit) {
        // These legs remain on the ground
        state[1].grounded = false;
        state[3].grounded = false;
        state[5].grounded = false;
    }

    float32_t paths[6][4][3];
    for (int i = 0; i<state.size(); i++) {
        const LegState &leg_state = state[i];

        if (leg_state.grounded) {
            continue;
        }

        float32_t *p_current = state[i].tip_position_in_body_frame.translation;

        float32_t origin[2] = {robot.leg[i].tip_starting_position[0], robot.leg[i].tip_starting_position[1]};
        float32_t point[2];
        project_point_on_circle(_step_size, origin, movement_vector, point);

        float32_t target[3] = {point[0], point[1], robot.leg[i].tip_starting_position[2]};

        calculate_path(p_current, target, _lift_height, _lift_incline_factor, paths[i]);

        arm_vec_copy_f32(target, state[i].tip_target, 3);
    }

    float32_t longest_path = 0.f;
    for (int i = 0; i<state.size(); i++) {
        LegState &leg_state = state[i];
        if (leg_state.grounded) {
            float32_t movement_vector_grounded[3];
            arm_vec_mult_scalar_f32(movement_vector, -1, movement_vector_grounded, 3);

            float32_t origin[2] = {robot.leg[i].tip_starting_position[0], robot.leg[i].tip_starting_position[1]};
            float32_t point[2];
            project_point_on_circle(_step_size, origin, movement_vector_grounded, point);

            leg_state.tip_target[0] = point[0];
            leg_state.tip_target[1] = point[1];
            leg_state.tip_target[2] = robot.leg[i].tip_starting_position[2];

            float32_t *p_current = leg_state.tip_position_in_body_frame.translation;
            arm_vec_copy_f32(p_current, paths[i][0], 3);
            arm_vec_copy_f32(p_current, paths[i][1], 3);
            arm_vec_copy_f32(p_current, paths[i][2], 3);
            arm_vec_copy_f32(leg_state.tip_target, paths[i][3], 3);

            float32_t path_length = arm_euclidean_distance_f32(p_current, leg_state.tip_target, 3);
            longest_path = fmaxf(path_length, longest_path);
        }
    }

    for (int i = 0; i<state.size(); i++) {
        float32_t velocity = arm_vec_magnitude_f32(movement_vector, 3);
        float32_t substeps = longest_path / velocity;

        float32_t path_length = calculate_path_length(paths[i]);
        float32_t step_length = path_length / substeps;

        float32_t delta[3];
        interpolate(paths[i], step_length, delta);
        arm_vec_copy_f32(delta, state[i].tip_interpolated_target, 3);
    }
}