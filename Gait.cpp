//
// Created by Hugo Trippaers on 02/11/2024.
//

#include "Gait.h"
#include "Robot.h"

#include "arm_math.h"
#include "hexapod.h"

uint32_t almost_equal(float32_t srcA, float32_t srcB, float32_t epsilon);
void calculate_path(float32_t current[3], float32_t target[3], float32_t lift_height, float32_t lift_incline_factor, float32_t path[4][3]);
void interpolate(float32_t path[4][3], float32_t step_size, float32_t next[3]);
float32_t calculate_path_length(float32_t path[4][3]);

void Gait::init() {

}

void Gait::update(const Robot &robot, std::array<LegState, 6> &state, const float32_t movement_vector[3], uint64_t delta_t) {
    uint32_t lifted_leg = state[0].grounded ? 1 : 0;
    float32_t remaining_travel = arm_euclidean_distance_f32(state[lifted_leg].tip_target, state[lifted_leg].tip_interpolated_target, 3);
    if (almost_equal(remaining_travel, 0, 0.05)) {
        // Close enough to the destination after the next update, swap legs
        for (int i=0; i<6; i++) {
            state[i].grounded = !state[i].grounded;
        }
    }
}

void Gait::calculate(const Robot &robot, std::array<LegState, 6> &state, const float32_t movement_vector[3], uint64_t delta_t) {
    if (movement_vector[0] == 0 && movement_vector[1] == 0) {
        // No movement in the X-Y plane, ground all the legs
        for (int i = 0; i<state.size(); i++) {
            if (state[i].tip_position_in_world_frame.translation[2] > 0) {
                delta_tip[i][2] = 0;
            } else {
                state[i].grounded = true;
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

        if (false) {
            for (int k = 0; k < 4; k++) {
                std::cout << "Leg" << i << " path " << k << "(" << paths[i][k][0] << "," << paths[i][k][1] << "," << paths[i][k][2]
                          << ")" << std::endl;
            }
        }
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
        float32_t velocity = arm_vec_magnitude(movement_vector, 3);
        float32_t substeps = longest_path / velocity;

        float32_t path_length = calculate_path_length(paths[i]);
        float32_t step_length = path_length / substeps;

        float32_t delta[3];
        interpolate(paths[i], step_length, delta);
        arm_vec_copy_f32(delta, state[i].tip_interpolated_target, 3);
    }
}

/// Calculate the target for this section. The target on the XY plane is calculated using the direction
/// vector and the magnitude. The section Z target is set with the target_height parameter.
/// \param direction Direction vector, only the XY components will be used
/// \param magnitude Movement magnitude on the XY plane
/// \param lift_incline_factor Factor to set the inline vs travel distance
/// \param target_height The target height for the section
/// \param section [out] result vector
void calculate_section(float32_t direction[3], float32_t magnitude, float32_t lift_incline_factor, float32_t target_height, float32_t section[3]) {
    float32_t norm_vector[3];
    arm_vec_normalize_f32(direction, norm_vector, 3);
    arm_vec_mult_scalar_f32(norm_vector, magnitude / lift_incline_factor, section, 3);
    section[2] = target_height;
}

/// Calculate a path between the current and target position that lifts and lowers the leg.
/// \param current Current position
/// \param target Target position
/// \param lift_height Leg lift height
/// \param lift_incline_factor Factor used to calculate inclination of the lifting and lowering steps
/// \param path [out] The resulting path
void calculate_path(float32_t current[3], float32_t target[3], float32_t lift_height, float32_t lift_incline_factor, float32_t path[4][3]) {
    // Vector from the current point to the target point
    float32_t current_to_target[3];
    arm_vec_sub(target, current, current_to_target, 3);

    // Calculate the height difference between the desired lift height and the current height
    float32_t missing_height = target[2] + lift_height - current[2];
    uint32_t height_reached = almost_equal(missing_height, 0, 0.5f);

    // determine over how much xy distance the leg is lifted and lowered
    float32_t leg_lift_distance_xy = fabsf(missing_height) / lift_incline_factor;
    float32_t leg_lower_distance_xy = lift_height / lift_incline_factor;

    // Only calculate the XY distance, so set blocksize to 2
    float32_t total_distance_xy = arm_vec_magnitude(current_to_target, 2);

    // Give missing height a bit of leeway to account for rounding
    int sub_targets = 0;
    float32_t step_height = lift_height;

    if ((leg_lift_distance_xy + leg_lower_distance_xy) < total_distance_xy && !height_reached) {
        // We have some travelling to do and a change in height as well
        // targets for lifting, lowering and travelling
        sub_targets = 2;
    }
    else {
        // Limited travel distance remaining or we have reached the target height
        // step_height is affected by the remaining distance and the height difference
        sub_targets = 1;
        if (missing_height <= 2) {
            // I don't get this calculation yet
            step_height = fminf(lift_height - ((leg_lift_distance_xy + leg_lower_distance_xy - total_distance_xy) / 2) * lift_incline_factor, lift_height);
        } else {
            step_height = fminf(total_distance_xy * lift_incline_factor, lift_height);
        };
    };

    float32_t sections[3][3] = { {0,0,0}, {0,0,0}, {0,0,0}};

    float32_t current_to_target_xy[3];
    arm_vec_copy_f32(current_to_target, current_to_target_xy, 2);
    current_to_target_xy[2] = 0.0f;

    // Calculate the lowering section
    calculate_section(current_to_target_xy, step_height, lift_incline_factor, -1 * step_height, sections[2]);

    if (sub_targets == 2) {
        // Calculate the lifting section
        calculate_section(current_to_target_xy, fabsf(missing_height), lift_incline_factor, missing_height, sections[0]);
    }

    // Calculate the travel section
    for (int i=0; i<3; i++) {
        sections[1][i] = (target[i] - sections[2][i]) - (current[i] + sections[0][i]);
    }

    // Create a path with target postions using the sections
    for (int i=0; i<3; i++) {
        path[0][i] = current[i];
        path[1][i] = current[i] + sections[0][i];
        path[2][i] = current[i] + sections[0][i] + sections[1][i];
        path[3][i] = target[i];
    }
}

/// Determine the next intermediate target on this path using the current step size
/// \param path
/// \param step_size
/// \param next
void interpolate(float32_t path[4][3], float32_t step_size, float32_t next[3]) {
    float32_t path_length = 0.0f;
    float32_t path_target[3] = {path[0][0], path[0][1], path[0][2]};

    for (int i=1; i<4; i++) {
        float32_t segment[3];
        arm_vec_sub(path[i], path[i-1], segment, 3);

        float32_t segment_length = arm_vec_magnitude(segment, 3);

        if (path_length + segment_length > step_size) {
            float32_t factor = (step_size - path_length) / segment_length;
            float32_t travel_step[3];
            arm_vec_mult_scalar_f32(segment, factor, travel_step, 3);
            arm_vec_add(path_target, travel_step, next, 3);
            return;
        }

        arm_vec_add(path_target, segment, path_target, 3);
        path_length += segment_length;
    }
}

uint32_t almost_equal(float32_t srcA, float32_t srcB, float32_t epsilon) {
    return fabs(srcA - srcB) < epsilon;
}

float32_t calculate_path_length(float32_t path[4][3]) {
    float32_t accum = 0.f;
    for (uint32_t i=1; i<4; i++) {
        accum += arm_euclidean_distance_f32(path[i], path[i-1], 3);
    }
    return accum;
}