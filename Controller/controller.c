/*
 * SPDX-FileCopyrightText: 2025 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "controller.h"

#include <stdbool.h>
#include "robot_config.h"
#include "calculator.h"
#include "controller_config.h"
#include "controller_math.h"
#include "hexapodmath/additional_functions.h"
#include "hexapodmath/conversion_2d.h"
#include "hexapodmath/forward_kinematics.h"
#include "hexapodmath/hexapod.h"
#include "hexapodmath/inverse_kinematics.h"
#include "hexapodmath/matrix_3d.h"
#include "hexapodmath/pose.h"
#include "log.h"

void controller_init(controller_ctx_t *ctx) {
    ctx->cfg = &r;
    ctx->state = CTRL_BOOT;
    ctx->next_state = CTRL_BOOT;
    ctx->powerdown_timeout = CTRL_POWERDOWN_TIMEOUT;
    ctx->pending_yaw = 0.0f;
    ctx->on_state_change = NULL;

    // Do a bunch of static calculations that depend on the robot configuration in robot.h
    pose_set(&ctx->robot.hexapod, 0, 0, 0, 0, 0, 0);
    pose_set(&ctx->robot.body, 0, 0, 150, 0, 0, 0);

    MATRIX4(Thexapod);
    MATRIX4(Tbody);

    pose_get_transformation(&ctx->robot.hexapod, &Thexapod);
    pose_get_transformation(&ctx->robot.body, &Tbody);

    MATRIX4(Thexapod_body);
    arm_mat_mult_f32(&Thexapod, &Tbody, &Thexapod_body);

    for (int i = 0; i < 6; i++) {
        float32_t mount_point_xy[2];
        const struct leg *current_leg = &ctx->cfg->leg[i];
        struct leg_state *current_leg_state = &ctx->robot.leg_state[i];

        arm_mat_init_f32(&current_leg_state->coxa_mat, 4, 4, current_leg_state->coxa_mat_data);
        arm_mat_init_f32(&current_leg_state->coxa_mat_inv, 4, 4, current_leg_state->coxa_mat_inv_data);

        convert_2d_polar_to_cartesian(current_leg->mount_point_polar, mount_point_xy);
        pose_set(&current_leg_state->coxa_body_joint,
                 mount_point_xy[0], mount_point_xy[1], 0.0f,
                 0.0f, 0.0f, current_leg->mount_point_polar[1]);

        pose_get_transformation(&current_leg_state->coxa_body_joint, &current_leg_state->coxa_mat);
        matrix_3d_invert(&current_leg_state->coxa_mat, &current_leg_state->coxa_mat_inv);

        MATRIX4(T);
        arm_mat_mult_f32(&Thexapod_body, &current_leg_state->coxa_mat, &T);

        float32_t tip_in_coxa[3];
        forward_kinematics(current_leg->tip_home_angles, tip_in_coxa);
        matrix_3d_vec_transform(&T, tip_in_coxa, current_leg_state->tip_home);

        current_leg_state->grounded = 1; // All legs assumed to be grounded, STANDUP will take care of that
    }
}

void controller_set_state_callback(
    controller_ctx_t *ctx,
    controller_state_cb_t cb,
    void *user_data
) {
    ctx->on_state_change = cb;
    ctx->cb_user_data = user_data;
}

void swap_legs(controller_ctx_t *ctx) {
    for (int i = 0; i < 6; i++) {
        struct leg_state *current_leg_state = &ctx->robot.leg_state[i];
        current_leg_state->grounded = !current_leg_state->grounded;
    }
}

void tripod_init_gait(controller_ctx_t *ctx, const arm_matrix_instance_f32 *Thexapod_body) {
    // Reinitialize the gait when all legs are on the ground at the same time
    uint8_t re_init = 1;
    for (int i = 0; i < 6; i++) {
        if (!ctx->robot.leg_state[i].grounded) {
            re_init = 0;
        }
    }

    if (re_init) {
        LOG_INFO("(Re)Initializing tripod gait");
        ctx->robot.leg_state[1].grounded = 0;
        ctx->robot.leg_state[3].grounded = 0;
        ctx->robot.leg_state[5].grounded = 0;

        // Use the actual angles to determine the current world coordinates of the tip
        for (int i = 0; i < 6; i++) {
            struct leg_state *current_leg_state = &ctx->robot.leg_state[i];

            MATRIX4(T);
            arm_mat_mult_f32(Thexapod_body, &current_leg_state->coxa_mat, &T);

            float32_t tip_in_coxa[3];
            forward_kinematics(current_leg_state->actual_joint_angles, tip_in_coxa);
            matrix_3d_vec_transform(&T, tip_in_coxa, current_leg_state->tip_world_coordinates);
        }
    }
}

/**
 * Calculates the current position of leg in the body frame using
 * the actual joint angles and forward kinematics.
 *
 * @param current_leg_state
 * @param p_current_in_body_frame
 */
void leg_current_position_body(struct leg_state *current_leg_state, float32_t p_current_in_body_frame[3]) {
    float32_t p_current_in_coxa_frame[3];
    forward_kinematics(current_leg_state->actual_joint_angles, p_current_in_coxa_frame);
    matrix_3d_vec_transform(&current_leg_state->coxa_mat, p_current_in_coxa_frame, p_current_in_body_frame);
}

/**
 * Generate a path from the origin to a projected destination on the radius using
 * the direction from arc_disp.
 *
 * @param ctx
 * @param p_current_in_body_frame
 * @param step_size
 * @param origin
 * @param arc_disp
 * @param path
 * @return
 */
float32_t leg_generate_path(float32_t body_z, float32_t p_current_in_body_frame[3], float32_t step_size,
                            float32_t origin[2], float32_t arc_disp[2], bool with_lift, float32_t path[4][3]) {
    float32_t point[2];
    project_point_on_circle(step_size, origin, arc_disp, point);
    float32_t p_target_in_body_frame[3] = {point[0], point[1], body_z * -1};

    if (with_lift) {
        calculate_path(p_current_in_body_frame, p_target_in_body_frame, 25, 2.0f, path);
    } else {
        arm_vec_copy_f32(p_current_in_body_frame, path[0], 3);
        arm_vec_copy_f32(p_current_in_body_frame, path[1], 3);
        arm_vec_copy_f32(p_current_in_body_frame, path[2], 3);
        arm_vec_copy_f32(p_target_in_body_frame, path[3], 3);
    }

    return arm_euclidean_distance_f32(p_current_in_body_frame, p_target_in_body_frame,
                                      3);
}

void controller_update(controller_ctx_t *ctx, const controller_attitude_t *attitude, const controller_command_t *cmd, float32_t dt_s) {
    (void)attitude;

    MATRIX4(Thexapod);
    MATRIX4(Tbody);
    MATRIX4(Thexapod_body);

    if (dt_s < 1e-6f) {
        return;
    }

    // Update the control values, might be changed from SPI interface
    float32_t velocity = cmd->velocity;
    float32_t height = cmd->height;
    float32_t heading = cmd->heading;

    if (ctx->state == CTRL_WALKING || ctx->state == CTRL_STANDING) {
        if (height != ctx->robot.body.translation[2]) {
            ctx->robot.body.translation[2] = height; // Mirrors the height set in the target
        }
    }

    float32_t yaw = ctx->robot.hexapod.rotation[2];
    ctx->yaw_error = wrap_to_pi(heading - yaw);

    pose_get_transformation(&ctx->robot.hexapod, &Thexapod);
    pose_get_transformation(&ctx->robot.body, &Tbody);
    arm_mat_mult_f32(&Thexapod, &Tbody, &Thexapod_body);

    bool wants_translation = fabsf(velocity) > CTRL_EPS_VEL;
    bool wants_rotation = fabsf(ctx->yaw_error) > CTRL_EPS_ANG;

    // Rules for transitions
    if (ctx->state == CTRL_BOOT) {
        ctx->next_state = CTRL_SYNCING;
    }

    if (ctx->state == CTRL_STANDING) {
        if (ctx->powerdown_timeout <= 0.0f) {
            ctx->next_state = CTRL_POWERDOWN;
        } else {
            ctx->powerdown_timeout = ctx->powerdown_timeout - dt_s;
        }
    } else {
        ctx->powerdown_timeout = CTRL_POWERDOWN_TIMEOUT;
    }

    if (ctx->state == CTRL_POWERDOWN && (wants_translation || wants_rotation)) {
        ctx->next_state = CTRL_SYNCING;
    }

    if (ctx->state == CTRL_STANDING) {
        // Rotation without translation is a special movement case
        // with separate control logic. Rotation with translation is
        // part of the walking sequence.
        if (wants_rotation && !wants_translation) {
            ctx->next_state = CTRL_ROTATING;
        } else if (wants_translation) {
            ctx->next_state = CTRL_WALKING;
        }
    }

    if (ctx->state == CTRL_WALKING && !wants_translation && !wants_rotation) {
        ctx->next_state = CTRL_STANDING;
    }

    if (ctx->state == CTRL_ROTATING && !wants_rotation) {
        ctx->next_state = CTRL_STANDING;
    }

    if (ctx->state == CTRL_ROTATING && wants_translation) {
        ctx->next_state = CTRL_WALKING;
    }

    // State machine
    if (ctx->state != ctx->next_state) {
        LOG_INFO("Transitioning to motion state %s", controller_state_to_string(ctx->next_state));
        controller_state_t from = ctx->state;
        controller_state_t to = ctx->next_state;

        ctx->state = to;

        // Initialize next_joint_angles from actual position when entering STANDUP
        // to ensure smooth trajectory from current position
        if (to == CTRL_STANDUP) {
            for (int i = 0; i < 6; i++) {
                struct leg_state *leg = &ctx->robot.leg_state[i];
                leg->next_joint_angles[0] = leg->actual_joint_angles[0];
                leg->next_joint_angles[1] = leg->actual_joint_angles[1];
                leg->next_joint_angles[2] = leg->actual_joint_angles[2];
            }
        }

        if (ctx->on_state_change) {
            ctx->on_state_change(ctx, from, to, ctx->cb_user_data);
        }
    }

    if (ctx->state == CTRL_SYNCING) {
        // Make sure actual and next angles are set to the same value
        // Check if we have valid joint data (not all zeros)
        int has_valid_data = 0;
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 3; j++) {
                if (fabsf(ctx->robot.leg_state[i].actual_joint_angles[j]) > 0.01f) {
                    has_valid_data = 1;
                    break;
                }
            }
            if (has_valid_data) break;
        }

        if (!has_valid_data) {
            LOG_DEBUG("SYNC: Waiting for valid joint data...");
            return; // Stay in SYNCING until we get real data
        }

        for (int i = 0; i < 6; i++) {
            arm_vec_copy_f32(ctx->robot.leg_state[i].actual_joint_angles,
                             ctx->robot.leg_state[i].next_joint_angles, 3);
            if (i == 0) {
                LOG_DEBUG("SYNC Leg 0 angles: %.3f, %.3f, %.3f",
                    ctx->robot.leg_state[i].actual_joint_angles[0],
                    ctx->robot.leg_state[i].actual_joint_angles[1],
                    ctx->robot.leg_state[i].actual_joint_angles[2]);
            }
        }
        ctx->next_state = CTRL_STANDUP;
    } else if (ctx->state == CTRL_STANDUP) {
        int ready = 1;
        motion_param_t motion_param = {CTRL_DEFAULT_VEL, CTRL_LIFT_VEL, CTRL_LIFT_Z};
        // Perform the standup routine, follows on SYNCING
        for (int i = 0; i < 6; i++) {
            struct leg_state *current_leg_state = &ctx->robot.leg_state[i];

            // Target position of each leg after standup
            float32_t p_target_in_body_frame[3] = {
                current_leg_state->tip_home[0],
                current_leg_state->tip_home[1],
                -CTRL_BODY_Z
            };

            // Use last commanded position (next_joint_angles) instead of actual position
            // to ensure smooth, continuous trajectory that the servo task can track
            float32_t p_current_in_coxa_frame[3];
            forward_kinematics(current_leg_state->actual_joint_angles, p_current_in_coxa_frame);

            float32_t p_current_in_body_frame[3];
            matrix_3d_vec_transform(&current_leg_state->coxa_mat, p_current_in_coxa_frame, p_current_in_body_frame);

            float32_t p_next_in_body_frame[3];
            float32_t p_next_in_coxa_frame[3];
            float32_t distance_remaining;
            calculate_motion_step(&motion_param, p_current_in_body_frame, p_target_in_body_frame,
                                  dt_s, p_next_in_body_frame, &distance_remaining);

            float32_t origin[3] = {0.0f, 0.0f, 0.0f};
            matrix_3d_vec_transform(&current_leg_state->coxa_mat_inv, p_next_in_body_frame, p_next_in_coxa_frame);
            inverse_kinematics(origin, p_next_in_coxa_frame, current_leg_state->next_joint_angles);

            float32_t p_target_in_coxa_frame[3];
            float32_t target_joint_angles[3];
            matrix_3d_vec_transform(&current_leg_state->coxa_mat_inv, p_target_in_body_frame, p_target_in_coxa_frame);
            inverse_kinematics(origin, p_target_in_coxa_frame, target_joint_angles);

            if (distance_remaining > CTRL_CLOSE_THRESH) {
                ready = 0;
            };

            if (i==0) {
                LOG_DEBUG("C: %.3f, %.3f, %.3f; T: %.3f, %.3f, %.3f",
                    p_current_in_body_frame[0], p_current_in_body_frame[1], p_current_in_body_frame[2],
                    p_target_in_body_frame[0], p_target_in_body_frame[1], p_target_in_body_frame[2]);
                LOG_DEBUG("A: %.3f, %.3f, %.3f; N: %.3f, %.3f, %.3f; T: %.3f, %.3f, %.3f",
                    current_leg_state->actual_joint_angles[0], current_leg_state->actual_joint_angles[1], current_leg_state->actual_joint_angles[2],
                    current_leg_state->next_joint_angles[0], current_leg_state->next_joint_angles[1], current_leg_state->next_joint_angles[2],
                    target_joint_angles[0], target_joint_angles[1], target_joint_angles[2]);
            }
        }

        if (ready) {
            for (int i = 0; i < 6; i++) {
                struct leg_state *leg = &ctx->robot.leg_state[i];
                leg->grounded = true;
            }

            ctx->robot.body.translation[2] = CTRL_BODY_Z; // Mirrors the height set in the target
            pose_get_transformation(&ctx->robot.hexapod, &Thexapod);
            pose_get_transformation(&ctx->robot.body, &Tbody);
            arm_mat_mult_f32(&Thexapod, &Tbody, &Thexapod_body);

            ctx->next_state = CTRL_STANDING;
        }
    } else if (ctx->state == CTRL_WALKING) {
        // Ensure tripod consistency (only re-init if all legs grounded)
        tripod_init_gait(ctx, &Thexapod_body);

        LOG_DEBUG("Actual: %5.2f, %5.2f, %5.2f", ctx->robot.leg_state[1].actual_joint_angles[0], ctx->robot.leg_state[1].actual_joint_angles[1], ctx->robot.leg_state[1].actual_joint_angles[2] );
        LOG_DEBUG("Next: %5.2f, %5.2f, %5.2f", ctx->robot.leg_state[1].next_joint_angles[0], ctx->robot.leg_state[1].next_joint_angles[1], ctx->robot.leg_state[1].next_joint_angles[2] );

        // 1) Compute desired rotation for this timestep
        float32_t desired_omega = ctx->yaw_error / dt_s;
        desired_omega = clampf(desired_omega, -CTRL_MAX_YAW_RATE, +CTRL_MAX_YAW_RATE);
        float32_t rotation_step = desired_omega * dt_s;

        // 2a) World-frame desired motion (commanded)
        float32_t v_world[2] = {
            velocity * arm_cos_f32(yaw),
            velocity * arm_sin_f32(yaw)
        };

        // 2b) Convert world-frame velocity to body frame
        float32_t v_body[2] = {
            v_world[0] * arm_cos_f32(-yaw) - v_world[1] * arm_sin_f32(-yaw),
            v_world[0] * arm_sin_f32(-yaw) + v_world[1] * arm_cos_f32(-yaw)
        };

        // 2c) Create a vector describing the movement
        float32_t movement_vector[3] = {
            v_body[0] * dt_s,
            v_body[1] * dt_s,
            0.0f
        };

        // 2d) Movement_vector is BODY frame, convert to world
        float32_t yaw_mid = yaw + 0.5f * rotation_step;

        float32_t dx_world =
                movement_vector[0] * arm_cos_f32(yaw_mid) -
                movement_vector[1] * arm_sin_f32(yaw_mid);

        float32_t dy_world =
                movement_vector[0] * arm_sin_f32(yaw_mid) +
                movement_vector[1] * arm_cos_f32(yaw_mid);

        // 3a) Update world frame of the robot
        ctx->robot.hexapod.translation[0] += dx_world;
        ctx->robot.hexapod.translation[1] += dy_world;
        ctx->robot.hexapod.rotation[2] += rotation_step;

        // 3b) Update the translations so they are performed with respect to the new location
        pose_get_transformation(&ctx->robot.hexapod, &Thexapod);
        pose_get_transformation(&ctx->robot.body, &Tbody);
        arm_mat_mult_f32(&Thexapod, &Tbody, &Thexapod_body);

        // 5) Generate paths for each leg
        float32_t longest_path = 0.f;
        float32_t longest_lifted_path = 0.f;
        float32_t remaining_path_length = 0.f;
        float32_t paths[6][4][3];
        float32_t body_z = ctx->robot.body.translation[2];
        for (int i = 0; i < 6; i++) {
            struct leg_state *leg = &ctx->robot.leg_state[i];

            float32_t origin[2] = {leg->tip_home[0], leg->tip_home[1]};

            float32_t p_current[3];
            leg_current_position_body(leg, p_current);

            float32_t r[2] = {
                p_current[0],
                p_current[1]
            };

            // Perpendicular vector (z × r)
            float32_t r_perp[2] = {
                -r[1],
                r[0]
            };

            // Rotation displacement for this timestep
            float32_t rot_disp[2] = {
                r_perp[0] * rotation_step,
                r_perp[1] * rotation_step
            };

            float32_t max_rot_disp = CTRL_MAX_ROT_FOOT_MM;
            float32_t rot_mag = hypotf(rot_disp[0], rot_disp[1]);
            if (rot_mag > max_rot_disp) {
                rot_disp[0] *= max_rot_disp / rot_mag;
                rot_disp[1] *= max_rot_disp / rot_mag;
            }


            if (leg->grounded) {
                float32_t arc_disp[2] = {
                    -movement_vector[0] + rot_disp[0],
                    -movement_vector[1] + rot_disp[1]
                };

                float32_t path_length = leg_generate_path(
                    body_z,
                    p_current,
                    CTRL_LEG_RADIUS_MM,
                    origin,
                    arc_disp,
                    false,
                    paths[i]
                );

                longest_path = fmaxf(longest_path, path_length);
            } else {
                float32_t arc_disp[2] = {
                    movement_vector[0] + rot_disp[0],
                    movement_vector[1] + rot_disp[1]
                };

                float32_t path_length = leg_generate_path(
                    body_z,
                    p_current,
                    CTRL_LEG_RADIUS_MM,
                    origin,
                    arc_disp,
                    true,
                    paths[i]
                );

                longest_lifted_path = fmaxf(longest_lifted_path, path_length);
            }
        }

        longest_path = fminf(longest_path, longest_lifted_path);

        // Some form of clamping?
        float32_t max_r = 0.0f;
        for (int i = 0; i < 6; i++) {
            float32_t r = arm_euclidean_distance_f32(
                ctx->robot.leg_state[i].tip_home,
                ctx->robot.body.translation,
                2
            );
            max_r = fmaxf(max_r, r);
        }

        // 6) Execute paths
        MATRIX4(T);
        MATRIX4(Tinv);
        for (int i = 0; i < 6; i++) {
            struct leg_state *leg = &ctx->robot.leg_state[i];

            arm_mat_mult_f32(&Thexapod_body, &leg->coxa_mat, &T);
            matrix_3d_invert(&T, &Tinv);

            float32_t movement_velocity = arm_vec_magnitude_f32(movement_vector, 3);
            float32_t rotational_velocity = fabsf(rotation_step) * max_r;

            float32_t effective_velocity =
                    movement_velocity + rotational_velocity;

            effective_velocity = fmaxf(effective_velocity, 1e-3f);

            //if (longest_path < CTRL_CLOSE_THRESH) {
            //    continue;
            //}

            float32_t substeps = longest_path / effective_velocity;
            substeps = fmaxf(substeps, 1.0f);

            float32_t path_length = calculate_path_length(paths[i]);
            float32_t step_length = path_length / substeps;

            float32_t delta[3] = {0.0f, 0.0f, 0.0f};
            interpolate(paths[i], step_length, delta);

            float32_t p_next_body[3];
            arm_vec_copy_f32(delta, p_next_body, 3);

            float32_t p_next_world[3];
            matrix_3d_vec_transform(&Thexapod_body, p_next_body, p_next_world);

            if (leg->grounded) {
                // Use the existing coordinates for the world frame
                arm_vec_copy_f32(leg->tip_world_coordinates, p_next_world, 3);
            } else {
                remaining_path_length = fmaxf(
                    remaining_path_length,
                    arm_euclidean_distance_f32(
                        p_next_body,
                        paths[i][3],
                        3
                    )
                );
                if (i==1) {
                    LOG_DEBUG("Path length: %5.2f", remaining_path_length);
                    LOG_DEBUG("P_NEXT_BODY: %5.2f, %5.2f, %5.2f", p_next_body[0], p_next_body[1], p_next_body[2]);
                    LOG_DEBUG("P_TARGET: %5.2f, %5.2f, %5.2f", paths[i][3][0], paths[i][3][1], paths[i][3][2]);
                }
                arm_vec_copy_f32(p_next_world, leg->tip_world_coordinates, 3);
            }

            float32_t p_next_coxa[3];
            matrix_3d_vec_transform(&Tinv, p_next_world, p_next_coxa);

            float32_t origin3[3] = {0, 0, 0};
            inverse_kinematics(origin3, p_next_coxa, leg->next_joint_angles);
        }

        LOG_DEBUG("Remaining: %5.2f", remaining_path_length);
        if (remaining_path_length < CTRL_CLOSE_THRESH) {
            swap_legs(ctx);
        }
    } else if (ctx->state == CTRL_ROTATING) {
        // Ensure tripod consistency (only re-init if all legs grounded)
        tripod_init_gait(ctx, &Thexapod_body);

        // 1) Compute desired rotation for this timestep
        float32_t desired_omega = ctx->yaw_error / dt_s;
        desired_omega = clampf(desired_omega,
                                -CTRL_MAX_YAW_RATE,
                                +CTRL_MAX_YAW_RATE);

        float32_t rotation_step = desired_omega * dt_s;

        ctx->robot.hexapod.rotation[2] += rotation_step;
        pose_get_transformation(&ctx->robot.hexapod, &Thexapod);
        pose_get_transformation(&ctx->robot.body, &Tbody);
        arm_mat_mult_f32(&Thexapod, &Tbody, &Thexapod_body);

        // 5) Generate paths for each leg
        float32_t longest_path = 0.0f;
        float32_t longest_lifted_path = 0.0f;
        float32_t remaining_path_length = 0.0f;
        float32_t paths[6][4][3];
        float32_t body_z = ctx->robot.body.translation[2];
        for (int i = 0; i < 6; i++) {
            struct leg_state *leg = &ctx->robot.leg_state[i];

            float32_t origin[2] = {
                leg->tip_home[0],
                leg->tip_home[1]
            };

            float32_t p_current[3];
            leg_current_position_body(leg, p_current);

            // r = foot position in body frame
            float32_t r[2] = {
                p_current[0],
                p_current[1]
            };

            // ẑ × r
            float32_t r_perp[2] = {
                -r[1],
                r[0]
            };

            // Rotation displacement only
            float32_t rot_disp[2] = {
                r_perp[0] * rotation_step,
                r_perp[1] * rotation_step
            };

            // Clamp per-foot motion
            float32_t mag = hypotf(rot_disp[0], rot_disp[1]);
            if (mag > CTRL_MAX_ROT_FOOT_MM) {
                rot_disp[0] *= CTRL_MAX_ROT_FOOT_MM / mag;
                rot_disp[1] *= CTRL_MAX_ROT_FOOT_MM / mag;
            }

            if (leg->grounded) {
                // Grounded legs move opposite of body rotation
                float32_t arc_disp[2] = {
                    -rot_disp[0],
                    -rot_disp[1]
                };

                float32_t path_len = leg_generate_path(
                    body_z,
                    p_current,
                    CTRL_LEG_RADIUS_MM,
                    origin,
                    arc_disp,
                    false,
                    paths[i]
                );

                longest_path = fmaxf(longest_path, path_len);
            } else {
                // Swing legs move with lift
                float32_t arc_disp[2] = {
                    rot_disp[0],
                    rot_disp[1]
                };

                float32_t path_len = leg_generate_path(
                    body_z,
                    p_current,
                    CTRL_LEG_RADIUS_MM,
                    origin,
                    arc_disp,
                    true,
                    paths[i]
                );

                longest_lifted_path = fmaxf(longest_lifted_path, path_len);
            }
        }

        longest_path = fminf(longest_path, longest_lifted_path);

        // 6) Execute paths
        MATRIX4(T);
        MATRIX4(Tinv);
        for (int i = 0; i < 6; i++) {
            struct leg_state *leg = &ctx->robot.leg_state[i];

            arm_mat_mult_f32(&Thexapod_body, &leg->coxa_mat, &T);
            matrix_3d_invert(&T, &Tinv);

            float32_t distance_per_tick = CTRL_DEFAULT_VEL * dt_s;
            distance_per_tick = fmaxf(distance_per_tick, 1e-6f);

            float32_t substeps = fmaxf(longest_path / distance_per_tick, 1.0f);
            float32_t step_len = calculate_path_length(paths[i]) / substeps;

            float32_t delta[3] = {0};
            interpolate(paths[i], step_len, delta);

            float32_t p_next_body[3];
            arm_vec_copy_f32(delta, p_next_body, 3);

            float32_t p_next_world[3];
            matrix_3d_vec_transform(&Thexapod_body, p_next_body, p_next_world);

            if (leg->grounded) {
                // Use the existing coordinates for the world frame
                arm_vec_copy_f32(leg->tip_world_coordinates, p_next_world, 3);
            } else {
                remaining_path_length = fmaxf(
                    remaining_path_length,
                    arm_euclidean_distance_f32(p_next_body, paths[i][3], 3)
                );
                arm_vec_copy_f32(p_next_world, leg->tip_world_coordinates, 3);
            }

            float32_t p_next_coxa[3];
            matrix_3d_vec_transform(&Tinv, p_next_world, p_next_coxa);

            float32_t origin3[3] = {0, 0, 0};
            inverse_kinematics(origin3, p_next_coxa, leg->next_joint_angles);
        }

        if (remaining_path_length < CTRL_CLOSE_THRESH) {
            swap_legs(ctx);
        }
    }
}

const char *controller_state_to_string(controller_state_t state) {
    switch (state) {
        case CTRL_BOOT: return "BOOT";
        case CTRL_SYNCING: return "SYNCING";
        case CTRL_STANDUP: return "STANDUP";
        case CTRL_STANDING: return "STANDING";
        case CTRL_WALKING: return "WALKING";
        case CTRL_ROTATING: return "ROTATING";
        case CTRL_POWERDOWN: return "POWERDOWN";
        default: return "UNKNOWN";
    }
}
