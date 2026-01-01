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

void controller_update(controller_ctx_t *ctx, const controller_command_t *cmd, float32_t dt_s) {
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

    float32_t yaw = ctx->robot.hexapod.rotation[2];

    ctx->yaw_error = wrap_to_pi(heading - yaw);

    if (ctx->state == CTRL_WALKING || ctx->state == CTRL_STANDING) {
        if (height != ctx->robot.body.translation[2]) {
            ctx->robot.body.translation[2] = height; // Mirrors the height set in the target
        }
    }

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

    if (ctx->state == CTRL_STANDING && (wants_translation || wants_rotation)) {
        ctx->next_state = CTRL_WALKING;
    }

    if (ctx->state == CTRL_WALKING && !wants_translation && !wants_rotation) {
        ctx->next_state = CTRL_STANDING;
    }

    // State machine
    if (ctx->state != ctx->next_state) {
        LOG_INFO("Transitioning to motion state %s", controller_state_to_string(ctx->next_state));
        controller_state_t from = ctx->state;
        controller_state_t to = ctx->next_state;

        ctx->state = to;

        if (ctx->on_state_change) {
            ctx->on_state_change(ctx, from, to, ctx->cb_user_data);
        }
    }

    if (ctx->state == CTRL_SYNCING) {
        // Make sure actual and next angles are set to the same value
        for (int i = 0; i < 6; i++) {
            arm_vec_copy_f32(ctx->robot.leg_state[i].actual_joint_angles,
                             ctx->robot.leg_state[i].next_joint_angles, 3);
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

            float32_t p_current_in_coxa_frame[3];
            float32_t p_current_in_body_frame[3];
            forward_kinematics(current_leg_state->actual_joint_angles, p_current_in_coxa_frame);
            matrix_3d_vec_transform(&current_leg_state->coxa_mat, p_current_in_coxa_frame, p_current_in_body_frame);

            float32_t p_next_in_body_frame[3];
            float32_t p_next_in_coxa_frame[3];
            float32_t distance_remaining;
            calculate_motion_step(&motion_param, p_current_in_body_frame, p_target_in_body_frame,
                                  dt_s, p_next_in_body_frame, &distance_remaining);

            float32_t origin[3] = {0.0f, 0.0f, 0.0f};
            matrix_3d_vec_transform(&current_leg_state->coxa_mat_inv, p_next_in_body_frame, p_next_in_coxa_frame);
            inverse_kinematics(origin, p_next_in_coxa_frame, current_leg_state->next_joint_angles);

            if (distance_remaining > CTRL_CLOSE_THRESH) {
                ready = 0;
            };
        }

        if (ready) {
            ctx->robot.body.translation[2] = CTRL_BODY_Z; // Mirrors the height set in the target
            pose_get_transformation(&ctx->robot.hexapod, &Thexapod);
            pose_get_transformation(&ctx->robot.body, &Tbody);
            arm_mat_mult_f32(&Thexapod, &Tbody, &Thexapod_body);

            ctx->next_state = CTRL_STANDING;
        }
    } else if (ctx->state == CTRL_WALKING) {
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
                arm_mat_mult_f32(&Thexapod_body, &current_leg_state->coxa_mat, &T);

                float32_t tip_in_coxa[3];
                forward_kinematics(current_leg_state->actual_joint_angles, tip_in_coxa);
                matrix_3d_vec_transform(&T, tip_in_coxa, current_leg_state->tip_world_coordinates);
            }
        }

        bool rotate_only = (fabsf(velocity) < CTRL_EPS_VEL) &&
                   (fabsf(ctx->yaw_error) > CTRL_EPS_ANG);

        // Determine the movement
        float32_t desired_omega = ctx->yaw_error / dt_s;
        desired_omega = clampf(desired_omega, -CTRL_MAX_YAW_RATE, +CTRL_MAX_YAW_RATE);
        float32_t rotation_step = desired_omega * dt_s;

        // 1) World-frame desired motion (commanded)
        float32_t v_world[2] = {
            velocity * arm_cos_f32(yaw),
            velocity * arm_sin_f32(yaw)
        };

        // 2) Convert world-frame velocity to body frame
        float32_t v_body[2] = {
            v_world[0] * arm_cos_f32(-yaw) - v_world[1] * arm_sin_f32(-yaw),
            v_world[0] * arm_sin_f32(-yaw) + v_world[1] * arm_cos_f32(-yaw)
        };

        float32_t movement_vector[3] = {
            v_body[0] * dt_s,
            v_body[1] * dt_s,
            0.0f
        };

        // 3) Integrate rotation AFTER
        if (!rotate_only) {
            ctx->robot.hexapod.rotation[2] += rotation_step;
        } else {
            ctx->pending_yaw += rotation_step;
        }

        ctx->pending_yaw = clampf(
            ctx->pending_yaw,
            -CTRL_ROT_STEP_RAD,
            +CTRL_ROT_STEP_RAD
        );

        // movement_vector is BODY frame
        float32_t yaw_mid = yaw;
        if (!rotate_only) {
            yaw_mid += 0.5f * rotation_step;
        }

        float32_t dx_world =
                movement_vector[0] * arm_cos_f32(yaw_mid) -
                movement_vector[1] * arm_sin_f32(yaw_mid);

        float32_t dy_world =
                movement_vector[0] * arm_sin_f32(yaw_mid) +
                movement_vector[1] * arm_cos_f32(yaw_mid);

        ctx->robot.hexapod.translation[0] += dx_world;
        ctx->robot.hexapod.translation[1] += dy_world;

        // Update the translations so they are performed with respect to the new location
        pose_get_transformation(&ctx->robot.hexapod, &Thexapod);
        pose_get_transformation(&ctx->robot.body, &Tbody);

        arm_mat_mult_f32(&Thexapod, &Tbody, &Thexapod_body);

        float32_t longest_path = 0.f;
        float32_t longest_lifted_path = 0.f;
        float32_t remaining_path_length = 0.f;
        float32_t paths[6][4][3];
        for (int i = 0; i < 6; i++) {
            struct leg_state *current_leg_state = &ctx->robot.leg_state[i];

            float32_t step_size = CTRL_LEG_RADIUS_MM;
            float32_t origin[2] = {current_leg_state->tip_home[0], current_leg_state->tip_home[1]};

            float32_t p_current_in_body_frame[3];
            float32_t tip_in_coxa[3];
            forward_kinematics(current_leg_state->actual_joint_angles, tip_in_coxa);
            matrix_3d_vec_transform(&current_leg_state->coxa_mat, tip_in_coxa, p_current_in_body_frame);

            // Vector from body center to leg reference point (body frame)
            // float32_t r[2] = {
            //     origin[0] - _ctx->robot.body.translation[0],
            //     origin[1] - _ctx->robot.body.translation[1]
            // };
            float32_t r[2] = {
                p_current_in_body_frame[0],
                p_current_in_body_frame[1]
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

            if (current_leg_state->grounded) {
                float32_t arc_disp[2] = {
                    -movement_vector[0] + rot_disp[0],
                    -movement_vector[1] + rot_disp[1]
                };

                float32_t point[2];
                project_point_on_circle(step_size, origin, arc_disp, point);
                float32_t p_target_in_body_frame[3] = {point[0], point[1], ctx->robot.body.translation[2] * -1};

                arm_vec_copy_f32(p_current_in_body_frame, paths[i][0], 3);
                arm_vec_copy_f32(p_current_in_body_frame, paths[i][1], 3);
                arm_vec_copy_f32(p_current_in_body_frame, paths[i][2], 3);
                arm_vec_copy_f32(p_target_in_body_frame, paths[i][3], 3);

                float32_t path_length = arm_euclidean_distance_f32(p_current_in_body_frame, p_target_in_body_frame,
                                                                   3);

                longest_path = fmaxf(path_length, longest_path);
            } else {
                float32_t point[2];
                float32_t arc_disp[2] = {
                    movement_vector[0] + rot_disp[0],
                    movement_vector[1] + rot_disp[1]
                };

                project_point_on_circle(step_size, origin, arc_disp, point);

                float32_t p_target_in_body_frame[3] = {point[0], point[1], ctx->robot.body.translation[2] * -1};

                calculate_path(p_current_in_body_frame, p_target_in_body_frame, 25, 2.0f, paths[i]);
                float32_t path_length = arm_euclidean_distance_f32(p_current_in_body_frame, p_target_in_body_frame,
                                                                   3);

                longest_lifted_path = fmaxf(path_length, longest_lifted_path);
            }
        }

        longest_path = fminf(longest_path, longest_lifted_path);

        float32_t max_r = 0.0f;
        for (int i = 0; i < 6; i++) {
            float32_t r = arm_euclidean_distance_f32(
                ctx->robot.leg_state[i].tip_home,
                ctx->robot.body.translation,
                2
            );
            max_r = fmaxf(max_r, r);
        }

        for (int i = 0; i < 6; i++) {
            struct leg_state *current_leg_state = &ctx->robot.leg_state[i];

            MATRIX4(T);
            arm_mat_mult_f32(&Thexapod_body, &current_leg_state->coxa_mat, &T);
            MATRIX4(Tinv);
            matrix_3d_invert(&T, &Tinv);

            float32_t movement_velocity = arm_vec_magnitude_f32(movement_vector, 3);
            float32_t rotational_velocity = fabsf(rotation_step) * max_r;

            float32_t effective_velocity =
                    movement_velocity + rotational_velocity;

            effective_velocity = fmaxf(effective_velocity, 1e-3f);

            float32_t effective_path = longest_path;

            if (rotate_only) {
                effective_path = fmaxf(fabsf(rotation_step) * max_r,
                                        CTRL_ROT_STEP_RAD * max_r);
            }

            float32_t substeps = effective_path / effective_velocity;
            substeps = fmaxf(substeps, 1.0f);

            float32_t path_length = calculate_path_length(paths[i]);
            float32_t step_length = path_length / substeps;

            float32_t delta[3] = {0.0f, 0.0f, 0.0f};
            interpolate(paths[i], step_length, delta);

            float32_t p_next_in_body_frame[3];
            arm_vec_copy_f32(delta, p_next_in_body_frame, 3);

            float32_t p_next_in_world_frame[3];
            matrix_3d_vec_transform(&Thexapod_body, p_next_in_body_frame, p_next_in_world_frame);

            if (current_leg_state->grounded) {
                // Use the existing coordinates for the world frame
                arm_vec_copy_f32(current_leg_state->tip_world_coordinates, p_next_in_world_frame, 3);
            } else {
                if (rotate_only) {
                    remaining_path_length = fmaxf(
                        remaining_path_length,
                        fabsf(rotation_step)
                    );
                } else {
                    remaining_path_length = fmaxf(
                        remaining_path_length,
                        arm_euclidean_distance_f32(
                            p_next_in_body_frame,
                            paths[i][3],
                            3
                        )
                    );
                }
                arm_vec_copy_f32(p_next_in_world_frame, current_leg_state->tip_world_coordinates, 3);
            }

            float32_t p_next_in_coxa_frame[3];
            matrix_3d_vec_transform(&Tinv, p_next_in_world_frame, p_next_in_coxa_frame);

            float32_t origin[3] = {0, 0, 0};
            inverse_kinematics(origin, p_next_in_coxa_frame, ctx->robot.leg_state[i].next_joint_angles);
        }

        if (rotate_only) {
            if (fabsf(ctx->pending_yaw) >= CTRL_ROT_STEP_RAD) {

                // Apply rotation ONCE
                ctx->robot.hexapod.rotation[2] += ctx->pending_yaw;
                ctx->pending_yaw = 0.0f;

                swap_legs(ctx);
            }
        } else {
            if (remaining_path_length < CTRL_CLOSE_THRESH) {
                swap_legs(ctx);
            }
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
        case CTRL_POWERDOWN: return "POWERDOWN";
        default: return "UNKNOWN";
    }
}
