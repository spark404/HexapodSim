//
// Created by Hugo Trippaers on 09/07/2024.
//

#include "Controller.h"

#include "hexapodmath/inverse_kinematics.h"
#include "hexapodmath/forward_kinematics.h"
#include "hexapodmath/matrix_3d.h"

#include <hexapodmath/conversion_2d.h>

#include "log.h"
#include "calculator.h"
#include "hexapodmath/hexapod.h"

int getStringCode(const std::string &input);

static float clamp(float x, float min, float max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

static inline float wrap_to_pi(float angle)
{
    angle = fmodf(angle + M_PI, 2.0f * M_PI);
    if (angle < 0.0f) {
        angle += 2.0f * M_PI;
    }
    return angle - M_PI;
}

Controller::Controller() {
    _time_us = 0;
}

Controller::~Controller() = default;

void Controller::init() {
    typedef enum {
        BOOT,
        SYNCING,
        STANDUP,
        STANDING,
        WALKING,
        POWERDOWN,
    } state_t;

    // Do a bunch of static calculations that depend on the robot configuration in robot.h
    pose_set(&_robot_state.hexapod, 0, 0, 0, 0, 0, 0);
    pose_set(&_robot_state.body, 0, 0, 150, 0, 0, 0);

    MATRIX4(Thexapod);
    MATRIX4(Tbody);
    pose_get_transformation(&_robot_state.hexapod, &Thexapod);
    pose_get_transformation(&_robot_state.body, &Tbody);

    MATRIX4(Thexapod_body);
    arm_mat_mult_f32(&Thexapod, &Tbody, &Thexapod_body);

    for (int i = 0; i < 6; i++) {
        float32_t mount_point_xy[2];
        const struct leg *current_leg = &r.leg[i];
        struct leg_state *current_leg_state = &_robot_state.leg_state[i];

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

        // Setup the topics
        char buf[100];
        for (int j = 0; j < 3; j++) {
            const int servo = j + 1;
            snprintf(buf, 99, "/model/hexspider/joint/leg_%s_servo_%d/0/cmd_pos", leg_names[i].c_str(), servo);
            std::string topic = buf;
            auto p = _node.Advertise<gz::msgs::Double>(topic);
            if (!p) {
                std::cerr << "Error advertising topic [" << topic << "]" << std::endl;
            }
            _servo_publishers[i][j] = p;
            std::cout << "\t" << "Leg " << i << " Joint " << j << ": " << buf << std::endl;
        }
    }

    _time_us = 0;
}

void Controller::run() {
    using namespace std::chrono_literals;

    std::unique_lock<std::mutex> lk(_tick_mutex);
    uint64_t last_time_us = 0;

    std::cout << "Starting listeners for joint state" << std::endl;
    std::string joint_state_topic = "/world/hexspider_world/model/hexspider/joint_state";
    if (!(_node.Subscribe(joint_state_topic, &Controller::jointStateCallback, this))) {
        std::cerr << "Failed to subscribe to joint_state topic" << std::endl;
        return;
    }

    std::cout << "Starting listeners for velocity" << std::endl;
    std::string velocity_topic = "/world/hexspider_world/model/hexspider/velocity";
    if (!(_node.Subscribe(velocity_topic, &Controller::velocityCallback, this))) {
        std::cerr << "Failed to subscribe to velocity topic" << std::endl;
        return;
    }

    std::cout << "Starting listeners for heading" << std::endl;
    std::string heading_topic = "/world/hexspider_world/model/hexspider/heading";
    if (!(_node.Subscribe(heading_topic, &Controller::headingCallback, this))) {
        std::cerr << "Failed to subscribe to heading topic" << std::endl;
        return;
    }

    std::cout << "Starting listeners for height" << std::endl;
    std::string height_topic = "/world/hexspider_world/model/hexspider/height";
    if (!(_node.Subscribe(height_topic, &Controller::heightCallback, this))) {
        std::cerr << "Failed to subscribe to height topic" << std::endl;
        return;
    }

    std::cout << "Starting main loop using world clock ticks" << std::endl;
    std::string clock_topic = "/world/hexspider_world/clock";
    if (!(_node.Subscribe(clock_topic, &Controller::clockCallback, this))) {
        std::cerr << "Failed to subscribe to clock topic" << std::endl;
        return;
    }

    MATRIX4(Thexapod);
    MATRIX4(Tbody);
    MATRIX4(Thexapod_body);

    uint16_t powerdown_timeout = POWERDOWN_TIMEOUT;

    float32_t orientation_error = 0;
    float32_t rotation_velocity = 0.1; // radians/second, best between 0.3 and 1.0 while moving
    float32_t rotation_step = 0;

    /* Infinite loop */
    while (!terminate) {
        // Wait for a sync pulse from gazebo on the clock topic
        auto status = _tick.wait_for(lk, std::chrono::seconds(2));

        if (status == std::cv_status::timeout) {
            terminate = false;
            continue;
        }

        if (terminate) {
            break;
        }

        uint64_t delta_t_us = _time_us - last_time_us;
        if (delta_t_us < 10000) {
            continue;
        }
        last_time_us = _time_us;
        float32_t delta_t_s = (float32_t) delta_t_us / 1000000;

        // Update from the callback
        _velocity = _cmd_velocity;
        _orientation = _robot_state.hexapod.rotation[2];
        float32_t effective_heading = _robot_state.hexapod.rotation[2];

        orientation_error = wrap_to_pi(_cmd_heading - _orientation);

        if (_motion_state == WALKING || _motion_state == STANDING) {
            if (_cmd_height != _robot_state.body.translation[2]) {
                _robot_state.body.translation[2] = _cmd_height; // Mirrors the height set in the target
                pose_get_transformation(&_robot_state.hexapod, &Thexapod);
                pose_get_transformation(&_robot_state.body, &Tbody);
                arm_mat_mult_f32(&Thexapod, &Tbody, &Thexapod_body);
            }
        }

        bool wants_translation = fabsf(_velocity) > 1e-3f;
        bool wants_rotation    = fabsf(orientation_error) > 1e-3f;

        // Rules for transitions
        if (_motion_state == STANDING) {
            if (powerdown_timeout == 0) {
                _next_state = POWERDOWN;
            } else {
                powerdown_timeout--;
            }
        } else {
            powerdown_timeout = POWERDOWN_TIMEOUT;
        }
        if (_motion_state == POWERDOWN && (wants_translation || wants_rotation)) {
            _next_state = SYNCING;
        }
        // Enter walking if either translation OR rotation is requested
        if (_motion_state == STANDING && (wants_translation || wants_rotation)) {
            _next_state = WALKING;
        }

        // Leave walking only if NOTHING is requested
        if (_motion_state == WALKING && !wants_translation && !wants_rotation) {
            _next_state = STANDING;
        }

        // State machine
        if (_motion_state != _next_state) {
            LOG_INFO("Transitioning to motion state %d", _next_state);
            switch (_next_state) {
                case SYNCING:
                    for (int i = 0; i < 3 * 6; i++) {
                        LOG_INFO("Activating servo %d...", i);
                    }
                    break;
                case POWERDOWN:
                    for (int i = 0; i < 3 * 6; i++) {
                        LOG_INFO("Deactivating servo %d...", i);
                    }
                    break;
                case STANDUP:
                    break;
                case STANDING:
                    break;
                case WALKING:
                    break;
                default:
                    break;
            }
            _motion_state = _next_state;
        }

        // Determine the actual servo positions
        for (int i = 0; i < 6; i++) {
            struct leg_state *current_leg_state = &_robot_state.leg_state[i];
            const struct leg *current_leg = &r.leg[i];

            std::array<gz::transport::Node::Publisher, 3> leg_servos = {
                _servo_publishers[i][0],
                _servo_publishers[i][1],
                _servo_publishers[i][2],
            };
            float32_t measured_leg_servo_angles[3];

            if (read_actual_servo_position(i, 3, measured_leg_servo_angles) < 0) {
                // LOG_WARN("Failed to read servo position for leg %d\r\n", i);
                // Use the defined angles as a stop gap
                // FIXME, these angles are uncompensated
                arm_vec_copy_f32(_robot_state.leg_state[i].next_joint_angles,
                                 _robot_state.leg_state[i].actual_joint_angles, 3);
                continue;
            }

            // Compensate angles for geometry
            if (_motion_state == SYNCING) {
                // We exclusive use the measured position
                current_leg_state->actual_joint_angles[0] = measured_leg_servo_angles[0];
                current_leg_state->actual_joint_angles[1] = -measured_leg_servo_angles[1];
                current_leg_state->actual_joint_angles[2] = measured_leg_servo_angles[2] + D2R(25);
            } else {
                // We use a mix of the calculated angle and the measured angle to offset any measurement error
                // and compensate for a bit of deadzone at low speeds
                // Use alpha to tune the mix
                float32_t compensated_angles[3] = {
                    measured_leg_servo_angles[0],
                    -measured_leg_servo_angles[1],
                    measured_leg_servo_angles[2] + static_cast<float32_t>(D2R(25))
                };
                const float32_t alpha = 1.f;
                current_leg_state->actual_joint_angles[0] =
                        compensated_angles[0] * (1 - alpha) + alpha * current_leg_state->next_joint_angles[0];
                current_leg_state->actual_joint_angles[1] =
                        compensated_angles[1] * (1 - alpha) + alpha * current_leg_state->next_joint_angles[1];
                current_leg_state->actual_joint_angles[2] =
                        compensated_angles[2] * (1 - alpha) + alpha * current_leg_state->next_joint_angles[2];
            }
        }

        if (_motion_state == SYNCING) {
            // Make sure actual and next angles are set to the same value
            for (int i = 0; i < 6; i++) {
                arm_vec_copy_f32(_robot_state.leg_state[i].actual_joint_angles,
                                 _robot_state.leg_state[i].next_joint_angles, 3);
            }
            _next_state = STANDUP;
        } else if (_motion_state == STANDUP) {
            int ready = 1;
            motion_param_t motion_param = {50.0f, 20.0f, 50.0f};
            // Perform the standup routine, follows on SYNCING
            for (int i = 0; i < 6; i++) {
                struct leg_state *current_leg_state = &_robot_state.leg_state[i];

                // Target position of each leg after standup
                float32_t p_target_in_body_frame[3] = {
                    current_leg_state->tip_home[0],
                    current_leg_state->tip_home[1],
                    -100
                };

                float32_t p_current_in_coxa_frame[3];
                float32_t p_current_in_body_frame[3];
                forward_kinematics(current_leg_state->actual_joint_angles, p_current_in_coxa_frame);
                matrix_3d_vec_transform(&current_leg_state->coxa_mat, p_current_in_coxa_frame, p_current_in_body_frame);

                float32_t p_next_in_body_frame[3];
                float32_t p_next_in_coxa_frame[3];
                float32_t distance_remaining;
                calculate_motion_step(&motion_param, p_current_in_body_frame, p_target_in_body_frame,
                                      delta_t_s,
                                      p_next_in_body_frame, &distance_remaining);

                float32_t origin[3] = {0.0f, 0.0f, 0.0f};
                matrix_3d_vec_transform(&current_leg_state->coxa_mat_inv, p_next_in_body_frame, p_next_in_coxa_frame);
                inverse_kinematics(origin, p_next_in_coxa_frame, current_leg_state->next_joint_angles);

                if (distance_remaining > CLOSE_BY_THRESHOLD) {
                    ready = 0;
                };
            }

            if (ready) {
                _robot_state.body.translation[2] = 100; // Mirrors the height set in the target
                pose_get_transformation(&_robot_state.hexapod, &Thexapod);
                pose_get_transformation(&_robot_state.body, &Tbody);
                arm_mat_mult_f32(&Thexapod, &Tbody, &Thexapod_body);

                _next_state = STANDING;
            }
        } else if (_motion_state == WALKING) {
            // Reinitialize the gait when all legs are on the ground at the same time
            uint8_t re_init = 1;
            for (int i = 0; i < 6; i++) {
                if (!_robot_state.leg_state[i].grounded) {
                    re_init = 0;
                }
            }

            if (re_init) {
                LOG_INFO("(Re)Initializing tripod gait");
                _robot_state.leg_state[1].grounded = 0;
                _robot_state.leg_state[3].grounded = 0;
                _robot_state.leg_state[5].grounded = 0;

                // Use the actual angles to determine the current world coordinates of the tip
                for (int i = 0; i < 6; i++) {
                    struct leg_state *current_leg_state = &_robot_state.leg_state[i];

                    MATRIX4(T);
                    arm_mat_mult_f32(&Thexapod_body, &current_leg_state->coxa_mat, &T);

                    float32_t tip_in_coxa[3];
                    forward_kinematics(current_leg_state->actual_joint_angles, tip_in_coxa);
                    matrix_3d_vec_transform(&T, tip_in_coxa, current_leg_state->tip_world_coordinates);
                }
            }

            // Determine the movement
            float32_t desired_omega = orientation_error / delta_t_s;
            desired_omega = clamp(desired_omega, -rotation_velocity, +rotation_velocity);
            rotation_step = desired_omega * delta_t_s;

            // 1) World-frame desired motion (commanded)
            float32_t v_world[2] = {
                _velocity * arm_cos_f32(effective_heading),
                _velocity * arm_sin_f32(effective_heading)
            };

            // 2) Convert world-frame velocity to body frame
            float32_t yaw = _robot_state.hexapod.rotation[2];

            float32_t v_body[2] = {
                v_world[0] * arm_cos_f32(-yaw) - v_world[1] * arm_sin_f32(-yaw),
                v_world[0] * arm_sin_f32(-yaw) + v_world[1] * arm_cos_f32(-yaw)
            };

            float32_t movement_vector[3] = {
                v_body[0] * delta_t_s,
                v_body[1] * delta_t_s,
                0.0f
            };

            // 3) Integrate rotation AFTER
            _robot_state.hexapod.rotation[2] += rotation_step;

            // movement_vector is BODY frame
            float32_t yaw_mid = yaw + 0.5f * rotation_step;

            float32_t dx_world =
                movement_vector[0] * arm_cos_f32(yaw_mid) -
                movement_vector[1] * arm_sin_f32(yaw_mid);

            float32_t dy_world =
                movement_vector[0] * arm_sin_f32(yaw_mid) +
                movement_vector[1] * arm_cos_f32(yaw_mid);

            _robot_state.hexapod.translation[0] += dx_world;
            _robot_state.hexapod.translation[1] += dy_world;

            // Update the translations so they are performed with respect to the new location
            pose_get_transformation(&_robot_state.hexapod, &Thexapod);
            pose_get_transformation(&_robot_state.body, &Tbody);

            arm_mat_mult_f32(&Thexapod, &Tbody, &Thexapod_body);

            float32_t longest_path = 0.f;
            float32_t longest_lifted_path = 0.f;
            float32_t remaining_path_length = 0.f;
            float32_t paths[6][4][3];
            for (int i = 0; i < 6; i++) {
                struct leg_state *current_leg_state = &_robot_state.leg_state[i];

                float32_t step_size = 40;
                float32_t origin[2] = {current_leg_state->tip_home[0], current_leg_state->tip_home[1]};

                float32_t p_current_in_body_frame[3];
                float32_t tip_in_coxa[3];
                forward_kinematics(current_leg_state->actual_joint_angles, tip_in_coxa);
                matrix_3d_vec_transform(&current_leg_state->coxa_mat, tip_in_coxa, p_current_in_body_frame);

                // Vector from body center to leg reference point (body frame)
                // float32_t r[2] = {
                //     origin[0] - _robot_state.body.translation[0],
                //     origin[1] - _robot_state.body.translation[1]
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

                if (current_leg_state->grounded) {
                    float32_t arc_disp[2] = {
                        -movement_vector[0] + rot_disp[0],
                        -movement_vector[1] + rot_disp[1]
                    };

                    float32_t point[2];
                    project_point_on_circle(step_size, origin, arc_disp, point);
                    float32_t p_target_in_body_frame[3] = {point[0], point[1], _robot_state.body.translation[2] * -1};

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

                    float32_t p_target_in_body_frame[3] = {point[0], point[1], _robot_state.body.translation[2] * -1};

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
                    _robot_state.leg_state[i].tip_home,
                    _robot_state.body.translation,
                    2
                );
                max_r = fmaxf(max_r, r);
            }

            for (int i = 0; i < 6; i++) {
                struct leg_state *current_leg_state = &_robot_state.leg_state[i];

                MATRIX4(T);
                arm_mat_mult_f32(&Thexapod_body, &current_leg_state->coxa_mat, &T);
                MATRIX4(Tinv);
                matrix_3d_invert(&T, &Tinv);

                float32_t movement_velocity = arm_vec_magnitude_f32(movement_vector, 3);
                float32_t rotational_velocity = fabsf(rotation_step) * max_r;

                float32_t effective_velocity =
                        movement_velocity + rotational_velocity;

                effective_velocity = fmaxf(effective_velocity, 1e-3f);

                float32_t substeps = longest_path / effective_velocity;

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
                    remaining_path_length = fmaxf(remaining_path_length,
                                                  arm_euclidean_distance_f32(p_next_in_body_frame, paths[i][3], 3));
                    arm_vec_copy_f32(p_next_in_world_frame, current_leg_state->tip_world_coordinates, 3);
                }

                float32_t p_next_in_coxa_frame[3];
                matrix_3d_vec_transform(&Tinv, p_next_in_world_frame, p_next_in_coxa_frame);

                float32_t origin[3] = {0, 0, 0};
                inverse_kinematics(origin, p_next_in_coxa_frame, _robot_state.leg_state[i].next_joint_angles);
            }

            // printf("Remaining: %5.2f, Longest: %5.2f\n", remaining_path_length, longest_path);
            if (remaining_path_length < CLOSE_BY_THRESHOLD) {
                // LOG_DEBUG("Swap");
                for (int i = 0; i < 6; i++) {
                    struct leg_state *current_leg_state = &_robot_state.leg_state[i];
                    current_leg_state->grounded = !current_leg_state->grounded;
                }
            }
        }

        // Write next values to the servos
        for (int i = 0; i < 6; i++) {
            struct leg_state *current_leg_state = &_robot_state.leg_state[i];
            const struct leg *current_leg = &r.leg[i];

            std::array<gz::transport::Node::Publisher, 3> leg_servos = {
                _servo_publishers[i][0],
                _servo_publishers[i][1],
                _servo_publishers[i][2],
            };
            float32_t leg_servo_angles[3];

            // Compensate angles for geometry
            leg_servo_angles[0] = current_leg_state->next_joint_angles[0];
            leg_servo_angles[1] = -current_leg_state->next_joint_angles[1];
            leg_servo_angles[2] = current_leg_state->next_joint_angles[2] - D2R(25);

            uint8_t limit_alert = 0;
            for (int axis = 0; axis < 3; axis++) {
                if (leg_servo_angles[axis] < current_leg->limits[axis][0] || leg_servo_angles[axis] > current_leg->
                    limits[axis][1]) {
                    LOG_ERROR("Limit alert triggered, leg %d, axis %d", i, axis);
                    LOG_ERROR("Calculated value %5.2f, limits %5.2f, %5.2f", leg_servo_angles[axis],
                              current_leg->limits[axis][0], current_leg->limits[axis][1]);
                    limit_alert = 1;
                }
            }

            if (limit_alert && _motion_state == WALKING) {
                _cmd_velocity = 0.0f;
                _cmd_heading = 0.0f;
                _next_state = POWERDOWN;
                continue;
            }

            this->write_next_servo_position(leg_servos, 3, leg_servo_angles);
        }
    }

    std::cout << "Terminating the Controller" << std::endl;
    for (const auto &topic: _node.SubscribedTopics()) {
        std::cout << "Unsubscribing " << topic << std::endl;
        _node.Unsubscribe(topic);
    }

    for (const auto &topic: _node.AdvertisedTopics()) {
        std::cout << "Stop publishing " << topic << std::endl;
        _node.UnadvertiseSrv(topic);
    }

    std::this_thread::sleep_for(2000ms);
}

void Controller::shutdown() {
    terminate = true;
    _tick.notify_one();
}

int ticker = 0;

void Controller::clockCallback(const gz::msgs::Clock &clock) {
    const uint64_t time_us = (clock.sim().sec() * 1000000) + (clock.sim().nsec() / 1000);
    if (time_us <= _time_us) {
        return;
    }
    _time_us = time_us;

    ticker++;
    if (ticker == 2) {
        // Tick the main loop of the controller
        ticker = 0;
        _tick.notify_one();
    }
}

void Controller::jointStateCallback(const gz::msgs::Model &model) {
    for (int i = 0; i < model.joint_size(); i++) {
        const auto &joint = model.joint(i);
        const auto &str = joint.name();

        if (str.find("servo") != std::string::npos) {
            std::string leg = str.substr(4, 2);
            int leg_index = getStringCode(leg);

            int joint_index = 0;
            std::size_t pos = str.find_last_of('_'); // Find the last underscore
            if (pos != std::string::npos && pos + 1 < str.size()) {
                std::string number_str = str.substr(pos + 1); // Extract everything after the last underscore
                bool is_number = true;
                for (char c: number_str) {
                    if (!std::isdigit(c)) {
                        is_number = false;
                        break;
                    }
                }

                if (is_number) {
                    joint_index = std::stoi(number_str); // Convert to integer
                }
            }

            auto v = (float32_t) joint.axis1().position();
            auto joint_id = joint_index - 1;

            this->_measured_servo_angles[i][joint_id] = v;
        }
    }
}

void Controller::velocityCallback(const gz::msgs::Double &velocity) {
    printf("Setting velocity to %5.2f\n", velocity.data());
    _cmd_velocity = velocity.data();
}

void Controller::headingCallback(const gz::msgs::Double &heading) {
    printf("Setting heading to %5.2f\n", heading.data());
    _cmd_heading = heading.data();
}

void Controller::heightCallback(const gz::msgs::Double &height) {
    printf("Setting height to %5.2f\n", height.data());
    _cmd_height = height.data();
}

int Controller::read_actual_servo_position(const int leg_id, uint8_t servo_count, float32_t *actual_servo_angles) {
    for (int i = 0; i < servo_count; i++) {
        float32_t angle = _measured_servo_angles[leg_id][i];
        if (i == 1) {
            angle = -angle;
        }
        actual_servo_angles[i] = angle;
    }
    return 0;
}


int Controller::write_next_servo_position(const std::array<gz::transport::Node::Publisher, 3> &servos,
                                          uint8_t servo_count, float32_t *actual_servo_angles) {
    for (int i = 0; i < servo_count; i++) {
        float32_t angle = actual_servo_angles[i];
        if (i == 1) {
            angle = -angle;
        }
        gz::msgs::Double msg;
        msg.set_data(angle);
        auto p = servos[i];
        p.Publish(msg);
    }
    return 0;
}


int getStringCode(const std::string &input) {
    // Create a mapping of strings to numbers
    static const std::unordered_map<std::string, int> stringToCode = {
        {"fr", 0},
        {"cr", 1},
        {"br", 2},
        {"fl", 3},
        {"cl", 4},
        {"bl", 5}
    };

    // Find the input string in the map
    auto it = stringToCode.find(input);
    if (it != stringToCode.end()) {
        return it->second; // Return the corresponding number
    } else {
        return -1; // Return -1 if the input string is not valid
    }
}
