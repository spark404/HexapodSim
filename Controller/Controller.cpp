//
// Created by Hugo Trippaers on 09/07/2024.
//

#include "Controller.h"

#include "inverse_kinematics.h"
#include "forward_kinematics.h"
#include "matrix_3d.h"

#include <utility>
#include <conversion_2d.h>

#include "Gait.h"
#include "Standup.h"

int getStringCode(const std::string &input);

Controller::Controller(Robot robot) {
    _robot = std::move(robot);
    _time_us = 0;
}

Controller::~Controller() = default;

int Controller::init() {
    pose_set(&_hexapod, 0, 0, 150, 0, 0, 0);
    pose_set(&_body, 0, 0, 0, 0, 0, 0);

    MATRIX4(Thexapod);
    MATRIX4(Tbody);

    pose_get_transformation(&_hexapod, &Thexapod);
    pose_get_transformation(&_body, &Tbody);

    MATRIX4(T1);
    arm_mat_mult_f32(&Thexapod, &Tbody, &T1);

    // Prepare leg configurations
    int i = 0;
    for (Leg &leg: _robot.leg) {
        std::cout << "Preparing leg " << leg.name << std::endl;
        float32_t leg_coordinates[2];
        convert_2d_polar_to_cartesian(leg.polar_joint_offset, leg_coordinates);

        // Configure the coxa pose in the body frame
        pose_set(&_state[i].coxa_joint_in_body_frame, leg_coordinates[0], leg_coordinates[1], 0, 0, 0, leg.polar_joint_offset[1]);
        std::cout << "\t" << "Coxa (body) at (" << leg_coordinates[0] << "," << leg_coordinates[1] << ")"
                  << std::endl;

        MATRIX4(Tcoxa);
        pose_get_transformation(&_state[i].coxa_joint_in_body_frame, &Tcoxa);

        MATRIX4(T);
        arm_mat_mult_f32(&T1, &Tcoxa, &T);

        float32_t pos[3];
        forward_kinematics((float32_t *) leg.tip_starting_angles, pos);
        matrix_3d_vec_transform(&T, pos, leg.tip_starting_position);

        // Tip position in world frame
        pose_set(&_state[i].tip_position_in_world_frame, leg.tip_starting_position[0], leg.tip_starting_position[1],
                 leg.tip_starting_position[2], 0, 0, 0);

        for (int j = 0; j < 3; j++) {
            _state[i].joint_angles[j] = leg.tip_starting_angles[j];
        }

        MATRIX4(Ti);
        matrix_3d_invert(&Thexapod, &Ti);
        matrix_3d_vec_transform(&Ti,
                                _state[i].tip_position_in_world_frame.translation,
                                _state[i].tip_position_in_body_frame.translation);

        leg.tip_starting_position[0] = _state[i].tip_position_in_body_frame.translation[0];
        leg.tip_starting_position[1] = _state[i].tip_position_in_body_frame.translation[1];
        leg.tip_starting_position[2] = _state[i].tip_position_in_body_frame.translation[2];

        std::cout << "\t" << "Tip (world) at ("
                << _state[i].tip_position_in_world_frame.translation[0] << ","
                << _state[i].tip_position_in_world_frame.translation[1] << ","
                << _state[i].tip_position_in_world_frame.translation[2] << ")"
                << std::endl;
        std::cout << "\t" << "Tip (body) at ("
                << _state[i].tip_position_in_body_frame.translation[0] << ","
                << _state[i].tip_position_in_body_frame.translation[1] << ","
                << _state[i].tip_position_in_body_frame.translation[2] << ")"
                << std::endl;

        // Setup the topics
        char buf[100];
        for (int j = 0; j < 3; j++) {
            int servo = j + 1;
            snprintf(buf, 99, "/model/hexspider/joint/leg_%s_servo_%d/0/cmd_pos", leg.name.c_str(), servo);
            std::string topic = buf;
            auto p = _node.Advertise<gz::msgs::Double>(topic);
            if (!p) {
                std::cerr << "Error advertising topic [" << topic << "]" << std::endl;
                return -1;
            }
            _state[i].servo_publishers[j] = p;
            std::cout << "\t" << "Publisher for topic " << buf << std::endl;
        }

        i++;
    }

    _time_us = 0;

    return 1;
}

int Controller::run() {
    using namespace std::chrono_literals;

    std::unique_lock<std::mutex> lk(_tick_mutex);
    uint64_t last_time_us = 0;

    std::cout << "Starting listeners for joint state" << std::endl;
    std::string joint_state_topic = "/world/hexspider_world/model/hexspider/joint_state";
    if (!(_node.Subscribe(joint_state_topic, &Controller::jointStateCallback, this))) {
        std::cerr << "Failed to subscribe to joint_state topic" << std::endl;
        return 0;
    }

    std::cout << "Starting main loop using world clock ticks" << std::endl;
    std::string clock_topic = "/world/hexspider_world/clock";
    if (!(_node.Subscribe(clock_topic, &Controller::clockCallback, this))) {
        std::cerr << "Failed to subscribe to clock topic" << std::endl;
        return 0;
    }

    pose_set(&_hexapod, 0, 0, 150, 0, 0, 0);
    pose_set(&_body, 0, 0, -50, 0, 0, 0);

    float32_t velocity = 0; // mm/s
    float32_t heading = 0;

    Gait gait_controller{};
    gait_controller.init();

    Standup standup_controller{};
    standup_controller.init();

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

        uint64_t delta_t = _time_us - last_time_us;
        last_time_us = _time_us;

        if (delta_t > 10000) {
            continue;
        }

        if (_motion_state == INITIALIZING) {
            if (_state[0].actual_joint_angles[0] == 0.0f &&
            _state[0].actual_joint_angles[1] == 0.0f &&
            _state[0].actual_joint_angles[2] == 0.0f) {
                // no joint state update yet
                continue;
            }
            for (int i=0; i<6; i++) {
                arm_vec_copy_f32(_state[i].actual_joint_angles, _state[i].prev_joint_angles, 3);
            }
            _motion_state = STANDING;
            _base_motion = &standup_controller;
        }

        float32_t stepsize = velocity * (float32_t)delta_t / 1000000;
        _hexapod.translation[0] += stepsize * cos(heading);
        _hexapod.translation[1] += stepsize * sin(heading);

        float32_t motion_vector[3] = { velocity * cos(heading), velocity * sin(heading), 0};
        float32_t movement_vector[3];
        arm_vec_mult_scalar_f32(motion_vector, (float32_t)delta_t / 1000000, movement_vector, 3);

        MATRIX4(Thexapod);
        MATRIX4(Tbody);

        pose_get_transformation(&_hexapod, &Thexapod);
        pose_get_transformation(&_body, &Tbody);

        MATRIX4(T1);
        arm_mat_mult_f32(&Thexapod, &Tbody, &T1);

        // Calculate the targets for the current cycle
        _base_motion->calculate(_robot, _state, movement_vector);

        if (_motion_state == WALKING) {
            // Calculate the servo angles and publish
            for (int i = 0; i < 6; i++) {
                MATRIX4(Tcoxa);
                pose_get_transformation(&_state[i].coxa_joint_in_body_frame, &Tcoxa);

                MATRIX4(T);
                arm_mat_mult_f32(&T1, &Tcoxa, &T);

                MATRIX4(Ti);
                matrix_3d_invert(&T, &Ti);

                MATRIX4(Thexapod_to_body);
                matrix_3d_invert(&Thexapod, &Thexapod_to_body);

                float32_t *tip_in_world = _state[i].tip_position_in_world_frame.translation;
                float32_t *tip_in_body = _state[i].tip_position_in_body_frame.translation;

                if (!_state[i].grounded) {
                    // We have a new position in the body frame, calculate the posistion in the
                    // updated world frame
                    matrix_3d_vec_transform(&Thexapod, _state[i].tip_interpolated_target, tip_in_world);
                    arm_vec_copy_f32(_state[i].tip_interpolated_target,
                                     _state[i].tip_position_in_body_frame.translation, 3);
                } else {
                    // Recalculate the tip position in body frame
                    matrix_3d_vec_transform(&Thexapod_to_body, tip_in_world, tip_in_body);
                }

                float32_t tip_in_coxa[3];
                matrix_3d_vec_transform(&Ti, tip_in_world, tip_in_coxa);

                float32_t origin[3] = {0, 0, 0};
                float32_t angles[3];
                inverse_kinematics(origin, tip_in_coxa, angles);

                _state[i].joint_angles[0] = angles[0];
                _state[i].joint_angles[1] = angles[1];
                _state[i].joint_angles[2] = angles[2];
            }
        }

        // Update the legs
        if (_motion_state != INITIALIZING) {
            for (auto &s: _state) {
                for (int j = 0; j < 3; j++) {
                    double angle = s.joint_angles[j];
                    if (j == 2) {
                        angle -= D2R(25);
                    }
                    gz::msgs::Double msg;
                    msg.set_data(angle);

                    s.servo_publishers[j].Publish(msg);
                }
            }
        }

        // Process any updates needed for the next cycle
        _base_motion->update(_state);
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

    return 1;
}

void Controller::shutdown() {
    terminate = true;
    _tick.notify_one();
}

void Controller::clockCallback(const gz::msgs::Clock &clock) {
    const uint64_t time_us = (clock.sim().sec() * 1000000) + (clock.sim().nsec() / 1000);
    if (time_us <= _time_us) {
        return;
    }
    _time_us = time_us;
    _tick.notify_one();
    // std::cout << "Tick" << std::endl;
}

void Controller::jointStateCallback(const gz::msgs::Model &model) {
    for (int i = 0; i < model.joint_size(); i++) {
        const auto& joint = model.joint(i);
        const auto& str = joint.name();

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
            if (joint_id == 2) {
                v += D2R(25);
            }

            this->_state[leg_index].actual_joint_angles[joint_id] = v;
        }
    }
}




int getStringCode(const std::string &input) {
    // Create a mapping of strings to numbers
    static const std::unordered_map<std::string, int> stringToCode = {{"fr", 0},
                                                                      {"cr", 1},
                                                                      {"br", 2},
                                                                      {"fl", 3},
                                                                      {"cl", 4},
                                                                      {"bl", 5}};

    // Find the input string in the map
    auto it = stringToCode.find(input);
    if (it != stringToCode.end()) {
        return it->second; // Return the corresponding number
    } else {
        return -1; // Return -1 if the input string is not valid
    }
}