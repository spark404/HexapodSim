//
// Created by Hugo Trippaers on 09/07/2024.
//

#include "HexapodController.h"

#include "hexapodmath/inverse_kinematics.h"
#include "hexapodmath/forward_kinematics.h"
#include "hexapodmath/matrix_3d.h"

#include <hexapodmath/conversion_2d.h>

#include "log.h"
#include "calculator.h"
#include "hexapodmath/additional_functions.h"
#include "hexapodmath/hexapod.h"

int getStringCode(const std::string &input);

HexapodController::HexapodController() {
    _time_us = 0;
}

HexapodController::~HexapodController() = default;

void HexapodController::init() {
    controller_init(&_ctx);

    for (int i = 0; i < 6; i++) {
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

void HexapodController::run() {
    using namespace std::chrono_literals;

    std::unique_lock<std::mutex> lk(_tick_mutex);
    uint64_t last_time_us = 0;

    std::cout << "Starting listeners for joint state" << std::endl;
    std::string joint_state_topic = "/world/hexspider_world/model/hexspider/joint_state";
    if (!(_node.Subscribe(joint_state_topic, &HexapodController::jointStateCallback, this))) {
        std::cerr << "Failed to subscribe to joint_state topic" << std::endl;
        return;
    }

    std::cout << "Starting listeners for velocity" << std::endl;
    std::string velocity_topic = "/world/hexspider_world/model/hexspider/velocity";
    if (!(_node.Subscribe(velocity_topic, &HexapodController::velocityCallback, this))) {
        std::cerr << "Failed to subscribe to velocity topic" << std::endl;
        return;
    }

    std::cout << "Starting listeners for heading" << std::endl;
    std::string heading_topic = "/world/hexspider_world/model/hexspider/heading";
    if (!(_node.Subscribe(heading_topic, &HexapodController::headingCallback, this))) {
        std::cerr << "Failed to subscribe to heading topic" << std::endl;
        return;
    }

    std::cout << "Starting listeners for height" << std::endl;
    std::string height_topic = "/world/hexspider_world/model/hexspider/height";
    if (!(_node.Subscribe(height_topic, &HexapodController::heightCallback, this))) {
        std::cerr << "Failed to subscribe to height topic" << std::endl;
        return;
    }

    std::cout << "Starting main loop using world clock ticks" << std::endl;
    std::string clock_topic = "/world/hexspider_world/clock";
    if (!(_node.Subscribe(clock_topic, &HexapodController::clockCallback, this))) {
        std::cerr << "Failed to subscribe to clock topic" << std::endl;
        return;
    }

    controller_command_t cmd;

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
        cmd.heading = _cmd_heading;
        cmd.velocity = _cmd_velocity;
        cmd.height = _cmd_height;

        // Determine the actual servo positions
        for (int i = 0; i < 6; i++) {
            struct leg_state *current_leg_state = &_ctx.robot.leg_state[i];
            const struct leg *current_leg = &_ctx.cfg->leg[i];

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
                arm_vec_copy_f32(_ctx.robot.leg_state[i].next_joint_angles,
                                 _ctx.robot.leg_state[i].actual_joint_angles, 3);
                continue;
            }

            // Compensate angles for geometry
            if (_ctx.state == CTRL_SYNCING) {
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

        controller_update(&_ctx, &cmd, delta_t_s);

        // Write next values to the servos
        for (int i = 0; i < 6; i++) {
            struct leg_state *current_leg_state = &_ctx.robot.leg_state[i];
            const struct leg *current_leg = &_ctx.cfg->leg[i];

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

            if (limit_alert && _ctx.state == CTRL_WALKING) {
                cmd.velocity = 0.0f;
                cmd.heading = 0.0f;
                _ctx.next_state = CTRL_POWERDOWN;
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

void HexapodController::shutdown() {
    terminate = true;
    _tick.notify_one();
}

int ticker = 0;

void HexapodController::clockCallback(const gz::msgs::Clock &clock) {
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

void HexapodController::jointStateCallback(const gz::msgs::Model &model) {
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

void HexapodController::velocityCallback(const gz::msgs::Double &velocity) {
    printf("Setting velocity to %5.2f\n", velocity.data());
    _cmd_velocity = velocity.data();
}

void HexapodController::headingCallback(const gz::msgs::Double &heading) {
    printf("Setting heading to %5.2f\n", heading.data());
    _cmd_heading = heading.data();
}

void HexapodController::heightCallback(const gz::msgs::Double &height) {
    printf("Setting height to %5.2f\n", height.data());
    _cmd_height = height.data();
}

int HexapodController::read_actual_servo_position(const int leg_id, uint8_t servo_count, float32_t *actual_servo_angles) {
    for (int i = 0; i < servo_count; i++) {
        float32_t angle = _measured_servo_angles[leg_id][i];
        if (i == 1) {
            angle = -angle;
        }
        actual_servo_angles[i] = angle;
    }
    return 0;
}


int HexapodController::write_next_servo_position(const std::array<gz::transport::Node::Publisher, 3> &servos,
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
