//
// Created by Hugo Trippaers on 09/07/2024.
//

#include "Controller.h"

#include "inverse_kinematics.h"
#include "forward_kinematics.h"

#include <utility>
#include <conversion_2d.h>

#include "Gait.h"

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

        // Configure te coxa pose in the body frame
        pose_set(&_state[i].body_position, leg_coordinates[0], leg_coordinates[1], 0, 0, 0, leg.polar_joint_offset[1]);
        std::cout << "\t" << "Coxa position at (" << leg_coordinates[0] << "," << leg_coordinates[1] << ")"
                  << std::endl;

        MATRIX4(Tcoxa);
        pose_get_transformation(&_state[i].body_position, &Tcoxa);

        MATRIX4(T);
        arm_mat_mult_f32(&T1, &Tcoxa, &T);

        float32_t pos[4];
        forward_kinematics((float32_t *) leg.tip_starting_angles, pos);
        pos[3] = 1;
        float32_t vec[4];
        arm_mat_vec_mult_f32(&T, pos, vec);
        leg.tip_starting_position[0] = vec[0];
        leg.tip_starting_position[1] = vec[1];
        leg.tip_starting_position[2] = vec[2];

        // Tip position in world frame
        pose_set(&_state[i].tip_position, leg.tip_starting_position[0], leg.tip_starting_position[1],
                 leg.tip_starting_position[2], 0, 0, 0);

        for (int i = 0; i < 3; i++) {
            _state[i].joint_angles[i] = leg.tip_starting_angles[i];
        }
        std::cout << "\t" << "Tip position at (" << _state[i].tip_position.translation[0] << ","
                  << _state[i].tip_position.translation[1] << ")" << std::endl;

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

    if (!createModel()) {
        std::cerr << "Model setup failed" << std::endl;
        return 0;
    }

    if (!pauze(false)) {
        std::cerr << "Unable to start the simulation" << std::endl;
        return 0;
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

    pose_set(&_hexapod, 0, 0, 100, 0, 0, 0);
    pose_set(&_body, 0, 0, 0, 0, 0, 0);

    uint64_t t = 0;
    float32_t velocity = 100; // mm/s
    float32_t heading = 0;

    Gait gait_controller{};
    gait_controller.init(velocity, heading);

    while (!terminate) {
        // Wait for a sync pulse from gazebo on the clock topic
        auto status = _tick.wait_for(lk, std::chrono::seconds(2));

        if (status == std::cv_status::timeout) {
            terminate = true;
            break;
        }

        if (terminate) {
            break;
        }

        uint64_t delta_t = _time_us - last_time_us;
        last_time_us = _time_us;

        // std::cout << "Delta " << delta_t << std::endl;

        if (delta_t > 10000) {
            continue;
        }

        float32_t stepsize = velocity * delta_t / 1000000;
        _hexapod.translation[0] += stepsize * cos(heading);
        _hexapod.translation[1] += stepsize * sin(heading);

        MATRIX4(Thexapod);
        MATRIX4(Tbody);

        pose_get_transformation(&_hexapod, &Thexapod);
        pose_get_transformation(&_body, &Tbody);

        MATRIX4(T1);
        arm_mat_mult_f32(&Thexapod, &Tbody, &T1);

        gait_controller.update(_robot, delta_t);

        int i = 0;
        for (const Leg &leg: _robot.leg) {
            MATRIX4(Tcoxa);
            pose_get_transformation(&_state[i].body_position, &Tcoxa);

            MATRIX4(T);
            arm_mat_mult_f32(&T1, &Tcoxa, &T);

            MATRIX4(Ti);
            arm_mat_inverse_f32(&T, &Ti);

            _state[i].tip_position.translation[0] += gait_controller.delta_tip[i][0];
            _state[i].tip_position.translation[1] += gait_controller.delta_tip[i][1];
            _state[i].tip_position.translation[2] = gait_controller.delta_tip[i][2];

            float32_t tip_in_world[4] = {_state[i].tip_position.translation[0], _state[i].tip_position.translation[1],
                                         _state[i].tip_position.translation[2], 1};

            float32_t tip_in_coxa[4];
            arm_mat_vec_mult_f32(&Ti, tip_in_world, tip_in_coxa);

            float32_t origin[3] = {0, 0, 0};
            float32_t target[3] = {tip_in_coxa[0], tip_in_coxa[1], tip_in_coxa[2]};
            float32_t angles[3];
            inverse_kinematics(origin, target, angles);

            _state[i].joint_angles[0] = angles[0];
            _state[i].joint_angles[1] = angles[1];
            _state[i].joint_angles[2] = angles[2];

            for (int j = 0; j < 3; j++) {
                double angle = _state[i].joint_angles[j];
                if (j == 2) {
                    angle -= D2R(25);
                }
                gz::msgs::Double msg;
                msg.set_data(angle);

                _state[i].servo_publishers[j].Publish(msg);
            }
            i++;
        }
        t++;
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

    if (!pauze(true)) {
        std::cerr << "Unable to pauze the simulation" << std::endl;
        return 0;
    }

    std::this_thread::sleep_for(5000ms);

    if (!this->removeModel()) {
        std::cerr << "Unable to remove the model" << std::endl;
        return 0;
    }


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
        auto joint = model.joint(i);
        auto str = joint.name();
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

            this->_state[leg_index].actual_joint_angles[joint_index - 1] = (float32_t) joint.axis1().position();
        }
    }
}

int Controller::createModel() {
    gz::msgs::EntityFactory req{};
    gz::msgs::Boolean res;
    bool result;

    req.set_sdf_filename("/models/hexspider/hexspider.sdf");
    req.set_name("hexspider"); // New name for the entity, overrides the name on the SDF.
    req.set_allow_renaming(false); // allowed to rename the entity in case of overlap with existing entities


    bool executed = _node.Request("/world/hexspider_world/create", req, 1000, res, result);
    if (!executed) {
        std::cerr << "Timeout while calling create service" << std::endl;
        return 0;
    }

    if (!result) {
        std::cerr << "Service call failed" << std::endl;
        return 0;
    }

    std::cout << "Model created" << std::endl;
    return 1;
}

int Controller::removeModel() {
    gz::msgs::Entity req{};
    gz::msgs::Boolean res;
    bool result;

    req.set_name("hexspider");
    req.set_type(gz::msgs::Entity_Type_MODEL);

    bool executed = _node.Request("/world/hexspider_world/remove", req, 1000, res, result);
    if (!executed) {
        std::cerr << "Timeout while calling create service" << std::endl;
        return 0;
    }

    if (!result) {
        std::cerr << "Service call failed" << std::endl;
        return 0;
    }

    std::cout << "Model removed" << std::endl;
    return 1;
}

int Controller::pauze(bool pause) {
    gz::msgs::WorldControl req{};
    gz::msgs::Boolean res;
    bool result;

    req.set_pause(pause);

    bool executed = _node.Request("/world/hexspider_world/control", req, 1000, res, result);
    if (!executed) {
        std::cerr << "Timeout while calling create service" << std::endl;
        return 0;
    }

    if (!result) {
        std::cerr << "Service call failed" << std::endl;
        return 0;
    }

    std::cout << "Pause set to " << pause << "." << std::endl;
    return 1;
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


/**
*  @brief Motion controller takes the rc inputs an determines where the new body should be after at a specific
 *  time in the future. It does this by determining the velocity and rotation speeds. The result is a new
 *  pose for the hexapod, which can be used in subsequent steps
 *
 *  @param rc_inputs float32_t[3] [in]
*/
void motion_controller(float32_t rc_inputs[3]) {

}

/**
 * @brief gait controller determines the target postions of the legs, combining the
 * body target and the type of gait selected.
 *
 * It should maintain state about the position of the leg tips.
 *
 * @param
 */


