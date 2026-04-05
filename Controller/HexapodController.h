//
// Created by Hugo Trippaers on 09/07/2024.
//

#pragma once

#include <gz/transport.hh>
#include <gz/msgs.hh>

#include "robot.h"
#include "controller.h"

const std::string leg_names[6] {
    "fr",
    "cr",
    "br",
    "fl",
    "cl",
    "bl"
};

class HexapodController {
public:
     HexapodController();
    ~HexapodController();

    void init();
    void run();
    void shutdown();

private:
    void clockCallback(const gz::msgs::Clock &clock);
    void jointStateCallback(const gz::msgs::Model &model);
    void imuCallback(const gz::msgs::IMU &imu);
    void velocityCallback(const gz::msgs::Double &velocity);
    void headingCallback(const gz::msgs::Double &heading);
    void heightCallback(const gz::msgs::Double &heading);

    int read_actual_servo_position(int leg_id, uint8_t servo_count, float32_t *actual_servo_angles) const;
    static int write_next_servo_position(const std::array<gz::transport::Node::Publisher, 3>& servos, uint8_t servo_count, const float32_t *actual_servo_angles);

    volatile bool terminate = false;

    std::array<std::array<gz::transport::Node::Publisher, 3>, 6> _servo_publishers;

    gz::transport::Node _node;
    std::mutex _node_mutex;

    std::mutex _tick_mutex;
    std::condition_variable _tick;

    float32_t _cmd_velocity = 0;
    float32_t _cmd_heading = 0;
    float32_t _cmd_height = 100;

    controller_attitude_t _attitude{};

    uint64_t _time_us{};

    controller_ctx_t _ctx;

    std::array<std::array<float32_t, 3>, 6> _measured_servo_angles{};

    float32_t _actual_joint_angles[6][3]{};   // written by servo on every tick, read by controller tick
    float32_t _target_joint_angles[6][3]{};   // written by controller tick, read by servo tick
    float32_t _last_servo_velocity[6][3]{};   // velocity state for servo interpolator
};
