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
    void velocityCallback(const gz::msgs::Double &velocity);
    void headingCallback(const gz::msgs::Double &heading);
    void heightCallback(const gz::msgs::Double &heading);

    int read_actual_servo_position(int leg_id, uint8_t servo_count, float32_t *actual_servo_angles);
    int write_next_servo_position(const std::array<gz::transport::Node::Publisher, 3>& servos, uint8_t servo_count, float32_t *actual_servo_angles);

    volatile bool terminate = false;

    std::array<std::array<gz::transport::Node::Publisher, 3>, 6> _servo_publishers;

    gz::transport::Node _node;
    std::mutex _node_mutex;

    std::mutex _tick_mutex;
    std::condition_variable _tick;

    float32_t _cmd_velocity = 60;
    float32_t _cmd_heading = 0;
    float32_t _cmd_height = 100;

    uint64_t _time_us{};

    controller_ctx_t _ctx;

    std::array<std::array<float32_t, 3>, 6> _measured_servo_angles{};
};
