//
// Created by Hugo Trippaers on 09/07/2024.
//

#pragma once

#include <gz/transport.hh>
#include <gz/msgs.hh>

#include "Robot.h"

#define POWERDOWN_TIMEOUT 120 // seconds
#define CLOSE_BY_THRESHOLD 2 // mm

typedef enum {
    BOOT,
    SYNCING,
    STANDUP,
    STANDING,
    WALKING,
    POWERDOWN,
} state_t;

const std::string leg_names[6] {
    "fr",
    "cr",
    "br",
    "fl",
    "cl",
    "bl"
};

class Controller {
public:
     Controller();
    ~Controller();

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

    robot_state _robot_state{};
    std::array<std::array<gz::transport::Node::Publisher, 3>, 6> _servo_publishers;
    state_t _motion_state = BOOT;
    state_t _next_state = SYNCING;

    gz::transport::Node _node;
    std::mutex _node_mutex;

    std::mutex _tick_mutex;
    std::condition_variable _tick;

    float32_t _cmd_velocity = 60;
    float32_t _cmd_heading = 0;
    float32_t _cmd_height = 100;

    float32_t _velocity = 60;
    float32_t _orientation = 0;

    uint64_t _time_us{};

    std::array<std::array<float32_t, 3>, 6> _measured_servo_angles{};
};
