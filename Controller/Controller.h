//
// Created by Hugo Trippaers on 09/07/2024.
//

#pragma once

#include <gz/transport.hh>
#include <gz/msgs.hh>

#include "Robot.h"
#include "motion.h"

enum motion_state {
    INITIALIZING,
    STANDING,
    WALKING
};

class Controller {
public:
    explicit Controller(Robot robot);
    ~Controller();

    int init();
    int run();
    void shutdown();

private:
    void clockCallback(const gz::msgs::Clock &clock);
    void jointStateCallback(const gz::msgs::Model &model);

    volatile bool terminate = false;

    Robot _robot;
    struct pose _hexapod{};
    struct pose _body{};

    motion_state _motion_state = INITIALIZING;
    std::array<LegState, 6> _state;

    gz::transport::Node _node;
    std::mutex _node_mutex;

    std::mutex _tick_mutex;
    std::condition_variable _tick;

    BaseMotion *_base_motion = nullptr;

    uint64_t _time_us{};
};
