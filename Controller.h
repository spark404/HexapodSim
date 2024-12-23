//
// Created by Hugo Trippaers on 09/07/2024.
//

#pragma once

#include <gz/transport.hh>
#include <gz/msgs.hh>

#include "Robot.h"

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

    int createModel();

    volatile bool terminate = false;

    Robot _robot;
    struct pose _hexapod{};
    struct pose _body{};

    std::array<LegState, 6> _state;

    gz::transport::Node _node;
    std::mutex _node_mutex;

    std::mutex _tick_mutex;
    std::condition_variable _tick;

    uint64_t _time_us{};

    int removeModel();

    int pauze(bool pause);
};
