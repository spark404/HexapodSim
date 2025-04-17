//
// Created by Hugo Trippaers on 16/04/2025.
//
#include<string>
#include<iostream>
#include<gz/msgs.hh>

#include "ServoTest.h"

#include <additional_functions.h>

int ServoTest::init() {
    return 1;
}

int ServoTest::run() {
    using namespace std::chrono_literals;

    std::unique_lock<std::mutex> lk(_tick_mutex);
    uint64_t last_time_us = 0;

    std::cout << "Starting listeners for joint state" << std::endl;
    std::string joint_state_topic = "/world/hexspider_world/model/hexspider/joint_state";
    if (!(_node.Subscribe(joint_state_topic, &ServoTest::jointStateCallback, this))) {
        std::cerr << "Failed to subscribe to joint_state topic" << std::endl;
        return 0;
    }

    std::cout << "Starting main loop using world clock ticks" << std::endl;
    std::string clock_topic = "/world/hexspider_world/clock";
    if (!(_node.Subscribe(clock_topic, &ServoTest::clockCallback, this))) {
        std::cerr << "Failed to subscribe to clock topic" << std::endl;
        return 0;
    }

    const std::string topic = "/model/hexspider/joint/leg_cr_servo_2/0/cmd_pos";
    auto p = _node.Advertise<gz::msgs::Double>(topic);
    if (!p) {
        std::cerr << "Error advertising topic [" << topic << "]" << std::endl;
        return -1;
    }

    int first_tick = 1;
    int countdown = 1000;
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

        if (countdown == 0 && first_tick) {
            gz::msgs::Double msg;
            msg.set_data(D2R(-45));

            p.Publish(msg);
            first_tick = 0;
        }

        if (first_tick) { countdown--; }

        std::cout << delta_t << std::endl;
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


void ServoTest::shutdown() {
    terminate = true;
    _tick.notify_one();
}

void ServoTest::clockCallback(const gz::msgs::Clock &clock) {
    const uint64_t time_us = (clock.sim().sec() * 1000000) + (clock.sim().nsec() / 1000);
    if (time_us <= _time_us) {
        return;
    }
    _time_us = time_us;

    // Tick the main loop of the controller
    _tick.notify_one();
}

void ServoTest::jointStateCallback(const gz::msgs::Model &model) {
    for (int i = 0; i < model.joint_size(); i++) {
        const auto& joint = model.joint(i);
        const auto& str = joint.name();

        if (str.find("leg_cr_servo_2") != std::string::npos) {
            auto v = (float32_t) joint.axis1().position();

            std::cout << v << std::endl;
        }
    }
}


