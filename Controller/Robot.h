//
// Created by Hugo Trippaers on 10/07/2024.
//

#pragma once

#include <string>
#include <array>
#include <gz/transport.hh>

#include <additional_functions.h>
#include <pose.h>

struct Leg {
    std::string name;
    float32_t polar_joint_offset[2];  // Polar coordinates for the coxa joint in the body frame
    float32_t tip_starting_position[3];
    float32_t tip_starting_angles[3];
};

struct Robot {
    std::array<Leg, 6> leg;
};

const Robot r {{{
    {"fr", {90, D2R(-55)},
     {1, 1, 1},
     {0, 0, M_PI_2}},
    {"cr", {70, D2R(-90)},
     {1, 1, 1},
     {0, 0, M_PI_2}},
    {"br",
     {90, D2R(-125)},
     {1, 1, 1},
     {0, 0, M_PI_2}},
    {"fl",
     {90, D2R(55)},
     {1, 1, 1},
     {0, 0, M_PI_2}},
    {"cl",
     {70, D2R(90)},
     {1, 1, 1},
     {0, 0, M_PI_2}},
    {"bl",
     {90, D2R(125)},
     {1, 1, 1},
     {0, 0, M_PI_2}}
}}};

struct LegState {
    struct pose coxa_joint_in_body_frame{};
    struct pose tip_position_in_world_frame{};
    struct pose tip_position_in_body_frame{};
    float32_t tip_target[3]{};
    float32_t tip_interpolated_target[3]{};
    float32_t joint_angles[3]{}; // 0 = coxa, 1 = femur, 2 = tibia
    float32_t actual_joint_angles[3]{}; // 0 = coxa, 1 = femur, 2 = tibia
    float32_t prev_joint_angles[3]{}; // 0 = coxa, 1 = femur, 2 = tibia
    bool grounded = true;
    std::array<gz::transport::Node::Publisher, 3> servo_publishers;
};