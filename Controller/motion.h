//
// Created by Hugo Trippaers on 12/04/2025.
//

#ifndef MOTION_H
#define MOTION_H

#include "Robot.h"

class BaseMotion {
public:
    virtual ~BaseMotion() = default;

    virtual void init() = 0;
    virtual void calculate(const Robot &robot, std::array<LegState, 6> &state, const float32_t movement_vector[3], float32_t delta_t_ms) = 0;
    virtual void update(std::array<LegState, 6> &state) = 0;
};


#endif //MOTION_H
