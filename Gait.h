//
// Created by Hugo Trippaers on 02/11/2024.
//

#ifndef CONTROLLER_GAIT_H
#define CONTROLLER_GAIT_H

#include "Robot.h"

class Gait {
public:
    void init();
    void calculate(const Robot &robot, std::array<LegState, 6> &state, const float32_t movement_vector[3], uint64_t delta_t);
    void update(const Robot &robot, std::array<LegState, 6> &state, const float32_t movement_vector[3], uint64_t delta_t);
    float32_t delta_tip[6][3];
private:
    float32_t _step_size = 40.0f; // mm
    float32_t _lift_height = 25.0f; // mm
    float32_t _lift_incline_factor = 2.0f;
};


#endif //CONTROLLER_GAIT_H
