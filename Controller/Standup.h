//
// Created by Hugo Trippaers on 11/01/2025.
//

#ifndef CONTROLLER_STANDUP_H
#define CONTROLLER_STANDUP_H

#include "Robot.h"

class Standup {
public:
    void init();
    void calculate(const Robot &robot, std::array<LegState, 6> &state, const float32_t movement_vector[3]) const;
    static void update(std::array<LegState, 6> &state);
private:
    int current_leg = 0;
    float32_t _lift_height = 50;
    float32_t _lift_incline_factor = 2;
};


#endif //CONTROLLER_STANDUP_H
