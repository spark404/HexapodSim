//
// Created by Hugo Trippaers on 11/01/2025.
//

#ifndef CONTROLLER_STANDUP_H
#define CONTROLLER_STANDUP_H

#include "Robot.h"
#include "motion.h"

class Standup final : public BaseMotion {
public:
    void init() override;
    void calculate(const Robot &robot, std::array<LegState, 6> &state, const float32_t movement_vector[3]) const override;
    void update(std::array<LegState, 6> &state) override;
private:
    int current_leg = 0;
    float32_t _lift_height = 50;
    float32_t _lift_incline_factor = 2;
};


#endif //CONTROLLER_STANDUP_H
