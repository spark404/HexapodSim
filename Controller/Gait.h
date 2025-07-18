//
// Created by Hugo Trippaers on 02/11/2024.
//

#ifndef CONTROLLER_GAIT_H
#define CONTROLLER_GAIT_H

#include "Robot.h"
#include "motion.h"

class Gait final : public BaseMotion {
public:
    void init() override;
    void calculate(const Robot &robot, std::array<LegState, 6> &state, const float32_t movement_vector[3], float32_t delta_t_ms) override;
    void update(std::array<LegState, 6> &state) override;
private:
    float32_t _step_size = 40.0f; // mm
    float32_t _lift_height = 25.0f; // mm
    float32_t _lift_velocity = 10.0f; // mm/s
    float32_t _lift_incline_factor = 2.0f;
    void calculate_motion_step(float32_t current[3], float32_t target[3], float32_t next[3], float32_t delta_t_s);
};

#endif //CONTROLLER_GAIT_H
