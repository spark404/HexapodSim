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
    void calculate(const Robot &robot, std::array<LegState, 6> &state, const float32_t movement_vector[3], float32_t delta_t_ms) override;
    void update(std::array<LegState, 6> &state) override;
private:
    float32_t _lift_height = 30;   // mm
    float32_t _lift_velocity = 80; //mm/s
    float32_t _velocity = 50;      // mm/s

    void calculate_motion_step(float32_t current[3], float32_t target[3], arm_matrix_instance_f32 *transform, float32_t delta_t_s, float32_t angles[3]);
};


#endif //CONTROLLER_STANDUP_H
