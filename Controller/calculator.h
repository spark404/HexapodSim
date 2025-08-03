//
// Created by Hugo Trippaers on 20/07/2025.
//

#ifndef CALCULATOR_H
#define CALCULATOR_H

#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /// Velocity in the horizontal plane (mm/s)
    float32_t velocity;
    /// Velocity in the vertical plane (mm/s)
    float32_t lift_velocity;
    /// Lift height from target position (mm)
    float32_t lift_height;
} motion_param_t;

void calculate_motion_step(motion_param_t *motion, float32_t current[3], float32_t target[3], float32_t delta_t_s, float32_t next[3], float32_t *distance_remaining);

#ifdef __cplusplus
}
#endif

#endif //CALCULATOR_H
