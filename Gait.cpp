//
// Created by Hugo Trippaers on 02/11/2024.
//

#include "Gait.h"
#include "Robot.h"

#include "arm_math.h"

void Gait::init(float32_t velocity, float32_t angular_velocity) {
    // number of steps in a single gait cycle
    _velocity = velocity;
    _angular_velocity = angular_velocity;
    _steps_in_cycle = 80 / velocity * 1000;
    _current_step = _steps_in_cycle / 2;

    _state = 0; // Starting
}

void Gait::update(const Robot &robot, uint64_t delta_t) {
    int i = 0;
    float32_t stepsize = _velocity * delta_t / 1000000;
    float32_t rotation = _angular_velocity * delta_t / 1000000;
    float32_t heading = 0;

    for (const Leg &leg: robot.leg) {
        float32_t current_z = arm_sin_f32(M_PI / _steps_in_cycle * (_current_step % _steps_in_cycle)) * 40;
        float32_t step = stepsize * 2;
        if (i == 1 || i == 3 || i == 5) {
            // plus 40 mm in the direction of the motion in the body frame
            // then calculate steps remaining to go from current to target with the current velocity
            // which is twice the velocity of the current motion.
            if (_state) {
                delta_tip[i][0] = step * cos(heading);
                delta_tip[i][1] = step * sin(heading);
                delta_tip[i][2] = current_z - leg.tip_starting_position[2];
            } else {
                delta_tip[i][0] = 0;
                delta_tip[i][1] = 0;
                delta_tip[i][2] = 0;
            }
        } else {
            if (!_state) {
                delta_tip[i][0] = step * cos(heading);
                delta_tip[i][1] = step * sin(heading);
                delta_tip[i][2] = current_z - leg.tip_starting_position[2];
            } else {
                delta_tip[i][0] = 0;
                delta_tip[i][1] = 0;
                delta_tip[i][2] = 0;
            }
        }


        i++;
    }
    _current_step = (_current_step + 1) % _steps_in_cycle;
    if (_current_step == 0) {
        _state = !_state;
    }
}


