//
// Created by Hugo Trippaers on 02/11/2024.
//

#ifndef CONTROLLER_GAIT_H
#define CONTROLLER_GAIT_H

#include "Robot.h"

class Gait {
public:
    void init(float32_t velocity, float32_t heading);
    void update(const Robot &robot, uint64_t delta_t);
    float32_t delta_tip[6][3];
private:
    int _state;
    float32_t _velocity;
    float32_t _angular_velocity;
    int _steps_in_cycle;
    int _current_step;

};


#endif //CONTROLLER_GAIT_H
