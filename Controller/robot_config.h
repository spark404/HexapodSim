/*
 * SPDX-FileCopyrightText: 2025 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "robot.h"
#include "hexapodmath/additional_functions.h"

static const struct robot r = {
    .step_size = 80, // mm
    .leg = {
        { // Front Right
            .mount_point_polar = {90, D2R(-55)},
            .tip_home_angles = {0.0f, 0.0f, D2R(90)},
            .servos = {16, 17, 18},
            .limits = {
                {-0.785398f, 0.785398f},
                {-1.919862f, 1.919862f},
                {-1.919862f, 2.617993f }
            }
        },
        { // Center Right
            .mount_point_polar = {70, D2R(-90)},
            .tip_home_angles = {0.0f, 0.0f, D2R(90)},
            .servos = {10, 11, 12},
            .limits = {
                {-0.785398f, 0.785398f},
                {-1.919862f, 1.919862f},
                {-1.919862f, 2.617993f }
            }
        },
        { // Rear Right
            .mount_point_polar = {90, D2R(-125)},
            .tip_home_angles = {0.0f, 0.0f, D2R(90)},
            .servos = {4, 5, 6},
            .limits = {
                {-0.785398f, 0.785398f},
                {-1.919862f, 1.919862f},
                {-1.919862f, 2.617993f }
            }
        },
        { // Front Left
            .mount_point_polar = {90, D2R(55)},
            .tip_home_angles = {0.0f, 0.0f, D2R(90)},
            .servos = {7, 8, 9},
            .limits = {
                {-0.785398f, 0.785398f},
                {-1.919862f, 1.919862f},
                {-1.919862f, 2.617993f }
            }
        },
        { // Center Left
            .mount_point_polar = {70, D2R(90)},
            .tip_home_angles = {0.0f, 0.0f, D2R(90)},
            .servos = {13, 14, 15},
            .limits = {
                {-0.785398f, 0.785398f},
                {-1.919862f, 1.919862f},
                {-1.919862f, 2.617993f }
            }
        },
        { // Rear Left
            .mount_point_polar = {90, D2R(125)},
            .tip_home_angles = {0.0f, 0.0f, D2R(90)},
            .servos = {1, 2, 3},
            .limits = {
                {-0.785398f, 0.785398f},
                {-1.919862f, 1.919862f},
                {-1.919862f, 2.617993f }
            }
        }
    }
};

#endif //ROBOT_CONFIG_H
