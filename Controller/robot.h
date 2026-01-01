/*
 * SPDX-FileCopyrightText: 2025 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ROBOT_H
#define ROBOT_H

#include "arm_math.h"
#include "hexapodmath/pose.h"

struct leg {
    /// Mount point of the leg to the body in polar coordinates
    float32_t mount_point_polar[2];
    /// The angles defined as home position
    float32_t tip_home_angles[3];
    /// The servos making up this leg's joints, order Coxa, Femur, Tibia.
    /// Use the actual servo ID
    uint8_t servos[3];
    /// Servo limits min and max
    float32_t limits[3][2];
};

struct robot {
    struct leg leg[6];
    float32_t step_size;
};

struct leg_state {
    struct pose coxa_body_joint;
    arm_matrix_instance_f32 coxa_mat;
    arm_matrix_instance_f32 coxa_mat_inv;
    float32_t coxa_mat_data[16];
    float32_t coxa_mat_inv_data[16];
    /// The position of the leg tip in the body frame with the angles at the home position
    float32_t tip_home[3];
    /// The actual joint angles for this leg (coxa, femur, tibia) in rad
    float32_t actual_joint_angles[3];
    /// The calculated next joint angles for this leg (coxa, femur, tibia) in rad
    float32_t next_joint_angles[3];
    /// Indicates if the leg is lifted off the ground or not
    uint8_t grounded;
    /// Tip position in the world fram
    float32_t tip_world_coordinates[3];
};

struct robot_state {
    struct pose hexapod;
    struct pose body;
    struct leg_state leg_state[6];
};

#endif //ROBOT_H
