/*
 * SPDX-FileCopyrightText: 2025 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONTROLLER_TYPES_H
#define CONTROLLER_TYPES_H

#include <arm_math.h>
#include "robot.h"

typedef enum {
    CTRL_BOOT,
    CTRL_SYNCING,
    CTRL_STANDUP,
    CTRL_STANDING,
    CTRL_WALKING,
    CTRL_POWERDOWN,
} controller_state_t;

typedef struct {
    float32_t velocity;   // mm/s
    float32_t heading;    // rad (world)
    float32_t height;     // mm
} controller_command_t;

/* Forward declaration */
struct controller_ctx;

/* State change callback signature */
typedef void (*controller_state_cb_t)(
    struct controller_ctx *ctx,
    controller_state_t from,
    controller_state_t to,
    void *user_data
);

typedef struct controller_ctx {
    const struct robot *cfg;

    controller_state_t state;
    controller_state_t next_state;

    float32_t powerdown_timeout;

    float32_t yaw_error;
    float32_t delta_yaw;
    float32_t pending_yaw;

    struct robot_state robot;

/* Callback */
    controller_state_cb_t on_state_change;
    void *cb_user_data;
} controller_ctx_t;

#endif //CONTROLLER_TYPES_H
