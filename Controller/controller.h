/*
 * SPDX-FileCopyrightText: 2025 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controller_types.h"
#ifdef __cplusplus
extern "C" {
#endif
void controller_init(controller_ctx_t *ctx);

void controller_set_state_callback(
    controller_ctx_t *ctx,
    controller_state_cb_t cb,
    void *user_data
);

void controller_update(
    controller_ctx_t *ctx,
    const controller_command_t *cmd,
    float32_t dt_s
);

const char *controller_state_to_string(controller_state_t state);
#ifdef __cplusplus
}
#endif
#endif //CONTROLLER_H
