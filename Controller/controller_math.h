/*
 * SPDX-FileCopyrightText: 2025 Hugo Trippaers <hugo@trippaers.nl>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CONTROLLER_MATH_H
#define CONTROLLER_MATH_H

#include <arm_math.h>

static inline float32_t clampf(float32_t x, float32_t min, float32_t max) {
    return (x < min) ? min : (x > max) ? max : x;
}

static inline float32_t wrap_to_pi(float32_t angle) {
    angle = fmodf(angle + M_PI, 2.0f * M_PI);
    if (angle < 0.0f) angle += 2.0f * M_PI;
    return angle - M_PI;
}

#endif //CONTROLLER_MATH_H
