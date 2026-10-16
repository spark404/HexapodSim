// Drives controller.c through stand up, walking forward and stopping, with ideal servos
// (the measured joint angles equal the angles commanded on the previous tick), and writes
// the commanded joint angles of every leg on every tick:
//
//   <tick> <controller state> <18 joint angles>
//
// run.sh builds this against the firmware's controller.c and the simulator's controller.c
// and compares the two outputs. See run.sh.
#include <stdio.h>
#include <string.h>

#include "controller.h"
#include "log.h"

#ifdef FW
/* The firmware's log.h wants the application to define the level; silence the log */
int g_log_level = LOG_LEVEL_FATAL + 1;
#endif

#define TICKS 450
#define WALK_FROM 40
#define WALK_TO 340

static controller_ctx_t ctx;

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <output file>\n", argv[0]);
        return 2;
    }
    FILE *out = fopen(argv[1], "w");
    if (out == NULL) {
        perror(argv[1]);
        return 2;
    }

    controller_init(&ctx);
    controller_set_state_callback(&ctx, NULL, NULL);

    controller_attitude_t attitude = {0};
    controller_command_t cmd = {.velocity = 0, .heading = 0, .height = 100};

    /* The standing pose of the real robot, so the first real data is already valid */
    for (int i = 0; i < 6; i++) {
        const float rest[3] = {0.0f, -0.574f, 2.0f};
        memcpy(ctx.robot.leg_state[i].actual_joint_angles, rest, sizeof(rest));
    }

    for (int tick = 0; tick < TICKS; tick++) {
        cmd.velocity = (tick >= WALK_FROM && tick < WALK_TO) ? 50.0f : 0.0f;
        controller_update(&ctx, &attitude, &cmd, 0.1f);

        fprintf(out, "%d %d", tick, (int) ctx.state);
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 3; j++) {
                fprintf(out, " %.5f", ctx.robot.leg_state[i].next_joint_angles[j]);
            }
        }
        fprintf(out, "\n");

        /* Ideal servos: every leg is where it was told to be */
        for (int i = 0; i < 6; i++) {
            memcpy(ctx.robot.leg_state[i].actual_joint_angles, ctx.robot.leg_state[i].next_joint_angles,
                   sizeof(float) * 3);
        }
    }

    fclose(out);
    return 0;
}
