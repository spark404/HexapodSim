# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
# From the Controller directory
mkdir build && cd build
cmake ..
ninja
```

The executables are output to `build/Controller` and `build/HexapodControl` (or `cmake-build-debug/` for CLion debug builds). Always rebuild from the `Controller/` directory, not from inside `build/`, to keep the working directory correct for relative paths.

## Running the Simulation

Three terminals are typically needed:

**Terminal 1 — Gazebo simulator:**
```bash
cd ../HexSpider
./run.sh   # sets GZ_IP=127.0.0.1 and GZ_SIM_RESOURCE_PATH automatically
```

**Terminal 2 — Controller (after Gazebo is up):**
```bash
GZ_IP=127.0.0.1 ./build/Controller
```

**Terminal 3 — Send motion commands:**
```bash
GZ_IP=127.0.0.1 ./build/HexapodControl --velocity 50        # mm/s forward
GZ_IP=127.0.0.1 ./build/HexapodControl --heading 0.5        # radians
GZ_IP=127.0.0.1 ./build/HexapodControl --height 120         # mm
```

**`GZ_IP=127.0.0.1` is required** for all Controller and HexapodControl invocations and for any tool that subscribes to Gazebo topics (e.g. `gz topic`, `log_joint.py`). Without it, Gazebo transport discovery silently fails and topics appear empty.

The controller spawns the hexspider model on startup; the SDF changes take effect each time the controller is restarted (world reload is only needed when the world SDF itself changes).

## Joint Data Logging

`../log_joint.py` subscribes to both the command and actual position for one joint and writes a CSV:

```bash
GZ_IP=127.0.0.1 python3 ../log_joint.py leg_fr_servo_2 /tmp/servo2.csv
# Ctrl-C to stop; columns: time_s  cmd  actual
```

**Timing note:** start the logger *after* the controller has fully initialised and begun publishing cmd_pos topics (~8 seconds after launch). Starting it too early results in `cmd=nan` for the entire run. You can confirm topics are live with:
```bash
GZ_IP=127.0.0.1 gz topic -l | grep cmd_pos
```

## Architecture

The project is split into two sibling directories under `HexapodSim/`:

- **`Controller/`** — C/C++ control logic
- **`HexSpider/`** — Gazebo SDF model and world definition

### Controller Layer

`main.cpp` → `HexapodController` (C++) → `controller_update()` (C)

- **`HexapodController.cpp`** — Gazebo transport bridge. Subscribes to clock, joint state, IMU, velocity, heading, and height topics. On each clock tick (100ms minimum gate), reads measured servo positions, applies angle compensation, calls `controller_update()`, then publishes 18 servo command topics.

- **`controller.c`** — Core state machine and gait logic. States: `CTRL_BOOT → CTRL_SYNCING → CTRL_STANDUP → CTRL_STANDING ↔ CTRL_WALKING / CTRL_ROTATING → CTRL_POWERDOWN`. Implements tripod gait (legs [0,2,4] and [1,3,5] alternate), inverse kinematics via the `HexapodMath` library, and 4-point swing trajectory generation.

- **`calculator.c`** — Trajectory interpolation helper used during gait.

- **`HexapodControl.cpp`** — Standalone CLI tool. Publishes a single velocity/heading/height command to the running controller and exits.

### Key Configuration Files

- **`robot_config.h`** — Leg mount positions, servo angle limits, step size (80mm), and servo ID mappings for all 6 legs (FR, CR, BR, FL, CL, BL).
- **`controller_config.h`** — Control loop timing (100ms controller tick; the 25 Hz servo loop lives in `HexapodController.cpp`), motion parameters (body height 100mm, lift height 50mm, default velocity 50mm/s).
- **`controller_types.h`** — State machine enums and the main `controller_ctx_t` struct.
- **`robot.h`** — Runtime robot state structs (leg poses, angles, tip positions, grounded status).

### Angle Compensation

There is a coupling between the physical servo geometry and the software model in `HexapodController.cpp`:
- Femur measured angle is negated: `-measured[1]`
- Tibia has a 25° mechanical offset: add `D2R(25)` on measurement read, subtract on command write
- `alpha` (currently `0.f`) blends measured vs commanded angles as feedback. `alpha=0` = full sensor feedback; `alpha=1` = open-loop (ignore measurements). Keep at `0.f`.

### Gazebo Topics

| Direction | Topic pattern |
|-----------|---------------|
| In | `/world/hexspider_world/clock` |
| In | `/world/hexspider_world/model/hexspider/joint_state` |
| In | `/model/hexspider/imu` |
| In | `/world/hexspider_world/model/hexspider/velocity` |
| In | `/world/hexspider_world/model/hexspider/heading` |
| In | `/world/hexspider_world/model/hexspider/height` |
| Out | `/model/hexspider/joint/leg_<NAME>_servo_<N>/0/cmd_pos` (18 topics) |

### External Dependencies

Fetched automatically by CMake if not found locally:
- **`HexapodMath`** (spark404/HexapodMath) — forward/inverse kinematics and 3D matrix transforms
- **`cmsis-dsp`** — ARM CMSIS-DSP vector/matrix math (`arm_vec_*_f32`, `arm_mat_*_f32`)

## SDF Physics Parameters

### Joint limits (`HexSpider/models/hexspider/hexspider.sdf`)

All 18 joints use `gz-sim-joint-position-controller-system` in force (PID) mode, configured to behave like a Dynamixel XL430 / 2XL430 in position mode at ~11.1 V. All servos use the same values:

| Parameter | Value | Dynamixel equivalent |
|-----------|-------|----------------------|
| `p_gain` | 5 Nm/rad | Position P Gain 640 (default): full PWM at ~0.27 rad error |
| `i_gain` / `d_gain` | 0 | Position I / D Gain 0 (default) |
| `cmd_max` / `cmd_min`, `<effort>` | ±1.4 Nm | stall torque at 11.1 V (1.5 Nm at 12 V) |
| joint `<damping>` | 0.23 Nm·s/rad | stall torque / no-load speed (1.4 / 5.97 rad/s): motor back-EMF |
| joint `<friction>` | 0.03 Nm | gearbox friction (estimate, not measured) |
| `<velocity>` | 10 rad/s | not limiting; the damping sets the ~57 rpm top speed |

The P loop capped at stall torque plus joint damping reproduces the DC motor's linear torque-speed curve. Do not switch back to `use_velocity_commands`: that mode tracks the target at error/dt (1 ms), an effectively rigid servo that hides compliance and lets stance legs fight each other through the ground (±10% run-to-run scatter, and it needs ~5 Nm to walk).

Because the servos are compliant, a loaded joint needs a standing position error to carry load (the body sags ~7 mm at standing height). The controller therefore plans stand-up and swing starts from the last commanded joint angles (`next_joint_angles`), not the measured ones. Planning from measured angles stalls STANDUP.

### Contact / friction (`HexSpider/models/hexspider/hexspider.sdf` and `hexspider_world.sdf`)

Foot collision geometry: the femur mesh (`Femur_20260405_1.stl`) on each `leg_XX_femur` link; the contact point is the lowest part of the mesh and rolls along it as the leg moves.

Both the foot surface and the ground plane use the values below. Lower ground friction (e.g. 0.8/0.4) causes measurable foot slip and speed loss:
```xml
<surface>
  <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
  <contact><ode><kp>10000</kp><kd>10</kd></ode></contact>
</surface>
```

`kp=10000` is chosen to keep the ODE stability ratio `kp·dt²/m < 1` at the 1ms physics step. Higher values (>28k N/m for the 28g femur link) cause numerical blow-up. This allows ~0.6mm of static foot sinking, which is imperceptible but numerically stable.
