#!/bin/bash
#
# Checks that the simulator's controller.c still behaves like the firmware's.
#
# The two copies differ on purpose in one respect: the simulator plans from the last
# commanded joint angles, the firmware from the measured ones. With ideal servos (the
# measured angles equal the commanded ones, see harness.c) that difference does not show,
# so everything else, state machine, swing trajectory, gait, must give the same joint
# angles on every tick. The test fails when one copy changes and the other does not.
#
#   tests/controller_equivalence/run.sh
#
# Needs a C compiler and the host builds of the CMSIS-DSP and HexapodMath libraries that the
# simulator's CMake build fetches: build the Controller once (cmake-build-debug or build).
#
# Environment:
#   FIRMWARE_DIR  the Hexapod-Controller checkout   (default: ../Hexapod-Controller next to this repo)
#   DEPS_DIR      the _deps directory of the build   (default: cmake-build-debug/_deps or build/_deps)
#   SIM_DIR       the simulator controller sources   (default: Controller)
set -u

# Defaults are relative to this script, so SIM_DIR can point somewhere else (for example
# at an older copy of the simulator controller) without moving the other defaults
HERE="$(cd "$(dirname "$0")" && pwd)"
CONTROLLER_DIR="$(cd "$HERE/../.." && pwd)"
SIM_DIR="${SIM_DIR:-$CONTROLLER_DIR}"
FIRMWARE_DIR="${FIRMWARE_DIR:-$(cd "$CONTROLLER_DIR/../../Hexapod-Controller" 2>/dev/null && pwd)}"

DEPS_DIR="${DEPS_DIR:-}"
if [ -z "$DEPS_DIR" ]; then
    for d in "$CONTROLLER_DIR/cmake-build-debug/_deps" "$CONTROLLER_DIR/build/_deps"; do
        [ -f "$d/hexapodmath-build/src/libHexapodMath.a" ] && DEPS_DIR="$(cd "$d" && pwd)" && break
    done
fi

fail() { echo "controller_equivalence: $*" >&2; exit 2; }

[ -n "$FIRMWARE_DIR" ] && [ -f "$FIRMWARE_DIR/Core/Src/controller.c" ] || fail "firmware checkout not found, set FIRMWARE_DIR"
[ -n "$DEPS_DIR" ] || fail "no built dependencies found, build the Controller first or set DEPS_DIR"
DSP="$DEPS_DIR/cmsis-dsp-src/Include"
HM="$DEPS_DIR/hexapodmath-src/include"
LIBS="$DEPS_DIR/hexapodmath-build/src/libHexapodMath.a $DEPS_DIR/cmsis-dsp-build/Source/libCMSISDSP.a"
for f in "$DSP/arm_math.h" "$HM/hexapodmath/hexapod.h" $LIBS; do
    [ -e "$f" ] || fail "missing $f"
done

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

COMMON="-std=gnu11 -O1 -w -I$DSP -I$DSP/../PrivateInclude -I$HM -I$FIRMWARE_DIR/Drivers/CMSIS/Include"

cc $COMMON -DFW -I"$FIRMWARE_DIR/Core/Inc" "$HERE/harness.c" \
    "$FIRMWARE_DIR/Core/Src/controller.c" "$FIRMWARE_DIR/Core/Src/calculator.c" $LIBS -lm -o "$WORK/firmware" \
    || fail "building the firmware controller failed"
cc $COMMON -I"$SIM_DIR" "$HERE/harness.c" \
    "$SIM_DIR/controller.c" "$SIM_DIR/calculator.c" $LIBS -lm -o "$WORK/simulator" \
    || fail "building the simulator controller failed"

"$WORK/firmware" "$WORK/firmware.txt" > /dev/null || fail "the firmware controller run failed"
"$WORK/simulator" "$WORK/simulator.txt" > /dev/null || fail "the simulator controller run failed"

# The scenario must have exercised the code: syncing, standing up, standing and walking
read -r syncing standup walking < <(awk '$2==1{a++} $2==2{b++} $2==4{c++} END{print a+0, b+0, c+0}' "$WORK/firmware.txt")
if [ "$syncing" -lt 1 ] || [ "$standup" -lt 1 ] || [ "$walking" -lt 200 ]; then
    fail "the scenario did not reach the states it should (syncing $syncing, standup $standup, walking $walking ticks)"
fi

total=$(wc -l < "$WORK/firmware.txt" | tr -d " ")
if cmp -s "$WORK/firmware.txt" "$WORK/simulator.txt"; then
    echo "controller_equivalence: ok, identical joint angles on all $total ticks ($walking walking)"
    exit 0
fi

# Report where they part: the first tick, and the largest joint angle difference
paste -d'\n' "$WORK/firmware.txt" "$WORK/simulator.txt" | awk -v total="$total" '
    NR % 2 == 1 { split($0, f, " "); next }
    {
        split($0, s, " "); d = 0
        for (i = 3; i <= 20; i++) { x = f[i] - s[i]; if (x < 0) x = -x; if (x > d) d = x; if (x > m) m = x }
        if (d > 0.00001) { n++; if (!first) first = f[1] + 0 }
    }
    END { printf "controller_equivalence: FAILED, %d of %d ticks differ, the first at tick %d, largest joint angle difference %.5f rad\n", n, total, first, m }'
exit 1
