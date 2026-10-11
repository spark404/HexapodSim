#!/usr/bin/env python3
"""
Publish a fixed target position to all 18 servo joints for stability testing.

Usage:
  python3 servo_test_position.py                              # defaults: 0.0, -0.517, 2.001
  python3 servo_test_position.py 0.0 -0.517 2.001             # explicit angles (rad)
  python3 servo_test_position.py --rate 25                    # republish at 25 Hz
  python3 servo_test_position.py --rate 25 --monitor          # republish + live error table

The three angles map to servo_1 (yaw), servo_2 (hip tilt), servo_3 (knee).
"""

import argparse
import os
import subprocess
import sys
import threading
import time

LEGS = ["br", "cr", "fr", "bl", "cl", "fl"]
GZ_ENV = {**os.environ, "GZ_IP": "127.0.0.1"}
JOINT_STATE_TOPIC = "/world/hexspider_world/model/hexspider/joint_state"


def publish(topic, value):
    subprocess.run(
        ["gz", "topic", "-t", topic, "-m", "gz.msgs.Double", "-p", f"data: {value}"],
        check=False, env=GZ_ENV, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )


def publish_all(angles):
    for leg in LEGS:
        for servo_idx, angle in enumerate(angles, start=1):
            topic = f"/model/hexspider/joint/leg_{leg}_servo_{servo_idx}/0/cmd_pos"
            publish(topic, angle)


# ── Joint state monitor ───────────────────────────────────────────────────────

# Shared dict: joint_name → actual position (rad)
actual_positions = {}
actual_lock = threading.Lock()


def monitor_joint_states():
    """Parse gz topic -e output for joint_state and update actual_positions."""
    proc = subprocess.Popen(
        ["gz", "topic", "-e", "-t", JOINT_STATE_TOPIC],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
        env=GZ_ENV
    )
    current_name = None
    in_axis1 = False
    for line in proc.stdout:
        line = line.strip()
        if line.startswith("name:"):
            current_name = line.split(":", 1)[1].strip().strip('"')
            in_axis1 = False
        elif line == "axis1 {":
            in_axis1 = True
        elif in_axis1 and line.startswith("position:"):
            try:
                val = float(line.split(":", 1)[1].strip())
                if current_name:
                    with actual_lock:
                        actual_positions[current_name] = val
            except ValueError:
                pass
            in_axis1 = False


def print_error_table(angles):
    """Print a one-shot table of target, actual, and error for all 18 joints."""
    header = f"{'Joint':<22} {'Target':>8} {'Actual':>8} {'Error':>8}"
    print(header)
    print("-" * len(header))
    max_error = 0.0
    with actual_lock:
        snap = dict(actual_positions)
    for leg in LEGS:
        for servo_idx, target in enumerate(angles, start=1):
            name = f"leg_{leg}_servo_{servo_idx}"
            actual = snap.get(name)
            if actual is not None:
                error = actual - target
                max_error = max(max_error, abs(error))
                flag = " !" if abs(error) > 0.05 else "  "
                print(f"{name:<22} {target:>8.4f} {actual:>8.4f} {error:>+8.4f}{flag}")
            else:
                print(f"{name:<22} {target:>8.4f} {'---':>8} {'---':>8}")
    print(f"\nMax error: {max_error:.4f} rad ({max_error:.1f} rad)")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Publish fixed joint targets to all legs.")
    parser.add_argument("angles", nargs="*", type=float,
                        default=[0.0, -0.517, 1.501],
                        help="Three joint angles in radians: servo_1 servo_2 servo_3")
    parser.add_argument("--rate", type=float, default=None,
                        help="Republish rate in Hz. Omit to publish once and exit.")
    parser.add_argument("--monitor", action="store_true",
                        help="Print live actual vs target error table (requires --rate).")
    args = parser.parse_args()

    if len(args.angles) != 3:
        print("Error: exactly 3 angles required (servo_1, servo_2, servo_3)", file=sys.stderr)
        sys.exit(1)

    angles = args.angles
    print(f"Target: servo_1={angles[0]:.4f}  servo_2={angles[1]:.4f}  servo_3={angles[2]:.4f} rad")
    print(f"Joints: {len(LEGS) * 3}  ({len(LEGS)} legs × 3 servos)")

    if args.monitor and args.rate is None:
        print("Warning: --monitor has no effect without --rate", file=sys.stderr)

    if args.monitor:
        t = threading.Thread(target=monitor_joint_states, daemon=True)
        t.start()
        time.sleep(0.5)   # let the subscriber get its first message

    if args.rate is None:
        publish_all(angles)
        print("Done.")
        return

    interval = 1.0 / args.rate
    print(f"Publishing at {args.rate} Hz — Ctrl-C to stop\n")

    try:
        while True:
            t0 = time.monotonic()
            publish_all(angles)

            if args.monitor:
                print(f"\033[2J\033[H", end="")   # clear screen
                print(f"Target: servo_1={angles[0]:.4f}  servo_2={angles[1]:.4f}  servo_3={angles[2]:.4f} rad\n")
                print_error_table(angles)

            elapsed = time.monotonic() - t0
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
