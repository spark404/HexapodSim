#!/usr/bin/env python3
"""
Log commanded and actual position for one joint to a CSV file.

Usage: python3 log_joint.py <joint_name> <output_csv>
Example: python3 log_joint.py leg_fr_servo_1 /tmp/joint_leg_fr_servo_1.csv

The CSV has three columns: time_s, commanded, actual
NaN is written when only one of the two values is available for that timestamp.
"""

import sys
import subprocess
import threading
import time

joint = sys.argv[1] if len(sys.argv) > 1 else "leg_fr_servo_1"
csv_path = sys.argv[2] if len(sys.argv) > 2 else f"/tmp/joint_{joint}.csv"

cmd_topic   = f"/model/hexspider/joint/{joint}/0/cmd_pos"
state_topic = "/world/hexspider_world/model/hexspider/joint_state"

t0 = time.monotonic()
lock = threading.Lock()

latest_cmd    = None   # most recent commanded position
latest_actual = None   # most recent actual position

def now():
    return time.monotonic() - t0


def read_cmd():
    """Parse gz topic -e output for a Double message (just 'data: <value>')."""
    global latest_cmd
    proc = subprocess.Popen(
        ["gz", "topic", "-e", "-t", cmd_topic],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True
    )
    for line in proc.stdout:
        line = line.strip()
        if line.startswith("data:"):
            try:
                val = float(line.split(":")[1].strip())
                with lock:
                    latest_cmd = (now(), val)
            except ValueError:
                pass


def read_actual():
    """
    Parse gz topic -e output for a Model (joint_state) message.
    The text format groups fields per joint:
        joint {
          name: "leg_fr_servo_1"
          axis1 {
            position: 0.1234
          }
        }
    """
    global latest_actual
    proc = subprocess.Popen(
        ["gz", "topic", "-e", "-t", state_topic],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True
    )

    in_target_joint = False
    in_axis1 = False

    for line in proc.stdout:
        line = line.strip()

        if line.startswith('name:'):
            name = line.split(':', 1)[1].strip().strip('"')
            in_target_joint = (name == joint)
            in_axis1 = False

        elif in_target_joint and line == "axis1 {":
            in_axis1 = True

        elif in_axis1 and line.startswith("position:"):
            try:
                val = float(line.split(":")[1].strip())
                with lock:
                    latest_actual = (now(), val)
            except ValueError:
                pass
            in_axis1 = False

#        elif line == "}":
#            if in_axis1:
#                in_axis1 = False


# Start reader threads
threading.Thread(target=read_cmd,    daemon=True).start()
threading.Thread(target=read_actual, daemon=True).start()

print(f"Logging {joint} → {csv_path}  (Ctrl-C to stop)", flush=True)

with open(csv_path, "w") as f:
    f.write("# time_s cmd actual\n")

    try:
        while True:
            time.sleep(0.04)   # ~25 Hz log rate

            with lock:
                t_cmd,    v_cmd    = latest_cmd    if latest_cmd    else (None, None)
                t_actual, v_actual = latest_actual if latest_actual else (None, None)

            if v_cmd is None and v_actual is None:
                continue

            t   = t_cmd    if t_cmd    is not None else t_actual
            cmd = v_cmd    if v_cmd    is not None else float('nan')
            act = v_actual if v_actual is not None else float('nan')


            f.write(f"{t:.4f} {cmd:.6f} {act:.6f}\n")
            f.flush()

    except KeyboardInterrupt:
        pass
