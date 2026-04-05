#!/usr/bin/env python3
"""
Test sequence:
  1. Start joint logger
  2. Start controller (via PTY so stdout stays line-buffered)
  3. Wait for "STANDING" state in controller output
  4. Publish velocity=50 for 10 seconds
  5. Publish velocity=0, wait for STANDING again
  6. Shutdown controller
  7. Plot commanded vs actual with gnuplot

Usage:
  python3 test_walk.py [joint_name]
  python3 test_walk.py leg_fr_servo_2      # femur of front-right leg
"""

import sys
import os
import subprocess
import threading
import time
import signal
import pty
import select

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
CONTROLLER   = os.path.join(SCRIPT_DIR, "Controller", "cmake-build-debug", "Controller")
LOG_SCRIPT   = os.path.join(SCRIPT_DIR, "log_joint.py")

# Ensure Gazebo transport uses loopback — must be set before any gz subprocess
GZ_ENV = {**os.environ, "GZ_IP": "127.0.0.1"}

JOINT        = sys.argv[1] if len(sys.argv) > 1 else "leg_fr_servo_2"
CSV          = f"/tmp/joint_{JOINT}_test.csv"
VEL_TOPIC    = "/world/hexspider_world/model/hexspider/velocity"
VELOCITY     = 50.0
WALK_SECONDS = 10
STANDING_TIMEOUT = 60   # seconds to wait for STANDING before giving up


def gz_publish(topic, value):
    subprocess.run(
        ["gz", "topic", "-t", topic, "-m", "gz.msgs.Double",
         "-p", f"data: {value}"],
        check=False, env=GZ_ENV
    )


def print_ts(msg):
    t = time.strftime("%H:%M:%S")
    print(f"[{t}] {msg}", flush=True)


# ── 1. Start joint logger ────────────────────────────────────────────────────
print_ts(f"Starting joint logger for {JOINT} → {CSV}")
with open(CSV, "w") as f:
    f.write("# time_s cmd actual\n")

logger = subprocess.Popen(
    ["python3", LOG_SCRIPT, JOINT, CSV],
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    env=GZ_ENV
)

# ── 2. Start controller via PTY (keeps C stdio line-buffered) ────────────────
print_ts("Starting controller...")
master_fd, slave_fd = pty.openpty()
controller = subprocess.Popen(
    [CONTROLLER],
    stdin=slave_fd, stdout=slave_fd, stderr=slave_fd,
    cwd=os.path.join(SCRIPT_DIR, "Controller"),
    env=GZ_ENV
)
os.close(slave_fd)

# Collect controller output in a thread so we can scan it
output_lines = []
output_lock  = threading.Lock()

def read_controller_output():
    buf = b""
    while True:
        try:
            r, _, _ = select.select([master_fd], [], [], 1.0)
            if r:
                data = os.read(master_fd, 4096)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    decoded = line.decode(errors="replace").rstrip("\r")
                    print(decoded, flush=True)          # mirror to terminal
                    with output_lock:
                        output_lines.append(decoded)
        except OSError:
            break

reader_thread = threading.Thread(target=read_controller_output, daemon=True)
reader_thread.start()


def wait_for_state(state_name, timeout=STANDING_TIMEOUT):
    """Block until the controller logs a transition to state_name."""
    deadline = time.monotonic() + timeout
    seen_up_to = 0
    while time.monotonic() < deadline:
        with output_lock:
            new_lines = output_lines[seen_up_to:]
            seen_up_to = len(output_lines)
        for line in new_lines:
            if f"Transitioning to motion state {state_name}" in line:
                return True
        time.sleep(0.1)
    return False


try:
    # ── 3. Wait for STANDING ─────────────────────────────────────────────────
    print_ts("Waiting for STANDING state...")
    if not wait_for_state("STANDING"):
        print_ts("WARNING: timed out waiting for STANDING — continuing anyway")
    else:
        print_ts("Robot is STANDING")

    time.sleep(0.5)   # brief pause before commanding motion

    # ── 4. Walk for WALK_SECONDS ─────────────────────────────────────────────
    print_ts(f"Publishing velocity={VELOCITY} mm/s")
    gz_publish(VEL_TOPIC, VELOCITY)

    print_ts(f"Walking for {WALK_SECONDS} seconds...")
    time.sleep(WALK_SECONDS)

    # ── 5. Stop and wait for STANDING again ──────────────────────────────────
    print_ts("Publishing velocity=0")
    gz_publish(VEL_TOPIC, 0.0)

    print_ts("Waiting for robot to return to STANDING...")
    if not wait_for_state("STANDING", timeout=15):
        print_ts("WARNING: timed out waiting for STANDING after stop")
    else:
        print_ts("Robot back to STANDING")

    time.sleep(0.5)

finally:
    # ── 6. Shutdown ───────────────────────────────────────────────────────────
    print_ts("Shutting down controller...")
    controller.send_signal(signal.SIGINT)
    try:
        controller.wait(timeout=5)
    except subprocess.TimeoutExpired:
        controller.kill()

    logger.send_signal(signal.SIGINT)
    try:
        logger.wait(timeout=3)
    except subprocess.TimeoutExpired:
        logger.kill()

    try:
        os.close(master_fd)
    except OSError:
        pass

    print_ts(f"Data saved to {CSV}")


# ── 7. Plot with gnuplot ──────────────────────────────────────────────────────
print_ts("Launching gnuplot...")
gp_file = f"/tmp/plot_{JOINT}.gp"
png_file = f"/tmp/plot_{JOINT}.png"
with open(gp_file, "w") as f:
    f.write(f"""set terminal pngcairo size 1200,500 enhanced font "Sans,11"
set output "{png_file}"
set title "Joint: {JOINT}"
set xlabel "Time (s)"
set ylabel "Position (rad)"
set grid
set key top right
set style line 1 lw 2 lc rgb "#e74c3c"
set style line 2 lw 2 lc rgb "#3498db"
plot "{CSV}" using 1:2 with lines ls 1 title "commanded", \\
     "{CSV}" using 1:3 with lines ls 2 title "actual"
""")
subprocess.run(["gnuplot", gp_file])
print_ts(f"Plot saved to {png_file}")
