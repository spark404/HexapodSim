#!/usr/bin/env python3
"""
Validate that commanded motion results in the same motion in Gazebo.

For each case a fresh headless gz sim + controller is started, the robot is
brought to STANDING, a command is held for HOLD_S of *sim* time, then stopped.
Body pose is recorded from /world/hexspider_world/pose/info (sim timestamps).

Reported per case:
  distance   straight-line displacement (start of command -> end of hold)
  speed      slope of along-track position vs sim time (steady state, first
             SETTLE_S discarded)
  ratio      measured / commanded speed  (1.0 = no slip)
  lateral    cross-track drift, and heading change

Usage: python3 motion_validation.py [case ...]
  cases: v25 v50 v75 rotate arc   (default: v50)
"""
import math, os, pty, re, select, signal, subprocess, sys, threading, time

DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLER = os.environ.get("CONTROLLER_BIN", os.path.join(DIR, "Controller", "build", "Controller"))
HEXSPIDER = os.environ.get("HEXSPIDER_DIR", os.path.join(DIR, "HexSpider"))
ENV = {**os.environ, "GZ_IP": "127.0.0.1",
       "GZ_SIM_RESOURCE_PATH": os.path.join(HEXSPIDER, "models")}
POSE = "/world/hexspider_world/pose/info"
VEL = "/world/hexspider_world/model/hexspider/velocity"
HDG = "/world/hexspider_world/model/hexspider/heading"
HOLD_S = 20.0
SETTLE_S = 4.0

CASES = {  # name: (velocity mm/s, heading rad)
    "v25": (25.0, 0.0), "v50": (50.0, 0.0), "v75": (75.0, 0.0),
    "rotate": (0.0, 1.57), "arc": (50.0, 1.57),
}


def pub(topic, val):
    subprocess.run(["gz", "topic", "-t", topic, "-m", "gz.msgs.Double",
                    "-p", f"data: {val}"], env=ENV, check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class PoseRecorder:
    """Parses `gz topic -e` text output for the hexspider model pose."""
    def __init__(self):
        self.samples = []  # (sim_t, x, y, yaw)
        self.links = []    # (sim_t, {name: (px,py,pz,qx,qy,qz,qw)}) for model + femur links
        self.lock = threading.Lock()
        self.proc = subprocess.Popen(["gz", "topic", "-e", "-t", POSE], env=ENV,
                                     stdout=subprocess.PIPE, text=True,
                                     stderr=subprocess.DEVNULL)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        sec = nsec = 0
        in_model = False
        cur = {}
        section = None
        msg = {}
        for line in self.proc.stdout:
            s = line.strip()
            m = re.match(r"(sec|nsec): (\d+)", s)
            if m and not in_model:
                if m.group(1) == "sec": sec = int(m.group(2))
                else: nsec = int(m.group(2))
            if s == "pose {":
                in_model, cur, section = True, {}, None
            elif in_model and (m := re.match(r'name: "(\w+)"', s)):
                cur["name"] = m.group(1)
                if cur["name"] == "hexspider":
                    cur["is"] = True
            elif in_model and s in ("position {", "orientation {"):
                section = s.split()[0]
            elif in_model and (m := re.match(r"([xyzw]): (\S+)", s)) and section:
                cur[section[0] + m.group(1)] = float(m.group(2))
            elif s == "}" and in_model and section:
                section = None
            elif s == "}" and in_model:
                if cur.get("is"):
                    w, x, y, z = (cur.get("ow", 1.0), cur.get("ox", 0.0),
                                  cur.get("oy", 0.0), cur.get("oz", 0.0))
                    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                    with self.lock:
                        self.samples.append((sec + nsec * 1e-9,
                                             cur.get("px", 0.0), cur.get("py", 0.0), yaw))
                nm = cur.get("name", "")
                if nm == "hexspider" or re.fullmatch(r"leg_\w\w_femur", nm):
                    q = [cur.get("o" + c, 0.0) for c in "xyz"] + [cur.get("ow", 1.0 if not any(
                        k.startswith("o") for k in cur) else 0.0)]
                    msg[nm] = (cur.get("px", 0.0), cur.get("py", 0.0), cur.get("pz", 0.0), *q)
                    if len(msg) == 7:
                        with self.lock:
                            self.links.append((sec + nsec * 1e-9, msg))
                        msg = {}
                in_model = False

    def now(self):
        with self.lock:
            return self.samples[-1][0] if self.samples else 0.0

    def slice(self, t0, t1):
        with self.lock:
            return [s for s in self.samples if t0 <= s[0] <= t1]

    def stop(self):
        self.proc.terminate()


class Stack:
    def __init__(self):
        self.lines, self.lock = [], threading.Lock()
        self.gz = subprocess.Popen(["gz", "sim", "-s", "-r", "world/hexspider_world.sdf"],
                                   cwd=HEXSPIDER, env=ENV,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(5)
        master, slave = pty.openpty()
        self.ctrl = subprocess.Popen([CONTROLLER], stdin=slave, stdout=slave, stderr=slave,
                                     cwd=os.path.join(DIR, "Controller"), env=ENV)
        os.close(slave)
        self.master = master
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        buf = b""
        while True:
            try:
                r, _, _ = select.select([self.master], [], [], 1.0)
                if not r:
                    continue
                d = os.read(self.master, 4096)
                if not d:
                    return
                buf += d
                while b"\n" in buf:
                    l, buf = buf.split(b"\n", 1)
                    with self.lock:
                        self.lines.append(l.decode(errors="replace"))
            except OSError:
                return

    def wait_state(self, name, timeout, after=0):
        end = time.monotonic() + timeout
        seen = after
        while time.monotonic() < end:
            with self.lock:
                new = self.lines[seen:]
                seen += len(new)
            if any(f"motion state {name}" in l for l in new):
                return True
            time.sleep(0.2)
        return False

    def mark(self):
        with self.lock:
            return len(self.lines)

    def states(self):
        with self.lock:
            return [l.split("motion state ")[1] for l in self.lines if "motion state" in l]

    def stop(self):
        for p in (self.ctrl, self.gz):
            p.send_signal(signal.SIGINT)
        for p in (self.ctrl, self.gz):
            try: p.wait(timeout=5)
            except subprocess.TimeoutExpired: p.kill()
        subprocess.run(["pkill", "-f", "gz-sim-main"], check=False)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def run_case(name):
    if subprocess.run(["pgrep", "-f", "gz-sim-main"], capture_output=True).stdout.strip():
        sys.exit("A gz-sim-main is already running; kill it first (stale world would corrupt the test)")
    v, h = CASES[name]
    print(f"\n=== {name}: velocity {v} mm/s, heading {h} rad, hold {HOLD_S}s sim ===", flush=True)
    st = Stack()
    rec = PoseRecorder()
    try:
        if not st.wait_state("STANDING", 180):
            print("FAIL: never reached STANDING; states:", st.states()); return None
        time.sleep(3)  # let it settle
        loggers = []
        for j in os.environ.get("LOG_JOINTS", "").split(","):
            if j:
                loggers.append(subprocess.Popen(
                    ["python3", os.path.join(DIR, "log_joint.py"), j, f"/tmp/mv_{name}_{j}.csv"],
                    env=ENV, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        time.sleep(2)
        t_start = rec.now()
        pub(HDG, h); pub(VEL, v)
        wall0 = time.monotonic()
        while rec.now() < t_start + HOLD_S:
            if time.monotonic() - wall0 > 4 * HOLD_S:
                print("FAIL: sim running too slowly (hold exceeded 4x wall time)"); return None
            time.sleep(0.2)
        t_end = rec.now()
        pub(VEL, 0.0)
        for lg in loggers:
            lg.send_signal(signal.SIGINT)
        if h != 0.0 and v == 0.0:
            pass  # rotation completes on its own; heading stays
        st.wait_state("STANDING", 30, after=st.mark())
        data = rec.slice(t_start, t_end)
        if len(data) < 10:
            print("FAIL: no pose data"); return None
        t0, x0, y0, yaw0 = data[0]
        # along/cross-track relative to the *initial body heading*
        c, s = math.cos(yaw0), math.sin(yaw0)
        tr = [(t - t0, (x - x0) * c + (y - y0) * s, -(x - x0) * s + (y - y0) * c,
               wrap(yw - yaw0)) for t, x, y, yw in data]
        tN, a, b, dyaw = tr[-1]
        dist = math.hypot(tr[-1][1], tr[-1][2])
        steady = [r for r in tr if r[0] >= SETTLE_S]
        n = len(steady)
        mt = sum(r[0] for r in steady) / n
        speed_xy = None
        if n > 5:
            # per-axis least-squares slope, speed = hypot of slopes
            def slope(i):
                m = sum(r[i] for r in steady) / n
                return (sum((r[0] - mt) * (r[i] - m) for r in steady) /
                        sum((r[0] - mt) ** 2 for r in steady))
            speed_xy = math.hypot(slope(1), slope(2)) * 1000
        print(f"states: {' > '.join(st.states())}")
        print(f"hold duration  : {tN:.2f} s sim")
        print(f"displacement   : along {a*1000:.1f} mm, cross {b*1000:.1f} mm "
              f"(total {dist*1000:.1f} mm)")
        print(f"yaw            : start {math.degrees(yaw0):.1f} deg -> end "
              f"{math.degrees(wrap(yaw0 + dyaw)):.1f} deg (change {math.degrees(dyaw):.2f}, "
              f"commanded heading {math.degrees(h):.1f})")
        if v > 0 and speed_xy is not None:
            print(f"commanded dist : {v*tN:.1f} mm   ratio {dist*1000/(v*tN):.3f}")
            print(f"steady speed   : {speed_xy:.1f} mm/s vs {v:.1f} commanded "
                  f"(ratio {speed_xy/v:.3f})")
        return dict(case=name, a=a, b=b, dist=dist, dyaw=dyaw, speed=speed_xy, T=tN)
    finally:
        tag = os.environ.get("TAG")
        if tag:
            with open(f"/tmp/mv_pose_{tag}.csv", "w") as f:
                f.write("t,x,y,yaw\n")
                for r in rec.samples: f.write("%.4f,%.5f,%.5f,%.5f\n" % r)
            with open(f"/tmp/mv_links_{tag}.csv", "w") as f:
                names = sorted(rec.links[0][1]) if rec.links else []
                f.write("t," + ",".join(f"{n}:{c}" for n in names for c in "pxpypzqxqyqzqw".replace("pxpypz", "px py pz ").replace("qxqyqzqw", "qx qy qz qw").split()) + "\n")
                for t, d in rec.links:
                    f.write("%.4f," % t + ",".join("%.6f" % v for n in names for v in d.get(n, (0,) * 7)) + "\n")
        with open(f"/tmp/mv_{name}.log", "w") as f:
            f.write("\n".join(st.lines))
        rec.stop(); st.stop(); time.sleep(2)


if __name__ == "__main__":
    for c in (sys.argv[1:] or ["v50"]):
        run_case(c)
