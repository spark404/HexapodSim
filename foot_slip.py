#!/usr/bin/env python3
"""
Foot slip analysis for runs recorded by motion_validation.py (TAG=<tag> HEX_TRACE=...).

Needs /tmp/mv_links_<tag>.csv (model + femur link poses) and /tmp/mv_trace_<tag>.csv
(controller trace with the grounded mask per tick, from the traced controller build).

Foot tip = lowest point of the femur collision mesh when standing, taken in the link
frame and carried through link and model poses. pose/info link poses are model
relative, so the model pose is applied on top.

A stance foot that does not slip stays fixed in the world. Per gait half-cycle and per
stance leg the script reports how far the tip moved in the world while it should have
been planted, and compares that with how much of the controller's body motion the body
actually made.

Usage: python3 foot_slip.py <tag> [<tag> ...]
"""
import csv, math, struct, sys
import numpy as np

HERE = __file__.rsplit("/", 1)[0] if "/" in __file__ else "."
MESH = f"{HERE}/HexSpider/models/hexspider/meshes/Femur_20260405_1.stl"
COLL_POS = np.array([-0.24056, 0.0, -0.038])   # collision pose in the femur link frame
COLL_PITCH = -0.44
LEGS = ["fr", "cr", "br", "fl", "cl", "bl"]     # controller leg order
CONTACT_Z = 0.004                               # tip below this = touching ground (m)
SETTLE_S = 0.3                                  # ignore the touchdown transient


def rot_y(p):
    c, s = math.cos(p), math.sin(p)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def quat_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat_R_many(q):  # q: (N,4) x y z w
    x, y, z, w = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0] = np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], 1)
    R[:, 1] = np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], 1)
    R[:, 2] = np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], 1)
    return R


def mesh_in_link_frame():
    raw = open(MESH, "rb").read()
    n = struct.unpack("<I", raw[80:84])[0]
    a = np.frombuffer(raw[84:], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), count=n)
    v = a["v"].reshape(-1, 3).astype(float) * 0.001
    return v @ rot_y(COLL_PITCH).T + COLL_POS


def load(tag):
    rows = list(csv.reader(open(f"/tmp/mv_links_{tag}.csv")))
    ci = {h: i for i, h in enumerate(rows[0])}
    data = np.array(rows[1:], dtype=float)
    t = data[:, 0]

    def pose(name):
        p = data[:, [ci[f"{name}:p{c}"] for c in "xyz"]]
        q = data[:, [ci[f"{name}:q{c}"] for c in ("x", "y", "z", "w")]]
        return p, quat_R_many(q)

    trace = []
    for l in open(f"/tmp/mv_trace_{tag}.csv"):
        if l.startswith("C,"):
            trace.append([float(x) for x in l.strip().split(",")[1:]])
    return t, pose, np.array(trace)


def analyse(tag, tip=None, quiet=False):
    t, pose, C = load(tag)
    walk = C[C[:, 7] > 0]
    t_walk = walk[0, 0]
    pm, Rm = pose("hexspider")
    link = {leg: pose(f"leg_{leg}_femur") for leg in LEGS}
    Vl = mesh_in_link_frame()

    if tip is None:  # lowest point of the mesh, standing, just before the walk command
        k = int(np.argmin(abs(t - (t_walk - 0.3))))
        lows = []
        for leg in LEGS:
            p, R = link[leg]
            W = (Vl @ R[k].T + p[k]) @ Rm[k].T + pm[k]
            i = np.argmin(W[:, 2])
            lows.append(Vl[W[:, 2] < W[i, 2] + 0.0005].mean(0))
        tip = np.mean(lows, axis=0)

    foot = {}
    for i, leg in enumerate(LEGS):
        p, R = link[leg]
        loc = np.einsum("nij,j->ni", R, tip) + p
        foot[i] = np.einsum("nij,nj->ni", Rm, loc) + pm      # (N,3) world

    # travel direction and body progress
    sel = (t >= t_walk) & (t <= walk[-1, 0])
    d = pm[sel][-1, :2] - pm[sel][0, :2]
    u = d / np.linalg.norm(d)

    mask = walk[:, 6].astype(int)
    b = [0] + [i for i in range(1, len(walk)) if mask[i] != mask[i - 1]] + [len(walk) - 1]
    out = []
    contact_ok = []
    for k, (a, z) in enumerate(zip(b, b[1:])):
        ta, tz = walk[a, 0], walk[z, 0]
        ctrl = math.hypot(walk[z, 3] - walk[a, 3], walk[z, 4] - walk[a, 4])
        ia, iz = np.searchsorted(t, ta), np.searchsorted(t, tz)
        body = np.linalg.norm(pm[iz, :2] - pm[ia, :2])
        w0 = np.searchsorted(t, ta + SETTLE_S)
        slides = []
        for leg in range(6):
            if not (mask[a] >> leg) & 1:
                continue
            seg = foot[leg][w0:iz]
            if len(seg) < 3:
                continue
            on = (seg[:, 2] < CONTACT_Z).mean()
            contact_ok.append(on)
            disp = seg[-1, :2] - seg[0, :2]
            # tip path length in the plane while planted: total movement, not just net
            path = np.linalg.norm(np.diff(seg[:, :2], axis=0), axis=1).sum()
            # sliding while in ground contact on both samples (excludes hops)
            both = (seg[1:, 2] < CONTACT_Z) & (seg[:-1, 2] < CONTACT_Z)
            dd = np.diff(seg[:, :2], axis=0)[both]
            slides.append((float(disp @ u), float(np.linalg.norm(disp)), float(path),
                           float(np.linalg.norm(dd, axis=1).sum()), float(np.minimum(dd @ u, 0).sum())))
        if slides:
            out.append(dict(k=k, ratio=body * 1000 / ctrl if ctrl else float("nan"), body=body * 1000, ctrl=ctrl,
                            along=np.mean([s[0] for s in slides]) * 1000,
                            net=np.mean([s[1] for s in slides]) * 1000,
                            worst=max(s[1] for s in slides) * 1000,
                            path=np.mean([s[2] for s in slides]) * 1000,
                            csl=np.mean([s[3] for s in slides]) * 1000,
                            back=np.mean([s[4] for s in slides]) * 1000,
                            dur=tz - ta))
    if not quiet:
        print(f"{tag}: tip (link frame) {tip.round(4)}  stance-foot-on-ground fraction {np.mean(contact_ok):.2f}")
        print("  k  progress  body/ctrl(mm)  stance-foot slide: along-track  net  worst  path")
        for o in out:
            flag = " <-- low" if o["ratio"] < 0.8 and o["k"] > 0 else ""
            print(f"  {o['k']:2d}   {o['ratio']:.2f}    {o['body']:5.0f}/{o['ctrl']:.0f}       "
                  f"{o['along']:+6.1f}   {o['net']:5.1f}  {o['worst']:5.1f}  {o['path']:5.1f}{flag}")
    return out, tip


if __name__ == "__main__":
    tip = None
    allo = []
    for tag in sys.argv[1:]:
        o, tip = analyse(tag, tip)
        allo += [dict(x, tag=tag) for x in o if x["k"] > 0]
    if len(allo) > 5:
        ctrl = np.array([x["ctrl"] for x in allo]); body = np.array([x["body"] for x in allo])
        along = np.array([x["along"] for x in allo])
        deficit = ctrl - body
        slope, icpt = np.polyfit(-along, deficit, 1)
        print(f"\n{len(allo)} half-cycles (startup excluded)")
        print(f"body progress deficit (ctrl - body): mean {deficit.mean():.1f} mm of {ctrl.mean():.0f} mm")
        print(f"mean stance-foot along-track world displacement: {along.mean():+.1f} mm")
        print(f"deficit vs -foot displacement: slope {slope:.2f}, intercept {icpt:.1f} mm, "
              f"R^2 {np.corrcoef(-along, deficit)[0, 1] ** 2:.2f}")
        print(f"contact slide (sum |dxy| while planted) mean {np.mean([x['csl'] for x in allo]):.1f} mm, "
              f"backward part mean {np.mean([x['back'] for x in allo]):.1f} mm per stance leg per half-cycle")
        r = np.array([x["ratio"] for x in allo])
        for key in ("along", "net", "worst", "path"):
            v = np.array([x[key] for x in allo])
            print(f"corr(progress, {key:6s}) = {np.corrcoef(r, v)[0, 1]:+.2f}")
