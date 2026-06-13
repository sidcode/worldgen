"""Evaluation harness: run recorded simulations and score them (stdlib only).

Inputs per world (produced by ``docker/run.sh record <world> <secs>`` into
``docker/output/``):

  * ``<name>.csv``       cross-track error as a ``t,cte`` CSV (the recorder
                         timestamps it off the same clock as the trajectory;
                         legacy ``data: <float>`` / bare-float captures are
                         still accepted, just without per-sample times).
  * ``<name>_traj.csv``  robot trajectory, ``t,x,y`` CSV (odom frame; the
                         scorer transforms it into the course world frame
                         using the sidecar spawn pose).

Ground truth comes from the ``<name>.course.json`` sidecar written next to
the generated world (see course.py).

Scoring:
  * cte metrics: mean |cte|, max |cte|, RMS cte, % of samples within +-0.2 m
    (cte samples are treated as uniformly spaced in time; the follower
    publishes at a fixed rate).
  * completion: every odometry point is projected onto the course centerline
    polyline; the furthest (monotonically increasing, wrap-unwrapped for
    closed loops) arc length reached, relative to where the robot started,
    divided by the remaining path length.  Closed loops can exceed 100%
    (multiple laps).
  * stall: the trajectory is downsampled to ~2 Hz and scanned for any
    >= 8 s window in which the robot never moves more than 0.15 m while not
    within 1.0 m (arc length) of the course end -> "stalled/crashed".

Verdicts: PASS if completion > 90% and mean|cte| < 0.3 m and no stall;
FAIL if stalled, completion < 60%, or mean|cte| > 0.5 m; WARN otherwise.
"""

import json
import math
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from . import course as course_mod

CTE_BAND = 0.2          # "within band" threshold (m)
STALL_WINDOW = 8.0      # seconds without progress => stalled
STALL_DISP = 0.15       # max displacement (m) inside the window
END_TOL = 1.0           # arc-length tolerance for "reached the end" (m)
END_MARGIN_S = 2.0      # grace period after forward progress peaks before
                        # cte is treated as post-course (robot off the wall)

PASS_COMPLETION = 90.0
PASS_MEAN_CTE = 0.3
FAIL_COMPLETION = 60.0
FAIL_MEAN_CTE = 0.5


# ------------------------------------------------------------------ parsing

def parse_cte_file(path):
    """(t, cte) samples from a cte capture.

    Accepts the recorder's ``t,cte`` CSV (timestamped), and -- for backward
    compatibility -- ``data: <float>`` lines and bare floats, for which the
    timestamp is None.
    """
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith(("---", "#")):
                continue
            t = None
            if line.startswith("data:"):
                line = line.split(":", 1)[1]
            elif "," in line:
                parts = line.split(",")
                try:
                    t = float(parts[0])
                except ValueError:
                    t = None          # header row, e.g. "t,cte"
                line = parts[-1]
            try:
                rows.append((t, float(line)))
            except ValueError:
                continue  # header or junk line
    return rows


def parse_traj_file(path):
    """(t, x, y) tuples from a trajectory CSV (header optional)."""
    rows = []
    with open(path) as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) < 3:
                continue
            try:
                rows.append((float(parts[0]), float(parts[1]),
                             float(parts[2])))
            except ValueError:
                continue
    return rows


# ------------------------------------------------------------------ scoring

def to_world(traj, spawn):
    """Transform odometry (t, x, y) into the world frame of the course path.

    ``/husky_velocity_controller/odom`` is reported in the odom frame, whose
    origin is the robot's spawn pose; the course centerline lives in Gazebo
    world coordinates.  The sidecar records ``spawn = [x, y, yaw]`` (the Husky
    always spawns facing +x, so yaw is 0), so world = R(yaw)*odom + (x, y).
    Without this the projection is offset by the spawn translation (~3.6 m)
    and both completion and the off-course trim come out wrong.
    """
    sx, sy = spawn[0], spawn[1]
    yaw = spawn[2] if len(spawn) > 2 else 0.0
    c, s = math.cos(yaw), math.sin(yaw)
    return [(t, sx + x * c - y * s, sy + x * s + y * c) for (t, x, y) in traj]


def _arclengths(path_pts):
    acc = [0.0]
    for (ax, ay), (bx, by) in zip(path_pts, path_pts[1:]):
        acc.append(acc[-1] + math.hypot(bx - ax, by - ay))
    return acc


def _project(p, path_pts, acc):
    """(arc length, perpendicular distance) of the closest path point to p."""
    best_d, best_s = float("inf"), 0.0
    px, py = p
    for i in range(len(path_pts) - 1):
        ax, ay = path_pts[i]
        bx, by = path_pts[i + 1]
        vx, vy = bx - ax, by - ay
        denom = vx * vx + vy * vy
        t = 0.0 if denom == 0 else max(
            0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / denom))
        cx, cy = ax + t * vx, ay + t * vy
        d = math.hypot(px - cx, py - cy)
        if d < best_d:
            best_d, best_s = d, acc[i] + t * math.sqrt(denom)
    return best_s, best_d


def project_progress(traj, path_pts, closed=False):
    """Per-trajectory-point (unwrapped arc-length progress, perp distance), L.

    For closed loops the raw projection wraps from L back to 0 once per lap;
    the smallest signed wrap-around step is accumulated instead so progress
    keeps growing across laps.
    """
    acc = _arclengths(path_pts)
    L = acc[-1]
    out, dists, prev_raw, unwrapped = [], [], None, 0.0
    for (_t, x, y) in traj:
        s, d = _project((x, y), path_pts, acc)
        if prev_raw is None:
            unwrapped = s
        elif closed and L > 0:
            step = (s - prev_raw + L / 2.0) % L - L / 2.0
            unwrapped += step
        else:
            unwrapped = s
        prev_raw = s
        out.append(unwrapped)
        dists.append(d)
    return out, dists, L


def detect_stall(traj, progress, path_len, closed=False,
                 window=STALL_WINDOW, max_disp=STALL_DISP, end_tol=END_TOL):
    """True if the robot stopped advancing mid-course (see module doc)."""
    if len(traj) < 2:
        return False, None
    # downsample to ~2 Hz so the windowed scan stays cheap
    ds, last_t = [], -1e9
    for row, prog in zip(traj, progress):
        if row[0] - last_t >= 0.5:
            ds.append((row[0], row[1], row[2], prog))
            last_t = row[0]
    for i in range(len(ds)):
        moved = False
        seen_window = False
        for j in range(i + 1, len(ds)):
            if math.hypot(ds[j][1] - ds[i][1], ds[j][2] - ds[i][2]) > max_disp:
                moved = True
                break
            if ds[j][0] - ds[i][0] >= window:
                seen_window = True
        if not moved and seen_window:
            at_end = (not closed) and (ds[i][3] >= path_len - end_tol)
            if not at_end:
                return True, ds[i][0]
    return False, None


def score_world(name, course, output_dir):
    """Score one recorded run; returns a metrics dict (see module doc)."""
    output_dir = Path(output_dir)
    m = {"name": name, "error": None}

    cte_path = output_dir / f"{name}.csv"
    traj_path = output_dir / f"{name}_traj.csv"

    cte = parse_cte_file(cte_path) if cte_path.is_file() else []
    traj = parse_traj_file(traj_path) if traj_path.is_file() else []
    if not cte and not traj:
        m["error"] = f"no recorded data ({cte_path.name}, {traj_path.name})"
        m["verdict"] = "FAIL"
        return m

    # ---- trajectory / completion first (it bounds the cte scoring window) ----
    path_pts = [tuple(p) for p in course["path"]]
    closed = bool(course.get("closed"))
    if traj:
        traj = to_world(traj, course.get("spawn", [0.0, 0.0, 0.0]))
    cte_cutoff = None       # absolute time after which cte is post-course
    cte_frac = 1.0          # fallback when cte has no timestamps
    if traj and len(path_pts) >= 2:
        progress, _dists, L = project_progress(traj, path_pts, closed)
        s0 = progress[0]
        peak = max(progress)
        m["dist_along_path"] = max(0.0, peak - s0)
        denom = L if closed else max(L - s0, 1e-9)
        m["path_length"] = L
        m["completion_pct"] = 100.0 * m["dist_along_path"] / denom
        T = traj[-1][0] - traj[0][0]
        m["duration_s"] = T
        stalled, t_stall = detect_stall(traj, progress, L, closed)
        m["stalled"] = stalled
        m["stall_t"] = t_stall
        # On a FINITE (open) course the robot drives off the wall's end and
        # spins in open space once forward progress peaks; that
        # post-completion cte is not a tracking error.  cte and the
        # trajectory share a clock (the recorder timestamps both), so cut the
        # cte stream at the peak-progress time (+ a small grace margin).
        if not closed:
            t_peak = traj[progress.index(peak)][0]
            cte_cutoff = t_peak + END_MARGIN_S
            cte_frac = min(1.0, (t_peak - traj[0][0] + END_MARGIN_S)
                           / T) if T > 0 else 1.0
    else:
        m["error"] = (m.get("error") or
                      f"no trajectory data ({traj_path.name})")

    # ---- cte metrics over the on-course window ----
    if cte:
        if cte_cutoff is not None and any(t is not None for t, _ in cte):
            scored = [v for t, v in cte if t is None or t <= cte_cutoff]
        else:  # no timestamps: fall back to a fraction of the stream
            keep = max(1, int(round(cte_frac * len(cte))))
            scored = [v for _, v in cte[:keep]]
        scored = scored or [v for _, v in cte]
        m["n_cte"] = len(scored)
        m["n_cte_total"] = len(cte)
        m["mean_abs_cte"] = sum(abs(v) for v in scored) / len(scored)
        m["max_abs_cte"] = max(abs(v) for v in scored)
        m["rms_cte"] = math.sqrt(sum(v * v for v in scored) / len(scored))
        m["pct_in_band"] = 100.0 * sum(
            1 for v in scored if abs(v) <= CTE_BAND) / len(scored)
    else:
        m["error"] = m.get("error") or f"no cte data ({cte_path.name})"

    m["verdict"] = _verdict(m)
    return m


def _verdict(m):
    if m.get("error"):
        return "FAIL"
    if (m.get("stalled") or m["completion_pct"] < FAIL_COMPLETION
            or m["mean_abs_cte"] > FAIL_MEAN_CTE):
        return "FAIL"
    if (m["completion_pct"] > PASS_COMPLETION
            and m["mean_abs_cte"] < PASS_MEAN_CTE and not m.get("stalled")):
        return "PASS"
    return "WARN"


# ------------------------------------------------------------------ harness

def find_docker_dir(start=None):
    """Walk up from this package until a directory holding docker/run.sh."""
    here = Path(start or __file__).resolve()
    for parent in [here] + list(here.parents):
        cand = parent / "docker" / "run.sh"
        if cand.is_file():
            return cand.parent
    raise FileNotFoundError(
        "could not locate docker/run.sh above " + str(here)
        + "; pass --docker-dir")


def run_record(docker_dir, world_name, secs):
    """Invoke docker/run.sh record <world> <secs> (blocking)."""
    cmd = ["bash", "run.sh", "record", world_name, str(int(secs))]
    print(f"### eval: {' '.join(cmd)}  (cwd={docker_dir})")
    res = subprocess.run(cmd, cwd=str(docker_dir))
    if res.returncode != 0:
        print(f"WARNING: record exited with {res.returncode} for "
              f"{world_name}; scoring whatever was captured",
              file=sys.stderr)


# ------------------------------------------------------------------ report

_COLS = [
    ("world", "name", "{}"),
    ("dur(s)", "duration_s", "{:.0f}"),
    ("mean|cte|", "mean_abs_cte", "{:.3f}"),
    ("max|cte|", "max_abs_cte", "{:.3f}"),
    ("rms", "rms_cte", "{:.3f}"),
    ("in±0.2m", "pct_in_band", "{:.0f}%"),
    ("compl", "completion_pct", "{:.0f}%"),
    ("dist(m)", "dist_along_path", "{:.1f}"),
    ("stall", "stalled", "{}"),
    ("verdict", "verdict", "{}"),
]


def _cell(m, key, fmt):
    v = m.get(key)
    if v is None:
        return "-"
    if key == "stalled":
        return f"yes@{m['stall_t']:.0f}s" if v else "no"
    return fmt.format(v)


def format_table(results, md=False):
    rows = [[h for h, _, _ in _COLS]]
    for m in results:
        rows.append([_cell(m, k, f) for _, k, f in _COLS])
    if md:
        lines = ["| " + " | ".join(rows[0]) + " |",
                 "|" + "|".join("---" for _ in _COLS) + "|"]
        lines += ["| " + " | ".join(r) + " |" for r in rows[1:]]
        return "\n".join(lines)
    widths = [max(len(r[i]) for r in rows) for i in range(len(_COLS))]
    return "\n".join("  ".join(c.ljust(w) for c, w in zip(r, widths))
                     for r in rows)


def write_report(results, report_path):
    lines = [
        "# Wall-follower evaluation report",
        "",
        f"Generated {datetime.now().isoformat(timespec='seconds')} by "
        "`worldgen eval`.",
        "",
        format_table(results, md=True),
        "",
        "## Verdicts",
        "",
        f"* **PASS**: completion > {PASS_COMPLETION:.0f}%, "
        f"mean |cte| < {PASS_MEAN_CTE} m, no stall.",
        f"* **FAIL**: stalled/crashed, completion < {FAIL_COMPLETION:.0f}%, "
        f"or mean |cte| > {FAIL_MEAN_CTE} m.",
        "* **WARN**: everything in between.",
        "",
    ]
    for m in results:
        note = m.get("error") or (
            f"stalled at t={m['stall_t']:.0f}s" if m.get("stalled") else "ok")
        lines.append(f"* `{m['name']}`: **{m['verdict']}** — {note}")
    lines.append("")
    Path(report_path).write_text("\n".join(lines))


# ------------------------------------------------------------------ driver

def evaluate(worlds, worlds_dir, output_dir, docker_dir=None,
             score_only=False, default_secs=90):
    """Run (unless score_only) + score a list of (name, secs|None) worlds.

    Course sidecars are loaded from ``worlds_dir``.  Returns metrics list.
    """
    worlds_dir = Path(worlds_dir)
    results = []
    for name, secs in worlds:
        secs = secs or default_secs
        sidecar = worlds_dir / f"{name}.course.json"
        if not sidecar.is_file():
            results.append({"name": name, "verdict": "FAIL",
                            "error": f"missing sidecar {sidecar}"})
            continue
        crs = course_mod.load_course(sidecar)
        if not score_only:
            run_record(docker_dir or find_docker_dir(), name, secs)
        results.append(score_world(name, crs, output_dir))
    return results
