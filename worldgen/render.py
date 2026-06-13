"""Top-down run renderer: lidar-mapped walls + trajectory + cte panel -> video.

This is the reusable recording base used by ``worldgen video`` and
``worldgen eval --video``.  It overlays, for one recorded run:

  * the walls as the robot actually sensed them (every lidar return over the
    run, faint), so the map is built from the data itself,
  * the live 2-D lidar scan at each frame (bright), so you can watch the beams
    sweep the wall,
  * the robot's path and current pose (with a heading arrow),
  * a stats box (time, cte, running mean |cte|, speed),
  * a cross-track-error-vs-time panel with the +-0.2 m band.

Everything is drawn in the odometry frame.  The robot pose and its lidar both
live in that frame, so they stay consistent for the whole run.  (Wheel odom
drifts a few degrees relative to the Gazebo world and there is no world-pose
topic in this sim, so painting the walls from the lidar is more honest than
trying to register odom against the static .world geometry.)

The LMS1xx here sweeps clockwise (a wall on the robot's physical LEFT reads at
NEGATIVE scan angles), so a beam at scan angle ``a`` points along ``yaw - a``,
the same ``side_sign = -1`` convention the controller uses.

Rendering is an OPTIONAL feature: it needs matplotlib (the ``video`` extra) and
ffmpeg on PATH for mp4 (it falls back to an animated GIF otherwise).  Run it
with ``uv run --extra video worldgen ...``.
"""

import math
import shutil
from pathlib import Path

from . import evaluate

MAX_BEAM = 15.0           # ignore lidar returns beyond this (m)
BEAM_STRIDE = 2           # draw every Nth beam of the live scan
MAP_SCAN_STRIDE = 3       # use every Nth scan for the accumulated wall map
BAND = 0.2                # +-0.2 m target band drawn on the cte panel


def _import_mpl():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.animation import (FFMpegWriter, FuncAnimation,
                                          PillowWriter)
        return plt, FuncAnimation, FFMpegWriter, PillowWriter
    except ImportError as exc:
        raise RuntimeError(
            "video rendering needs matplotlib. Run it with the 'video' extra:\n"
            "    uv run --extra video worldgen video ...\n"
            "    uv run --extra video worldgen eval --video") from exc


# ------------------------------------------------------------------ parsing

def load_traj(path):
    """(t, x, y, yaw) in the odom frame from a t,x,y[,yaw] CSV."""
    rows = []
    for line in open(path):
        p = line.strip().split(",")
        if len(p) < 3:
            continue
        try:
            t, x, y = float(p[0]), float(p[1]), float(p[2])
        except ValueError:
            continue
        yaw = None
        if len(p) > 3:
            try:
                yaw = float(p[3])
            except ValueError:
                yaw = None
        rows.append((t, x, y, yaw))
    return rows


def load_scan(path):
    """(t, angle_min, angle_increment, [ranges]) per recorded scan."""
    scans = []
    for line in open(path):
        p = line.strip().split(",")
        if len(p) < 4:
            continue
        try:
            t, amin, ainc = float(p[0]), float(p[1]), float(p[2])
            ranges = [float(v) for v in p[3:]]
        except ValueError:
            continue
        scans.append((t, amin, ainc, ranges))
    return scans


# ------------------------------------------------------------------ helpers

def _headings(traj):
    """Fill any missing yaw from the direction of travel."""
    out = []
    for i, (_t, _x, _y, yaw) in enumerate(traj):
        if yaw is None:
            j = min(i + 1, len(traj) - 1)
            k = max(i - 1, 0)
            dx, dy = traj[j][1] - traj[k][1], traj[j][2] - traj[k][2]
            yaw = math.atan2(dy, dx) if (dx or dy) else 0.0
        out.append(yaw)
    return out


def _beam_points(scan, pose, stride=BEAM_STRIDE):
    """odom-frame (xs, ys) of the lidar returns for one scan at `pose`."""
    _t, amin, ainc, ranges = scan
    px, py, pyaw = pose
    xs, ys = [], []
    for i in range(0, len(ranges), stride):
        r = ranges[i]
        if not math.isfinite(r) or r <= 0.05 or r > MAX_BEAM:
            continue
        ang = pyaw - (amin + i * ainc)      # clockwise scan -> physical = -a
        xs.append(px + r * math.cos(ang))
        ys.append(py + r * math.sin(ang))
    return xs, ys


def _nearest(times, t):
    """Index of the timestamp in sorted `times` closest to t."""
    lo, hi = 0, len(times) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if times[mid] < t:
            lo = mid + 1
        else:
            hi = mid
    if lo > 0 and abs(times[lo - 1] - t) <= abs(times[lo] - t):
        return lo - 1
    return lo


def _pose_at(traj, yaws, t):
    """(x, y, yaw) of the robot at time t (nearest sample)."""
    k = _nearest([p[0] for p in traj], t)
    return traj[k][1], traj[k][2], yaws[k]


# ------------------------------------------------------------------ render

def render_run(traj_csv, out_path, cte_csv=None, scan_csv=None,
               world_path=None, title="", fps=20, seconds=30.0, **_ignored):
    """Render one recorded run to out_path (.mp4, or .gif without ffmpeg).

    ``world_path`` is accepted for API compatibility but not required; the wall
    map is built from the lidar.  Extra keyword args (e.g. ``spawn``) are
    ignored.
    """
    plt, FuncAnimation, FFMpegWriter, PillowWriter = _import_mpl()

    traj = load_traj(traj_csv)
    if not traj:
        raise RuntimeError(f"no trajectory points in {traj_csv}")
    yaws = _headings(traj)
    cte = (evaluate.parse_cte_file(cte_csv)
           if cte_csv and Path(cte_csv).is_file() else [])
    scans = load_scan(scan_csv) if scan_csv and Path(scan_csv).is_file() else []
    scan_t = [s[0] for s in scans]

    try:
        plt.style.use("seaborn-v0_8-whitegrid")
    except OSError:
        plt.style.use("ggplot")

    t0 = traj[0][0]
    txy = [(p[1], p[2]) for p in traj]

    # accumulated lidar = the sensed wall map (static background)
    map_x, map_y = [], []
    for si in range(0, len(scans), MAP_SCAN_STRIDE):
        bx, by = _beam_points(scans[si],
                              _pose_at(traj, yaws, scans[si][0]), stride=2)
        map_x.extend(bx)
        map_y.extend(by)

    pts_x = [x for x, _ in txy] + map_x
    pts_y = [y for _, y in txy] + map_y
    pad = 1.5
    xmin, xmax = min(pts_x) - pad, max(pts_x) + pad
    ymin, ymax = min(pts_y) - pad, max(pts_y) + pad
    span_x, span_y = xmax - xmin, ymax - ymin

    fig = plt.figure(figsize=(8.0, 8.0 * span_y / span_x + 2.4))
    gs = fig.add_gridspec(2, 1, height_ratios=[span_y / span_x * 8.0, 2.0],
                          hspace=0.34)
    axm = fig.add_subplot(gs[0])
    axc = fig.add_subplot(gs[1])
    axm.set_aspect("equal")
    axm.set_xlim(xmin, xmax)
    axm.set_ylim(ymin, ymax)
    axm.set_xlabel("x [m] (odom)")
    axm.set_ylabel("y [m] (odom)")
    if title:
        fig.suptitle(title, fontsize=13, fontweight="bold")

    if map_x:
        axm.scatter(map_x, map_y, s=2, color="0.6", alpha=0.25,
                    label="sensed walls", zorder=1)
    axm.plot(txy[0][0], txy[0][1], "o", color="tab:green", ms=10,
             label="start", zorder=5)
    beam = axm.scatter([], [], s=7, color="tab:orange", alpha=0.7,
                       label="live lidar", zorder=3)
    (trail,) = axm.plot([], [], "-", color="tab:blue", lw=1.6, alpha=0.85,
                        label="path", zorder=4)
    (robot,) = axm.plot([], [], "o", color="tab:red", ms=11, zorder=6)
    (heading,) = axm.plot([], [], "-", color="tab:red", lw=2.2, zorder=6)
    axm.legend(loc="lower right", fontsize=9, framealpha=0.9)
    stats = axm.text(0.015, 0.985, "", transform=axm.transAxes, va="top",
                     ha="left", fontsize=10, family="monospace", zorder=7,
                     bbox=dict(boxstyle="round", fc="white", ec="0.7",
                               alpha=0.9))

    # cte panel
    if cte:
        ct = [(t if t is not None else i / len(cte) * (traj[-1][0] - t0))
              for i, (t, _v) in enumerate(cte)]
        cv = [v for _t, v in cte]
        abscte = [abs(v) for v in cv]
        axc.axhspan(-BAND, BAND, color="tab:green", alpha=0.15)
        axc.axhline(0.0, color="0.6", lw=0.8)
        axc.plot(ct, cv, color="tab:blue", lw=1.0, alpha=0.9)
        axc.set_xlim(0, max(ct) if ct else 1)
        ylim = max(0.5, min(2.0, max(abscte) * 1.1))
        axc.set_ylim(-ylim, ylim)
        (cmark,) = axc.plot([], [], "o", color="tab:red", ms=7)
        cvline = axc.axvline(0.0, color="tab:red", lw=1.0, alpha=0.7)
    else:
        ct = cv = abscte = []
        cmark = cvline = None
        axc.text(0.5, 0.5, "no cte recorded", ha="center", va="center",
                 transform=axc.transAxes, color="0.5")
    axc.set_xlabel("t [s]")
    axc.set_ylabel("cte [m]")

    n_frames = max(2, int(fps * seconds))
    step = max(1, len(traj) // n_frames)
    idx = list(range(0, len(traj), step))
    hl = 0.05 * min(span_x, span_y)

    def update(fi):
        k = idx[fi]
        t = traj[k][0]
        rx, ry, ryaw = txy[k][0], txy[k][1], yaws[k]
        trail.set_data([p[0] for p in txy[:k + 1]],
                       [p[1] for p in txy[:k + 1]])
        robot.set_data([rx], [ry])
        heading.set_data([rx, rx + hl * math.cos(ryaw)],
                         [ry, ry + hl * math.sin(ryaw)])
        artists = [trail, robot, heading, stats]
        if scans:
            bx, by = _beam_points(scans[_nearest(scan_t, t)], (rx, ry, ryaw))
            beam.set_offsets(list(zip(bx, by)) if bx else [(rx, ry)])
            artists.append(beam)
        j, kk = min(k + 1, len(traj) - 1), max(k - 1, 0)
        dt = traj[j][0] - traj[kk][0]
        spd = (math.hypot(txy[j][0] - txy[kk][0], txy[j][1] - txy[kk][1]) / dt
               if dt > 0 else 0.0)
        if abscte:
            ci = min(len(abscte) - 1, int(k / len(traj) * len(abscte)))
            line = ("t    = %5.1f s\ncte  = %+5.2f m\nmean = %4.2f m\n"
                    "v    = %4.2f m/s" %
                    (t - t0, cv[ci], sum(abscte[:ci + 1]) / (ci + 1), spd))
            cmark.set_data([ct[ci]], [cv[ci]])
            cvline.set_xdata([ct[ci], ct[ci]])
            artists += [cmark, cvline]
        else:
            line = "t = %5.1f s\nv = %4.2f m/s" % (t - t0, spd)
        stats.set_text(line)
        return artists

    anim = FuncAnimation(fig, update, frames=len(idx), blit=False,
                         interval=1000.0 / fps)
    out = Path(out_path)
    if out.suffix.lower() == ".gif" or not shutil.which("ffmpeg"):
        out = out.with_suffix(".gif")
        anim.save(str(out), writer=PillowWriter(fps=fps))
    else:
        anim.save(str(out), writer=FFMpegWriter(fps=fps, bitrate=2400))
    plt.close(fig)
    return str(out)
