"""Clutter: small static obstacles hugging the followed (left) wall.

Obstacles are boxes (~0.4-0.8 m cubes) and cylinders placed against the
inner face of the left wall so they protrude 0.3-0.6 m into the corridor.
They perturb the lidar's view of the wall without ever blocking the course:

  * placement is parameterised by arc length along the course path, with
    per-seed jitter (deterministic for a given random.Random);
  * nothing is placed within ``min_spawn_clear`` (1.2 m) of the spawn;
  * the protrusion is clamped so the obstacle's innermost point keeps at
    least ``min_path_clear`` (1.6 m) of clearance from the path centerline
    (at the default offset 2.0 that caps the effective protrusion at 0.3 m
    -- the clearance constraint wins over the protrusion range).
"""

import math
from dataclasses import dataclass

from .geometry import SPAWN, WALL_THICKNESS, mirror_point


@dataclass
class Obstacle:
    kind: str          # "box" or "cylinder"
    x: float
    y: float
    yaw: float
    sx: float          # box: size; cylinder: 2*radius
    sy: float
    sz: float          # height (cylinder length)

    @property
    def half_extent(self):
        return 0.5 * max(self.sx, self.sy)


def mirror_obstacles(obstacles, about_y=SPAWN[1]):
    """Reflect obstacles about the y = about_y line through the spawn."""
    out = []
    for o in obstacles:
        _, my = mirror_point((o.x, o.y), about_y)
        out.append(Obstacle(o.kind, o.x, my, -o.yaw, o.sx, o.sy, o.sz))
    return out


def _path_lengths(pts):
    acc, total = [0.0], 0.0
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        total += math.hypot(bx - ax, by - ay)
        acc.append(total)
    return acc, total


def _point_at(pts, acc, s):
    """Point, direction and left normal at arc length s along pts."""
    s = max(0.0, min(s, acc[-1]))
    for i in range(len(acc) - 1):
        if s <= acc[i + 1] or i == len(acc) - 2:
            seg = acc[i + 1] - acc[i]
            t = 0.0 if seg <= 0 else (s - acc[i]) / seg
            (ax, ay), (bx, by) = pts[i], pts[i + 1]
            dx, dy = bx - ax, by - ay
            norm = math.hypot(dx, dy) or 1.0
            dx, dy = dx / norm, dy / norm
            return ((ax + t * (bx - ax), ay + t * (by - ay)),
                    (dx, dy), (-dy, dx))
    raise AssertionError("unreachable")


def place_clutter(path_pts, offset, count, rng,
                  min_spawn_clear=1.2, min_path_clear=1.6,
                  start_margin=2.0, end_margin=1.5):
    """Place ``count`` obstacles against the left wall along ``path_pts``.

    Returns (obstacles, warnings).  Deterministic for a given ``rng`` state.
    Positions are spread roughly evenly along the course with jitter; a slot
    is dropped (with a warning) if it cannot satisfy the spawn clearance.
    """
    warnings = []
    wall_face = offset - WALL_THICKNESS / 2.0   # inner face of the left wall
    max_pro = wall_face - min_path_clear
    if max_pro < 0.1:
        return [], [f"clutter skipped: offset {offset} leaves no room for "
                    f"obstacles ({min_path_clear} m centerline clearance "
                    f"required)"]

    acc, total = _path_lengths(path_pts)
    lo, hi = start_margin, total - end_margin
    if hi <= lo or count <= 0:
        return [], (["clutter skipped: course too short"] if count else [])

    obstacles = []
    slot = (hi - lo) / count
    for i in range(count):
        placed = False
        for _attempt in range(8):
            s = lo + slot * (i + rng.uniform(0.15, 0.85))
            p, d, n = _point_at(path_pts, acc, s)
            pro = min(rng.uniform(0.3, 0.6), max_pro)
            if rng.random() < 0.5:
                side = rng.uniform(0.4, 0.8)
                center_dist = (wall_face - pro) + side / 2.0
                obst = Obstacle("box",
                                p[0] + center_dist * n[0],
                                p[1] + center_dist * n[1],
                                math.atan2(d[1], d[0]),
                                side, side, side)
            else:
                radius = rng.uniform(0.2, 0.4)
                center_dist = (wall_face - pro) + radius
                obst = Obstacle("cylinder",
                                p[0] + center_dist * n[0],
                                p[1] + center_dist * n[1],
                                0.0, 2 * radius, 2 * radius, 0.8)
            spawn_d = (math.hypot(obst.x - SPAWN[0], obst.y - SPAWN[1])
                       - obst.half_extent)
            if spawn_d >= min_spawn_clear:
                obstacles.append(obst)
                placed = True
                break
        if not placed:
            warnings.append(
                f"clutter slot {i} dropped: could not keep "
                f"{min_spawn_clear} m clearance from the spawn")
    return obstacles, warnings
