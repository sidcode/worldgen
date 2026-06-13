"""ASCII top-down preview of a wall world, plus a .world file parser so the
stock (or any hand-made) worlds can be previewed too."""

import math
import xml.etree.ElementTree as ET

from .geometry import SPAWN, Wall


def parse_world(path):
    """Extract wall boxes from an SDF .world file.

    Model poses are overridden by the <state> block when present (the stock
    worlds were saved from a live Gazebo session, where <state> is the truth).
    """
    root = ET.parse(path).getroot()
    world = root.find("world")

    state_poses = {}
    state = world.find("state")
    if state is not None:
        for m in state.findall("model"):
            pose = m.find("pose")
            if pose is not None:
                state_poses[m.get("name")] = [float(v)
                                              for v in pose.text.split()]

    walls = []
    for model in world.findall("model"):
        name = model.get("name")
        if name == "ground_plane":
            continue
        box = model.find("./link/collision/geometry/box/size")
        if box is None:
            continue
        sx, sy, _ = (float(v) for v in box.text.split())
        pose_el = model.find("pose")
        pose = ([float(v) for v in pose_el.text.split()]
                if pose_el is not None else [0.0] * 6)
        pose = state_poses.get(name, pose)
        walls.append(Wall(cx=pose[0], cy=pose[1], yaw=pose[5],
                          length=max(sx, sy)))
    return walls


def render(walls, cols=72, spawn=SPAWN, obstacles=()):
    """Top-down ASCII map: '#' walls, 'o' clutter obstacles, 'R' robot spawn
    (faces +x / east)."""
    pts = [p for w in walls for p in w.endpoints()] + [spawn]
    pts += [(o.x, o.y) for o in obstacles]
    xmin = min(p[0] for p in pts) - 1.0
    xmax = max(p[0] for p in pts) + 1.0
    ymin = min(p[1] for p in pts) - 1.0
    ymax = max(p[1] for p in pts) + 1.0

    sx = (cols - 1) / (xmax - xmin)
    sy = sx * 0.5                      # terminal cells are ~2x taller than wide
    rows = max(int((ymax - ymin) * sy) + 1, 3)

    grid = [[" "] * cols for _ in range(rows)]

    def put(x, y, ch):
        c = int((x - xmin) * sx)
        r = rows - 1 - int((y - ymin) * sy)
        if 0 <= r < rows and 0 <= c < cols:
            grid[r][c] = ch

    for w in walls:
        (ax, ay), (bx, by) = w.endpoints()
        steps = max(int(w.length * sx) * 3, 2)
        for i in range(steps + 1):
            t = i / steps
            put(ax + t * (bx - ax), ay + t * (by - ay), "#")
    for o in obstacles:
        put(o.x, o.y, "o")
    put(*spawn, "R")

    metres_per_col = 1.0 / sx
    lines = ["".join(row) for row in grid]
    lines.append(f"R = robot spawn {spawn} facing east (+x); wall on its "
                 f"LEFT (up)   scale: 1 col = {metres_per_col:.2f} m")
    return "\n".join(lines)
