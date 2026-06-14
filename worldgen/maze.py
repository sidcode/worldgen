"""Grid-maze generation for wall-following.

A perfect maze (recursive-backtracker spanning tree) rendered as static wall
boxes, plus the LEFT-HAND-RULE route through it as a course centerline so the
scorer can measure how far the follower gets.  Left-wall following *is* the
left-hand maze-solving rule, so a left-wall follower that rounds corners and
wall ends correctly will tour the whole maze.

Layout / spawn alignment
------------------------
The Husky always spawns at ``geometry.SPAWN = (-3, 2)`` facing +x (the launch
file is fixed), so the maze is placed with cell ``(0, 0)`` centred at
``(0, 2)`` and its WEST side open: the robot drives east out of the spawn,
straight into the entrance corridor, with the cell's north wall already on its
left.  Cells extend +x (columns) and +y (rows).

Coordinates
-----------
Cell ``(c, r)`` is centred at ``(c*pitch, 2 + r*pitch)`` and spans
``pitch`` metres in x and y.  Walls live on the shared cell edges (grid lines)
and on the outer boundary; the only boundary opening is the entrance on the
west of ``(0, 0)``.
"""

import math
import random

from .geometry import Wall, WALL_THICKNESS, END_EXTEND, SPAWN

# Directions as (dc, dr) with names; +x = East, +y = North.
_DIRS = {"E": (1, 0), "N": (0, 1), "W": (-1, 0), "S": (0, -1)}
# Left turn order used by the left-hand rule (relative preference).
_LEFT_OF = {"E": "N", "N": "W", "W": "S", "S": "E"}
_RIGHT_OF = {"E": "S", "S": "W", "W": "N", "N": "E"}
_BACK_OF = {"E": "W", "W": "E", "N": "S", "S": "N"}


def carve(cols, rows, seed):
    """Recursive-backtracker perfect maze.

    Returns a set of frozenset({(c,r),(c2,r2)}) of OPEN passages between
    adjacent cells (every cell reachable, no loops)."""
    rng = random.Random(f"maze-{cols}x{rows}-{seed}")
    visited = [[False] * rows for _ in range(cols)]
    openings = set()
    stack = [(0, 0)]
    visited[0][0] = True
    while stack:
        c, r = stack[-1]
        nbrs = []
        for dc, dr in _DIRS.values():
            nc, nr = c + dc, r + dr
            if 0 <= nc < cols and 0 <= nr < rows and not visited[nc][nr]:
                nbrs.append((nc, nr))
        if not nbrs:
            stack.pop()
            continue
        nc, nr = rng.choice(nbrs)
        openings.add(frozenset({(c, r), (nc, nr)}))
        visited[nc][nr] = True
        stack.append((nc, nr))
    return openings


def _cell_center(c, r, pitch):
    return (SPAWN[0] + 3.0 + c * pitch, SPAWN[1] + r * pitch)


STUB_X0 = -4.0   # entrance-corridor stub starts here (robot spawns at x=-3)


def maze_walls(cols, rows, openings, pitch, extend=END_EXTEND):
    """Wall boxes for every closed cell edge + the boundary (entrance open).

    The entrance is the WEST side of cell (0,0); every other boundary edge and
    every un-carved interior edge becomes a wall box on the grid line.  A short
    entrance corridor (north + south walls) leads in from the spawn so the
    robot always starts with a wall on its left, whatever the maze carved.
    """
    walls = []
    h = pitch / 2.0

    # entrance corridor stub from the spawn into the west side of cell (0, 0)
    ex0, ex1 = STUB_X0, SPAWN[0] + 3.0 - h   # west edge of cell (0,0)
    if ex1 > ex0:
        stub_cx, stub_len = (ex0 + ex1) / 2.0, ex1 - ex0
        walls.append(Wall(stub_cx, SPAWN[1] + h, 0.0, stub_len + 2.0 * extend))
        walls.append(Wall(stub_cx, SPAWN[1] - h, 0.0, stub_len + 2.0 * extend))

    def hwall(cx, cy, length):
        walls.append(Wall(cx, cy, 0.0, length + 2.0 * extend))

    def vwall(cx, cy, length):
        walls.append(Wall(cx, cy, math.pi / 2.0, length + 2.0 * extend))

    for c in range(cols):
        for r in range(rows):
            x, y = _cell_center(c, r, pitch)
            # EAST edge: wall unless carved to (c+1, r); boundary on last col
            if c == cols - 1:
                vwall(x + h, y, pitch)
            elif frozenset({(c, r), (c + 1, r)}) not in openings:
                vwall(x + h, y, pitch)
            # NORTH edge
            if r == rows - 1:
                hwall(x, y + h, pitch)
            elif frozenset({(c, r), (c, r + 1)}) not in openings:
                hwall(x, y + h, pitch)
            # SOUTH boundary (interior souths handled as another cell's north)
            if r == 0:
                hwall(x, y - h, pitch)
            # WEST boundary: wall except the entrance at (0, 0)
            if c == 0 and not (c == 0 and r == 0):
                vwall(x - h, y, pitch)
    return walls


def left_hand_path(cols, rows, openings, pitch, max_steps=4000):
    """Centerline waypoints for the left-hand-rule tour of the maze.

    Starts west of the entrance heading East and keeps the left hand on the
    wall (prefer left, then straight, then right, then back) until it returns
    to the entrance.  Returns a list of (x, y) cell-centre vertices, prefixed
    with the spawn approach so the path begins behind the robot.
    """
    def is_open(c, r, d):
        dc, dr = _DIRS[d]
        nc, nr = c + dc, r + dr
        if not (0 <= nc < cols and 0 <= nr < rows):
            return False
        return frozenset({(c, r), (nc, nr)}) in openings

    pts = [(STUB_X0 + 0.5, SPAWN[1]), _cell_center(0, 0, pitch)]
    c, r, heading = 0, 0, "E"
    seen = set()
    for _ in range(max_steps):
        state = (c, r, heading)
        if state in seen:
            break                       # a (cell, heading) repeat closes the tour
        seen.add(state)
        for nxt in (_LEFT_OF[heading], heading, _RIGHT_OF[heading], _BACK_OF[heading]):
            if is_open(c, r, nxt):
                dc, dr = _DIRS[nxt]
                c, r, heading = c + dc, r + dr, nxt
                pts.append(_cell_center(c, r, pitch))
                break
        else:
            break                       # fully enclosed cell (should not happen)
    pts.append((SPAWN[0] - 4.0, SPAWN[1]))   # exit back out the entrance
    return pts


def maze_extent(cols, rows, pitch):
    """(xmin, ymin, xmax, ymax) of the maze footprint including walls."""
    x0, y0 = _cell_center(0, 0, pitch)
    x1, y1 = _cell_center(cols - 1, rows - 1, pitch)
    h = pitch / 2.0 + WALL_THICKNESS
    return (x0 - h, y0 - h, x1 + h, y1 + h)
