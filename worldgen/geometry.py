"""Path / wall geometry for the wall-following worlds.

Conventions (match the assignment setup):
  * The Husky spawns at SPAWN facing +x (yaw 0), see gazebo.launch.py.
  * The wall follower tracks the wall on the robot's LEFT (+y at spawn).
  * A "path" is the course the robot is expected to drive, given as a list of
    (segment_length_m, turn_after_deg) tuples.  Positive turn = left (CCW).
  * Walls are static boxes; the left wall is the path offset to the left by
    `offset` metres, the optional right wall is offset to the right.

Corner joins: interior vertices of an offset polyline normally use a miter
join so consecutive wall boxes meet exactly.  Toward 180-degree turns the
miter point runs away to infinity, so when the join sits on the OUTSIDE of a
turn and its length exceeds ``miter_limit * |offset|`` (default 3x) we fall
back to a bevel: the two natural offset endpoints joined by a short chamfer
box.  Inside joins always miter (a bevel there would punch a hole in the
corner).
"""

import math
from dataclasses import dataclass

SPAWN = (-3.0, 2.0)          # husky spawn position (gazebo.launch.py)
SPAWN_YAW = 0.0              # facing +x
WALL_THICKNESS = 0.2
WALL_HEIGHT = 2.8
PATH_START = (-7.0, 2.0)     # path begins behind the robot so the wall is
                             # already beside it at t=0
END_EXTEND = 0.15            # lengthen each wall box slightly to seal corners
MITER_LIMIT = 3.0            # bevel fallback once miter length > limit*offset

# Robot / controller constants used by the kinematic feasibility checks.
DESIRED_DISTANCE = 1.0       # wall follower setpoint (m)
HUSKY_HALF_WIDTH = 0.5       # approximate half footprint of the Husky (m)
FORWARD_SPEED = 0.7          # solution forward speed (m/s)
MAX_ANGULAR = 1.2            # solution max yaw rate (rad/s)
MIN_TURN_RADIUS = FORWARD_SPEED / MAX_ANGULAR   # ~0.58 m


@dataclass
class Wall:
    cx: float
    cy: float
    yaw: float
    length: float

    def endpoints(self):
        hx = 0.5 * self.length * math.cos(self.yaw)
        hy = 0.5 * self.length * math.sin(self.yaw)
        return (self.cx - hx, self.cy - hy), (self.cx + hx, self.cy + hy)


def build_path(segments, start=PATH_START, heading=SPAWN_YAW):
    """Turn (length, turn_after_deg) segments into a vertex list."""
    pts = [start]
    h = heading
    for length, turn in segments:
        x, y = pts[-1]
        pts.append((x + length * math.cos(h), y + length * math.sin(h)))
        if turn is not None:
            h += math.radians(turn)
    return pts


def _join(p, d0, d1, n0, n1, offset, miter_limit):
    """Offset join at vertex p between directions d0 -> d1.

    Returns one point (miter) or two points (bevel chamfer).  The bevel is
    used only on the outside of the turn, where the miter spike would
    otherwise grow without bound as the turn approaches 180 degrees.
    """
    dot = n0[0] * n1[0] + n0[1] * n1[1]
    denom = 1.0 + dot
    cross = d0[0] * d1[1] - d0[1] * d1[0]     # > 0: left turn
    outside = (cross * offset) < 0.0          # offset side is turn's outside
    if denom > 1e-9:
        k = offset / denom                    # miter join
        mx, my = k * (n0[0] + n1[0]), k * (n0[1] + n1[1])
        if not (outside and math.hypot(mx, my) > miter_limit * abs(offset)):
            return [(p[0] + mx, p[1] + my)]
    # bevel: the two natural endpoints of the adjacent offset segments
    return [(p[0] + offset * n0[0], p[1] + offset * n0[1]),
            (p[0] + offset * n1[0], p[1] + offset * n1[1])]


def offset_polyline(pts, offset, miter_limit=MITER_LIMIT, closed=False):
    """Offset a polyline to its left by `offset` (negative = right).

    Interior vertices miter; over-long outside miters become bevels (see
    module docstring).  A bevelled vertex contributes two output points, so
    the result can be longer than the input.

    With ``closed=True`` the input must repeat its first point at the end
    (pts[0] == pts[-1]); the output is closed the same way and the seam
    vertex is joined properly.
    """
    n = len(pts)
    dirs, normals = [], []
    for i in range(n - 1):
        dx, dy = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
        length = math.hypot(dx, dy)
        dirs.append((dx / length, dy / length))
        normals.append((-dy / length, dx / length))

    if closed:
        nseg = n - 1
        out = []
        for i in range(nseg):
            prev = (i - 1) % nseg
            out.extend(_join(pts[i], dirs[prev], dirs[i],
                             normals[prev], normals[i], offset, miter_limit))
        out.append(out[0])
        return out

    out = []
    for i in range(n):
        if i == 0:
            nx, ny = normals[0]
            out.append((pts[0][0] + offset * nx, pts[0][1] + offset * ny))
        elif i == n - 1:
            nx, ny = normals[-1]
            out.append((pts[-1][0] + offset * nx, pts[-1][1] + offset * ny))
        else:
            out.extend(_join(pts[i], dirs[i - 1], dirs[i],
                             normals[i - 1], normals[i], offset, miter_limit))
    return out


def walls_from_polyline(opts, extend=END_EXTEND):
    walls = []
    for (ax, ay), (bx, by) in zip(opts, opts[1:]):
        length = math.hypot(bx - ax, by - ay)
        if length < 1e-6:
            continue
        walls.append(Wall(cx=(ax + bx) / 2.0, cy=(ay + by) / 2.0,
                          yaw=math.atan2(by - ay, bx - ax),
                          length=length + 2.0 * extend))
    return walls


def walls_for_path(segments, offset=2.0, two_sided=False, width=None,
                   miter_limit=MITER_LIMIT):
    """Build the wall set for a path.  `width` = corridor width (two-sided)."""
    pts = build_path(segments)
    walls = walls_from_polyline(offset_polyline(pts, offset, miter_limit))
    if two_sided:
        right = offset - (width if width is not None else 2.0 * offset)
        walls += walls_from_polyline(offset_polyline(pts, right, miter_limit))
    return walls


def walls_with_gaps(a, b, gaps, extend=END_EXTEND):
    """Wall boxes along the straight line a->b with rectangular holes.

    ``gaps`` is a list of (center_distance_from_a, width) measured along the
    a->b line.  The outermost ends keep the corner-sealing END_EXTEND
    extension, but gap-facing ends are exact, so each hole is exactly
    ``width`` metres wide edge-to-edge.
    """
    L = math.hypot(b[0] - a[0], b[1] - a[1])
    ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
    yaw = math.atan2(uy, ux)

    cuts = sorted((c - w / 2.0, c + w / 2.0) for c, w in gaps)
    last = 0.0
    for s0, s1 in cuts:
        if s0 < last or s1 > L:
            raise ValueError(
                f"gap [{s0:.2f}, {s1:.2f}] overlaps another gap or falls "
                f"outside the wall (length {L:.2f} m)")
        last = s1

    bounds = [0.0] + [v for c in cuts for v in c] + [L]
    walls = []
    for i in range(0, len(bounds), 2):
        s0, s1 = bounds[i], bounds[i + 1]
        s0e = s0 - (extend if s0 <= 0.0 else 0.0)   # extend only outer ends
        s1e = s1 + (extend if s1 >= L else 0.0)
        if s1e - s0e < 1e-6:
            continue
        mid = (s0e + s1e) / 2.0
        walls.append(Wall(a[0] + mid * ux, a[1] + mid * uy, yaw, s1e - s0e))
    return walls


def arc_segments(radius, angle_deg, seg_len=0.6):
    """Approximate a constant-radius arc by short course segments.

    Returns ``k`` tuples of (chord_length, per_step_turn_deg); the chords
    polygonalise the arc, turning ``angle_deg / k`` after each one.  The
    caller stitches them into a (length, turn) course: the step turn must
    also be applied *before* the first chord (i.e. attached to the previous
    segment) so the polyline stays tangent to the arc.  See
    ``presets.path_with_arc`` for the canonical assembly.
    """
    arc_len = abs(math.radians(angle_deg)) * radius
    k = max(2, math.ceil(arc_len / seg_len))
    return [(arc_len / k, angle_deg / k)] * k


def room_walls(x0=-8.0, y0=-6.0, x1=14.0, y1=4.0):
    """A closed rectangular room (wall centerlines on the rectangle edges).

    With the default top edge at y=4 the spawn (-3, 2) sits 2 m from the left
    wall, like the stock worlds.  Following the left wall drives a clockwise
    loop with four inside corners.
    """
    ext = WALL_THICKNESS  # overlap the corners so the room is sealed
    return [
        Wall((x0 + x1) / 2.0, y1, 0.0, (x1 - x0) + ext),            # top
        Wall((x0 + x1) / 2.0, y0, 0.0, (x1 - x0) + ext),            # bottom
        Wall(x0, (y0 + y1) / 2.0, math.pi / 2.0, (y1 - y0) + ext),  # west
        Wall(x1, (y0 + y1) / 2.0, math.pi / 2.0, (y1 - y0) + ext),  # east
    ]


def room_course_path(x0=-8.0, y0=-6.0, x1=14.0, y1=4.0,
                     inset=DESIRED_DISTANCE + WALL_THICKNESS / 2.0):
    """The closed clockwise loop the robot is expected to drive in the room:
    the wall rectangle inset by desired_distance + half wall thickness."""
    a, b, c, d = x0 + inset, y0 + inset, x1 - inset, y1 - inset
    return [(a, d), (c, d), (c, b), (a, b), (a, d)]


def rounded_rect_path(x0, y0, x1, y1, radii, seg_len=0.7, start=PATH_START):
    """Closed clockwise course on rectangle (x0..x1) x (y0..y1), starting at
    ``start`` on the top edge (y1) heading +x.  ``radii`` are the corner
    fillet radii in driving order (TR, BR, BL, TL); 0 = sharp 90-degree
    corner.  Returns a closed vertex list (first == last).

    Guaranteed closure: each fillet only trims ``r`` off the two sides that
    meet at its corner, so the loop always returns to ``start``.  The caller
    must keep ``start`` outside every fillet (corner_x + r < start_x on the
    top edge).
    """
    corners = [(x1, y1), (x1, y0), (x0, y0), (x0, y1)]
    dirs = [(1, 0), (0, -1), (-1, 0), (0, 1)]   # heading INTO each corner
    pts = [start]
    for i, ((cx, cy), r) in enumerate(zip(corners, radii)):
        d_in = dirs[i]
        d_out = dirs[(i + 1) % 4]
        if r <= 0.0:
            pts.append((cx, cy))
            continue
        sx, sy = cx - r * d_in[0], cy - r * d_in[1]      # fillet start
        # clockwise (right) turn: center is to the right of d_in
        ox, oy = sx + r * d_in[1], sy - r * d_in[0]
        a0 = math.atan2(sy - oy, sx - ox)
        k = max(2, math.ceil(r * (math.pi / 2.0) / seg_len))
        for j in range(k + 1):
            a = a0 - (math.pi / 2.0) * j / k             # sweep -90 deg
            pts.append((ox + r * math.cos(a), oy + r * math.sin(a)))
    pts.append(start)
    return pts


def mirror_point(p, about_y=SPAWN[1]):
    """Reflect a point about the horizontal line y = about_y (default: the
    y=2 line through the spawn, so the spawn is a fixed point)."""
    return (p[0], 2.0 * about_y - p[1])


def mirror_walls(walls, about_y=SPAWN[1]):
    """Reflect walls about the y = about_y line through the spawn.

    The robot still spawns at (-3, 2) facing +x, but the followed wall ends
    up on its RIGHT — use this to test right-wall following (side_sign=+1).
    """
    return [Wall(w.cx, 2.0 * about_y - w.cy, -w.yaw, w.length) for w in walls]


# --------------------------------------------------------------- validation

def _seg_seg_distance(p1, p2, p3, p4):
    """Minimum distance between segments p1-p2 and p3-p4."""
    def dot(a, b):
        return a[0] * b[0] + a[1] * b[1]

    def point_seg(p, a, b):
        ab = (b[0] - a[0], b[1] - a[1])
        ap = (p[0] - a[0], p[1] - a[1])
        denom = dot(ab, ab)
        t = 0.0 if denom == 0 else max(0.0, min(1.0, dot(ap, ab) / denom))
        cx, cy = a[0] + t * ab[0], a[1] + t * ab[1]
        return math.hypot(p[0] - cx, p[1] - cy)

    d1 = (p2[0] - p1[0], p2[1] - p1[1])
    d2 = (p4[0] - p3[0], p4[1] - p3[1])
    cross = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(cross) > 1e-12:
        qp = (p3[0] - p1[0], p3[1] - p1[1])
        t = (qp[0] * d2[1] - qp[1] * d2[0]) / cross
        u = (qp[0] * d1[1] - qp[1] * d1[0]) / cross
        if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
            return 0.0  # they intersect
    return min(point_seg(p1, p3, p4), point_seg(p2, p3, p4),
               point_seg(p3, p1, p2), point_seg(p4, p1, p2))


def validate_walls(walls, min_wall_clearance=0.8, min_spawn_clearance=1.2):
    """Return a list of human-readable problems (empty = world looks sane).

    Adjacent wall boxes are allowed to touch (corners); everything else must
    keep some clearance or the lidar/robot would see an unintended pinch.
    """
    problems = []
    segs = [w.endpoints() for w in walls]
    corner_tol = WALL_THICKNESS + 2.0 * END_EXTEND

    def touch(sa, sb):
        return min(math.hypot(a[0] - b[0], a[1] - b[1])
                   for a in sa for b in sb) <= corner_tol

    for i in range(len(segs)):
        for j in range(i + 2, len(segs)):  # skip self and adjacent neighbours
            d = _seg_seg_distance(*segs[i], *segs[j])
            if d < min_wall_clearance:
                # endpoint-to-endpoint contact is a sealed corner, not a pinch
                ends = min(math.hypot(a[0] - b[0], a[1] - b[1])
                           for a in segs[i] for b in segs[j])
                # A genuine corner/continuation touches only at the shared
                # endpoint, so the segment MIDPOINTS stay clear.  Parallel
                # walls running alongside each other (a real pinch) are close
                # along their whole length, so their midpoints are close too.
                mid_i = ((segs[i][0][0] + segs[i][1][0]) / 2.0,
                         (segs[i][0][1] + segs[i][1][1]) / 2.0)
                mid_j = ((segs[j][0][0] + segs[j][1][0]) / 2.0,
                         (segs[j][0][1] + segs[j][1][1]) / 2.0)
                mids_clear = (
                    _seg_seg_distance(mid_i, mid_i, *segs[j]) >= min_wall_clearance
                    and _seg_seg_distance(mid_j, mid_j, *segs[i]) >= min_wall_clearance)
                if ends <= corner_tol and mids_clear:
                    continue
                # short chords of a smooth arc: i..i+2 chained end-to-end
                if j == i + 2 and touch(segs[i], segs[i + 1]) \
                        and touch(segs[i + 1], segs[j]):
                    continue
                problems.append(
                    f"walls {i} and {j} are only {d:.2f} m apart "
                    f"(< {min_wall_clearance} m): unintended pinch/overlap")
    for i, w in enumerate(walls):
        (a, b) = w.endpoints()
        d = _seg_seg_distance(SPAWN, SPAWN, a, b)
        if d < min_spawn_clearance:
            problems.append(
                f"wall {i} is {d:.2f} m from the robot spawn {SPAWN} "
                f"(< {min_spawn_clearance} m): robot may collide at spawn")
    return problems


def corner_warnings(segments, desired_distance=DESIRED_DISTANCE,
                    min_radius=MIN_TURN_RADIUS):
    """Kinematic feasibility of the corners of a (length, turn) course.

    Outside corners (positive turn for a left-side wall) are always
    trackable: the robot just pivots around the wall end at roughly the
    desired distance (1.0 m >> the ~0.58 m minimum turn radius at forward
    0.7 m/s, max yaw 1.2 rad/s).  Inside corners (negative turn) squeeze the
    robot into the wedge between the old and new wall: the largest arc that
    fits while keeping the desired distance has radius approximately
    ``desired * tan((180 - |turn|) / 2)``, which dips below the robot's
    minimum radius just past 120 degrees.
    """
    warnings = []
    for i, (_length, turn) in enumerate(segments):
        if turn is None or turn >= 0.0:
            continue
        th = abs(math.radians(turn))
        if th >= math.pi - 1e-9:
            avail = 0.0
        else:
            avail = desired_distance * math.tan((math.pi - th) / 2.0)
        if avail < min_radius:
            warnings.append(
                f"segment {i}: {turn:+.0f} deg inside corner leaves a "
                f"turning radius of ~{avail:.2f} m, below the robot's "
                f"minimum ~{min_radius:.2f} m (forward {FORWARD_SPEED} m/s, "
                f"max yaw {MAX_ANGULAR} rad/s): expect overshoot or a "
                f"wall hit")
    return warnings


def corridor_warnings(pts, offset, width=None, closed=False,
                      desired_distance=DESIRED_DISTANCE,
                      margin=0.3, miter_limit=MITER_LIMIT):
    """Check a two-sided corridor fits the robot at every point.

    Builds both offset chains and measures the minimum face-to-face distance
    between them (corners and bevels included).  The robot tracks the left
    wall at ``desired_distance`` and is ~``HUSKY_HALF_WIDTH`` wide, so the
    corridor must clear ``desired + half_width + margin`` everywhere.
    """
    w = width if width is not None else 2.0 * offset
    need = desired_distance + HUSKY_HALF_WIDTH + margin
    left = offset_polyline(pts, offset, miter_limit, closed=closed)
    right = offset_polyline(pts, offset - w, miter_limit, closed=closed)
    lsegs = list(zip(left, left[1:]))
    rsegs = list(zip(right, right[1:]))
    min_d = min(_seg_seg_distance(a, b, c, d)
                for (a, b) in lsegs for (c, d) in rsegs)
    clear = min_d - WALL_THICKNESS   # centerline distance -> face-to-face
    if clear < need:
        return [
            f"corridor narrows to {clear:.2f} m face-to-face, less than the "
            f"{need:.2f} m the robot needs (desired {desired_distance} m + "
            f"half-width {HUSKY_HALF_WIDTH} m + {margin} m margin)"]
    return []
