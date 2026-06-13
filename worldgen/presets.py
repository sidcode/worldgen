"""Course presets and the random course generator.

Each preset is a list of (segment_length_m, turn_after_deg) tuples describing
the course the robot drives.  Positive turn = LEFT (the followed wall falls
away: outside corner).  Negative turn = RIGHT (the followed wall cuts across
the robot's path: inside corner, exercises the front-cone safety override).

Arcs are not special: a constant-radius curve is just many short segments
with small turns (see geometry.arc_segments / path_with_arc), so every
downstream stage (offsetting, SDF, preview, sidecar) handles them for free.
"""

import math
import random

from . import geometry

# Default gap parameters for the 'gaps' preset (overridable from the CLI).
GAP_WIDTH = 2.5
GAP_COUNT = 2
_GAPS_COURSE_LEN = 30.0


def path_with_arc(lead_in, radius, angle_deg, lead_out, seg_len=0.8):
    """Course: straight lead-in, constant-radius arc, straight lead-out.

    The arc is ``k`` chords of ~seg_len with a turn of angle/k applied after
    the lead-in and after each chord but the last, so the polyline stays
    tangent to the circle and the total heading change is exactly angle_deg.
    """
    arc = geometry.arc_segments(radius, angle_deg, seg_len)
    chord, step = arc[0]
    return ([(lead_in, step)] + [(chord, step)] * (len(arc) - 1)
            + [(chord, 0.0), (lead_out, None)])


PRESETS = {
    "straight": {
        "segments": [(28.0, None)],
        "doc": "Single long wall. Baseline convergence / steady-state test.",
    },
    "left_turn": {
        "segments": [(14.0, 90.0), (14.0, None)],
        "doc": "One 90 deg outside corner (wall falls away to the left).",
    },
    "right_turn": {
        "segments": [(14.0, -90.0), (14.0, None)],
        "doc": "One 90 deg inside corner (wall ahead, front-override test).",
    },
    "zigzag": {
        "segments": [(10.0, 60.0), (9.0, -60.0), (10.0, 60.0),
                     (9.0, -60.0), (10.0, None)],
        "doc": "Alternating +/-60 deg turns. Oscillation / gain tuning test.",
    },
    "u_turn": {
        "segments": [(12.0, 90.0), (8.0, 90.0), (16.0, None)],
        "doc": "Two consecutive left 90s: a full 180 around the wall end.",
    },
    "s_curve": {
        "segments": [(8.0, 45.0), (9.0, -45.0), (9.0, -45.0),
                     (9.0, 45.0), (8.0, None)],
        "doc": "Gentle S of 45 deg turns. Smooth-tracking test.",
    },
    "gauntlet": {
        "segments": [(10.0, -90.0), (8.0, 90.0), (8.0, 90.0),
                     (8.0, -90.0), (12.0, None)],
        "doc": "Mixed inside + outside corners back to back.",
    },
    "room": {
        "room": True,
        "doc": "Closed rectangular room; endless clockwise loop with four "
               "inside corners.",
    },
    "gaps": {
        "gaps": True,
        "segments": [(_GAPS_COURSE_LEN, None)],
        "doc": "Straight left wall with doorway gaps (default 2 x 2.5 m). "
               "Wall-loss / reacquisition test.",
    },
    "curve_left": {
        "segments": path_with_arc(6.0, 7.0, 135.0, 6.0),
        "doc": "Constant-radius 135 deg LEFT arc (R=7 m, wall inside the "
               "bend). Smooth curvature-tracking test.",
    },
    "curve_right": {
        "segments": path_with_arc(6.0, 7.0, -135.0, 6.0),
        "doc": "Constant-radius 135 deg RIGHT arc (R=7 m, wall outside the "
               "bend). Sustained inside-curve test.",
    },
}


def _gaps_walls(offset, two_sided, width, gap_width, gap_count):
    """Straight course whose LEFT wall has 1-2 doorway-style gaps."""
    if gap_count not in (1, 2):
        raise ValueError("--gap-count must be 1 or 2")
    pts = geometry.build_path(PRESETS["gaps"]["segments"])
    left = geometry.offset_polyline(pts, offset)
    a, b = left[0], left[-1]
    L = math.hypot(b[0] - a[0], b[1] - a[1])
    centers = [0.4 * L, 0.7 * L] if gap_count == 2 else [0.5 * L]
    walls = geometry.walls_with_gaps(a, b, [(c, gap_width) for c in centers])
    if two_sided:
        right = offset - (width if width is not None else 2.0 * offset)
        walls += geometry.walls_from_polyline(
            geometry.offset_polyline(pts, right))
    return walls


def preset_walls(name, offset=2.0, two_sided=False, width=None,
                 gap_width=GAP_WIDTH, gap_count=GAP_COUNT):
    spec = PRESETS[name]
    if spec.get("room"):
        if two_sided:
            raise ValueError("the 'room' preset is always one-sided")
        return geometry.room_walls()
    if spec.get("gaps"):
        return _gaps_walls(offset, two_sided, width, gap_width, gap_count)
    return geometry.walls_for_path(spec["segments"], offset=offset,
                                   two_sided=two_sided, width=width)


def preset_path(name):
    """The course centerline the robot is expected to drive (sidecar truth).

    Open presets: the offset path vertex list.  'room': the closed loop at
    desired-distance inside the walls.
    """
    spec = PRESETS[name]
    if spec.get("room"):
        return geometry.room_course_path(), True
    return geometry.build_path(spec["segments"]), False


def random_walls(seed, n_segments=6, offset=2.0, two_sided=False, width=None,
                 min_len=6.0, max_len=13.0, min_turn=30.0, max_turn=90.0,
                 arcs=False, max_attempts=500):
    """Random course that passes validation (no self-pinch, spawn clear).

    Deterministic for a given seed + parameters.  With ``arcs=True`` roughly
    half the corners (per the seeded rng) are replaced by constant-radius
    arcs of 6-8 m sweeping the same angle.
    """
    rng = random.Random(seed)
    for _ in range(max_attempts):
        segments = []
        for i in range(n_segments):
            length = rng.uniform(min_len, max_len)
            if i == n_segments - 1:
                segments.append((length, None))
                break
            turn = rng.choice([-1.0, 1.0]) * rng.uniform(min_turn, max_turn)
            if arcs and rng.random() < 0.5:
                radius = rng.uniform(6.0, 8.0)
                arc = geometry.arc_segments(radius, turn, seg_len=0.8)
                chord, step = arc[0]
                segments.append((length, step))
                segments.extend([(chord, step)] * (len(arc) - 1))
                segments.append((chord, 0.0))
            else:
                segments.append((length, turn))
        walls = geometry.walls_for_path(segments, offset=offset,
                                        two_sided=two_sided, width=width)
        if not geometry.validate_walls(walls):
            return walls, segments
    raise RuntimeError(
        f"could not find a valid random course in {max_attempts} attempts; "
        "try a different seed, fewer segments or gentler turns")


def closed_loop(seed, offset=2.0, seg_len=0.7, max_attempts=200):
    """Random CLOSED course: a rounded-rectangle loop around the spawn.

    The course is a clockwise loop through PATH_START on its top edge; the
    left-offset wall therefore encloses it completely, so the robot can
    follow it forever (like the 'room' preset but randomized).  Rounded
    rectangles with per-corner fillets guarantee closure and
    non-self-intersection by construction, unlike free polylines.

    Returns (walls, path_pts, params); path_pts is closed (first == last).
    Deterministic per seed.
    """
    rng = random.Random(f"closed-{seed}")
    y1 = geometry.PATH_START[1]
    for _ in range(max_attempts):
        x0 = rng.uniform(-13.0, -10.5)
        x1 = rng.uniform(8.0, 16.0)
        y0 = y1 - rng.uniform(10.0, 16.0)
        radii = [0.0 if rng.random() < 0.4 else round(rng.uniform(2.0, 3.5), 2)
                 for _ in range(4)]
        # keep PATH_START on a straight stretch of the top edge
        if x0 + radii[3] > geometry.PATH_START[0] - 0.5:
            continue
        pts = geometry.rounded_rect_path(x0, y0, x1, y1, radii, seg_len)
        walls = geometry.walls_from_polyline(
            geometry.offset_polyline(pts, offset, closed=True))
        if not geometry.validate_walls(walls):
            params = {"x0": round(x0, 2), "y0": round(y0, 2),
                      "x1": round(x1, 2), "y1": y1, "radii": radii}
            return walls, pts, params
    raise RuntimeError(
        f"could not find a valid closed loop in {max_attempts} attempts; "
        "try a different seed")
