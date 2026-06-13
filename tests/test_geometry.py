"""Geometry unit tests: offsets, bevels, gaps, mirror, validation."""

import math

import pytest

from worldgen import geometry
from worldgen.geometry import (END_EXTEND, Wall, build_path, corner_warnings,
                               corridor_warnings, mirror_walls,
                               offset_polyline, room_walls, validate_walls,
                               walls_with_gaps)

SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]


def test_offset_square_left():
    """Left offset of a CCW square path: every vertex moves 2 m inward."""
    out = offset_polyline(SQUARE, 2.0)
    assert out == pytest.approx([(0, 2), (8, 2), (8, 8), (0, 8)])


def test_offset_square_right():
    # open polyline: endpoints take the plain segment-normal offset (no miter,
    # matching test_offset_square_left); only the interior vertices miter.
    out = offset_polyline(SQUARE, -2.0)
    assert out == pytest.approx([(0, -2), (12, -2), (12, 12), (0, 12)])


def test_offset_closed_square():
    pts = SQUARE + [SQUARE[0]]
    out = offset_polyline(pts, -2.0, closed=True)   # outward ring
    assert out[0] == pytest.approx(out[-1])
    assert out[:-1] == pytest.approx([(-2, -2), (12, -2), (12, 12), (-2, 12)])


def test_miter_join_under_limit_unchanged():
    """A 90-degree turn miters (no bevel): 3 output points, exact corner."""
    pts = build_path([(10.0, -90.0), (10.0, None)], start=(0, 0), heading=0)
    out = offset_polyline(pts, 2.0)   # offset on the OUTSIDE of the turn
    assert len(out) == 3              # still a single miter point
    # miter point: intersection of y=2 and x=12 offset lines
    assert out[1] == pytest.approx((12.0, 2.0))


def test_bevel_fallback_triggers_on_sharp_outside_turn():
    """A 150-degree outside turn would need a 7.7 m miter (> 3x offset):
    the join must fall back to a 2-point bevel chamfer."""
    pts = build_path([(10.0, -150.0), (10.0, None)], start=(0, 0), heading=0)
    out = offset_polyline(pts, 2.0)
    assert len(out) == 4              # vertex expanded into two bevel points
    v = pts[1]
    for b in out[1:3]:
        assert math.hypot(b[0] - v[0], b[1] - v[1]) == pytest.approx(2.0)


def test_bevel_not_used_on_inside_of_sharp_turn():
    """Same 150-degree turn, offset on the INSIDE: bevel would hole the
    corner, so the join stays a miter."""
    pts = build_path([(10.0, 150.0), (10.0, None)], start=(0, 0), heading=0)
    out = offset_polyline(pts, 2.0)
    assert len(out) == 3


def test_walls_with_gaps_hole_size():
    walls = walls_with_gaps((0.0, 0.0), (30.0, 0.0), [(12.0, 2.5)])
    assert len(walls) == 2
    (a0, b0), (a1, b1) = walls[0].endpoints(), walls[1].endpoints()
    # gap-facing edges exactly 2.5 m apart, centered at x=12
    assert b0[0] == pytest.approx(12.0 - 1.25)
    assert a1[0] == pytest.approx(12.0 + 1.25)
    # outer ends keep the sealing extension
    assert a0[0] == pytest.approx(-END_EXTEND)
    assert b1[0] == pytest.approx(30.0 + END_EXTEND)


def test_walls_with_gaps_two_gaps():
    walls = walls_with_gaps((0.0, 0.0), (30.0, 0.0),
                            [(20.0, 3.0), (10.0, 2.0)])
    assert len(walls) == 3
    assert walls[0].endpoints()[1][0] == pytest.approx(9.0)
    assert walls[1].endpoints()[0][0] == pytest.approx(11.0)


def test_walls_with_gaps_rejects_overlap():
    with pytest.raises(ValueError):
        walls_with_gaps((0, 0), (30, 0), [(10.0, 5.0), (12.0, 5.0)])


def test_mirror_walls():
    w = Wall(cx=3.0, cy=4.0, yaw=0.7, length=5.0)
    (m,) = mirror_walls([w])
    assert (m.cx, m.cy, m.yaw, m.length) == pytest.approx((3.0, 0.0, -0.7, 5.0))
    # reflection about y=2: endpoints mirror exactly
    for (ox, oy), (mx, my) in zip(w.endpoints(), m.endpoints()):
        assert (mx, my) == pytest.approx((ox, 4.0 - oy))
    # the spawn line is a fixed point
    assert geometry.mirror_point(geometry.SPAWN) == pytest.approx(
        geometry.SPAWN)


def test_validate_catches_pinch():
    walls = [
        Wall(0.0, 4.0, 0.0, 10.0),
        Wall(20.0, 4.0, 0.0, 5.0),     # spacer so the pair is non-adjacent
        Wall(0.0, 4.3, 0.0, 10.0),     # 0.3 m from wall 0: deliberate pinch
    ]
    problems = validate_walls(walls)
    assert any("pinch" in p for p in problems)


def test_validate_passes_room():
    assert validate_walls(room_walls()) == []


def test_validate_allows_arc_chains():
    """Short chords of a smooth arc must not be flagged as pinches."""
    from worldgen import presets
    walls = presets.preset_walls("curve_left")
    assert validate_walls(walls) == []


def test_corner_warnings_sharp_inside():
    assert corner_warnings([(10.0, -150.0), (10.0, None)])
    assert corner_warnings([(10.0, -90.0), (10.0, None)]) == []
    # outside corners are always trackable
    assert corner_warnings([(10.0, 150.0), (10.0, None)]) == []


def test_corridor_warnings_narrow():
    pts = build_path([(20.0, None)])
    assert corridor_warnings(pts, 2.0, width=1.5)      # too narrow
    assert corridor_warnings(pts, 2.0, width=4.0) == []


def test_rounded_rect_closes():
    pts = geometry.rounded_rect_path(-12.0, -10.0, 10.0, 2.0,
                                     [2.5, 0.0, 3.0, 2.0])
    assert pts[0] == pytest.approx(pts[-1])
    assert pts[0] == pytest.approx(geometry.PATH_START)
