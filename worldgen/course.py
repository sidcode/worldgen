"""Course sidecar files: ground-truth intent for the eval scorer.

Every generated world gets a ``<name>.course.json`` next to the ``.world``
describing what the robot is *supposed* to do, so ``worldgen eval`` can score
a recorded run without re-deriving geometry from the SDF:

    {
      "schema_version": 1,
      "name": "gen_zigzag",
      "spawn": [-3.0, 2.0, 0.0],          # x, y, yaw
      "desired_distance": 1.0,
      "offset": 2.0,
      "two_sided": false,
      "mirror": false,
      "path": [[x, y], ...],              # course centerline vertices
      "closed": false,                    # closed loop (path[0] == path[-1])
      "generator": {"preset": "zigzag", ...}   # or {"seed": ..., ...}
    }

The ``path`` is the centerline the robot is expected to drive (already
mirrored when the world was generated with --mirror).  The scorer projects
the recorded odometry onto it to measure completion.
"""

import json
from pathlib import Path

from .geometry import DESIRED_DISTANCE, SPAWN, SPAWN_YAW

SCHEMA_VERSION = 1


def course_dict(name, path_pts, offset, two_sided, mirror, closed, generator,
                desired_distance=DESIRED_DISTANCE):
    return {
        "schema_version": SCHEMA_VERSION,
        "name": name,
        "spawn": [SPAWN[0], SPAWN[1], SPAWN_YAW],
        "desired_distance": desired_distance,
        "offset": offset,
        "two_sided": two_sided,
        "mirror": mirror,
        "path": [[round(x, 4), round(y, 4)] for x, y in path_pts],
        "closed": closed,
        "generator": generator,
    }


def write_course(out_dir, data):
    """Write <name>.course.json into out_dir; returns the path."""
    path = Path(out_dir) / f"{data['name']}.course.json"
    path.write_text(json.dumps(data, indent=2) + "\n")
    return path


def load_course(path):
    with open(path) as f:
        return json.load(f)


def path_length(path_pts):
    total = 0.0
    for (ax, ay), (bx, by) in zip(path_pts, path_pts[1:]):
        total += ((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5
    return total


def suggested_duration(path_pts, closed=False,
                       speed=0.7, fudge=1.6, lo=45, hi=300):
    """Run duration estimate: path length / forward speed x fudge factor.

    Closed loops get ~1.2 laps so the scorer can observe sustained looping.
    Clamped to [lo, hi] seconds.
    """
    length = path_length(path_pts) * (1.2 if closed else 1.0)
    return int(max(lo, min(hi, round(length / speed * fudge))))
