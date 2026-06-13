# worldgen: wall-following test world generator + evaluation harness

Generates Gazebo `.world` files for the `wall_following_assigment` package so
the PID wall follower can be tested on arbitrary wall configurations (not just
the two stock worlds), **and** scores how well the follower drives them.

All geometry respects the assignment's conventions: the Husky spawns at
**(-3, 2) facing +x** (`gazebo.launch.py`) and follows the wall on its
**left**, so every generated course starts with a wall ~2 m to the robot's
left (same as `walls_one_sided.world`). Walls are the same static
0.2 m-thick, 2.8 m-tall grey boxes the stock worlds use.

Runtime is **stdlib-only**; `pytest` is a dev dependency. Run everything from
this directory with [uv](https://docs.astral.sh/uv/).

## Generating worlds

```bash
uv run worldgen list                        # presets + what each one tests
uv run worldgen generate zigzag             # write gen_zigzag.world + preview
uv run worldgen generate gaps --gap-width 3.0
uv run worldgen generate curve_left --clutter 4 --seed 7
uv run worldgen generate right_turn --two-sided --mirror
uv run worldgen random --seed 42 --segments 7 --arcs
uv run worldgen random --seed 5 --closed    # endless randomized loop
uv run worldgen preview ../wall_following_assigment/worlds/walls_two_sided.world
```

Worlds are written to `../wall_following_assigment/worlds/` by default
(`--out` to override), prefixed `gen_`. Add `--dry-run` to preview/validate
without writing. Each world is accompanied by a **`<name>.course.json`**
sidecar (see below) and embeds its generator parameters in an XML comment.

### Presets

| preset        | what it stresses                                          |
|---------------|-----------------------------------------------------------|
| `straight`    | convergence to the desired distance, steady-state error   |
| `left_turn`   | outside corner — wall falls away                          |
| `right_turn`  | inside corner — front-cone safety override                |
| `zigzag`      | oscillation / PID gain tuning (±60°)                      |
| `s_curve`     | smooth tracking through gentle 45° bends                  |
| `u_turn`      | back-to-back outside corners (180°)                       |
| `gauntlet`    | mixed inside + outside corners                            |
| `room`        | closed loop, four inside corners, runs forever            |
| `gaps`        | doorway gaps in the wall → **wall-loss / reacquire**      |
| `curve_left`  | constant-radius 135° left arc (wall inside the bend)      |
| `curve_right` | constant-radius 135° right arc (wall outside the bend)    |

### Flags (generate / random)

| flag | effect |
|------|--------|
| `--two-sided` | build a corridor (walls on both sides), like `walls_two_sided.world` |
| `--offset M` | lateral distance from course centerline to the left wall (default 2.0) |
| `--width M` | corridor width for `--two-sided` (default `2*offset`) |
| `--clutter N` | place N small boxes/cylinders against the followed wall (deterministic per `--seed`) |
| `--mirror` | reflect the world about the spawn line so the wall is on the robot's **right** (test `side_sign=+1`); name gets `_mirror` |
| `--gap-width M`, `--gap-count {1,2}` | doorway geometry for the `gaps` preset |
| `--arcs` *(random)* | replace ~half the corners with constant-radius 6–8 m arcs |
| `--closed` *(random)* | generate a closed rounded-rectangle loop the robot can lap forever |
| `--max-turn DEG` *(random)* | corner sharpness, now up to ±120° (bevel join past the miter limit) |
| `--seed`, `--segments`, `--min/max-len`, `--min/max-turn` | random course controls |

`random` is fully deterministic for a given seed + parameters and rejects
courses that self-intersect, pinch, or clip the spawn. Every generated world
is validated; problems (pinch/overlap, unreachable corners sharper than the
robot's ~0.58 m turn radius, corridors too narrow for desired-distance +
footprint) are printed to stderr as warnings.

### Course sidecar (`<name>.course.json`)

The ground-truth "intent" of each world, used by the scorer:

```json
{ "schema_version": 1, "name": "...", "spawn": [-3.0, 2.0, 0.0],
  "desired_distance": 1.0, "offset": 2.0, "two_sided": false,
  "mirror": false, "closed": false,
  "path": [[x, y], ...],            // course centerline vertices (world frame)
  "generator": { ... } }            // preset/seed/params for reproducibility
```

## Evaluation suite

Generate a battery of worlds, run the follower on each headless, and score it.

```bash
uv run worldgen suite --seeds 1,2,3          # 11 presets + 3 random worlds + manifest
uv run worldgen eval                         # run+score every world in the manifest
uv run worldgen eval gen_gaps gen_curve_left # just these two
uv run worldgen eval --score-only            # rescore existing recordings, no sim
```

`suite` writes the worlds, their sidecars, and a `suite_manifest.json` (each
entry has a suggested recording duration scaled to the course length).

`eval` drives `docker/run.sh record <world> <secs>` for each world (which
deploys the world into the container, runs it headless, and captures cte +
trajectory CSVs into `docker/output/`), then scores against the sidecar:

| metric | meaning |
|--------|---------|
| `mean/max/rms |cte|` | cross-track error over the **on-course** portion of the run |
| `in±0.2m` | % of cte samples within ±0.2 m of the setpoint |
| `compl` | completion: furthest arc-length reached ÷ course length (closed loops can exceed 100%) |
| `dist(m)` | distance travelled along the course |
| `stall` | robot stopped advancing ≥ 8 s mid-course → likely crash |

cte and the trajectory are timestamped off a shared clock, and the trajectory
is transformed from the odom frame into the course world frame via the sidecar
spawn pose. Once forward progress peaks (the robot reaches the end of a finite
course and drives off the wall), later cte is **not** scored — driving off the
end of the wall is not a tracking error.

**Verdicts:** `PASS` if completion > 90 %, mean |cte| < 0.3 m, and no stall;
`FAIL` if stalled, completion < 60 %, or mean |cte| > 0.5 m; `WARN` otherwise.
A markdown table is printed and written to `docker/output/eval_report.md`;
`worldgen eval` exits non-zero if any world FAILs.

## Using a single world interactively

`docker/run.sh` accepts any world name and copies host-generated worlds into
the running container (no image rebuild):

```bash
cd ../../docker
./run.sh solution gen_zigzag     # Gazebo GUI + Husky + PID follower + rqt
./run.sh viz gen_zigzag          # headless Gazebo + RViz instead of gzclient
./run.sh record gen_zigzag 90    # headless run, saves cte + traj CSV + bag
```

In a plain ROS 2 workspace (no Docker), rebuild so the installed share picks
up the new world:

```bash
colcon build --packages-select wall_following_assigment
ros2 launch wall_following_assigment solution.launch.py world:=gen_zigzag.world
```

## Tests

```bash
uv run --group dev pytest -q
```

`tests/test_geometry.py` covers offsets, bevel fallback, gaps, mirror, and
validation; `tests/test_evaluate.py` covers the scoring math against synthetic
clean / oscillating / stalled recordings.
