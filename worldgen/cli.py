"""worldgen -- generate Gazebo wall-following test worlds.

Examples:
    worldgen list
    worldgen generate zigzag
    worldgen generate gaps --gap-width 3.0
    worldgen generate curve_left --clutter 4 --seed 7
    worldgen generate right_turn --two-sided --mirror
    worldgen random --seed 42 --segments 7 --arcs
    worldgen random --seed 5 --closed
    worldgen suite --seeds 1,2,3
    worldgen eval --score-only
    worldgen eval gen_gaps --video
    worldgen video ../wall_following_assigment/worlds/walls_one_sided.world \
        --traj ../../docker/output/improved_one_traj.csv \
        --cte ../../docker/output/improved_one.csv \
        --scan ../../docker/output/improved_one_scan.csv \
        --out improved_one.mp4 --title "improved walls_one_sided"
    worldgen preview ../wall_following_assigment/worlds/walls_one_sided.world
"""

import argparse
import datetime
import json
import random
import sys
from pathlib import Path

from . import (clutter as clutter_mod, course as course_mod, evaluate,
               geometry, presets, preview, sdf)


def _default_worlds_dir():
    """wall_following_assigment/worlds, found relative to this tool."""
    candidate = (Path(__file__).resolve().parents[2]
                 / "wall_following_assigment" / "worlds")
    return candidate if candidate.is_dir() else Path.cwd()


def _warn(problems):
    for p in problems:
        print(f"WARNING: {p}", file=sys.stderr)


def _write_world(walls, name, out_dir, comment, course_data=None,
                 obstacles=(), show_preview=True, dry_run=False,
                 quiet=False):
    """Validate, preview, and write <name>.world (+ <name>.course.json)."""
    _warn(geometry.validate_walls(walls))

    if show_preview:
        print(preview.render(walls, obstacles=obstacles))
        print()

    content = sdf.world_sdf(walls, comment=comment, obstacles=obstacles)
    out_path = Path(out_dir) / f"{name}.world"
    if dry_run:
        print(f"[dry-run] would write {len(walls)} walls"
              + (f" + {len(obstacles)} obstacles" if obstacles else "")
              + f" to {out_path}")
        return out_path

    Path(out_dir).mkdir(parents=True, exist_ok=True)
    out_path.write_text(content)
    if course_data is not None:
        course_mod.write_course(out_dir, course_data)
    print(f"wrote {len(walls)} walls"
          + (f" + {len(obstacles)} obstacles" if obstacles else "")
          + f" to {out_path}"
          + (f" (+ {name}.course.json)" if course_data is not None else ""))
    if not quiet:
        print("\nTo watch the wall follower run it (from the course "
              "docker/ dir):")
        print(f"  ./run.sh solution {out_path.stem}")
        print("Or inside an existing ROS 2 workspace, after colcon build:")
        print(f"  ros2 launch wall_following_assigment solution.launch.py "
              f"world:={out_path.name}")
    return out_path


def _add_common_args(p):
    p.add_argument("--two-sided", action="store_true",
                   help="build a corridor (walls on both sides of the course)")
    p.add_argument("--offset", type=float, default=2.0,
                   help="distance from course centerline to the left wall "
                        "(default 2.0, like the stock worlds)")
    p.add_argument("--width", type=float, default=None,
                   help="corridor width for --two-sided (default 2*offset)")
    p.add_argument("--clutter", type=int, default=0, metavar="N",
                   help="place N small boxes/cylinders against the followed "
                        "wall (deterministic per --seed; non-room courses)")
    p.add_argument("--mirror", action="store_true",
                   help="reflect the world about the y=2 line through the "
                        "spawn so the wall is on the robot's RIGHT (for "
                        "testing right-wall following, side_sign=+1); the "
                        "output name gets a _mirror suffix")
    p.add_argument("--name", default=None, help="output file name (no .world)")
    p.add_argument("--out", type=Path, default=None,
                   help="output directory (default: the package worlds/ dir)")
    p.add_argument("--no-preview", action="store_true",
                   help="skip the ASCII preview")
    p.add_argument("--dry-run", action="store_true",
                   help="preview + validate only, write nothing")


def _finalize(args, walls, obstacles, path_pts, closed, generator,
              base_name, comment, out_dir, quiet=False):
    """Mirror / sidecar / write, shared by generate and random."""
    name = base_name + ("_mirror" if args.mirror else "")
    if args.mirror:
        walls = geometry.mirror_walls(walls)
        obstacles = clutter_mod.mirror_obstacles(obstacles)
        path_pts = [geometry.mirror_point(p) for p in path_pts]
        comment += " mirror=True"
    course_data = course_mod.course_dict(
        name, path_pts, args.offset, args.two_sided, args.mirror, closed,
        generator)
    return _write_world(walls, name, out_dir, comment,
                        course_data=course_data, obstacles=obstacles,
                        show_preview=not args.no_preview,
                        dry_run=args.dry_run, quiet=quiet)


def _make_clutter(args, path_pts, seed):
    if not args.clutter:
        return []
    rng = random.Random(f"clutter-{seed}")
    obstacles, warnings = clutter_mod.place_clutter(
        path_pts, args.offset, args.clutter, rng)
    _warn(warnings)
    return obstacles


def _build_generate(args, stamp):
    """Walls/obstacles/sidecar pieces for the `generate` subcommand."""
    spec = presets.PRESETS[args.preset]
    walls = presets.preset_walls(args.preset, offset=args.offset,
                                 two_sided=args.two_sided, width=args.width,
                                 gap_width=args.gap_width,
                                 gap_count=args.gap_count)
    path_pts, closed = presets.preset_path(args.preset)
    segments = spec.get("segments")

    if segments:
        _warn(geometry.corner_warnings(segments))
        if args.two_sided:
            _warn(geometry.corridor_warnings(
                geometry.build_path(segments), args.offset, args.width))

    if args.clutter and spec.get("room"):
        raise SystemExit("error: --clutter needs a non-room preset")
    obstacles = _make_clutter(args, path_pts, args.seed)

    generator = {"preset": args.preset, "offset": args.offset,
                 "two_sided": args.two_sided, "width": args.width,
                 "clutter": args.clutter, "seed": args.seed}
    if spec.get("gaps"):
        generator["gap_width"] = args.gap_width
        generator["gap_count"] = args.gap_count
    base = args.name or ("gen_" + args.preset
                         + ("_2s" if args.two_sided else ""))
    comment = (f"generated by worldgen on {stamp}: preset={args.preset} "
               f"two_sided={args.two_sided} offset={args.offset}")
    return walls, obstacles, path_pts, closed, generator, base, comment


def _build_random(args, stamp):
    """Walls/obstacles/sidecar pieces for the `random` subcommand."""
    if args.closed:
        if args.two_sided:
            raise SystemExit("error: --closed loops are one-sided")
        walls, path_pts, params = presets.closed_loop(
            args.seed, offset=args.offset)
        closed = True
        generator = {"seed": args.seed, "closed": True,
                     "offset": args.offset, "clutter": args.clutter,
                     "loop": params}
        base = args.name or f"gen_random_s{args.seed}_closed"
        comment = (f"generated by worldgen on {stamp}: seed={args.seed} "
                   f"closed loop {params} offset={args.offset}")
    else:
        walls, segments = presets.random_walls(
            args.seed, n_segments=args.segments, offset=args.offset,
            two_sided=args.two_sided, width=args.width,
            min_len=args.min_len, max_len=args.max_len,
            min_turn=args.min_turn, max_turn=args.max_turn, arcs=args.arcs)
        path_pts, closed = geometry.build_path(segments), False
        _warn(geometry.corner_warnings(segments))
        if args.two_sided:
            _warn(geometry.corridor_warnings(path_pts, args.offset,
                                             args.width))
        generator = {"seed": args.seed, "segments": args.segments,
                     "min_len": args.min_len, "max_len": args.max_len,
                     "min_turn": args.min_turn, "max_turn": args.max_turn,
                     "arcs": args.arcs, "offset": args.offset,
                     "two_sided": args.two_sided, "width": args.width,
                     "clutter": args.clutter}
        base = args.name or (f"gen_random_s{args.seed}"
                             + ("_2s" if args.two_sided else ""))
        seg_str = ", ".join(
            f"({l:.1f} m, {'end' if t is None else f'{t:+.0f} deg'})"
            for l, t in segments)
        comment = (f"generated by worldgen on {stamp}: seed={args.seed} "
                   f"segments=[{seg_str}] two_sided={args.two_sided} "
                   f"offset={args.offset}")
    obstacles = _make_clutter(args, path_pts, args.seed)
    return walls, obstacles, path_pts, closed, generator, base, comment


# ------------------------------------------------------------------- suite

def _cmd_suite(args):
    out_dir = args.out or _default_worlds_dir()
    if not args.dry_run:
        Path(out_dir).mkdir(parents=True, exist_ok=True)
    if args.count is not None:
        seeds = list(range(1, args.count + 1))
    else:
        seeds = [int(s) for s in args.seeds.split(",") if s.strip()]
    stamp = datetime.date.today().isoformat()
    entries = []

    def emit(walls, obstacles, path_pts, closed, generator, name, comment):
        _warn(geometry.validate_walls(walls))
        course_data = course_mod.course_dict(
            name, path_pts, 2.0, False, False, closed, generator)
        dur = course_mod.suggested_duration(path_pts, closed)
        if args.dry_run:
            print(f"[dry-run] suite: {name}.world ({len(walls)} walls, "
                  f"{dur}s)")
        else:
            Path(out_dir, f"{name}.world").write_text(
                sdf.world_sdf(walls, comment=comment, obstacles=obstacles))
            course_mod.write_course(out_dir, course_data)
            print(f"suite: wrote {name}.world ({len(walls)} walls, "
                  f"suggested {dur}s)")
        entries.append({"name": name, "world": f"{name}.world",
                        "course": f"{name}.course.json", "duration_s": dur})

    for preset in sorted(presets.PRESETS):
        walls = presets.preset_walls(preset)
        path_pts, closed = presets.preset_path(preset)
        generator = {"preset": preset, "offset": 2.0, "two_sided": False,
                     "width": None, "clutter": 0, "seed": 0}
        emit(walls, [], path_pts, closed, generator, f"gen_{preset}",
             f"generated by worldgen suite on {stamp}: preset={preset}")

    for seed in seeds:
        walls, segments = presets.random_walls(seed)
        path_pts = geometry.build_path(segments)
        generator = {"seed": seed, "segments": 6, "min_len": 6.0,
                     "max_len": 13.0, "min_turn": 30.0, "max_turn": 90.0,
                     "arcs": False, "offset": 2.0, "two_sided": False,
                     "width": None, "clutter": 0}
        emit(walls, [], path_pts, False, generator, f"gen_random_s{seed}",
             f"generated by worldgen suite on {stamp}: seed={seed}")

    manifest = {"schema_version": 1,
                "generated": datetime.datetime.now().isoformat(
                    timespec="seconds"),
                "worlds": entries}
    mpath = Path(out_dir) / "suite_manifest.json"
    if args.dry_run:
        print(f"[dry-run] would write manifest with {len(entries)} worlds "
              f"to {mpath}")
    else:
        mpath.write_text(json.dumps(manifest, indent=2) + "\n")
        print(f"\nwrote {mpath} ({len(entries)} worlds)")
        print("run the battery with:  uv run worldgen eval")
    return 0


# -------------------------------------------------------------------- eval

def _cmd_eval(args):
    worlds_dir = args.worlds_dir or _default_worlds_dir()
    docker_dir = args.docker_dir
    if docker_dir is None and not args.score_only:
        docker_dir = evaluate.find_docker_dir()
    output_dir = args.output_dir
    if output_dir is None:
        output_dir = (docker_dir or evaluate.find_docker_dir()) / "output"

    if args.worlds:
        worlds = [(w[:-6] if w.endswith(".world") else w, args.secs)
                  for w in args.worlds]
    else:
        mpath = args.manifest or (Path(worlds_dir) / "suite_manifest.json")
        if not Path(mpath).is_file():
            print(f"error: no worlds given and no manifest at {mpath} "
                  "(run `worldgen suite` first)", file=sys.stderr)
            return 1
        with open(mpath) as f:
            manifest = json.load(f)
        worlds = [(e["name"], args.secs or e.get("duration_s"))
                  for e in manifest["worlds"]]

    results = evaluate.evaluate(
        worlds, worlds_dir, output_dir, docker_dir=docker_dir,
        score_only=args.score_only, default_secs=args.secs or 90,
        make_video=args.video)

    print()
    print(evaluate.format_table(results))
    report = args.report or (Path(output_dir) / "eval_report.md")
    evaluate.write_report(results, report)
    print(f"\nreport written to {report}")
    return 1 if any(m["verdict"] == "FAIL" for m in results) else 0


# -------------------------------------------------------------------- main

def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="worldgen",
        description="Generate Gazebo worlds for the HW1 wall follower.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("Examples:")[1] if "Examples:" in __doc__ else "")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="list the available presets")

    gen = sub.add_parser("generate", help="generate a world from a preset")
    gen.add_argument("preset", choices=sorted(presets.PRESETS))
    gen.add_argument("--seed", type=int, default=0,
                     help="seed for clutter placement (default 0)")
    gen.add_argument("--gap-width", type=float, default=presets.GAP_WIDTH,
                     help="gap width in metres for the 'gaps' preset "
                          f"(default {presets.GAP_WIDTH})")
    gen.add_argument("--gap-count", type=int, default=presets.GAP_COUNT,
                     choices=(1, 2),
                     help="number of gaps for the 'gaps' preset (default "
                          f"{presets.GAP_COUNT})")
    _add_common_args(gen)

    rnd = sub.add_parser("random", help="generate a random (seeded) course")
    rnd.add_argument("--seed", type=int, default=0)
    rnd.add_argument("--segments", type=int, default=6,
                     help="number of course segments (default 6)")
    rnd.add_argument("--min-turn", type=float, default=30.0)
    rnd.add_argument("--max-turn", type=float, default=90.0,
                     help="turn magnitudes in degrees (default 30..90; up "
                          "to 120 is supported, sharper inside corners get "
                          "a kinematic warning)")
    rnd.add_argument("--min-len", type=float, default=6.0)
    rnd.add_argument("--max-len", type=float, default=13.0,
                     help="segment lengths in metres (default 6..13)")
    rnd.add_argument("--arcs", action="store_true",
                     help="replace ~half the corners with constant-radius "
                          "6-8 m arcs")
    rnd.add_argument("--closed", action="store_true",
                     help="generate a CLOSED loop course (randomized "
                          "rounded rectangle; the robot can follow it "
                          "forever; ignores --segments/--*-len/--*-turn)")
    _add_common_args(rnd)

    prv = sub.add_parser("preview",
                         help="ASCII-preview any existing .world file")
    prv.add_argument("world_file", type=Path)

    ste = sub.add_parser(
        "suite", help="generate the evaluation battery of worlds + manifest")
    ste.add_argument("--seeds", default="1,2,3",
                     help="comma-separated random seeds (default 1,2,3)")
    ste.add_argument("--count", type=int, default=None,
                     help="use seeds 1..N instead of --seeds")
    ste.add_argument("--out", type=Path, default=None,
                     help="output directory (default: the package worlds/ "
                          "dir)")
    ste.add_argument("--dry-run", action="store_true")

    evl = sub.add_parser(
        "eval", help="record + score wall-follower runs (see docker/run.sh)")
    evl.add_argument("worlds", nargs="*",
                     help="world names (default: every world in the suite "
                          "manifest)")
    evl.add_argument("--manifest", type=Path, default=None,
                     help="suite manifest (default: worlds dir / "
                          "suite_manifest.json)")
    evl.add_argument("--secs", type=int, default=None,
                     help="recording duration override (default: manifest "
                          "suggestion, else 90)")
    evl.add_argument("--score-only", action="store_true",
                     help="skip running the sim; rescore existing CSVs")
    evl.add_argument("--worlds-dir", type=Path, default=None,
                     help="where the .course.json sidecars live")
    evl.add_argument("--output-dir", type=Path, default=None,
                     help="where the recorded CSVs live (default "
                          "docker/output)")
    evl.add_argument("--docker-dir", type=Path, default=None,
                     help="directory containing run.sh (default: auto)")
    evl.add_argument("--report", type=Path, default=None,
                     help="markdown report path (default "
                          "<output-dir>/eval_report.md)")
    evl.add_argument("--video", action="store_true",
                     help="also render a top-down clip per world (needs "
                          "matplotlib + ffmpeg)")

    vid = sub.add_parser(
        "video", help="render a top-down clip for one recorded run")
    vid.add_argument("world_file", type=Path, nargs="?", default=None,
                     help="optional .world (the wall map is built from lidar, "
                          "so this is not required)")
    vid.add_argument("--traj", type=Path, required=True,
                     help="trajectory CSV (t,x,y[,yaw])")
    vid.add_argument("--cte", type=Path, default=None,
                     help="cross-track-error CSV (t,cte)")
    vid.add_argument("--scan", type=Path, default=None,
                     help="lidar CSV (t,angle_min,angle_increment,ranges...)")
    vid.add_argument("--out", type=Path, required=True,
                     help="output .mp4 (or .gif if ffmpeg is missing)")
    vid.add_argument("--title", default="")
    vid.add_argument("--fps", type=int, default=20)
    vid.add_argument("--seconds", type=float, default=30.0,
                     help="target clip length (trajectory is resampled)")

    args = parser.parse_args(argv)

    if args.cmd == "list":
        for name in sorted(presets.PRESETS):
            print(f"  {name:<12} {presets.PRESETS[name]['doc']}")
        return 0

    if args.cmd == "preview":
        walls = preview.parse_world(args.world_file)
        if not walls:
            print(f"no wall boxes found in {args.world_file}",
                  file=sys.stderr)
            return 1
        print(preview.render(walls))
        return 0

    if args.cmd == "suite":
        return _cmd_suite(args)

    if args.cmd == "eval":
        return _cmd_eval(args)

    if args.cmd == "video":
        from . import render
        out = render.render_run(
            args.traj, args.out, cte_csv=args.cte, scan_csv=args.scan,
            world_path=args.world_file, title=args.title, fps=args.fps,
            seconds=args.seconds)
        print(f"wrote {out}")
        return 0

    out_dir = args.out or _default_worlds_dir()
    stamp = datetime.date.today().isoformat()

    if args.cmd == "generate":
        pieces = _build_generate(args, stamp)
    else:  # random
        pieces = _build_random(args, stamp)

    _finalize(args, *pieces, out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
