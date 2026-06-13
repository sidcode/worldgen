"""Scoring unit tests with synthetic recordings: a clean run, an oscillating
run, and a stalled run.  No simulation involved -- these exercise the metric
math in evaluate.score_world against hand-built CSV fixtures."""

from worldgen import evaluate


# A 30 m straight course centerline along +x at y=2 (the spawn line).
STRAIGHT_PATH = [[-7.0, 2.0], [23.0, 2.0]]


def _course(path=STRAIGHT_PATH, closed=False):
    return {"path": path, "closed": closed, "desired_distance": 1.0}


def _write(tmp_path, name, cte_lines, traj_rows):
    """Write <name>.csv (ros2-echo style) and <name>_traj.csv fixtures."""
    cte = "\n".join(f"data: {v}\n---" for v in cte_lines)
    (tmp_path / f"{name}.csv").write_text(cte + "\n")
    traj = "t,x,y\n" + "\n".join(f"{t:.3f},{x:.4f},{y:.4f}"
                                 for t, x, y in traj_rows)
    (tmp_path / f"{name}_traj.csv").write_text(traj + "\n")


def test_perfect_run_passes(tmp_path):
    """Near-zero cte, robot drives the full course -> PASS."""
    cte = [0.02, -0.03, 0.01, 0.0, -0.02] * 20
    # advance from x=-7 to x=23 over 30 s at ~1 m/s, on the centerline
    traj = [(t, -7.0 + t, 2.0) for t in range(0, 31)]
    _write(tmp_path, "perfect", cte, traj)
    m = evaluate.score_world("perfect", _course(), tmp_path)
    assert m["verdict"] == "PASS"
    assert m["mean_abs_cte"] < 0.05
    assert m["completion_pct"] > 95
    assert not m["stalled"]


def test_oscillating_run_not_pass(tmp_path):
    """Large oscillating cte inflates the mean above the PASS threshold."""
    cte = [0.6, -0.6, 0.55, -0.65, 0.6] * 20
    traj = [(t, -7.0 + t, 2.0) for t in range(0, 31)]
    _write(tmp_path, "osc", cte, traj)
    m = evaluate.score_world("osc", _course(), tmp_path)
    assert m["mean_abs_cte"] > evaluate.PASS_MEAN_CTE
    assert m["verdict"] in ("WARN", "FAIL")
    assert m["max_abs_cte"] >= 0.6


def test_stalled_run_fails(tmp_path):
    """Robot advances then freezes mid-course -> stall detected -> FAIL."""
    cte = [0.05] * 50
    # drive to x=3 over 10 s, then sit there for 15 s (well short of x=23)
    traj = [(float(t), -7.0 + t, 2.0) for t in range(0, 11)]
    traj += [(10.0 + t, 3.0, 2.0) for t in range(1, 16)]
    _write(tmp_path, "stall", cte, traj)
    m = evaluate.score_world("stall", _course(), tmp_path)
    assert m["stalled"] is True
    assert m["verdict"] == "FAIL"


def test_missing_data_fails(tmp_path):
    m = evaluate.score_world("nope", _course(), tmp_path)
    assert m["verdict"] == "FAIL"
    assert m["error"]


def test_cte_band_and_rms(tmp_path):
    """Half the samples inside +-0.2 m -> pct_in_band == 50."""
    cte = [0.1, 0.1, 0.5, 0.5]            # 2 of 4 within the band
    traj = [(t, -7.0 + t, 2.0) for t in range(0, 31)]
    _write(tmp_path, "band", cte, traj)
    m = evaluate.score_world("band", _course(), tmp_path)
    assert m["pct_in_band"] == 50.0
    assert m["max_abs_cte"] == 0.5
