"""End-to-end scenario tests against PRD §9.2 scenarios + §9.3 metrics."""
from pathlib import Path

import pytest

from mics.config import load_scenario
from mics.sim import Simulation

SCN = Path(__file__).resolve().parent.parent / "scenarios"


def run(name, seed=None):
    cfg = load_scenario(SCN / name)
    if seed is not None:
        cfg.seed = seed
    return Simulation(cfg).run().summary()


def test_happy_path_captures():
    s = run("happy_path.yaml")
    assert s["captures"] == 1
    assert s["false_fires"] == 0
    assert s["handoffs"] >= 1


def test_evasive_single_commit_no_false_fire():
    s = run("evasive_reassign.yaml")
    assert s["captures"] == 1
    assert s["false_fires"] == 0
    # exactly one drone should commit to the lone target
    assert s["handoffs"] == 1


def test_allocation_stress_covers_all_targets():
    s = run("allocation_stress.yaml")
    assert s["n_targets"] == 3
    assert s["captures"] == 3
    assert s["false_fires"] == 0
    # PRD §9.3: inter-drone separation must stay above the safety bound
    assert s["min_inter_drone_separation_m"] is None or s["min_inter_drone_separation_m"] > 5.0


def test_zero_false_fire_across_seeds():
    """PRD §9.3: false-fire rate must be 0 in sim interlock tests."""
    for seed in range(8):
        s = run("allocation_stress.yaml", seed=seed)
        assert s["false_fires"] == 0


@pytest.mark.parametrize("seed", range(6))
def test_happy_path_robust_across_seeds(seed):
    s = run("happy_path.yaml", seed=seed)
    assert s["captures"] == 1
    assert s["false_fires"] == 0


def test_ghost_capture_scored_as_false_fire():
    """A declared SUCCESS far from the true target must be scored as a false
    fire, never a capture (PRD §9.3 truth-based scoring)."""
    import numpy as np

    from mics.msgs import CaptureEvent, CaptureResult
    cfg = load_scenario(SCN / "happy_path.yaml")
    sim = Simulation(cfg)
    # true target is at its start; inject a capture claim 1 km away
    bogus = CaptureEvent(stamp=1.0, drone_id=1, target_id=1,
                         result=CaptureResult.SUCCESS,
                         engagement_point=np.array([9999.0, 0, 0]))
    sim.drones[0].capture_events.append(bogus)
    sim._collect_metrics(1.0)
    assert sim.metrics.false_fires == 1
    assert sim.metrics.captures == 0
