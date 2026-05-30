import numpy as np

from mics.attacker import Attacker


def test_ingress_moves_toward_asset():
    a = Attacker(target_id=1, profile="ingress", speed_mps=10.0,
                 position=np.array([-100.0, 0, 50]), asset=np.array([0.0, 0, 50]))
    d0 = np.linalg.norm(a.position - a.asset)
    for _ in range(50):
        a.step(0.1, [])
    d1 = np.linalg.norm(a.position - a.asset)
    assert d1 < d0


def test_waypoint_cycles():
    wps = [np.array([10.0, 0, 0]), np.array([10.0, 10, 0]), np.array([0.0, 10, 0])]
    a = Attacker(target_id=1, profile="waypoint", speed_mps=5.0,
                 position=np.array([0.0, 0, 0]), waypoints=wps)
    for _ in range(200):
        a.step(0.1, [])
    # it should have moved and still be alive within the waypoint box
    assert a.alive
    assert -5 < a.position[0] < 15


def test_evasive_jinks_when_pursued():
    rng = np.random.default_rng(0)
    a = Attacker(target_id=1, profile="evasive", speed_mps=15.0, r_evade_m=50.0,
                 position=np.array([0.0, 0, 50]), asset=np.array([300.0, 0, 50]),
                 rng=rng)
    # straight ingress heading first (no pursuer)
    for _ in range(10):
        a.step(0.1, [np.array([1000.0, 1000, 50])])
    straight_heading = a._heading.copy()
    # now a pursuer is close -> expect heading to deviate over time
    deviated = False
    for _ in range(40):
        a.step(0.1, [a.position + np.array([5.0, 0, 0])])
        if np.linalg.norm(a._heading - straight_heading) > 0.2:
            deviated = True
    assert deviated


def test_replay_follows_path():
    path = [np.array([float(i), 0, 0]) for i in range(20)]
    a = Attacker(target_id=1, profile="replay", position=np.array([0.0, 0, 0]),
                 replay_path=path)
    for _ in range(10):
        a.step(0.1, [])
    assert a.position[0] >= 5.0  # advanced along the recorded path


def test_kill_stops_motion():
    a = Attacker(target_id=1, profile="ingress", speed_mps=10.0,
                 position=np.array([0.0, 0, 0]), asset=np.array([100.0, 0, 0]))
    a.kill()
    p = a.position.copy()
    a.step(0.1, [])
    assert np.allclose(a.position, p)
