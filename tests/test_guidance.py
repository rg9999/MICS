import numpy as np

from mics.guidance import ProportionalNavigation


def test_command_capped_at_max_speed():
    pn = ProportionalNavigation(nav_constant=3.0, max_speed=20.0)
    sp = pn.command(np.zeros(3), np.zeros(3),
                    np.array([100.0, 0, 0]), np.zeros(3), 0.05)
    assert np.linalg.norm(sp) <= 20.0 + 1e-6


def test_heads_toward_stationary_target():
    pn = ProportionalNavigation(max_speed=20.0)
    sp = pn.command(np.zeros(3), np.zeros(3),
                    np.array([50.0, 0, 0]), np.zeros(3), 0.05)
    # velocity should point predominantly +x
    assert sp[0] > 15.0
    assert abs(sp[1]) < 1.0


def test_closing_speed_positive_when_approaching():
    pn = ProportionalNavigation()
    cs = pn.closing_speed(np.zeros(3), np.array([10.0, 0, 0]),
                          np.array([50.0, 0, 0]), np.zeros(3))
    assert cs > 0


def test_closing_speed_negative_when_separating():
    pn = ProportionalNavigation()
    cs = pn.closing_speed(np.zeros(3), np.array([-10.0, 0, 0]),
                          np.array([50.0, 0, 0]), np.zeros(3))
    assert cs < 0


def test_pn_intercepts_crossing_target():
    """Closed-loop: PN should drive miss distance to near zero on a crossing
    target moving perpendicular to initial LOS."""
    pn = ProportionalNavigation(nav_constant=4.0, max_speed=30.0)
    drone_p = np.zeros(3)
    drone_v = np.array([20.0, 0.0, 0.0])
    tgt_p = np.array([200.0, 60.0, 0.0])
    tgt_v = np.array([0.0, -8.0, 0.0])
    dt = 0.02
    min_range = 1e9
    for _ in range(2000):
        sp = pn.command(drone_p, drone_v, tgt_p, tgt_v, dt)
        drone_v = sp
        drone_p = drone_p + drone_v * dt
        tgt_p = tgt_p + tgt_v * dt
        min_range = min(min_range, float(np.linalg.norm(tgt_p - drone_p)))
    assert min_range < 5.0
