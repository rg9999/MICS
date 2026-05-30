import numpy as np

from mics.safety import Safety, SafetyConfig


def make(**kw):
    return Safety(SafetyConfig(**kw))


def test_geofence_breach_forces_rtl():
    s = make(geofence_radius_m=100.0)
    assert s.geofence_ok(np.array([50.0, 0, 0]))
    assert not s.geofence_ok(np.array([150.0, 0, 0]))
    assert s.must_rtl(np.array([150.0, 0, 0]), battery_pct=100)


def test_low_battery_forces_rtl():
    s = make(low_battery_pct=15.0)
    assert not s.must_rtl(np.array([0.0, 0, 0]), battery_pct=50)
    assert s.must_rtl(np.array([0.0, 0, 0]), battery_pct=10)


def test_kill_switch_forces_rtl_and_blocks_capture():
    s = make()
    s.kill()
    assert s.must_rtl(np.array([0.0, 0, 0]), battery_pct=100)
    assert not s.capture_permitted(np.array([0.0, 0, 0]), True, [])


def test_capture_requires_geometry():
    s = make()
    assert s.capture_permitted(np.array([0.0, 0, 0]), geometry_ok=True, teammate_positions=[])
    assert not s.capture_permitted(np.array([0.0, 0, 0]), geometry_ok=False, teammate_positions=[])


def test_capture_blocked_near_teammate():
    s = make(capture_min_teammate_sep_m=10.0)
    near = [np.array([3.0, 0, 0])]
    far = [np.array([50.0, 0, 0])]
    assert not s.capture_permitted(np.zeros(3), True, near)
    assert s.capture_permitted(np.zeros(3), True, far)


def test_disarm_blocks_capture():
    s = make()
    s.disarm()
    assert not s.capture_permitted(np.zeros(3), True, [])
