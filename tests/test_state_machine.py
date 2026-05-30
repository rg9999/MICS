import numpy as np

from mics.attacker import Attacker
from mics.drone import Drone, DroneParams
from mics.msgs import Assignment, DroneState, TargetTrack, TrackSource
from mics.sensors import SensorSuite


def make_drone(pos=(0, 0, 50), **pkw):
    rng = np.random.default_rng(0)
    d = Drone(drone_id=1, position=np.array(pos, float),
              params=DroneParams(**pkw), rng=rng)
    d.sensors = SensorSuite(mode="ideal", rng=rng)
    return d


def cue(tid, pos, vel=(0, 0, 0)):
    return TargetTrack(stamp=0.0, target_id=tid, position=np.array(pos, float),
                       velocity=np.array(vel, float), source=TrackSource.INTERNAL_SIM)


def test_idle_until_assigned():
    d = make_drone()
    assert d.state == DroneState.IDLE
    d.step(0.0, 0.05, None, None, [])
    assert d.state == DroneState.IDLE


def test_assignment_moves_to_midcourse():
    d = make_drone()
    d.set_assignment(Assignment(stamp=0.0, drone_id=1, target_id=5))
    assert d.state == DroneState.ASSIGNED
    d.step(0.0, 0.05, cue(5, [100, 0, 50]), None, [])
    assert d.state == DroneState.MIDCOURSE


def test_unassign_returns_to_idle():
    d = make_drone()
    d.set_assignment(Assignment(stamp=0.0, drone_id=1, target_id=5))
    d.step(0.0, 0.05, cue(5, [100, 0, 50]), None, [])
    d.set_assignment(Assignment(stamp=0.0, drone_id=1, target_id=0))
    assert d.state == DroneState.IDLE
    assert d.assignment is None


def test_full_engagement_reaches_capture():
    """Closed loop: assign a drone to a slow nearby target with ideal sensors;
    it should progress through handoff to TERMINAL and CAPTURED."""
    d = make_drone(pos=(0, 0, 50), max_speed=30, cruise_speed=25,
                   r_capture_m=5.0, t_lock=0.2, q_handoff=0.5)
    atk = Attacker(target_id=5, profile="ingress", speed_mps=4.0,
                   position=np.array([120.0, 0, 50]), asset=np.array([0.0, 0, 50]))
    d.set_assignment(Assignment(stamp=0.0, drone_id=1, target_id=5))
    states_seen = set()
    t = 0.0
    for _ in range(2000):
        t += 0.05
        atk.step(0.05, [d.position])
        c = cue(5, atk.position + np.array([3.0, 1.0, 0.0]))  # slightly noisy cue
        d.step(t, 0.05, c, atk.state(), [])
        states_seen.add(d.state)
        if d.state in (DroneState.CAPTURED,):
            break
    assert DroneState.MIDCOURSE in states_seen
    assert DroneState.TERMINAL in states_seen
    assert d.state == DroneState.CAPTURED
    assert any(ev.result.name == "SUCCESS" for ev in d.capture_events) or d.state == DroneState.CAPTURED


def test_terminal_track_loss_fails():
    d = make_drone(t_lost=0.3, t_lock=0.1, q_handoff=0.5)
    # force into TERMINAL with a track seeded FAR away (unreachable within t_lost,
    # so capture can't trigger), then starve of measurements -> track lost -> FAILED.
    from mics.sensors import LidarMeas
    d.state = DroneState.TERMINAL
    d.assignment = Assignment(stamp=0.0, drone_id=1, target_id=5)
    d.fusion.set_ownship(d.position, d.velocity)
    d.fusion.update_lidar(LidarMeas(stamp=0.0, position=np.array([200.0, 0, 0])))
    t = 0.0
    for _ in range(40):
        t += 0.05
        d.step(t, 0.05, None, None, [])
        if d.state == DroneState.FAILED:
            break
    assert d.state == DroneState.FAILED
    assert getattr(d, "fail_reason", None) == "track_lost"


def test_safety_rtl_on_geofence_breach():
    d = make_drone(pos=(0, 0, 50))
    d.safety.cfg.geofence_radius_m = 50.0
    d.set_assignment(Assignment(stamp=0.0, drone_id=1, target_id=5))
    d.position = np.array([100.0, 0, 50])  # outside geofence
    d.step(0.0, 0.05, cue(5, [200, 0, 50]), None, [])
    assert d.state == DroneState.FAILED
