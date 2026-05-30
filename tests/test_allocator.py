import numpy as np

from mics.allocator import Allocator, AllocatorConfig
from mics.msgs import DroneState, DroneStatus, TargetTrack, TrackSource


def status(did, pos, state=DroneState.IDLE, target=0, batt=100.0):
    return DroneStatus(stamp=0.0, drone_id=did, position=np.array(pos, float),
                       state=state, current_target=target, battery_pct=batt)


def track(tid, pos, vel=None):
    return TargetTrack(stamp=0.0, target_id=tid, position=np.array(pos, float),
                       velocity=np.array(vel if vel else [0, 0, 0], float),
                       has_velocity=vel is not None)


def test_assigns_nearest_drone():
    al = Allocator(AllocatorConfig(algorithm="hungarian"))
    al.update_roster([status(1, [0, 0, 0]), status(2, [100, 0, 0])])
    al.allocate(0.0, [track(7, [10, 0, 0])])
    # drone 1 is closest -> should get the target
    assert al.assignment_for(1).target_id == 7
    assert al.assignment_for(2) is None or al.assignment_for(2).target_id == 0


def test_one_target_one_drone_no_double_assign():
    al = Allocator(AllocatorConfig(algorithm="hungarian", redundancy=1))
    al.update_roster([status(i, [i * 10, 0, 0]) for i in range(1, 5)])
    al.allocate(0.0, [track(1, [5, 0, 0])])
    assigned = [d for d in range(1, 5)
                if al.assignment_for(d) and al.assignment_for(d).target_id == 1]
    assert len(assigned) == 1


def test_does_not_pile_onto_busy_serviced_target():
    al = Allocator(AllocatorConfig(algorithm="hungarian", redundancy=1))
    # drone 1 already committed (MIDCOURSE) to target 1
    al.update_roster([
        status(1, [4, 0, 0], state=DroneState.MIDCOURSE, target=1),
        status(2, [6, 0, 0]),
    ])
    al._assignments[1] = __import__("mics.msgs", fromlist=["Assignment"]).Assignment(
        stamp=0.0, drone_id=1, target_id=1)
    al.allocate(0.0, [track(1, [5, 0, 0])])
    # idle drone 2 must NOT be assigned to the already-serviced target
    assert al.assignment_for(2) is None or al.assignment_for(2).target_id == 0


def test_redundancy_two_assigns_two():
    al = Allocator(AllocatorConfig(algorithm="hungarian", redundancy=2))
    al.update_roster([status(i, [i * 10, 0, 0]) for i in range(1, 5)])
    al.allocate(0.0, [track(1, [5, 0, 0])])
    assigned = [d for d in range(1, 5)
                if al.assignment_for(d) and al.assignment_for(d).target_id == 1]
    assert len(assigned) == 2


def test_reassign_on_failed_drops_assignment():
    al = Allocator(AllocatorConfig(algorithm="hungarian"))
    al.update_roster([status(1, [0, 0, 0]), status(2, [50, 0, 0])])
    al.allocate(0.0, [track(1, [5, 0, 0])])
    assert al.assignment_for(1).target_id == 1
    # drone 1 fails -> its assignment must be dropped, target re-coverable
    al.update_roster([status(1, [5, 0, 0], state=DroneState.FAILED, target=1),
                      status(2, [50, 0, 0])])
    al.allocate(1.0, [track(1, [5, 0, 0])])
    assert al.assignment_for(1).target_id == 0
    # drone 2 should now pick it up
    assert al.assignment_for(2).target_id == 1


def test_two_targets_two_drones_distinct():
    al = Allocator(AllocatorConfig(algorithm="hungarian"))
    al.update_roster([status(1, [0, 0, 0]), status(2, [100, 0, 0])])
    al.allocate(0.0, [track(1, [5, 0, 0]), track(2, [95, 0, 0])])
    assert al.assignment_for(1).target_id == 1
    assert al.assignment_for(2).target_id == 2


def test_greedy_also_covers_targets():
    al = Allocator(AllocatorConfig(algorithm="greedy"))
    al.update_roster([status(1, [0, 0, 0]), status(2, [100, 0, 0])])
    al.allocate(0.0, [track(1, [5, 0, 0]), track(2, [95, 0, 0])])
    assert al.assignment_for(1).target_id == 1
    assert al.assignment_for(2).target_id == 2


def test_paused_allocator_makes_no_assignments():
    al = Allocator(AllocatorConfig())
    al.paused = True
    al.update_roster([status(1, [0, 0, 0])])
    al.allocate(0.0, [track(1, [5, 0, 0])])
    assert al.assignment_for(1) is None
