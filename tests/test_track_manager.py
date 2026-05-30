import numpy as np

from mics.msgs import TargetTrack, TrackSource
from mics.track_manager import TrackManager


def trk(tid, pos, t=0.0):
    return TargetTrack(stamp=t, target_id=tid, position=np.array(pos, float),
                       source=TrackSource.INTERNAL_SIM)


def test_ingest_creates_track():
    tm = TrackManager()
    tm.ingest(0.0, [trk(1, [10, 0, 0])])
    assert tm.get(1) is not None
    assert len(tm.tracks()) == 1


def test_ttl_ages_out_stale_track():
    tm = TrackManager(ttl_s=1.0)
    tm.ingest(0.0, [trk(1, [10, 0, 0])])
    tm.step(0.5)
    assert tm.get(1) is not None
    tm.step(2.0)  # past TTL with no new update
    assert tm.get(1) is None


def test_smoothing_blends_updates():
    tm = TrackManager(smoothing=0.5)
    tm.ingest(0.0, [trk(1, [0, 0, 0])])
    tm.ingest(0.1, [trk(1, [10, 0, 0], t=0.1)])
    # fused x should be between 0 and 10
    x = tm.get(1).position[0]
    assert 0 < x < 10


def test_keeps_multiple_distinct_targets():
    tm = TrackManager()
    tm.ingest(0.0, [trk(1, [0, 0, 0]), trk(2, [100, 0, 0])])
    assert len(tm.tracks()) == 2
