import numpy as np

from mics.attacker import AttackerState
from mics.cue_degrader import CueDegrader


def truth(tid=1, pos=(0, 0, 0), vel=(10, 0, 0)):
    return AttackerState(tid, np.array(pos, float), np.array(vel, float), True)


def test_latency_delays_release():
    cd = CueDegrader(pos_sigma_m=0, dropout_pct=0, latency_ms=300, vel_dropout_pct=0,
                     rng=np.random.default_rng(0))
    cd.ingest(0.0, [truth()])
    # before latency window elapses -> nothing
    assert cd.release(0.1) == []
    assert cd.release(0.2) == []
    # after 0.3 s -> released
    out = cd.release(0.35)
    assert len(out) == 1
    assert out[0].target_id == 1


def test_dropout_drops_updates():
    cd = CueDegrader(pos_sigma_m=0, dropout_pct=100, latency_ms=0, vel_dropout_pct=0,
                     rng=np.random.default_rng(0))
    cd.ingest(0.0, [truth()])
    assert cd.release(0.0) == []  # everything dropped


def test_position_noise_applied():
    cd = CueDegrader(pos_sigma_m=20, dropout_pct=0, latency_ms=0, vel_dropout_pct=0,
                     rng=np.random.default_rng(1))
    cd.ingest(0.0, [truth(pos=(100, 0, 0))])
    out = cd.release(0.0)
    assert len(out) == 1
    # noise should perturb position away from exact truth
    assert not np.allclose(out[0].position, np.array([100.0, 0, 0]))
    # covariance reflects configured sigma
    assert abs(out[0].position_covariance[0, 0] - 400.0) < 1e-6


def test_velocity_dropout_marks_unset():
    cd = CueDegrader(pos_sigma_m=0, dropout_pct=0, latency_ms=0, vel_dropout_pct=100,
                     rng=np.random.default_rng(0))
    cd.ingest(0.0, [truth(vel=(15, 0, 0))])
    out = cd.release(0.0)
    assert len(out) == 1
    assert out[0].has_velocity is False
    assert np.allclose(out[0].velocity, 0)


def test_dead_targets_not_emitted():
    cd = CueDegrader(pos_sigma_m=0, dropout_pct=0, latency_ms=0,
                     rng=np.random.default_rng(0))
    st = truth()
    st.alive = False
    cd.ingest(0.0, [st])
    assert cd.release(0.0) == []
