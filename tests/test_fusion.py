import numpy as np

from mics.fusion import FusionEKF
from mics.sensors import CameraMeas, LidarMeas, RadarMeas


def test_lidar_seed_and_converge():
    ekf = FusionEKF()
    ekf.set_ownship(np.zeros(3), np.zeros(3))
    target = np.array([10.0, 2.0, 1.0])
    for k in range(20):
        ekf.predict(0.05)
        ekf.update_lidar(LidarMeas(stamp=k * 0.05, position=target.copy()))
    est = ekf.estimate(1.0)
    assert est.valid
    assert np.linalg.norm(est.position - target) < 1.0
    assert est.quality > 0.8


def test_radar_seeds_when_uninitialised():
    ekf = FusionEKF()
    ekf.set_ownship(np.zeros(3), np.zeros(3))
    assert not ekf.initialized
    ekf.update_radar(RadarMeas(stamp=0.0, range_m=50.0, range_rate=-10.0, azimuth=0.0))
    assert ekf.initialized


def test_quality_zero_after_timeout():
    ekf = FusionEKF(lost_timeout=1.0)
    ekf.set_ownship(np.zeros(3), np.zeros(3))
    ekf.update_lidar(LidarMeas(stamp=0.0, position=np.array([5.0, 0, 0])))
    assert ekf.quality(0.1) > 0.0
    assert ekf.quality(2.0) == 0.0  # no updates for >timeout


def test_camera_does_not_initialise_alone():
    ekf = FusionEKF()
    ekf.set_ownship(np.zeros(3), np.zeros(3))
    ok = ekf.update_camera(CameraMeas(stamp=0.0, azimuth=0.1, elevation=0.0,
                                      class_confidence=0.9))
    assert ok is False
    assert not ekf.initialized


def test_gating_rejects_outlier():
    ekf = FusionEKF(gate_chi2=9.0)
    ekf.set_ownship(np.zeros(3), np.zeros(3))
    # establish a tight track at ~[10,0,0]
    for k in range(30):
        ekf.predict(0.05)
        ekf.update_lidar(LidarMeas(stamp=k * 0.05, position=np.array([10.0, 0, 0])))
    # a wild outlier should be gated out (returns False)
    rejected = ekf.update_lidar(LidarMeas(stamp=2.0, position=np.array([500.0, 500, 0])))
    assert rejected is False


def test_radar_estimates_closing_velocity_sign():
    ekf = FusionEKF()
    ekf.set_ownship(np.zeros(3), np.zeros(3))
    # target at 50m on +x moving toward us: lidar seeds position, radar adds rate
    pos = np.array([50.0, 0, 0])
    vel = np.array([-12.0, 0, 0])
    for k in range(40):
        t = k * 0.05
        ekf.predict(0.05)
        ekf.update_lidar(LidarMeas(stamp=t, position=pos.copy()))
        ekf.update_radar(RadarMeas(stamp=t, range_m=50.0, range_rate=-12.0, azimuth=0.0))
        pos = pos + vel * 0.05
        ekf.set_ownship(np.zeros(3), np.zeros(3))
    est = ekf.estimate(2.0)
    # x-velocity should be clearly negative (approaching)
    assert est.velocity[0] < -3.0
