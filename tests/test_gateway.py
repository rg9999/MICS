"""viewer-gateway pure-logic tests (ROS-free, host-runnable).

Covers the choke-point invariants that the browser depends on: geodesy
round-trip, derived-field math, wire-schema shape + JSON-encodability, the
log channel, the scenario catalog, and the recorder->replay byte-identity that
makes the single live/replay code path possible.
"""

import json
import math

import numpy as np
import pytest

from viewer_gateway.aggregator import (Aggregator, GwAssignment, GwCapture,
                                       GwDrone, GwTrack)
from viewer_gateway.catalog import list_scenarios
from viewer_gateway.config import GatewayConfig, RecordingConfig, load_config
from viewer_gateway.geodesy import Datum, Transformer
from viewer_gateway.logbus import LogBus, level_name
from viewer_gateway.recorder import Recorder
from viewer_gateway import replayer as P


# --- geodesy ---------------------------------------------------------------

def test_geodesy_round_trip():
    tf = Transformer(Datum(lat=32.0853, lon=34.7818, alt=30.0))
    for enu in ([0, 0, 0], [1000, -2500, 300], [-5000, 4000, -50]):
        v = np.array(enu, dtype=float)
        lat, lon, alt = tf.enu_to_geodetic(v)
        back = tf.geodetic_to_enu(lat, lon, alt)
        assert np.allclose(v, back, atol=1e-6), (enu, back)


def test_enu_to_lonlatalt_order():
    tf = Transformer(Datum(lat=10.0, lon=20.0, alt=5.0))
    lonlatalt = tf.enu_to_lonlatalt(np.zeros(3))
    assert pytest.approx(lonlatalt[0], abs=1e-6) == 20.0  # lon first (Cesium)
    assert pytest.approx(lonlatalt[1], abs=1e-6) == 10.0
    assert pytest.approx(lonlatalt[2], abs=1e-4) == 5.0


# --- aggregator derived fields ---------------------------------------------

def _agg():
    return Aggregator(Transformer(Datum(lat=32.0, lon=34.0, alt=0.0)))


def test_range_and_eta():
    a = _agg()
    a.ingest_drone(GwDrone(drone_id=1, enu=np.zeros(3),
                           vel_enu=np.array([100.0, 0, 0]), state=2))
    a.ingest_track(GwTrack(target_id=5, enu=np.array([1000.0, 0, 0]),
                           vel_enu=np.zeros(3)))
    a.ingest_assignment(GwAssignment(drone_id=1, target_id=5, role=0))
    snap = a.build_snapshot()
    d = snap["dronesDerived"][0]
    assert pytest.approx(d["rangeToTarget"], rel=1e-6) == 1000.0
    assert pytest.approx(d["etaToInterceptS"], rel=1e-6) == 10.0  # 1000 / 100
    assert d["allocatedTarget"] == 5


def test_eta_none_when_not_closing():
    a = _agg()
    a.ingest_drone(GwDrone(drone_id=1, enu=np.zeros(3),
                           vel_enu=np.array([-10.0, 0, 0]), state=2))  # receding
    a.ingest_track(GwTrack(target_id=5, enu=np.array([1000.0, 0, 0])))
    a.ingest_assignment(GwAssignment(drone_id=1, target_id=5, role=0))
    d = a.build_snapshot()["dronesDerived"][0]
    assert d["etaToInterceptS"] is None


def test_allocation_kinds():
    a = _agg()
    # engaged: active-state primary drone assigned to target
    a.ingest_drone(GwDrone(drone_id=1, enu=np.zeros(3),
                           vel_enu=np.zeros(3), state=3))  # ACQUIRING (active)
    a.ingest_track(GwTrack(target_id=5, enu=np.array([10.0, 0, 0])))
    a.ingest_assignment(GwAssignment(drone_id=1, target_id=5, role=0))
    # unengaged target with no assignment
    a.ingest_track(GwTrack(target_id=6, enu=np.array([20.0, 0, 0])))
    by_id = {t["targetId"]: t["allocation"] for t in a.build_snapshot()["targetsDerived"]}
    assert by_id[5] == {"kind": "ENGAGED", "byDrone": 1}
    assert by_id[6] == {"kind": "UNENGAGED"}

    a.ingest_capture(GwCapture(drone_id=1, target_id=5, result=1,
                               enu=np.array([10.0, 0, 0])))
    by_id = {t["targetId"]: t["allocation"] for t in a.build_snapshot()["targetsDerived"]}
    assert by_id[5]["kind"] == "CAPTURED"


def test_events_drained_each_frame():
    a = _agg()
    a.ingest_capture(GwCapture(drone_id=1, target_id=5, result=0,
                               enu=np.zeros(3), stamp=1.0))
    assert len(a.build_snapshot()["events"]) == 1
    assert a.build_snapshot()["events"] == []  # drained


def test_snapshot_is_json_encodable_and_shaped():
    a = _agg()
    a.ingest_drone(GwDrone(drone_id=1, enu=np.array([1.0, 2, 3]),
                           vel_enu=np.array([4.0, 5, 6]), state=1))
    a.ingest_track(GwTrack(target_id=5, enu=np.array([7.0, 8, 9]),
                           vel_enu=np.array([0.0, 0, 0])))
    snap = a.build_snapshot()
    s = json.dumps(snap)  # must not raise (no numpy scalars leaking through)
    again = json.loads(s)
    for k in ("type", "stamp", "datum", "drones", "tracks", "assignments",
              "estimates", "events", "dronesDerived", "targetsDerived"):
        assert k in again
    assert again["type"] == "frame"
    drone = again["drones"][0]
    assert len(drone["position"]) == 3 and len(drone["enu"]) == 3
    track = again["tracks"][0]
    assert len(track["covEnu"]) == 9
    assert track["posSigmaM"] == pytest.approx(math.sqrt(1.0))  # trace(I)/3 = 1


# --- log channel -----------------------------------------------------------

def test_level_name_buckets():
    assert level_name(20) == "INFO"
    assert level_name(35) == "WARN"   # rounds down
    assert level_name(99) == "FATAL"
    assert level_name(5) == "DEBUG"


def test_logbus_filters_and_batches():
    from viewer_gateway.aggregator import GwLog
    bus = LogBus(min_level=20, ring_rows=10)
    bus.ingest(GwLog(stamp=1.0, level=10, source="x", msg="debug"))  # dropped
    bus.ingest(GwLog(stamp=2.0, level=30, source="x", msg="warn"))
    batch = bus.drain_batch()
    assert batch["type"] == "logs"
    assert len(batch["records"]) == 1
    assert batch["records"][0]["level"] == "WARN"
    assert bus.drain_batch() is None  # drained


# --- scenario catalog ------------------------------------------------------

def test_list_scenarios(tmp_path):
    (tmp_path / "happy_path.yaml").write_text(
        "name: Happy Path\ndescription: demo\n"
        "defenders:\n  count: 3\n"
        "attackers:\n  - id: 1\n  - id: 2\n"
        "target_source: internal\nsensor_mode: ideal\n")
    out = list_scenarios(str(tmp_path))
    assert len(out) == 1
    s = out[0]
    assert s["scenarioId"] == "happy_path"
    assert s["defenderCount"] == 3 and s["attackerCount"] == 2
    assert "path" not in s  # server-side path never exposed to clients


def test_list_scenarios_missing_dir():
    assert list_scenarios("/no/such/dir") == []


# --- recorder -> replay round-trip -----------------------------------------

@pytest.mark.parametrize("compress", [True, False])
def test_recorder_replay_round_trip(tmp_path, compress):
    cfg = RecordingConfig(dir=str(tmp_path), compress=compress,
                          segment_rotate_seconds=3600, include_logs=True)
    rec = Recorder(cfg, datum={"lat": 32.0, "lon": 34.0, "alt": 30.0})
    sid = rec.start(scenario="demo")
    frames, logs = [], []
    for i in range(6):
        f = {"type": "frame", "stamp": float(i),
             "drones": [{"droneId": 1}], "tracks": [{"targetId": 9}]}
        frames.append(f)
        rec.write_state(f)
        lb = {"type": "logs", "records": [
            {"stamp": float(i) + 0.5, "level": "INFO", "source": "n",
             "msg": "m%d" % i, "file": "", "func": "", "line": 0}]}
        logs.append(lb)
        rec.write_logs(lb)
    rec.stop()

    session = tmp_path / sid
    man = P.load_manifest(session)
    assert man["scenario"] == "demo"
    assert man["droneCount"] == 1 and man["targetCount"] == 1
    assert man["stateSegments"] and man["logSegments"]

    assert list(P.iter_state(session)) == frames  # byte-identical reconstruction
    assert list(P.iter_logs(session)) == logs

    merged = list(P.iter_merged(session))
    stamps = [s for s, _ in merged]
    assert stamps == sorted(stamps)
    assert len(merged) == 12

    recs = P.list_recordings(str(tmp_path))
    assert [r["sessionId"] for r in recs] == [sid]


# --- config ----------------------------------------------------------------

def test_load_config_defaults():
    cfg = load_config(None)
    assert isinstance(cfg, GatewayConfig)
    assert cfg.ws_port == 8080
    assert cfg.snapshot_period == pytest.approx(1.0 / cfg.snapshot_rate_hz)


def test_load_config_yaml(tmp_path):
    p = tmp_path / "gw.yaml"
    p.write_text(
        "connection:\n  ws_port: 9090\n"
        "datum:\n  lat: 10.0\n  lon: 20.0\n  alt: 5.0\n"
        "performance:\n  snapshot_rate_hz: 50\n"
        "grids:\n  process_log:\n    min_level: WARN\n")
    cfg = load_config(str(p))
    assert cfg.ws_port == 9090
    assert cfg.datum.lat == 10.0 and cfg.datum.lon == 20.0
    assert cfg.snapshot_rate_hz == 50
    assert cfg.log_min_level == 30  # WARN
