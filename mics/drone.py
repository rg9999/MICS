"""Per-drone onboard autonomy — PRD §3.3, §6.1, §6.2.

Hosts the MICS state machine plus a simple kinematic flight model (the SITL
autopilot is abstracted away: we command a velocity setpoint and integrate it
with a first-order response). Ties together fusion, guidance, and safety.

State machine (FR-OB-1):
  IDLE -> ASSIGNED -> MIDCOURSE -> ACQUIRING -> TERMINAL -> CAPTURED | FAILED -> IDLE
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .fusion import FusionEKF
from .geometry import angle_between, clamp_norm, norm, unit
from .guidance import ProportionalNavigation
from .msgs import (
    Assignment,
    CaptureEvent,
    CaptureResult,
    DroneState,
    DroneStatus,
    TargetTrack,
)
from .safety import Safety, SafetyConfig
from .sensors import SensorSuite


@dataclass
class DroneParams:
    max_speed: float = 25.0
    accel_tau: float = 0.4          # velocity first-order time constant (s)
    cruise_speed: float = 20.0
    t_lock: float = 0.5             # s of quality>Q to declare handoff
    q_handoff: float = 0.55         # onboard quality threshold for handoff
    t_lost: float = 1.0             # s without a fresh measurement -> FAILED
    t_fresh: float = 0.3            # max measurement age to permit capture
    r_capture_m: float = 3.0        # capture range
    capture_cone_deg: float = 35.0  # aspect cone for capture geometry
    miss_distance_m: float = 60.0   # if range grows beyond this in TERMINAL -> miss risk
    battery_drain_per_s: float = 0.05
    nav_constant: float = 3.0


@dataclass
class Drone:
    drone_id: int
    position: np.ndarray
    params: DroneParams = field(default_factory=DroneParams)
    safety_cfg: SafetyConfig = field(default_factory=SafetyConfig)
    rng: np.random.Generator = None

    # runtime
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    state: DroneState = DroneState.IDLE
    assignment: Assignment = None
    battery_pct: float = 100.0
    home: np.ndarray = None

    fusion: FusionEKF = None
    guidance: ProportionalNavigation = None
    safety: Safety = None
    sensors: SensorSuite = None

    _lock_timer: float = 0.0
    _lost_timer: float = 0.0
    _last_est_pos: np.ndarray = None
    capture_events: list = field(default_factory=list)

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float)
        if self.home is None:
            self.home = self.position.copy()
        if self.rng is None:
            self.rng = np.random.default_rng()
        if self.fusion is None:
            self.fusion = FusionEKF()
        if self.guidance is None:
            self.guidance = ProportionalNavigation(
                nav_constant=self.params.nav_constant, max_speed=self.params.max_speed)
        if self.safety is None:
            self.safety = Safety(self.safety_cfg)
        if self.sensors is None:
            self.sensors = SensorSuite(rng=self.rng)

    # --- comms ------------------------------------------------------------

    def status(self, now: float) -> DroneStatus:
        return DroneStatus(
            stamp=now,
            drone_id=self.drone_id,
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            state=self.state,
            current_target=(self.assignment.target_id if self.assignment else 0),
            battery_pct=self.battery_pct,
            track_quality=self.fusion.quality(now),
        )

    def set_assignment(self, a: Assignment) -> None:
        if a.target_id == 0:
            self.assignment = None
            if self.state in (DroneState.ASSIGNED, DroneState.MIDCOURSE,
                              DroneState.ACQUIRING, DroneState.TERMINAL):
                self._to_idle()
            return
        # new or changed assignment
        if self.assignment is None or self.assignment.target_id != a.target_id:
            self.assignment = a
            self._reset_track()
            if self.state == DroneState.IDLE:
                self.state = DroneState.ASSIGNED

    # --- main step --------------------------------------------------------

    def step(self, now: float, dt: float, cue: TargetTrack | None,
             attacker_truth, teammates: list[DroneStatus]) -> None:
        """Advance one tick. `attacker_truth` is the assigned target's truth
        (AttackerState) or None; used only by sensor models."""
        self.battery_pct = max(0.0, self.battery_pct - self.params.battery_drain_per_s * dt)

        # Safety always first (FR-OB-8). RTL overrides everything.
        if self.state not in (DroneState.IDLE,) and self.safety.must_rtl(self.position, self.battery_pct):
            self._fail(now, "safety_rtl")

        if self.state == DroneState.IDLE:
            self._fly_to(self.home, dt, speed=self.params.cruise_speed)
        elif self.state == DroneState.ASSIGNED:
            self.state = DroneState.MIDCOURSE
            self._step_midcourse(now, dt, cue, attacker_truth)
        elif self.state == DroneState.MIDCOURSE:
            self._step_midcourse(now, dt, cue, attacker_truth)
        elif self.state == DroneState.ACQUIRING:
            self._step_acquiring(now, dt, attacker_truth)
        elif self.state == DroneState.TERMINAL:
            self._step_terminal(now, dt, attacker_truth, teammates)
        elif self.state in (DroneState.CAPTURED, DroneState.FAILED):
            # transient: return home then go idle
            done = self._fly_to(self.home, dt, speed=self.params.cruise_speed)
            if done:
                self._to_idle()

    # --- per-state handlers ----------------------------------------------

    def _run_sensors_and_fuse(self, now: float, dt: float, attacker_truth) -> None:
        self.fusion.set_ownship(self.position, self.velocity)
        self.fusion.predict(dt)
        if attacker_truth is None or not attacker_truth.alive:
            return
        heading = unit(self.velocity) if norm(self.velocity) > 1e-6 else np.array([1.0, 0, 0])
        cam = self.sensors.camera(now, self.position, heading, attacker_truth.position)
        rad = self.sensors.radar(now, self.position, self.velocity,
                                 attacker_truth.position, attacker_truth.velocity)
        lid = self.sensors.lidar(now, self.position, attacker_truth.position)
        # radar/lidar can seed; camera only refines
        if rad is not None:
            self.fusion.update_radar(rad)
        if lid is not None:
            self.fusion.update_lidar(lid)
        if cam is not None:
            self.fusion.update_camera(cam)

    def _step_midcourse(self, now, dt, cue, attacker_truth) -> None:
        # fly toward the (uncertain) cue region; track source = GS cue
        target_pos = cue.position if cue is not None else self._cue_fallback()
        self._fly_to(target_pos, dt, speed=self.params.cruise_speed)
        # run onboard sensors continuously so we can detect lock
        self._run_sensors_and_fuse(now, dt, attacker_truth)
        if self.fusion.quality(now) >= self.params.q_handoff:
            self._lock_timer += dt
            if self._lock_timer >= self.params.t_lock:
                self.state = DroneState.ACQUIRING
        else:
            self._lock_timer = 0.0

    def _meas_age(self, now: float) -> float:
        return now - self.fusion.last_update

    def _step_acquiring(self, now, dt, attacker_truth) -> None:
        self._run_sensors_and_fuse(now, dt, attacker_truth)
        est = self.fusion.estimate(now)
        # keep closing while confirming a stable onboard track
        if est.valid:
            self._fly_to(est.position, dt, speed=self.params.max_speed)
        # lost if no fresh measurement for t_lost (FR-OB-6)
        if self._meas_age(now) > self.params.t_lost:
            self._fail(now, "lost_in_acquire")
            return
        if self.fusion.quality(now) >= self.params.q_handoff:
            self._lock_timer += dt
            if self._lock_timer >= self.params.t_lock:
                self.state = DroneState.TERMINAL  # handoff complete
        else:
            self._lock_timer = 0.0

    def _step_terminal(self, now, dt, attacker_truth, teammates) -> None:
        self._run_sensors_and_fuse(now, dt, attacker_truth)
        est = self.fusion.estimate(now)
        # lost if no fresh measurement for t_lost (FR-OB-6)
        if self._meas_age(now) > self.params.t_lost:
            self._fail(now, "track_lost")
            return

        # PN guidance on the onboard estimate
        sp = self.guidance.command(self.position, self.velocity,
                                   est.position, est.velocity, dt)
        self._apply_velocity(sp, dt)

        # capture geometry predicate (FR-OB-5). Capture is gated on a FRESH
        # measurement so we never fire on a stale/ghost estimate.
        fresh = self._meas_age(now) <= self.params.t_fresh
        rng = norm(est.position - self.position)
        closing = self.guidance.closing_speed(self.position, self.velocity,
                                               est.position, est.velocity)
        aspect = angle_between(unit(self.velocity) if norm(self.velocity) > 1e-6
                               else np.array([1.0, 0, 0]),
                               est.position - self.position)
        geometry_ok = (fresh and rng < self.params.r_capture_m and closing > 0.0
                       and aspect < np.deg2rad(self.params.capture_cone_deg))

        if geometry_ok:
            teammate_pos = [t.position for t in teammates if t.drone_id != self.drone_id]
            if self.safety.capture_permitted(self.position, geometry_ok, teammate_pos):
                self._capture(now, est)
                return
        # miss detection: range diverging well past capture range
        if rng > self.params.miss_distance_m and closing < 0.0:
            self._fail(now, "miss")

    # --- transitions ------------------------------------------------------

    def _capture(self, now: float, est) -> None:
        ev = CaptureEvent(
            stamp=now, drone_id=self.drone_id,
            target_id=self.assignment.target_id if self.assignment else 0,
            result=CaptureResult.SUCCESS,
            engagement_point=est.position.copy(),
        )
        self.capture_events.append(ev)
        self.state = DroneState.CAPTURED

    def _fail(self, now: float, reason: str) -> None:
        if self.assignment is not None:
            ev = CaptureEvent(
                stamp=now, drone_id=self.drone_id,
                target_id=self.assignment.target_id,
                result=CaptureResult.MISS,
                engagement_point=self.position.copy(),
            )
            self.capture_events.append(ev)
        self.state = DroneState.FAILED
        self.fail_reason = reason

    def _to_idle(self) -> None:
        self.state = DroneState.IDLE
        self.assignment = None
        self._reset_track()

    def _reset_track(self) -> None:
        self.fusion = FusionEKF(
            accel_sigma=self.fusion.accel_sigma,
            gate_chi2=self.fusion.gate_chi2,
        )
        self._lock_timer = 0.0
        self._lost_timer = 0.0

    # --- kinematics -------------------------------------------------------

    def _apply_velocity(self, setpoint: np.ndarray, dt: float) -> None:
        setpoint = clamp_norm(setpoint, self.params.max_speed)
        alpha = dt / (self.params.accel_tau + dt)
        self.velocity = self.velocity + alpha * (setpoint - self.velocity)
        self.position = self.position + self.velocity * dt

    def _fly_to(self, target: np.ndarray, dt: float, speed: float) -> bool:
        to = target - self.position
        d = norm(to)
        if d < 0.5:
            self._apply_velocity(np.zeros(3), dt)
            return True
        sp = unit(to) * min(speed, d / max(dt, 1e-3))
        self._apply_velocity(sp, dt)
        return False

    def _cue_fallback(self) -> np.ndarray:
        # no cue available: loiter forward of current heading
        h = unit(self.velocity) if norm(self.velocity) > 1e-6 else np.array([1.0, 0, 0])
        return self.position + h * 50.0
