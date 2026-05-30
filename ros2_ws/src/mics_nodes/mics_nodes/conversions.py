"""Conversions between mics core dataclasses and mics_msgs ROS2 messages.

Keeping this in one place means the rclpy nodes stay thin: they translate at the
DDS boundary and delegate all logic to the (tested) mics core.
"""

from __future__ import annotations

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, Twist, Vector3

from mics_msgs.msg import Assignment as AssignmentMsg
from mics_msgs.msg import CaptureEvent as CaptureEventMsg
from mics_msgs.msg import DroneStatus as DroneStatusMsg
from mics_msgs.msg import TargetTrack as TargetTrackMsg

from mics.msgs import (
    Assignment,
    AssignmentRole,
    CaptureEvent,
    CaptureResult,
    DroneState,
    DroneStatus,
    TargetTrack,
    TrackSource,
)


def stamp_from(t: float) -> Time:
    sec = int(t)
    return Time(sec=sec, nanosec=int((t - sec) * 1e9))


def stamp_to(time: Time) -> float:
    return time.sec + time.nanosec * 1e-9


def _point(v: np.ndarray) -> Point:
    return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))


def _vec3(v: np.ndarray) -> Vector3:
    return Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))


def _arr(p) -> np.ndarray:
    return np.array([p.x, p.y, p.z], dtype=float)


# --- TargetTrack ---------------------------------------------------------

def track_to_msg(t: TargetTrack) -> TargetTrackMsg:
    m = TargetTrackMsg()
    m.header.stamp = stamp_from(t.stamp)
    m.header.frame_id = "map"
    m.target_id = int(t.target_id)
    m.position = _point(t.position)
    m.velocity = _vec3(t.velocity)
    m.position_covariance = [float(x) for x in np.asarray(t.position_covariance).reshape(9)]
    m.class_confidence = float(t.class_confidence)
    m.source = int(t.source)
    m.age = float(t.age)
    m.has_velocity = bool(t.has_velocity)
    return m


def track_from_msg(m: TargetTrackMsg) -> TargetTrack:
    return TargetTrack(
        stamp=stamp_to(m.header.stamp),
        target_id=int(m.target_id),
        position=_arr(m.position),
        velocity=_arr(m.velocity),
        position_covariance=np.array(m.position_covariance, dtype=float).reshape(3, 3),
        class_confidence=float(m.class_confidence),
        source=TrackSource(int(m.source)),
        age=float(m.age),
        has_velocity=bool(m.has_velocity),
    )


# --- Assignment ----------------------------------------------------------

def assignment_to_msg(a: Assignment) -> AssignmentMsg:
    m = AssignmentMsg()
    m.header.stamp = stamp_from(a.stamp)
    m.drone_id = int(a.drone_id)
    m.target_id = int(a.target_id)
    m.role = int(a.role)
    return m


def assignment_from_msg(m: AssignmentMsg) -> Assignment:
    return Assignment(
        stamp=stamp_to(m.header.stamp),
        drone_id=int(m.drone_id),
        target_id=int(m.target_id),
        role=AssignmentRole(int(m.role)),
    )


# --- DroneStatus ---------------------------------------------------------

def status_to_msg(s: DroneStatus) -> DroneStatusMsg:
    m = DroneStatusMsg()
    m.header.stamp = stamp_from(s.stamp)
    m.drone_id = int(s.drone_id)
    pose = Pose()
    pose.position = _point(s.position)
    m.pose = pose
    twist = Twist()
    twist.linear = _vec3(s.velocity)
    m.twist = twist
    m.state = int(s.state)
    m.current_target = int(s.current_target)
    m.battery_pct = float(s.battery_pct)
    m.track_quality = float(s.track_quality)
    return m


def status_from_msg(m: DroneStatusMsg) -> DroneStatus:
    return DroneStatus(
        stamp=stamp_to(m.header.stamp),
        drone_id=int(m.drone_id),
        position=_arr(m.pose.position),
        velocity=_arr(m.twist.linear),
        state=DroneState(int(m.state)),
        current_target=int(m.current_target),
        battery_pct=float(m.battery_pct),
        track_quality=float(m.track_quality),
    )


# --- CaptureEvent --------------------------------------------------------

def capture_to_msg(c: CaptureEvent) -> CaptureEventMsg:
    m = CaptureEventMsg()
    m.header.stamp = stamp_from(c.stamp)
    m.drone_id = int(c.drone_id)
    m.target_id = int(c.target_id)
    m.result = int(c.result)
    m.engagement_point = _point(c.engagement_point)
    return m


def capture_from_msg(m: CaptureEventMsg) -> CaptureEvent:
    return CaptureEvent(
        stamp=stamp_to(m.header.stamp),
        drone_id=int(m.drone_id),
        target_id=int(m.target_id),
        result=CaptureResult(int(m.result)),
        engagement_point=_arr(m.engagement_point),
    )
