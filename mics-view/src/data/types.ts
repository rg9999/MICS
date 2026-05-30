// TypeScript mirror of the ACTUAL viewer-gateway wire schema
// (viewer_gateway/aggregator.py build_snapshot, logbus.py, server.py).
//
// NOTE: positions are delivered pre-transformed to geodetic [lon, lat, alt]
// (Cesium order) by the gateway; `enu` carries the raw local-frame vector. The
// client never does ENU<->geodetic math — the datum lives server-side.

export type DroneStateName =
  | "IDLE" | "ASSIGNED" | "MIDCOURSE" | "ACQUIRING"
  | "TERMINAL" | "CAPTURED" | "FAILED" | "unknown";

export type TrackSourceName = "internal_sim" | "external" | "onboard" | "unknown";
export type AssignmentRoleName = "primary" | "standby" | "unknown";
export type CaptureResultName = "attempt" | "success" | "miss" | "unknown";

export type LonLatAlt = [number, number, number]; // [lon, lat, alt] degrees/m
export type Vec3 = [number, number, number];

export interface DroneView {
  droneId: number;
  state: DroneStateName;
  currentTarget: number;
  batteryPct: number;
  trackQuality: number;
  position: LonLatAlt;
  enu: Vec3;
  velEnu: Vec3;
}

export interface TrackView {
  targetId: number;
  position: LonLatAlt;
  enu: Vec3;
  source: TrackSourceName;
  classConfidence: number;
  age: number;
  posSigmaM: number;        // sqrt(trace(cov)/3) — 1-sigma sphere radius (m)
  covEnu: number[];         // length 9, row-major 3x3
  velEnu: Vec3 | null;
}

export interface AssignmentView {
  droneId: number;
  targetId: number;         // 0 == unassigned
  role: AssignmentRoleName;
}

export interface CaptureEventView {
  stamp: number;
  droneId: number;
  targetId: number;
  result: CaptureResultName;
  position: LonLatAlt;
  enu: Vec3;
}

export interface DroneDerived {
  droneId: number;
  speed: number;
  altitude: number;
  allocatedTarget: number | null;
  rangeToTarget: number | null;
  etaToInterceptS: number | null;
}

export type AllocationStatus =
  | { kind: "UNENGAGED" }
  | { kind: "ENGAGED"; byDrone: number }
  | { kind: "CAPTURED" };

export interface TargetDerived {
  targetId: number;
  speed: number | null;
  altitude: number;
  allocation: AllocationStatus;
}

export interface Datum {
  lat: number;
  lon: number;
  alt: number;
}

export interface FrameSnapshot {
  type: "frame";
  stamp: number;
  datum: Datum;
  drones: DroneView[];
  tracks: TrackView[];
  assignments: AssignmentView[];
  estimates: Record<string, TrackView>; // keyed by drone id (string)
  events: CaptureEventView[];
  dronesDerived: DroneDerived[];
  targetsDerived: TargetDerived[];
}

export type LogLevelName = "DEBUG" | "INFO" | "WARN" | "ERROR" | "FATAL";

export interface LogRecord {
  stamp: number;
  level: LogLevelName;
  source: string;
  msg: string;
  file: string;
  func: string;
  line: number;
}

export interface LogBatch {
  type: "logs";
  records: LogRecord[];
}

export interface HelloMessage {
  type: "hello";
  mode: "live" | "replay";
  source: string;
  datum: Datum;
  snapshotRateHz: number;
  controlsEnabled: boolean;
  logRingRows: number;
}

export interface AckMessage {
  type: "ack";
  action: string;
  ok: boolean;
  result?: unknown;
  error?: string;
}

export interface SimStatusMessage {
  type: "status";
  channel: "sim";
  state: number;
  scenarioId: string;
  runId: string;
  elapsedS: number;
  dronesUp: number;
  targetsUp: number;
  recordingSessionId: string;
  requestedRtf: number;
  actualRtf: number;
  rtfControllable: boolean;
  message: string;
}

export type ServerMessage =
  | FrameSnapshot | LogBatch | HelloMessage | AckMessage | SimStatusMessage;

// --- control channel (client -> gateway) ---------------------------------

export interface ControlMessage {
  type: "control";
  action: string;
  [k: string]: unknown;
}

// --- catalog / recording metadata (control-channel results) ---------------

export interface ScenarioInfo {
  scenarioId: string;
  name: string;
  description: string;
  targetSource: string;
  defenderCount: number;
  attackerCount: number;
  sensorMode: string;
}

export interface SegmentRef {
  file: string;
  startStamp: number;
  endStamp: number;
  rows: number;
}

export interface RecordingManifest {
  sessionId: string;
  startedAt: number;
  stoppedAt: number | null;
  durationS: number | null;
  scenario?: string;
  datum: Datum;
  droneCount: number;
  targetCount: number;
  stateSegments: SegmentRef[];
  logSegments: SegmentRef[];
  hasRosbag2: boolean;
}

export interface RecordingStatus {
  recording: boolean;
  sessionId: string;
  elapsedS: number;
  sizeBytes: number;
}

export const SIM_RUN_STATES = [
  "IDLE", "LAUNCHING", "RUNNING", "STOPPING", "STOPPED", "ERROR",
] as const;

export const LAYER_KEYS = [
  "defenders", "targets", "cues", "assignments", "fusedTracks",
  "events", "velocity", "trails", "radar", "fov", "lidar", "volumes",
] as const;
export type LayerKey = (typeof LAYER_KEYS)[number];
