import { create } from "zustand";
import { config, DEFAULT_LAYERS } from "./config";
import type {
  AssignmentView, CaptureEventView, Datum, DroneDerived, DroneView,
  FrameSnapshot, LayerKey, LogRecord, RecordingManifest, RecordingStatus,
  ScenarioInfo, ServerMessage, SimStatusMessage, TargetDerived, TrackView,
} from "../data/types";

const MAX_LOG_ROWS = 20000;
const MAX_TRAIL = 240;       // ~10 s of trail at 25 Hz
const MAX_EVENTS = 200;

export type Selection = { kind: "drone" | "target"; id: number } | null;

export interface TrailPoint { lon: number; lat: number; alt: number; t: number; }

export interface ReplayState {
  playhead: number;
  t0: number;
  t1: number;
  speed: number;
  paused: boolean;
}

interface StoreState {
  // connection
  connected: boolean;
  mode: "live" | "replay";
  sourceName: string;
  controlsEnabled: boolean;
  datum: Datum;

  // latest frame
  stamp: number;
  drones: DroneView[];
  tracks: TrackView[];
  assignments: AssignmentView[];
  estimates: Record<string, TrackView>;
  dronesDerived: Record<number, DroneDerived>;
  targetsDerived: Record<number, TargetDerived>;
  events: CaptureEventView[];
  trails: Map<string, TrailPoint[]>;

  // logs (bounded ring)
  logs: LogRecord[];

  // selection + layers
  selection: Selection;
  layers: Record<LayerKey, boolean>;

  // recording / scenarios / replay / sim
  recording: RecordingStatus | null;
  recordings: RecordingManifest[];
  scenarios: ScenarioInfo[];
  sim: SimStatusMessage | null;
  replay: ReplayState | null;

  // actions
  applyMessage: (m: ServerMessage) => void;
  setConnected: (v: boolean) => void;
  setSelection: (s: Selection) => void;
  toggleLayer: (k: LayerKey) => void;
  setRecordings: (r: RecordingManifest[]) => void;
  setRecording: (r: RecordingStatus) => void;
  setScenarios: (s: ScenarioInfo[]) => void;
  setReplay: (r: ReplayState) => void;
}

function trailKey(kind: "drone" | "target", id: number) {
  return `${kind}:${id}`;
}

export const useStore = create<StoreState>((set) => ({
  connected: false,
  mode: "live",
  sourceName: "",
  controlsEnabled: false,
  datum: config.datum,

  stamp: 0,
  drones: [],
  tracks: [],
  assignments: [],
  estimates: {},
  dronesDerived: {},
  targetsDerived: {},
  events: [],
  trails: new Map(),

  logs: [],

  selection: null,
  layers: { ...DEFAULT_LAYERS },

  recording: null,
  recordings: [],
  scenarios: [],
  sim: null,
  replay: null,

  applyMessage: (m) =>
    set((s) => {
      switch (m.type) {
        case "hello":
          return {
            mode: m.mode, sourceName: m.source, controlsEnabled: m.controlsEnabled,
            datum: m.datum,
          };
        case "frame":
          return applyFrame(s, m);
        case "logs": {
          const next = s.logs.concat(m.records);
          if (next.length > MAX_LOG_ROWS) next.splice(0, next.length - MAX_LOG_ROWS);
          return { logs: next };
        }
        case "status":
          return { sim: m };
        default:
          return {};
      }
    }),

  setConnected: (v) => set({ connected: v }),
  setSelection: (selection) => set({ selection }),
  toggleLayer: (k) => set((s) => ({ layers: { ...s.layers, [k]: !s.layers[k] } })),
  setRecordings: (recordings) => set({ recordings }),
  setRecording: (recording) => set({ recording }),
  setScenarios: (scenarios) => set({ scenarios }),
  setReplay: (replay) => set({ replay }),
}));

function applyFrame(s: StoreState, m: FrameSnapshot): Partial<StoreState> {
  const dronesDerived: Record<number, DroneDerived> = {};
  for (const d of m.dronesDerived) dronesDerived[d.droneId] = d;
  const targetsDerived: Record<number, TargetDerived> = {};
  for (const t of m.targetsDerived) targetsDerived[t.targetId] = t;

  // append to trails (new Map so subscribers relying on identity update)
  const trails = new Map(s.trails);
  const seen = new Set<string>();
  for (const d of m.drones) {
    pushTrail(trails, trailKey("drone", d.droneId), d.position, m.stamp);
    seen.add(trailKey("drone", d.droneId));
  }
  for (const t of m.tracks) {
    pushTrail(trails, trailKey("target", t.targetId), t.position, m.stamp);
    seen.add(trailKey("target", t.targetId));
  }
  // drop trails for entities no longer present
  for (const k of trails.keys()) if (!seen.has(k)) trails.delete(k);

  const events = m.events.length
    ? s.events.concat(m.events).slice(-MAX_EVENTS)
    : s.events;

  return {
    stamp: m.stamp, datum: m.datum, drones: m.drones, tracks: m.tracks,
    assignments: m.assignments, estimates: m.estimates,
    dronesDerived, targetsDerived, events, trails,
  };
}

function pushTrail(
  trails: Map<string, TrailPoint[]>, key: string,
  pos: [number, number, number], t: number,
) {
  const arr = trails.get(key) ?? [];
  const next = arr.concat({ lon: pos[0], lat: pos[1], alt: pos[2], t });
  if (next.length > MAX_TRAIL) next.splice(0, next.length - MAX_TRAIL);
  trails.set(key, next);
}
