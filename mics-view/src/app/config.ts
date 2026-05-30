import type { Datum, LayerKey } from "../data/types";

// Default WebSocket URL: prefer a build-time override, else derive from the
// page host so one build works on any host (browser is outside Docker; the
// gateway publishes :8080 to the host — see PRD §6.8 hop 2).
function defaultWsUrl(): string {
  const fromEnv = import.meta.env.VITE_WS_URL as string | undefined;
  if (fromEnv && fromEnv.length > 0) return fromEnv;
  const host = typeof window !== "undefined" ? window.location.hostname : "localhost";
  return `ws://${host}:8080`;
}

function num(v: string | undefined, fallback: number): number {
  const n = v === undefined ? NaN : Number(v);
  return Number.isFinite(n) ? n : fallback;
}

export const config = {
  wsUrl: defaultWsUrl(),
  // Fallback datum; the gateway also sends the authoritative datum in `hello`
  // and every frame, which the store adopts as the single source of truth.
  datum: {
    lat: num(import.meta.env.VITE_DATUM_LAT, 32.0853),
    lon: num(import.meta.env.VITE_DATUM_LON, 34.7818),
    alt: num(import.meta.env.VITE_DATUM_ALT, 30.0),
  } as Datum,
  reconnectMs: 1500,
};

export const DEFAULT_LAYERS: Record<LayerKey, boolean> = {
  defenders: true,
  targets: true,
  cues: true,
  assignments: true,
  fusedTracks: true,
  events: true,
  velocity: true,
  trails: true,
  radar: false,
  fov: false,
  lidar: false,
  volumes: false,
};

// State -> colour (CSS) for defenders; mirrors aggregator STATE_NAMES.
export const STATE_COLORS: Record<string, string> = {
  IDLE: "#8a8f98",
  ASSIGNED: "#4da3ff",
  MIDCOURSE: "#ffd24d",
  ACQUIRING: "#ff9f43",
  TERMINAL: "#ff5e5e",
  CAPTURED: "#46d17a",
  FAILED: "#b14de0",
  unknown: "#cccccc",
};

export const LEVEL_COLORS: Record<string, string> = {
  DEBUG: "#8a8f98",
  INFO: "#cfd3da",
  WARN: "#ffd24d",
  ERROR: "#ff5e5e",
  FATAL: "#ff2d2d",
};
