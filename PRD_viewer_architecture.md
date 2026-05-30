# PRD + Architecture — MICS 3D Situational-Awareness Viewer ("MICS-View")

**Version:** 0.1 (draft)
**Status:** For engineering review
**Companion to:** *PRD — Multi-Drone Cooperative Interception System (MICS) v0.1*
**Last updated:** 2026-05-30

---

## 0. Relationship to the MICS PRD

MICS-View is the **frontend** of the ground station's `mics_monitor` capability (MICS PRD §7). It consumes the same ROS2 topics and message schemas defined there (`/tracks`, `/state_sharing_drone_N`, `/assignments`, `TargetEstimate`, `CaptureEvent`, plus sensor topics) and renders them in a single interactive 3D scene. It inherits the same **non-kinetic, capture-based** framing: the viewer displays pursuit/capture activity and never exposes any kinetic functionality.

---

## 1. Overview & purpose

MICS-View is a browser-based 3D viewer that shows, in one interactive globe scene:
- **Attackers** (targets) and **defenders** (interceptors) as live, time-dynamic entities.
- **Sensor plots**: radar coverage cones, camera FOV frustums, LiDAR point clouds, and onboard fused-track estimates with uncertainty.
- **Cues, tracks, assignments, and events** from the ground station and the swarm.

It operates in two modes:
- **Live**: streaming from the running system (sim or real) via WebSocket.
- **Replay**: scrub/playback of a recorded scenario for after-action review and debugging.

### 1.1 Chosen technology (decision)

| Concern | Choice | Rationale |
|---|---|---|
| 3D engine | **CesiumJS** | WGS84 globe + terrain, time-dynamic entities with built-in clock/timeline, native volumetric primitives (cones/frustums/volumes), 3D Tiles point clouds, glTF models. Apache-2.0, self-hostable (Cesium Ion **not** required). |
| React binding | **Resium** | React wrapper for CesiumJS; fits component model. |
| Language / build | **TypeScript + Vite** | Type-safe message contracts, fast dev server. |
| ROS transport | **rosbridge_suite** (`rosbridge_server`) + **roslib.js** | Standard ROS2↔WebSocket bridge. |
| Throughput control | **viewer-gateway** (thin Node/Python service) | Throttles/decimates high-rate topics, does ENU→geodetic transforms server-side, emits compact per-frame snapshots. |
| Point clouds | **3D Tiles** (large/streamed) or `PointPrimitiveCollection` (small/live) | Scales from a few k points live to streamed tilesets. |
| Data grids | **AG Grid (Community / MIT)** | High-performance virtualized rows, async transaction updates built for streaming data, sorting/filtering/conditional formatting. Community tier is free and covers all required features. |
| Log firehose (option) | **Glide Data Grid** (canvas) | Canvas-rendered grid for extreme row volumes; use only if `/rosout` rates exceed what AG Grid's row virtualization handles comfortably. |

> CesiumJS the library is free and offline-capable; the optional **Cesium Ion** cloud service (terrain/asset hosting) is *not* a dependency. Self-host terrain and imagery to keep the system fully open-source and air-gappable.

---

## 2. Scope & non-goals

**In scope:** visualization of all entities/sensors/tracks/assignments/events; live streaming + replay; layer toggles, filtering, camera/follow controls; telemetry panels; an **optional, permission-gated** operator-control panel that proxies the MICS GS controls (arm/disarm-all, abort/RTL-all, pause-allocation).

**Non-goals:** the viewer is **not** the autonomy or allocation logic (that's the GS/onboard stack); it does **not** perform sensor fusion (it displays the fused output); it does **not** introduce any kinetic capability; it is **not** a flight-control UI for piloting individual drones.

---

## 3. Users & use cases

| User | Use case |
|---|---|
| **Operator** | Live monitoring of an engagement; sees who is pursuing what, sensor coverage, and capture outcomes; can abort/RTL all via gated controls. |
| **Autonomy engineer** | Debug allocation/handoff/reassignment by watching assignments, track quality, and state transitions in real time or replay. |
| **Perception engineer** | Inspect sensor coverage vs target geometry, fused-track vs ground-truth error, point-cloud quality. |
| **Reviewer** | After-action replay with timeline scrub and event markers; export clips/snapshots. |

---

## 4. Functional requirements

IDs `FR-V-<n>`. Priority M/S/C.

### 4.1 Entities
- **FR-V-1 (M)** Render each defender as a glTF model at its live pose; label with drone ID, state, battery, track quality.
- **FR-V-2 (M)** Colour defenders by state (IDLE/ASSIGNED/MIDCOURSE/ACQUIRING/TERMINAL/CAPTURED/FAILED).
- **FR-V-3 (M)** Render each target/attacker as a distinct entity; show classification confidence.
- **FR-V-4 (M)** Show velocity vectors and configurable trailing path (track history) per entity.
- **FR-V-5 (S)** Optional "truth" debug layer (sim only) showing attacker ground truth as a ghost entity, with live error to the displayed cue/track.

### 4.2 Sensor plots
- **FR-V-6 (M)** Radar coverage as an orientable cone/volume attached to each drone (range + beamwidth from config).
- **FR-V-7 (M)** Camera FOV as a frustum attached to each drone (FOV + range).
- **FR-V-8 (M)** LiDAR point cloud rendered in-scene (3D Tiles or live point primitives), per drone, toggleable.
- **FR-V-9 (M)** Onboard fused `TargetEstimate` shown as a marker + uncertainty ellipsoid (from covariance), visually distinct from GS cues.
- **FR-V-10 (S)** Highlight the **handoff moment** (cue→onboard) per drone.

### 4.3 Cues, tracks, assignments, events
- **FR-V-11 (M)** GS `/tracks` (cues) as markers with uncertainty ellipsoids (from `position_covariance`) and staleness/age styling.
- **FR-V-12 (M)** `/assignments` as a polyline linking each drone to its assigned target, styled by role (primary/standby).
- **FR-V-13 (M)** `CaptureEvent` as a timestamped marker/animation (attempt/success/miss) and an entry in the event log.
- **FR-V-14 (S)** Geofence and engagement corridors as translucent volumes.

### 4.4 Time, recording & replay
- **FR-V-15 (M)** Live mode: smooth interpolation of entity motion between updates.
- **FR-V-16 (M)** Replay mode: load a recorded session and play it back with Cesium timeline scrub, play/pause, and variable speed. Replay reconstructs **scene + grids + logs** (not scene-only).
- **FR-V-17 (S)** Bookmark/jump to events (captures, reassignments, failures).

**Recording controls**
- **FR-V-37 (M)** Operator can **Start** and **Stop** recording from the UI. Recording is performed **server-side in the gateway** (so it captures full fidelity, survives browser refresh, and runs even with no browser attached).
- **FR-V-38 (M)** Each Start creates a **new, timestamped session directory**; each Stop finalizes it. Sessions are immutable once stopped.
- **FR-V-39 (M)** Within a session, data is written as **rolling segment files** (rotate by size or duration), so long runs are bounded, crash-resilient, and seekable.
- **FR-V-40 (M)** A recording captures **both** the state stream (snapshots: entities, sensors, assignments, events, derived grid fields) **and** all logs (`/rosout`). Logs are part of every recording.
- **FR-V-41 (M)** The UI lists available recordings (with metadata: start/stop time, duration, scenario, drone/target counts) and lets the user **select one to view**, switching the viewer into replay mode against it.
- **FR-V-42 (S)** Recording status indicator (idle / recording / elapsed time / current session size).
- **FR-V-43 (C)** Export a selected recording (or a time window) to **CZML** for sharing a scene-only replay outside MICS-View.

### 4.5 Camera & interaction
- **FR-V-18 (M)** Free orbit/pan/zoom; click an entity to select and pin a detail panel.
- **FR-V-19 (M)** Follow modes: free, follow-drone (chase/onboard-ish), top-down tactical.
- **FR-V-20 (S)** "Frame all active engagements" auto-fit.

### 4.6 Panels, layers, filtering
- **FR-V-21 (M)** Layer toggles for each visual class (defenders, targets, radar, FOV, LiDAR, cues, fused tracks, assignments, geofence, truth-debug).
- **FR-V-22 (M)** Entity detail panel (telemetry for selected entity).
- **FR-V-23 (M)** Roster panel (all drones, states, assignments) and event log.
- **FR-V-24 (S)** Filter by state, by target, or by individual drone.

### 4.7 Operator controls (optional, gated)
- **FR-V-25 (S)** Permission-gated control panel proxying GS controls: arm/disarm-all, abort/RTL-all, pause-allocation. Disabled by default; requires auth + explicit enablement; visually segregated; every action confirmed.
- **FR-V-26 (M)** When controls are disabled, the viewer is strictly read-only.

### 4.8 Tabular / grid views

High-performance grids, displayed as dockable panels alongside the 3D scene and synchronized with it (selecting a row highlights/flies-to the entity, and vice versa). All grids update live and support sorting, column filtering, and conditional (colour) formatting.

**Interceptor grid (FR-V-27, M)** — one row per defender:

| Column | Source | Notes |
|---|---|---|
| Drone ID | `DroneStatus.droneId` | |
| State | `DroneStatus.state` | colour-coded |
| Power | `DroneStatus.batteryPct` | red/amber thresholds; sortable to surface low batteries |
| Speed | ‖`DroneStatus.velocity`‖ | derived |
| Altitude | `DroneStatus.position` (up / geodetic alt) | derived |
| Allocated target | join `Assignment.targetId` | "—" if unassigned/standby |
| Range to target | computed dist(drone, assigned target) | gateway-computed (§6.2) |
| Track quality | `DroneStatus.trackQuality` | 0..1 |

- **FR-V-28 (M)** Range-to-target is computed once in the gateway from drone + assigned-target positions and delivered as a field (clients don't recompute).
- **FR-V-29 (S)** Optional derived ETA-to-intercept column (range / closing speed).

**Target grid (FR-V-30, M)** — one row per target:

| Column | Source | Notes |
|---|---|---|
| Target ID | `TargetTrack.targetId` | |
| Class confidence | `TargetTrack.classConfidence` | |
| Speed | ‖`TargetTrack.velocity`‖ | if available |
| Altitude | `TargetTrack.position` | derived |
| Allocation status | derived | `UNENGAGED` / `ENGAGED (drone N)` / `CAPTURED` |
| Source | `TargetTrack.source` | internal_sim / external / onboard |
| Age | `TargetTrack.age` | stale rows styled/greyed |

- **FR-V-31 (M)** Allocation status is derived in the gateway: `ENGAGED` if any active `Assignment` targets it; `CAPTURED` once a `CaptureEvent{result: success}` is received; otherwise `UNENGAGED`. (Per the non-kinetic frame the terminal state is **CAPTURED**, not "destroyed".)

**Process-log grid (FR-V-32, M)** — one row per log record from all running processes:

| Column | Source (`rcl_interfaces/msg/Log`) | Notes |
|---|---|---|
| Time | `stamp` | |
| Level | `level` (DEBUG/INFO/WARN/ERROR/FATAL) | colour-coded by severity |
| Source | `name` (logger/node name) | the originating process/node |
| Message | `msg` | |
| Location | `file`:`line` / `function` | collapsible/optional column |

- **FR-V-33 (M)** Aggregate logs from **all** processes (all drones + ground station) via the ROS2 standard `/rosout` topic — **no custom DDS message needed** (see §7.1).
- **FR-V-34 (M)** Live tail with **auto-scroll + pause-on-scroll-up**; bounded **ring buffer** (configurable max rows) so memory stays flat under a firehose.
- **FR-V-35 (M)** Filter by level (≥ severity), by source node, and free-text search on message.
- **FR-V-36 (S)** Click a log row to jump the timeline (replay) to that timestamp; correlate with the 3D scene.

### 4.9 Scenario selection & simulation control

Lets the operator pick a simulation scenario from a vetted catalog and start/stop the run from the viewer. The run is executed by a new **simulation orchestrator** on the sim host (§6.7); the viewer only selects and commands.

- **FR-V-44 (M)** List available scenarios (the MICS scenario catalog, e.g. the YAMLs in MICS PRD Appendix A) with metadata: name, description, target source, defender/attacker counts, sensor mode.
- **FR-V-45 (M)** Select a scenario and **Run** it; optionally apply a profile override (e.g. the CPU-only laptop overlay, MICS PRD Appendix A.1).
- **FR-V-46 (M)** **Stop/abort** a running scenario from the UI.
- **FR-V-47 (M)** Show live run status: state (IDLE/LAUNCHING/RUNNING/STOPPING/STOPPED/ERROR), elapsed time, drones up, targets up, and any error detail.
- **FR-V-48 (S)** **Auto-start a recording** when a run launches (ties scenario → recording session), so every run is captured incl. logs.
- **FR-V-49 (M)** Run/Stop are **auth-gated** (same gate as operator controls) and selectable **only from the catalog** — the viewer never passes free-form commands or paths to the host.
- **FR-V-50 (C)** Re-run the scenario associated with a past recording ("run this again").

**Simulation speed (real-time factor).** Distinct from *replay* speed (FR-V-16, a viewer-clock multiplier over recorded data); this controls how fast the **live** sim advances vs wall-clock.
- **FR-V-51 (S)** Set an initial real-time factor (RTF) at launch as part of the run request (e.g. 1×, 4×, "as fast as possible").
- **FR-V-52 (S)** Adjust RTF **at runtime** for a running sim, via a gated command.
- **FR-V-53 (M)** Display both **requested** and **achieved** RTF in run status — requested ≠ achieved when the host is compute-bound (a requested 10× may only achieve 4×).
- **FR-V-54 (M)** RTF control applies to **pure SITL only**. On HITL (Jetson-in-the-loop) or real hardware the orchestrator forces RTF = 1× and disables the control (you cannot fast-forward a physical clock or a real autopilot).

> Speed ceiling is set by the slowest in-loop component: full-perception runs (camera render + YOLO) and HITL cap achievable RTF well below `ideal`/`stub` runs (ties back to the CPU-only profile, MICS PRD §4.5).

> Scope note: this controls the **simulation process only**. It launches sim scenarios; it does **not** arm or fly real hardware — that stays behind the separate gated operator controls (§4.7).

---

## 5. Non-functional requirements

- **NFR-1 (M)** Sustain ≥30 FPS with 8 defenders + 4 targets + all sensor layers on a mid-range laptop (integrated GPU acceptable — browser WebGL, independent of the headless sim host).
- **NFR-2 (M)** End-to-end glass-to-glass latency in live mode ≤300 ms under nominal load.
- **NFR-3 (M)** Degrade gracefully: dropping a high-rate layer (e.g. LiDAR) must not stall the scene.
- **NFR-4 (M)** Browsers: latest Chrome/Edge/Firefox (WebGL2). Safari best-effort.
- **NFR-5 (M)** Configurable, self-hostable assets (terrain/imagery) for offline/air-gapped use.
- **NFR-6 (S)** If controls are enabled, transport is authenticated and encrypted (WSS + token).
- **NFR-7 (M)** Grids remain responsive (no dropped frames in the 3D scene) at sustained log rates; the log grid uses row virtualization + a bounded buffer, and grid updates run off the Cesium render path.

---

## 6. Architecture

### 6.1 System context

```
            ROS2 graph (sim or real)
   /state_sharing_drone_N  /tracks  /assignments
   /drone_N/radar  /drone_N/fov  /drone_N/lidar
   /drone_N/target_estimate     /capture_events
                    │
                    ▼
          ┌──────────────────────┐
          │   rosbridge_server   │   (rosbridge_suite, WebSocket)
          └──────────┬───────────┘
                     ▼
          ┌──────────────────────┐   • subscribe to ROS topics
          │   viewer-gateway     │   • throttle / decimate
          │  (Node or Python)    │   • ENU → geodetic transform
          │                      │   • per-frame snapshot @ fixed rate
          │                      │   • CZML generation for replay
          └──────────┬───────────┘
                     │  WebSocket (compact JSON snapshots) / WSS if controls on
                     ▼
          ┌──────────────────────┐
          │   Browser (SPA)      │
          │  React + Resium      │
          │  ┌────────────────┐  │
          │  │ Entity store   │  │  ← snapshots → SampledPositionProperty
          │  │ (Zustand/Redux)│  │
          │  └───────┬────────┘  │
          │          ▼           │
          │   CesiumJS scene     │  entities • sensor primitives • point clouds
          │   + React HUD/panels │
          └──────────────────────┘
```

### 6.2 Why the viewer-gateway (key decision)

Raw rosbridge of high-rate topics (10 Hz status × 8 drones, LiDAR, radar) can overwhelm the browser and the WebSocket. The gateway:
1. **Throttles** each topic to a render-appropriate rate and **coalesces** into one **per-frame snapshot** (e.g. 20–30 Hz) — the browser then makes exactly one state update per frame.
2. **Transforms coordinates** ENU(local sim frame)→geodetic once, server-side, so the client never duplicates that math (§6.4).
3. **Decimates point clouds** to a budget before sending; large clouds go out as 3D Tiles instead.
4. **Generates CZML** from a rosbag for replay mode.
5. **Computes grid-derived fields** server-side: per-drone **range-to-target** (drone ↔ assigned target), speed, altitude, and ETA; and per-target **allocation status** (`UNENGAGED`/`ENGAGED (drone N)`/`CAPTURED`) by joining `/assignments` + `/capture_events`. Clients render, they don't recompute.
6. **Aggregates `/rosout`** from all processes into a **bounded, level-throttled** log stream (separate channel from the per-frame snapshot so a log burst can't stall the scene); optionally drops below a configurable severity before forwarding.
7. **Records** the live stream to disk on operator command — a new session directory per Start/Stop, rolling JSONL segments for both state and logs, plus an optional rosbag2 layer (§6.6).
8. **Orchestrates scenarios**: serves the scenario catalog, proxies run/stop to the `mics_sim_orchestrator` action, relays `SimRunStatus`, and (if requested) binds the run to a recording session (§6.7).
9. Is the single **auth boundary** if operator controls are enabled (covers operator controls, recording start/stop, and scenario run/stop).

For a first prototype you *can* skip the gateway and let roslib.js subscribe directly — acceptable for ≤2 drones, ideal-sensor mode. Add the gateway before scaling up.

**Distributed `/rosout` note:** `/rosout` aggregates within a single ROS graph/domain. Since MICS drones are separate machines bridged by **Zenoh**, run a `zenoh-bridge-ros2dds` (or matching `ROS_DOMAIN_ID` + discovery) so every node's `/rosout` reaches the gateway. With that bridge the standard topic already carries the whole fleet's logs — no custom message required.

### 6.3 Frontend component architecture

```
src/
├── main.tsx
├── app/
│   ├── ConnectionProvider.tsx     # WebSocket (gateway or rosbridge) lifecycle
│   ├── store.ts                   # entity/track/assignment/event state (Zustand)
│   └── config.ts                  # datum, sensor params, layer defaults
├── scene/
│   ├── CesiumViewer.tsx           # <Viewer> + clock/timeline wiring
│   ├── DefenderEntities.tsx       # glTF models, labels, paths, velocity vectors
│   ├── TargetEntities.tsx         # targets + cues + uncertainty ellipsoids
│   ├── RadarVolumes.tsx           # CylinderGraphics cones per drone
│   ├── FovFrustums.tsx            # camera frustum geometry per drone
│   ├── LidarClouds.tsx            # 3D Tiles / PointPrimitiveCollection
│   ├── FusedTracks.tsx            # onboard TargetEstimate + covariance ellipsoid
│   ├── AssignmentLinks.tsx        # polylines drone↔target
│   ├── EventMarkers.tsx           # capture/failure/reassignment markers
│   └── Volumes.tsx                # geofence + engagement corridors
├── ui/
│   ├── RosterPanel.tsx
│   ├── EntityDetailPanel.tsx
│   ├── EventLog.tsx
│   ├── LayerToggles.tsx
│   ├── CameraControls.tsx
│   ├── RecordingControls.tsx      # FR-V-37/42: Start/Stop + status indicator
│   ├── RecordingBrowser.tsx       # FR-V-41: list/select a recording to replay
│   ├── ScenarioPanel.tsx          # FR-V-44..54: pick scenario, run/stop, status, sim-speed (RTF)
│   ├── OperatorControls.tsx       # gated; hidden unless enabled+authed
│   └── grids/
│       ├── GridProvider.tsx       # AG Grid theme/setup + selection sync with scene
│       ├── InterceptorGrid.tsx    # FR-V-27: defenders (power/speed/alt/target/range)
│       ├── TargetGrid.tsx         # FR-V-30: targets + allocation status
│       └── ProcessLogGrid.tsx     # FR-V-32: /rosout firehose (virtualized + ring buffer)
└── data/
    ├── types.ts                   # TS mirrors of mics_msgs + LogRecord
    └── transforms.ts              # client-side helpers (if no gateway)
```

Grid↔scene selection is bidirectional via the shared store: selecting a grid row sets the selected entity (scene flies-to/highlights it), and selecting an entity in the 3D view scrolls/selects its grid row.

### 6.4 Coordinate frames (a real gotcha)

Sim drones typically operate in a **local ENU frame** relative to a datum (origin lat/lon/alt). Cesium works in **geodetic / ECEF**. Resolve this explicitly:
- Define a single **datum** in config (`origin: {lat, lon, alt}`).
- Convert ENU→geodetic in the **gateway** (preferred) or in `transforms.ts` using Cesium's `Transforms.eastNorthUpToFixedFrame` / `Cartesian3` from the datum.
- Orientation: convert body quaternions to Cesium `Quaternion` in ECEF so glTF models and sensor cones point correctly.
- Keep one source of truth for the datum; a mismatch is the most common "everything is in the ocean off Africa" bug.

### 6.5 Live vs replay data flow

Live and replay share **one client code path**: the viewer always consumes `FrameSnapshot` + `LogBatch` messages over the socket. The only difference is the source — the live ROS subscriptions, or a recording read back from disk.

- **Live:** gateway snapshot → store → each entity's position fed into a Cesium `SampledPositionProperty` so motion interpolates smoothly between updates; Cesium clock runs in real time. Logs arrive on the separate log channel.
- **Replay:** the gateway reads a selected recording and re-emits the recorded `FrameSnapshot`/`LogBatch` messages over the same socket, paced to the Cesium clock (which the user can scrub, pause, and speed-vary). Because the recorded messages are identical to live ones, scene, grids, and the process-log grid all reconstruct without special-casing.

This resolves the earlier open question: **replay is the gateway re-emitting its own recorded stream — not CZML.** CZML is retained only as an optional *export* artifact (FR-V-43) for sharing a scene-only replay outside MICS-View.

### 6.6 Recording subsystem (gateway-owned)

Recording lives in the gateway because it already sees the full, derived, transformed stream and runs independently of any browser.

**Lifecycle.** `Start` → gateway opens a new session directory and begins appending; `Stop` → flushes, finalizes the manifest, marks the session immutable. Start/Stop are commands over the control channel (auth-gated like other controls, §10).

**What is written.** Two parallel streams plus metadata:
- **State stream** — the `FrameSnapshot` sequence (entities, sensors, assignments, events, derived grid fields).
- **Log stream** — the `LogBatch` sequence from `/rosout` (**logs are always part of a recording**, FR-V-40).
- **Manifest** — session metadata and a segment index for fast seeking.

**Format & rotation.** Newline-delimited JSON (**JSONL**), one message per line, gzipped per segment. Segments **roll** when they hit a size or duration limit (configurable), giving bounded files, crash resilience (a crash loses at most the open segment), and seekability (the manifest maps time ranges → segments). JSONL is chosen so live and recorded messages are byte-identical and the replay path is trivial.

**Optional canonical layer.** For full-fidelity, stack-wide replay (not just the viewer), the gateway can additionally trigger a **rosbag2** recording of the raw topics (including `/rosout`) into the same session dir. The viewer replays the lightweight JSONL; rosbag2 is there if you later need to re-derive or feed the data back through the autonomy stack. This reuses the GS-side rosbag logging already in the MICS PRD.

**On-disk layout.**
```
recordings/
└── 2026-05-30T14-03-12_run-01/        # one dir per Start/Stop session
    ├── manifest.json                  # start/stop, datum, scenario, counts, segment index
    ├── state/
    │   ├── state-000001.jsonl.gz      # rolling FrameSnapshot segments
    │   └── state-000002.jsonl.gz
    ├── logs/
    │   └── logs-000001.jsonl.gz       # rolling LogBatch segments (/rosout)
    ├── rosbag2/                        # optional canonical raw recording
    └── export.czml                    # optional, generated on demand (FR-V-43)
```

**Selection for viewing.** The gateway exposes a recordings index (list + manifest metadata). The UI `RecordingBrowser` shows available sessions; selecting one switches the gateway into replay mode against that session's segments and the viewer reconstructs the full scene+grids+logs.

```
   [Browser]  Start/Stop, Select recording
        │  (control channel)
        ▼
  ┌───────────────┐   live:  ROS topics ─┐
  │ viewer-gateway│                       ├──▶ FrameSnapshot / LogBatch ──▶ socket ──▶ viewer
  │  recorder ◀───┼── writes ──┐ replay: recording files ─┘
  └───────────────┘            ▼
                         recordings/<session>/{manifest,state/,logs/,rosbag2/}
```

### 6.7 Simulation orchestration (new MICS node + interface)

Selecting and running a scenario requires a new component on the sim host and a new ROS2 interface — the **"new message"** this feature needs.

**New node: `mics_sim_orchestrator`** (added to the MICS stack, sim-host side). It:
- knows a **vetted scenarios directory** (the MICS scenario catalog);
- exposes the scenario list and a run/stop interface;
- runs against **always-on simulation containers** (chosen model): on run it **configures and starts a run inside the already-running sim** rather than creating containers — it resets the world, spawns the scenario's attackers, activates the needed subset of the pre-provisioned defender fleet, sets `sensor_mode` and RTF, and starts the clock. Stop/abort halts the run and resets the world to idle.
- publishes continuous run status.

**Always-on rationale & boundary.** Because the sim containers stay up, the orchestrator commands them purely over **ROS2** (a `sim_control` interface in the simulation container: world reset, entity spawn/despawn, physics/RTF, sensor-mode toggles) — it needs **no access to the Docker daemon / `docker.sock`**, which removes the root-equivalent privilege the launch-containers approach would have required. Trade-off: the sim stack consumes resources while idle, and changing the **fleet composition** itself (max vehicle count, PX4↔ArduPilot autopilot type) still requires re-provisioning containers, not a runtime scenario change. Scenarios may therefore vary trajectories, active-defender count (up to the provisioned max), targets, `sensor_mode`, and RTF freely; a different autopilot type or a larger fleet is a redeploy.

**Why a ROS2 Action (primary interface).** "Run a scenario" is a long-running task with progress and cancel — exactly what a ROS2 **Action** is for: a goal (which scenario), streamed feedback (state/elapsed/counts), cancel (abort), and a result (summary). A continuously-published status **topic** complements it so *any* number of viewers (and the gateway) can observe sim state without holding the action handle.

**New interfaces (add to `mics_msgs`):**

```
# mics_msgs/action/RunScenario.action
# ---- Goal ----
string  scenario_id          # from the catalog (never a free-form path/command)
string  overrides_yaml       # optional inline overlay (e.g. laptop profile); may be empty
bool    record               # auto-start a recording session for this run
float64 requested_rtf        # initial real-time factor; 0 = "as fast as possible", SITL only
---
# ---- Result ----
bool    success
string  message
string  run_id
string  recording_session_id # set if record=true
float64 duration_s
---
# ---- Feedback ----
uint8   state                # see SimRunStatus constants
string  detail
float64 elapsed_s
uint8   drones_up
uint8   targets_up
float64 actual_rtf           # achieved real-time factor
```

```
# mics_msgs/msg/SimRunStatus.msg   (published continuously on /sim_run_status)
uint8 IDLE=0
uint8 LAUNCHING=1
uint8 RUNNING=2
uint8 STOPPING=3
uint8 STOPPED=4
uint8 ERROR=5
std_msgs/Header header
uint8   state
string  scenario_id
string  run_id               # unique per run; "" when IDLE
float64 elapsed_s
uint8   drones_up
uint8   targets_up
string  recording_session_id # "" if not recording
float64 requested_rtf        # requested real-time factor (0 = max)
float64 actual_rtf           # achieved RTF (may be < requested when compute-bound)
bool    rtf_controllable     # false on HITL/real hardware (locked at 1.0)
string  message
```

```
# mics_msgs/srv/SetSimSpeed.srv   (runtime RTF change; gated)
float64 requested_rtf        # 0 = as fast as possible; ignored if not controllable
---
bool    success
float64 applied_rtf          # what the orchestrator actually set
string  message
```

```
# mics_msgs/msg/ScenarioInfo.msg   (catalog descriptor)
string scenario_id
string name
string description
string target_source         # internal | external
uint8  defender_count
uint8  attacker_count
string sensor_mode           # ideal | stub | full
string path                  # server-side path (gateway-internal; not exposed raw to clients)
```

**Control flow.**
```
[Browser ScenarioPanel]
   list ──▶ gateway (reads catalog → ScenarioInfo[])
   run(scenario_id, override, record) ──▶ gateway ──▶ RunScenario action goal ──▶ mics_sim_orchestrator
                                                                  │ ROS2 sim_control: reset → spawn → set RTF → start
                                                                  ▼ (always-on simulation container)
   /sim_run_status (SimRunStatus) ◀────────────────────────────── orchestrator
   gateway folds SimRunStatus into the status channel ──▶ ScenarioPanel (live state)
```

**Gateway role.** The gateway is the single choke point: it serves the scenario list (reading the catalog dir directly — no ROS round-trip needed for listing), proxies run/stop to the action, relays `SimRunStatus`, and — if `record=true` — coordinates with the recorder (§6.6) so the run and its recording share a session. It also proxies the gated `SetSimSpeed` service for runtime RTF changes.

**How RTF is applied (and its limits).** The orchestrator sets the simulator's real-time factor — in Gazebo, via the physics real-time-update-rate / max-step-size (the Harmonic physics params) — and PX4/ArduPilot SITL stay in sync through their **lockstep** mechanism, which is what makes faster-than-real-time possible at all. Key limits the orchestrator enforces and reports:
- **Compute-bound:** RTF is a *request*. If the host can't sustain it (especially `full` perception with camera render + YOLO), the achieved `actual_rtf` is lower; the orchestrator measures and reports it (never silently pretends).
- **HITL/real:** with a Jetson in the loop or real aircraft, the autopilot/perception run on a physical clock — `rtf_controllable=false`, RTF forced to 1.0, control disabled.
- **Determinism:** changing RTF mid-run can perturb timing-sensitive behaviour; for reproducible Monte-Carlo runs, prefer setting RTF once at launch.

**Security (critical).** With the always-on model the orchestrator commands the running sim over ROS2 and holds **no Docker-daemon access** (no `docker.sock`), so it cannot create host processes/containers — a deliberate privilege reduction. Remaining controls:
- run/stop and `SetSimSpeed` are **auth-gated** (same boundary as operator controls, §10);
- the client may only pass a **`scenario_id` from the vetted catalog** plus a schema-validated override overlay — **never** a path, shell string, or arbitrary launch args;
- all run/stop/speed actions are audit-logged.
This controls **simulation only** — it cannot arm or command real aircraft, and is hard-disabled on any host where real hardware could be present.

### 6.8 Deployment & container networking

MICS runs as containers. Two communication hops, solved differently:

**Hop 1 — gateway ↔ sim/GS containers (container-to-container).** Do **not** rely on DDS multicast discovery; it does not cross Docker bridge networks. Put all ROS containers on **one user-defined Docker network** (resolve by service name) and route ROS2 traffic over **Zenoh unicast** through a **`zenoh-router`** hub:
- *Recommended:* run `rmw_zenoh_cpp` in every container with the router as the discovery/peering endpoint (`tcp/zenoh-router:7447`) — no multicast anywhere, container- and WAN-friendly.
- *Alternative:* keep your current RMW (Fast DDS) and run a `zenoh-bridge-ros2dds` peered to the router for the topics that must cross containers, with DDS multicast disabled (or a discovery server).

This is the same Zenoh transport MICS already uses for `/state_sharing_*` and the cross-machine `/rosout` bridge.

**Hop 2 — browser ↔ gateway (out of Docker).** The browser runs on the operator's laptop, outside the Docker network, so the gateway **publishes only its WebSocket port to the host** (`8080:8080`); the browser connects to `ws://<host>:8080` (`wss://` + token once operator controls are enabled). That single port is the only thing exposed; all ROS/Zenoh traffic stays internal.

**Orchestrator (always-on model).** The sim containers stay up; `mics_sim_orchestrator` commands them over ROS2 and mounts the scenario catalog **read-only**. It does **not** mount `docker.sock` — no host/daemon access.

**Single-host dev compose (sketch).**
```yaml
networks:
  micsnet: { driver: bridge }   # one network; services resolve by name

services:
  zenoh-router:
    image: eclipse/zenoh:latest
    command: ["-l", "tcp/0.0.0.0:7447"]
    networks: [micsnet]         # 7447 stays internal

  simulation:                   # Gazebo Harmonic + sim_control + sensor models  (ALWAYS ON)
    image: mics/simulation:latest
    networks: [micsnet]
    environment: &rosenv
      ROS_DOMAIN_ID: "42"
      RMW_IMPLEMENTATION: rmw_zenoh_cpp
      ZENOH_ROUTER: tcp/zenoh-router:7447
    depends_on: [zenoh-router]
    # GPU only if running `full` perception; omit on the CPU-only laptop profile

  aircraft:                     # defender SITL + onboard stack  (ALWAYS ON)
    image: mics/aircraft:latest
    networks: [micsnet]
    environment: *rosenv
    deploy: { replicas: 8 }     # pre-provision MAX fleet; scenarios activate a subset
    depends_on: [zenoh-router, simulation]

  ground:                       # target_ingest / track_manager / allocator / monitor
    image: mics/ground:latest
    networks: [micsnet]
    environment: *rosenv
    depends_on: [zenoh-router]

  orchestrator:                 # commands the always-on sim via ROS2 — NO docker.sock
    image: mics/orchestrator:latest
    networks: [micsnet]
    environment: *rosenv
    volumes: ["./scenarios:/scenarios:ro"]    # vetted catalog, read-only
    depends_on: [zenoh-router, simulation]

  viewer-gateway:               # ROS2 <-> browser bridge
    image: mics/viewer-gateway:latest
    networks: [micsnet]
    environment: *rosenv
    ports: ["8080:8080"]        # the ONLY host-exposed port
    volumes: ["./recordings:/recordings"]
    depends_on: [zenoh-router, ground]
```

**Cross-platform note.** On a single Linux host you *could* instead use `network_mode: host` for the ROS containers (multicast works, simplest) — but it's Linux-only and less isolated. The shared-network + Zenoh-router + single published port topology above is the portable default and also works on Docker Desktop (Mac/Windows). Changing the **fleet composition** (replica count, PX4↔ArduPilot) is a compose change + redeploy, not a runtime scenario switch.

---

## 7. Topic → visual mapping (data contract)

| ROS2 topic | Message | Cesium representation | Layer |
|---|---|---|---|
| `/state_sharing_drone_N` | `DroneStatus` | glTF model + label + `SampledPositionProperty` + path (trail) + velocity polyline; colour by `state` | Defenders |
| `/tracks` | `TargetTrack` | billboard/point marker + `EllipsoidGraphics` from `position_covariance`; opacity by `age` | Cues |
| target classified as attacker | `TargetTrack`/truth | distinct entity model | Targets |
| `/assignments` | `Assignment` | `PolylineGraphics` drone↔target; dashed for standby | Assignments |
| `/drone_N/radar` | radar coverage params | `CylinderGraphics` (cone) attached + oriented to drone; range/beamwidth from config | Radar |
| `/drone_N/fov` | camera FOV params | frustum geometry (custom `PrimitiveCollection` or wall/polygon) | FOV |
| `/drone_N/lidar` | `PointCloud2` | 3D Tiles tileset (streamed) or `PointPrimitiveCollection` (live, decimated) | LiDAR |
| `/drone_N/target_estimate` | `TargetEstimate` | marker + covariance `EllipsoidGraphics`, distinct colour | Fused tracks |
| `/capture_events` | `CaptureEvent` | timestamped marker/animation (attempt/success/miss) + event-log row | Events |
| geofence/corridor config | static/params | translucent `CylinderGraphics`/`PolygonGraphics` volumes | Volumes |
| attacker truth (sim) | `/attacker/<id>/truth` | ghost entity + error line to displayed cue/track | Truth-debug |
| `/rosout` | `rcl_interfaces/msg/Log` | rows in the **Process-log grid** (no scene geometry) | Logs |
| `/sim_run_status` | `mics_msgs/SimRunStatus` | drives the **ScenarioPanel** state (no scene geometry) | Sim control |
| `run_scenario` (action) | `mics_msgs/RunScenario` | run/stop goal + feedback for scenario launch (no scene geometry) | Sim control |

Performance note: prefer `PrimitiveCollection`/`PointPrimitiveCollection` over the Entity API for high-count or high-rate layers (radar/FOV/LiDAR); reserve the convenient Entity API for the handful of drones/targets/cues.

### 7.1 Logging: use the ROS2 standard, no custom DDS message

ROS2 already has a standard logging path, so you do **not** need a bespoke DDS log message. By default, log messages from ROS2 nodes go to the console (stderr), to log files on disk, and to the `/rosout` topic on the network, and these targets can be enabled/disabled per node. The message type is **`rcl_interfaces/msg/Log`**, whose fields are: severity-level constants (DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50), `stamp`, `level`, `name`, `msg`, `file`, `function`, and `line`. Each node's logger automatically includes the node's name and namespace, so the `name` field already identifies the originating process.

Implication for MICS-View: the gateway subscribes to `/rosout`, maps each `Log` to a `LogRecord`, and streams it to the Process-log grid. The only distributed-systems caveat is the Zenoh bridge for cross-machine `/rosout` aggregation (§6.2). A custom message would only be worth it if you need structured fields beyond what `Log` carries (e.g. a per-run correlation ID) — in which case publish an *additional* topic, don't replace `/rosout`.

---

## 8. TypeScript data contracts

Mirror the `mics_msgs` schemas (MICS PRD §8) as TS types in `data/types.ts` so the wire format is type-checked end to end. Illustrative:

```ts
export type DroneState =
  | "IDLE" | "ASSIGNED" | "MIDCOURSE" | "ACQUIRING"
  | "TERMINAL" | "CAPTURED" | "FAILED";

export interface DroneStatus {
  stamp: number;
  droneId: number;
  position: [number, number, number];   // ENU from gateway, OR geodetic if pre-transformed
  velocity: [number, number, number];
  orientation: [number, number, number, number]; // quaternion xyzw
  state: DroneState;
  currentTarget: number;
  batteryPct: number;
  trackQuality: number;                  // 0..1
}

export interface TargetTrack {
  stamp: number;
  targetId: number;
  position: [number, number, number];
  velocity?: [number, number, number];
  positionCovariance: number[];          // 9 (3x3)
  classConfidence: number;
  source: "internal_sim" | "external" | "onboard";
  age: number;
}

export interface Assignment {
  droneId: number;
  targetId: number;                      // 0 = unassigned/standby
  role: "primary" | "standby";
}

export interface CaptureEvent {
  stamp: number;
  droneId: number;
  targetId: number;
  result: "attempt" | "success" | "miss";
  engagementPoint: [number, number, number];
}

// Gateway-derived, per-drone grid fields (computed server-side, §6.2):
export interface DroneDerived {
  droneId: number;
  speed: number;                  // ‖velocity‖
  altitude: number;               // geodetic / up
  allocatedTarget: number | null;
  rangeToTarget: number | null;   // null if unassigned
  etaToInterceptS: number | null; // range / closing speed
}

// Gateway-derived, per-target allocation status (§6.2):
export type AllocationStatus =
  | { kind: "UNENGAGED" }
  | { kind: "ENGAGED"; byDrone: number }
  | { kind: "CAPTURED" };          // non-kinetic terminal state (not "destroyed")

export interface TargetDerived {
  targetId: number;
  speed: number | null;
  altitude: number;
  allocation: AllocationStatus;
}

// Standard ROS2 log (rcl_interfaces/msg/Log) mapped for the grid:
export type LogLevel = "DEBUG" | "INFO" | "WARN" | "ERROR" | "FATAL";
export interface LogRecord {
  stamp: number;
  level: LogLevel;                // from numeric 10/20/30/40/50
  source: string;                 // Log.name (node/logger name)
  msg: string;
  file?: string;
  func?: string;
  line?: number;
}

// The compact per-frame snapshot the gateway emits (scene + grids):
export interface FrameSnapshot {
  stamp: number;
  drones: DroneStatus[];
  tracks: TargetTrack[];
  assignments: Assignment[];
  estimates: Record<number, TargetTrack>; // by drone
  events: CaptureEvent[];                  // since last frame
  dronesDerived: DroneDerived[];           // interceptor-grid fields
  targetsDerived: TargetDerived[];         // target-grid fields
}

// Logs travel on a SEPARATE channel from FrameSnapshot so a log burst
// cannot stall the scene; delivered as batches, capped client-side by a ring buffer:
export interface LogBatch {
  records: LogRecord[];
}

// Recording session metadata (manifest.json), surfaced in the RecordingBrowser:
export interface RecordingManifest {
  sessionId: string;              // e.g. "2026-05-30T14-03-12_run-01"
  startedAt: number;
  stoppedAt: number | null;       // null while still recording
  durationS: number | null;
  scenario?: string;              // scenario name/datum if known
  datum: { lat: number; lon: number; alt: number };
  droneCount: number;
  targetCount: number;
  stateSegments: SegmentRef[];    // for time-range seeking
  logSegments: SegmentRef[];
  hasRosbag2: boolean;
}
export interface SegmentRef {
  file: string;                   // relative path within the session dir
  startStamp: number;
  endStamp: number;
  rows: number;
}

// Scenario catalog + simulation run control (mirrors mics_msgs §6.7):
export interface ScenarioInfo {
  scenarioId: string;
  name: string;
  description: string;
  targetSource: "internal" | "external";
  defenderCount: number;
  attackerCount: number;
  sensorMode: "ideal" | "stub" | "full";
  // server-side `path` is intentionally NOT exposed to the client
}

export type SimRunState =
  | "IDLE" | "LAUNCHING" | "RUNNING" | "STOPPING" | "STOPPED" | "ERROR";

export interface SimRunStatus {
  stamp: number;
  state: SimRunState;
  scenarioId: string;
  runId: string;                  // "" when IDLE
  elapsedS: number;
  dronesUp: number;
  targetsUp: number;
  recordingSessionId: string;     // "" if not recording
  requestedRtf: number;           // 0 = as fast as possible
  actualRtf: number;              // achieved; may be < requested when compute-bound
  rtfControllable: boolean;       // false on HITL/real hardware (locked at 1.0)
  message: string;
}

// Browser → gateway run request (gateway maps to a RunScenario action goal):
export interface RunScenarioRequest {
  scenarioId: string;             // must be a catalog id, not a path/command
  overridesYaml?: string;         // optional, schema-validated overlay
  record: boolean;                // auto-start a recording for the run
  requestedRtf?: number;          // initial real-time factor; 0 = max (SITL only)
}

// Runtime speed change (gateway proxies to SetSimSpeed.srv):
export interface SetSimSpeedRequest {
  requestedRtf: number;           // 0 = as fast as possible
}
```

The log channel is intentionally decoupled from `FrameSnapshot`: the scene/grids tick at the snapshot rate, while logs arrive as independent batches that the Process-log grid appends into a bounded ring buffer.

---

## 9. Performance & scaling strategy

- **One state update per render frame** (gateway snapshot @ 20–30 Hz), not per ROS message.
- **Point-cloud budget**: decimate to a max point count for live; switch to streamed 3D Tiles beyond a threshold; allow per-drone LiDAR toggle.
- **Primitives over entities** for radar/FOV/LiDAR (see §7).
- **LOD / culling**: rely on Cesium frustum culling; hide off-screen sensor volumes.
- **Backpressure**: gateway drops stale snapshots rather than queueing; the browser never falls behind real time.
- **Layer gating**: heavy layers default off; user enables as needed.

---

## 10. Security

The viewer is **read-only by default** and needs no special privileges in that mode. If operator controls (§4.7) are enabled:
- Transport upgraded to **WSS**; client presents an auth token; gateway authorises before forwarding any command to the GS.
- Controls are visually segregated, individually confirmed, and audit-logged.
- The viewer never bypasses GS-side safety interlocks — it only requests the same actions the GS already exposes (arm/disarm-all, abort/RTL-all, pause-allocation).

---

## 11. Development environment setup

### 11.1 Prerequisites
- **Node.js ≥ 20** + a package manager (pnpm recommended).
- The MICS sim/stack running (or a recorded rosbag for replay-only dev).
- `ros-<distro>-rosbridge-suite` available in the ROS2 environment.

### 11.2 Bring up the data plane
```bash
# In the ROS2 environment (host or container), start the WebSocket bridge:
ros2 launch rosbridge_server rosbridge_websocket_launch.xml   # default ws://localhost:9090

# (Recommended) start the viewer-gateway, which subscribes to ROS topics
# and serves compact snapshots + handles ENU→geodetic + decimation:
cd viewer-gateway && pnpm install && pnpm start                # default ws://localhost:8080
```

### 11.3 Run the frontend
```bash
cd mics-view
pnpm install
# configure connection + datum
cp .env.example .env     # set VITE_WS_URL, VITE_DATUM_LAT/LON/ALT, asset paths
pnpm dev                 # Vite dev server, e.g. http://localhost:5173
```

### 11.4 Self-hosted assets (offline/air-gapped)
- Host terrain/imagery locally (e.g. a quantized-mesh terrain tileset + raster imagery served over HTTP) and point Cesium at the local URLs.
- No Cesium Ion token required when self-hosting; do **not** bake any Ion token into builds for air-gapped deployments.

### 11.5 Laptop/CPU-only note
The viewer runs in the browser's own WebGL context and is independent of the headless, GPU-less sim host (MICS PRD §4.5). For a handful of drones it runs fine on integrated graphics. Use **replay mode against a rosbag** to develop the viewer without running the full sim.

---

## 12. Testing plan

- **Unit**: coordinate transforms (ENU↔geodetic round-trip), covariance→ellipsoid mapping, snapshot decoding.
- **Component**: each scene module renders correctly from a fixture `FrameSnapshot`.
- **Integration (replay)**: load a known rosbag-derived CZML; verify entities, sensor volumes, assignments, and events match expected timeline.
- **Integration (live)**: against SITL in `ideal`/`stub` sensor mode; verify latency (NFR-2) and smooth interpolation.
- **Load**: 8 drones + 4 targets + all layers; confirm ≥30 FPS (NFR-1) and gateway backpressure behaviour.
- **Security** (if controls enabled): unauthorised command rejected; confirmations enforced; actions audit-logged.

---

## 13. Milestones

| Phase | Deliverable | Exit criteria |
|---|---|---|
| **V0** | Cesium scene + rosbridge direct, 1–2 drones | Defenders move live on the globe |
| **V1** | Entities + cues + assignments + roster/detail panels | Full coordination picture live |
| **V2** | Sensor layers (radar cones, FOV frustums, fused tracks + covariance) | Sensor coverage vs target visible |
| **V3** | LiDAR point clouds (decimation/3D Tiles) + layer gating | Point clouds render within FPS budget |
| **V4** | viewer-gateway (throttle/transform/snapshot) | 8 drones + all layers ≥30 FPS |
| **V5** | Recording subsystem (gateway-owned: Start/Stop, session dirs, rolling JSONL state+logs, manifest) + RecordingBrowser + replay over the same socket + timeline scrub | Record a run incl. logs; select it; replay reconstructs scene+grids+logs |
| **V6** | Gated operator controls (optional) | Authed abort/RTL/pause with audit log |
| **VS** | Scenario control: `mics_sim_orchestrator` node + `RunScenario` action + `SimRunStatus` topic + `SetSimSpeed` service + `ScenarioPanel` (list/run/stop/status/sim-speed), auto-record on run | Pick a catalog scenario, launch it, set/adjust RTF (SITL), watch requested-vs-achieved speed, stop it; run auto-records incl. logs; all gated |

---

## 14. Risks & open questions

- **Coordinate datum mismatch** — the top integration bug; single source of truth in config, validated by a round-trip unit test (§12).
- **High-rate firehose** — mitigated by the gateway snapshot model; prototype without it only at small scale.
- **Point-cloud volume** — live LiDAR can exceed budget; decimation + 3D Tiles + per-drone toggle.
- **Frustum/cone orientation correctness** — body→ECEF orientation is fiddly; verify visually against known geometry early.
- **Recording disk growth** — long/high-rate sessions accumulate; mitigated by gzip + rolling segments. **Open:** retention policy (auto-prune oldest, max total size, or manual only)? **Open:** record decimated LiDAR into sessions or keep heavy point clouds in rosbag2 only?
- **Process-control safety** — with the **always-on** model the orchestrator commands the running sim over ROS2 and holds **no `docker.sock`** (no host/daemon access), which removes the command-injection / root-equivalent risk; residual controls are auth gating, a vetted catalog, and schema-validated overrides. Trade-off: sim containers consume resources while idle, and fleet-composition changes (replica count, PX4↔ArduPilot) require a redeploy. **Open:** auto-suspend idle sim containers to reclaim resources? **Open:** orchestrator hard-disabled on any host where real hardware could be present.
- **Resolved:** replay is the gateway re-emitting its recorded JSONL stream over the same socket (single live/replay code path); CZML is export-only. **Open:** do operator controls (incl. Start/Stop recording) live here or in a separate hardened GS console? **Open:** multi-operator concurrent viewing — read-only is trivial, but control/recording arbitration needs a policy (who can Start/Stop).

---

## Appendix A — Viewer config (illustrative)

```yaml
connection:
  ws_url: ws://localhost:8080      # viewer-gateway (or :9090 for direct rosbridge)
  use_gateway: true
  wss: false                       # true when operator controls enabled
datum:                             # single source of truth for ENU origin
  lat: 32.0853
  lon: 34.7818
  alt: 30.0
assets:
  terrain_url: http://localhost:8000/terrain   # self-hosted, no Ion token
  imagery_url: http://localhost:8000/imagery
layers:                            # default visibility
  defenders: true
  targets: true
  cues: true
  assignments: true
  radar: false
  fov: false
  lidar: false
  fused_tracks: true
  volumes: true
  truth_debug: false
performance:
  snapshot_rate_hz: 25
  lidar_max_points: 50000
  lidar_use_3dtiles_above: 200000
grids:
  interceptor: { enabled: true }
  target:      { enabled: true }
  process_log:
    enabled: true
    source_topic: /rosout          # ROS2 standard rcl_interfaces/msg/Log
    min_level: INFO                # gateway drops below this severity
    ring_buffer_rows: 20000        # bounded; oldest evicted
    batch_rate_hz: 10              # log channel, separate from snapshot
    auto_scroll: true
controls:
  enabled: false                   # read-only unless explicitly enabled + authed
recording:
  enabled: true
  dir: ./recordings                # one subdirectory created per Start/Stop session
  session_name_template: "{iso}_run-{seq}"
  segment_rotate_mb: 64            # roll a segment at this size...
  segment_rotate_seconds: 300      # ...or this duration, whichever first
  compress: gzip                   # per-segment compression
  include_logs: true               # logs are always recorded (FR-V-40)
  also_record_rosbag2: false       # optional canonical raw layer
  czml_export: on_demand           # never | on_demand
scenarios:                         # simulation orchestration (§6.7)
  enabled: true
  catalog_dir: ./scenarios         # vetted scenario YAMLs; only these are runnable
  orchestrator_action: run_scenario
  status_topic: /sim_run_status
  auto_record_on_run: true         # bind each run to a recording session
  allow_overrides: true            # schema-validated overlays only (e.g. laptop profile)
  default_rtf: 1.0                 # initial real-time factor at launch (SITL)
  allow_runtime_speed: true        # expose SetSimSpeed control (auto-disabled on HITL/real)
```

## Appendix B — Build vs reuse

- **Reuse:** CesiumJS, Resium, roslib.js, rosbridge_suite, Vite, a state lib (Zustand), **AG Grid (Community/MIT)** for the grids (optionally **Glide Data Grid** for an extreme log firehose).
- **Build new:** `viewer-gateway` (throttle/transform/snapshot + derived grid fields + `/rosout` aggregation + **recorder/replayer** + **scenario orchestration proxy**), `mics_sim_orchestrator` node (sim-host), all `scene/*` modules, `ui/*` panels including `ui/grids/*`, `RecordingControls`/`RecordingBrowser`/`ScenarioPanel`, `data/types.ts` contracts, self-hosted asset serving.
- **Shared with MICS:** message schemas (`mics_msgs`, now incl. `RunScenario.action`, `SimRunStatus.msg`, `ScenarioInfo.msg`, `SetSimSpeed.srv`), topic names, the standard `/rosout` log topic, the scenario catalog (MICS PRD Appendix A), datum definition, non-kinetic framing.
