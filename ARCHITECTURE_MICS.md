# Architecture — Multi-Drone Cooperative Interception System (MICS)

**Version:** 0.1 (draft)
**Status:** For engineering review
**Derived from:** [PRD_multidrone_interception.md](PRD_multidrone_interception.md) · [PRD_viewer_architecture.md](PRD_viewer_architecture.md)
**Last updated:** 2026-05-30

---

## 1. Architecture goals & principles

| Goal | Principle |
|---|---|
| **Reuse over rebuild** | Sit on top of the aerial-autonomy-stack (AAS). Extend three packages, build the rest as new ROS2 packages — never fork the harness. |
| **Sim-first, sim-to-real** | Same ROS2 code runs in SITL and on the Jetson. The boundary is the autopilot/sensor drivers, nothing else. |
| **CPU-developable** | The behavior-defining layers (state machine, fusion math, allocation, safety) run CPU-only. GPU is an *accelerator* for rendering + YOLO, never a hard dependency for P0–P4. |
| **Decoupled by transport** | Zenoh between vehicles and GS (RF-friendly, lossy-tolerant); ROS2 DDS strictly *within* a vehicle. No cross-vehicle DDS. |
| **Safety is non-negotiable** | The `safety` node sits at the highest priority on every drone and can veto any actuation, including capture. |
| **Source-agnostic cueing** | Internal sim and external sensor feeds normalize to one `TargetTrack`. The autonomy never knows or cares which produced a cue. |
| **One choke point to the browser** | The `viewer-gateway` is the single boundary between the ROS2/Zenoh bus and the browser. It throttles, transforms (ENU→geodetic), records, proxies scenario control, and is the auth boundary. The browser never speaks DDS/Zenoh and only one host port (8080, WebSocket) is exposed. |
| **GPU-less host, GPU-only browser** | MICS-View renders on browser WebGL, so the always-on sim/GS host needs no GPU for visualization. Rendering load lives on the operator's machine. |
| **Standard logging** | All processes log via ROS2 `/rosout` (`rcl_interfaces/msg/Log`). No custom log message; the gateway aggregates and forwards as `LogBatch`. |

---

## 2. System context (C4 — level 1)

```
        ┌─────────────────────┐         ┌──────────────────────────┐
        │ External detection  │  UDP/   │                          │
        │ source (radar/RF or │  JSON / │      GROUND STATION       │
        │ 3rd-party sim)      │──Zenoh─▶│  (operator + autonomy bus)│
        └─────────────────────┘         └────────────┬─────────────┘
                                                      │ Zenoh (WAN/RF)
        ┌─────────────────────┐                       │
        │ Internal attacker   │   ground-truth        │
        │ simulator (Gazebo)  │──(sensor models +     │
        └─────────────────────┘    scoring only)      │
                                          ┌───────────┼───────────┐
                                          ▼           ▼           ▼
                                      ┌───────┐   ┌───────┐   ┌───────┐
                                      │Drone 1│   │Drone 2│...│Drone N│
                                      └───────┘   └───────┘   └───────┘
```

**Actors:** operator (browser — **MICS-View**, via the `viewer-gateway` over WebSocket), external detection source, attacker UAV(s) (simulated), defender drones (the MICS fleet).

---

## 3. Container view (C4 — level 2)

### 3.1 Ground Station

| Container | Responsibility | Pub / Sub | Build state |
|---|---|---|---|
| `mics_target_ingest` | Normalize internal + external sources → `TargetTrack`; host `cue_degrader` in internal mode | sub: ext UDP/Zenoh, `/attacker/*/truth`; pub: `/tracks_raw` | NEW |
| `mics_track_manager` | Associate, age-out (TTL), and fuse partial tracks into one picture | sub: `/tracks_raw`; pub: `/tracks` (≥5 Hz) | NEW |
| `mics_allocator` | Maintain roster, compute drone→target assignments, reassign on failure | sub: `/tracks`, `/state_sharing_*`; pub: `/assignments` | NEW |
| `mics_monitor` | Backend of the operator UI; **frontend is MICS-View** (browser, see §14). Owns the server side of situational awareness + operator commands | sub: all; pub: operator commands | NEW |
| `viewer-gateway` | The choke point to the browser: per-frame `FrameSnapshot`, ENU→geodetic transform, `/rosout` aggregation → `LogBatch`, recording/replay, scenario-run proxy to the orchestrator, auth boundary | sub: all bus topics + `/rosout` + `/sim_run_status`; pub (WS): snapshots/logs/status; proxies `RunScenario`/`SetSimSpeed` | NEW |
| `ground_system` | AAS `/tracks` plumbing | — | EXTENDED |

### 3.2 Per-drone (runs on each Jetson / SITL instance)

| Container | Responsibility | Build state |
|---|---|---|
| `autopilot_interface` | High-level actions (Takeoff/Orbit/Offboard/Land) via MAVLink | AAS reuse |
| `offboard_control` | Setpoint stream — **add PN guidance mode** | EXTENDED |
| `yolo_py` | Camera → YOLO → bearing + class | AAS reuse |
| `radar_driver` | Radar detections → ROS2 (sim plugin or real radar) | NEW |
| `lidar_driver` | LiDAR point cloud | AAS/std |
| `mics_fusion` | EKF/UKF over camera+radar+LiDAR → `TargetEstimate{pose,twist,cov,quality}` | NEW |
| `mics_terminal` | Acquisition, terminal PN pursuit, capture predicate + trigger | NEW |
| `mission` | Hosts the MICS state machine; reads `/assignments` | EXTENDED |
| `state_sharing` | Publishes `/state_sharing_drone_N` (pose+status+assignment) | AAS reuse |
| `safety` | Geofence, RTL, kill switch, capture interlocks — always-on, top priority | NEW |
| `mics_msgs` | `TargetTrack`, `Assignment`, `DroneStatus`, `CaptureEvent`, `ScenarioInfo`, `SimRunStatus`, `RunScenario`, `SetSimSpeed` | NEW |

### 3.3 Simulation host (always-on containers)

| Container | Responsibility | Pub / Sub | Build state |
|---|---|---|---|
| `gazebo_sim` + `sim_control` | Gazebo Harmonic world + faster-than-real-time stepping; reset/spawn/RTF/start over ROS2 | sub: sim_control cmds; pub: sim sensor streams, truth | EXTENDED |
| `attacker_sim` | Spawns targets, drives trajectories, emits ground truth (sensor models + scoring only) | pub: `/attacker/*/truth` | NEW |
| `mics_sim_orchestrator` | Commands the **always-on** sim over ROS2 (reset/spawn/RTF/start) for a run from the **vetted catalog**. Exposes `RunScenario` (action) + `SetSimSpeed` (srv); publishes `SimRunStatus`. **No `docker.sock`** — it controls the sim graph, not containers | sub: catalog; pub: `/sim_run_status`; srv/action: `run_scenario`, set-speed | NEW |

The orchestrator deliberately has **no Docker control plane**: containers are brought up once (always-on) and a "run" is a sim reset+spawn+start, never a container lifecycle action. This keeps the attack surface of operator-triggered runs inside the sim graph.

---

## 4. Runtime data flow

```
 attacker truth ──▶ sensor models ──▶ [camera|radar|lidar topics] ──▶ mics_fusion
                                                                         │
 GS cue (/tracks) ──────────────────────────────────────────┐           │ TargetEstimate
                                                             ▼           ▼
                                                          mission ◀── (track source switch
                                                             │          on handoff)
                              /assignments ──▶ mission ─────▶│
                                                             ▼
                                                       mics_terminal ──▶ offboard_control ──▶ autopilot
                                                             │
                                                       CaptureEvent ──▶ safety (interlock gate) ──▶ payload
                                                             │
 state_sharing ◀── every state transition ──────────────────┘
        │
        ▼ Zenoh
   mics_allocator (reassign on FAILED/LOST within ≤1 s)
```

**Track-source switch (handoff):** `mission` consumes the GS cue during MIDCOURSE; once `mics_fusion` reports `quality > Q` for `T_lock` seconds, the primary track source flips to onboard `TargetEstimate` and the drone enters TERMINAL.

---

## 5. Onboard state machine

```
IDLE
 └─(assignment received)─▶ ASSIGNED
       └─(takeoff/clearance)─▶ MIDCOURSE        track source = GS cue
             └─(onboard quality > Q for T_lock)─▶ ACQUIRING ─▶ handoff
                   └─(stable onboard track)─▶ TERMINAL       PN pursuit
                         ├─(capture geometry & interlocks)─▶ CAPTURED ─▶ report, RTL ─▶ IDLE
                         └─(track lost>T_lost | miss | no confirm)─▶ FAILED ─▶ release, RTL ─▶ IDLE
```

Every transition is broadcast on `/state_sharing_drone_N` so the allocator stays synchronized.

---

## 6. Coordination & allocation design

- **Roster:** built live from `/state_sharing_*` (pose, state, battery, track quality).
- **Cost function:** estimated time-to-intercept = range / closing speed, adjusted for drone state/battery.
- **Solver:** greedy nearest-available (v0) → **Hungarian algorithm** for optimal one-to-one (v1).
- **Constraints:** one target per drone unless `redundancy>1`; honor `standby` role; respect deconfliction corridors.
- **Reassignment:** triggered by a drone entering `FAILED`/`LOST` or a higher-priority target appearing; recompute over *available* drones only; republish within **≤1 s** (FR-ALLOC-4).
- **Anti-thrash:** add hysteresis / commit-time to prevent oscillation under noisy cues (see Risks).
- **Deconfliction:** the pursuing drone owns an engagement corridor; others maintain separation using shared positions.

---

## 7. Sensing & fusion architecture

| Sensor | Output | Fusion role |
|---|---|---|
| Camera (YOLO) | bearing + class | bearing/elevation update |
| Radar (NEW) | range + radial velocity + coarse bearing | range + range-rate update; survives glare/dark |
| LiDAR | precise 3D relative position (close range) | direct position update |

- **Estimator:** EKF (UKF if nonlinearity hurts), per-sensor measurement models with validity gating.
- **State:** target position + velocity in the drone's local frame.
- **Output:** `TargetEstimate{pose, twist, covariance, quality}` — `quality` drives handoff and failure logic.
- **Graceful degradation (FR-SENS-5):** maintain a usable track if any single sensor drops (e.g. radar-led when camera is degraded).

---

## 8. Transport & interface boundaries

| Boundary | Transport | Rate |
|---|---|---|
| Within a vehicle (node↔node) | ROS2 DDS | per-node |
| Drone ↔ GS, drone ↔ drone | Zenoh (RF/WAN bridge) | `/tracks` ≥5 Hz, `/state_sharing` ≥10 Hz |
| External detection ingress | UDP/JSON datagram **or** Zenoh/ROS2 bridge | source-defined |
| `viewer-gateway` ↔ browser | **WebSocket** (`:8080`, only host-exposed port) — `FrameSnapshot` / `LogBatch` / control | snapshots ≤ render budget, coalesced |
| Logging (all processes) | ROS2 `/rosout` (`rcl_interfaces/msg/Log`); cross-machine via `zenoh-bridge-ros2dds` | as-emitted, batched by gateway |
| Sim control (gateway ↔ orchestrator) | ROS2 action/srv (`RunScenario`, `SetSimSpeed`), status on `/sim_run_status` | on-demand / status ≥1 Hz |

**Message schemas** (`mics_msgs`): `TargetTrack`, `Assignment`, `DroneStatus`, `CaptureEvent`, plus the orchestration set `ScenarioInfo` / `SimRunStatus` / `RunScenario` / `SetSimSpeed` — defined in PRD §8 / §8.7. External UDP/JSON maps onto `TargetTrack`. Logging reuses the standard `rcl_interfaces/msg/Log` — **no custom log message**. The browser never speaks DDS/Zenoh; the gateway is the only translation point.

---

## 9. Safety architecture

The `safety` node is **always-on and highest priority** on every drone:
- Hard geofence → auto-RTL on breach.
- Per-drone arm/disarm; global kill switch from GS.
- **Capture interlock:** payload will not fire outside capture geometry or near a teammate (`capture_min_teammate_sep_m`).
- Low-battery RTL.
- Capture actuation is abstracted behind one interface (sim: log/scoring; HW: payload driver). **No kinetic logic anywhere.**

---

## 10. Deployment view

```
Docker-first — single user-defined network, containers always-on, one zenoh-router hub.
A "run" is a sim reset, not a container start.

┌─ simulation container ─┐  ┌─ ground container ─┐  ┌─ aircraft container ×N ─┐
│ Gazebo Harmonic        │  │ GS ROS2 nodes      │  │ PX4/AP SITL + onboard   │
│ PX4/AP SITL spawns     │  │ mics_* GS pkgs     │  │ ROS2 graph (mics_*)     │
│ attacker + sensor sim  │  │ mics_monitor (bk)  │  │ one per defender        │
│ mics_sim_orchestrator  │  │ viewer-gateway ────┼──┐                         │
└───────────┬────────────┘  └─────────┬──────────┘  │  └─────────────────────┘
            │                         │             │ WebSocket :8080
            └──────── zenoh-router (tcp/7447, internal) ───────┘   (only host-exposed port)
                                      │
                                      ▼
                        ┌─────────────────────────────┐
                        │ Browser — MICS-View (WebGL)  │  ← operator's machine, GPU here
                        └─────────────────────────────┘

HITL (P6): aircraft container → physical Jetson Orin running yolo_py + mics_fusion.
Live (P8): authorized range only, real autopilot + payload driver.
```

**Topology rules:**
- All containers join one user-defined Docker network and rendezvous through a single `zenoh-router` (`tcp/7447`, **internal only**).
- **Only `:8080` (the gateway WebSocket) is published to the host.** No DDS/Zenoh port is exposed; the browser cannot reach the bus directly.
- Containers are **always-on**; scenario runs are sim resets driven by `mics_sim_orchestrator`, never container lifecycle events (no `docker.sock` anywhere).
- The sim/GS host needs **no GPU for visualization** — MICS-View renders on the operator's browser. GPU on the host is only for `full` perception (YOLO/render).

**`sensor_mode` switch** (same scenarios, different hardware):
- `ideal` — near-truth detections, no render/YOLO. **CPU default, P1–P2.**
- `stub` — `stub_detector` adds realistic noise, no render/inference. CPU fusion testing, P3–P4.
- `full` — Gazebo camera render + YOLO + radar + LiDAR. GPU recommended.

---

## 11. Build-vs-reuse map

| Category | Components |
|---|---|
| **Reuse (AAS / std as-is)** | Docker/tmux harness, faster-than-real-time multi-vehicle Gazebo SITL, PX4/AP SITL, `autopilot_interface`, `state_sharing` (Zenoh), `yolo_py` + camera/LiDAR sim, Gymnasium wrapper, Jetson path, **ROS2 `/rosout` (`rcl_interfaces/msg/Log`)**, `zenoh-bridge-ros2dds`, **CesiumJS + Resium + rosbridge_suite/roslib.js** (MICS-View deps) |
| **Extend** | `mission` (state machine), `offboard_control` (PN guidance), `ground_system` (`/tracks`), `gazebo_sim`/`sim_control` (reset/spawn/RTF hooks) |
| **Build new** | radar sim + `radar_driver`, `mics_fusion`, `mics_terminal` (+capture interface), `mics_target_ingest` (+degrader), `mics_track_manager`, `mics_allocator`, `mics_monitor`, **`viewer-gateway`**, **`mics_sim_orchestrator`**, `mics_msgs` (incl. orchestration set), `safety`, attacker models, **MICS-View frontend** (companion deliverable, see PRD_viewer_architecture.md) |

---

## 12. Phasing → architecture mapping

| Phase | Architectural slice activated |
|---|---|
| **P0** | Deployment view: baseline AAS SITL up, faster-than-real-time confirmed |
| **P1** | Coordination + allocation (§6) + state machine (§5) with `ideal` sensors |
| **P2** | Attacker sim + `cue_degrader` in `mics_target_ingest` (§3.1) |
| **P3** | Sensing & fusion (§7) — EKF over camera+radar+LiDAR, graceful degradation |
| **P4** | Terminal pursuit + capture + safety interlocks (§9) |
| **P5** | External-source ingress (§8) drives system unchanged |
| **P6** | HITL deployment (§10) — real-time perception on Orin |
| **P7** | Gym env / policy learning (optional) |
| **P8** | Constrained live flight on authorized range |
| **V0–V6, VS** | MICS-View (§14) brought up in parallel against recorded rosbags then live: V0 scene from a rosbag → grids/logs → recording/replay → scenario run/stop + sim-speed (VS). See PRD_viewer_architecture.md §13 |

---

## 13. Key architectural risks

| Risk | Architectural mitigation |
|---|---|
| Radar sim fidelity (no AAS plugin) | Isolate behind `radar_driver` interface so sim model can be swapped without touching fusion |
| Sim-to-real gap | Same ROS2 code sim+HW; only drivers differ; Jetson HITL gate at P6 |
| Comms under RF stress | Zenoh-only across vehicles; deconfliction tolerant of dropouts; comms-in-the-loop test |
| Allocation thrash | Hysteresis / commit-time in `mics_allocator` |
| Capture safety | Hard interlock in `safety`, separate from `mics_terminal`; 0 false-fire required in sim |
| ENU↔geodetic datum mismatch | Single shared ENU datum in config, owned by `viewer-gateway`; one transform point so sim, GS, and viewer cannot drift |
| Topic firehose to browser | Gateway coalesces to per-frame `FrameSnapshot` + batched `LogBatch` under a render budget; browser never subscribes to raw bus |
| Operator process-control safety | Run/stop + sim-speed are auth-gated and act on **simulation only**, proxied through `mics_sim_orchestrator` (no `docker.sock`, no real-aircraft path) |
| Single exposed port as attack surface | Only `:8080` WS published; auth boundary at the gateway; operator/control messages gated server-side |

---

## 14. Visualization & sim-control layer (MICS-View)

The operator-facing UI is split into a backend (`mics_monitor` + `viewer-gateway`, on the GS) and a browser frontend (**MICS-View**). The frontend is owned by [PRD_viewer_architecture.md](PRD_viewer_architecture.md); the architecture-relevant boundaries are:

- **Gateway as sole boundary** — `viewer-gateway` is the only path between the ROS2/Zenoh bus and the browser. It (1) coalesces bus topics into per-frame `FrameSnapshot`, (2) transforms ENU→geodetic at one point against the shared datum, (3) aggregates `/rosout` into `LogBatch`, (4) records to disk and serves replay, (5) proxies `RunScenario`/`SetSimSpeed` to `mics_sim_orchestrator`, and (6) is the auth boundary. Transport is WebSocket on `:8080` (§8, §10).
- **Frontend** — CesiumJS + Resium 3D scene (entities, sensor volumes, point clouds), high-performance grids (interceptor/target/process-log), and panels (ScenarioPanel, Recording, gated Operator). Renders on browser WebGL — independent of the GPU-less sim host.
- **Live vs replay symmetry** — the frontend consumes the same `FrameSnapshot`/`LogBatch` stream whether live from the bus or replayed from a recording; the gateway is the switch.
- **Recording** — `recordings/<session>/` holds state, logs, manifest, and rosbag2, written by the gateway; replay reads it back through the same wire protocol.

This layer adds no autonomy behaviour and cannot actuate aircraft; its only write path is auth-gated sim control.

---

## Open questions (carried from PRD §11)

- Target prioritization policy when targets outnumber interceptors?
- Capture-confirm sensing method?
- Standby-drone commit threshold?
