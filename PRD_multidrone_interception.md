# Product Requirements Document — Multi-Drone Cooperative Interception System (MICS)

**Version:** 0.1 (draft)
**Status:** For engineering review
**Owner:** _[fill in]_
**Last updated:** 2026-05-30

---

## 0. Scope, framing, and non-goals

**System framing.** MICS is a **counter-UAV (C-UAV) system using non-kinetic, capture-based interception** (net capture / physical entanglement / "drone-catcher" payloads), in the same class as systems like Fortem DroneHunter or DroneCatcher. All terminal behaviour in this document terminates at **pursuit + capture**. The "engagement" is a payload trigger (e.g. net launch) fired when capture geometry is satisfied.

**Explicit non-goals (out of scope for this PRD):**
- Kinetic / explosive payloads or any munition guidance.
- Optimisation of lethal terminal effects.
- Operation outside a controlled test range or simulation without the appropriate airspace authorisation.
- Defeating cryptographic links or jamming of third-party systems.

**In scope:** the autonomy, multi-drone coordination, perception/sensor-fusion, simulation environment, dev tooling, and ground-station software needed to detect-cue → allocate → pursue → acquire → attempt capture → reassign-on-failure.

> **Compliance gate:** Real-flight testing of C-UAV systems is heavily regulated. This PRD assumes all live testing happens on an authorised range with the operator holding the relevant approvals. Simulation-only work has no such constraint and is where the bulk of development happens.

---

## 1. Overview

MICS is built as an extension of the **aerial-autonomy-stack (AAS)** open framework (PX4/ArduPilot + ROS2 + YOLO + LiDAR + Jetson, Dockerised, with faster-than-real-time multi-vehicle Gazebo SITL and a Gymnasium wrapper). We reuse AAS's coordination plumbing and simulation harness and add three new capability blocks: **multi-sensor target fusion**, **cooperative task allocation/reassignment**, and **terminal pursuit + capture**.

### 1.1 Operational concept (CONOPS)

1. An **external** detection system (or the **internal** simulator) publishes **partial target tracks** to the **Ground Station (GS)**.
2. GS maintains a fused track picture and a roster of available interceptors, then **assigns** interceptors to targets.
3. Each assigned interceptor flies a **midcourse** leg toward the cued (uncertain) target region.
4. When close, the interceptor **switches to its own sensors** (camera + radar + LiDAR), forms a high-quality onboard track, and runs **terminal pursuit**.
5. On capture-geometry satisfaction, it triggers the **capture payload**.
6. On **failure** (track lost, miss, no capture-confirm), the interceptor releases the assignment; GS **reassigns** to the next-best available teammate.

### 1.2 High-level architecture

```
            ┌──────────────────────────────────────────────────────────┐
 EXTERNAL   │                     GROUND STATION                        │
 TARGET ───▶│  target_ingest → track_manager → task_allocator → monitor │
 SOURCE     │            (publishes /tracks + /assignments via Zenoh)    │
            └───────────────┬──────────────────────────────────────────┘
                            │  Zenoh (WAN/RF bridge)
        ┌───────────────────┼───────────────────────────┐
        ▼                   ▼                            ▼
 ┌────────────┐      ┌────────────┐                ┌────────────┐
 │ Drone 1    │      │ Drone 2    │      ...        │ Drone N    │
 │ (Jetson)   │      │ (Jetson)   │                │ (Jetson)   │
 │  ROS2 nodes│      │  ROS2 nodes│                │  ROS2 nodes│
 │  + PX4/AP  │      │  + PX4/AP  │                │  + PX4/AP  │
 └────────────┘      └────────────┘                └────────────┘
   drones share /state_sharing_drone_N (pose + status + assignment) over Zenoh
```

### 1.3 Internal vs external target source

The system **must support both**, switchable by config (`target_source: internal | external`):

- **Internal simulator**: an attacker-trajectory generator node inside the sim that publishes ground-truth attacker states, then degrades them (noise, dropout, latency) into "partial tracks" before they reach the GS — so the GS sees realistically imperfect cues.
- **External source**: a documented network interface (UDP/JSON or ROS2/Zenoh bridge) that a third-party detection simulator or real radar/RF sensor feeds. The GS's `target_ingest` node normalises both into the same internal `TargetTrack` message.

---

## 2. Definitions

| Term | Meaning |
|---|---|
| **Attacker / target** | The UAV to be intercepted (simulated). |
| **Defender / interceptor** | A MICS drone. |
| **Cue** | A partial, uncertain target location from GS (not ground truth). |
| **Onboard track** | A target estimate built from the drone's own sensors. |
| **Handoff** | The moment a drone switches its primary track source from GS cue to onboard sensors. |
| **Assignment** | A (drone → target) mapping decided by the allocator. |
| **Capture geometry** | The range/closure/aspect conditions under which the capture payload may fire. |
| **SITL / HITL** | Software-/Hardware-in-the-loop simulation. |

---

## 3. Functional requirements

IDs use `FR-<area>-<n>`. Priority: **M** (must), **S** (should), **C** (could).

### 3.1 Target ingest & track management (GS)
- **FR-TGT-1 (M)** Ingest tracks from internal sim and external interface, normalised to `TargetTrack`.
- **FR-TGT-2 (M)** Each track carries position, velocity (if available), covariance, classification confidence, source ID, and `stamp`.
- **FR-TGT-3 (M)** Maintain track continuity (associate updates, age out stale tracks after configurable TTL).
- **FR-TGT-4 (S)** Fuse multiple partial sources for the same target into one GS track.
- **FR-TGT-5 (M)** Publish `/tracks` over Zenoh at ≥5 Hz.

### 3.2 Task allocation & coordination (GS + drones)
- **FR-ALLOC-1 (M)** Maintain roster of interceptors and their status from `/state_sharing_drone_N`.
- **FR-ALLOC-2 (M)** Assign drones to targets minimising estimated time-to-intercept; never assign two drones to one target unless `redundancy>1` configured.
- **FR-ALLOC-3 (M)** Publish `/assignments`; each drone obeys only its own assignment.
- **FR-ALLOC-4 (M)** On a drone reporting `FAILED` or `LOST`, drop the assignment and re-allocate within ≤1 s.
- **FR-ALLOC-5 (S)** Support a "next-best standby" so a second drone can be pre-positioned without committing.
- **FR-ALLOC-6 (M)** Inter-drone deconfliction: pursuing drone owns an engagement corridor; others maintain separation using shared positions.

### 3.3 Onboard autonomy (per drone)
- **FR-OB-1 (M)** State machine: `IDLE → ASSIGNED → MIDCOURSE → ACQUIRING → TERMINAL → CAPTURED | FAILED → IDLE`.
- **FR-OB-2 (M)** Midcourse: fly toward cue region using high-level autopilot actions (Takeoff/Offboard/Orbit).
- **FR-OB-3 (M)** Acquire: run camera+radar+LiDAR fusion; declare handoff when onboard track quality > threshold for `T_lock` seconds.
- **FR-OB-4 (M)** Terminal: pursuit guidance on the onboard track (proportional navigation).
- **FR-OB-5 (M)** Capture: fire payload only when capture-geometry predicate true AND safety interlocks satisfied.
- **FR-OB-6 (M)** Failure detection: onboard track lost > `T_lost`, miss-distance exceeded, or no capture-confirm → `FAILED`.
- **FR-OB-7 (M)** Broadcast pose + status + current assignment on `/state_sharing_drone_N` at ≥10 Hz.
- **FR-OB-8 (M)** Geofence + RTL + manual-override kill switch always active.

### 3.4 Sensing & fusion (per drone)
- **FR-SENS-1 (M)** Camera object detection (YOLO) → bearing + class.
- **FR-SENS-2 (M)** Radar → range + radial velocity (operates in glare/low-light).
- **FR-SENS-3 (M)** LiDAR → precise 3D relative position at close range.
- **FR-SENS-4 (M)** Fuse the three into a single EKF/UKF target estimate in the drone's local frame with covariance.
- **FR-SENS-5 (S)** Graceful degradation: maintain a usable track if any one sensor drops.

### 3.5 Simulation
- **FR-SIM-1 (M)** Simulate ≥1 attacker with scriptable trajectories (waypoint, evasive, random).
- **FR-SIM-2 (M)** Simulate ≥4 defenders concurrently (target: 8) in one host, faster-than-real-time.
- **FR-SIM-3 (M)** Simulate all three sensor modalities with realistic noise/FOV/range.
- **FR-SIM-4 (M)** Inject cue degradation (latency, dropout, position noise) on the GS feed.
- **FR-SIM-5 (S)** Jetson-in-the-loop: run perception on real Jetson against simulated camera feed.
- **FR-SIM-6 (S)** Gymnasium env exposing the scenario for RL of allocation/pursuit policies.
- **FR-SIM-7 (S)** Scenario orchestration: a `mics_sim_orchestrator` node configures and starts/stops a sim run from a vetted scenario catalog by commanding the **always-on** simulation over ROS2 (no container lifecycle / no `docker.sock`), and publishes run status. Driven by the viewer (see *MICS-View* PRD §6.7–§6.8).
- **FR-SIM-8 (S)** Simulation-speed control: set real-time factor (RTF) at launch and, for pure SITL, adjust it at runtime; report requested vs achieved RTF. Forced to 1× and disabled on HITL/real hardware.

### 3.6 Ground station UX
- **FR-GS-1 (M)** Live map: targets (with uncertainty), interceptors, assignments, states.
- **FR-GS-2 (M)** Operator controls: arm/disarm all, abort/RTL all, pause allocation.
- **FR-GS-3 (S)** Post-run logs + replay (rosbag/ulog).

---

## 4. Development environment setup

> AAS is **Docker-first** (each role runs in a container with a tmux entrypoint). This is the supported path; it isolates the PX4/ArduPilot + ROS2 + Gazebo + Jetson toolchains, which are painful to co-install on bare metal.

### 4.1 Host requirements

| Item | Minimum | Recommended |
|---|---|---|
| OS | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS |
| CPU | 8 cores | 16+ cores (more = more concurrent SITL drones) |
| RAM | 16 GB | 32–64 GB |
| GPU | NVIDIA, 8 GB VRAM (for YOLO + rendering) | RTX 4070+ / workstation GPU |
| Disk | 60 GB free | 150 GB SSD |
| Edge target | — | NVIDIA Jetson Orin (NX/Nano) for HITL |

Core software versions (pin these in your `docker-compose`/Dockerfiles):
- **ROS 2 Humble** (matches the stack's tooling)
- **Gazebo Sim Harmonic** (the AAS sim target)
- **PX4 Autopilot** (SITL) and **ArduPilot** (SITL) — autopilot-agnostic
- **Docker Engine ≥ 24** + **NVIDIA Container Toolkit**
- **Zenoh** (inter-process / RF transport)
- **Ultralytics YOLOv8** (perception)

### 4.2 Host bootstrap

```bash
# 1. Base tooling
sudo apt update && sudo apt install -y \
  git curl build-essential ca-certificates tmux

# 2. Docker Engine
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker "$USER"   # log out/in afterwards

# 3. NVIDIA Container Toolkit (GPU passthrough for YOLO + Gazebo rendering)
distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
# add the toolkit apt list per NVIDIA docs, then:
sudo apt update && sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# 4. Verify GPU is visible inside a container
docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi
```

### 4.3 Clone and build the stack

```bash
git clone https://github.com/JacopoPan/aerial-autonomy-stack.git
cd aerial-autonomy-stack

# Build the role images (simulation, aircraft, ground). Exact targets per repo README.
docker compose build           # or the provided build script

# Smoke test: launch baseline multi-vehicle SITL (no MICS logic yet)
docker compose up simulation
```

You should see Gazebo Harmonic come up with the baseline vehicle model(s) and faster-than-real-time stepping. Confirm before adding anything.

### 4.4 Repository layout (baseline AAS + MICS additions)

```
aerial-autonomy-stack/
├── aas-gym/                         # Gymnasium wrapper (RL of allocation/pursuit)
├── aircraft/aircraft_ws/src/
│   ├── autopilot_interface/         # [AAS] Takeoff / Orbit / Offboard / Land
│   ├── mission/                     # [AAS] action orchestrator  ── EXTENDED for MICS state machine
│   ├── offboard_control/            # [AAS] low-level refs       ── EXTENDED for PN guidance
│   ├── state_sharing/               # [AAS] /state_sharing_drone_N
│   ├── yolo_py/                     # [AAS] camera + YOLO
│   ├── mics_fusion/         # NEW   # camera+radar+lidar EKF/UKF
│   ├── mics_terminal/       # NEW   # acquisition + terminal pursuit + capture trigger
│   └── mics_msgs/           # NEW   # msgs: TargetTrack, Assignment, DroneStatus, CaptureEvent,
│                            #         SimRunStatus, ScenarioInfo; action: RunScenario; srv: SetSimSpeed
├── ground/ground_ws/src/
│   ├── ground_system/               # [AAS] /tracks publisher  ── EXTENDED
│   ├── mics_target_ingest/  # NEW   # internal+external normaliser
│   ├── mics_track_manager/  # NEW   # GS-side track fusion / TTL
│   ├── mics_allocator/      # NEW   # assignment + reassignment
│   ├── mics_monitor/        # NEW   # operator UI backend (frontend = MICS-View)
│   ├── viewer-gateway/      # NEW   # ROS2↔browser bridge (snapshots, logs, recording, scenario proxy)
│   └── mics_sim_orchestrator/ # NEW # launches/stops sim runs; RunScenario action + SetSimSpeed srv
└── simulation/simulation_resources/
    ├── aircraft_models/             # [AAS] X500v2 (PX4), Iris (AP), VTOL models
    ├── attacker_models/     # NEW   # target airframes + sensor signature
    ├── scenarios/           # NEW   # vetted scenario catalog (run from the viewer)
    └── sensors/             # NEW   # radar plugin config (camera+lidar already in AAS)
```

`[AAS]` = reuse as-is. `EXTENDED` = modify. `NEW` = build.

### 4.5 CPU-only / laptop dev profile (no NVIDIA GPU)

A discrete GPU is **not** required for the bulk of development. The GPU only accelerates two things — Gazebo camera **rendering** and **YOLO inference** — and both have CPU fallbacks. Everything that defines the system's behaviour (autopilot SITL, state machine, fusion math, allocation, reassignment, comms, safety) is CPU-only and runs fine on a laptop.

#### What works on CPU vs what to defer

| Capability | CPU-only laptop | Needs GPU (offload) |
|---|---|---|
| PX4/ArduPilot SITL | ✅ full speed | — |
| ROS2 logic: state machine, `mics_fusion`, allocator, track manager, safety | ✅ lightweight | — |
| Coordination layer (alloc/handoff/reassign/deconflict) with ideal sensors | ✅ **primary dev target** | — |
| LiDAR sim | ✅ (CPU ray-casting; reduce point density) | — |
| Gazebo **camera** rendering | ⚠️ software render (Mesa/llvmpipe), slow | ✅ for many-drone, high-rate |
| **YOLO** inference | ⚠️ YOLOv8n at a few FPS | ✅ for 50 Hz / 8-drone perception |
| Faster-than-real-time @ 8 drones with full perception | ❌ drops below real-time | ✅ |
| Jetson-in-the-loop (P6) | uses separate edge HW regardless | — |

**Skip in setup:** the NVIDIA Container Toolkit step in §4.2 (GPU passthrough only). Drop `--gpus all` from any `docker run`/compose GPU reservations.

#### Make it usable on CPU

1. **Run Gazebo headless.** Disable the GUI/client; rely on `mics_monitor` and rosbag replay for visualization. Software rendering of an unseen world is far cheaper.
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1      # force Mesa software GL where needed
   # launch simulation with the GUI client disabled (headless server only)
   ```
2. **Cut the render load.** Keep the camera at the stack default (~320×240 @ 8 Hz) or lower; reduce drone count to **2–4**; thin LiDAR point density.
3. **Use a stub detector instead of YOLO.** A `stub_detector` node emits bounding boxes (or directly a bearing+class) derived from attacker ground truth + configurable noise. This exercises the *entire* fusion → handoff → terminal → allocation pipeline with **zero** inference cost, and is the right tool for P1–P2.
4. **When you do need real perception**, run **YOLOv8n** on CPU for wiring/correctness checks only, at low FPS and few drones — don't expect real-time.
5. **Burst to a GPU when warranted.** For full-rate perception, 8-drone swarms, or many-seed Monte-Carlo runs, rent a cloud GPU VM (x86 + NVIDIA, Ubuntu 22.04) for a few hours rather than buying hardware. Same Docker images, just re-enable the toolkit and `--gpus all`.

#### `sensor_mode` config switch

Add a `sensor_mode` to the scenario YAML so the same scenarios run on a laptop or a GPU box unchanged:

- `ideal` — sensor models emit near-truth detections (no camera render, no YOLO). **Default for CPU dev.**
- `stub` — `stub_detector` provides realistic-noise detections without rendering/inference. For fusion testing on CPU.
- `full` — Gazebo camera render + YOLO + radar + LiDAR. GPU recommended.

#### Hardware caveats

- **Apple Silicon (M-series) Mac:** Docker + Gazebo on ARM is fragile and the stack is x86/Ubuntu-oriented. Expect real friction; prefer a Linux x86 machine or a cloud VM.
- **Intel/AMD laptop on Ubuntu 22.04 with integrated graphics:** the smooth path — no discrete GPU needed for P0–P2 and most of P3–P4.

> **Net effect on the roadmap:** P0–P2 (env, coordination core, attacker sim + cue degradation) are fully laptop-doable. P3–P4 (fusion, terminal) are doable on CPU using `ideal`/`stub` modes. Only `full`-perception runs, large swarms, and P6 Jetson HITL need a GPU/edge device.

---

## 5. Simulation design

### 5.1 What the simulator must produce

| Channel | Source | Consumer |
|---|---|---|
| Attacker ground-truth pose | internal attacker node | sensor models + scoring |
| Degraded cue (`/tracks`) | GS after degradation | interceptors |
| Camera frames | Gazebo camera plugin | `yolo_py` |
| LiDAR scans | Gazebo LiDAR plugin (Livox Mid-360 model) | `mics_fusion` |
| Radar returns | NEW radar plugin/node | `mics_fusion` |
| Defender poses | PX4/ArduPilot SITL | everything |

### 5.2 Attacker (target) simulation — `FR-SIM-1`

A `attacker_sim` node spawns one or more target airframes in Gazebo and drives them along a selectable profile:

- **`waypoint`** — straight legs between points (baseline).
- **`ingress`** — constant-heading run toward a defended asset.
- **`evasive`** — randomised heading/altitude jinks once it detects a pursuer within radius `R_evade` (tests reassignment + terminal robustness).
- **`replay`** — follow a recorded/CSV trajectory (e.g. from an external scenario tool).

The node publishes ground-truth `/attacker/<id>/truth` (used **only** by sensor models and scoring, never by the autonomy directly).

### 5.3 Defender (interceptor) simulation — `FR-SIM-2`

Use AAS's faster-than-real-time multi-vehicle SITL. Each defender is a PX4 (Holybro X500v2) or ArduPilot (Iris) model running its full onboard ROS2 graph. Start with **4** defenders; scale to **8**. Spawn config (count, home positions, autopilot type) lives in a scenario YAML.

### 5.4 Sensor simulation — `FR-SIM-3`

| Sensor | Model | Key sim params | Output |
|---|---|---|---|
| **Camera** | Gazebo camera (AAS already ships ~320×240 @ 8 Hz, ~100° FOV) | resolution, FOV, rate, motion blur, exposure for low-light tests | image → YOLO → bearing+class |
| **LiDAR** | Gazebo 3D LiDAR, Livox Mid-360 profile (AAS) | range, point density, scan rate, dropout | point cloud → relative 3D |
| **Radar** | **NEW** custom Gazebo plugin or ROS2 node | max range, range/velocity resolution, beamwidth, false-alarm rate, min RCS | detections: range + radial velocity + coarse bearing |

Each sensor model derives its measurement from attacker ground truth, then **adds realistic noise, latency, FOV gating, and dropout**. The radar must keep producing useful range/velocity when the camera is degraded (glare/dark scenario), so fusion can demonstrate graceful degradation (`FR-SENS-5`).

### 5.5 Cue degradation — `FR-SIM-4`

A `cue_degrader` (in `mics_target_ingest` when `target_source=internal`) converts attacker truth into **partial** GS tracks:
- position Gaussian noise (config σ, e.g. 5–30 m),
- velocity sometimes absent,
- update dropout (config %),
- latency (config ms),
- occasional false/ghost tracks (optional).

This guarantees interceptors are trained/tested against imperfect cues, matching the real external-source case.

### 5.6 External-source mode

When `target_source=external`, `mics_target_ingest` exposes a documented ingress:
- **UDP/JSON** datagram schema (one object per target per message), or
- a **Zenoh/ROS2 bridge** topic.
Both map onto the internal `TargetTrack`. A third-party detection simulator or a real sensor feed can drive the system unchanged.

### 5.7 Jetson-in-the-loop & Gym — `FR-SIM-5/6`

- **HITL:** route the simulated camera stream to a physical Jetson Orin running `yolo_py` (target 50 Hz YOLOv8) + `mics_fusion`, to validate real-time onboard compute before flight.
- **Gym:** wrap the scenario in the AAS Gymnasium env (synchronous stepping / `AsyncVectorEnv`) to *learn* allocation or terminal-pursuit policies instead of hand-tuning.

---

## 6. Onboard software stack (per drone)

ROS2 graph on each Jetson, alongside the PX4/ArduPilot link.

### 6.1 Nodes

| Node | Pkg | Role |
|---|---|---|
| `autopilot_interface` | AAS | High-level actions to autopilot via MAVLink. |
| `offboard_control` (EXT) | AAS | Setpoint stream; **add PN guidance mode**. |
| `yolo_py` | AAS | Camera → YOLO bounding boxes → bearing+class. |
| `radar_driver` | NEW | Radar detections → ROS2 (sim plugin or real radar). |
| `lidar_driver` | AAS/std | LiDAR point cloud. |
| `mics_fusion` | NEW | EKF/UKF over the three modalities → onboard `TargetEstimate`. |
| `mics_terminal` | NEW | Acquisition logic, terminal pursuit, capture predicate + trigger. |
| `mission` (EXT) | AAS | Hosts the MICS state machine; reads `/assignments`. |
| `state_sharing` | AAS | Publishes `/state_sharing_drone_N` (pose+status+assignment). |
| `safety` | NEW | Geofence, RTL, kill switch, capture interlocks. |

### 6.2 State machine (`mission` + `mics_terminal`)

```
IDLE
 └─(assignment received)─▶ ASSIGNED
       └─(takeoff/clearance)─▶ MIDCOURSE   (fly to cue region; track source = GS cue)
             └─(onboard track quality > Q for T_lock)─▶ ACQUIRING ─▶ handoff to own sensors
                   └─(stable onboard track)─▶ TERMINAL  (PN pursuit)
                         ├─(capture geometry & interlocks)─▶ CAPTURED ─▶ report, RTL ─▶ IDLE
                         └─(track lost>T_lost | miss | no confirm)─▶ FAILED ─▶ release, RTL ─▶ IDLE
```

Every transition broadcasts on `/state_sharing_drone_N` so the GS allocator stays in sync.

### 6.3 Fusion (`mics_fusion`) — design notes

- State: target position + velocity in the drone's local frame.
- Camera → bearing/elevation update; Radar → range + range-rate update; LiDAR → direct position update at close range.
- Use an EKF (or UKF if nonlinearity hurts) with per-sensor measurement models and validity gating.
- Publish `TargetEstimate{pose, twist, covariance, quality}`. **`quality`** drives the handoff and failure logic.

### 6.4 Terminal pursuit (`mics_terminal`) — kept at pursuit/capture level

- Guidance: **proportional navigation** generating velocity/attitude setpoints into `offboard_control`. Tune navigation constant in SITL.
- Capture predicate: range < `R_cap` AND closure within bounds AND aspect within cone AND safety interlocks OK.
- Trigger: emit `CaptureEvent`; payload actuation is abstracted behind a single interface (sim: log/scoring; HW: payload driver). **No kinetic logic.**

### 6.5 Safety (`safety`)
- Hard geofence (auto-RTL on breach), per-drone arm/disarm, global kill from GS, capture interlock (won't fire outside geometry or near a teammate), low-battery RTL. Always-on, highest priority.

---

## 7. Ground-station software stack

### 7.1 Nodes

| Node | Role |
|---|---|
| `mics_target_ingest` | Normalise internal/external sources → `TargetTrack`; host `cue_degrader` in internal mode. |
| `mics_track_manager` | Associate/age/fuse tracks; publish `/tracks` (≥5 Hz). |
| `mics_allocator` | Maintain roster from `/state_sharing_*`; compute assignments; publish `/assignments`; reassign on failure. |
| `mics_monitor` | Map UI, assignment overlay, operator controls, logging/replay. **Frontend = MICS-View** (CesiumJS viewer; see *MICS-View* PRD). |
| `viewer-gateway` | Bridges ROS2↔browser for MICS-View: per-frame snapshots, ENU→geodetic, log aggregation (`/rosout`), recording, scenario-run proxy. (Detailed in *MICS-View* PRD.) |
| `mics_sim_orchestrator` | **Sim-host node.** Commands the **always-on** simulation containers over ROS2 (`sim_control`: world reset, spawn/despawn, physics/RTF, sensor-mode) to run a scenario from the vetted catalog; serves `RunScenario` action + `SetSimSpeed` service; publishes `SimRunStatus`. Holds **no `docker.sock`**; sim-only; cannot command real hardware. |

### 7.2 Allocation algorithm

- Cost = estimated time-to-intercept (range / closing speed, adjusted for drone state/battery).
- Solve assignment (greedy nearest-available for v0; **Hungarian algorithm** for optimal one-to-one in v1).
- Constraints: one target per drone unless `redundancy>1`; respect `standby` role; honour deconfliction corridors.
- Reassignment trigger: a drone enters `FAILED`/`LOST`, or a new higher-priority target appears. Recompute over **available** drones only; must republish within ≤1 s (`FR-ALLOC-4`).

### 7.3 Operator UI (`mics_monitor`)
Live 2D/3D map: targets with uncertainty ellipses, interceptors coloured by state, assignment links, plus arm/disarm-all, abort/RTL-all, pause-allocation. Records rosbag/ulog for replay.

The operator UI is implemented as **MICS-View**, a browser-based CesiumJS 3D viewer specified in its own PRD (*MICS-View — 3D Situational-Awareness Viewer*). It additionally provides high-performance grids (interceptor/target/process-log), disk recording (incl. logs), scenario selection + run/stop, and simulation-speed control — all via the `viewer-gateway` and `mics_sim_orchestrator` above.

---

## 8. Interfaces & message schemas

Transport: **Zenoh** for inter-drone and drone↔GS (RF-friendly); ROS2 DDS within a vehicle.

### 8.1 `mics_msgs/TargetTrack`
```
std_msgs/Header header        # stamp, frame_id (global)
uint32   target_id
geometry_msgs/Point     position
geometry_msgs/Vector3   velocity        # may be zero/unset
float64[9] position_covariance
float64    class_confidence
uint8      source             # 0=internal_sim 1=external 2=onboard
float64    age                # seconds since last real update
```

### 8.2 `mics_msgs/Assignment`
```
std_msgs/Header header
uint32 drone_id
uint32 target_id              # 0 = unassigned/standby
uint8  role                   # 0=primary 1=standby
```

### 8.3 `mics_msgs/DroneStatus` (on `/state_sharing_drone_N`)
```
std_msgs/Header header
uint32 drone_id
geometry_msgs/Pose   pose
geometry_msgs/Twist  twist
uint8  state                  # IDLE..FAILED enum
uint32 current_target
float32 battery_pct
float32 track_quality         # 0..1, the onboard estimate quality
```

### 8.4 `mics_msgs/CaptureEvent`
```
std_msgs/Header header
uint32 drone_id
uint32 target_id
uint8  result                 # 0=attempt 1=success 2=miss
geometry_msgs/Point engagement_point
```

### 8.5 External ingress (UDP/JSON) — `target_source=external`
```json
{ "stamp": 1730000000.0, "target_id": 42,
  "position": {"lat": 0.0, "lon": 0.0, "alt": 0.0},
  "velocity": {"vn": 0.0, "ve": 0.0, "vd": 0.0},
  "pos_sigma_m": 12.0, "class_confidence": 0.7, "source": "ext_radar_1" }
```

### 8.6 Logging — standard `/rosout` (no custom message)
All MICS nodes (drones + GS) log via the ROS2 standard path. Log messages go to console, disk, and the `/rosout` topic using `rcl_interfaces/msg/Log` (fields: `stamp`, `level` [DEBUG=10/INFO=20/WARN=30/ERROR=40/FATAL=50], `name`, `msg`, `file`, `function`, `line`; the logger `name` identifies the originating node). The `viewer-gateway` aggregates `/rosout` for the process-log grid. For cross-machine aggregation over Zenoh, run a `zenoh-bridge-ros2dds` so every node's `/rosout` reaches the gateway. **Do not** introduce a custom log message; if structured run metadata is needed, publish an *additional* topic alongside `/rosout`.

### 8.7 Simulation control interfaces (drive the `mics_sim_orchestrator`)
```
# mics_msgs/action/RunScenario.action
# Goal
string  scenario_id          # from the vetted catalog (never a free-form path/command)
string  overrides_yaml       # optional schema-validated overlay (e.g. laptop profile)
bool    record               # auto-start a recording session for this run
float64 requested_rtf        # initial real-time factor; 0 = as fast as possible (SITL only)
---
# Result
bool    success
string  message
string  run_id
string  recording_session_id
float64 duration_s
---
# Feedback
uint8   state                # see SimRunStatus constants
string  detail
float64 elapsed_s
uint8   drones_up
uint8   targets_up
float64 actual_rtf
```
```
# mics_msgs/msg/SimRunStatus.msg   (published on /sim_run_status)
uint8 IDLE=0
uint8 LAUNCHING=1
uint8 RUNNING=2
uint8 STOPPING=3
uint8 STOPPED=4
uint8 ERROR=5
std_msgs/Header header
uint8   state
string  scenario_id
string  run_id
float64 elapsed_s
uint8   drones_up
uint8   targets_up
string  recording_session_id
float64 requested_rtf
float64 actual_rtf            # achieved; may be < requested when compute-bound
bool    rtf_controllable      # false on HITL/real hardware (locked at 1.0)
string  message
```
```
# mics_msgs/msg/ScenarioInfo.msg   (catalog descriptor)
string scenario_id
string name
string description
string target_source          # internal | external
uint8  defender_count
uint8  attacker_count
string sensor_mode            # ideal | stub | full
string path                   # server-side; not exposed raw to clients
```
```
# mics_msgs/srv/SetSimSpeed.srv   (runtime RTF change; auth-gated; SITL only)
float64 requested_rtf         # 0 = as fast as possible
---
bool    success
float64 applied_rtf
string  message
```
**Security:** with the **always-on** model the orchestrator commands the running sim over ROS2 and holds no Docker-daemon access (no `docker.sock`). `RunScenario`/`SetSimSpeed` are auth-gated, accept only catalog `scenario_id`s + schema-validated overrides (no shell strings or paths), and are audit-logged. They control **simulation only** and cannot arm or fly real aircraft. Deployment/container-networking detail is in *MICS-View* PRD §6.8.

---

## 9. Test & validation plan

### 9.1 Levels
1. **Unit** — fusion math, allocation, state-machine transitions.
2. **Pure SITL, ideal sensors** — validate coordination (allocation, handoff, reassignment, deconfliction) with perfect detections. *Build this first.*
3. **SITL, realistic sensors** — add noise/dropout/latency + cue degradation.
4. **Jetson-in-the-loop** — confirm perception+fusion run in real time on Orin.
5. **Constrained live flight** — authorised range only, single interceptor vs slow cooperative "target", capture payload last.

### 9.2 Scenarios (run in faster-than-real-time, many seeds)
- 1 defender vs 1 non-evasive target (happy path).
- 4 defenders vs 1 evasive target → **forced miss** → reassignment to next-best.
- 4 defenders vs 3 simultaneous targets (allocation stress).
- Degraded camera (glare/dark) → radar-led fusion holds track.
- Comms dropout on one drone → graceful degradation + deconfliction.

### 9.3 Metrics
- Capture success rate; mean time-to-intercept; reassignment latency (target ≤1 s); handoff success rate; min inter-drone separation (must stay > safety bound); false-fire rate (must be 0 in sim interlock tests); real-time factor sustained on Jetson.

---

## 10. Milestones (suggested phasing)

| Phase | Deliverable | Exit criteria |
|---|---|---|
| **P0** | Dev env + baseline AAS SITL running | Multi-vehicle Gazebo up, faster-than-real-time confirmed |
| **P1** | Coordination core (ideal sensors) | Allocation + handoff + reassignment + deconfliction pass in SITL |
| **P2** | Internal attacker sim + cue degradation | Evasive target + degraded cues; reassignment under forced misses |
| **P3** | Sensor models + fusion | Camera+radar+LiDAR EKF; graceful degradation demonstrated |
| **P4** | Terminal pursuit + capture (sim) | PN pursuit + capture predicate + interlocks; 0 false-fire |
| **P5** | External-source interface | Third-party/sim feed drives system unchanged |
| **P6** | Jetson-in-the-loop | Real-time perception+fusion on Orin |
| **P7** | Gym env / policy learning (optional) | RL allocation or pursuit policy beats hand-tuned baseline |
| **P8** | Constrained live flight (authorised) | Single interceptor capture on range |

---

## 11. Risks & open questions

- **Radar sim fidelity** — no radar plugin ships with AAS; the custom model's realism gates fusion credibility. *Open: build vs adapt an existing RF sim.*
- **Sim-to-real gap in fusion/guidance** — mitigated by AAS multi-target compilation (same code sim+HW) and Jetson HITL.
- **Comms under RF stress** — Zenoh config + deconfliction must tolerate dropouts; needs explicit comms-in-the-loop testing.
- **Allocation thrash** — rapid reassignment oscillation under noisy cues; add hysteresis/commit-time.
- **Regulatory** — live C-UAV testing approvals; confirm range and authorisations before any P8 work.
- **Open:** target prioritisation policy when targets outnumber interceptors? Capture-confirm sensing method? Standby-drone commit threshold?

---

## Appendix A — Scenario YAML (illustrative)

```yaml
target_source: internal          # internal | external
sensor_mode: full                # ideal | stub | full   (see §4.5)
defenders:
  count: 4
  autopilot: px4                  # px4 | ardupilot
  airframe: holybro_x500v2
attackers:
  - id: 1
    profile: evasive              # waypoint | ingress | evasive | replay
    speed_mps: 15
    r_evade_m: 80
cue_degradation:
  pos_sigma_m: 15
  dropout_pct: 20
  latency_ms: 300
allocation:
  algorithm: hungarian            # greedy | hungarian
  redundancy: 1
  reassign_max_latency_s: 1.0
terminal:
  guidance: proportional_navigation
  nav_constant: 3.0
  r_capture_m: 3.0
safety:
  geofence_radius_m: 500
  capture_min_teammate_sep_m: 10
```

### Appendix A.1 — CPU-only laptop preset

Override block for a no-GPU laptop (overlay onto the scenario above). Headless, reduced swarm, no rendering/inference.

```yaml
sensor_mode: stub                # ideal for P1–P2, stub for P3–P4 fusion tests
render: headless                 # no Gazebo GUI client
defenders:
  count: 2                       # 2–4 max on CPU
attackers:
  - id: 1
    profile: ingress             # start simple; switch to evasive once stable
    speed_mps: 10
sensors:
  camera: { enabled: false }     # skip rendering on CPU
  lidar:  { enabled: true, point_decimation: 4 }   # thin the cloud
  radar:  { enabled: true }      # cheap analytic model, fine on CPU
perception:
  detector: stub                 # stub | yolov8n | yolov8s  (stub = no inference)
sim:
  realtime_factor: auto          # don't force 10x; let it run as fast as CPU allows
```

To run a quick correctness check of real perception on CPU (slow, few drones):
```yaml
sensor_mode: full
defenders: { count: 1 }
sensors: { camera: { enabled: true, width: 320, height: 240, rate_hz: 5 } }
perception: { detector: yolov8n }
```


## Appendix B — Reuse map (what to build vs reuse)

- **Reuse from AAS:** Docker/tmux harness, faster-than-real-time multi-vehicle Gazebo SITL, PX4/ArduPilot SITL, `autopilot_interface`, `state_sharing` (Zenoh), `yolo_py` + camera & LiDAR sim models, Gymnasium wrapper, Jetson deployment path. Standard `/rosout` logging.
- **Extend:** `mission` (state machine), `offboard_control` (PN guidance), `ground_system` (`/tracks`).
- **Build new:** radar sim + driver, `mics_fusion`, `mics_terminal` (+capture interface), `mics_target_ingest` (internal+external+degrader), `mics_track_manager`, `mics_allocator`, `mics_monitor`, `mics_sim_orchestrator`, `viewer-gateway`, `mics_msgs` (incl. `RunScenario.action`, `SimRunStatus.msg`, `ScenarioInfo.msg`, `SetSimSpeed.srv`), `safety`.
- **Companion deliverable:** *MICS-View* — the browser-based 3D viewer that implements `mics_monitor`'s UI (grids, recording, scenario run + sim-speed control). Specified in its own PRD; shares `mics_msgs`, topic names, the scenario catalog (Appendix A), the datum, and the non-kinetic framing.
