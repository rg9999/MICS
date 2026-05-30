"""mics_sim_orchestrator — scenario run lifecycle (PRD viewer §6.7).

Commands the (software) simulation from a *vetted catalog* and reports status,
without any Docker-daemon access. In this single-container, pure-Python sim
profile a "run" is the `mics.launch.py` topology for one scenario; the
orchestrator spawns it as a child process group, watches it, and tears it down
on cancel/stop. The browser only ever passes a catalog `scenario_id` plus an
optional schema-validated override overlay — never a path or shell string.

Interfaces:
  - action  run_scenario     (mics_msgs/RunScenario)
  - service  set_sim_speed    (mics_msgs/SetSimSpeed)   gated; locked in this profile
  - topic    /sim_run_status  (mics_msgs/SimRunStatus)  continuous, for any observer
"""

from __future__ import annotations

import os
import signal
import subprocess
import tempfile
import time
import uuid
from pathlib import Path

import rclpy
import yaml
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64, Int32

from mics_msgs.action import RunScenario
from mics_msgs.msg import DroneStatus as DroneStatusMsg
from mics_msgs.msg import SimRunStatus
from mics_msgs.srv import SetSimSpeed

# SimRunStatus.state constants (mirrored so we don't depend on import order)
IDLE = SimRunStatus.IDLE
LAUNCHING = SimRunStatus.LAUNCHING
RUNNING = SimRunStatus.RUNNING
STOPPING = SimRunStatus.STOPPING
STOPPED = SimRunStatus.STOPPED
ERROR = SimRunStatus.ERROR


def _deep_merge(base: dict, overlay: dict) -> dict:
    """Recursively merge ``overlay`` onto a copy of ``base`` (overlay wins)."""
    out = dict(base)
    for k, v in (overlay or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


class SimOrchestratorNode(Node):
    def __init__(self):
        super().__init__("mics_sim_orchestrator")
        self.declare_parameter("catalog_dir", "/scenarios")
        self.declare_parameter("default_rtf", 2.0)
        # Software-sim sets its tick rate once at launch, so runtime RTF change
        # is not possible without relaunch — report it as locked (FR-V-54).
        self.declare_parameter("rtf_controllable", False)

        self.catalog_dir = Path(self.get_parameter("catalog_dir").value)
        self.default_rtf = float(self.get_parameter("default_rtf").value)
        self.rtf_controllable = bool(self.get_parameter("rtf_controllable").value)

        self._cb = ReentrantCallbackGroup()

        # --- live run state (shared across threads) ---
        self._proc: subprocess.Popen | None = None
        self._state = IDLE
        self._run_id = ""
        self._scenario_id = ""
        self._requested_rtf = 0.0
        self._recording_session_id = ""
        self._message = ""
        # honest achieved-RTF tracking: d(sim_clock)/d(wall_clock)
        self._sim_t = 0.0
        self._last_sim_t = 0.0
        self._last_wall = 0.0
        self._actual_rtf = 0.0
        self._targets_up = 0
        self._drone_ids: set[int] = set()
        self._run_start_wall = 0.0
        self._elapsed_s = 0.0

        self.status_pub = self.create_publisher(SimRunStatus, "/sim_run_status", 10)
        self.create_subscription(Float64, "/sim/clock", self._on_clock, 50,
                                 callback_group=self._cb)
        self.create_subscription(Int32, "/sim/alive_count", self._on_alive, 10,
                                 callback_group=self._cb)
        self.create_subscription(DroneStatusMsg, "/state_sharing", self._on_status, 50,
                                 callback_group=self._cb)
        self.create_timer(0.5, self._publish_status, callback_group=self._cb)

        self._action = ActionServer(
            self, RunScenario, "run_scenario",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._cb,
        )
        self.create_service(SetSimSpeed, "set_sim_speed", self._set_speed,
                            callback_group=self._cb)

        self.get_logger().info(
            f"mics_sim_orchestrator up: catalog={self.catalog_dir} "
            f"default_rtf={self.default_rtf} rtf_controllable={self.rtf_controllable}")

    # --- catalog -----------------------------------------------------------

    def _resolve_scenario(self, scenario_id: str) -> Path | None:
        """Map a catalog id to a vetted on-disk path, or None if not in catalog."""
        if not scenario_id or "/" in scenario_id or "\\" in scenario_id or ".." in scenario_id:
            return None
        for ext in (".yaml", ".yml"):
            p = (self.catalog_dir / f"{scenario_id}{ext}").resolve()
            # confine strictly to the catalog dir
            if p.is_file() and self.catalog_dir.resolve() in p.parents:
                return p
        return None

    def _materialize(self, base_path: Path, overrides_yaml: str) -> Path:
        """Apply a schema-validated override overlay and write a temp scenario file."""
        if not overrides_yaml.strip():
            return base_path
        base = yaml.safe_load(base_path.read_text()) or {}
        overlay = yaml.safe_load(overrides_yaml) or {}
        if not isinstance(overlay, dict):
            raise ValueError("overrides_yaml must be a YAML mapping")
        merged = _deep_merge(base, overlay)
        runs = Path(tempfile.gettempdir()) / "mics_runs"
        runs.mkdir(parents=True, exist_ok=True)
        out = runs / f"{base_path.stem}_{uuid.uuid4().hex[:8]}.yaml"
        out.write_text(yaml.safe_dump(merged))
        return out

    # --- status sources ----------------------------------------------------

    def _on_clock(self, m: Float64):
        self._sim_t = float(m.data)
        wall = time.monotonic()
        if self._last_wall > 0.0:
            dw = wall - self._last_wall
            if dw > 1e-3:
                inst = (self._sim_t - self._last_sim_t) / dw
                # light EMA so the reported RTF is stable, not jittery
                self._actual_rtf = 0.8 * self._actual_rtf + 0.2 * inst
        self._last_wall = wall
        self._last_sim_t = self._sim_t

    def _on_alive(self, m: Int32):
        self._targets_up = int(m.data)

    def _on_status(self, m: DroneStatusMsg):
        self._drone_ids.add(int(m.drone_id))

    def _publish_status(self):
        s = SimRunStatus()
        s.header.stamp = self.get_clock().now().to_msg()
        s.state = self._state
        s.scenario_id = self._scenario_id
        s.run_id = self._run_id
        if self._run_start_wall > 0.0 and self._state in (LAUNCHING, RUNNING, STOPPING):
            self._elapsed_s = time.monotonic() - self._run_start_wall
        s.elapsed_s = float(self._elapsed_s)
        s.drones_up = min(len(self._drone_ids), 255)
        s.targets_up = min(self._targets_up, 255)
        s.recording_session_id = self._recording_session_id
        s.requested_rtf = float(self._requested_rtf)
        s.actual_rtf = float(max(0.0, self._actual_rtf))
        s.rtf_controllable = self.rtf_controllable
        s.message = self._message
        self.status_pub.publish(s)

    def _reset_run_state(self):
        self._proc = None
        self._run_id = ""
        self._scenario_id = ""
        self._requested_rtf = 0.0
        self._recording_session_id = ""
        self._actual_rtf = 0.0
        self._targets_up = 0
        self._drone_ids.clear()
        self._run_start_wall = 0.0
        self._last_wall = 0.0
        self._last_sim_t = 0.0

    # --- action: run_scenario ----------------------------------------------

    def _on_goal(self, goal_request) -> GoalResponse:
        if self._state in (LAUNCHING, RUNNING, STOPPING):
            self.get_logger().warn("run rejected: a scenario is already active")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = RunScenario.Result()
        path = self._resolve_scenario(goal.scenario_id)
        if path is None:
            self._state = ERROR
            self._message = f"unknown scenario_id '{goal.scenario_id}'"
            self.get_logger().error(self._message)
            goal_handle.abort()
            result.success = False
            result.message = self._message
            return result

        try:
            scenario_path = self._materialize(path, goal.overrides_yaml)
        except Exception as exc:  # noqa: BLE001 — surface any overlay error to the client
            self._state = ERROR
            self._message = f"invalid overrides_yaml: {exc}"
            self.get_logger().error(self._message)
            goal_handle.abort()
            result.success = False
            result.message = self._message
            return result

        rtf = goal.requested_rtf if goal.requested_rtf > 0.0 else self.default_rtf
        self._reset_run_state()
        self._run_id = uuid.uuid4().hex[:12]
        self._scenario_id = goal.scenario_id
        self._requested_rtf = goal.requested_rtf
        # Recording is owned by the viewer-gateway (it sees the derived stream);
        # the orchestrator only runs the sim. The gateway binds record<->session.
        self._recording_session_id = ""
        self._state = LAUNCHING
        self._message = "launching"
        self._run_start_wall = time.monotonic()
        start_wall = self._run_start_wall

        cmd = ["ros2", "launch", "mics_nodes", "mics.launch.py",
               f"scenario:={scenario_path}", f"rate_factor:={rtf}"]
        self.get_logger().info(f"run {self._run_id}: {' '.join(cmd)}")
        # New session/process group so we can tear down the whole launch tree.
        popen_kwargs = {}
        if hasattr(os, "setsid"):
            popen_kwargs["start_new_session"] = True
        self._proc = subprocess.Popen(cmd, **popen_kwargs)
        self._state = RUNNING
        self._message = "running"

        # Watch the child: finish when it exits or the goal is cancelled.
        try:
            while True:
                if goal_handle.is_cancel_requested:
                    self._state = STOPPING
                    self._message = "stopping (cancel requested)"
                    self._terminate_proc()
                    goal_handle.canceled()
                    result.success = False
                    result.message = "cancelled"
                    result.run_id = self._run_id
                    result.duration_s = float(time.monotonic() - start_wall)
                    self._state = STOPPED
                    self._reset_run_state()
                    return result

                ret = self._proc.poll()
                if ret is not None:
                    break

                fb = RunScenario.Feedback()
                fb.state = self._state
                fb.detail = self._message
                fb.elapsed_s = float(time.monotonic() - start_wall)
                fb.drones_up = min(len(self._drone_ids), 255)
                fb.targets_up = min(self._targets_up, 255)
                fb.actual_rtf = float(max(0.0, self._actual_rtf))
                goal_handle.publish_feedback(fb)
                time.sleep(0.5)
        finally:
            pass

        ret = self._proc.returncode if self._proc else -1
        duration = float(time.monotonic() - start_wall)
        run_id = self._run_id
        self._state = STOPPED
        self._message = f"run complete (exit {ret})"
        self.get_logger().info(f"run {run_id} {self._message} in {duration:.1f}s")
        result.success = (ret == 0)
        result.message = self._message
        result.run_id = run_id
        result.duration_s = duration
        if not result.success:
            goal_handle.abort()
        else:
            goal_handle.succeed()
        self._reset_run_state()
        return result

    def _terminate_proc(self):
        if self._proc is None or self._proc.poll() is not None:
            return
        try:
            if hasattr(os, "killpg"):
                pgid = os.getpgid(self._proc.pid)
                os.killpg(pgid, signal.SIGINT)
            else:
                self._proc.send_signal(signal.SIGINT)
            try:
                self._proc.wait(timeout=10)
                return
            except subprocess.TimeoutExpired:
                pass
            if hasattr(os, "killpg"):
                os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
            else:
                self._proc.terminate()
            self._proc.wait(timeout=5)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                self._proc.kill()
            except ProcessLookupError:
                pass

    # --- service: set_sim_speed --------------------------------------------

    def _set_speed(self, request, response):
        if not self.rtf_controllable:
            response.success = False
            response.applied_rtf = float(self._requested_rtf)
            response.message = ("runtime RTF change is locked in the software-sim "
                                "profile; relaunch with requested_rtf instead")
            return response
        # (Reserved for the SITL/lockstep profile, where Gazebo physics RTF can
        # change at runtime.) Not reachable while rtf_controllable is False.
        response.success = False
        response.applied_rtf = float(self._requested_rtf)
        response.message = "no active controllable run"
        return response


def main(argv=None):
    rclpy.init(args=argv)
    node = SimOrchestratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node._terminate_proc()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
