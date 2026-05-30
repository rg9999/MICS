import { useState } from "react";
import { useStore } from "../app/store";
import { useGateway } from "../app/ConnectionProvider";
import { SIM_RUN_STATES } from "../data/types";

function stateName(i: number): string {
  return SIM_RUN_STATES[i] ?? "unknown";
}

// Browse the scenario catalog, launch/stop a run, and watch live run status
// (state, RTF, entity counts) relayed from the orchestrator.
export function ScenarioPanel() {
  const gw = useGateway();
  const scenarios = useStore((s) => s.scenarios);
  const sim = useStore((s) => s.sim);
  const controlsEnabled = useStore((s) => s.controlsEnabled);
  const connected = useStore((s) => s.connected);
  const [sel, setSel] = useState("");
  const [busy, setBusy] = useState(false);

  const running = sim ? ["LAUNCHING", "RUNNING", "STOPPING"].includes(stateName(sim.state)) : false;
  const disabled = !connected || !controlsEnabled || scenarios.length === 0;

  const run = async () => {
    if (!sel) return;
    setBusy(true);
    try { await gw.request("scenarios.run", { scenarioId: sel }); }
    catch (e) { console.warn(e); }
    finally { setBusy(false); }
  };

  const stop = async () => {
    setBusy(true);
    try { await gw.request("scenarios.stop"); }
    catch (e) { console.warn(e); }
    finally { setBusy(false); }
  };

  const setSpeed = async (rtf: number) => {
    try { await gw.request("sim.setSpeed", { requestedRtf: rtf }); }
    catch (e) { console.warn(e); }
  };

  return (
    <div className="card">
      <h3>Scenarios</h3>
      {!controlsEnabled && <div className="muted">controls disabled by gateway</div>}
      <select
        value={sel}
        onChange={(e) => setSel(e.target.value)}
        disabled={disabled}
      >
        <option value="">— select scenario —</option>
        {scenarios.map((s) => (
          <option key={s.scenarioId} value={s.scenarioId}>
            {s.name} ({s.defenderCount}v{s.attackerCount})
          </option>
        ))}
      </select>
      <div className="row" style={{ marginTop: 6 }}>
        <button className="primary" onClick={run} disabled={disabled || busy || !sel || running}>
          Run
        </button>
        <button className="danger" onClick={stop} disabled={!connected || busy || !running}>
          Stop
        </button>
      </div>

      {sim && (
        <div style={{ marginTop: 8 }}>
          <div className="kv"><span>State</span><span>{stateName(sim.state)}</span></div>
          <div className="kv"><span>Scenario</span><span>{sim.scenarioId || "—"}</span></div>
          <div className="kv"><span>Elapsed</span><span>{sim.elapsedS.toFixed(0)} s</span></div>
          <div className="kv"><span>Drones / Targets</span><span>{sim.dronesUp} / {sim.targetsUp}</span></div>
          <div className="kv"><span>RTF (act / req)</span><span>{sim.actualRtf.toFixed(2)} / {sim.requestedRtf.toFixed(2)}</span></div>
          {sim.message && <div className="muted" style={{ marginTop: 4 }}>{sim.message}</div>}
          {sim.rtfControllable && (
            <div className="row" style={{ marginTop: 6 }}>
              {[0.5, 1, 2, 4].map((r) => (
                <button key={r} onClick={() => setSpeed(r)}>{r}×</button>
              ))}
            </div>
          )}
        </div>
      )}
    </div>
  );
}
