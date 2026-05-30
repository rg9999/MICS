import { useStore } from "../app/store";
import { STATE_COLORS } from "../app/config";

// All defenders with state colour + assigned target; selecting a row drives the
// shared selection (scene highlights + flies-to via CameraControls).
export function RosterPanel() {
  const drones = useStore((s) => s.drones);
  const derived = useStore((s) => s.dronesDerived);
  const selection = useStore((s) => s.selection);
  const setSelection = useStore((s) => s.setSelection);

  return (
    <div className="card">
      <h3>Roster ({drones.length})</h3>
      {drones.length === 0 && <div className="muted">no defenders</div>}
      {drones.map((d) => {
        const sel = selection?.kind === "drone" && selection.id === d.droneId;
        const alloc = derived[d.droneId]?.allocatedTarget;
        return (
          <div
            key={d.droneId}
            className={`roster-row${sel ? " sel" : ""}`}
            onClick={() => setSelection({ kind: "drone", id: d.droneId })}
          >
            <span className="dot" style={{ background: STATE_COLORS[d.state] ?? "#ccc" }} />
            <span style={{ width: 34 }}>D{d.droneId}</span>
            <span style={{ flex: 1 }}>{d.state}</span>
            <span className="muted">{alloc ? `→T${alloc}` : "—"}</span>
            <span className="muted">{d.batteryPct.toFixed(0)}%</span>
          </div>
        );
      })}
    </div>
  );
}
