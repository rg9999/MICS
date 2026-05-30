import { useStore } from "../app/store";
import type { CaptureResultName } from "../data/types";

const RESULT_CSS: Record<CaptureResultName, string> = {
  success: "#46d17a",
  miss: "#ff5e5e",
  attempt: "#ffd24d",
  unknown: "#8a8f98",
};

function clock(stamp: number): string {
  const d = new Date(stamp * 1000);
  return d.toLocaleTimeString([], { hour12: false });
}

// Reverse-chronological list of capture events (attempt/success/miss); clicking
// a row selects the involved target.
export function EventLog() {
  const events = useStore((s) => s.events);
  const setSelection = useStore((s) => s.setSelection);
  const recent = events.slice().reverse();

  return (
    <div className="card" style={{ flex: 1, minHeight: 0, display: "flex", flexDirection: "column" }}>
      <h3>Events ({events.length})</h3>
      <div style={{ overflowY: "auto", flex: 1 }}>
        {recent.length === 0 && <div className="muted">no capture events</div>}
        {recent.map((e, i) => (
          <div
            key={`${e.stamp}-${e.droneId}-${e.targetId}-${i}`}
            className="roster-row"
            onClick={() => setSelection({ kind: "target", id: e.targetId })}
          >
            <span className="dot" style={{ background: RESULT_CSS[e.result] }} />
            <span className="muted" style={{ width: 64 }}>{clock(e.stamp)}</span>
            <span style={{ flex: 1 }}>{e.result.toUpperCase()}</span>
            <span className="muted">D{e.droneId}→T{e.targetId}</span>
          </div>
        ))}
      </div>
    </div>
  );
}
