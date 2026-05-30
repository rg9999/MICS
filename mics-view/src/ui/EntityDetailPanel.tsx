import { useStore } from "../app/store";
import { STATE_COLORS } from "../app/config";
import type { AllocationStatus } from "../data/types";

function fmt(n: number | null | undefined, unit = "", digits = 1): string {
  if (n === null || n === undefined || !Number.isFinite(n)) return "—";
  return `${n.toFixed(digits)}${unit}`;
}

function allocLabel(a: AllocationStatus): string {
  switch (a.kind) {
    case "ENGAGED": return `engaged by D${a.byDrone}`;
    case "CAPTURED": return "captured";
    default: return "unengaged";
  }
}

// Details for the currently selected entity (drone or target), driven by the
// shared selection. Empty when nothing is selected.
export function EntityDetailPanel() {
  const selection = useStore((s) => s.selection);
  const drones = useStore((s) => s.drones);
  const tracks = useStore((s) => s.tracks);
  const dronesDerived = useStore((s) => s.dronesDerived);
  const targetsDerived = useStore((s) => s.targetsDerived);
  const setSelection = useStore((s) => s.setSelection);

  if (!selection) {
    return (
      <div className="card">
        <h3>Detail</h3>
        <div className="muted">select a defender or target</div>
      </div>
    );
  }

  if (selection.kind === "drone") {
    const d = drones.find((x) => x.droneId === selection.id);
    const der = dronesDerived[selection.id];
    if (!d) return <Missing label={`D${selection.id}`} onClear={() => setSelection(null)} />;
    return (
      <div className="card">
        <h3>Defender D{d.droneId}</h3>
        <KV k="State">
          <span className="dot" style={{ background: STATE_COLORS[d.state] ?? "#ccc" }} />
          {d.state}
        </KV>
        <KV k="Target">{d.currentTarget ? `T${d.currentTarget}` : "—"}</KV>
        <KV k="Battery">{fmt(d.batteryPct, "%", 0)}</KV>
        <KV k="Track quality">{fmt(d.trackQuality, "", 2)}</KV>
        <KV k="Speed">{fmt(der?.speed, " m/s")}</KV>
        <KV k="Altitude">{fmt(der?.altitude, " m")}</KV>
        <KV k="Range to tgt">{fmt(der?.rangeToTarget, " m")}</KV>
        <KV k="ETA intercept">{fmt(der?.etaToInterceptS, " s")}</KV>
        <KV k="ENU">{d.enu.map((v) => v.toFixed(0)).join(", ")}</KV>
      </div>
    );
  }

  const t = tracks.find((x) => x.targetId === selection.id);
  const der = targetsDerived[selection.id];
  if (!t) return <Missing label={`T${selection.id}`} onClear={() => setSelection(null)} />;
  return (
    <div className="card">
      <h3>Target T{t.targetId}</h3>
      <KV k="Source">{t.source}</KV>
      <KV k="Confidence">{fmt(t.classConfidence * 100, "%", 0)}</KV>
      <KV k="Track age">{fmt(t.age, " s")}</KV>
      <KV k="Pos σ">{fmt(t.posSigmaM, " m")}</KV>
      <KV k="Speed">{fmt(der?.speed ?? null, " m/s")}</KV>
      <KV k="Altitude">{fmt(der?.altitude, " m")}</KV>
      <KV k="Allocation">{der ? allocLabel(der.allocation) : "—"}</KV>
      <KV k="ENU">{t.enu.map((v) => v.toFixed(0)).join(", ")}</KV>
    </div>
  );
}

function KV({ k, children }: { k: string; children: React.ReactNode }) {
  return (
    <div className="kv">
      <span>{k}</span>
      <span style={{ display: "flex", alignItems: "center", gap: 6 }}>{children}</span>
    </div>
  );
}

function Missing({ label, onClear }: { label: string; onClear: () => void }) {
  return (
    <div className="card">
      <h3>Detail</h3>
      <div className="muted">{label} is no longer present.</div>
      <div className="row" style={{ marginTop: 6 }}>
        <button onClick={onClear}>Clear selection</button>
      </div>
    </div>
  );
}
