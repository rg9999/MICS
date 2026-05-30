import { useStore } from "../app/store";
import type { ReplayState } from "../app/store";
import { useGateway } from "../app/ConnectionProvider";
import type { RecordingManifest } from "../data/types";

function fmtDur(s: number | null): string {
  if (s === null || !Number.isFinite(s)) return "—";
  const m = Math.floor(s / 60);
  return m > 0 ? `${m}m ${Math.round(s % 60)}s` : `${s.toFixed(0)}s`;
}

function fmtTime(stamp: number): string {
  return new Date(stamp * 1000).toLocaleString([], { hour12: false });
}

// Live mode: lists past recordings (replay is a gateway launch-time source, so
// this is informational). Replay mode: shows playback transport bound to the
// gateway clock.
export function RecordingBrowser() {
  const mode = useStore((s) => s.mode);
  const recordings = useStore((s) => s.recordings);

  if (mode === "replay") return <ReplayTransport />;

  return (
    <div className="card">
      <h3>Recordings ({recordings.length})</h3>
      {recordings.length === 0 && <div className="muted">no recordings on disk</div>}
      <div style={{ maxHeight: 180, overflowY: "auto" }}>
        {recordings.map((r) => (
          <RecordingRow key={r.sessionId} r={r} />
        ))}
      </div>
      {recordings.length > 0 && (
        <div className="muted" style={{ marginTop: 6, fontSize: 11 }}>
          To replay: launch the gateway with{" "}
          <code>--source replay --session &lt;id&gt;</code>.
        </div>
      )}
    </div>
  );
}

function RecordingRow({ r }: { r: RecordingManifest }) {
  return (
    <div className="roster-row" style={{ flexDirection: "column", alignItems: "stretch", gap: 2 }}>
      <div className="row">
        <span style={{ flex: 1 }}>{r.sessionId}</span>
        <span className="muted">{fmtDur(r.durationS)}</span>
      </div>
      <div className="row">
        <span className="muted" style={{ flex: 1 }}>{fmtTime(r.startedAt)}</span>
        <span className="muted">{r.droneCount}v{r.targetCount}{r.scenario ? ` · ${r.scenario}` : ""}</span>
      </div>
    </div>
  );
}

function ReplayTransport() {
  const gw = useGateway();
  const replay = useStore((s) => s.replay);
  const stamp = useStore((s) => s.stamp);

  const set = (r: ReplayState) => useStore.getState().setReplay(r);
  const span = replay ? replay.t1 - replay.t0 : 0;
  const playhead = replay ? Math.max(replay.t0, Math.min(replay.t1, stamp)) : 0;
  const frac = span > 0 ? (playhead - replay!.t0) / span : 0;

  const send = async (action: string, extra: Record<string, unknown> = {}) => {
    try { set(await gw.request<ReplayState>(action, extra)); }
    catch (e) { console.warn(e); }
  };

  return (
    <div className="card">
      <h3>Replay</h3>
      <div className="row">
        <button className="primary" onClick={() => send("replay.play")} disabled={replay ? !replay.paused : false}>▶</button>
        <button onClick={() => send("replay.pause")} disabled={replay ? replay.paused : false}>⏸</button>
        <span className="muted" style={{ flex: 1, textAlign: "right" }}>
          {(playhead - (replay?.t0 ?? 0)).toFixed(1)} / {span.toFixed(1)} s
        </span>
      </div>
      <input
        type="range" min={0} max={1} step={0.001} value={frac}
        style={{ marginTop: 8 }}
        onChange={(e) => {
          if (!replay) return;
          const stampTo = replay.t0 + Number(e.target.value) * span;
          void send("replay.seek", { stamp: stampTo });
        }}
      />
      <div className="row" style={{ marginTop: 6 }}>
        <span className="muted">Speed</span>
        {[0.25, 0.5, 1, 2, 4, 8].map((sp) => (
          <button
            key={sp}
            className={replay?.speed === sp ? "primary" : ""}
            onClick={() => send("replay.setSpeed", { speed: sp })}
          >
            {sp}×
          </button>
        ))}
      </div>
    </div>
  );
}
