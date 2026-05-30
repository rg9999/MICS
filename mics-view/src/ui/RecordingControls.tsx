import { useEffect } from "react";
import { useStore } from "../app/store";
import { useGateway } from "../app/ConnectionProvider";
import type { RecordingStatus } from "../data/types";

function fmtBytes(n: number): string {
  if (n < 1024) return `${n} B`;
  if (n < 1024 * 1024) return `${(n / 1024).toFixed(0)} KB`;
  return `${(n / 1024 / 1024).toFixed(1)} MB`;
}

// Start/stop session recording with a live status indicator. Only meaningful in
// live mode (the gateway rejects recording while replaying).
export function RecordingControls() {
  const gw = useGateway();
  const controlsEnabled = useStore((s) => s.controlsEnabled);
  const mode = useStore((s) => s.mode);
  const recording = useStore((s) => s.recording);
  const connected = useStore((s) => s.connected);

  // poll status while a recording is active so elapsed/size update
  useEffect(() => {
    if (!recording?.recording) return;
    const id = setInterval(async () => {
      try {
        useStore.getState().setRecording(await gw.request<RecordingStatus>("recording.status"));
      } catch { /* ignore */ }
    }, 1000);
    return () => clearInterval(id);
  }, [gw, recording?.recording]);

  const start = async () => {
    try {
      await gw.request("recording.start");
      useStore.getState().setRecording(await gw.request<RecordingStatus>("recording.status"));
    } catch (e) { console.warn(e); }
  };

  const stop = async () => {
    try {
      await gw.request("recording.stop");
      useStore.getState().setRecording(await gw.request<RecordingStatus>("recording.status"));
      useStore.getState().setRecordings(await gw.request("recordings.list"));
    } catch (e) { console.warn(e); }
  };

  const active = recording?.recording === true;
  const disabled = !connected || !controlsEnabled || mode === "replay";

  return (
    <div className="card">
      <h3>Recording</h3>
      {!controlsEnabled && <div className="muted">controls disabled by gateway</div>}
      <div className="row">
        <button className="primary" onClick={start} disabled={disabled || active}>
          ● Record
        </button>
        <button className="danger" onClick={stop} disabled={disabled || !active}>
          ■ Stop
        </button>
      </div>
      {active && (
        <div className="kv" style={{ marginTop: 6 }}>
          <span>{recording.sessionId}</span>
          <span>{recording.elapsedS.toFixed(0)} s · {fmtBytes(recording.sizeBytes)}</span>
        </div>
      )}
    </div>
  );
}
