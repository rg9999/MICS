import { useStore } from "./app/store";
import { CesiumViewer } from "./scene/CesiumViewer";
import { LayerToggles } from "./ui/LayerToggles";
import { RosterPanel } from "./ui/RosterPanel";
import { CameraControls } from "./ui/CameraControls";
import { RecordingControls } from "./ui/RecordingControls";
import { ScenarioPanel } from "./ui/ScenarioPanel";
import { RecordingBrowser } from "./ui/RecordingBrowser";
import { EntityDetailPanel } from "./ui/EntityDetailPanel";
import { EventLog } from "./ui/EventLog";
import { Dock } from "./ui/Dock";

export function App() {
  const connected = useStore((s) => s.connected);
  const mode = useStore((s) => s.mode);
  const sourceName = useStore((s) => s.sourceName);
  const stamp = useStore((s) => s.stamp);

  return (
    <div className="app">
      <div className="scene">
        <CesiumViewer />
      </div>

      <div className="topbar">
        <span className="title">MICS-View</span>
        <span className={`badge ${connected ? "ok" : "bad"}`}>
          {connected ? "connected" : "disconnected"}
        </span>
        <span className="badge mode">{mode}</span>
        {sourceName && <span className="badge">{sourceName}</span>}
        <span className="spacer" />
        <span className="muted">
          {stamp ? new Date(stamp * 1000).toLocaleTimeString([], { hour12: false }) : "—"}
        </span>
      </div>

      <div className="sidebar">
        <RosterPanel />
        <LayerToggles />
        <CameraControls />
        <RecordingControls />
        <ScenarioPanel />
        <RecordingBrowser />
      </div>

      <div className="detail">
        <EntityDetailPanel />
        <div style={{ marginTop: 8, display: "flex", flexDirection: "column", maxHeight: 300 }}>
          <EventLog />
        </div>
      </div>

      <Dock />
    </div>
  );
}
