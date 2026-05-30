import { useStore } from "../app/store";
import { LAYER_KEYS, type LayerKey } from "../data/types";

const LABELS: Record<LayerKey, string> = {
  defenders: "Defenders",
  targets: "Targets",
  cues: "Cues (σ)",
  assignments: "Assignments",
  fusedTracks: "Fused tracks",
  events: "Events",
  velocity: "Velocity",
  trails: "Trails",
  radar: "Radar*",
  fov: "FOV*",
  lidar: "LiDAR*",
  volumes: "Volumes*",
};

// Layers marked * have no gateway data source yet (no sensor topics flowing);
// the toggle exists but renders nothing until those streams are added.
export function LayerToggles() {
  const layers = useStore((s) => s.layers);
  const toggle = useStore((s) => s.toggleLayer);

  return (
    <div className="card">
      <h3>Layers</h3>
      <div className="toggle-grid">
        {LAYER_KEYS.map((k) => (
          <label className="toggle" key={k}>
            <input type="checkbox" checked={layers[k]} onChange={() => toggle(k)} />
            {LABELS[k]}
          </label>
        ))}
      </div>
    </div>
  );
}
