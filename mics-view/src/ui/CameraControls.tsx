import { useEffect, useRef, useState } from "react";
import { Cartesian3, HeadingPitchRange, Math as CMath, Matrix4 } from "cesium";
import { useStore } from "../app/store";
import { getViewer } from "../scene/viewerRef";
import type { LonLatAlt } from "../data/types";

function selectedPosition(): LonLatAlt | null {
  const s = useStore.getState();
  if (!s.selection) return null;
  if (s.selection.kind === "drone") {
    return s.drones.find((d) => d.droneId === s.selection!.id)?.position ?? null;
  }
  return s.tracks.find((t) => t.targetId === s.selection!.id)?.position ?? null;
}

// Camera helpers: one-shot fly-to-selection, home (datum), and a follow toggle
// that keeps the selected entity centred each frame.
export function CameraControls() {
  const selection = useStore((s) => s.selection);
  const datum = useStore((s) => s.datum);
  const stamp = useStore((s) => s.stamp);
  const [follow, setFollow] = useState(false);
  const followRef = useRef(false);
  followRef.current = follow;

  // re-centre on every frame while following
  useEffect(() => {
    if (!follow) return;
    const v = getViewer();
    const pos = selectedPosition();
    if (!v || !pos) return;
    v.camera.lookAt(
      Cartesian3.fromDegrees(pos[0], pos[1], pos[2]),
      new HeadingPitchRange(CMath.toRadians(0), CMath.toRadians(-30), 600),
    );
  }, [follow, stamp, selection]);

  // release the camera transform when follow turns off
  useEffect(() => {
    if (follow) return;
    getViewer()?.camera.lookAtTransform(Matrix4.IDENTITY);
  }, [follow]);

  const flyToSelected = () => {
    const v = getViewer();
    const pos = selectedPosition();
    if (!v || !pos) return;
    v.camera.flyToBoundingSphere(
      { center: Cartesian3.fromDegrees(pos[0], pos[1], pos[2]), radius: 400 } as never,
      { duration: 0.8 },
    );
  };

  const flyHome = () => {
    const v = getViewer();
    if (!v) return;
    setFollow(false);
    v.camera.flyTo({
      destination: Cartesian3.fromDegrees(datum.lon, datum.lat - 0.03, 4000),
      orientation: { heading: 0, pitch: CMath.toRadians(-35), roll: 0 },
      duration: 0.8,
    });
  };

  return (
    <div className="card">
      <h3>Camera</h3>
      <div className="row">
        <button onClick={flyToSelected} disabled={!selection}>Fly to selected</button>
        <button onClick={flyHome}>Home</button>
      </div>
      <label className="toggle" style={{ marginTop: 6 }}>
        <input
          type="checkbox"
          checked={follow}
          disabled={!selection}
          onChange={(e) => setFollow(e.target.checked)}
        />
        Follow selected
      </label>
    </div>
  );
}
