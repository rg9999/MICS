import { Entity, PolylineGraphics } from "resium";
import { Color, PolylineDashMaterialProperty } from "cesium";
import { useStore } from "../app/store";
import { toCartesian } from "./transforms";

// One polyline per assignment, drone -> assigned target. Solid for primary,
// dashed for standby (FR-V-12). targetId 0 == unassigned (skipped).
export function AssignmentLinks() {
  const assignments = useStore((s) => s.assignments);
  const drones = useStore((s) => s.drones);
  const tracks = useStore((s) => s.tracks);
  const layers = useStore((s) => s.layers);

  if (!layers.assignments) return null;

  const droneById = new Map(drones.map((d) => [d.droneId, d]));
  const trackById = new Map(tracks.map((t) => [t.targetId, t]));

  return (
    <>
      {assignments.map((a) => {
        if (!a.targetId) return null;
        const d = droneById.get(a.droneId);
        const t = trackById.get(a.targetId);
        if (!d || !t) return null;
        const standby = a.role === "standby";
        const color = standby ? Color.GRAY : Color.LIME;
        return (
          <Entity key={`assign-${a.droneId}-${a.targetId}`}>
            <PolylineGraphics
              positions={[toCartesian(d.position), toCartesian(t.position)]}
              width={standby ? 1.5 : 2.5}
              material={
                standby
                  ? new PolylineDashMaterialProperty({ color: color.withAlpha(0.7) })
                  : color.withAlpha(0.8)
              }
            />
          </Entity>
        );
      })}
    </>
  );
}
