import { Entity, PolylineGraphics } from "resium";
import { ArcType, Color } from "cesium";
import { useStore } from "../app/store";
import { STATE_COLORS } from "../app/config";
import { trailToCartesians } from "./transforms";

// Movement history for defenders + targets. Rendered as standalone polyline
// entities (no `position`) so the geometry is driven purely by `positions`;
// ArcType.NONE keeps the line on the actual 3D points instead of draping it to
// the ellipsoid surface. Trail points are accumulated in the store per frame.
export function Trails() {
  const trails = useStore((s) => s.trails);
  const drones = useStore((s) => s.drones);
  const layers = useStore((s) => s.layers);

  if (!layers.trails) return null;

  const droneColor = new Map<number, Color>();
  for (const d of drones) {
    droneColor.set(d.droneId,
      Color.fromCssColorString(STATE_COLORS[d.state] ?? STATE_COLORS.unknown));
  }

  return (
    <>
      {Array.from(trails.entries()).map(([key, pts]) => {
        if (pts.length < 2) return null;
        const [kind, idStr] = key.split(":");
        const id = Number(idStr);
        const color = kind === "drone"
          ? (droneColor.get(id) ?? Color.fromCssColorString(STATE_COLORS.unknown))
          : Color.ORANGERED;
        return (
          <Entity key={`trail-${key}`}>
            <PolylineGraphics
              positions={trailToCartesians(pts)}
              width={2}
              material={color.withAlpha(0.55)}
              arcType={ArcType.NONE}
            />
          </Entity>
        );
      })}
    </>
  );
}
