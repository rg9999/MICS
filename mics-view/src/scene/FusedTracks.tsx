import { Entity, PointGraphics, EllipsoidGraphics } from "resium";
import { Cartesian3, Color } from "cesium";
import { useStore } from "../app/store";
import { toCartesian } from "./transforms";

// Onboard fused TargetEstimate per drone (gateway `estimates`, keyed by drone).
// Visually distinct from GS cues: a cyan marker + uncertainty sphere (FR-V-9).
export function FusedTracks() {
  const estimates = useStore((s) => s.estimates);
  const layers = useStore((s) => s.layers);

  if (!layers.fusedTracks) return null;

  return (
    <>
      {Object.entries(estimates).map(([droneId, est]) => {
        const sigma = Math.max(est.posSigmaM, 1);
        return (
          <Entity key={`est-${droneId}`} position={toCartesian(est.position)}>
            <PointGraphics pixelSize={8} color={Color.CYAN} outlineColor={Color.BLACK} outlineWidth={1} />
            <EllipsoidGraphics
              radii={new Cartesian3(sigma, sigma, sigma)}
              material={Color.CYAN.withAlpha(0.15)}
              outline
              outlineColor={Color.CYAN.withAlpha(0.5)}
            />
          </Entity>
        );
      })}
    </>
  );
}
