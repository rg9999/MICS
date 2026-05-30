import {
  Entity, PointGraphics, LabelGraphics, EllipsoidGraphics, PolylineGraphics,
} from "resium";
import {
  Cartesian2, Cartesian3, Color, LabelStyle, VerticalOrigin,
} from "cesium";
import { useStore } from "../app/store";
import { toCartesian, enuTip } from "./transforms";

const VELOCITY_SCALE = 3.0;

// Targets/cues: a red marker + a 1-sigma uncertainty sphere (posSigmaM), with
// opacity falling off as the track ages (staleness styling, FR-V-11).
export function TargetEntities() {
  const tracks = useStore((s) => s.tracks);
  const layers = useStore((s) => s.layers);
  const selection = useStore((s) => s.selection);
  const setSelection = useStore((s) => s.setSelection);

  if (!layers.targets && !layers.cues) return null;

  return (
    <>
      {tracks.map((t) => {
        const selected = selection?.kind === "target" && selection.id === t.targetId;
        const pos = toCartesian(t.position);
        const staleness = Math.max(0.25, 1 - Math.min(t.age, 10) / 10);
        const sigma = Math.max(t.posSigmaM, 1);
        return (
          <Entity
            key={`target-${t.targetId}`}
            position={pos}
            onClick={() => setSelection({ kind: "target", id: t.targetId })}
          >
            <PointGraphics
              pixelSize={selected ? 15 : 10}
              color={Color.ORANGERED.withAlpha(staleness)}
              outlineColor={selected ? Color.WHITE : Color.BLACK}
              outlineWidth={selected ? 3 : 1}
            />
            <LabelGraphics
              text={`T${t.targetId} ${(t.classConfidence * 100).toFixed(0)}%`}
              font="12px monospace"
              fillColor={Color.WHITE}
              outlineColor={Color.BLACK}
              outlineWidth={3}
              style={LabelStyle.FILL_AND_OUTLINE}
              verticalOrigin={VerticalOrigin.BOTTOM}
              pixelOffset={new Cartesian2(0, -16)}
            />
            {layers.cues && (
              <EllipsoidGraphics
                radii={new Cartesian3(sigma, sigma, sigma)}
                material={Color.ORANGERED.withAlpha(0.18 * staleness)}
                outline
                outlineColor={Color.ORANGERED.withAlpha(0.4)}
              />
            )}
            {layers.velocity && t.velEnu && (
              <PolylineGraphics
                positions={[pos, enuTip(t.position, t.velEnu, VELOCITY_SCALE)]}
                width={3}
                material={Color.YELLOW.withAlpha(0.85)}
              />
            )}
          </Entity>
        );
      })}
    </>
  );
}
