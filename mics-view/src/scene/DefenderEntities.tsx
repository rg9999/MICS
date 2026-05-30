import { Entity, PointGraphics, LabelGraphics, PolylineGraphics } from "resium";
import {
  Cartesian2, Color, LabelStyle, VerticalOrigin, HeightReference,
} from "cesium";
import { useStore } from "../app/store";
import { STATE_COLORS } from "../app/config";
import { toCartesian, enuTip } from "./transforms";

const VELOCITY_SCALE = 3.0; // seconds of look-ahead drawn as the arrow length

export function DefenderEntities() {
  const drones = useStore((s) => s.drones);
  const layers = useStore((s) => s.layers);
  const selection = useStore((s) => s.selection);
  const setSelection = useStore((s) => s.setSelection);

  if (!layers.defenders) return null;

  return (
    <>
      {drones.map((d) => {
        const color = Color.fromCssColorString(STATE_COLORS[d.state] ?? STATE_COLORS.unknown);
        const selected = selection?.kind === "drone" && selection.id === d.droneId;
        const pos = toCartesian(d.position);
        return (
          <Entity
            key={`drone-${d.droneId}`}
            position={pos}
            onClick={() => setSelection({ kind: "drone", id: d.droneId })}
          >
            <PointGraphics
              pixelSize={selected ? 16 : 11}
              color={color}
              outlineColor={selected ? Color.WHITE : Color.BLACK}
              outlineWidth={selected ? 3 : 1}
              heightReference={HeightReference.NONE}
            />
            <LabelGraphics
              text={`D${d.droneId} ${d.state}`}
              font="12px monospace"
              fillColor={Color.WHITE}
              outlineColor={Color.BLACK}
              outlineWidth={3}
              style={LabelStyle.FILL_AND_OUTLINE}
              verticalOrigin={VerticalOrigin.BOTTOM}
              pixelOffset={new Cartesian2(0, -16)}
              showBackground
              backgroundColor={Color.fromCssColorString("#101418cc")}
            />
            {layers.velocity && (
              <PolylineGraphics
                positions={[pos, enuTip(d.position, d.velEnu, VELOCITY_SCALE)]}
                width={3}
                material={Color.CYAN.withAlpha(0.9)}
              />
            )}
          </Entity>
        );
      })}
    </>
  );
}
