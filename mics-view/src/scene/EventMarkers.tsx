import { Entity, PointGraphics, LabelGraphics } from "resium";
import { Cartesian2, Color, LabelStyle, VerticalOrigin } from "cesium";
import { useStore } from "../app/store";
import { toCartesian } from "./transforms";

const RESULT_COLOR: Record<string, Color> = {
  success: Color.LIMEGREEN,
  miss: Color.RED,
  attempt: Color.YELLOW,
  unknown: Color.GRAY,
};

// Capture events (attempt/success/miss) as timestamped markers (FR-V-13).
export function EventMarkers() {
  const events = useStore((s) => s.events);
  const layers = useStore((s) => s.layers);

  if (!layers.events) return null;

  return (
    <>
      {events.map((e, i) => {
        const color = RESULT_COLOR[e.result] ?? RESULT_COLOR.unknown;
        return (
          <Entity key={`evt-${e.droneId}-${e.targetId}-${e.stamp}-${i}`} position={toCartesian(e.position)}>
            <PointGraphics pixelSize={14} color={color.withAlpha(0.85)} outlineColor={Color.WHITE} outlineWidth={2} />
            <LabelGraphics
              text={`${e.result.toUpperCase()} D${e.droneId}->T${e.targetId}`}
              font="11px monospace"
              fillColor={Color.WHITE}
              outlineColor={Color.BLACK}
              outlineWidth={3}
              style={LabelStyle.FILL_AND_OUTLINE}
              verticalOrigin={VerticalOrigin.TOP}
              pixelOffset={new Cartesian2(0, 14)}
            />
          </Entity>
        );
      })}
    </>
  );
}
