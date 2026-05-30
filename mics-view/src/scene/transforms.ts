import { Cartesian3, Matrix4, Transforms } from "cesium";
import type { LonLatAlt, Vec3 } from "../data/types";

export function toCartesian(pos: LonLatAlt): Cartesian3 {
  return Cartesian3.fromDegrees(pos[0], pos[1], pos[2]);
}

// Tip of an ENU vector anchored at a geodetic origin: origin + R_enu->ecef * (v*scale).
// Used for velocity arrows so a local-frame vector renders correctly on the globe.
export function enuTip(origin: LonLatAlt, v: Vec3, scale: number): Cartesian3 {
  const o = toCartesian(origin);
  const frame = Transforms.eastNorthUpToFixedFrame(o);
  const local = new Cartesian3(v[0] * scale, v[1] * scale, v[2] * scale);
  const rotated = Matrix4.multiplyByPointAsVector(frame, local, new Cartesian3());
  return Cartesian3.add(o, rotated, new Cartesian3());
}

export function trailToCartesians(
  pts: { lon: number; lat: number; alt: number }[],
): Cartesian3[] {
  return pts.map((p) => Cartesian3.fromDegrees(p.lon, p.lat, p.alt));
}
