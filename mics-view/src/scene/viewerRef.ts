import type { Viewer as CesiumViewerType } from "cesium";

// Module-level handle to the live Cesium viewer so non-scene UI (CameraControls)
// can fly the camera without threading a ref through React context.
let current: CesiumViewerType | null = null;

export function setViewer(v: CesiumViewerType | null) {
  current = v;
}

export function getViewer(): CesiumViewerType | null {
  return current;
}
