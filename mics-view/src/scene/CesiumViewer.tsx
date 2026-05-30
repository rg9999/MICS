import { useEffect, useRef } from "react";
import { Viewer } from "resium";
import {
  Cartesian2, Cartesian3, Ion, Math as CMath, ScreenSpaceEventType,
  type Viewer as CesiumViewerType,
} from "cesium";
import { useStore } from "../app/store";
import { setViewer } from "./viewerRef";
import { DefenderEntities } from "./DefenderEntities";
import { TargetEntities } from "./TargetEntities";
import { AssignmentLinks } from "./AssignmentLinks";
import { FusedTracks } from "./FusedTracks";
import { EventMarkers } from "./EventMarkers";
import { Trails } from "./Trails";

// Air-gapped: never contact Cesium Ion. An empty token + no base imagery keeps
// the globe fully offline (PRD §1.1, §11.4). Self-hosted terrain/imagery can be
// added later by pointing at local tile URLs.
Ion.defaultAccessToken = "";

export function CesiumViewer() {
  const ref = useRef<{ cesiumElement?: CesiumViewerType }>(null);
  const datum = useStore((s) => s.datum);
  const setSelection = useStore((s) => s.setSelection);

  useEffect(() => {
    const viewer = ref.current?.cesiumElement;
    if (!viewer) return;
    setViewer(viewer);

    // initial camera: look at the datum from the south-east, a few km up
    viewer.camera.setView({
      destination: Cartesian3.fromDegrees(datum.lon, datum.lat - 0.03, 4000),
      orientation: { heading: CMath.toRadians(0), pitch: CMath.toRadians(-35), roll: 0 },
    });
    // we drive selection ourselves; clicking empty space clears it
    viewer.selectedEntity = undefined;
    viewer.screenSpaceEventHandler.setInputAction((click: { position: Cartesian2 }) => {
      const picked = viewer.scene.pick(click.position);
      if (!picked) setSelection(null);
    }, ScreenSpaceEventType.LEFT_CLICK);
    return () => setViewer(null);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  return (
    <Viewer
      ref={ref}
      full
      baseLayer={false as unknown as undefined}
      baseLayerPicker={false}
      geocoder={false}
      homeButton={false}
      sceneModePicker={false}
      navigationHelpButton={false}
      timeline={true}
      animation={true}
      infoBox={false}
      selectionIndicator={false}
      creditContainer={undefined}
    >
      <Trails />
      <DefenderEntities />
      <TargetEntities />
      <AssignmentLinks />
      <FusedTracks />
      <EventMarkers />
    </Viewer>
  );
}
