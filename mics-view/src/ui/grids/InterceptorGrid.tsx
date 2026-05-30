import { useMemo, useRef } from "react";
import { AgGridReact } from "ag-grid-react";
import type { ColDef, GridApi, GridReadyEvent, RowClickedEvent } from "ag-grid-community";
import { useStore } from "../../app/store";
import type { DroneDerived, DroneView } from "../../data/types";

interface Row extends DroneView {
  speed: number | null;
  altitude: number | null;
  rangeToTarget: number | null;
  etaToInterceptS: number | null;
}

function num1(p: { value: number | null | undefined }) {
  return p.value === null || p.value === undefined || !Number.isFinite(p.value)
    ? "—" : p.value.toFixed(1);
}

// Tabular defender roster (one row per drone) with derived kinematics; row
// selection is synced to the shared store selection.
export function InterceptorGrid() {
  const drones = useStore((s) => s.drones);
  const derived = useStore((s) => s.dronesDerived);
  const apiRef = useRef<GridApi<Row> | null>(null);

  const rows: Row[] = useMemo(() => drones.map((d) => {
    const dd: DroneDerived | undefined = derived[d.droneId];
    return {
      ...d,
      speed: dd?.speed ?? null,
      altitude: dd?.altitude ?? null,
      rangeToTarget: dd?.rangeToTarget ?? null,
      etaToInterceptS: dd?.etaToInterceptS ?? null,
    };
  }), [drones, derived]);

  const cols = useMemo<ColDef<Row>[]>(() => [
    { headerName: "ID", valueGetter: (p) => `D${p.data?.droneId}`, width: 70, pinned: "left" },
    { field: "state", headerName: "State", width: 110 },
    { headerName: "Target", valueGetter: (p) => p.data?.currentTarget ? `T${p.data.currentTarget}` : "—", width: 80 },
    { field: "speed", headerName: "Speed", valueFormatter: num1, width: 90 },
    { field: "altitude", headerName: "Alt", valueFormatter: num1, width: 90 },
    { field: "rangeToTarget", headerName: "Range", valueFormatter: num1, width: 90 },
    { field: "etaToInterceptS", headerName: "ETA", valueFormatter: num1, width: 80 },
    { field: "batteryPct", headerName: "Batt%", valueFormatter: (p) => p.value?.toFixed(0) ?? "—", width: 80 },
    { field: "trackQuality", headerName: "TrkQ", valueFormatter: (p) => p.value?.toFixed(2) ?? "—", width: 80 },
  ], []);

  const onReady = (e: GridReadyEvent<Row>) => { apiRef.current = e.api; };
  const onRowClicked = (e: RowClickedEvent<Row>) => {
    if (e.data) useStore.getState().setSelection({ kind: "drone", id: e.data.droneId });
  };

  return (
    <div className="ag-theme-quartz grid-fill">
      <AgGridReact<Row>
        rowData={rows}
        columnDefs={cols}
        getRowId={(p) => `D${p.data.droneId}`}
        onGridReady={onReady}
        onRowClicked={onRowClicked}
        rowSelection="single"
        suppressCellFocus
        headerHeight={28}
        rowHeight={26}
      />
    </div>
  );
}
