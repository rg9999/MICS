import { useMemo } from "react";
import { AgGridReact } from "ag-grid-react";
import type { ColDef, RowClickedEvent } from "ag-grid-community";
import { useStore } from "../../app/store";
import type { AllocationStatus, TargetDerived, TrackView } from "../../data/types";

interface Row extends TrackView {
  speed: number | null;
  altitude: number | null;
  allocation: AllocationStatus | null;
}

function num1(p: { value: number | null | undefined }) {
  return p.value === null || p.value === undefined || !Number.isFinite(p.value)
    ? "—" : p.value.toFixed(1);
}

function allocText(a: AllocationStatus | null): string {
  if (!a) return "—";
  if (a.kind === "ENGAGED") return `D${a.byDrone}`;
  if (a.kind === "CAPTURED") return "captured";
  return "unengaged";
}

// Tabular target/track list (one row per fused track) with derived kinematics
// and allocation status; row selection syncs to the shared store selection.
export function TargetGrid() {
  const tracks = useStore((s) => s.tracks);
  const derived = useStore((s) => s.targetsDerived);

  const rows: Row[] = useMemo(() => tracks.map((t) => {
    const td: TargetDerived | undefined = derived[t.targetId];
    return {
      ...t,
      speed: td?.speed ?? null,
      altitude: td?.altitude ?? null,
      allocation: td?.allocation ?? null,
    };
  }), [tracks, derived]);

  const cols = useMemo<ColDef<Row>[]>(() => [
    { headerName: "ID", valueGetter: (p) => `T${p.data?.targetId}`, width: 70, pinned: "left" },
    { field: "source", headerName: "Source", width: 110 },
    { field: "classConfidence", headerName: "Conf", valueFormatter: (p) => p.value != null ? `${(p.value * 100).toFixed(0)}%` : "—", width: 80 },
    { field: "speed", headerName: "Speed", valueFormatter: num1, width: 90 },
    { field: "altitude", headerName: "Alt", valueFormatter: num1, width: 90 },
    { field: "posSigmaM", headerName: "σ (m)", valueFormatter: num1, width: 90 },
    { field: "age", headerName: "Age", valueFormatter: num1, width: 80 },
    { headerName: "Alloc", valueGetter: (p) => allocText(p.data?.allocation ?? null), width: 110 },
  ], []);

  const onRowClicked = (e: RowClickedEvent<Row>) => {
    if (e.data) useStore.getState().setSelection({ kind: "target", id: e.data.targetId });
  };

  return (
    <div className="ag-theme-quartz grid-fill">
      <AgGridReact<Row>
        rowData={rows}
        columnDefs={cols}
        getRowId={(p) => `T${p.data.targetId}`}
        onRowClicked={onRowClicked}
        rowSelection="single"
        suppressCellFocus
        headerHeight={28}
        rowHeight={26}
      />
    </div>
  );
}
