import { useMemo, useState } from "react";
import { AgGridReact } from "ag-grid-react";
import type { CellClassParams, ColDef } from "ag-grid-community";
import { useStore } from "../../app/store";
import { LEVEL_COLORS } from "../../app/config";
import type { LogLevelName, LogRecord } from "../../data/types";

const LEVEL_ORDER: Record<LogLevelName, number> = {
  DEBUG: 0, INFO: 1, WARN: 2, ERROR: 3, FATAL: 4,
};
const LEVELS: LogLevelName[] = ["DEBUG", "INFO", "WARN", "ERROR", "FATAL"];

function clock(stamp: number): string {
  const d = new Date(stamp * 1000);
  return `${d.toLocaleTimeString([], { hour12: false })}.${String(d.getMilliseconds()).padStart(3, "0")}`;
}

// Aggregated /rosout process log (ring buffer). Min-level filter + free-text
// quick filter; newest rows on top, coloured by severity.
export function ProcessLogGrid() {
  const logs = useStore((s) => s.logs);
  const [minLevel, setMinLevel] = useState<LogLevelName>("INFO");
  const [quick, setQuick] = useState("");

  const rows: LogRecord[] = useMemo(() => {
    const floor = LEVEL_ORDER[minLevel];
    const out: LogRecord[] = [];
    for (let i = logs.length - 1; i >= 0; i--) {
      const r = logs[i];
      if (LEVEL_ORDER[r.level] < floor) continue;
      out.push(r);
    }
    return out;
  }, [logs, minLevel]);

  const cols = useMemo<ColDef<LogRecord>[]>(() => [
    { headerName: "Time", valueGetter: (p) => p.data ? clock(p.data.stamp) : "", width: 130, pinned: "left" },
    {
      field: "level", headerName: "Level", width: 80,
      cellStyle: (p: CellClassParams<LogRecord>) => ({
        color: LEVEL_COLORS[p.value as string] ?? "#cfd3da", fontWeight: 600,
      }),
    },
    { field: "source", headerName: "Source", width: 160 },
    { field: "msg", headerName: "Message", flex: 1, minWidth: 240, tooltipField: "msg" },
  ], []);

  return (
    <div style={{ display: "flex", flexDirection: "column", height: "100%" }}>
      <div className="row" style={{ padding: "4px 8px" }}>
        <span className="muted">Level</span>
        <select
          value={minLevel}
          onChange={(e) => setMinLevel(e.target.value as LogLevelName)}
          style={{ width: 100 }}
        >
          {LEVELS.map((l) => <option key={l} value={l}>{l}+</option>)}
        </select>
        <input
          type="text"
          placeholder="filter…"
          value={quick}
          onChange={(e) => setQuick(e.target.value)}
          style={{ flex: 1 }}
        />
        <span className="muted">{rows.length} rows</span>
      </div>
      <div className="ag-theme-quartz" style={{ flex: 1, minHeight: 0 }}>
        <AgGridReact<LogRecord>
          rowData={rows}
          columnDefs={cols}
          quickFilterText={quick}
          suppressCellFocus
          headerHeight={28}
          rowHeight={24}
        />
      </div>
    </div>
  );
}
