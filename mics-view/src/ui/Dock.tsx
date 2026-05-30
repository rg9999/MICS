import { useState } from "react";
import { InterceptorGrid } from "./grids/InterceptorGrid";
import { TargetGrid } from "./grids/TargetGrid";
import { ProcessLogGrid } from "./grids/ProcessLogGrid";

type Tab = "interceptors" | "targets" | "log";

// Bottom dock: tabbed AG Grid tables (defenders / targets / process log).
export function Dock() {
  const [tab, setTab] = useState<Tab>("interceptors");

  return (
    <div className="dock">
      <div className="dock-tabs">
        <button className={tab === "interceptors" ? "active" : ""} onClick={() => setTab("interceptors")}>
          Interceptors
        </button>
        <button className={tab === "targets" ? "active" : ""} onClick={() => setTab("targets")}>
          Targets
        </button>
        <button className={tab === "log" ? "active" : ""} onClick={() => setTab("log")}>
          Process Log
        </button>
      </div>
      <div className="dock-body">
        {tab === "interceptors" && <InterceptorGrid />}
        {tab === "targets" && <TargetGrid />}
        {tab === "log" && <ProcessLogGrid />}
      </div>
    </div>
  );
}
