import React from "react";
import ReactDOM from "react-dom/client";
import "cesium/Build/Cesium/Widgets/widgets.css";
import "ag-grid-community/styles/ag-grid.css";
import "ag-grid-community/styles/ag-theme-quartz.css";
import "./styles.css";
import { ConnectionProvider } from "./app/ConnectionProvider";
import { App } from "./App";

ReactDOM.createRoot(document.getElementById("root")!).render(
  <React.StrictMode>
    <ConnectionProvider>
      <App />
    </ConnectionProvider>
  </React.StrictMode>,
);
