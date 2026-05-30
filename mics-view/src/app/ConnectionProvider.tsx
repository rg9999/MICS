import React, { createContext, useContext, useEffect, useMemo, useRef } from "react";
import { config } from "./config";
import { useStore } from "./store";
import type { AckMessage, ServerMessage } from "../data/types";

// Thin WebSocket client: routes frame/log/status/hello into the store and
// turns control actions into request/response promises keyed by action name.
class GatewayClient {
  private ws: WebSocket | null = null;
  private url: string;
  private pending = new Map<string, ((ack: AckMessage) => void)[]>();
  private closed = false;

  constructor(url: string) {
    this.url = url;
  }

  connect() {
    this.closed = false;
    this.open();
  }

  private open() {
    const ws = new WebSocket(this.url);
    this.ws = ws;
    ws.onopen = () => {
      useStore.getState().setConnected(true);
      // hydrate control-channel-derived panels
      void this.refresh();
    };
    ws.onclose = () => {
      useStore.getState().setConnected(false);
      this.rejectAll();
      if (!this.closed) setTimeout(() => this.open(), config.reconnectMs);
    };
    ws.onerror = () => ws.close();
    ws.onmessage = (ev) => this.onMessage(ev.data as string);
  }

  private onMessage(raw: string) {
    let msg: ServerMessage;
    try {
      msg = JSON.parse(raw) as ServerMessage;
    } catch {
      return;
    }
    if (msg.type === "ack") {
      const q = this.pending.get(msg.action);
      const resolve = q?.shift();
      if (resolve) resolve(msg);
      return;
    }
    useStore.getState().applyMessage(msg);
  }

  private rejectAll() {
    this.pending.clear();
  }

  request<T = unknown>(action: string, extra: Record<string, unknown> = {}): Promise<T> {
    return new Promise<T>((resolve, reject) => {
      if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
        reject(new Error("not connected"));
        return;
      }
      const q = this.pending.get(action) ?? [];
      q.push((ack) => {
        if (ack.ok) resolve(ack.result as T);
        else reject(new Error(ack.error ?? "control error"));
      });
      this.pending.set(action, q);
      this.ws.send(JSON.stringify({ type: "control", action, ...extra }));
    });
  }

  async refresh() {
    const st = useStore.getState();
    try {
      st.setRecording(await this.request("recording.status"));
    } catch { /* controls may be disabled */ }
    try {
      st.setRecordings(await this.request("recordings.list"));
    } catch { /* ignore */ }
    try {
      st.setScenarios(await this.request("scenarios.list"));
    } catch { /* ignore */ }
  }

  close() {
    this.closed = true;
    this.ws?.close();
  }
}

const Ctx = createContext<GatewayClient | null>(null);

export function ConnectionProvider({ children }: { children: React.ReactNode }) {
  const clientRef = useRef<GatewayClient | null>(null);
  if (clientRef.current === null) clientRef.current = new GatewayClient(config.wsUrl);

  useEffect(() => {
    const c = clientRef.current!;
    c.connect();
    return () => c.close();
  }, []);

  const value = useMemo(() => clientRef.current!, []);
  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}

export function useGateway(): GatewayClient {
  const c = useContext(Ctx);
  if (!c) throw new Error("useGateway must be used within ConnectionProvider");
  return c;
}
