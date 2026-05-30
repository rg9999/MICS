import React, { createContext, useContext, useEffect, useMemo, useRef } from "react";
import { config } from "./config";
import { useStore } from "./store";
import type { AckMessage, ServerMessage } from "../data/types";

// Render is throttled to this rate: frame messages arrive from the gateway at
// ~25 Hz, but applying every one re-renders all Cesium entities and starves the
// renderer. We coalesce to the latest frame and flush at FRAME_FLUSH_HZ.
const FRAME_FLUSH_HZ = 5;
const FRAME_FLUSH_MS = 1000 / FRAME_FLUSH_HZ;

// Thin WebSocket client: routes frame/log/status/hello into the store and
// turns control actions into request/response promises keyed by action name.
class GatewayClient {
  private ws: WebSocket | null = null;
  private url: string;
  private pending = new Map<string, ((ack: AckMessage) => void)[]>();
  private closed = false;
  private pendingFrame: Extract<ServerMessage, { type: "frame" }> | null = null;
  private flushTimer: ReturnType<typeof setInterval> | null = null;

  constructor(url: string) {
    this.url = url;
  }

  connect() {
    this.closed = false;
    if (this.flushTimer === null) {
      this.flushTimer = setInterval(() => this.flushFrame(), FRAME_FLUSH_MS);
    }
    this.open();
  }

  private flushFrame() {
    const m = this.pendingFrame;
    if (m === null) return;
    this.pendingFrame = null;
    useStore.getState().applyMessage(m);
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
    // Coalesce frames to the latest, but carry forward un-applied capture
    // events so throttling the render never drops them.
    if (msg.type === "frame") {
      if (this.pendingFrame && this.pendingFrame.events.length) {
        msg.events = [...this.pendingFrame.events, ...msg.events];
      }
      this.pendingFrame = msg;
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
    if (this.flushTimer !== null) {
      clearInterval(this.flushTimer);
      this.flushTimer = null;
    }
    this.pendingFrame = null;
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
