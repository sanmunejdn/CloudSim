type Handler = (data: string, eventType: string) => void;

/** 单一 EventSource，按事件名 fan-out */
export class EventHub {
  private es: EventSource | null = null;
  private handlers = new Map<string, Set<Handler>>();
  private anyHandlers = new Set<Handler>();

  start(url = "/api/events") {
    this.stop();
    this.es = new EventSource(url);
    // Gateway 仅推 data: JSON（无 event: 名）；按 payload.type 再 fan-out
    this.es.onmessage = (ev) => {
      this.dispatch("message", ev.data);
      try {
        const t = (JSON.parse(ev.data) as { type?: string }).type;
        if (typeof t === "string" && t) this.dispatch(t, ev.data);
      } catch {
        /* 非 JSON keepalive 忽略 */
      }
    };
    this.es.addEventListener("RobotKinematicsApplied", (ev) =>
      this.dispatch("RobotKinematicsApplied", (ev as MessageEvent).data),
    );
    this.es.addEventListener("PoseCommitted", (ev) => this.dispatch("PoseCommitted", (ev as MessageEvent).data));
    this.es.addEventListener("ObjectPatched", (ev) => this.dispatch("ObjectPatched", (ev as MessageEvent).data));
    this.es.addEventListener("RobotCoordinateFramesChanged", (ev) =>
      this.dispatch("RobotCoordinateFramesChanged", (ev as MessageEvent).data),
    );
    this.es.addEventListener("SelectionChanged", (ev) => this.dispatch("SelectionChanged", (ev as MessageEvent).data));
    this.es.addEventListener("ProjectLoaded", (ev) => this.dispatch("ProjectLoaded", (ev as MessageEvent).data));
    this.es.addEventListener("ProjectSaved", (ev) => this.dispatch("ProjectSaved", (ev as MessageEvent).data));
    this.es.addEventListener("BackendObjectCreated", (ev) =>
      this.dispatch("BackendObjectCreated", (ev as MessageEvent).data),
    );
    this.es.addEventListener("BackendObjectRegistered", (ev) =>
      this.dispatch("BackendObjectRegistered", (ev as MessageEvent).data),
    );
    this.es.addEventListener("BackendObjectRemoved", (ev) =>
      this.dispatch("BackendObjectRemoved", (ev as MessageEvent).data),
    );
    this.es.addEventListener("WorkspaceModeChanged", (ev) =>
      this.dispatch("WorkspaceModeChanged", (ev as MessageEvent).data),
    );
    this.es.addEventListener("IoSignalsChanged", (ev) =>
      this.dispatch("IoSignalsChanged", (ev as MessageEvent).data),
    );
  }

  stop() {
    this.es?.close();
    this.es = null;
  }

  on(type: string, fn: Handler): () => void {
    if (!this.handlers.has(type)) this.handlers.set(type, new Set());
    this.handlers.get(type)!.add(fn);
    return () => {
      this.handlers.get(type)?.delete(fn);
    };
  }

  onAny(fn: Handler): () => void {
    this.anyHandlers.add(fn);
    return () => {
      this.anyHandlers.delete(fn);
    };
  }

  private dispatch(type: string, data: string) {
    this.handlers.get(type)?.forEach((fn) => fn(data, type));
    this.anyHandlers.forEach((fn) => fn(data, type));
  }
}

export const eventHub = new EventHub();
