/// @file uiEvents.ts
/// @brief 网页端领域事件总线（替代 window CustomEvent 隐式耦合）

type Handler<T> = (detail: T) => void;

class UiEventBus {
  private map = new Map<string, Set<Handler<unknown>>>();

  on<T>(type: string, fn: Handler<T>): () => void {
    if (!this.map.has(type)) this.map.set(type, new Set());
    const set = this.map.get(type)!;
    const wrapped = fn as Handler<unknown>;
    set.add(wrapped);
    return () => set.delete(wrapped);
  }

  emit<T>(type: string, detail: T): void {
    this.map.get(type)?.forEach((fn) => fn(detail));
  }
}

export const uiEvents = new UiEventBus();

export type PickHighlightDetail = {
  clear?: boolean;
  polys?: number[][][];
  soup?: number[] | Float32Array;
};

export type RawPreviewDetail = {
  preview?: {
    ok?: boolean;
    pointsMm?: number[][];
    eulersDeg?: number[][];
    axesX?: number[][];
    axesY?: number[][];
    axesZ?: number[][];
    segmentEndExclusive?: number[];
    pointCount?: number;
  } | null;
  axisOpts?: { x: boolean; y: boolean; z: boolean; interval: number };
};

export type MateFaceDetail = {
  slot: 0 | 1;
  faceIndex: number;
  backendId: string;
  pickWorldMm?: number[];
  soupWorldMm?: unknown;
};

export type PickCommitDetail = Record<string, unknown>;

export const UI_EVT = {
  focusProps: "focusProps",
  pickHighlight: "pickHighlight",
  pickCommit: "pickCommit",
  rawPreview: "rawPreview",
  mateFace: "mateFace",
} as const;
