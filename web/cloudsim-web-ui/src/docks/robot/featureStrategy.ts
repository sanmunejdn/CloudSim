export type StrategyInfo = {
  strategyId: string;
  displayNameZh?: string;
  affinity?: string;
};

export type FeatureGeom = {
  strategyId: string;
  geometry?: { faceIndices?: number[]; edgeIndices?: number[] };
};

export function isFaceStrategy(id: string, catalog: StrategyInfo[] = []) {
  const hit = catalog.find((s) => s.strategyId === id);
  if (hit?.affinity) return hit.affinity === "Face";
  return String(id || "").startsWith("Face");
}

/** 对齐桌面 defaultStrategyIdForGeometry：面→FaceBoundary，线→EdgeChain */
export function defaultStrategyForPick(pickFace: boolean, catalog: StrategyInfo[]) {
  const required = pickFace ? "Face" : "Line";
  const preferred = pickFace ? "FaceBoundary" : "EdgeChain";
  if (catalog.some((s) => s.strategyId === preferred)) return preferred;
  const hit = catalog.find((s) => s.affinity === required);
  if (hit) return hit.strategyId;
  return preferred;
}

/** 拾取模式与离散策略对齐，避免面拾取仍走线策略 */
export function resolveFeatureStrategy(
  pickMode: "face" | "edge" | null,
  current: string,
  catalog: StrategyInfo[],
) {
  if (pickMode === "face") {
    if (!isFaceStrategy(current, catalog)) return defaultStrategyForPick(true, catalog);
    return current || defaultStrategyForPick(true, catalog);
  }
  if (pickMode === "edge") {
    if (!current || isFaceStrategy(current, catalog)) return defaultStrategyForPick(false, catalog);
    return current;
  }
  return current || defaultStrategyForPick(false, catalog);
}

export function normalizeFeatureStrategy(f: FeatureGeom, catalog: StrategyInfo[]) {
  const hasFace = (f.geometry?.faceIndices || []).length > 0;
  const hasEdge = (f.geometry?.edgeIndices || []).length > 0;
  if (hasFace === hasEdge) return f.strategyId;
  if (hasFace && !isFaceStrategy(f.strategyId, catalog)) return defaultStrategyForPick(true, catalog);
  if (hasEdge && isFaceStrategy(f.strategyId, catalog)) return defaultStrategyForPick(false, catalog);
  return f.strategyId;
}

export function strategyFilterForFeature(f: FeatureGeom | undefined): "Face" | "Line" | "Any" {
  if (!f) return "Any";
  const faces = f.geometry?.faceIndices || [];
  const edges = f.geometry?.edgeIndices || [];
  if (faces.length && !edges.length) return "Face";
  if (edges.length && !faces.length) return "Line";
  return "Any";
}

export function filterStrategies(catalog: StrategyInfo[], filter: "Face" | "Line" | "Any") {
  if (filter === "Any") return catalog;
  return catalog.filter((s) => !s.affinity || s.affinity === filter || s.affinity === "Any");
}
