export type StrategyInfo = {
  strategyId: string;
  displayNameZh?: string;
  affinity?: string;
  geometrySchema?: {
    allowCrossWorkpiece?: boolean;
    primaryRole?: string;
    slots?: { role?: string; kind?: string; min?: number; max?: number }[];
  };
};

export type GeometryRef = {
  kind: "Edge" | "Face" | "Polyline";
  role?: string;
  index?: number;
  workpiece?: { backendIdUtf8?: string; stepPathUtf8?: string };
  polylineXyz?: number[];
};

export type FeatureGeom = {
  strategyId: string;
  geometry?: {
    refs?: GeometryRef[];
    /** 旧 JSON 升级兼容 */
    faceIndices?: number[];
    edgeIndices?: number[];
  };
  workpiece?: { backendIdUtf8?: string; stepPathUtf8?: string };
};

export function geometryRefs(g: FeatureGeom["geometry"] | undefined): GeometryRef[] {
  if (!g) return [];
  if (Array.isArray(g.refs) && g.refs.length) return g.refs;
  const out: GeometryRef[] = [];
  for (const index of g.edgeIndices || []) out.push({ kind: "Edge", index });
  for (const index of g.faceIndices || []) out.push({ kind: "Face", index });
  return out;
}

export function collectIndices(g: FeatureGeom["geometry"] | undefined, kind: "Edge" | "Face"): number[] {
  return geometryRefs(g)
    .filter((r) => r.kind === kind && typeof r.index === "number" && r.index >= 0)
    .map((r) => r.index as number);
}

export function isFaceStrategy(id: string, catalog: StrategyInfo[] = []) {
  const hit = catalog.find((s) => s.strategyId === id);
  if (hit?.affinity) return hit.affinity === "Face";
  return String(id || "").startsWith("Face");
}

export function schemaAllowsCrossWorkpiece(id: string, catalog: StrategyInfo[] = []) {
  const hit = catalog.find((s) => s.strategyId === id);
  if (hit?.geometrySchema) return !!hit.geometrySchema.allowCrossWorkpiece;
  return id === "FaceOffsetCurve";
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
  const hasFace = collectIndices(f.geometry, "Face").length > 0;
  const hasEdge = collectIndices(f.geometry, "Edge").length > 0;
  if (hasFace === hasEdge) return f.strategyId;
  if (hasFace && !isFaceStrategy(f.strategyId, catalog)) return defaultStrategyForPick(true, catalog);
  if (hasEdge && isFaceStrategy(f.strategyId, catalog)) return defaultStrategyForPick(false, catalog);
  return f.strategyId;
}

export function strategyFilterForFeature(f: FeatureGeom | undefined): "Face" | "Line" | "Any" {
  if (!f) return "Any";
  const faces = collectIndices(f.geometry, "Face");
  const edges = collectIndices(f.geometry, "Edge");
  if (faces.length && !edges.length) return "Face";
  if (edges.length && !faces.length) return "Line";
  return "Any";
}

export function filterStrategies(catalog: StrategyInfo[], filter: "Face" | "Line" | "Any") {
  if (filter === "Any") return catalog;
  return catalog.filter((s) => !s.affinity || s.affinity === filter || s.affinity === "Any");
}

export function nextSlotRole(
  strategyId: string,
  geometry: FeatureGeom["geometry"] | undefined,
  pickFace: boolean,
  catalog: StrategyInfo[],
): string | null {
  const schema = catalog.find((s) => s.strategyId === strategyId)?.geometrySchema;
  const want = pickFace ? "Face" : "Edge";
  const refs = geometryRefs(geometry);
  if (!schema?.slots?.length) return "";
  for (const slot of schema.slots) {
    if ((slot.kind || "") !== want) continue;
    const role = slot.role || "";
    const have = role
      ? refs.filter((r) => r.role === role).length
      : refs.filter((r) => r.kind === want).length;
    const max = slot.max == null || slot.max < 0 ? 1e9 : slot.max;
    if (have < max) return role;
  }
  return null;
}
