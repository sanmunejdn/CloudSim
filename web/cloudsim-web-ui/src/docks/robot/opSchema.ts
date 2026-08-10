import { trajOpSchema } from "../../api";
import type { SchemaField } from "./featureSchema";

export type OpSchemaField = SchemaField & {
  vec3SuffixX?: string;
  vec3SuffixY?: string;
  vec3SuffixZ?: string;
  visibleWhenScopeKind?: string;
  visibleWhenFieldKey?: string;
  visibleWhenIntValue?: number;
  messageEn?: string;
};

export type PipelineOp = {
  kind: string;
  enabled?: boolean;
  scope?: Record<string, unknown>;
  params?: Record<string, unknown>;
};

export type OpSchema = {
  ok?: boolean;
  kind?: string;
  displayNameZh?: string;
  fields?: OpSchemaField[];
  values?: Record<string, unknown>;
  enabled?: boolean;
  error?: string;
};

/** 与 C++ OpScope::Kind 一致 */
export const OpScopeKind = {
  EntireProgram: 0,
  Group: 1,
  PointIndexRange: 2,
  InstructionIds: 3,
} as const;

const SCOPE_KIND_TOKENS: Record<number, string> = {
  [OpScopeKind.EntireProgram]: "EntireProgram",
  [OpScopeKind.Group]: "Group",
  [OpScopeKind.PointIndexRange]: "PointIndexRange",
  [OpScopeKind.InstructionIds]: "InstructionIds",
};

export function scopeKindToken(kind: unknown): string {
  const n = Number(kind);
  if (Number.isFinite(n) && SCOPE_KIND_TOKENS[n]) return SCOPE_KIND_TOKENS[n];
  const s = String(kind ?? "").trim();
  return s || "EntireProgram";
}

/** 对齐桌面 defaultScopeForNewOp：有 raw 点云 → P 范围 1…N */
export function defaultScopeForNewOp(
  rawPointCount: number,
  groupId?: string,
): Record<string, unknown> {
  const n = Math.floor(Number(rawPointCount) || 0);
  if (n > 0) {
    return { kind: OpScopeKind.PointIndexRange, pointFrom: 1, pointTo: n };
  }
  const gid = String(groupId || "").trim();
  if (gid) return { kind: OpScopeKind.Group, groupId: gid };
  return { kind: OpScopeKind.EntireProgram };
}

export function applySchemaValueToOp(op: PipelineOp, key: string, value: unknown) {
  if (!op.scope) op.scope = { kind: 0 };
  if (!op.params) op.params = {};
  if (key.startsWith("scope.")) {
    const sk = key.slice("scope.".length);
    if (sk === "kind") op.scope.kind = Number(value);
    else if (sk === "groupId") op.scope.groupId = String(value);
    else if (sk === "pointFrom") op.scope.pointFrom = Number(value);
    else if (sk === "pointTo") op.scope.pointTo = Number(value);
    else op.scope[sk] = value;
  } else {
    op.params[key] = value;
  }
}

export function readSchemaValueFromOp(op: PipelineOp, key: string) {
  if (key.startsWith("scope.")) {
    const sk = key.slice("scope.".length);
    return op.scope ? op.scope[sk] : undefined;
  }
  return op.params ? op.params[key] : undefined;
}

export function isExternalTcpBackendField(f: { key?: string }) {
  const k = f.key || "";
  return k === "toWorkpiece.externalTcpBackendId" || /externalTcpBackendId$/i.test(k);
}

const TO_WORKPIECE_MANUAL_TCP_KEYS = new Set([
  "toWorkpiece.externalTcpXMm",
  "toWorkpiece.externalTcpYMm",
  "toWorkpiece.externalTcpZMm",
  "toWorkpiece.externalTcpRxDeg",
  "toWorkpiece.externalTcpRyDeg",
  "toWorkpiece.externalTcpRzDeg",
]);

export function isOpFieldVisible(f: OpSchemaField, op: PipelineOp) {
  // schema 用 Group/PointIndexRange 令牌，op.scope.kind 是数字
  const token = scopeKindToken(op.scope?.kind ?? 0);
  const kindNum = String(Number(op.scope?.kind ?? 0));
  let vis = true;
  if (f.visibleWhenScopeKind) {
    const allowed = String(f.visibleWhenScopeKind)
      .split(",")
      .map((x) => x.trim())
      .filter(Boolean);
    if (allowed.length) vis = allowed.includes(token) || allowed.includes(kindNum);
  }
  if (f.visibleWhenFieldKey) {
    const other = readSchemaValueFromOp(op, f.visibleWhenFieldKey);
    vis = vis && Number(other) === Number(f.visibleWhenIntValue);
  }
  const externalTcpId = String(readSchemaValueFromOp(op, "toWorkpiece.externalTcpBackendId") || "").trim();
  if (TO_WORKPIECE_MANUAL_TCP_KEYS.has(f.key) && externalTcpId) vis = false;
  return vis;
}

/** 缺省/非法 P 范围时填 1…N；已是合法子区间则不动 */
export function ensurePointRangeScope(op: PipelineOp, rawPointCount: number): boolean {
  const n = Math.floor(Number(rawPointCount) || 0);
  if (n <= 0 || scopeKindToken(op.scope?.kind) !== "PointIndexRange") return false;
  if (!op.scope) op.scope = { kind: OpScopeKind.PointIndexRange };
  let from = Number(op.scope.pointFrom);
  let to = Number(op.scope.pointTo);
  const missing = !Number.isFinite(from) || !Number.isFinite(to);
  if (!Number.isFinite(from)) from = 1;
  if (!Number.isFinite(to)) to = 1;
  // 默认 1…1 视为未设置；合法窄区间（含单点）保留
  const unsetDefault = from === 1 && to === 1 && n > 1;
  const invalid = from < 1 || to < from || to > n || from > n;
  if (!missing && !unsetDefault && !invalid) return false;
  op.scope.pointFrom = 1;
  op.scope.pointTo = n;
  return true;
}

export async function loadOpSchema(kind: string, opIndex: number): Promise<OpSchema | null> {
  if (!kind) return null;
  try {
    const r = (await trajOpSchema(kind, opIndex)) as OpSchema;
    if (!r?.ok) return r || null;
    return r;
  } catch {
    return null;
  }
}

/** 对齐桌面：scope 整块在上（P 起/止相邻），再按 order 排算子参数 */
export function sortOpSchemaFields(fields: OpSchemaField[]): OpSchemaField[] {
  return [...fields].sort((a, b) => {
    const ag = String(a.key || "").startsWith("scope.") ? 0 : 1;
    const bg = String(b.key || "").startsWith("scope.") ? 0 : 1;
    if (ag !== bg) return ag - bg;
    const ao = a.order || 0;
    const bo = b.order || 0;
    if (ao !== bo) return ao - bo;
    return String(a.key || "").localeCompare(String(b.key || ""));
  });
}
