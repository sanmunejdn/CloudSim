import type { Instruction, PropRow } from "../../api";
import { enumOptionsForKey, propertyDisplayLabel } from "./propLabels";

export type InstrPropViewRow = PropRow & { kind?: "text" | "enum"; options?: string[] };

const NUMERIC_KEYS = new Set([
  "motion.target.pose.x",
  "motion.target.pose.y",
  "motion.target.pose.z",
  "motion.target.euler.rx",
  "motion.target.euler.ry",
  "motion.target.euler.rz",
  "motion.speed",
  "motion.acc",
  "motion.blendRadius",
]);

function isSkippedKey(key: string) {
  if (!key) return true;
  if (key.startsWith("context.") || key.startsWith("render.") || key.startsWith("legacy.")) return true;
  if (key === "motion.durationSec" || key === "motion.pointIndex") return true;
  if (key === "motion.tool.frameId" || key === "motion.user.frameId" || key === "motion.target.frame") return true;
  return false;
}

function fmtValue(key: string, raw: string | undefined) {
  const s = raw ?? "";
  if (!NUMERIC_KEYS.has(key)) return s;
  const n = Number(s);
  if (!Number.isFinite(n)) return s;
  return n.toFixed(3);
}

function formatPointIndex(ins?: Instruction | null) {
  const pi = Number(ins?.pointIndex) || 0;
  if (pi <= 0) return "-";
  return `P${pi}（第 ${pi} 点）`;
}

/** 对齐桌面 InstructionPropertyPanel 的字段集合与中文标签 */
export function buildInstrPropView(
  apiRows: PropRow[],
  instructionId: string,
  instruction?: Instruction | null,
): InstrPropViewRow[] {
  const byKey = new Map(apiRows.map((r) => [r.key, r]));

  const out: InstrPropViewRow[] = [
    {
      key: "core.id",
      label: propertyDisplayLabel("core.id"),
      value: instructionId,
      editable: false,
      kind: "text",
    },
    {
      key: "core.name",
      label: propertyDisplayLabel("core.name"),
      value: instruction?.name || byKey.get("core.name")?.value || "",
      editable: false,
      kind: "text",
    },
  ];

  const type = String(instruction?.type || "").toLowerCase();
  if (["ptp", "line", "arc"].includes(type) || byKey.has("motion.target.pose.x")) {
    out.push({
      key: "motion.pointIndex",
      label: propertyDisplayLabel("motion.pointIndex"),
      value: formatPointIndex(instruction),
      editable: false,
      kind: "text",
    });
  }

  // 工具/用户/目标系（桌面单独插入）
  const ext = instruction?.extensions || {};
  const toolVal = ext["motion.tool.frameId"] || ext["motion.toolFrameId"] || "active";
  const userVal = ext["motion.user.frameId"] || ext["motion.userFrameId"] || "active";
  let frameVal = ext["motion.target.frame"] || byKey.get("motion.target.frame")?.value || "base";
  if (frameVal === "active_user") frameVal = "user";
  if (["ptp", "line", "arc"].includes(type) || byKey.has("motion.target.pose.x")) {
    out.push({
      key: "motion.tool.frameId",
      label: propertyDisplayLabel("motion.tool.frameId"),
      value: toolVal,
      editable: true,
      kind: "text",
    });
    out.push({
      key: "motion.user.frameId",
      label: propertyDisplayLabel("motion.user.frameId"),
      value: userVal,
      editable: true,
      kind: "text",
    });
    out.push({
      key: "motion.target.frame",
      label: propertyDisplayLabel("motion.target.frame"),
      value: frameVal,
      editable: true,
      kind: "enum",
      options: enumOptionsForKey("motion.target.frame") || ["base", "user"],
    });
  }

  const preferredOrder = [
    "motion.target.pose.x",
    "motion.target.pose.y",
    "motion.target.pose.z",
    "motion.target.euler.rx",
    "motion.target.euler.ry",
    "motion.target.euler.rz",
    "motion.speed",
    "motion.acc",
    "motion.blendRadius",
    "motion.axisConfig.preset",
    "motion.axisConfig.elbow",
    "motion.axisConfig.wrist",
    "motion.axisConfig.arm",
    "motion.axisConfig.turn.j1",
    "motion.axisConfig.turn.j4",
    "motion.axisConfig.turn.j6",
  ];

  const emit = (r: PropRow) => {
    const key = r.key;
    if (isSkippedKey(key)) return;
    if (out.some((x) => x.key === key)) return;
    const opts = enumOptionsForKey(key);
    const value = fmtValue(key, r.value);
    out.push({
      key,
      label: propertyDisplayLabel(key, r.label),
      value,
      editable: !!r.editable,
      kind: opts ? "enum" : "text",
      options: opts || undefined,
    });
  };

  for (const key of preferredOrder) {
    const r = byKey.get(key);
    if (r) emit(r);
  }
  for (const r of apiRows) {
    if (!preferredOrder.includes(r.key)) emit(r);
  }
  return out;
}
