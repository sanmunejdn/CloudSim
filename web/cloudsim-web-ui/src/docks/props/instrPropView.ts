import type { Instruction, PropRow } from "../../api";
import { enumOptionsForKey, ioSignalKindForInstrProp, propertyDisplayLabel } from "./propLabels";

export type InstrPropViewRow = PropRow & { kind?: "text" | "enum"; options?: string[] };

export type SignalNameOptions = { di: string[]; do: string[]; ao: string[] };

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

function withEmpty(names: string[]) {
  return names[0] === "" ? names : ["", ...names.filter(Boolean)];
}

function signalOptionsForKey(key: string, instrType: string | undefined, signalOpts?: SignalNameOptions) {
  if (!signalOpts) return null;
  const kind = ioSignalKindForInstrProp(key, instrType);
  if (kind === "DI") return withEmpty(signalOpts.di);
  if (kind === "AO") return withEmpty(signalOpts.ao);
  if (kind === "DO") return withEmpty(signalOpts.do);
  return null;
}

/** 对齐桌面 InstructionPropertyPanel 的字段集合与中文标签 */
export function buildInstrPropView(
  apiRows: PropRow[],
  instructionId: string,
  instruction?: Instruction | null,
  signalOpts?: SignalNameOptions,
): InstrPropViewRow[] {
  const byKey = new Map(apiRows.map((r) => [r.key, r]));
  const instrType = String(instruction?.type || "");

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

  const type = instrType.toLowerCase();
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
    "logic.condition.kind",
    "logic.condition.signalName",
    "logic.condition.port",
    "logic.condition.equals",
    "logic.io.signalName",
    "logic.io.port",
    "logic.io.boolValue",
    "logic.io.analogValue",
  ];

  const emit = (r: PropRow) => {
    const key = r.key;
    if (isSkippedKey(key)) return;
    if (out.some((x) => x.key === key)) return;
    let opts = signalOptionsForKey(key, instrType, signalOpts) || enumOptionsForKey(key);
    if (key === "logic.condition.kind" && type === "wait") {
      opts = ["always", "io"];
    }
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
