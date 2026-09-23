/// @file ioProgramSteps.ts
/// @brief 程序 Run 时 IO 步解析（与桌面信号网络同源）

import { fetchIoNetwork } from "../api/ioNetwork";
import type { Instruction } from "../api";

export type IoStepFields = Instruction & {
  port?: number;
  value?: boolean | number | string;
  signalName?: string;
  ioPort?: number;
  digitalValue?: boolean | number | string;
  analogValue?: number;
  ioBoolValue?: boolean;
  ioAnalogValue?: number;
};

export async function resolveIoPort(
  step: IoStepFields,
  kind: "DO" | "AO",
): Promise<{ ownerId: string; port: number }> {
  const net = await fetchIoNetwork();
  const ownerId = net.primaryOwnerId || Object.keys(net.owners || {})[0] || "";
  const signals = (ownerId && net.owners?.[ownerId]?.signals) || [];
  const name = String(step.signalName || "").trim();
  if (name) {
    const hit = signals.find((s) => s.kind === kind && s.name === name);
    if (hit) return { ownerId, port: Number(hit.port) || 0 };
  }
  const raw = step.port ?? step.ioPort;
  const port = Number(raw);
  return { ownerId, port: Number.isFinite(port) ? port : 0 };
}

export function doValueText(step: IoStepFields): string {
  const raw = step.value ?? step.digitalValue ?? step.ioBoolValue;
  if (raw === false || raw === 0 || raw === "0" || raw === "false") return "0";
  if (raw === true || raw === 1 || raw === "1" || raw === "true") return "1";
  // Host 默认写 true；缺省按拉高处理
  return "1";
}

export async function waitIoCondition(step: Instruction, abort: () => boolean): Promise<boolean> {
  const cond = step.condition;
  if (!cond || cond.kind !== "io") return true;
  const port = Number(cond.port ?? cond.ioPort);
  const name = cond.signalName || "";
  for (let i = 0; i < 600; i++) {
    if (abort()) return false;
    const net = await fetchIoNetwork();
    const oid = net.primaryOwnerId || "";
    const signals = (oid && net.owners?.[oid]?.signals) || [];
    const hit = signals.find(
      (s) => s.kind === "DI" && ((name && s.name === name) || (Number.isFinite(port) && s.port === port)),
    );
    if (hit && hit.value === "1") return true;
    await new Promise((r) => setTimeout(r, 100));
  }
  return false;
}
