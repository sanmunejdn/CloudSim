import { fetchJoints, planInstruction, postJoints, type Instruction } from "../api";

export function reshapeJointFrames(flat: number[] | undefined, dof: number): number[][] {
  const arr = (flat || []).map(Number).filter((n) => !Number.isNaN(n));
  if (!dof || dof < 1 || !arr.length) return [];
  if (arr.length === dof) return [arr];
  if (arr.length % dof === 0) {
    const frames: number[][] = [];
    for (let i = 0; i < arr.length; i += dof) frames.push(arr.slice(i, i + dof));
    return frames;
  }
  return [arr.slice(0, dof)];
}

export function buildPlanBody(rootId: string, step: Instruction, jointRadCsvFallback = "") {
  const type = String(step.type || "").toLowerCase();
  const jointRadCsv =
    (step.extensions && step.extensions["context.currentJointRadCsv"]) || jointRadCsvFallback || "";
  const body: Record<string, unknown> = {
    sceneRootBackendId: rootId,
    instructionType: type,
    jointRadCsv,
    targetPose: {
      positionMm: [step.pose?.x || 0, step.pose?.y || 0, step.pose?.z || 0],
      eulerDeg: [step.eulerDeg?.x || 0, step.eulerDeg?.y || 0, step.eulerDeg?.z || 0],
    },
  };
  if (type === "arc") {
    body.extensions = {
      viaPose: step.viaPose || { x: 0, y: 0, z: 0 },
      viaEulerDeg: step.viaEulerDeg || { x: 0, y: 0, z: 0 },
    };
  }
  return body;
}

export async function planStepFrames(
  rootId: string,
  step: Instruction,
): Promise<{ ok: true; frames: number[][] } | { ok: false; error: string }> {
  const type = String(step.type || "").toLowerCase();
  if (!["ptp", "line", "arc"].includes(type)) return { ok: false, error: "非运动指令" };
  let dof = 6;
  let jointCsv = "";
  try {
    const j = await fetchJoints(rootId);
    const joints = j.joints || [];
    if (joints.length) {
      dof = joints.length;
      jointCsv = joints.map((x) => x.angleRad.toFixed(6)).join(",");
    }
  } catch {
    /* 用默认 DOF */
  }
  const r = await planInstruction(buildPlanBody(rootId, step, jointCsv));
  if (!r.ok) return { ok: false, error: r.error || "规划失败" };
  const frames = reshapeJointFrames(r.jointTargetsRad, dof);
  if (!frames.length) return { ok: false, error: "规划无关节结果" };
  return { ok: true, frames };
}

export function estimateStepDurationSec(step: Instruction, frameCount: number) {
  const speed = Number(step.speed) || 100;
  if (frameCount > 1) return Math.max(0.35, frameCount * 0.04 * (100 / speed));
  return Math.max(0.4, 1.2 * (100 / speed));
}

function sleepMs(ms: number) {
  return new Promise((r) => setTimeout(r, ms));
}

export async function playJointFrames(
  rootId: string,
  frames: number[][],
  durationSec: number,
  simRate: number,
  shouldAbort: () => boolean,
): Promise<boolean> {
  const rate = Math.max(0.1, simRate || 1);
  const dur = Math.max(0.15, durationSec / rate);
  if (frames.length === 1) {
    let start = frames[0].map(() => 0);
    try {
      const j = await fetchJoints(rootId);
      if (j.joints?.length) start = j.joints.map((x) => x.angleRad);
    } catch {
      /* 从零插值 */
    }
    const end = frames[0];
    const n = Math.max(6, Math.ceil(dur * 24));
    for (let i = 1; i <= n; ++i) {
      if (shouldAbort()) return false;
      const u = i / n;
      const joints = end.map((t, k) => (start[k] ?? 0) + (t - (start[k] ?? 0)) * u);
      const ok = await postJoints(rootId, joints);
      if (!ok.ok) return false;
      await sleepMs((dur * 1000) / n);
    }
    return true;
  }
  const dt = dur / Math.max(1, frames.length - 1);
  for (const f of frames) {
    if (shouldAbort()) return false;
    const ok = await postJoints(rootId, f);
    if (!ok.ok) return false;
    await sleepMs(dt * 1000);
  }
  return true;
}
