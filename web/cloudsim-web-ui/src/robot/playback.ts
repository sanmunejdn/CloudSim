import {
  fetchJoints,
  fetchPlaybackStatus,
  planInstruction,
  postJoints,
  runProgram,
  stopProgram,
  type Instruction,
  type RunProgramBody,
} from "../api";
import { eventHub } from "../sse/EventHub";

export type PlaybackFrame = {
  ok?: boolean;
  running?: boolean;
  instructionId?: string;
  jointAnglesRad?: number[];
  progress?: number;
  tickResult?: number;
  abortSummary?: string;
};

export async function probeServerPlayback(): Promise<boolean> {
  try {
    const s = await fetchPlaybackStatus();
    return !!s.ok;
  } catch {
    return false;
  }
}

function sleepMs(ms: number) {
  return new Promise((r) => setTimeout(r, ms));
}

/** 订阅 SSE + 轮询 status，直到服务端回放结束或 abort */
export async function waitServerPlayback(
  shouldAbort: () => boolean,
  onFrame?: (frame: PlaybackFrame) => void,
): Promise<{ ok: boolean; abortSummary?: string }> {
  let done = false;
  let unsub = () => {};
  let poll = 0;

  const cleanup = () => {
    unsub();
    if (poll) window.clearInterval(poll);
  };

  return new Promise((resolve) => {
    const end = (ok: boolean, abortSummary?: string) => {
      if (done) return;
      done = true;
      cleanup();
      resolve({ ok, abortSummary });
    };

    unsub = eventHub.on("PlaybackFrame", (data) => {
      try {
        const frame = JSON.parse(data) as PlaybackFrame;
        onFrame?.(frame);
        if (frame.running === false) end(true, frame.abortSummary);
      } catch {
        /* 非 JSON */
      }
    });

    poll = window.setInterval(() => {
      void (async () => {
        if (shouldAbort()) {
          try {
            await stopProgram();
          } catch {
            /* Host 可选 */
          }
          end(false);
          return;
        }
        try {
          const st = await fetchPlaybackStatus();
          if (st.ok && !st.running) end(true);
        } catch {
          /* 轮询兜底 */
        }
      })();
    }, 400);
  });
}

export async function startServerPlayback(
  body: RunProgramBody,
  shouldAbort: () => boolean,
  onFrame?: (frame: PlaybackFrame) => void,
): Promise<{ ok: boolean; error?: string; abortSummary?: string }> {
  const r = await runProgram(body);
  if (!r.ok) return { ok: false, error: r.error || "服务端运行失败" };
  const waited = await waitServerPlayback(shouldAbort, onFrame);
  return { ok: waited.ok, abortSummary: waited.abortSummary };
}

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

function taughtJointCsvFromStep(step: Instruction): string {
  return (step.extensions && step.extensions["context.currentJointRadCsv"]) || "";
}

function parseJointCsv(csv: string): number[] {
  return csv
    .split(",")
    .map((s) => Number(s.trim()))
    .filter((n) => !Number.isNaN(n));
}

export function buildPlanBody(rootId: string, step: Instruction, jointRadCsvFallback = "") {
  const type = String(step.type || "").toLowerCase();
  const taughtJointRadCsv = taughtJointCsvFromStep(step);
  // seed=当前链；taught=示教落点（勿再把示教关节塞进 seed，否则会当成「已在目标」去解笛卡尔 IK）
  const body: Record<string, unknown> = {
    sceneRootBackendId: rootId,
    instructionType: type,
    jointRadCsv: jointRadCsvFallback || "",
    targetPose: {
      positionMm: [step.pose?.x || 0, step.pose?.y || 0, step.pose?.z || 0],
      eulerDeg: [step.eulerDeg?.x || 0, step.eulerDeg?.y || 0, step.eulerDeg?.z || 0],
    },
  };
  if (taughtJointRadCsv) body.taughtJointRadCsv = taughtJointRadCsv;
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
  // 对齐桌面：PTP 优先示教关节，避免罗盘位姿与钳制关节不一致时再解 IK 超限
  if (type === "ptp") {
    const taught = parseJointCsv(taughtJointCsvFromStep(step));
    if (taught.length) {
      if (dof < 1) dof = taught.length;
      return { ok: true, frames: [taught] };
    }
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
