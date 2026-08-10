import { apiJson, postJson, putJson, patchJson, type ApiOk } from "./client";

export type Vec3 = { x: number; y: number; z: number };

export type Instruction = {
  id: string;
  type: string;
  name?: string;
  pose?: Vec3;
  eulerDeg?: Vec3;
  viaPose?: Vec3;
  viaEulerDeg?: Vec3;
  speed?: number;
  accel?: number;
  blendRadius?: number;
  durationSec?: number;
  pointIndex?: number;
  phase?: string;
  sourceFeature?: unknown;
  extensions?: Record<string, string>;
  then?: Instruction[];
  else?: Instruction[];
  body?: Instruction[];
  condition?: { kind?: string; ioPort?: number; ioEquals?: boolean; compareLeft?: string; compareOp?: string; compareRight?: number };
  ioPort?: number;
  ioBoolValue?: boolean;
  ioAnalogValue?: number;
};

export type InstructionGroup = {
  id: string;
  name?: string;
  role?: string;
  pathPlanInstructionId?: string;
  memberIds?: string[];
};

export type RobotProgram = {
  id: string;
  name?: string;
  isMain?: boolean;
  instructions: Instruction[];
  groups: InstructionGroup[];
};

export type ProgramCatalog = {
  sceneBackendId: string;
  activeProgramId?: string;
  programs: RobotProgram[];
};

export const fetchPrograms = () =>
  apiJson<{ ok?: boolean; programs?: ProgramCatalog[] } | ProgramCatalog[]>("/api/robot/programs");

export const putPrograms = (catalogs: ProgramCatalog[]) =>
  putJson<ApiOk>("/api/robot/programs", { programs: catalogs });

export const fetchInstructionProperties = (id: string) =>
  apiJson<{ ok: boolean; properties?: { key: string; label?: string; value?: string; editable?: boolean }[]; error?: string }>(
    `/api/robot/instructions/${encodeURIComponent(id)}/properties`,
  );

export const patchInstruction = (id: string, key: string, value: string) =>
  patchJson<ApiOk>(`/api/robot/instructions/${encodeURIComponent(id)}`, { key, value });

export type RobotInstanceInfo = {
  sceneRootBackendId: string;
  /** Gateway 字段；旧前端曾误读为 name */
  label?: string;
  name?: string;
  urdfPath?: string;
  jointCount?: number;
};

export const fetchRobotInstances = () =>
  apiJson<{ ok: boolean; instances?: RobotInstanceInfo[] }>("/api/robot/instances");

/** 与桌面/设备库一致：包名 → URDF 主名 → RobotURDF_ 后缀 → label */
export function robotModelNameFromInstance(inst: RobotInstanceInfo): string {
  const urdf = String(inst.urdfPath || "").replace(/\\/g, "/");
  if (urdf) {
    const parts = urdf.split("/").filter(Boolean);
    const urdfDir = parts.findIndex((p) => p.toLowerCase() === "urdf");
    if (urdfDir > 0) return parts[urdfDir - 1];
    const file = parts[parts.length - 1] || "";
    const base = file.replace(/\.urdf$/i, "");
    if (base) return base;
  }
  const rootId = String(inst.sceneRootBackendId || "");
  if (rootId.startsWith("RobotURDF_")) {
    const rest = rootId.slice("RobotURDF_".length);
    if (rest) return rest;
  }
  const label = String(inst.label || inst.name || "").trim();
  if (label && label !== "base_link" && label !== "base") return label;
  return label;
}

export const resolveRobot = (backendId: string) =>
  apiJson<{
    ok: boolean;
    sceneRootBackendId?: string;
    flangeBackendId?: string;
    anchorBackendId?: string;
  }>(`/api/robot/resolve?backendId=${encodeURIComponent(backendId)}`);

export const importUrdf = (urdfPath: string) => {
  const path = String(urdfPath || "").trim();
  if (!path) {
    return Promise.resolve({ ok: false as const, error: "缺少 urdfPath" });
  }
  return postJson<{ ok: boolean; sceneRootBackendId?: string; error?: string }>("/api/robot/urdf/import", {
    urdfPath: path,
  });
};

export const fetchJoints = (sceneRootBackendId: string) =>
  apiJson<{
    ok: boolean;
    joints?: { name: string; angleRad: number; lowerRad?: number; upperRad?: number }[];
  }>(`/api/robot/joints?sceneRootBackendId=${encodeURIComponent(sceneRootBackendId)}`);

export const postJoints = (sceneRootBackendId: string, jointAnglesRad: number[]) =>
  postJson<ApiOk>("/api/robot/joints", { sceneRootBackendId, jointAnglesRad });

export const planInstruction = (body: unknown) =>
  postJson<{ ok: boolean; jointTargetsRad?: number[]; error?: string; frames?: number[][] }>("/api/robot/plan", body);

export const runProgram = () => postJson<ApiOk>("/api/robot/run");
export const stopProgram = () => postJson<ApiOk>("/api/robot/stop");

export const placeRobot = (body: unknown) => postJson<ApiOk>("/api/robot/place", body);
export const tcpIk = (body: unknown) =>
  postJson<{ ok: boolean; jointAnglesRad?: number[]; incomplete?: boolean; error?: string }>("/api/robot/tcp-ik", body);
export const tcpPose = (sceneRootBackendId: string) =>
  apiJson<{
    ok: boolean;
    positionMm?: number[];
    eulerDeg?: number[];
    jointRadCsv?: string;
    worldMatrix?: number[];
    error?: string;
  }>(`/api/robot/tcp-pose?sceneRootBackendId=${encodeURIComponent(sceneRootBackendId)}`);

export const createPathPlan = (sceneRootBackendId: string) =>
  postJson<{ ok: boolean; pathPlanId?: string; error?: string }>("/api/robot/path-plan", { sceneRootBackendId });
