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
  condition?: {
    kind?: string;
    /** @deprecated 仅兼容旧工程；新写入用 port */
    ioPort?: number;
    port?: number;
    ioEquals?: boolean;
    signalName?: string;
    compareLeft?: string;
    compareOp?: string;
    compareRight?: number;
  };
  /** 与 Host RobotInstructionFactory 一致 */
  port?: number;
  value?: boolean | number | string;
  signalName?: string;
  /** @deprecated 仅兼容读旧前端字段 */
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

export type RunProgramBody = {
  sceneRootBackendId?: string;
  programId?: string;
  playbackRate?: number;
};

export type PlaybackStatus = {
  ok: boolean;
  running?: boolean;
  sceneRootBackendId?: string;
  jointAnglesRad?: number[];
  error?: string;
};

export type ExportProgramBody = {
  sceneRootBackendId?: string;
  programId?: string;
  brand?: string;
  outputPath?: string;
  scriptStem?: string;
};

export const runProgram = (body?: RunProgramBody) =>
  postJson<ApiOk & { status?: string; error?: string }>("/api/robot/run", body ?? {});

export const stopProgram = () => postJson<ApiOk & { status?: string }>("/api/robot/stop");

export const fetchPlaybackStatus = () => apiJson<PlaybackStatus>("/api/robot/playback/status");

export const exportProgram = (body: ExportProgramBody) =>
  postJson<{ ok: boolean; path?: string; canonicalPath?: string; error?: string }>("/api/robot/export", body);

/** 与桌面 BrandProgramExportDialog::allBrands 对齐 */
export const ROBOT_EXPORT_BRANDS = [
  { id: "abb", label: "ABB", filter: "ABB RAPID (*.MOD)" },
  { id: "air", label: "AIR", filter: "AIR ARL (*.arl)" },
  { id: "fanuc", label: "FANUC", filter: "FANUC LS (*.LS)" },
  { id: "inovance", label: "INOVANCE", filter: "INOVANCE PRO (*.pro)" },
  { id: "lineheating", label: "LineHeating", filter: "LineHeating LS (*.LS)" },
  { id: "rokae", label: "ROKAE", filter: "ROKAE MOD (*.mod)" },
] as const;

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

export type CollisionSettings = {
  enabled?: boolean;
  securityMarginMm?: number;
  whiteListBackendIds?: string[];
  blackListBackendIds?: string[];
};

export const fetchCollisionSettings = () =>
  apiJson<CollisionSettings & { ok: boolean; error?: string; stub?: boolean }>("/api/robot/collision-settings");

export const putCollisionSettings = (body: CollisionSettings) =>
  putJson<ApiOk>("/api/robot/collision-settings", body);

/** 规划路由未上线时由面板回退为仅保存设置 */
export async function postCollisionPlan(
  body: CollisionSettings,
): Promise<{ ok: boolean; error?: string; routeMissing?: boolean }> {
  const r = await fetch("/api/robot/collision/plan", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (r.status === 404) return { ok: false, routeMissing: true };
  return (await r.json()) as { ok: boolean; error?: string };
}

export const switchRobotProgram = (programId: string, sceneRootBackendId?: string) =>
  postJson<{ ok: boolean; programId?: string; programName?: string; error?: string }>(
    "/api/robot/programs/switch",
    { programId, ...(sceneRootBackendId ? { sceneRootBackendId } : {}) },
  );

export const undoProgramEdit = (body: { sceneRootBackendId?: string } = {}) =>
  postJson<{ ok: boolean; canUndo?: boolean; error?: string; stub?: boolean }>(
    "/api/robot/program-edit/undo",
    body,
  );

export const redoProgramEdit = (body: { sceneRootBackendId?: string } = {}) =>
  postJson<{ ok: boolean; canRedo?: boolean; error?: string; stub?: boolean }>(
    "/api/robot/program-edit/redo",
    body,
  );
