import { apiJson, postJson, putJson, type ApiOk } from "./client";

export type Rigid = { positionMm: number[]; eulerDeg: number[] };

export type ToolFrame = {
  id: string;
  name: string;
  T_flange_tool?: Rigid;
  flangeLinkName?: string;
  showInScene?: boolean;
};

export type UserFrame = {
  id: string;
  name: string;
  T_base_user?: Rigid;
  showInScene?: boolean;
};

export type FrameSet = {
  toolFrames: ToolFrame[];
  userFrames: UserFrame[];
  activeToolFrameId?: string;
  activeUserFrameId?: string;
  showToolFrame?: boolean;
  showUserFrames?: boolean;
  flangeLinkName?: string;
};

export const fetchFrames = (sceneRootBackendId: string) =>
  apiJson<{ ok: boolean; frames?: FrameSet; linkNames?: string[]; error?: string }>(
    `/api/robot/frames?sceneRootBackendId=${encodeURIComponent(sceneRootBackendId)}`,
  );

export const putFrames = (sceneRootBackendId: string, frames: FrameSet) =>
  putJson<ApiOk>("/api/robot/frames", { sceneRootBackendId, frames });

export const mutateFrames = (sceneRootBackendId: string, action: string, id?: string) =>
  postJson<ApiOk>("/api/robot/frames/mutate", { sceneRootBackendId, action, id });

export const captureTool = (sceneRootBackendId: string) =>
  postJson<ApiOk>("/api/robot/frames/capture-tool", { sceneRootBackendId });

export const resetTool = (sceneRootBackendId: string) =>
  postJson<ApiOk>("/api/robot/frames/reset-tool", { sceneRootBackendId });

export const captureUser = (sceneRootBackendId: string) =>
  postJson<ApiOk>("/api/robot/frames/capture-user", { sceneRootBackendId });

export type FrameOverlayItem = {
  id?: string;
  name?: string;
  active?: boolean;
  positionMm?: number[];
  eulerDeg?: number[];
  worldMatrix?: number[];
  showInScene?: boolean;
};

export const fetchFrameOverlays = (sceneRootBackendId: string) =>
  apiJson<{ ok: boolean; tools?: FrameOverlayItem[]; users?: FrameOverlayItem[]; error?: string }>(
    `/api/robot/frames/overlays?sceneRootBackendId=${encodeURIComponent(sceneRootBackendId)}`,
  );
