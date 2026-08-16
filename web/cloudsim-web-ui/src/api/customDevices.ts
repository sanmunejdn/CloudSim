import { apiJson, postJson, putJson, type ApiOk } from "./client";

export type CustomDeviceSummary = {
  id: string;
  name: string;
  axisCount: number;
  jointCount: number;
  linkCount: number;
};

export type CustomDeviceMotion = {
  enabled?: boolean;
  displayName?: string;
  jointName?: string;
  motionType?: "Rotate" | "Translate" | "rotate" | "translate" | string;
  lower?: number;
  upper?: number;
  home?: number;
  axis?: number[];
  originMm?: number[];
  motionCenterFrameBackendId?: string;
};

export type CustomDeviceLinkDto = {
  id: string;
  displayName?: string;
  geometryBackendId?: string;
  fixed?: boolean;
  canvasX?: number;
  canvasY?: number;
};

export type CustomDeviceJointDto = {
  id: string;
  parentLinkId: string;
  childLinkId: string;
  motion?: CustomDeviceMotion;
};

export type CustomDeviceDetail = {
  ok: boolean;
  error?: string;
  id?: string;
  name?: string;
  q?: number[];
  namedPoses?: Array<{ id: string; name: string; q: number[] }>;
  poseSignalBindings?: Array<{
    id: string;
    signalName: string;
    poseId: string;
    durationSec?: number;
    enabled?: boolean;
  }>;
  links?: CustomDeviceLinkDto[];
  joints?: CustomDeviceJointDto[];
  signals?: unknown[];
};

export type AssemblyCandidate = { id: string; name: string; className?: string };

export const fetchCustomDevices = () =>
  apiJson<{ ok: boolean; devices?: CustomDeviceSummary[]; error?: string }>("/api/custom-devices");

export const fetchCustomDevice = (id: string) =>
  apiJson<CustomDeviceDetail>(`/api/custom-devices/${encodeURIComponent(id)}`);

export const putCustomDevice = (id: string, body: Record<string, unknown>) =>
  putJson<ApiOk>(`/api/custom-devices/${encodeURIComponent(id)}`, body);

export const applyCustomDeviceQ = (id: string, q: number[]) =>
  postJson<ApiOk>(`/api/custom-devices/${encodeURIComponent(id)}/apply-q`, { q });

export const gotoCustomDevicePose = (id: string, poseId: string) =>
  postJson<ApiOk>(`/api/custom-devices/${encodeURIComponent(id)}/goto-pose`, { poseId });

export const postCustomDeviceAssembly = (body: {
  id?: string;
  name?: string;
  links: CustomDeviceLinkDto[];
  joints: CustomDeviceJointDto[];
}) => postJson<ApiOk & { id?: string }>("/api/custom-devices", body);

export const ensureCustomDevice = (body: { id?: string; name?: string }) =>
  postJson<ApiOk & { id?: string }>("/api/custom-devices/ensure", body);

export const attachCustomDeviceChildren = (deviceId: string, childIds: string[]) =>
  postJson<ApiOk>(`/api/custom-devices/${encodeURIComponent(deviceId)}/attach`, { childIds });

export const fetchAssemblyCandidates = () =>
  apiJson<{ ok: boolean; objects?: AssemblyCandidate[]; error?: string }>(
    "/api/custom-devices/assembly-candidates",
  );

export const exportCustomDeviceUrdf = (deviceId: string, packageParentDir: string) =>
  postJson<ApiOk & { packageDir?: string }>(
    `/api/custom-devices/${encodeURIComponent(deviceId)}/export-urdf`,
    { packageParentDir },
  );

export function defaultRotateMotion(jointId: string): CustomDeviceMotion {
  return {
    enabled: true,
    displayName: "Rotate",
    jointName: jointId,
    motionType: "Rotate",
    lower: -Math.PI,
    upper: Math.PI,
    home: 0,
    axis: [0, 0, 1],
    originMm: [0, 0, 0],
    motionCenterFrameBackendId: "",
  };
}

export function defaultTranslateMotion(jointId: string): CustomDeviceMotion {
  return {
    enabled: true,
    displayName: "Translate",
    jointName: jointId,
    motionType: "Translate",
    lower: 0,
    upper: 1000,
    home: 0,
    axis: [1, 0, 0],
    originMm: [0, 0, 0],
    motionCenterFrameBackendId: "",
  };
}
