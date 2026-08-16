import { apiJson, deleteJson, patchJson, postJson, putJson, type ApiOk } from "./client";
import type { IoSignalRow } from "./ioSignals";

export type IoNetworkOwner = {
  kind: "robot" | "device" | string;
  displayName: string;
  canvasX?: number;
  canvasY?: number;
  signals?: IoSignalRow[];
};

export type IoNetworkWire = {
  id: string;
  fromOwnerId: string;
  fromSignal: string;
  toOwnerId: string;
  toSignal: string;
};

export type IoNetworkPayload = {
  ok: boolean;
  error?: string;
  owners?: Record<string, IoNetworkOwner>;
  wires?: IoNetworkWire[];
  primaryOwnerId?: string;
};

export const fetchIoNetwork = () => apiJson<IoNetworkPayload>("/api/io/network");

export const putOwnerSignals = (ownerId: string, signals: IoSignalRow[]) =>
  putJson<ApiOk>(`/api/io/network/owners/${encodeURIComponent(ownerId)}/signals`, { signals });

export const postIoWire = (wire: Omit<IoNetworkWire, "id"> & { id?: string }) =>
  postJson<ApiOk>("/api/io/network/wires", wire);

export const deleteIoWire = (wireId: string) =>
  deleteJson<ApiOk>(`/api/io/network/wires/${encodeURIComponent(wireId)}`);

export const patchOwnerLayout = (ownerId: string, canvasX: number, canvasY: number) =>
  patchJson<ApiOk>(`/api/io/network/owners/${encodeURIComponent(ownerId)}/layout`, { canvasX, canvasY });

export const patchIoNetworkRuntime = (body: {
  ownerId?: string;
  kind: string;
  port: number;
  value: string;
  forced?: boolean;
}) => postJson<ApiOk>("/api/io/network/runtime", body);

export const resetIoNetworkRuntime = (ownerId?: string) =>
  postJson<ApiOk>("/api/io/network/reset-runtime", ownerId ? { ownerId } : {});
