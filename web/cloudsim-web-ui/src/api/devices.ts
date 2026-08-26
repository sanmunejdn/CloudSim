import { apiJson, postJson, putJson, type ApiOk } from "./client";

export type DevicePackage = {
  type: string;
  brand: string;
  name: string;
  urdfPath: string;
  thumbnailUrl?: string;
};

export const fetchDeviceCatalog = () =>
  apiJson<{
    ok: boolean;
    types?: string[];
    brandsByType?: Record<string, string[]>;
    packages?: DevicePackage[];
    modelsRoot?: string;
  }>("/api/devices/catalog");

export const pointcloudOp = (body: unknown) => postJson<ApiOk & { error?: string }>("/api/pointcloud/op", body);

export type DevicePanelEntry = {
  id?: string;
  name?: string;
  enabled?: boolean;
  address?: string;
  note?: string;
};

export const fetchPlcDevices = () =>
  apiJson<{ ok: boolean; devices?: DevicePanelEntry[]; note?: string; error?: string }>("/api/devices/plc");

export const putPlcDevices = (body: { devices?: DevicePanelEntry[] }) =>
  putJson<ApiOk>("/api/devices/plc", body);

export const fetchCameraDevices = () =>
  apiJson<{ ok: boolean; devices?: DevicePanelEntry[]; note?: string; error?: string }>("/api/devices/camera");

export const putCameraDevices = (body: { devices?: DevicePanelEntry[] }) =>
  putJson<ApiOk>("/api/devices/camera", body);
