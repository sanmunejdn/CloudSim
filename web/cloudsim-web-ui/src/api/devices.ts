import { apiJson, postJson, type ApiOk } from "./client";

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

export const aiChat = (prompt: string) =>
  postJson<{ ok: boolean; reply?: string; error?: string }>("/api/ai/chat", { prompt });

export const geometryOp = (body: unknown) => postJson<ApiOk & { error?: string }>("/api/geometry/op", body);
export const pointcloudOp = (body: unknown) => postJson<ApiOk & { error?: string }>("/api/pointcloud/op", body);
