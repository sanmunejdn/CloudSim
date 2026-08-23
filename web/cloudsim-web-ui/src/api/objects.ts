import { apiJson, patchJson, postJson, type ApiOk } from "./client";

export type Pose = { positionMm: number[]; eulerDeg: number[] };

export type BackendObject = {
  id: string;
  name: string;
  className: string;
  visible: boolean;
  hasGeometry: boolean;
  geometryKind: number;
  parentIds: string[];
  childIds: string[];
  pose: Pose;
  /** Three.js 列主序 4x4（Gateway 已转换）；机器人连杆以此定位 */
  worldMatrix?: number[];
  color?: { r: number; g: number; b: number; a?: number };
};

export const fetchObjects = () =>
  apiJson<{ objects: BackendObject[]; projectPath?: string }>("/api/objects");

export const fetchObjectDetail = (id: string) =>
  apiJson<{ ok?: boolean; object?: BackendObject; properties?: PropRow[]; error?: string }>(
    `/api/objects/${encodeURIComponent(id)}`,
  );

export type PropRow = { key: string; label?: string; value?: string; editable?: boolean };

export const patchObject = (id: string, body: unknown) =>
  patchJson<ApiOk>(`/api/objects/${encodeURIComponent(id)}`, body);

export const deleteObject = (id: string) =>
  apiJson<ApiOk>(`/api/objects/${encodeURIComponent(id)}`, { method: "DELETE" });

export const postSelection = (backendId: string) =>
  postJson<ApiOk>("/api/selection", { backendId });

export const importObject = (path: string, isPointCloud = false) =>
  postJson<{ ok: boolean; id?: string; error?: string }>("/api/objects/import", { path, isPointCloud });

export async function fetchMeshSoup(id: string): Promise<Float32Array | null> {
  const r = await fetch(`/api/mesh/${encodeURIComponent(id)}`);
  if (!r.ok) return null;
  return new Float32Array(await r.arrayBuffer());
}

export const createCoordinateFrame = (body: unknown) =>
  postJson<{ ok: boolean; id?: string; error?: string }>("/api/objects/coordinate-frame", body);

export const attachObjectChild = (parentId: string, childId: string) =>
  postJson<ApiOk>("/api/objects/attach", { parentId, childId });

export const listCoordinateFrames = () =>
  apiJson<{ ok: boolean; frames?: unknown[] }>("/api/objects/coordinate-frames");
