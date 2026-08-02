export type Health = { ok: boolean; role: string; pid: number; port: number };

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
};

async function json<T>(url: string, init?: RequestInit): Promise<T> {
  const r = await fetch(url, init);
  return (await r.json()) as T;
}

export const fetchHealth = () => json<Health>("/api/health");
export const fetchObjects = () => json<{ objects: BackendObject[]; projectPath?: string }>("/api/objects");
export const openProject = (path: string) =>
  json<{ ok: boolean; error?: string; objectCount?: number }>("/api/project/open", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ path }),
  });
export const newProject = () => json<{ ok: boolean }>("/api/project/new", { method: "POST" });
export const saveProject = (path: string) =>
  json<{ ok: boolean; error?: string; path?: string }>("/api/project/save", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ path }),
  });
export const patchObject = (id: string, body: unknown) =>
  json<{ ok: boolean; error?: string }>(`/api/objects/${encodeURIComponent(id)}`, {
    method: "PATCH",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
export const postSelection = (backendId: string) =>
  json<{ ok: boolean }>("/api/selection", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ backendId }),
  });
export const importObject = (path: string, isPointCloud = false) =>
  json<{ ok: boolean; id?: string; error?: string }>("/api/objects/import", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ path, isPointCloud }),
  });
export const fetchModes = () =>
  json<{ modes: { id: string; title: string }[]; active: string }>("/api/modes");
export const setWorkspaceMode = (mode: string) =>
  json<{ ok: boolean }>("/api/sidecar/workspaceMode", {
    method: "PUT",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ mode }),
  });

export async function fetchMeshSoup(id: string): Promise<Float32Array | null> {
  const r = await fetch(`/api/mesh/${encodeURIComponent(id)}`);
  if (!r.ok) return null;
  return new Float32Array(await r.arrayBuffer());
}
