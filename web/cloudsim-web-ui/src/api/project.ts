import { apiJson, postJson, type ApiOk } from "./client";

export type Health = { ok: boolean; role: string; pid: number; port: number };

export const fetchHealth = () => apiJson<Health>("/api/health");

export const fetchModes = () =>
  apiJson<{ modes: { id: string; title: string }[]; active: string }>("/api/modes");

export const setWorkspaceMode = (mode: string) => putMode(mode);

function putMode(mode: string) {
  return apiJson<ApiOk>("/api/sidecar/workspaceMode", {
    method: "PUT",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ mode }),
  });
}

/** 与 Gateway nativeDialog purpose 对齐；缺省 project 会落到 .pcp 工程过滤器 */
export type DialogOpenOpts = {
  title?: string;
  purpose?:
    | "project"
    | "import"
    | "model"
    | "file"
    | "pointcloud"
    | "urdf"
    | "directory"
    | "folder"
    | "saveProject"
    | "saveFile"
    | "save";
  filter?: string;
  /** @deprecated 使用 filter */
  filters?: string;
  directory?: boolean;
};

export type DialogOpenResult = {
  ok: boolean;
  path?: string;
  /** 多选时返回；缺省时用 path 兜底成单元素 */
  paths?: string[];
  error?: string;
  cancelled?: boolean;
  folder?: string;
};

export const dialogOpen = (opts: DialogOpenOpts = {}) => {
  const purpose = opts.directory ? "directory" : opts.purpose ?? "project";
  const filter = opts.filter ?? opts.filters;
  return postJson<DialogOpenResult>("/api/dialog/open", {
    title: opts.title,
    purpose,
    filter,
  });
};

/** 统一取出对话框路径列表（兼容仅返回 path） */
export function dialogPaths(d: DialogOpenResult): string[] {
  if (Array.isArray(d.paths) && d.paths.length) {
    return d.paths.map(String).filter(Boolean);
  }
  if (d.path) return [d.path];
  return [];
}

export const newProject = () => postJson<ApiOk>("/api/project/new");
export const openProject = (path: string) =>
  postJson<{ ok: boolean; error?: string; objectCount?: number }>("/api/project/open", { path });
export const saveProject = (path: string) =>
  postJson<{ ok: boolean; error?: string; path?: string }>("/api/project/save", { path });
