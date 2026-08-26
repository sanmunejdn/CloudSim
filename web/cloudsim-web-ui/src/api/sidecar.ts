import { apiJson } from "./client";

export async function fetchSidecar<T>(key: string): Promise<T> {
  return apiJson<T>(`/api/sidecar/${encodeURIComponent(key)}`);
}

export async function putSidecar<T>(key: string, body: unknown): Promise<T> {
  return apiJson<T>(`/api/sidecar/${encodeURIComponent(key)}`, {
    method: "PUT",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
}

export type AnnotationRow = { id?: string; title?: string; note?: string };

export const fetchAnnotations = () => fetchSidecar<AnnotationRow[]>("annotations");

export const putAnnotations = (rows: AnnotationRow[]) => putSidecar<{ ok?: boolean; error?: string }>("annotations", rows);
