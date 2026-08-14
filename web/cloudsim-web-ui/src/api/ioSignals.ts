import { apiJson, postJson, putJson, type ApiOk } from "./client";

export type IoSignalKind = "DI" | "DO" | "AI" | "AO";

export type IoSignalRow = {
  id: string;
  name: string;
  kind: IoSignalKind | string;
  port: number;
  defaultBool?: boolean;
  defaultAnalog?: number;
  description?: string;
  simForceable?: boolean;
  value?: string;
  forced?: boolean;
};

export const fetchIoSignals = () =>
  apiJson<{ ok: boolean; signals?: IoSignalRow[]; error?: string }>("/api/io/signals");

export const putIoSignals = (signals: IoSignalRow[]) =>
  putJson<ApiOk>("/api/io/signals", { signals });

export const fetchIoSignalNames = (kind?: string) => {
  const q = kind ? `?kind=${encodeURIComponent(kind)}` : "";
  return apiJson<{ ok: boolean; names?: string[]; kind?: string }>(`/api/io/signals/names${q}`);
};

export const patchIoSignalRuntime = (body: {
  kind: string;
  port: number;
  value: string;
  forced?: boolean;
}) => postJson<ApiOk>("/api/io/signals/runtime", body);

export const resetIoSignalRuntime = () => postJson<ApiOk>("/api/io/signals/reset-runtime", {});
