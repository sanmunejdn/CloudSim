import { apiJson, postJson, putJson, type ApiOk } from "./client";

export const trajSession = () => apiJson<Record<string, unknown>>("/api/trajectory/session");
export const trajBeginEdit = () => postJson<ApiOk>("/api/trajectory/begin-edit");
export const trajCancelEdit = () => postJson<ApiOk>("/api/trajectory/cancel-edit");
export const trajBind = (pathPlanId: string, sceneRootBackendId: string) =>
  postJson<ApiOk>("/api/trajectory/bind", { pathPlanId, sceneRootBackendId });
export const trajPathPlans = (sceneRootBackendId: string) =>
  apiJson<{ pathPlans?: { id: string; name?: string; phase?: string; bound?: boolean }[] }>(
    `/api/trajectory/path-plans?sceneRootBackendId=${encodeURIComponent(sceneRootBackendId)}`,
  );
export const trajDiscretize = (body: unknown) => postJson<ApiOk & { error?: string }>("/api/trajectory/discretize", body);
export const trajMeshSpec = (spec: unknown) => postJson<ApiOk & { error?: string }>("/api/trajectory/mesh-spec", spec);
export const trajPreview = () => postJson<Record<string, unknown>>("/api/trajectory/preview");
export const trajPreviewRaw = () => postJson<Record<string, unknown>>("/api/trajectory/preview-raw");
export const trajEmit = () => postJson<ApiOk & { error?: string }>("/api/trajectory/emit");
export const trajApply = () => postJson<ApiOk & { error?: string }>("/api/trajectory/apply");
export const trajRecipe = (recipe: string) =>
  postJson<ApiOk & { error?: string }>("/api/trajectory/recipe", { recipe });
export const trajReset = () => postJson<ApiOk>("/api/trajectory/reset");
export const trajUndo = () => postJson<ApiOk>("/api/trajectory/undo");
export const trajRedo = () => postJson<ApiOk>("/api/trajectory/redo");
export const trajGetPipeline = () => apiJson<unknown>("/api/trajectory/pipeline");
export const trajPutPipeline = (pipeline: unknown[]) => putJson<ApiOk>("/api/trajectory/pipeline", pipeline);
export type OpPaletteEntry = { kind: string; displayNameZh?: string };

/** Host 返回字段为 ops（非 entries） */
export const trajOpPalette = () =>
  apiJson<{ ok?: boolean; ops?: OpPaletteEntry[]; entries?: OpPaletteEntry[]; error?: string }>(
    "/api/trajectory/op-palette",
  );
export const trajOpSchema = (kind: string, opIndex = 0) =>
  apiJson<Record<string, unknown>>(`/api/trajectory/op-schema?kind=${encodeURIComponent(kind)}&opIndex=${opIndex}`);
export const trajFeatureCatalog = (workpiece: string) =>
  apiJson<Record<string, unknown>>(`/api/trajectory/feature-catalog?workpiece=${encodeURIComponent(workpiece)}`);
export const trajFeatureSchema = (strategyId?: string) =>
  apiJson<Record<string, unknown>>(
    strategyId
      ? `/api/trajectory/feature-schema?strategyId=${encodeURIComponent(strategyId)}`
      : "/api/trajectory/feature-schema",
  );

export const pickHover = (body: unknown) => postJson<Record<string, unknown>>("/api/pick/hover", body);
export const pickMeshElement = (body: unknown) => postJson<Record<string, unknown>>("/api/pick/mesh-element", body);
