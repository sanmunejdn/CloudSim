import { apiJson, postJson, putJson, type ApiOk } from "./client";

export type GeomodelFeature = {
  id: string;
  name?: string;
  kind?: string;
  suppressed?: boolean;
  visible?: boolean;
  lengthMm?: number;
  draftAngleDeg?: number;
  endCondition?: string;
  radiusMm?: number;
  chamferDistMm?: number;
  revolveAngleDeg?: number;
  shellThicknessMm?: number;
  sketchRefId?: string;
};

export type GeomodelBody = {
  backendId?: string;
  name?: string;
  className?: string;
  hasGeometry?: boolean;
  featureCount?: number;
  features?: GeomodelFeature[];
};

export type GeomodelSummary = ApiOk & {
  bodies?: GeomodelBody[];
  count?: number;
  undoCount?: number;
  redoCount?: number;
  error?: string;
};

export type GeomodelOpResult = ApiOk & {
  backendId?: string;
  name?: string;
  featureCount?: number;
  hasGeometry?: boolean;
  undoCount?: number;
  redoCount?: number;
  path?: string;
  removed?: boolean;
  error?: string;
};

export const fetchGeomodelSummary = () => apiJson<GeomodelSummary>("/api/geomodeling/summary");

export const fetchGeomodelHistory = (backendId: string) =>
  apiJson<ApiOk & { history?: Record<string, unknown>; error?: string }>(
    `/api/geomodeling/history?backendId=${encodeURIComponent(backendId)}`,
  );

export const putGeomodelHistory = (backendId: string, history: unknown) =>
  putJson<GeomodelOpResult>("/api/geomodeling/history", { backendId, history });

export const geomodelOp = (body: unknown) => postJson<GeomodelOpResult>("/api/geomodeling/op", body);
