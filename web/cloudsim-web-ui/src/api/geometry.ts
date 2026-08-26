import { postJson, type ApiOk } from "./client";

export type GeometryOpResult = ApiOk & {
  error?: string;
  backendId?: string;
  triangleCount?: number;
  avgEdgeLengthMm?: number;
};

export const geometryOp = (body: unknown) => postJson<GeometryOpResult>("/api/geometry/op", body);

export const discretizeGeometry = (body: unknown) =>
  postJson<GeometryOpResult>("/api/geometry/discretize", body);
