import { postJson, type ApiOk } from "./client";

export type MateFaceRef = {
  backendId: string;
  faceIndex: number;
  pickWorldMm: number[];
};

export type MateKind =
  | "coincident"
  | "parallel"
  | "perpendicular"
  | "tangent"
  | "concentric"
  | "lock"
  | "distance"
  | "angle";

export type MateApplyBody = {
  action?: "apply" | "restore";
  grounded?: MateFaceRef;
  moving?: MateFaceRef;
  kind?: MateKind;
  alignment?: "aligned" | "antiAligned";
  distanceMm?: number;
  angleDeg?: number;
  commit?: boolean;
  movingWorldSnapshot?: number[];
  backendId?: string;
};

export type MateApplyResult = ApiOk & {
  movingWorldSnapshot?: number[];
  movingBackendId?: string;
  commit?: boolean;
};

export const applyAssemblyMate = (body: MateApplyBody) =>
  postJson<MateApplyResult>("/api/assembly/mate", body);

export const restoreAssemblyMate = (backendId: string, movingWorldSnapshot: number[]) =>
  postJson<MateApplyResult>("/api/assembly/mate", {
    action: "restore",
    backendId,
    movingWorldSnapshot,
  });
