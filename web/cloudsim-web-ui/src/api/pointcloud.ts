import { apiJson, postJson, type ApiOk } from "./client";

export const POINT_CLOUD_MIXED_THRESHOLD = 500_000;
export const POINT_CLOUD_PREVIEW_MAX = 500_000;
export const POINT_CLOUD_CHUNK_SIZE = 250_000;

export type PointCloudInfo = {
  pointCount: number;
  hasPerVertexColors?: boolean;
  hasPointNormals?: boolean;
  bounds?: { valid?: boolean; minMm?: number[]; maxMm?: number[] };
  mixedRenderThreshold?: number;
};

export type PointCloudMeasure = {
  centroidMm?: number[];
  averageSpacingMm?: number;
  bounds?: { valid?: boolean; minMm?: number[]; maxMm?: number[] };
};

export type PointCloudJobResult = ApiOk & {
  backendId?: string;
  newBackendId?: string;
  pointCountAfter?: number;
  rmseMm?: number;
  meanErrorMm?: number;
  path?: string;
  patchCount?: number;
  maxDeviationMm?: number;
  debugReport?: string;
};

export const fetchPointCloudInfo = (id: string) =>
  apiJson<{ ok: boolean; info?: PointCloudInfo; error?: string }>(`/api/pointcloud/info/${encodeURIComponent(id)}`);

export const fetchPointCloudMeasure = (id: string) =>
  apiJson<{ ok: boolean; measure?: PointCloudMeasure; error?: string }>(
    `/api/pointcloud/measure/${encodeURIComponent(id)}`,
  );

export async function fetchPointCloudPreview(id: string, maxPoints = POINT_CLOUD_PREVIEW_MAX): Promise<Float32Array | null> {
  const r = await fetch(`/api/pointcloud/preview/${encodeURIComponent(id)}?maxPoints=${maxPoints}`);
  if (!r.ok) return null;
  return new Float32Array(await r.arrayBuffer());
}

export type PointCloudChunkMeta = {
  lod?: number;
  index?: number;
  chunkCount?: number;
  pointCount?: number;
  totalPoints?: number;
};

export async function fetchPointCloudChunk(
  id: string,
  index: number,
  lod = 0,
  maxPoints = POINT_CLOUD_CHUNK_SIZE,
): Promise<{ soup: Float32Array; meta: PointCloudChunkMeta } | null> {
  const r = await fetch(
    `/api/pointcloud/chunk/${encodeURIComponent(id)}?lod=${lod}&index=${index}&maxPoints=${maxPoints}`,
  );
  if (!r.ok) return null;
  const metaHeader = r.headers.get("X-Chunk-Meta");
  let meta: PointCloudChunkMeta = {};
  if (metaHeader) {
    try {
      meta = JSON.parse(metaHeader) as PointCloudChunkMeta;
    } catch {
      /* ignore */
    }
  }
  return { soup: new Float32Array(await r.arrayBuffer()), meta };
}

export const downsamplePointCloud = (body: unknown) => postJson<PointCloudJobResult>("/api/pointcloud/downsample", body);
export const cropPointCloud = (body: unknown) => postJson<PointCloudJobResult>("/api/pointcloud/crop", body);
export const preprocessPointCloud = (body: unknown) => postJson<PointCloudJobResult>("/api/pointcloud/preprocess", body);
export const registerPointCloud = (body: unknown) => postJson<PointCloudJobResult>("/api/pointcloud/register", body);
export const reconstructPointCloud = (body: unknown) => postJson<PointCloudJobResult>("/api/pointcloud/reconstruct", body);
export const meshPostPointCloud = (body: unknown) => postJson<PointCloudJobResult>("/api/pointcloud/mesh/post", body);
export const meshExportPly = (body: unknown) => postJson<PointCloudJobResult>("/api/pointcloud/mesh/export-ply", body);
export const surfaceRun = (body: unknown) => postJson<PointCloudJobResult>("/api/pointcloud/surface/run", body);
export const surfaceReset = (body: unknown) => postJson<PointCloudJobResult>("/api/pointcloud/surface/reset", body);
