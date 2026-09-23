/// @file meshLoad.ts
/// @brief 视口网格/点云装载与叠加组清理

import * as THREE from "three";
import {
  fetchPointCloudChunk,
  fetchPointCloudInfo,
  fetchPointCloudPreview,
  POINT_CLOUD_CHUNK_SIZE,
  POINT_CLOUD_MIXED_THRESHOLD,
  POINT_CLOUD_PREVIEW_MAX,
  type BackendObject,
} from "../api";
import { disposeObject3D } from "./objectMesh";
import { isSceneCoordinateFrame } from "./frameAxes";

export const VIEW_BG = 0xe8eaed;

export function geomSignature(objects: BackendObject[], pointCloudRevision: number) {
  return objects
    .filter((o) => o.visible && (isSceneCoordinateFrame(o) || o.hasGeometry))
    .map((o) =>
      isSceneCoordinateFrame(o) ? `f:${o.id}` : o.geometryKind === 1 ? `p:${o.id}:${pointCloudRevision}` : o.id,
    )
    .sort()
    .join(",");
}

export async function loadPointCloudObject(obj: BackendObject): Promise<THREE.Points | null> {
  const info = await fetchPointCloudInfo(obj.id);
  const count = info.ok && info.info ? info.info.pointCount : 0;
  const positions: number[] = [];
  if (count > POINT_CLOUD_MIXED_THRESHOLD) {
    let index = 0;
    for (;;) {
      const chunk = await fetchPointCloudChunk(obj.id, index, 0, POINT_CLOUD_CHUNK_SIZE);
      if (!chunk || chunk.soup.length < 3) break;
      for (let i = 0; i < chunk.soup.length; i++) positions.push(chunk.soup[i]);
      const total = chunk.meta.chunkCount ?? 1;
      index += 1;
      if (index >= total) break;
    }
  } else {
    const soup = await fetchPointCloudPreview(obj.id, POINT_CLOUD_PREVIEW_MAX);
    if (soup) for (let i = 0; i < soup.length; i++) positions.push(soup[i]);
  }
  if (positions.length < 3) return null;
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(positions), 3));
  geo.computeBoundingSphere();
  geo.computeBoundingBox();
  // mm 场景下 sizeAttenuation 会把点缩成看不见
  const pts = new THREE.Points(
    geo,
    new THREE.PointsMaterial({
      color: 0x4fc3f7,
      size: 3,
      sizeAttenuation: false,
      depthTest: true,
    }),
  );
  pts.frustumCulled = true;
  pts.userData.backendId = obj.id;
  pts.userData.isPointCloud = true;
  return pts;
}

export function clearPickOverlayGroup(g: THREE.Group | undefined) {
  if (!g) return;
  while (g.children.length) {
    const c = g.children[0];
    g.remove(c);
    disposeObject3D(c);
  }
}

export function drainGroup(g: THREE.Object3D | undefined) {
  if (!g) return;
  while (g.children.length) {
    const c = g.children[0];
    g.remove(c);
    disposeObject3D(c);
  }
}
