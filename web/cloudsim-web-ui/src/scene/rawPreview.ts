import * as THREE from "three";
import { eulerZyxDegToQuat } from "./objectMesh";

export type RawPreviewPayload = {
  ok?: boolean;
  pointsMm?: number[][];
  eulersDeg?: number[][];
  axesX?: number[][];
  axesY?: number[][];
  axesZ?: number[][];
  segmentEndExclusive?: number[];
  pointCount?: number;
};

export type PreviewAxisOpts = {
  x: boolean;
  y: boolean;
  z: boolean;
  interval: number;
};

export function rawPreviewSegmentRanges(pointCount: number, segmentEndExclusive: unknown): [number, number][] {
  if (pointCount < 2) return [];
  const ends = Array.isArray(segmentEndExclusive)
    ? segmentEndExclusive.map((v) => Math.trunc(Number(v))).filter((v) => v > 0 && v <= pointCount)
    : [];
  if (!ends.length) return [[0, pointCount]];
  const ranges: [number, number][] = [];
  let start = 0;
  for (const end of ends) {
    if (end > start && end <= pointCount) {
      ranges.push([start, end]);
      start = end;
    }
  }
  if (start + 1 < pointCount) ranges.push([start, pointCount]);
  return ranges;
}

function collectPreviewAxisIndices(pointCount: number, axisInterval: number) {
  if (pointCount <= 0) return [];
  if (pointCount === 1) return [0];
  const autoStride = Math.max(1, Math.floor(pointCount / 20));
  const stride = axisInterval > 0 ? axisInterval : autoStride;
  const set = new Set([0, pointCount - 1]);
  for (let i = stride; i < pointCount; i += stride) set.add(i);
  return [...set].sort((a, b) => a - b);
}

function disposeTree(obj: THREE.Object3D) {
  obj.traverse((o) => {
    const m = o as THREE.Mesh;
    m.geometry?.dispose?.();
    const mat = m.material as THREE.Material | THREE.Material[] | undefined;
    if (Array.isArray(mat)) mat.forEach((x) => x.dispose());
    else mat?.dispose?.();
  });
}

export function clearRawPreviewGroup(group: THREE.Group) {
  for (const c of [...group.children]) {
    group.remove(c);
    disposeTree(c);
  }
  group.visible = false;
}

/** 对齐桌面/旧网页：折线 + 全量点 + 可选姿态轴 */
export function applyRawPreviewToGroup(group: THREE.Group, preview: RawPreviewPayload | null, axisOpts: PreviewAxisOpts) {
  clearRawPreviewGroup(group);
  const ptsArr = preview?.pointsMm || [];
  if (ptsArr.length < 1) return;

  const toVec = (p: number[]) =>
    new THREE.Vector3(Number(p[0]) || 0, Number(p[1]) || 0, Number(p[2]) || 0);

  const lineMat = new THREE.LineBasicMaterial({ color: 0x33d9ff, depthTest: true });
  const ranges = rawPreviewSegmentRanges(ptsArr.length, preview?.segmentEndExclusive);
  for (const [begin, endEx] of ranges) {
    if (endEx <= begin + 1) continue;
    const pts: THREE.Vector3[] = [];
    for (let i = begin; i < endEx; i++) pts.push(toVec(ptsArr[i] || []));
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    geo.computeBoundingSphere();
    const line = new THREE.Line(geo, lineMat);
    line.frustumCulled = false;
    group.add(line);
  }

  const allPts = ptsArr.map((p) => toVec(p || []));
  const ptGeo = new THREE.BufferGeometry().setFromPoints(allPts);
  ptGeo.computeBoundingSphere();
  const ptsObj = new THREE.Points(
    ptGeo,
    new THREE.PointsMaterial({ color: 0x00e676, size: 3, sizeAttenuation: false, depthTest: true }),
  );
  ptsObj.frustumCulled = false;
  group.add(ptsObj);

  if (axisOpts.x || axisOpts.y || axisOpts.z) {
    const eulers = preview?.eulersDeg || [];
    const axesX = preview?.axesX || [];
    const axesY = preview?.axesY || [];
    const axesZ = preview?.axesZ || [];
    const indices = collectPreviewAxisIndices(ptsArr.length, axisOpts.interval);
    for (const i of indices) {
      const p = ptsArr[i] || [];
      const g = new THREE.Group();
      g.position.set(Number(p[0]) || 0, Number(p[1]) || 0, Number(p[2]) || 0);
      const ax = axesX[i];
      const ay = axesY[i];
      const az = axesZ[i];
      const hasDirs = !!(ax || ay || az);
      if (!hasDirs) {
        const ev = eulers[i];
        if (Array.isArray(ev)) g.quaternion.copy(eulerZyxDegToQuat(Number(ev[0]) || 0, Number(ev[1]) || 0, Number(ev[2]) || 0));
      }
      const len = 40;
      const addAxis = (dir: THREE.Vector3, color: number) => {
        const d = dir.clone().normalize().multiplyScalar(len);
        const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), d]);
        g.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color, depthTest: true })));
      };
      const worldOrLocal = (worldArr: number[] | undefined, local: THREE.Vector3) => {
        if (Array.isArray(worldArr) && worldArr.length >= 3) {
          return new THREE.Vector3(Number(worldArr[0]) || 0, Number(worldArr[1]) || 0, Number(worldArr[2]) || 0);
        }
        return local;
      };
      if (axisOpts.x) addAxis(worldOrLocal(ax, new THREE.Vector3(1, 0, 0)), 0xe53935);
      if (axisOpts.y) addAxis(worldOrLocal(ay, new THREE.Vector3(0, 1, 0)), 0x43a047);
      if (axisOpts.z) addAxis(worldOrLocal(az, new THREE.Vector3(0, 0, 1)), 0x1e88e5);
      const tip = new THREE.BufferGeometry();
      tip.setAttribute("position", new THREE.Float32BufferAttribute([0, 0, 0], 3));
      g.add(
        new THREE.Points(
          tip,
          new THREE.PointsMaterial({ color: 0x33d9ff, size: 2, sizeAttenuation: false, depthTest: true }),
        ),
      );
      group.add(g);
    }
  }

  group.visible = true;
}

export function publishRawPreview(preview: RawPreviewPayload | null, axisOpts: PreviewAxisOpts) {
  window.dispatchEvent(new CustomEvent("cloudsim-raw-preview", { detail: { preview, axisOpts } }));
}
