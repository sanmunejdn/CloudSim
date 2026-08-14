import * as THREE from "three";
import type { BackendObject } from "../api";

export function eulerZyxDegToQuat(ex: number, ey: number, ez: number): THREE.Quaternion {
  const qx = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), THREE.MathUtils.degToRad(ex));
  const qy = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), THREE.MathUtils.degToRad(ey));
  const qz = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), THREE.MathUtils.degToRad(ez));
  return qz.multiply(qy).multiply(qx);
}

export function colorFromObject(o: BackendObject | undefined, selected: boolean): THREE.Color {
  const c = o?.color;
  if (c && Number.isFinite(c.r) && Number.isFinite(c.g) && Number.isFinite(c.b)) {
    let r = c.r;
    let g = c.g;
    let b = c.b;
    // 兼容 0–255 误传
    if (r > 1 || g > 1 || b > 1) {
      r /= 255;
      g /= 255;
      b /= 255;
    }
    const col = new THREE.Color(r, g, b);
    const hsl = { h: 0, s: 0, l: 0 };
    col.getHSL(hsl);
    if (hsl.l < 0.22) col.offsetHSL(0, 0, 0.22 - hsl.l);
    if (selected) col.offsetHSL(0, 0.04, 0.08);
    return col;
  }
  return new THREE.Color(selected ? 0x2b79c2 : 0xb0b4b8);
}

export function createMeshMaterial(color: THREE.Color, selected: boolean) {
  return new THREE.MeshPhongMaterial({
    color,
    specular: new THREE.Color(0x2e2e2e),
    shininess: 42,
    side: THREE.DoubleSide,
    emissive: new THREE.Color(selected ? 0x1a4060 : 0x000000),
    emissiveIntensity: selected ? 0.28 : 0,
  });
}

export function applyMeshSelectionStyle(mesh: THREE.Object3D, o: BackendObject | undefined, selected: boolean) {
  const m = mesh as THREE.Mesh;
  const mat = m.material as THREE.MeshPhongMaterial | undefined;
  if (!mat?.color) return;
  mat.color.copy(colorFromObject(o, selected));
  if (mat.emissive) {
    mat.emissive.setHex(selected ? 0x1a4060 : 0x000000);
    mat.emissiveIntensity = selected ? 0.28 : 0;
  }
}

/** Gateway 已转为 Three/OpenGL 列主序；连杆 FK 必须用 worldMatrix，不能只用 pose 欧拉 */
export function applyObjectTransform(mesh: THREE.Object3D, o: BackendObject) {
  const wm = o.worldMatrix;
  if (Array.isArray(wm) && wm.length === 16) {
    mesh.matrixAutoUpdate = false;
    mesh.matrix.fromArray(wm.map(Number));
    mesh.matrix.decompose(mesh.position, mesh.quaternion, mesh.scale);
    mesh.updateMatrixWorld(true);
    return;
  }
  mesh.matrixAutoUpdate = true;
  const p = o.pose?.positionMm || [0, 0, 0];
  const e = o.pose?.eulerDeg || [0, 0, 0];
  mesh.position.set(p[0], p[1], p[2]);
  mesh.quaternion.copy(eulerZyxDegToQuat(e[0], e[1], e[2]));
  mesh.scale.set(1, 1, 1);
}

/** worldMatrix 路径下 mesh 禁自动更新；挂 TransformControls 前必须解开 */
export function prepareMeshForGizmo(mesh: THREE.Object3D) {
  if (!mesh.matrixAutoUpdate) {
    mesh.matrix.decompose(mesh.position, mesh.quaternion, mesh.scale);
    mesh.matrixAutoUpdate = true;
  }
}

export function localMatrixArray(obj: THREE.Object3D): number[] {
  obj.updateMatrix();
  return Array.from(obj.matrix.elements);
}

/** 列主序 4×4 → 位姿（ZYX 欧拉仅示教落盘） */
export function poseFromMatrix4Elements(elements: number[]): {
  positionMm: [number, number, number];
  eulerDeg: [number, number, number];
} {
  const m = new THREE.Matrix4().fromArray(elements.map(Number));
  const p = new THREE.Vector3();
  const q = new THREE.Quaternion();
  const s = new THREE.Vector3();
  m.decompose(p, q, s);
  const e = new THREE.Euler().setFromQuaternion(q, "ZYX");
  return {
    positionMm: [p.x, p.y, p.z],
    eulerDeg: [
      THREE.MathUtils.radToDeg(e.x),
      THREE.MathUtils.radToDeg(e.y),
      THREE.MathUtils.radToDeg(e.z),
    ],
  };
}

/** 把代理对齐到目标 mesh 在 root（Z-up）下的局部位姿 */
export function snapProxyToMesh(root: THREE.Object3D, proxy: THREE.Object3D, mesh: THREE.Object3D) {
  mesh.updateMatrixWorld(true);
  root.updateMatrixWorld(true);
  const local = new THREE.Matrix4().copy(root.matrixWorld).invert().multiply(mesh.matrixWorld);
  local.decompose(proxy.position, proxy.quaternion, proxy.scale);
  proxy.matrixAutoUpdate = true;
  proxy.updateMatrix();
  proxy.updateMatrixWorld(true);
}

/** 锚点无可见网格时，用 Gateway 下发的 worldMatrix 对齐代理 */
export function snapProxyFromWorldMatrix(proxy: THREE.Object3D, wm16: number[] | undefined) {
  if (!Array.isArray(wm16) || wm16.length < 16) return false;
  const m = new THREE.Matrix4().fromArray(wm16.map(Number));
  m.decompose(proxy.position, proxy.quaternion, proxy.scale);
  proxy.matrixAutoUpdate = true;
  proxy.updateMatrix();
  proxy.updateMatrixWorld(true);
  return true;
}

export function disposeObject3D(obj: THREE.Object3D) {
  obj.traverse((x) => {
    const m = x as THREE.Mesh;
    m.geometry?.dispose?.();
    const mat = m.material as THREE.Material | THREE.Material[] | undefined;
    if (Array.isArray(mat)) mat.forEach((mm) => mm.dispose());
    else mat?.dispose?.();
  });
}
