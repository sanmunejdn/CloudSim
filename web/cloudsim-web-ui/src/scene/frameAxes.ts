import * as THREE from "three";
import type { BackendObject } from "../api";
import { eulerZyxDegToQuat } from "./objectMesh";

export function isSceneCoordinateFrame(o: BackendObject | undefined | null) {
  if (!o) return false;
  const c = String(o.className || "");
  return c === "FrameBackendData" || c === "CoordinateFrame" || c === "Frame";
}

/** 场景 FrameBackendData：客户端合成 RGB 短轴（无 mesh soup） */
export function makeCoordinateFrameAxes(axisLengthMm = 100) {
  const len = Math.max(1, Number(axisLengthMm) || 100);
  const g = new THREE.Group();
  const mk = (dir: number[], color: number) => {
    const geo = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(dir[0] * len, dir[1] * len, dir[2] * len),
    ]);
    return new THREE.Line(geo, new THREE.LineBasicMaterial({ color, depthTest: true }));
  };
  g.add(mk([1, 0, 0], 0xe53935));
  g.add(mk([0, 1, 0], 0x43a047));
  g.add(mk([0, 0, 1], 0x1e88e5));
  g.userData.isCoordinateFrame = true;
  return g;
}

export type OverlayFrame = {
  active?: boolean;
  positionMm?: number[];
  eulerDeg?: number[];
  worldMatrix?: number[];
};

export function clearFrameOverlayGroup(group: THREE.Group) {
  for (const c of [...group.children]) {
    group.remove(c);
    c.traverse((o) => {
      const m = o as THREE.Mesh & THREE.Line;
      m.geometry?.dispose?.();
      const mat = m.material as THREE.Material | THREE.Material[] | undefined;
      if (Array.isArray(mat)) mat.forEach((x) => x.dispose());
      else mat?.dispose?.();
    });
  }
}

export function addFrameAxisMarker(
  group: THREE.Group,
  worldMatrixOrPose: number[] | undefined,
  euler: number[] | undefined,
  active: boolean,
  kind: "tool" | "user",
) {
  const g = new THREE.Group();
  g.userData.frameKind = kind;
  g.userData.active = !!active;
  if (Array.isArray(worldMatrixOrPose) && worldMatrixOrPose.length >= 16) {
    const m = new THREE.Matrix4().fromArray(worldMatrixOrPose.map(Number));
    m.decompose(g.position, g.quaternion, g.scale);
  } else if (worldMatrixOrPose && worldMatrixOrPose.length >= 3) {
    g.position.set(Number(worldMatrixOrPose[0]) || 0, Number(worldMatrixOrPose[1]) || 0, Number(worldMatrixOrPose[2]) || 0);
    if (euler) g.quaternion.copy(eulerZyxDegToQuat(Number(euler[0]) || 0, Number(euler[1]) || 0, Number(euler[2]) || 0));
  } else {
    return;
  }
  const len = active ? 55 : 40;
  const addAxis = (dir: THREE.Vector3, color: number) => {
    const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), dir.clone().multiplyScalar(len)]);
    g.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color, depthTest: true })));
  };
  addAxis(new THREE.Vector3(1, 0, 0), kind === "user" ? 0xffaa66 : 0xff6666);
  addAxis(new THREE.Vector3(0, 1, 0), kind === "user" ? 0xaaff66 : 0x66ff66);
  addAxis(new THREE.Vector3(0, 0, 1), kind === "user" ? 0x66aaff : 0x6699ff);
  const pt = new THREE.BufferGeometry();
  pt.setAttribute("position", new THREE.Float32BufferAttribute([0, 0, 0], 3));
  g.add(
    new THREE.Points(
      pt,
      new THREE.PointsMaterial({
        color: active ? 0xffcc00 : kind === "user" ? 0x66ccff : 0x00ff88,
        size: active ? 7 : 5,
        sizeAttenuation: false,
        depthTest: true,
      }),
    ),
  );
  group.add(g);
}

/** 拖动示教：活动工具 TCP 跟罗盘代理，勿等 FK 刷新 */
export function syncActiveToolOverlayFromProxy(group: THREE.Group | undefined, proxy: THREE.Object3D) {
  if (!group) return;
  proxy.updateMatrix();
  let marker: THREE.Object3D | undefined;
  for (const c of group.children) {
    if (c.userData.frameKind === "tool" && c.userData.active) {
      marker = c;
      break;
    }
  }
  if (!marker) {
    for (const c of group.children) {
      if (c.userData.frameKind === "tool") {
        marker = c;
        break;
      }
    }
  }
  if (!marker) {
    proxy.updateMatrix();
    addFrameAxisMarker(group, Array.from(proxy.matrix.elements), undefined, true, "tool");
    return;
  }
  marker.position.copy(proxy.position);
  marker.quaternion.copy(proxy.quaternion);
  marker.scale.set(1, 1, 1);
  marker.updateMatrixWorld(true);
}
