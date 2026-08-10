import * as THREE from "three";
import type { Instruction } from "../api";
import { disposeObject3D, eulerZyxDegToQuat } from "./objectMesh";

/** 对齐旧网页 clearInstrMarkerChildren */
export function clearInstrMarkerGroup(group: THREE.Group) {
  for (const c of [...group.children]) {
    group.remove(c);
    disposeObject3D(c);
  }
}

function addPoseMarker(
  group: THREE.Group,
  pose: { x?: number; y?: number; z?: number } | undefined,
  euler: { x?: number; y?: number; z?: number } | undefined,
  selected: boolean,
  tag: string,
) {
  if (!pose) return;
  const g = new THREE.Group();
  g.userData.instrTag = tag || "";
  g.position.set(Number(pose.x) || 0, Number(pose.y) || 0, Number(pose.z) || 0);
  if (euler) {
    g.quaternion.copy(eulerZyxDegToQuat(Number(euler.x) || 0, Number(euler.y) || 0, Number(euler.z) || 0));
  }
  const len = selected ? 50 : 40;
  const addAxis = (dir: THREE.Vector3, color: number) => {
    const geo = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      dir.clone().multiplyScalar(len),
    ]);
    g.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color, depthTest: true })));
  };
  addAxis(new THREE.Vector3(1, 0, 0), 0xff6666);
  addAxis(new THREE.Vector3(0, 1, 0), 0x66ff66);
  addAxis(new THREE.Vector3(0, 0, 1), 0x6699ff);
  const pt = new THREE.BufferGeometry();
  pt.setAttribute("position", new THREE.Float32BufferAttribute([0, 0, 0], 3));
  g.add(
    new THREE.Points(
      pt,
      new THREE.PointsMaterial({
        color: selected ? 0xff6600 : 0x00ff00,
        size: selected ? 6 : 5,
        sizeAttenuation: false,
        depthTest: true,
      }),
    ),
  );
  group.add(g);
}

/** 对齐旧网页 refreshInstructionMarkers：PTP/LINE/ARC 路点 + 姿态轴 */
export function refreshInstrMarkers(
  group: THREE.Group,
  steps: Instruction[],
  selectedInstrId: string | null,
  opts: { hideForRawPreview: boolean; playing: boolean },
) {
  if (opts.hideForRawPreview && !opts.playing) {
    clearInstrMarkerGroup(group);
    return;
  }
  if (!steps.length) {
    clearInstrMarkerGroup(group);
    return;
  }
  clearInstrMarkerGroup(group);
  for (const step of steps) {
    const type = String(step.type || "").toLowerCase();
    if (!["ptp", "line", "arc"].includes(type) || !step.pose) continue;
    const sel = step.id === selectedInstrId;
    if (type === "arc" && step.viaPose) {
      addPoseMarker(group, step.viaPose, step.viaEulerDeg, false, "via");
    }
    addPoseMarker(group, step.pose, step.eulerDeg, sel, step.id);
  }

  // 折线串起路点，稠密路径才看得见（旧版仅 marker，React 补线不挡 marker）
  const poly: THREE.Vector3[] = [];
  for (const step of steps) {
    const type = String(step.type || "").toLowerCase();
    if (!["ptp", "line", "arc"].includes(type) || !step.pose) continue;
    poly.push(
      new THREE.Vector3(Number(step.pose.x) || 0, Number(step.pose.y) || 0, Number(step.pose.z) || 0),
    );
  }
  if (poly.length >= 2) {
    group.add(
      new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(poly),
        new THREE.LineBasicMaterial({ color: 0x33d9ff, depthTest: true }),
      ),
    );
  }
}
