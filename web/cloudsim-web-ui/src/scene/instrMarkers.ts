import * as THREE from "three";
import type { Instruction } from "../api";
import { disposeObject3D, eulerZyxDegToQuat } from "./objectMesh";

/** 对齐桌面 kPointPickHitRadiusPx*1.5 */
export const WAYPOINT_PICK_HIT_RADIUS_PX = 48;

export type InstrWaypointHit = {
  instructionId: string;
  isArcVia: boolean;
  /** 标记组局部坐标（Z-up mm） */
  positionMm: THREE.Vector3;
};

/** 对齐旧网页 clearInstrMarkerChildren */
export function clearInstrMarkerGroup(group: THREE.Group) {
  for (const c of [...group.children]) {
    if (c.name === "waypointPickHoverRing") continue;
    group.remove(c);
    disposeObject3D(c);
  }
}

function addPoseMarker(
  group: THREE.Group,
  pose: { x?: number; y?: number; z?: number } | undefined,
  euler: { x?: number; y?: number; z?: number } | undefined,
  selected: boolean,
  instructionId: string,
  isArcVia: boolean,
) {
  if (!pose || !instructionId) return;
  const g = new THREE.Group();
  g.userData.instrId = instructionId;
  g.userData.isArcVia = isArcVia;
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
  opts: { hideForRawPreview: boolean; playing: boolean; preferVia?: boolean },
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
  const preferVia = !!opts.preferVia;
  for (const step of steps) {
    const type = String(step.type || "").toLowerCase();
    if (!["ptp", "line", "arc"].includes(type) || !step.pose) continue;
    const isSel = step.id === selectedInstrId;
    if (type === "arc" && step.viaPose) {
      addPoseMarker(
        group,
        step.viaPose,
        step.viaEulerDeg,
        isSel && preferVia,
        step.id,
        true,
      );
    }
    addPoseMarker(
      group,
      step.pose,
      step.eulerDeg,
      isSel && !preferVia,
      step.id,
      false,
    );
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

/** 屏幕空间最近路点（对齐桌面 tryPickInstructionWaypointAt） */
export function tryPickInstrWaypointAt(
  markers: THREE.Group,
  camera: THREE.Camera,
  clientX: number,
  clientY: number,
  canvas: HTMLCanvasElement,
  hitRadiusPx = WAYPOINT_PICK_HIT_RADIUS_PX,
): InstrWaypointHit | null {
  const rect = canvas.getBoundingClientRect();
  const vw = rect.width;
  const vh = rect.height;
  if (vw <= 1 || vh <= 1) return null;
  const mouseX = clientX - rect.left;
  const mouseY = clientY - rect.top;
  const hitR2 = hitRadiusPx * hitRadiusPx;
  const ndc = new THREE.Vector3();
  const world = new THREE.Vector3();
  let best: InstrWaypointHit | null = null;
  let bestD2 = hitR2;
  let bestDepth = Number.POSITIVE_INFINITY;

  markers.updateWorldMatrix(true, true);
  for (const child of markers.children) {
    const instrId = String(child.userData?.instrId || "");
    if (!instrId) continue;
    child.getWorldPosition(world);
    ndc.copy(world).project(camera);
    if (ndc.z < -1 || ndc.z > 1) continue;
    const sx = (ndc.x * 0.5 + 0.5) * vw;
    const sy = (1 - (ndc.y * 0.5 + 0.5)) * vh;
    const dx = sx - mouseX;
    const dy = sy - mouseY;
    const d2 = dx * dx + dy * dy;
    if (d2 > hitR2) continue;
    const depth = ndc.z;
    const depthTie = 1e-4;
    if (
      !best ||
      depth + depthTie < bestDepth ||
      (Math.abs(depth - bestDepth) <= depthTie && d2 < bestD2)
    ) {
      bestD2 = d2;
      bestDepth = depth;
      best = {
        instructionId: instrId,
        isArcVia: !!child.userData.isArcVia,
        positionMm: child.position.clone(),
      };
    }
  }
  return best;
}

function ensureHoverRing(markers: THREE.Group): THREE.Sprite {
  let ring = markers.getObjectByName("waypointPickHoverRing") as THREE.Sprite | undefined;
  if (ring) return ring;
  const size = 64;
  const canvas = document.createElement("canvas");
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext("2d")!;
  ctx.clearRect(0, 0, size, size);
  const stroke = (r: number, alpha: number, lineW: number) => {
    ctx.beginPath();
    ctx.arc(size / 2, size / 2, r, 0, Math.PI * 2);
    ctx.strokeStyle = `rgba(51, 242, 115, ${alpha})`;
    ctx.lineWidth = lineW;
    ctx.stroke();
  };
  stroke(22, 0.95, 3);
  stroke(28, 0.4, 2);
  const tex = new THREE.CanvasTexture(canvas);
  tex.needsUpdate = true;
  ring = new THREE.Sprite(
    new THREE.SpriteMaterial({
      map: tex,
      transparent: true,
      depthTest: false,
      depthWrite: false,
      sizeAttenuation: false,
    }),
  );
  ring.name = "waypointPickHoverRing";
  ring.visible = false;
  // sizeAttenuation=false 时 scale 近似屏幕像素
  ring.scale.set(56, 56, 1);
  markers.add(ring);
  return ring;
}

/** 悬停绿环：位姿在标记组局部 Z-up */
export function updateWaypointPickHover(
  markers: THREE.Group,
  _camera: THREE.Camera,
  hit: InstrWaypointHit | null,
) {
  const ring = ensureHoverRing(markers);
  if (!hit) {
    ring.visible = false;
    return;
  }
  ring.position.copy(hit.positionMm);
  ring.visible = true;
}

export function clearWaypointPickHover(markers: THREE.Group | undefined | null) {
  if (!markers) return;
  const ring = markers.getObjectByName("waypointPickHoverRing");
  if (ring) ring.visible = false;
}
