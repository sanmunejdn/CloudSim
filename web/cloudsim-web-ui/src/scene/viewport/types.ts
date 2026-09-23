/// @file types.ts
/// @brief 视口共享类型与 Z-up→Y-up 矩阵

import type { MutableRefObject } from "react";
import * as THREE from "three";
import type { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import type { TransformControls } from "three/examples/jsm/controls/TransformControls.js";
import type { ViewCubeHud, WorldAxesHud } from "../viewOrientationHud";

export const zUpToYUp = new THREE.Matrix4().makeRotationX(-Math.PI / 2);

export type RobotMeta = {
  isRobot: boolean;
  sceneRootBackendId?: string;
  anchorBackendId?: string;
  flangeBackendId?: string;
};

export type TeachPose = {
  positionMm: [number, number, number];
  eulerDeg: [number, number, number];
  jointRadCsv?: string;
  tcpLinkName?: string;
  flangeLinkName?: string;
  urdfPath?: string;
  toolFrameMat4Csv?: string;
  activeToolFrameId?: string;
  activeUserFrameId?: string;
  targetTransformQuatCsv?: string;
  targetTransformTransMmCsv?: string;
};

export type GizmoCtx = {
  selectedId: string | null;
  interactMode: "view" | "select";
  robotDragMode: boolean;
  gizmoTransformMode: "translate" | "rotate";
  gizmoSpace: "local" | "world";
  robotMeta: RobotMeta;
  activeRootId: string | null;
  refreshObjects: () => Promise<void>;
  setStatus: (m: string, k?: "info" | "err" | "warn") => void;
  setRobotDragTeachPose: (p: TeachPose | null) => void;
};

export type ViewportRefs = {
  mountRef: MutableRefObject<HTMLDivElement | null>;
  sceneRef: MutableRefObject<THREE.Scene | undefined>;
  rendererRef: MutableRefObject<THREE.WebGLRenderer | undefined>;
  cameraRef: MutableRefObject<THREE.PerspectiveCamera | undefined>;
  controlsRef: MutableRefObject<OrbitControls | undefined>;
  transformRef: MutableRefObject<TransformControls | undefined>;
  rootRef: MutableRefObject<THREE.Group | undefined>;
  contentRef: MutableRefObject<THREE.Group | undefined>;
  dragProxyRef: MutableRefObject<THREE.Object3D | undefined>;
  overlayRef: MutableRefObject<THREE.Group | undefined>;
  frameOverlayRef: MutableRefObject<THREE.Group | undefined>;
  rawPreviewRef: MutableRefObject<THREE.Group | undefined>;
  instrMarkersRef: MutableRefObject<THREE.Group | undefined>;
  idToMesh: MutableRefObject<Map<string, THREE.Object3D>>;
  boxRef: MutableRefObject<THREE.Box3>;
  geomSigRef: MutableRefObject<string>;
  focusPendingRef: MutableRefObject<boolean>;
  hoverPickSeqRef: MutableRefObject<number>;
  rawPreviewActiveRef: MutableRefObject<boolean>;
  viewCubeHudRef: MutableRefObject<ViewCubeHud | null>;
  axesHudRef: MutableRefObject<WorldAxesHud | null>;
  wireframeOnRef: MutableRefObject<boolean>;
  gizmoCtxRef: MutableRefObject<GizmoCtx>;
};
