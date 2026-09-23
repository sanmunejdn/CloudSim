/// @file SceneViewport.tsx
/// @brief Three 视口主机：组装 core / mesh / teach / interaction / overlays

import { useRef, useImperativeHandle, forwardRef } from "react";
import * as THREE from "three";
import type { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import type { TransformControls } from "three/examples/jsm/controls/TransformControls.js";
import { useScene } from "../state/sceneStore";
import { useStatus } from "../state/statusStore";
import { useRobotProgram } from "../state/robotProgramStore";
import { useFrames } from "../state/frameStore";
import { useTrajectory } from "../state/trajectoryStore";
import { usePointCloud } from "../state/pointCloudStore";
import { disposeObject3D } from "./objectMesh";
import { applyWorldViewDirection, type ViewCubeHud, type WorldAxesHud } from "./viewOrientationHud";
import { clearPickOverlayGroup } from "./meshLoad";
import type { GizmoCtx, ViewportRefs } from "./viewport/types";
import { useViewportCore } from "./viewport/useViewportCore";
import { useMeshSync } from "./viewport/useMeshSync";
import { useRobotTeach } from "./viewport/useRobotTeach";
import { useViewportInteraction } from "./viewport/useViewportInteraction";
import { useSceneOverlays } from "./viewport/useSceneOverlays";

export type SceneViewportHandle = {
  focusAll: () => void;
  homeView: () => void;
  /** 世界 Z-up 视线方向（从目标指向相机） */
  setViewDirection: (eyeWorld: [number, number, number], upWorld?: [number, number, number]) => void;
  setOverlayPolylines: (polys: number[][][]) => void;
  setOverlaySoup: (soup: Float32Array | null) => void;
  setWireframe: (on: boolean) => void;
  capturePng: () => void;
};

const SceneViewport = forwardRef<SceneViewportHandle>(function SceneViewport(_, ref) {
  const mountRef = useRef<HTMLDivElement>(null);
  const {
    objects,
    selectedId,
    selectObject,
    interactMode,
    robotDragMode,
    gizmoTransformMode,
    gizmoSpace,
    focusRequest,
    refreshObjects,
    setGizmoTransformMode,
    setRobotDragTeachPose,
    mateFacePickSlot,
    setMateFacePickSlot,
  } = useScene();
  const { setStatus } = useStatus();
  const { pickMode, featureEditActive, workpieceId, setWorkpieceId } = useTrajectory();
  const { polylinePickActive, addPolylinePoint, polylineScreenXy, renderRevision: pointCloudRevision } = usePointCloud();
  const {
    activeRootId,
    activeProgram,
    selectedInstrId,
    selectedInstrPreferVia,
    setSelectedInstrId,
    waypointPickMode,
    setWaypointPickMode,
    playing,
  } = useRobotProgram();
  const { frames } = useFrames();

  const sceneRef = useRef<THREE.Scene>();
  const rendererRef = useRef<THREE.WebGLRenderer>();
  const cameraRef = useRef<THREE.PerspectiveCamera>();
  const controlsRef = useRef<OrbitControls>();
  const transformRef = useRef<TransformControls>();
  const rootRef = useRef<THREE.Group>();
  const contentRef = useRef<THREE.Group>();
  const dragProxyRef = useRef<THREE.Object3D>();
  const overlayRef = useRef<THREE.Group>();
  const frameOverlayRef = useRef<THREE.Group>();
  const rawPreviewRef = useRef<THREE.Group>();
  const instrMarkersRef = useRef<THREE.Group>();
  const idToMesh = useRef(new Map<string, THREE.Object3D>());
  const boxRef = useRef(new THREE.Box3());
  const geomSigRef = useRef("");
  const focusPendingRef = useRef(false);
  const hoverPickSeqRef = useRef(0);
  const rawPreviewActiveRef = useRef(false);
  const viewCubeHudRef = useRef<ViewCubeHud | null>(null);
  const axesHudRef = useRef<WorldAxesHud | null>(null);
  const instrStepsRef = useRef(activeProgram?.instructions || []);
  const selectedInstrRef = useRef(selectedInstrId);
  const selectedPreferViaRef = useRef(selectedInstrPreferVia);
  const waypointPickModeRef = useRef(waypointPickMode);
  const mateFacePickSlotRef = useRef(mateFacePickSlot);
  const playingRef = useRef(playing);
  const wireframeOnRef = useRef(false);
  const gizmoCtxRef = useRef<GizmoCtx>({
    selectedId: null,
    interactMode: "view",
    robotDragMode: false,
    gizmoTransformMode: "translate",
    gizmoSpace: "local",
    robotMeta: { isRobot: false },
    activeRootId: null,
    refreshObjects: async () => {},
    setStatus: () => {},
    setRobotDragTeachPose: () => {},
  });

  const refs: ViewportRefs = {
    mountRef,
    sceneRef,
    rendererRef,
    cameraRef,
    controlsRef,
    transformRef,
    rootRef,
    contentRef,
    dragProxyRef,
    overlayRef,
    frameOverlayRef,
    rawPreviewRef,
    instrMarkersRef,
    idToMesh,
    boxRef,
    geomSigRef,
    focusPendingRef,
    hoverPickSeqRef,
    rawPreviewActiveRef,
    viewCubeHudRef,
    axesHudRef,
    wireframeOnRef,
    gizmoCtxRef,
  };

  instrStepsRef.current = activeProgram?.instructions || [];
  selectedInstrRef.current = selectedInstrId;
  selectedPreferViaRef.current = selectedInstrPreferVia;
  waypointPickModeRef.current = waypointPickMode;
  mateFacePickSlotRef.current = mateFacePickSlot;
  playingRef.current = playing;

  useViewportCore(refs);
  useMeshSync(refs, objects, selectedId, pointCloudRevision);

  const robotMeta = useRobotTeach(refs, {
    selectedId,
    interactMode,
    robotDragMode,
    gizmoTransformMode,
    gizmoSpace,
    objects,
    activeRootId,
    setGizmoTransformMode,
    setStatus,
  });

  // 事件回调读 ref：须在 render 同步刷新（勿放到 effect）
  gizmoCtxRef.current = {
    selectedId,
    interactMode,
    robotDragMode,
    gizmoTransformMode,
    gizmoSpace,
    robotMeta,
    activeRootId,
    refreshObjects,
    setStatus,
    setRobotDragTeachPose: setRobotDragTeachPose as GizmoCtx["setRobotDragTeachPose"],
  };

  useViewportInteraction(refs, {
    interactMode,
    pickMode,
    featureEditActive,
    workpieceId,
    setWorkpieceId,
    selectObject,
    setStatus,
    setSelectedInstrId,
    polylinePickActive,
    addPolylinePoint,
    polylineScreenXy,
    mateFacePickSlot,
    setMateFacePickSlot,
    waypointPickMode,
    setWaypointPickMode,
    mateFacePickSlotRef,
    waypointPickModeRef,
  });

  useSceneOverlays(refs, {
    activeRootId,
    objectsRevisionKey: objects,
    frames,
    robotDragMode,
    focusRequest,
    activeProgramInstructions: activeProgram?.instructions,
    selectedInstrId,
    selectedInstrPreferVia,
    playing,
    instrStepsRef,
    selectedInstrRef,
    selectedPreferViaRef,
    playingRef,
  });

  useImperativeHandle(ref, () => ({
    focusAll() {
      const box = boxRef.current;
      if (box.isEmpty() || !cameraRef.current || !controlsRef.current) return;
      const size = Math.max(box.getSize(new THREE.Vector3()).length(), 100);
      const center = box.getCenter(new THREE.Vector3());
      controlsRef.current.target.copy(center);
      cameraRef.current.position.copy(center.clone().add(new THREE.Vector3(size, size * 0.7, size)));
      controlsRef.current.update();
    },
    homeView() {
      if (!cameraRef.current || !controlsRef.current) return;
      cameraRef.current.position.set(800, 600, 1000);
      cameraRef.current.up.set(0, 1, 0);
      controlsRef.current.target.set(0, 0, 0);
      controlsRef.current.update();
    },
    setViewDirection(eyeWorld, upWorld) {
      if (!cameraRef.current || !controlsRef.current) return;
      applyWorldViewDirection(
        cameraRef.current,
        controlsRef.current,
        new THREE.Vector3(eyeWorld[0], eyeWorld[1], eyeWorld[2]),
        upWorld ? new THREE.Vector3(upWorld[0], upWorld[1], upWorld[2]) : new THREE.Vector3(0, 0, 1),
      );
    },
    setOverlayPolylines(polys) {
      const g = overlayRef.current;
      if (!g) return;
      clearPickOverlayGroup(g);
      const mat = new THREE.LineBasicMaterial({ color: 0xff9800 });
      for (const poly of polys) {
        const pts = poly.map((p) => new THREE.Vector3(p[0], p[1], p[2]));
        if (pts.length < 2) continue;
        const geo = new THREE.BufferGeometry().setFromPoints(pts);
        g.add(new THREE.Line(geo, mat));
      }
    },
    setOverlaySoup(soup) {
      const g = overlayRef.current;
      if (!g) return;
      const old = g.getObjectByName("pickSoup");
      if (old) {
        g.remove(old);
        disposeObject3D(old);
      }
      if (!soup || soup.length < 9) return;
      const geo = new THREE.BufferGeometry();
      geo.setAttribute("position", new THREE.BufferAttribute(soup, 3));
      geo.computeVertexNormals();
      const mesh = new THREE.Mesh(
        geo,
        new THREE.MeshBasicMaterial({ color: 0x2196f3, transparent: true, opacity: 0.35, side: THREE.DoubleSide }),
      );
      mesh.name = "pickSoup";
      g.add(mesh);
    },
    setWireframe(on) {
      wireframeOnRef.current = on;
      const root = contentRef.current;
      if (!root) return;
      root.traverse((o) => {
        if (o.userData?.isPointCloud) return;
        const mesh = o as THREE.Mesh;
        if (!mesh.isMesh || !mesh.material) return;
        const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
        for (const m of mats) {
          if (m && "wireframe" in m) (m as THREE.MeshBasicMaterial).wireframe = on;
        }
      });
    },
    capturePng() {
      const renderer = rendererRef.current;
      if (!renderer) return;
      const url = renderer.domElement.toDataURL("image/png");
      const a = document.createElement("a");
      a.href = url;
      a.download = `cloudsim-viewport-${Date.now()}.png`;
      a.click();
    },
  }));

  return <div className="scene" ref={mountRef} />;
});

export default SceneViewport;
