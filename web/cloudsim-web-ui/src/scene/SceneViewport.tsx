import { useEffect, useRef, useImperativeHandle, forwardRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { TransformControls } from "three/examples/jsm/controls/TransformControls.js";
import {
  fetchFrameOverlays,
  fetchMeshSoup,
  fetchPointCloudChunk,
  fetchPointCloudInfo,
  fetchPointCloudPreview,
  POINT_CLOUD_CHUNK_SIZE,
  POINT_CLOUD_MIXED_THRESHOLD,
  POINT_CLOUD_PREVIEW_MAX,
  patchObject,
  placeRobot,
  resolveRobot,
  tcpIk,
  tcpPose,
  type BackendObject,
} from "../api";
import { useScene } from "../state/sceneStore";
import { useStatus } from "../state/statusStore";
import { useRobotProgram } from "../state/robotProgramStore";
import { useFrames } from "../state/frameStore";
import { pickHover } from "../api/trajectory";
import { useTrajectory } from "../state/trajectoryStore";
import { usePointCloud } from "../state/pointCloudStore";
import {
  applyMeshSelectionStyle,
  applyObjectTransform,
  colorFromObject,
  createMeshMaterial,
  disposeObject3D,
  localMatrixArray,
  poseFromMatrix4Elements,
  prepareMeshForGizmo,
  snapProxyFromWorldMatrix,
  snapProxyToMesh,
} from "./objectMesh";
import {
  addFrameAxisMarker,
  clearFrameOverlayGroup,
  isSceneCoordinateFrame,
  makeCoordinateFrameAxes,
  syncActiveToolOverlayFromProxy,
} from "./frameAxes";
import { applyRawPreviewToGroup, clearRawPreviewGroup, type PreviewAxisOpts, type RawPreviewPayload } from "./rawPreview";
import { refreshInstrMarkers, tryPickInstrWaypointAt, updateWaypointPickHover, clearWaypointPickHover } from "./instrMarkers";
import {
  applyWorldViewDirection,
  createViewCubeHud,
  createWorldAxesHud,
  type ViewCubeHud,
  type WorldAxesHud,
} from "./viewOrientationHud";

const zUpToYUp = new THREE.Matrix4().makeRotationX(-Math.PI / 2);
const VIEW_BG = 0xe8eaed;

function geomSignature(objects: BackendObject[], pointCloudRevision: number) {
  return objects
    .filter((o) => o.visible && (isSceneCoordinateFrame(o) || o.hasGeometry))
    .map((o) =>
      isSceneCoordinateFrame(o) ? `f:${o.id}` : o.geometryKind === 1 ? `p:${o.id}:${pointCloudRevision}` : o.id,
    )
    .sort()
    .join(",");
}

async function loadPointCloudObject(obj: BackendObject): Promise<THREE.Points | null> {
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
  // mm 场景下 sizeAttenuation 会把点缩成看不见；与轨迹预览一致用像素尺寸
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

function clearPickOverlayGroup(g: THREE.Group | undefined) {
  if (!g) return;
  while (g.children.length) {
    const c = g.children[0];
    g.remove(c);
    disposeObject3D(c);
  }
}

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

type RobotMeta = {
  isRobot: boolean;
  sceneRootBackendId?: string;
  anchorBackendId?: string;
  flangeBackendId?: string;
};

const SceneViewport = forwardRef<SceneViewportHandle>(function SceneViewport(_, ref) {
  const mountRef = useRef<HTMLDivElement>(null);
  const { objects, selectedId, selectObject, interactMode, robotDragMode, gizmoTransformMode, gizmoSpace, focusRequest, refreshObjects, setGizmoTransformMode, setRobotDragTeachPose, mateFacePickSlot, setMateFacePickSlot } =
    useScene();
  const { setStatus } = useStatus();
  const { pickMode, featureEditActive, workpieceId, setWorkpieceId } = useTrajectory();
  const { polylinePickActive, addPolylinePoint, polylineScreenXy, renderRevision: pointCloudRevision } = usePointCloud();
  const { activeRootId, activeProgram, selectedInstrId, selectedInstrPreferVia, setSelectedInstrId, waypointPickMode, setWaypointPickMode, playing } =
    useRobotProgram();
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
  const [robotMeta, setRobotMeta] = useState<RobotMeta>({ isRobot: false });
  const gizmoCtxRef = useRef({
    selectedId: null as string | null,
    interactMode: "view" as "view" | "select",
    robotDragMode: false,
    gizmoTransformMode: "translate" as "translate" | "rotate",
    gizmoSpace: "local" as "local" | "world",
    robotMeta: { isRobot: false } as RobotMeta,
    activeRootId: null as string | null,
    refreshObjects: (async () => {}) as () => Promise<void>,
    setStatus: ((_m: string, _k?: "info" | "err" | "warn") => {}) as (
      m: string,
      k?: "info" | "err" | "warn",
    ) => void,
    setRobotDragTeachPose: ((_p: ReturnType<typeof poseFromMatrix4Elements> & { jointRadCsv?: string } | null) =>
      {}) as (p: (ReturnType<typeof poseFromMatrix4Elements> & { jointRadCsv?: string }) | null) => void,
  });
  const wireframeOnRef = useRef(false);
  instrStepsRef.current = activeProgram?.instructions || [];
  selectedInstrRef.current = selectedInstrId;
  selectedPreferViaRef.current = selectedInstrPreferVia;
  waypointPickModeRef.current = waypointPickMode;
  mateFacePickSlotRef.current = mateFacePickSlot;
  playingRef.current = playing;
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
    setRobotDragTeachPose,
  };

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
        upWorld
          ? new THREE.Vector3(upWorld[0], upWorld[1], upWorld[2])
          : new THREE.Vector3(0, 0, 1),
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

  // 拾取结束/取消后清掉面/边高亮（对齐旧版 hover 在 pickMode 为空时 clear）
  useEffect(() => {
    if (pickMode && featureEditActive) return;
    clearPickOverlayGroup(overlayRef.current);
  }, [pickMode, featureEditActive]);

  // 对象选择与路点拾取互斥
  useEffect(() => {
    if (interactMode === "select" && waypointPickMode) setWaypointPickMode(false);
  }, [interactMode, waypointPickMode, setWaypointPickMode]);

  useEffect(() => {
    if (!waypointPickMode) clearWaypointPickHover(instrMarkersRef.current);
  }, [waypointPickMode]);

  useEffect(() => {
    if (!waypointPickMode) return;
    const onKey = (ev: KeyboardEvent) => {
      if (ev.key === "Escape") {
        setWaypointPickMode(false);
        clearWaypointPickHover(instrMarkersRef.current);
        setStatus("已退出路点拾取");
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [waypointPickMode, setWaypointPickMode, setStatus]);

  useEffect(() => {
    const mount = mountRef.current!;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(VIEW_BG);
    const camera = new THREE.PerspectiveCamera(50, 1, 1, 1e7);
    camera.position.set(800, 600, 1000);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setClearColor(VIEW_BG, 1);
    renderer.setPixelRatio(window.devicePixelRatio);
    mount.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    // 对齐常见 CAD：中键平移，左键旋转，滚轮缩放
    controls.mouseButtons.LEFT = THREE.MOUSE.ROTATE;
    controls.mouseButtons.MIDDLE = THREE.MOUSE.PAN;
    controls.mouseButtons.RIGHT = THREE.MOUSE.DOLLY;

    const viewCube = createViewCubeHud((eyeWorld, upWorld) => {
      applyWorldViewDirection(camera, controls, eyeWorld, upWorld);
    });
    const axesHud = createWorldAxesHud();
    mount.appendChild(viewCube.canvas);
    mount.appendChild(axesHud.canvas);
    viewCubeHudRef.current = viewCube;
    axesHudRef.current = axesHud;
    const transform = new TransformControls(camera, renderer.domElement);
    transform.setSize(1.25);
    transform.setMode("translate");
    transform.setSpace("local");
    transform.enabled = false;
    scene.add(transform.getHelper());
    transform.getHelper().visible = false;

    scene.add(new THREE.HemisphereLight(0xffffff, 0xb8bcc2, 0.9));
    const keyLight = new THREE.DirectionalLight(0xffffff, 1.2);
    keyLight.position.set(0.65, 1.1, 0.55);
    scene.add(keyLight);
    const fillLight = new THREE.DirectionalLight(0xf2f5fa, 0.55);
    fillLight.position.set(-0.8, 0.35, -0.45);
    scene.add(fillLight);
    const rimLight = new THREE.DirectionalLight(0xffffff, 0.28);
    rimLight.position.set(-0.2, 0.6, -1);
    scene.add(rimLight);

    const root = new THREE.Group();
    root.applyMatrix4(zUpToYUp);
    scene.add(root);
    // 网格进 content，proxy 与 content 并列，避免重建时 dispose 掉拖拽代理
    const content = new THREE.Group();
    content.name = "sceneContent";
    root.add(content);
    const dragProxy = new THREE.Object3D();
    dragProxy.name = "robotDragProxy";
    root.add(dragProxy);
    const overlay = new THREE.Group();
    overlay.applyMatrix4(zUpToYUp);
    scene.add(overlay);
    const frameOverlays = new THREE.Group();
    frameOverlays.name = "frameOverlays";
    frameOverlays.applyMatrix4(zUpToYUp);
    scene.add(frameOverlays);
    const rawPreview = new THREE.Group();
    rawPreview.name = "rawPreviewOverlay";
    rawPreview.visible = false;
    rawPreview.applyMatrix4(zUpToYUp);
    scene.add(rawPreview);
    const instrMarkers = new THREE.Group();
    instrMarkers.name = "instrMarkers";
    instrMarkers.applyMatrix4(zUpToYUp);
    scene.add(instrMarkers);

    sceneRef.current = scene;
    rendererRef.current = renderer;
    cameraRef.current = camera;
    controlsRef.current = controls;
    transformRef.current = transform;
    rootRef.current = root;
    contentRef.current = content;
    dragProxyRef.current = dragProxy;
    overlayRef.current = overlay;
    frameOverlayRef.current = frameOverlays;
    rawPreviewRef.current = rawPreview;
    instrMarkersRef.current = instrMarkers;

    const resize = () => {
      const w = mount.clientWidth;
      const h = Math.max(mount.clientHeight, 1);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    };
    resize();
    const ro = new ResizeObserver(resize);
    ro.observe(mount);

    let raf = 0;
    const tick = () => {
      controls.update();
      renderer.render(scene, camera);
      viewCube.sync(camera);
      axesHud.sync(camera);
      raf = requestAnimationFrame(tick);
    };
    tick();

    return () => {
      cancelAnimationFrame(raf);
      ro.disconnect();
      transform.dispose();
      controls.dispose();
      viewCube.dispose();
      axesHud.dispose();
      viewCubeHudRef.current = null;
      axesHudRef.current = null;
      if (viewCube.canvas.parentElement === mount) mount.removeChild(viewCube.canvas);
      if (axesHud.canvas.parentElement === mount) mount.removeChild(axesHud.canvas);

      const drainGroup = (g: THREE.Group | undefined) => {
        if (!g) return;
        while (g.children.length) {
          const c = g.children[0];
          g.remove(c);
          disposeObject3D(c);
        }
      };
      drainGroup(contentRef.current);
      drainGroup(overlayRef.current);
      drainGroup(frameOverlayRef.current);
      drainGroup(rawPreviewRef.current);
      drainGroup(instrMarkersRef.current);
      if (rootRef.current) {
        while (rootRef.current.children.length) {
          const c = rootRef.current.children[0];
          rootRef.current.remove(c);
          disposeObject3D(c);
        }
      }
      idToMesh.current.clear();
      if (sceneRef.current) {
        while (sceneRef.current.children.length) {
          const c = sceneRef.current.children[0];
          sceneRef.current.remove(c);
          disposeObject3D(c);
        }
      }

      renderer.dispose();
      mount.removeChild(renderer.domElement);
    };
  }, []);

  // mesh / 场景坐标系：集合变化才重拉；FK 只同步 worldMatrix
  useEffect(() => {
    const content = contentRef.current;
    if (!content) return;
    const sig = geomSignature(objects, pointCloudRevision);
    let cancelled = false;

    const syncExisting = () => {
      const box = new THREE.Box3();
      for (const obj of objects) {
        const mesh = idToMesh.current.get(obj.id);
        if (!mesh) continue;
        applyObjectTransform(mesh, obj);
        mesh.visible = !!obj.visible;
        if (!isSceneCoordinateFrame(obj)) {
          applyMeshSelectionStyle(mesh, obj, obj.id === selectedId);
        }
        if (mesh.visible) box.expandByObject(mesh);
      }
      boxRef.current = box;
    };

    if (sig === geomSigRef.current && idToMesh.current.size > 0) {
      syncExisting();
      return;
    }

    (async () => {
      while (content.children.length) {
        const c = content.children[0];
        content.remove(c);
        disposeObject3D(c);
      }
      idToMesh.current.clear();
      const box = new THREE.Box3();
      const snapshot = objects.slice();
      for (const obj of snapshot) {
        if (cancelled) return;
        if (!obj.visible) continue;
        let mesh: THREE.Object3D | null = null;
        if (isSceneCoordinateFrame(obj)) {
          mesh = makeCoordinateFrameAxes(100);
          mesh.userData.backendId = obj.id;
          applyObjectTransform(mesh, obj);
        } else {
          if (!obj.hasGeometry) continue;
          if (obj.geometryKind === 1) {
            const pts = await loadPointCloudObject(obj);
            if (cancelled || !pts) continue;
            mesh = pts;
            applyObjectTransform(mesh, obj);
          } else {
            const soup = await fetchMeshSoup(obj.id);
            if (cancelled || !soup || soup.length < 9) continue;
            const selected = obj.id === selectedId;
            const geo = new THREE.BufferGeometry();
            geo.setAttribute("position", new THREE.BufferAttribute(soup, 3));
            geo.computeVertexNormals();
            mesh = new THREE.Mesh(geo, createMeshMaterial(colorFromObject(obj, selected), selected));
            mesh.userData.backendId = obj.id;
            applyObjectTransform(mesh, obj);
          }
        }
        if (!mesh) continue;
        content.add(mesh);
        idToMesh.current.set(obj.id, mesh);
        mesh.updateMatrixWorld(true);
        box.expandByObject(mesh);
      }
      if (cancelled) return;
      geomSigRef.current = sig;
      boxRef.current = box;
      if (focusPendingRef.current && !box.isEmpty() && cameraRef.current && controlsRef.current) {
        focusPendingRef.current = false;
        const size = Math.max(box.getSize(new THREE.Vector3()).length(), 100);
        const center = box.getCenter(new THREE.Vector3());
        controlsRef.current.target.copy(center);
        cameraRef.current.position.copy(center.clone().add(new THREE.Vector3(size, size * 0.7, size)));
        controlsRef.current.update();
      }
    })();

    return () => {
      cancelled = true;
    };
  }, [objects, selectedId, pointCloudRevision]);

  // 末端 TCP / 用户坐标系叠加（对齐旧版 refreshFrameOverlays）
  useEffect(() => {
    const group = frameOverlayRef.current;
    if (!group) return;
    // 按住拖动时 TCP 由罗盘代理驱动，避免 FK 残差把轴拽离鼠标
    if (robotDragMode && transformRef.current?.dragging) return;
    let cancelled = false;
    (async () => {
      if (!activeRootId) {
        clearFrameOverlayGroup(group);
        return;
      }
      try {
        const r = await fetchFrameOverlays(activeRootId);
        if (cancelled) return;
        clearFrameOverlayGroup(group);
        if (!r.ok) return;
        for (const t of r.tools || []) {
          if (t.showInScene === false) continue;
          const wm = Array.isArray(t.worldMatrix) && t.worldMatrix.length >= 16 ? t.worldMatrix : undefined;
          addFrameAxisMarker(group, wm || t.positionMm, t.eulerDeg, !!t.active, "tool");
        }
        for (const u of r.users || []) {
          if (u.showInScene === false) continue;
          const wm = Array.isArray(u.worldMatrix) && u.worldMatrix.length >= 16 ? u.worldMatrix : undefined;
          addFrameAxisMarker(group, wm || u.positionMm, u.eulerDeg, !!u.active, "user");
        }
      } catch {
        /* ignore */
      }
    })();
    return () => {
      cancelled = true;
    };
  }, [activeRootId, objects, frames, robotDragMode]);

  // 选中变化时解析机器人锚点/法兰（整机 place / 末端 IK）
  useEffect(() => {
    let cancelled = false;
    if (!selectedId) {
      setRobotMeta({ isRobot: false });
      return;
    }
    void resolveRobot(selectedId).then((r) => {
      if (cancelled) return;
      if (r.ok && r.sceneRootBackendId && r.anchorBackendId) {
        setRobotMeta({
          isRobot: true,
          sceneRootBackendId: r.sceneRootBackendId,
          anchorBackendId: r.anchorBackendId,
          flangeBackendId: r.flangeBackendId || r.anchorBackendId,
        });
      } else {
        setRobotMeta({ isRobot: false });
      }
    });
    return () => {
      cancelled = true;
    };
  }, [selectedId]);

  // 挂 gizmo：机器人走 dragProxy；普通物体先解开 matrixAutoUpdate
  useEffect(() => {
    const transform = transformRef.current;
    const root = rootRef.current;
    const proxy = dragProxyRef.current;
    if (!transform || !root || !proxy) return;

    const detach = () => {
      transform.detach();
      transform.enabled = false;
      transform.getHelper().visible = false;
    };

    if (transform.dragging) return;

    if (robotDragMode) {
      let cancelled = false;
      void (async () => {
        let flangeId = robotMeta.flangeBackendId;
        let sceneRoot = robotMeta.sceneRootBackendId || activeRootId || undefined;
        const resolveId = selectedId || activeRootId;
        if ((!flangeId || !sceneRoot) && resolveId) {
          const r = await resolveRobot(resolveId);
          if (cancelled) return;
          if (r.ok) {
            flangeId = flangeId || r.flangeBackendId || r.anchorBackendId;
            sceneRoot = sceneRoot || r.sceneRootBackendId;
          }
        }
        if (cancelled || !flangeId) {
          detach();
          return;
        }
        // IK 目标是工具 TCP：贴 tcp-pose，勿贴法兰 mesh（visual 偏移会漂到原点/错位）
        let snapped = false;
        if (sceneRoot) {
          const pose = await tcpPose(sceneRoot);
          if (cancelled) return;
          snapped = !!(pose.ok && snapProxyFromWorldMatrix(proxy, pose.worldMatrix));
        }
        if (!snapped) {
          const mesh = idToMesh.current.get(flangeId);
          if (!mesh) {
            detach();
            return;
          }
          snapProxyToMesh(root, proxy, mesh);
        }
        transform.enabled = true;
        transform.attach(proxy);
        const mode = gizmoCtxRef.current.gizmoTransformMode;
        transform.setMode(mode);
        transform.setSpace(gizmoCtxRef.current.gizmoSpace);
        transform.getHelper().visible = true;
        syncActiveToolOverlayFromProxy(frameOverlayRef.current, proxy);
      })();
      return () => {
        cancelled = true;
      };
    }

    if (interactMode !== "select" || !selectedId) {
      detach();
      return;
    }

    if (robotMeta.isRobot && robotMeta.anchorBackendId) {
      const mesh = idToMesh.current.get(robotMeta.anchorBackendId);
      if (mesh) {
        snapProxyToMesh(root, proxy, mesh);
      } else {
        const o = objects.find((x) => x.id === robotMeta.anchorBackendId);
        if (!snapProxyFromWorldMatrix(proxy, o?.worldMatrix)) {
          detach();
          return;
        }
      }
      transform.enabled = true;
      transform.attach(proxy);
      {
        const mode = gizmoCtxRef.current.gizmoTransformMode;
        transform.setMode(mode);
        transform.setSpace(gizmoCtxRef.current.gizmoSpace);
      }
      transform.getHelper().visible = true;
      return;
    }

    const mesh = idToMesh.current.get(selectedId);
    if (!mesh) {
      detach();
      return;
    }
    prepareMeshForGizmo(mesh);
    transform.enabled = true;
    transform.attach(mesh);
    {
      const mode = gizmoCtxRef.current.gizmoTransformMode;
      transform.setMode(mode);
      transform.setSpace(gizmoCtxRef.current.gizmoSpace);
    }
    transform.getHelper().visible = true;
  }, [selectedId, interactMode, robotDragMode, robotMeta, objects, activeRootId]);

  // 切换移动/旋转或物体系/世界系时只改罗盘，不重贴 TCP
  useEffect(() => {
    const transform = transformRef.current;
    const proxy = dragProxyRef.current;
    if (!transform?.object || transform.dragging) return;
    // 局部轴：旋转后切回移动，平移轴跟着 TCP 姿态走
    proxy?.updateMatrixWorld(true);
    transform.setMode(gizmoTransformMode);
    transform.setSpace(gizmoSpace);
  }, [gizmoTransformMode, gizmoSpace]);

  // G=移动 R=旋转（对齐旧版；状态栏/日志提示，无工具栏态）
  useEffect(() => {
    const onKey = (ev: KeyboardEvent) => {
      const t = ev.target as HTMLElement | null;
      if (t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA" || t.isContentEditable)) return;
      const transform = transformRef.current;
      if (!transform?.object || !transform.enabled) return;
      if (ev.key === "g" || ev.key === "G") {
        ev.preventDefault();
        setGizmoTransformMode("translate");
        setStatus("罗盘：移动（G）");
      } else if (ev.key === "r" || ev.key === "R") {
        ev.preventDefault();
        setGizmoTransformMode("rotate");
        setStatus("罗盘：旋转（R）");
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [setGizmoTransformMode, setStatus]);

  // place / tcp-ik / PATCH：对齐桌面 TCP 拖动（8ms 节流、追赶续推、拖动中不回折罗盘）
  useEffect(() => {
    const transform = transformRef.current;
    const controls = controlsRef.current;
    const root = rootRef.current;
    const proxy = dragProxyRef.current;
    if (!transform || !controls || !root || !proxy) return;

    let posePushPending = false;
    let posePushInFlight = false;
    let tcpIkContinue = false;
    let lastTcpIkAt = 0;
    let lastMeshRefreshAt = 0;
    let tcpIkTimer: ReturnType<typeof setTimeout> | null = null;
    const kTcpIkMinIntervalMs = 8;
    const kMeshRefreshMinIntervalMs = 50;
    const translateOriLock = new THREE.Quaternion();
    let translateOriLocked = false;
    let translateOriArming = false;

    const snapTcpProxy = async () => {
      const ctx = gizmoCtxRef.current;
      let sceneRoot = ctx.robotMeta.sceneRootBackendId || ctx.activeRootId || undefined;
      let flangeId = ctx.robotMeta.flangeBackendId;
      if ((!sceneRoot || !flangeId) && (ctx.selectedId || ctx.activeRootId)) {
        const r = await resolveRobot(ctx.selectedId || ctx.activeRootId || "");
        if (r.ok) {
          sceneRoot = sceneRoot || r.sceneRootBackendId;
          flangeId = flangeId || r.flangeBackendId || r.anchorBackendId;
        }
      }
      if (sceneRoot) {
        const pose = await tcpPose(sceneRoot);
        if (pose.ok && snapProxyFromWorldMatrix(proxy, pose.worldMatrix)) return;
      }
      if (flangeId) {
        const mesh = idToMesh.current.get(flangeId);
        if (mesh) snapProxyToMesh(root, proxy, mesh);
      }
    };

    const scheduleTcpIkFlush = (delayMs: number) => {
      if (tcpIkTimer != null) return;
      tcpIkTimer = setTimeout(() => {
        tcpIkTimer = null;
        posePushPending = true;
        void flush();
      }, Math.max(0, delayMs));
    };

    const pushOnce = async () => {
      const ctx = gizmoCtxRef.current;
      const meta = ctx.robotMeta;

      if (ctx.robotDragMode && transform.object === proxy) {
        let flangeId = meta.flangeBackendId;
        if (!flangeId) {
          const id = ctx.selectedId || ctx.activeRootId;
          if (id) {
            const r = await resolveRobot(id);
            if (r.ok) flangeId = r.flangeBackendId || r.anchorBackendId;
          }
        }
        if (!flangeId) return;
        const elapsed = performance.now() - lastTcpIkAt;
        if (elapsed < kTcpIkMinIntervalMs) {
          scheduleTcpIkFlush(kTcpIkMinIntervalMs - elapsed);
          return;
        }
        proxy.updateMatrix();
        // 平移锁 FK 姿态（开拖已 snap）；未就绪则等
        const translateOnly = ctx.gizmoTransformMode === "translate";
        if (translateOnly) {
          if (!translateOriLocked) {
            if (!translateOriArming) scheduleTcpIkFlush(16);
            return;
          }
          proxy.quaternion.copy(translateOriLock);
          proxy.updateMatrix();
        } else {
          translateOriLocked = false;
          translateOriArming = false;
        }
        const wm = localMatrixArray(proxy);
        lastTcpIkAt = performance.now();
        const r = await tcpIk({ flangeBackendId: flangeId, worldMatrix: wm, translateOnly });
        if (!r.ok) {
          ctx.setStatus(r.error || "末端 IK 失败", "err");
          tcpIkContinue = false;
          return;
        }
        // 示教落点跟罗盘真值，勿用 FK 欧拉（对齐桌面 m_lastTcpDragTarget）
        const snap = poseFromMatrix4Elements(wm);
        const joints = Array.isArray(r.jointAnglesRad) ? r.jointAnglesRad.map(Number) : [];
        ctx.setRobotDragTeachPose({
          ...snap,
          jointRadCsv: joints.length ? joints.map((v) => v.toFixed(6)).join(",") : undefined,
        });
        // 拖动中罗盘跟鼠标目标；松手后再贴 FK，勿用残差拽回罗盘
        const now = performance.now();
        if (now - lastMeshRefreshAt >= kMeshRefreshMinIntervalMs || !transform.dragging) {
          lastMeshRefreshAt = now;
          await ctx.refreshObjects();
        }
        // 按住拖动时持续追赶；松手由 onDraggingChanged 排空 incomplete
        tcpIkContinue = !!r.incomplete;
        if (tcpIkContinue && transform.dragging) scheduleTcpIkFlush(kTcpIkMinIntervalMs);
        return;
      }

      if (
        ctx.interactMode === "select" &&
        meta.isRobot &&
        meta.anchorBackendId &&
        transform.object === proxy
      ) {
        proxy.updateMatrix();
        const r = await placeRobot({
          anchorBackendId: meta.anchorBackendId,
          worldMatrix: localMatrixArray(proxy),
        });
        if (!r.ok) {
          ctx.setStatus(r.error || "整机放置失败", "err");
          return;
        }
        await ctx.refreshObjects();
        if (!transform.dragging && meta.anchorBackendId) {
          const mesh = idToMesh.current.get(meta.anchorBackendId);
          if (mesh) {
            snapProxyToMesh(root, proxy, mesh);
            transform.enabled = true;
            transform.attach(proxy);
            transform.getHelper().visible = true;
          }
        }
        return;
      }

      const obj = transform.object;
      if (!obj || !ctx.selectedId || obj === proxy) return;
      const e = new THREE.Euler().setFromQuaternion(obj.quaternion, "ZYX");
      const r = await patchObject(ctx.selectedId, {
        pose: {
          positionMm: [obj.position.x, obj.position.y, obj.position.z],
          eulerDeg: [
            THREE.MathUtils.radToDeg(e.x),
            THREE.MathUtils.radToDeg(e.y),
            THREE.MathUtils.radToDeg(e.z),
          ],
        },
      });
      if (r.ok) await ctx.refreshObjects();
    };

    const flush = async () => {
      if (posePushInFlight || !posePushPending) return;
      posePushInFlight = true;
      try {
        while (posePushPending) {
          posePushPending = false;
          await pushOnce();
        }
      } finally {
        posePushInFlight = false;
        if (posePushPending) void flush();
      }
    };

    const onDraggingChanged = (ev: { value?: boolean }) => {
      const dragging = !!ev.value;
      controls.enabled = !dragging;
      if (dragging) {
        translateOriLocked = false;
        translateOriArming = false;
        if (gizmoCtxRef.current.robotDragMode) {
          if (gizmoCtxRef.current.gizmoTransformMode === "translate") {
            // 先贴 FK 再锁姿，与后端 tcpDragOri 同源
            translateOriArming = true;
            void (async () => {
              await snapTcpProxy();
              translateOriLock.copy(proxy.quaternion);
              translateOriLocked = true;
              translateOriArming = false;
              syncActiveToolOverlayFromProxy(frameOverlayRef.current, proxy);
              posePushPending = true;
              void flush();
            })();
          } else {
            syncActiveToolOverlayFromProxy(frameOverlayRef.current, proxy);
          }
        }
        return;
      }
      translateOriArming = false;
      const keepTranslateOri =
        gizmoCtxRef.current.robotDragMode && gizmoCtxRef.current.gizmoTransformMode === "translate"
          ? translateOriLock.clone()
          : null;
      translateOriLocked = false;
      void (async () => {
        posePushPending = true;
        await flush();
        for (let i = 0; i < 60 && tcpIkContinue; ++i) {
          posePushPending = true;
          await flush();
        }
        const ctx = gizmoCtxRef.current;
        if (ctx.robotDragMode) {
          await ctx.refreshObjects();
          await snapTcpProxy();
          if (keepTranslateOri) {
            proxy.quaternion.copy(keepTranslateOri);
            proxy.updateMatrix();
          }
          transform.enabled = true;
          transform.attach(proxy);
          transform.getHelper().visible = true;
        } else if (ctx.interactMode === "select" && ctx.robotMeta.isRobot && ctx.robotMeta.anchorBackendId) {
          const mesh = idToMesh.current.get(ctx.robotMeta.anchorBackendId);
          if (mesh) snapProxyToMesh(root, proxy, mesh);
          transform.enabled = true;
          transform.attach(proxy);
          transform.getHelper().visible = true;
        }
      })();
    };

    const onObjectChange = () => {
      const ctx = gizmoCtxRef.current;
      if (ctx.robotDragMode) {
        if (ctx.gizmoTransformMode === "translate" && translateOriLocked) {
          proxy.quaternion.copy(translateOriLock);
          proxy.updateMatrix();
        }
        syncActiveToolOverlayFromProxy(frameOverlayRef.current, proxy);
        posePushPending = true;
        void flush();
        return;
      }
      if (ctx.interactMode === "select" && transform.object) {
        posePushPending = true;
        void flush();
      }
    };

    transform.addEventListener("dragging-changed", onDraggingChanged as never);
    transform.addEventListener("objectChange", onObjectChange);
    return () => {
      if (tcpIkTimer != null) clearTimeout(tcpIkTimer);
      transform.removeEventListener("dragging-changed", onDraggingChanged as never);
      transform.removeEventListener("objectChange", onObjectChange);
    };
  }, []);

  useEffect(() => {
    if (focusRequest <= 0) return;
    const box = boxRef.current;
    if (!box.isEmpty() && cameraRef.current && controlsRef.current) {
      const size = Math.max(box.getSize(new THREE.Vector3()).length(), 100);
      const center = box.getCenter(new THREE.Vector3());
      controlsRef.current.target.copy(center);
      cameraRef.current.position.copy(center.clone().add(new THREE.Vector3(size, size * 0.7, size)));
      controlsRef.current.update();
      focusPendingRef.current = false;
    } else {
      // 几何尚在异步加载（点云 preview）
      focusPendingRef.current = true;
    }
  }, [focusRequest]);

  // 点选 + 轨迹拾取
  useEffect(() => {
    const renderer = rendererRef.current;
    const camera = cameraRef.current;
    const root = rootRef.current;
    if (!renderer || !camera || !root) return;

    const raycaster = new THREE.Raycaster();
    const pointer = new THREE.Vector2();

    const worldRay = (ev: MouseEvent) => {
      const rect = renderer.domElement.getBoundingClientRect();
      pointer.x = ((ev.clientX - rect.left) / rect.width) * 2 - 1;
      pointer.y = -((ev.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(pointer, camera);
      // 相机射线经 zUpToYUp 逆变换到模型 Z-up
      const originY = raycaster.ray.origin.clone();
      const dirY = raycaster.ray.direction.clone();
      const inv = zUpToYUp.clone().invert();
      originY.applyMatrix4(inv);
      dirY.transformDirection(inv);
      const hits = raycaster.intersectObjects([...idToMesh.current.values()], true);
      let hitBackendId = "";
      let hitPointWorldMm: number[] | null = null;
      let hitNormalWorld: number[] | null = null;
      if (hits[0]) {
        let o: THREE.Object3D | null = hits[0].object;
        while (o && !o.userData.backendId) o = o.parent;
        hitBackendId = o?.userData.backendId ? String(o.userData.backendId) : "";
        const p = hits[0].point.clone().applyMatrix4(inv);
        hitPointWorldMm = [p.x, p.y, p.z];
        if (hits[0].face) {
          const n = hits[0].face.normal.clone().transformDirection(hits[0].object.matrixWorld).transformDirection(inv);
          hitNormalWorld = [n.x, n.y, n.z];
        }
      }
      return {
        originMm: [originY.x, originY.y, originY.z],
        dir: [dirY.x, dirY.y, dirY.z],
        hitBackendId,
        hitPointWorldMm,
        hitNormalWorld,
      };
    };

    const onClick = async (ev: MouseEvent) => {
      if (polylinePickActive) {
        const rect = renderer.domElement.getBoundingClientRect();
        if (ev.button === 2) {
          if (polylineScreenXy.length >= 6) {
            setStatus(`多边形 ${polylineScreenXy.length / 2} 点，请在面板点「应用多边形」完成裁剪`);
          } else {
            setStatus("多边形至少需要 3 个点", "warn");
          }
          return;
        }
        addPolylinePoint(ev.clientX - rect.left, ev.clientY - rect.top);
        setStatus(`多边形顶点 ${polylineScreenXy.length / 2 + 1}`);
        return;
      }
      const ray = worldRay(ev);
      const mateSlot = mateFacePickSlotRef.current;
      if (mateSlot != null) {
        const wp = ray.hitBackendId;
        if (!wp || !ray.hitPointWorldMm) {
          setStatus("配合拾取：未命中 B-rep", "warn");
          return;
        }
        const body = {
          mode: "face" as const,
          workpieceBackendId: wp,
          originMm: ray.originMm,
          dir: ray.dir,
          hitPointWorldMm: ray.hitPointWorldMm,
          hitNormalWorld: ray.hitNormalWorld,
        };
        const r = await pickHover(body);
        if (!r.ok) {
          setStatus(String(r.error || "配合面拾取失败"), "err");
          return;
        }
        const faceIndex = Number(r.faceIndex ?? -1);
        const hitArr = Array.isArray(r.hitPointWorldMm)
          ? (r.hitPointWorldMm as number[])
          : ray.hitPointWorldMm;
        if (faceIndex < 0) {
          setStatus("需要有效 B-rep 面", "err");
          return;
        }
        setMateFacePickSlot(null);
        window.dispatchEvent(
          new CustomEvent("cloudsim-mate-face", {
            detail: {
              slot: mateSlot,
              backendId: String(r.workpieceBackendId || wp),
              faceIndex,
              pickWorldMm: hitArr.slice(0, 3),
              soupWorldMm: r.soupWorldMm,
            },
          }),
        );
        setStatus(mateSlot === 0 ? "已拾取固定面" : "已拾取动件面");
        return;
      }
      if (pickMode && featureEditActive) {
        const wp = ray.hitBackendId || workpieceId;
        if (wp) setWorkpieceId(wp);
        if (!wp || !ray.hitPointWorldMm) {
          setStatus("未命中工件", "warn");
          return;
        }
        const body = {
          mode: pickMode,
          workpieceBackendId: wp,
          originMm: ray.originMm,
          dir: ray.dir,
          hitPointWorldMm: ray.hitPointWorldMm,
          hitNormalWorld: ray.hitNormalWorld,
        };
        const r = await pickHover(body);
        if (!r.ok) {
          setStatus(String(r.error || "拾取失败"), "err");
          return;
        }
        hoverPickSeqRef.current += 1;
        clearPickOverlayGroup(overlayRef.current);
        setStatus(`已拾取 ${pickMode}`);
        // 提交后退出拾取；effect 再清一次，防异步 hover 回写
        window.dispatchEvent(new CustomEvent("cloudsim-pick-commit", { detail: { ...body, result: r } }));
        return;
      }
      if (waypointPickModeRef.current && instrMarkersRef.current && cameraRef.current) {
        const hit = tryPickInstrWaypointAt(
          instrMarkersRef.current,
          cameraRef.current,
          ev.clientX,
          ev.clientY,
          renderer.domElement,
        );
        if (hit) {
          setSelectedInstrId(hit.instructionId, { preferVia: hit.isArcVia });
          window.dispatchEvent(new CustomEvent("cloudsim-focus-props"));
          setStatus(hit.isArcVia ? `已选 ARC via ${hit.instructionId}` : `已选路点 ${hit.instructionId}`);
          return;
        }
        setStatus("未命中路点", "warn");
        return;
      }
      if (interactMode === "select" && ray.hitBackendId) {
        await selectObject(ray.hitBackendId);
      }
    };

    let hoverTimer: number | null = null;
    const clearHl = () => {
      window.dispatchEvent(new CustomEvent("cloudsim-pick-highlight", { detail: { clear: true } }));
    };
    const onMove = (ev: MouseEvent) => {
      if (waypointPickModeRef.current && instrMarkersRef.current && cameraRef.current) {
        const hit = tryPickInstrWaypointAt(
          instrMarkersRef.current,
          cameraRef.current,
          ev.clientX,
          ev.clientY,
          renderer.domElement,
        );
        updateWaypointPickHover(instrMarkersRef.current, cameraRef.current, hit);
        renderer.domElement.style.cursor = hit ? "pointer" : "crosshair";
        return;
      }
      if (mateFacePickSlotRef.current != null) {
        renderer.domElement.style.cursor = "crosshair";
        if (hoverTimer) window.clearTimeout(hoverTimer);
        hoverTimer = window.setTimeout(async () => {
          const seq = ++hoverPickSeqRef.current;
          const ray = worldRay(ev);
          const wp = ray.hitBackendId;
          if (!wp || !ray.hitPointWorldMm) {
            if (seq === hoverPickSeqRef.current) clearHl();
            return;
          }
          const r = await pickHover({
            mode: "face",
            workpieceBackendId: wp,
            originMm: ray.originMm,
            dir: ray.dir,
            hitPointWorldMm: ray.hitPointWorldMm,
            hitNormalWorld: ray.hitNormalWorld,
          });
          if (seq !== hoverPickSeqRef.current) return;
          if (!r.ok) {
            clearHl();
            return;
          }
          window.dispatchEvent(
            new CustomEvent("cloudsim-pick-highlight", {
              detail: { clear: false, soupWorldMm: r.soupWorldMm, mode: "face" },
            }),
          );
        }, 80);
        return;
      }
      if (!pickMode || !featureEditActive) return;
      if (hoverTimer) window.clearTimeout(hoverTimer);
      hoverTimer = window.setTimeout(async () => {
        const seq = ++hoverPickSeqRef.current;
        const ray = worldRay(ev);
        const wp = ray.hitBackendId || workpieceId;
        if (!wp || !ray.hitPointWorldMm) {
          if (seq === hoverPickSeqRef.current) clearHl();
          return;
        }
        const r = await pickHover({
          mode: pickMode,
          workpieceBackendId: wp,
          originMm: ray.originMm,
          dir: ray.dir,
          hitPointWorldMm: ray.hitPointWorldMm,
          hitNormalWorld: ray.hitNormalWorld,
        });
        if (seq !== hoverPickSeqRef.current) return;
        if (r.ok) {
          window.dispatchEvent(
            new CustomEvent("cloudsim-pick-highlight", {
              detail: { polys: r.polylinesWorld || [], soup: r.soupWorldMm },
            }),
          );
        } else {
          clearHl();
        }
      }, 80);
    };
    const onLeave = () => {
      if (hoverTimer) window.clearTimeout(hoverTimer);
      hoverPickSeqRef.current += 1;
      clearWaypointPickHover(instrMarkersRef.current);
      if (waypointPickModeRef.current) renderer.domElement.style.cursor = "crosshair";
      if (pickMode) clearHl();
    };

    renderer.domElement.addEventListener("click", onClick);
    renderer.domElement.addEventListener("contextmenu", (e) => {
      if (polylinePickActive) e.preventDefault();
    });
    renderer.domElement.addEventListener("pointermove", onMove);
    renderer.domElement.addEventListener("pointerleave", onLeave);
    return () => {
      renderer.domElement.removeEventListener("click", onClick);
      renderer.domElement.removeEventListener("pointermove", onMove);
      renderer.domElement.removeEventListener("pointerleave", onLeave);
      if (hoverTimer) window.clearTimeout(hoverTimer);
      hoverPickSeqRef.current += 1;
    };
  }, [
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
  ]);

  // 多边形拾取屏幕折线叠加
  useEffect(() => {
    const g = overlayRef.current;
    if (!g) return;
    if (!polylinePickActive || polylineScreenXy.length < 4) {
      if (!pickMode) clearPickOverlayGroup(g);
      return;
    }
    clearPickOverlayGroup(g);
    const pts: THREE.Vector3[] = [];
    for (let i = 0; i + 1 < polylineScreenXy.length; i += 2) {
      pts.push(new THREE.Vector3(polylineScreenXy[i], polylineScreenXy[i + 1], 0));
    }
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    g.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color: 0xffeb3b })));
  }, [polylinePickActive, polylineScreenXy, pickMode]);

  // 点云多边形裁剪：暴露视口 MVP（列主序 16）
  useEffect(() => {
    (window as unknown as { cloudsimViewportPick?: () => { mvpMatrix: number[]; viewportWidth: number; viewportHeight: number } | null }).cloudsimViewportPick = () => {
      const camera = cameraRef.current;
      const renderer = rendererRef.current;
      if (!camera || !renderer) return null;
      camera.updateMatrixWorld();
      const mvp = new THREE.Matrix4().multiplyMatrices(camera.projectionMatrix, camera.matrixWorldInverse);
      return {
        mvpMatrix: Array.from(mvp.elements),
        viewportWidth: renderer.domElement.clientWidth,
        viewportHeight: renderer.domElement.clientHeight,
      };
    };
    return () => {
      delete (window as unknown as { cloudsimViewportPick?: unknown }).cloudsimViewportPick;
    };
  }, []);

  // Raw 轨迹预览（离散后点/线）；与指令路点互斥（对齐旧版）
  useEffect(() => {
    const syncMarkers = () => {
      const g = instrMarkersRef.current;
      if (!g) return;
      refreshInstrMarkers(g, instrStepsRef.current, selectedInstrRef.current, {
        hideForRawPreview: rawPreviewActiveRef.current,
        playing: playingRef.current,
        preferVia: selectedPreferViaRef.current,
      });
    };
    const onRaw = (ev: Event) => {
      const g = rawPreviewRef.current;
      if (!g) return;
      const d = (ev as CustomEvent).detail as {
        preview?: RawPreviewPayload | null;
        axisOpts?: PreviewAxisOpts;
      };
      const axisOpts = d.axisOpts || { x: true, y: true, z: true, interval: 0 };
      if (!d.preview) {
        clearRawPreviewGroup(g);
        rawPreviewActiveRef.current = false;
        syncMarkers();
        return;
      }
      if (playingRef.current) {
        clearRawPreviewGroup(g);
        rawPreviewActiveRef.current = false;
      } else {
        applyRawPreviewToGroup(g, d.preview, axisOpts);
        rawPreviewActiveRef.current = !!(d.preview.pointsMm && d.preview.pointsMm.length);
      }
      syncMarkers();
    };
    window.addEventListener("cloudsim-raw-preview", onRaw);
    return () => window.removeEventListener("cloudsim-raw-preview", onRaw);
  }, []);

  // 程序路点 / 选中变化时刷新 3D 标记
  useEffect(() => {
    const g = instrMarkersRef.current;
    if (!g) return;
    if (playing) {
      const raw = rawPreviewRef.current;
      if (raw) clearRawPreviewGroup(raw);
      rawPreviewActiveRef.current = false;
    }
    refreshInstrMarkers(g, activeProgram?.instructions || [], selectedInstrId, {
      hideForRawPreview: rawPreviewActiveRef.current,
      playing,
      preferVia: selectedInstrPreferVia,
    });
  }, [activeProgram?.instructions, selectedInstrId, selectedInstrPreferVia, playing, activeRootId]);

  // 叠加层事件
  useEffect(() => {
    const onHl = (ev: Event) => {
      const d = (ev as CustomEvent).detail as {
        clear?: boolean;
        polys?: number[][][];
        soup?: number[] | Float32Array;
      };
      const g = overlayRef.current;
      if (!g) return;
      clearPickOverlayGroup(g);
      if (d?.clear) return;
      const mat = new THREE.LineBasicMaterial({ color: 0xff9800 });
      for (const poly of d.polys || []) {
        // Host 可能给扁平数组或点对象数组
        let points: THREE.Vector3[] = [];
        if (Array.isArray(poly) && poly.length && typeof poly[0] === "number") {
          const flat = poly as unknown as number[];
          for (let i = 0; i + 2 < flat.length; i += 3) {
            points.push(new THREE.Vector3(flat[i], flat[i + 1], flat[i + 2]));
          }
        } else if (Array.isArray(poly)) {
          points = (poly as number[][]).map((p) => new THREE.Vector3(p[0], p[1], p[2]));
        }
        if (points.length >= 2) {
          g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), mat));
        }
      }
      if (d.soup) {
        const arr = d.soup instanceof Float32Array ? d.soup : new Float32Array(d.soup);
        if (arr.length >= 9) {
          const geo = new THREE.BufferGeometry();
          geo.setAttribute("position", new THREE.BufferAttribute(arr, 3));
          geo.computeVertexNormals();
          g.add(
            new THREE.Mesh(
              geo,
              new THREE.MeshBasicMaterial({
                color: 0xffcc33,
                transparent: true,
                opacity: 0.55,
                side: THREE.DoubleSide,
                depthWrite: false,
              }),
            ),
          );
        }
      }
    };
    window.addEventListener("cloudsim-pick-highlight", onHl);
    return () => window.removeEventListener("cloudsim-pick-highlight", onHl);
  }, []);

  return <div className="scene" ref={mountRef} />;
});

export default SceneViewport;
