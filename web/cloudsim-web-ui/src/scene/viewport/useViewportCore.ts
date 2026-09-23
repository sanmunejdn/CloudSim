/// @file useViewportCore.ts
/// @brief Three 场景挂载、渲染循环与整体 dispose

import { useEffect } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { TransformControls } from "three/examples/jsm/controls/TransformControls.js";
import { applyWorldViewDirection, createViewCubeHud, createWorldAxesHud } from "../viewOrientationHud";
import { drainGroup, VIEW_BG } from "../meshLoad";
import { zUpToYUp, type ViewportRefs } from "./types";

export function useViewportCore(refs: ViewportRefs) {
  const {
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
    viewCubeHudRef,
    axesHudRef,
  } = refs;

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

      drainGroup(contentRef.current);
      drainGroup(overlayRef.current);
      drainGroup(frameOverlayRef.current);
      drainGroup(rawPreviewRef.current);
      drainGroup(instrMarkersRef.current);
      drainGroup(rootRef.current);
      idToMesh.current.clear();
      drainGroup(sceneRef.current);

      renderer.dispose();
      mount.removeChild(renderer.domElement);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps -- 仅挂载一次
  }, []);
}
