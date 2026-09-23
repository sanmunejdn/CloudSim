/// @file useViewportInteraction.ts
/// @brief 点选、配合面、轨迹拾取、多边形、路点 ESC、MVP 暴露

import { useEffect, type MutableRefObject } from "react";
import * as THREE from "three";
import { pickHover } from "../../api/trajectory";
import { clearPickOverlayGroup } from "../meshLoad";
import {
  tryPickInstrWaypointAt,
  updateWaypointPickHover,
  clearWaypointPickHover,
} from "../instrMarkers";
import { uiEvents, UI_EVT, type PickCommitDetail } from "../../ui/uiEvents";
import { zUpToYUp, type ViewportRefs } from "./types";

export function useViewportInteraction(
  refs: ViewportRefs,
  opts: {
    interactMode: "view" | "select";
    pickMode: "edge" | "face" | null;
    featureEditActive: boolean;
    workpieceId: string;
    setWorkpieceId: (id: string) => void;
    selectObject: (id: string | null) => Promise<void>;
    setStatus: (m: string, k?: "info" | "err" | "warn") => void;
    setSelectedInstrId: (id: string | null, o?: { preferVia?: boolean }) => void;
    polylinePickActive: boolean;
    addPolylinePoint: (x: number, y: number) => void;
    polylineScreenXy: number[];
    mateFacePickSlot: 0 | 1 | null;
    setMateFacePickSlot: (s: 0 | 1 | null) => void;
    waypointPickMode: boolean;
    setWaypointPickMode: (v: boolean) => void;
    mateFacePickSlotRef: MutableRefObject<0 | 1 | null>;
    waypointPickModeRef: MutableRefObject<boolean>;
  },
) {
  const {
    rendererRef,
    cameraRef,
    rootRef,
    overlayRef,
    instrMarkersRef,
    idToMesh,
    hoverPickSeqRef,
  } = refs;

  useEffect(() => {
    if (opts.pickMode && opts.featureEditActive) return;
    clearPickOverlayGroup(overlayRef.current);
  }, [opts.pickMode, opts.featureEditActive, overlayRef]);

  useEffect(() => {
    if (opts.interactMode === "select" && opts.waypointPickMode) opts.setWaypointPickMode(false);
  }, [opts.interactMode, opts.waypointPickMode, opts.setWaypointPickMode]);

  useEffect(() => {
    if (!opts.waypointPickMode) clearWaypointPickHover(instrMarkersRef.current);
  }, [opts.waypointPickMode, instrMarkersRef]);

  useEffect(() => {
    if (!opts.waypointPickMode) return;
    const onKey = (ev: KeyboardEvent) => {
      if (ev.key === "Escape") {
        opts.setWaypointPickMode(false);
        clearWaypointPickHover(instrMarkersRef.current);
        opts.setStatus("已退出路点拾取");
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [opts.waypointPickMode, opts.setWaypointPickMode, opts.setStatus, instrMarkersRef]);

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
          const n = hits[0].face.normal
            .clone()
            .transformDirection(hits[0].object.matrixWorld)
            .transformDirection(inv);
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
      if (opts.polylinePickActive) {
        const rect = renderer.domElement.getBoundingClientRect();
        if (ev.button === 2) {
          if (opts.polylineScreenXy.length >= 6) {
            opts.setStatus(`多边形 ${opts.polylineScreenXy.length / 2} 点，请在面板点「应用多边形」完成裁剪`);
          } else {
            opts.setStatus("多边形至少需要 3 个点", "warn");
          }
          return;
        }
        opts.addPolylinePoint(ev.clientX - rect.left, ev.clientY - rect.top);
        opts.setStatus(`多边形顶点 ${opts.polylineScreenXy.length / 2 + 1}`);
        return;
      }
      const ray = worldRay(ev);
      const mateSlot = opts.mateFacePickSlotRef.current;
      if (mateSlot != null) {
        const wp = ray.hitBackendId;
        if (!wp || !ray.hitPointWorldMm) {
          opts.setStatus("配合拾取：未命中 B-rep", "warn");
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
          opts.setStatus(String(r.error || "配合面拾取失败"), "err");
          return;
        }
        const faceIndex = Number(r.faceIndex ?? -1);
        const hitArr = Array.isArray(r.hitPointWorldMm)
          ? (r.hitPointWorldMm as number[])
          : ray.hitPointWorldMm;
        if (faceIndex < 0) {
          opts.setStatus("需要有效 B-rep 面", "err");
          return;
        }
        opts.setMateFacePickSlot(null);
        uiEvents.emit(UI_EVT.mateFace, {
          slot: mateSlot,
          backendId: String(r.workpieceBackendId || wp),
          faceIndex,
          pickWorldMm: hitArr.slice(0, 3),
          soupWorldMm: r.soupWorldMm,
        });
        opts.setStatus(mateSlot === 0 ? "已拾取固定面" : "已拾取动件面");
        return;
      }
      if (opts.pickMode && opts.featureEditActive) {
        const wp = ray.hitBackendId || opts.workpieceId;
        if (wp) opts.setWorkpieceId(wp);
        if (!wp || !ray.hitPointWorldMm) {
          opts.setStatus("未命中工件", "warn");
          return;
        }
        const body = {
          mode: opts.pickMode,
          workpieceBackendId: wp,
          originMm: ray.originMm,
          dir: ray.dir,
          hitPointWorldMm: ray.hitPointWorldMm,
          hitNormalWorld: ray.hitNormalWorld,
        };
        const r = await pickHover(body);
        if (!r.ok) {
          opts.setStatus(String(r.error || "拾取失败"), "err");
          return;
        }
        hoverPickSeqRef.current += 1;
        clearPickOverlayGroup(overlayRef.current);
        opts.setStatus(`已拾取 ${opts.pickMode}`);
        uiEvents.emit(UI_EVT.pickCommit, { ...body, result: r } as PickCommitDetail);
        return;
      }
      if (opts.waypointPickModeRef.current && instrMarkersRef.current && cameraRef.current) {
        const hit = tryPickInstrWaypointAt(
          instrMarkersRef.current,
          cameraRef.current,
          ev.clientX,
          ev.clientY,
          renderer.domElement,
        );
        if (hit) {
          opts.setSelectedInstrId(hit.instructionId, { preferVia: hit.isArcVia });
          uiEvents.emit(UI_EVT.focusProps, undefined);
          opts.setStatus(hit.isArcVia ? `已选 ARC via ${hit.instructionId}` : `已选路点 ${hit.instructionId}`);
          return;
        }
        opts.setStatus("未命中路点", "warn");
        return;
      }
      if (opts.interactMode === "select" && ray.hitBackendId) {
        await opts.selectObject(ray.hitBackendId);
      }
    };

    let hoverTimer: number | null = null;
    const clearHl = () => {
      uiEvents.emit(UI_EVT.pickHighlight, { clear: true });
    };
    const onMove = (ev: MouseEvent) => {
      if (opts.waypointPickModeRef.current && instrMarkersRef.current && cameraRef.current) {
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
      if (opts.mateFacePickSlotRef.current != null) {
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
          uiEvents.emit(UI_EVT.pickHighlight, {
            clear: false,
            soup: r.soupWorldMm as number[] | undefined,
          });
        }, 80);
        return;
      }
      if (!opts.pickMode || !opts.featureEditActive) return;
      if (hoverTimer) window.clearTimeout(hoverTimer);
      hoverTimer = window.setTimeout(async () => {
        const seq = ++hoverPickSeqRef.current;
        const ray = worldRay(ev);
        const wp = ray.hitBackendId || opts.workpieceId;
        if (!wp || !ray.hitPointWorldMm) {
          if (seq === hoverPickSeqRef.current) clearHl();
          return;
        }
        const r = await pickHover({
          mode: opts.pickMode,
          workpieceBackendId: wp,
          originMm: ray.originMm,
          dir: ray.dir,
          hitPointWorldMm: ray.hitPointWorldMm,
          hitNormalWorld: ray.hitNormalWorld,
        });
        if (seq !== hoverPickSeqRef.current) return;
        if (r.ok) {
          uiEvents.emit(UI_EVT.pickHighlight, {
            polys: (r.polylinesWorld || []) as number[][][],
            soup: r.soupWorldMm as number[] | undefined,
          });
        } else {
          clearHl();
        }
      }, 80);
    };
    const onLeave = () => {
      if (hoverTimer) window.clearTimeout(hoverTimer);
      hoverPickSeqRef.current += 1;
      clearWaypointPickHover(instrMarkersRef.current);
      if (opts.waypointPickModeRef.current) renderer.domElement.style.cursor = "crosshair";
      if (opts.pickMode) clearHl();
    };

    renderer.domElement.addEventListener("click", onClick);
    renderer.domElement.addEventListener("contextmenu", (e) => {
      if (opts.polylinePickActive) e.preventDefault();
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
    opts.interactMode,
    opts.pickMode,
    opts.featureEditActive,
    opts.workpieceId,
    opts.setWorkpieceId,
    opts.selectObject,
    opts.setStatus,
    opts.setSelectedInstrId,
    opts.polylinePickActive,
    opts.addPolylinePoint,
    opts.polylineScreenXy,
    opts.mateFacePickSlot,
    opts.setMateFacePickSlot,
    opts.mateFacePickSlotRef,
    opts.waypointPickModeRef,
    rendererRef,
    cameraRef,
    rootRef,
    overlayRef,
    instrMarkersRef,
    idToMesh,
    hoverPickSeqRef,
  ]);

  useEffect(() => {
    const g = overlayRef.current;
    if (!g) return;
    if (!opts.polylinePickActive || opts.polylineScreenXy.length < 4) {
      if (!opts.pickMode) clearPickOverlayGroup(g);
      return;
    }
    clearPickOverlayGroup(g);
    const pts: THREE.Vector3[] = [];
    for (let i = 0; i + 1 < opts.polylineScreenXy.length; i += 2) {
      pts.push(new THREE.Vector3(opts.polylineScreenXy[i], opts.polylineScreenXy[i + 1], 0));
    }
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    g.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color: 0xffeb3b })));
  }, [opts.polylinePickActive, opts.polylineScreenXy, opts.pickMode, overlayRef]);

  useEffect(() => {
    (window as unknown as { cloudsimViewportPick?: () => { mvpMatrix: number[]; viewportWidth: number; viewportHeight: number } | null }).cloudsimViewportPick =
      () => {
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
  }, [cameraRef, rendererRef]);
}
