/// @file useSceneOverlays.ts
/// @brief 坐标系叠加、Raw 预览、指令路点、拾取高亮、聚焦

import { useEffect, type MutableRefObject } from "react";
import * as THREE from "three";
import { fetchFrameOverlays, type Instruction } from "../../api";
import {
  addFrameAxisMarker,
  clearFrameOverlayGroup,
} from "../frameAxes";
import { applyRawPreviewToGroup, clearRawPreviewGroup } from "../rawPreview";
import { refreshInstrMarkers } from "../instrMarkers";
import { clearPickOverlayGroup } from "../meshLoad";
import { uiEvents, UI_EVT, type PickHighlightDetail, type RawPreviewDetail } from "../../ui/uiEvents";
import type { ViewportRefs } from "./types";

export function useSceneOverlays(
  refs: ViewportRefs,
  opts: {
    activeRootId: string | null;
    objectsRevisionKey: unknown;
    frames: unknown;
    robotDragMode: boolean;
    focusRequest: number;
    activeProgramInstructions: Instruction[] | undefined;
    selectedInstrId: string | null;
    selectedInstrPreferVia: boolean;
    playing: boolean;
    instrStepsRef: MutableRefObject<Instruction[]>;
    selectedInstrRef: MutableRefObject<string | null>;
    selectedPreferViaRef: MutableRefObject<boolean>;
    playingRef: MutableRefObject<boolean>;
  },
) {
  const {
    frameOverlayRef,
    transformRef,
    cameraRef,
    controlsRef,
    boxRef,
    focusPendingRef,
    rawPreviewRef,
    instrMarkersRef,
    overlayRef,
    rawPreviewActiveRef,
  } = refs;

  // 末端 TCP / 用户坐标系叠加
  useEffect(() => {
    const group = frameOverlayRef.current;
    if (!group) return;
    if (opts.robotDragMode && transformRef.current?.dragging) return;
    let cancelled = false;
    (async () => {
      if (!opts.activeRootId) {
        clearFrameOverlayGroup(group);
        return;
      }
      try {
        const r = await fetchFrameOverlays(opts.activeRootId);
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
  }, [opts.activeRootId, opts.objectsRevisionKey, opts.frames, opts.robotDragMode, frameOverlayRef, transformRef]);

  useEffect(() => {
    if (opts.focusRequest <= 0) return;
    const box = boxRef.current;
    if (!box.isEmpty() && cameraRef.current && controlsRef.current) {
      const size = Math.max(box.getSize(new THREE.Vector3()).length(), 100);
      const center = box.getCenter(new THREE.Vector3());
      controlsRef.current.target.copy(center);
      cameraRef.current.position.copy(center.clone().add(new THREE.Vector3(size, size * 0.7, size)));
      controlsRef.current.update();
      focusPendingRef.current = false;
    } else {
      focusPendingRef.current = true;
    }
  }, [opts.focusRequest, boxRef, cameraRef, controlsRef, focusPendingRef]);

  useEffect(() => {
    const syncMarkers = () => {
      const g = instrMarkersRef.current;
      if (!g) return;
      refreshInstrMarkers(g, opts.instrStepsRef.current, opts.selectedInstrRef.current, {
        hideForRawPreview: rawPreviewActiveRef.current,
        playing: opts.playingRef.current,
        preferVia: opts.selectedPreferViaRef.current,
      });
    };
    const onRaw = (d: RawPreviewDetail) => {
      const g = rawPreviewRef.current;
      if (!g) return;
      const axisOpts = d.axisOpts || { x: true, y: true, z: true, interval: 0 };
      if (!d.preview) {
        clearRawPreviewGroup(g);
        rawPreviewActiveRef.current = false;
        syncMarkers();
        return;
      }
      if (opts.playingRef.current) {
        clearRawPreviewGroup(g);
        rawPreviewActiveRef.current = false;
      } else {
        applyRawPreviewToGroup(g, d.preview, axisOpts);
        rawPreviewActiveRef.current = !!(d.preview.pointsMm && d.preview.pointsMm.length);
      }
      syncMarkers();
    };
    return uiEvents.on(UI_EVT.rawPreview, onRaw);
  }, [instrMarkersRef, rawPreviewRef, rawPreviewActiveRef, opts.instrStepsRef, opts.selectedInstrRef, opts.playingRef, opts.selectedPreferViaRef]);

  useEffect(() => {
    const g = instrMarkersRef.current;
    if (!g) return;
    if (opts.playing) {
      const raw = rawPreviewRef.current;
      if (raw) clearRawPreviewGroup(raw);
      rawPreviewActiveRef.current = false;
    }
    refreshInstrMarkers(g, opts.activeProgramInstructions || [], opts.selectedInstrId, {
      hideForRawPreview: rawPreviewActiveRef.current,
      playing: opts.playing,
      preferVia: opts.selectedInstrPreferVia,
    });
  }, [
    opts.activeProgramInstructions,
    opts.selectedInstrId,
    opts.selectedInstrPreferVia,
    opts.playing,
    opts.activeRootId,
    instrMarkersRef,
    rawPreviewRef,
    rawPreviewActiveRef,
  ]);

  useEffect(() => {
    const onHl = (d: PickHighlightDetail) => {
      const g = overlayRef.current;
      if (!g) return;
      clearPickOverlayGroup(g);
      if (d?.clear) return;
      const mat = new THREE.LineBasicMaterial({ color: 0xff9800 });
      for (const poly of d.polys || []) {
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
    return uiEvents.on(UI_EVT.pickHighlight, onHl);
  }, [overlayRef]);
}
