/// @file useRobotTeach.ts
/// @brief 机器人锚点解析、Gizmo 挂接、G/R 快捷键、place/tcp-ik 推送

import { useEffect, useState } from "react";
import * as THREE from "three";
import { patchObject, placeRobot, resolveRobot, tcpIk, tcpPose, type BackendObject } from "../../api";
import {
  localMatrixArray,
  prepareMeshForGizmo,
  snapProxyFromWorldMatrix,
  snapProxyToMesh,
} from "../objectMesh";
import { syncActiveToolOverlayFromProxy } from "../frameAxes";
import type { RobotMeta, ViewportRefs } from "./types";

export function useRobotTeach(
  refs: ViewportRefs,
  opts: {
    selectedId: string | null;
    interactMode: "view" | "select";
    robotDragMode: boolean;
    gizmoTransformMode: "translate" | "rotate";
    gizmoSpace: "local" | "world";
    objects: BackendObject[];
    activeRootId: string | null;
    setGizmoTransformMode: (m: "translate" | "rotate") => void;
    setStatus: (m: string, k?: "info" | "err" | "warn") => void;
  },
) {
  const {
    transformRef,
    rootRef,
    dragProxyRef,
    idToMesh,
    frameOverlayRef,
    controlsRef,
    gizmoCtxRef,
  } = refs;

  const [robotMeta, setRobotMeta] = useState<RobotMeta>({ isRobot: false });

  useEffect(() => {
    let cancelled = false;
    if (!opts.selectedId) {
      setRobotMeta({ isRobot: false });
      return;
    }
    void resolveRobot(opts.selectedId).then((r) => {
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
  }, [opts.selectedId]);

  // 挂 gizmo
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

    if (opts.robotDragMode) {
      let cancelled = false;
      void (async () => {
        let flangeId = robotMeta.flangeBackendId;
        let sceneRoot = robotMeta.sceneRootBackendId || opts.activeRootId || undefined;
        const resolveId = opts.selectedId || opts.activeRootId;
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

    if (opts.interactMode !== "select" || !opts.selectedId) {
      detach();
      return;
    }

    if (robotMeta.isRobot && robotMeta.anchorBackendId) {
      const mesh = idToMesh.current.get(robotMeta.anchorBackendId);
      if (mesh) {
        snapProxyToMesh(root, proxy, mesh);
      } else {
        const o = opts.objects.find((x) => x.id === robotMeta.anchorBackendId);
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

    const mesh = idToMesh.current.get(opts.selectedId);
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
  }, [
    opts.selectedId,
    opts.interactMode,
    opts.robotDragMode,
    robotMeta,
    opts.objects,
    opts.activeRootId,
    transformRef,
    rootRef,
    dragProxyRef,
    idToMesh,
    frameOverlayRef,
    gizmoCtxRef,
  ]);

  useEffect(() => {
    const transform = transformRef.current;
    const proxy = dragProxyRef.current;
    if (!transform?.object || transform.dragging) return;
    proxy?.updateMatrixWorld(true);
    transform.setMode(opts.gizmoTransformMode);
    transform.setSpace(opts.gizmoSpace);
  }, [opts.gizmoTransformMode, opts.gizmoSpace, transformRef, dragProxyRef]);

  useEffect(() => {
    const onKey = (ev: KeyboardEvent) => {
      const t = ev.target as HTMLElement | null;
      if (t && (t.tagName === "INPUT" || t.tagName === "TEXTAREA" || t.isContentEditable)) return;
      const transform = transformRef.current;
      if (!transform?.object || !transform.enabled) return;
      if (ev.key === "g" || ev.key === "G") {
        ev.preventDefault();
        opts.setGizmoTransformMode("translate");
        opts.setStatus("罗盘：移动（G）");
      } else if (ev.key === "r" || ev.key === "R") {
        ev.preventDefault();
        opts.setGizmoTransformMode("rotate");
        opts.setStatus("罗盘：旋转（R）");
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [opts.setGizmoTransformMode, opts.setStatus, transformRef]);

  // place / tcp-ik / PATCH
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

    /** 从 tcp-ik/tcp-pose 的 FK 实际到达写入示教缓存（对齐桌面 m_lastTcpDragTargetInBase） */
    const applyTeachTargetFromIk = (
      r: {
        positionMm?: number[];
        eulerDeg?: number[];
        jointAnglesRad?: number[];
        jointRadCsv?: string;
        tcpLinkName?: string;
        flangeLinkName?: string;
        urdfPath?: string;
        toolFrameMat4Csv?: string;
        activeToolFrameId?: string;
        activeUserFrameId?: string;
        targetTransformQuatCsv?: string;
        targetTransformTransMmCsv?: string;
      },
      jointRadCsvFallback?: string,
    ) => {
      const joints = Array.isArray(r.jointAnglesRad) ? r.jointAnglesRad.map(Number) : [];
      const jointRadCsv =
        r.jointRadCsv ||
        (joints.length ? joints.map((v) => Number(v).toFixed(6)).join(",") : jointRadCsvFallback);
      if (!r.positionMm || !r.eulerDeg || r.positionMm.length < 3 || r.eulerDeg.length < 3) return;
      gizmoCtxRef.current.setRobotDragTeachPose({
        positionMm: [
          Number(r.positionMm[0]) || 0,
          Number(r.positionMm[1]) || 0,
          Number(r.positionMm[2]) || 0,
        ],
        eulerDeg: [
          Number(r.eulerDeg[0]) || 0,
          Number(r.eulerDeg[1]) || 0,
          Number(r.eulerDeg[2]) || 0,
        ],
        jointRadCsv,
        tcpLinkName: r.tcpLinkName || undefined,
        flangeLinkName: r.flangeLinkName || undefined,
        urdfPath: r.urdfPath || undefined,
        toolFrameMat4Csv: r.toolFrameMat4Csv || undefined,
        activeToolFrameId: r.activeToolFrameId || undefined,
        activeUserFrameId: r.activeUserFrameId || undefined,
        targetTransformQuatCsv: r.targetTransformQuatCsv || undefined,
        targetTransformTransMmCsv: r.targetTransformTransMmCsv || undefined,
      });
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
        applyTeachTargetFromIk(r);
        const now = performance.now();
        if (now - lastMeshRefreshAt >= kMeshRefreshMinIntervalMs || !transform.dragging) {
          lastMeshRefreshAt = now;
          await ctx.refreshObjects();
        }
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
          // 拖完以当前 FK 覆盖示教缓存，避免追赶未完成时仍用期望目标落盘
          const sceneRoot =
            ctx.robotMeta.sceneRootBackendId || ctx.activeRootId || undefined;
          if (sceneRoot) {
            const pose = await tcpPose(sceneRoot);
            if (pose.ok) applyTeachTargetFromIk(pose, pose.jointRadCsv);
          }
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
  }, [transformRef, controlsRef, rootRef, dragProxyRef, idToMesh, frameOverlayRef, gizmoCtxRef]);

  return robotMeta;
}
