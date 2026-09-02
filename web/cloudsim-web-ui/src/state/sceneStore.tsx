import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
  type ReactNode,
} from "react";
import {
  fetchObjects,
  fetchObjectDetail,
  postSelection,
  importObject,
  type BackendObject,
} from "../api";
import { dialogOpen, dialogPaths } from "../api/project";
import { eventHub } from "../sse/EventHub";
import { useProject } from "./projectStore";
import { useStatus } from "./statusStore";

export type InteractMode = "view" | "select";
export type GizmoTransformMode = "translate" | "rotate";
export type GizmoSpace = "local" | "world";

export type RobotDragTeachPose = {
  positionMm: [number, number, number];
  eulerDeg: [number, number, number];
  jointRadCsv?: string;
};

type SceneCtx = {
  objects: BackendObject[];
  selectedId: string | null;
  interactMode: InteractMode;
  robotDragMode: boolean;
  gizmoTransformMode: GizmoTransformMode;
  gizmoSpace: GizmoSpace;
  robotDragTeachPose: RobotDragTeachPose | null;
  refreshObjects: () => Promise<void>;
  selectObject: (id: string | null) => Promise<void>;
  setInteractMode: (m: InteractMode) => void;
  setRobotDragMode: (v: boolean) => void;
  setGizmoTransformMode: (m: GizmoTransformMode) => void;
  setGizmoSpace: (s: GizmoSpace) => void;
  setRobotDragTeachPose: (p: RobotDragTeachPose | null) => void;
  doImport: () => Promise<void>;
  doOpenModel: () => Promise<void>;
  doOpenPointCloud: () => Promise<void>;
  /** 插入配合：0=固定面 1=动件面；null=关闭 */
  mateFacePickSlot: 0 | 1 | null;
  setMateFacePickSlot: (s: 0 | 1 | null) => void;
  focusRequest: number;
  requestFocus: () => void;
};

const Ctx = createContext<SceneCtx | null>(null);

export function SceneProvider({ children }: { children: ReactNode }) {
  const { onProjectChanged, setPath } = useProject();
  const { setStatus } = useStatus();
  const [objects, setObjects] = useState<BackendObject[]>([]);
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [interactMode, setInteractMode] = useState<InteractMode>("view");
  const [robotDragMode, setRobotDragModeState] = useState(false);
  const [gizmoTransformMode, setGizmoTransformMode] = useState<GizmoTransformMode>("translate");
  const [gizmoSpace, setGizmoSpace] = useState<GizmoSpace>("local");
  const [robotDragTeachPose, setRobotDragTeachPose] = useState<RobotDragTeachPose | null>(null);
  const [mateFacePickSlot, setMateFacePickSlot] = useState<0 | 1 | null>(null);
  const [focusRequest, setFocusRequest] = useState(0);
  const robotDragModeRef = useRef(false);
  robotDragModeRef.current = robotDragMode;

  const setRobotDragMode = useCallback((v: boolean) => {
    setRobotDragModeState(v);
    if (!v) setRobotDragTeachPose(null);
  }, []);

  const refreshObjects = useCallback(async () => {
    const list = await fetchObjects();
    setObjects(list.objects || []);
    if (list.projectPath) setPath(list.projectPath);
  }, [setPath]);

  const mergeObjectById = useCallback(async (id: string) => {
    if (!id) return;
    const d = await fetchObjectDetail(id);
    const obj = (d.object ?? (d as unknown as BackendObject)) as BackendObject;
    if (!obj || !obj.id) return;
    setObjects((prev) => {
      const i = prev.findIndex((o) => o.id === obj.id);
      if (i < 0) return [...prev, obj];
      const next = prev.slice();
      next[i] = { ...prev[i], ...obj };
      return next;
    });
  }, []);

  useEffect(() => {
    void refreshObjects();
  }, [onProjectChanged, refreshObjects]);

  useEffect(() => {
    const off = eventHub.onAny((_d, type) => {
      if (type === "RobotKinematicsApplied" && robotDragModeRef.current) {
        return;
      }
      if (type === "PoseCommitted" || type === "ObjectPatched") {
        try {
          const j = JSON.parse(_d) as { backendId?: string; id?: string };
          const id = j.backendId || j.id;
          if (id) {
            void mergeObjectById(id);
            return;
          }
        } catch {
          /* 无 payload 则全量 */
        }
        void refreshObjects();
        return;
      }
      if (type === "RobotKinematicsApplied") {
        try {
          const j = JSON.parse(_d) as { sceneRootBackendId?: string; backendId?: string };
          const id = j.sceneRootBackendId || j.backendId;
          if (id) {
            void mergeObjectById(id);
            return;
          }
        } catch {
          /* fallthrough */
        }
        void refreshObjects();
        return;
      }
      if (
        type === "SceneChanged" ||
        type === "BackendObjectCreated" ||
        type === "BackendObjectRegistered" ||
        type === "BackendObjectRemoved" ||
        type === "ProjectLoaded"
      ) {
        void refreshObjects();
      }
      if (type === "SelectionChanged") {
        try {
          const j = JSON.parse(_d) as { backendId?: string };
          if (j.backendId) setSelectedId(j.backendId);
        } catch {
          /* ignore */
        }
      }
    });
    return off;
  }, [refreshObjects, mergeObjectById]);

  const selectObject = useCallback(async (id: string | null) => {
    setSelectedId(id);
    if (id) await postSelection(id);
  }, []);

  const doImport = useCallback(async () => {
    const d = await dialogOpen({ purpose: "import", title: "导入模型/点云" });
    if (!d.ok) return;
    const paths = dialogPaths(d);
    if (!paths.length) return;
    let okN = 0;
    let lastErr = "";
    for (const p of paths) {
      const lower = p.toLowerCase();
      const isPc = [".pcd", ".ply", ".las", ".laz", ".xyz"].some((x) => lower.endsWith(x));
      const r = await importObject(p, isPc);
      if (r.ok) ++okN;
      else lastErr = r.error || "导入失败";
    }
    setStatus(
      okN === paths.length
        ? `已导入 ${okN} 个文件`
        : okN > 0
          ? `已导入 ${okN}/${paths.length}；失败：${lastErr}`
          : lastErr || "导入失败",
      okN > 0 ? "info" : "err",
    );
    await refreshObjects();
  }, [refreshObjects, setStatus]);

  const requestFocus = useCallback(() => setFocusRequest((n) => n + 1), []);

  // 对齐桌面「打开模型」：多选；始终按网格/CAD 导入
  const doOpenModel = useCallback(async () => {
    const d = await dialogOpen({ purpose: "model", title: "打开模型" });
    if (!d.ok) return;
    const paths = dialogPaths(d);
    if (!paths.length) return;
    let okN = 0;
    let lastErr = "";
    for (const p of paths) {
      const r = await importObject(p, false);
      if (r.ok) ++okN;
      else lastErr = r.error || "打开模型失败";
    }
    setStatus(
      okN === paths.length
        ? `已打开 ${okN} 个模型`
        : okN > 0
          ? `已打开 ${okN}/${paths.length}；失败：${lastErr}`
          : lastErr || "打开模型失败",
      okN > 0 ? "info" : "err",
    );
    await refreshObjects();
    if (okN > 0) requestFocus();
  }, [refreshObjects, setStatus, requestFocus]);

  // 对齐桌面「打开点云」：dialog 过滤器走 Host purpose=pointcloud
  const doOpenPointCloud = useCallback(async () => {
    const d = await dialogOpen({ purpose: "pointcloud", title: "打开点云" });
    if (!d.ok) return;
    const paths = dialogPaths(d);
    if (!paths.length) return;
    let okN = 0;
    let lastErr = "";
    for (const p of paths) {
      const r = await importObject(p, true);
      if (r.ok) ++okN;
      else lastErr = r.error || "打开点云失败";
    }
    setStatus(
      okN === paths.length
        ? `已打开 ${okN} 个点云`
        : okN > 0
          ? `已打开 ${okN}/${paths.length}；失败：${lastErr}`
          : lastErr || "打开点云失败",
      okN > 0 ? "info" : "err",
    );
    await refreshObjects();
    if (okN > 0) requestFocus();
  }, [refreshObjects, setStatus, requestFocus]);

  const value = useMemo(
    () => ({
      objects,
      selectedId,
      interactMode,
      robotDragMode,
      gizmoTransformMode,
      gizmoSpace,
      robotDragTeachPose,
      refreshObjects,
      selectObject,
      setInteractMode,
      setRobotDragMode,
      setGizmoTransformMode,
      setGizmoSpace,
      setRobotDragTeachPose,
      doImport,
      doOpenModel,
      doOpenPointCloud,
      mateFacePickSlot,
      setMateFacePickSlot,
      focusRequest,
      requestFocus,
    }),
    [
      objects,
      selectedId,
      interactMode,
      robotDragMode,
      gizmoTransformMode,
      gizmoSpace,
      robotDragTeachPose,
      refreshObjects,
      selectObject,
      setRobotDragMode,
      doImport,
      doOpenModel,
      doOpenPointCloud,
      mateFacePickSlot,
      focusRequest,
      requestFocus,
    ],
  );

  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}

export function useScene() {
  const v = useContext(Ctx);
  if (!v) throw new Error("SceneProvider missing");
  return v;
}
