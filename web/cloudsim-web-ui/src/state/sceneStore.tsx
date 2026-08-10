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
  postSelection,
  importObject,
  type BackendObject,
} from "../api";
import { dialogOpen } from "../api/project";
import { eventHub } from "../sse/EventHub";
import { useProject } from "./projectStore";
import { useStatus } from "./statusStore";

export type InteractMode = "view" | "select";
export type GizmoTransformMode = "translate" | "rotate";

type SceneCtx = {
  objects: BackendObject[];
  selectedId: string | null;
  interactMode: InteractMode;
  robotDragMode: boolean;
  gizmoTransformMode: GizmoTransformMode;
  refreshObjects: () => Promise<void>;
  selectObject: (id: string | null) => Promise<void>;
  setInteractMode: (m: InteractMode) => void;
  setRobotDragMode: (v: boolean) => void;
  setGizmoTransformMode: (m: GizmoTransformMode) => void;
  doImport: () => Promise<void>;
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
  const [robotDragMode, setRobotDragMode] = useState(false);
  const [gizmoTransformMode, setGizmoTransformMode] = useState<GizmoTransformMode>("translate");
  const [focusRequest, setFocusRequest] = useState(0);
  const robotDragModeRef = useRef(false);
  robotDragModeRef.current = robotDragMode;

  const refreshObjects = useCallback(async () => {
    const list = await fetchObjects();
    setObjects(list.objects || []);
    if (list.projectPath) setPath(list.projectPath);
  }, [setPath]);

  useEffect(() => {
    void refreshObjects();
  }, [onProjectChanged, refreshObjects]);

  useEffect(() => {
    const off = eventHub.onAny((_d, type) => {
      // 末端拖动时由 SceneViewport 节流刷新，避免 SSE 与 IK 双拉卡顿
      if (type === "RobotKinematicsApplied" && robotDragModeRef.current) {
        return;
      }
      if (
        type === "ObjectPatched" ||
        type === "PoseCommitted" ||
        type === "RobotKinematicsApplied" ||
        type === "BackendObjectCreated" ||
        type === "BackendObjectRegistered" ||
        type === "BackendObjectRemoved" ||
        type === "message"
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
  }, [refreshObjects]);

  const selectObject = useCallback(async (id: string | null) => {
    setSelectedId(id);
    if (id) await postSelection(id);
  }, []);

  const doImport = useCallback(async () => {
    const d = await dialogOpen({ purpose: "import", title: "导入模型/点云" });
    if (!d.ok || !d.path) return;
    const lower = d.path.toLowerCase();
    const isPc = [".pcd", ".ply", ".las", ".laz", ".xyz"].some((x) => lower.endsWith(x));
    const r = await importObject(d.path, isPc);
    setStatus(r.ok ? "导入成功" : r.error || "导入失败", r.ok ? "info" : "err");
    await refreshObjects();
  }, [refreshObjects, setStatus]);

  const requestFocus = useCallback(() => setFocusRequest((n) => n + 1), []);

  const value = useMemo(
    () => ({
      objects,
      selectedId,
      interactMode,
      robotDragMode,
      gizmoTransformMode,
      refreshObjects,
      selectObject,
      setInteractMode,
      setRobotDragMode,
      setGizmoTransformMode,
      doImport,
      focusRequest,
      requestFocus,
    }),
    [
      objects,
      selectedId,
      interactMode,
      robotDragMode,
      gizmoTransformMode,
      refreshObjects,
      selectObject,
      doImport,
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
