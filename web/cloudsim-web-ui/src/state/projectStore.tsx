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
  fetchHealth,
  fetchModes,
  dialogOpen,
  newProject,
  openProject,
  saveProject,
  setWorkspaceMode,
  type Health,
} from "../api";
import { eventHub } from "../sse/EventHub";
import { useStatus } from "./statusStore";

type ProjectCtx = {
  health: Health | null;
  path: string;
  setPath: (p: string) => void;
  modes: { id: string; title: string }[];
  mode: string;
  setMode: (m: string) => void;
  docTitle: string;
  dirty: boolean;
  refreshHealth: () => Promise<void>;
  doNew: () => Promise<void>;
  doOpen: () => Promise<void>;
  doOpenFolder: () => Promise<void>;
  doSave: () => Promise<void>;
  onProjectChanged: number;
};

const Ctx = createContext<ProjectCtx | null>(null);

/** 对齐桌面关页：未保存时先问是否保存，再问是否丢弃 */
async function confirmDiscardIfDirty(dirty: boolean, title: string, doSave: () => Promise<boolean>): Promise<boolean> {
  if (!dirty) return true;
  if (window.confirm(`「${title}」有未保存更改。\n确定：保存后继续\n取消：下一步选择是否丢弃`)) {
    return doSave();
  }
  return window.confirm(`丢弃对「${title}」的未保存更改？`);
}

export function ProjectProvider({ children }: { children: ReactNode }) {
  const { setStatus } = useStatus();
  const [health, setHealth] = useState<Health | null>(null);
  const [path, setPath] = useState("");
  const [modes, setModes] = useState<{ id: string; title: string }[]>([]);
  const [mode, setModeState] = useState("scene3d");
  const [bump, setBump] = useState(0);
  const [dirty, setDirty] = useState(false);
  const dirtyRef = useRef(false);
  dirtyRef.current = dirty;

  const docTitle = path ? path.split(/[/\\]/).pop() || path : "未命名1";

  const refreshHealth = useCallback(async () => {
    try {
      setHealth(await fetchHealth());
    } catch {
      setHealth(null);
    }
  }, []);

  const setMode = useCallback(async (m: string) => {
    setModeState(m);
    await setWorkspaceMode(m);
  }, []);

  useEffect(() => {
    void refreshHealth();
    void fetchModes().then((m) => {
      setModes(m.modes || []);
      if (m.active) setModeState(m.active);
    });
    const t = setInterval(() => void refreshHealth(), 5000);
    eventHub.start();
    const offAny = eventHub.onAny((_d, type) => {
      if (type === "ProjectLoaded" || type === "ProjectSaved") {
        setDirty(false);
        setBump((n) => n + 1);
        return;
      }
      if (
        type === "BackendObjectCreated" ||
        type === "BackendObjectRegistered" ||
        type === "BackendObjectRemoved" ||
        type === "PoseCommitted" ||
        type === "ObjectPatched" ||
        type === "SceneChanged" ||
        type === "message"
      ) {
        if (
          type === "BackendObjectCreated" ||
          type === "BackendObjectRegistered" ||
          type === "BackendObjectRemoved" ||
          type === "PoseCommitted" ||
          type === "ObjectPatched" ||
          type === "SceneChanged"
        ) {
          setDirty(true);
        }
        setBump((n) => n + 1);
      }
    });
    const offMode = eventHub.on("WorkspaceModeChanged", (data) => {
      try {
        const j = JSON.parse(data) as { mode?: string };
        if (j.mode) setModeState(j.mode);
      } catch {
        /* keepalive */
      }
    });
    return () => {
      clearInterval(t);
      offAny();
      offMode();
      eventHub.stop();
    };
  }, [refreshHealth]);

  const saveNow = useCallback(async (): Promise<boolean> => {
    let p = path;
    if (!p) {
      const d = await dialogOpen({ purpose: "saveProject", title: "保存工程" });
      if (!d.ok || !d.path) return false;
      p = d.path;
    }
    const r = await saveProject(p);
    if (r.ok) {
      setPath(r.path || p);
      setDirty(false);
      setStatus("已保存");
      return true;
    }
    setStatus(r.error || "保存失败", "err");
    return false;
  }, [path, setStatus]);

  const doNew = useCallback(async () => {
    if (!(await confirmDiscardIfDirty(dirtyRef.current, docTitle, saveNow))) return;
    const r = await newProject();
    setStatus(r.ok ? "已新建工程" : r.error || "新建失败", r.ok ? "info" : "err");
    setPath("");
    setDirty(false);
    setBump((n) => n + 1);
  }, [docTitle, saveNow, setStatus]);

  const doOpen = useCallback(async () => {
    if (!(await confirmDiscardIfDirty(dirtyRef.current, docTitle, saveNow))) return;
    const d = await dialogOpen({ purpose: "project", title: "打开工程" });
    if (!d.ok || !d.path) return;
    const r = await openProject(d.path);
    if (r.ok) {
      setPath(d.path);
      setDirty(false);
      setStatus(`已打开（${r.objectCount ?? 0} 对象）`);
      setBump((n) => n + 1);
    } else setStatus(r.error || "打开失败", "err");
  }, [docTitle, saveNow, setStatus]);

  const doOpenFolder = useCallback(async () => {
    if (!(await confirmDiscardIfDirty(dirtyRef.current, docTitle, saveNow))) return;
    const d = await dialogOpen({ purpose: "directory", title: "打开文件夹", directory: true });
    if (!d.ok || !d.path) return;
    const r = await openProject(d.path);
    if (r.ok) {
      setPath(d.path);
      setDirty(false);
      setStatus(`已打开文件夹`);
      setBump((n) => n + 1);
    } else setStatus(r.error || "打开失败", "err");
  }, [docTitle, saveNow, setStatus]);

  const doSave = useCallback(async () => {
    await saveNow();
  }, [saveNow]);

  const value = useMemo(
    () => ({
      health,
      path,
      setPath,
      modes,
      mode,
      setMode,
      docTitle,
      dirty,
      refreshHealth,
      doNew,
      doOpen,
      doOpenFolder,
      doSave,
      onProjectChanged: bump,
    }),
    [health, path, modes, mode, setMode, docTitle, dirty, refreshHealth, doNew, doOpen, doOpenFolder, doSave, bump],
  );

  return <Ctx.Provider value={value}>{children}</Ctx.Provider>;
}

export function useProject() {
  const v = useContext(Ctx);
  if (!v) throw new Error("ProjectProvider missing");
  return v;
}
