import { useCallback, useEffect, useState } from "react";
import {
  fetchHealth,
  fetchModes,
  fetchObjects,
  importObject,
  newProject,
  openProject,
  postSelection,
  saveProject,
  setWorkspaceMode,
  type BackendObject,
  type Health,
} from "./api";
import SceneView from "./SceneView";

export default function App() {
  const [health, setHealth] = useState<Health | null>(null);
  const [path, setPath] = useState("");
  const [status, setStatus] = useState("");
  const [objects, setObjects] = useState<BackendObject[]>([]);
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [modes, setModes] = useState<{ id: string; title: string }[]>([]);
  const [mode, setMode] = useState("scene3d");

  const refreshHealth = useCallback(async () => {
    try {
      setHealth(await fetchHealth());
    } catch {
      setHealth(null);
    }
  }, []);

  const refreshObjects = useCallback(async () => {
    const list = await fetchObjects();
    setObjects(list.objects);
    if (list.projectPath) setPath(list.projectPath);
  }, []);

  useEffect(() => {
    void refreshHealth();
    void fetchModes().then((m) => {
      setModes(m.modes);
      setMode(m.active);
    });
    const t = setInterval(() => void refreshHealth(), 5000);
    const es = new EventSource("/api/events");
    es.onmessage = () => void refreshObjects();
    return () => {
      clearInterval(t);
      es.close();
    };
  }, [refreshHealth, refreshObjects]);

  const onSelect = async (id: string | null) => {
    setSelectedId(id);
    if (id) await postSelection(id);
  };

  return (
    <div className="app-shell">
      <header className="menubar">
        <strong>CloudSim Web</strong>
        <button type="button" onClick={() => void newProject().then(refreshObjects)}>
          新建
        </button>
        <button
          type="button"
          onClick={() =>
            void openProject(path).then((r) => {
              setStatus(r.ok ? `loaded ${r.objectCount}` : r.error ?? "fail");
              void refreshObjects();
            })
          }
        >
          打开
        </button>
        <button
          type="button"
          onClick={() =>
            void saveProject(path).then((r) => setStatus(r.ok ? `saved ${r.path}` : r.error ?? "fail"))
          }
        >
          保存
        </button>
        <button
          type="button"
          onClick={() => {
            const p = prompt("导入路径");
            if (p) void importObject(p).then(refreshObjects);
          }}
        >
          导入
        </button>
        <select
          value={mode}
          onChange={(e) => {
            const v = e.target.value;
            setMode(v);
            void setWorkspaceMode(v);
          }}
        >
          {modes.map((m) => (
            <option key={m.id} value={m.id}>
              {m.title}
            </option>
          ))}
        </select>
        <span className={`pill ${health ? "ok" : "bad"}`}>
          {health ? `:${health.port}` : "offline"}
        </span>
      </header>
      <div className="main">
        <aside className="sidebar">
          <input value={path} onChange={(e) => setPath(e.target.value)} placeholder="工程路径" />
          <p className="status">{status}</p>
          <ul className="tree">
            {objects.map((o) => (
              <li
                key={o.id}
                className={o.id === selectedId ? "sel" : ""}
                onClick={() => void onSelect(o.id)}
              >
                <span className="name">{o.name || o.id}</span>
                <span className="cls">{o.className}</span>
              </li>
            ))}
          </ul>
        </aside>
        <SceneView objects={objects} selectedId={selectedId} onSelect={(id) => void onSelect(id)} />
      </div>
    </div>
  );
}
