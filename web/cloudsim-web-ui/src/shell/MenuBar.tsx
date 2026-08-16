import { useState } from "react";
import { useProject } from "../state/projectStore";
import { useScene } from "../state/sceneStore";
import { dialogOpen } from "../api/project";
import { importUrdf } from "../api/robot";
import { useStatus } from "../state/statusStore";

type Props = {
  onInsertFrame: () => void;
  onFocus: () => void;
};

export default function MenuBar({ onInsertFrame, onFocus }: Props) {
  const { health, modes, mode, setMode, doNew, doOpen, doOpenFolder, doSave } = useProject();
  const { interactMode, setInteractMode, doImport, doOpenModel, requestFocus } = useScene();
  const { setStatus } = useStatus();
  const [openMenu, setOpenMenu] = useState<string | null>(null);

  const toggle = (m: string) => setOpenMenu((v) => (v === m ? null : m));

  return (
    <header className="menubar" onMouseLeave={() => setOpenMenu(null)}>
      <div className={`menu ${openMenu === "file" ? "open" : ""}`}>
        <button type="button" className="menu-btn" onClick={() => toggle("file")}>
          文件
        </button>
        <div className="menu-drop">
          <button type="button" onClick={() => void doNew()}>
            新建
          </button>
          <button type="button" onClick={() => void doOpen()}>
            打开…
          </button>
          <button type="button" onClick={() => void doOpenFolder()}>
            打开文件夹…
          </button>
          <button type="button" onClick={() => void doOpenModel()}>
            打开模型…
          </button>
          <button type="button" onClick={() => void doSave()}>
            保存
          </button>
          <button type="button" onClick={() => void doImport()}>
            导入…
          </button>
        </div>
      </div>
      <div className={`menu ${openMenu === "view" ? "open" : ""}`}>
        <button type="button" className="menu-btn" onClick={() => toggle("view")}>
          视图
        </button>
        <div className="menu-drop">
          <button
            type="button"
            className={`menu-check ${interactMode === "view" ? "on" : ""}`}
            onClick={() => setInteractMode("view")}
          >
            视图移动
          </button>
          <button
            type="button"
            className={`menu-check ${interactMode === "select" ? "on" : ""}`}
            onClick={() => setInteractMode("select")}
          >
            对象选择
          </button>
          <hr className="menu-sep" />
          <button
            type="button"
            onClick={() => {
              requestFocus();
              onFocus();
            }}
          >
            聚焦全部
          </button>
        </div>
      </div>
      <div className={`menu ${openMenu === "insert" ? "open" : ""}`}>
        <button type="button" className="menu-btn" onClick={() => toggle("insert")}>
          插入
        </button>
        <div className="menu-drop">
          <button
            type="button"
            onClick={async () => {
              const d = await dialogOpen({ purpose: "urdf", title: "导入 URDF" });
              if (!d.ok || !d.path) return;
              const r = await importUrdf(d.path);
              setStatus(r.ok ? "URDF 已导入" : r.error || "导入失败", r.ok ? "info" : "err");
            }}
          >
            导入 URDF…
          </button>
          <button type="button" onClick={onInsertFrame}>
            坐标系…
          </button>
        </div>
      </div>
      <div className={`menu ${openMenu === "settings" ? "open" : ""}`}>
        <button type="button" className="menu-btn" onClick={() => toggle("settings")}>
          设置
        </button>
        <div className="menu-drop">
          <label className="menu-label">
            工作区模式
            <select value={mode} onChange={(e) => void setMode(e.target.value)}>
              {modes.map((m) => (
                <option key={m.id} value={m.id}>
                  {m.title}
                </option>
              ))}
            </select>
          </label>
        </div>
      </div>
      <div className={`menu ${openMenu === "help" ? "open" : ""}`}>
        <button type="button" className="menu-btn" onClick={() => toggle("help")}>
          帮助
        </button>
        <div className="menu-drop">
          <button type="button" onClick={() => setStatus("CloudSim Web · Vite React 坞式壳")}>
            关于 CloudSim Web
          </button>
        </div>
      </div>
      <div className="menubar-spacer" />
      <div className={`pill ${health ? "ok" : "bad"}`}>{health ? `:${health.port}` : "checking…"}</div>
    </header>
  );
}
