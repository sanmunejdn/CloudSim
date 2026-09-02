import { useState } from "react";
import { useProject } from "../state/projectStore";
import { useScene } from "../state/sceneStore";
import { dialogOpen } from "../api/project";
import { importUrdf } from "../api/robot";
import { useStatus } from "../state/statusStore";
import { useI18n } from "../i18n/useI18n";

type DockProps = {
  leftVisible: boolean;
  rightVisible: boolean;
  setLeftVisible: (v: boolean) => void;
  setRightVisible: (v: boolean) => void;
  resetLayout: () => void;
};

type Props = {
  onInsertFrame: () => void;
  onInsertMate: () => void;
  onFocus: () => void;
  onAbout: () => void;
  docks: DockProps;
};

export default function MenuBar({ onInsertFrame, onInsertMate, onFocus, onAbout, docks }: Props) {
  const { health, modes, mode, setMode, doNew, doOpen, doOpenFolder, doSave } = useProject();
  const {
    interactMode,
    setInteractMode,
    gizmoSpace,
    setGizmoSpace,
    doImport,
    doOpenModel,
    doOpenPointCloud,
    requestFocus,
  } = useScene();
  const { setStatus } = useStatus();
  const { t, lang, setLang, theme, toggleTheme } = useI18n();
  const [openMenu, setOpenMenu] = useState<string | null>(null);

  const toggle = (m: string) => setOpenMenu((v) => (v === m ? null : m));

  return (
    <header className="menubar" onMouseLeave={() => setOpenMenu(null)}>
      <div className={`menu ${openMenu === "file" ? "open" : ""}`}>
        <button type="button" className="menu-btn" onClick={() => toggle("file")}>
          {t("menu.file", "文件")}
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
          <button type="button" onClick={() => void doOpenPointCloud()}>
            打开点云…
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
          <button type="button" onClick={() => docks.resetLayout()}>
            重置布局
          </button>
          <button
            type="button"
            className={`menu-check ${docks.leftVisible ? "on" : ""}`}
            onClick={() => docks.setLeftVisible(!docks.leftVisible)}
          >
            左侧面板
          </button>
          <button
            type="button"
            className={`menu-check ${docks.rightVisible ? "on" : ""}`}
            onClick={() => docks.setRightVisible(!docks.rightVisible)}
          >
            右侧面板
          </button>
          <hr className="menu-sep" />
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
            className={`menu-check ${gizmoSpace === "local" ? "on" : ""}`}
            onClick={() => setGizmoSpace("local")}
          >
            变换：物体系
          </button>
          <button
            type="button"
            className={`menu-check ${gizmoSpace === "world" ? "on" : ""}`}
            onClick={() => setGizmoSpace("world")}
          >
            变换：世界系
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
          <button type="button" onClick={onInsertMate}>
            配合…
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
          <label className="menu-label">
            语言
            <select value={lang} onChange={(e) => setLang(e.target.value)}>
              <option value="zh">中文</option>
              <option value="en">English</option>
            </select>
          </label>
          <button type="button" onClick={toggleTheme}>
            {theme === "light" ? "切换深色主题" : "切换浅色主题"}
          </button>
        </div>
      </div>
      <div className={`menu ${openMenu === "help" ? "open" : ""}`}>
        <button type="button" className="menu-btn" onClick={() => toggle("help")}>
          帮助
        </button>
        <div className="menu-drop">
          <button type="button" onClick={onAbout}>
            关于 CloudSim Web
          </button>
        </div>
      </div>
      <div className="menubar-spacer" />
      <div className={`pill ${health ? "ok" : "bad"}`}>{health ? `:${health.port}` : "checking…"}</div>
    </header>
  );
}
