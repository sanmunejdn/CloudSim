import { useRef, useState } from "react";
import { AppProviders } from "./state/AppProviders";
import MenuBar from "./shell/MenuBar";
import LeftDock from "./shell/LeftDock";
import RightDock from "./shell/RightDock";
import StatusBar from "./shell/StatusBar";
import DocTabs from "./shell/DocTabs";
import DockSplitter from "./shell/DockSplitter";
import { useResizableDocks } from "./shell/useResizableDocks";
import SceneViewport, { type SceneViewportHandle } from "./scene/SceneViewport";
import AssemblyMateDialog from "./shell/AssemblyMateDialog";
import { createCoordinateFrame } from "./api";
import { useStatus } from "./state/statusStore";
import { useScene } from "./state/sceneStore";
import { useProject } from "./state/projectStore";
import WorkspaceModeRouter from "./shell/WorkspaceModeRouter";
import "./styles/shell.css";

function InsertFrameDialog({ open, onClose }: { open: boolean; onClose: () => void }) {
  const { setStatus } = useStatus();
  const { refreshObjects } = useScene();
  const [name, setName] = useState("Frame");
  const [pos, setPos] = useState([0, 0, 0]);
  const [eu, setEu] = useState([0, 0, 0]);
  if (!open) return null;
  return (
    <div className="dlg-mask">
      <div className="dlg">
        <h3>插入坐标系</h3>
        <p className="dlg-hint">在场景中创建一个坐标系对象</p>
        <label className="dlg-field">
          名称
          <input value={name} onChange={(e) => setName(e.target.value)} />
        </label>
        <div className="dlg-spins">
          {(["X", "Y", "Z"] as const).map((lab, i) => (
            <label key={lab}>
              {lab}
              <input
                type="number"
                value={pos[i]}
                onChange={(e) => setPos(pos.map((v, j) => (j === i ? Number(e.target.value) : v)))}
              />
            </label>
          ))}
          {(["Rx", "Ry", "Rz"] as const).map((lab, i) => (
            <label key={lab}>
              {lab}
              <input
                type="number"
                value={eu[i]}
                onChange={(e) => setEu(eu.map((v, j) => (j === i ? Number(e.target.value) : v)))}
              />
            </label>
          ))}
        </div>
        <div className="dlg-actions">
          <button type="button" onClick={onClose}>
            取消
          </button>
          <button
            type="button"
            className="primary"
            onClick={async () => {
              const r = await createCoordinateFrame({ name, positionMm: pos, eulerDeg: eu });
              setStatus(r.ok ? "坐标系已创建" : r.error || "创建失败", r.ok ? "info" : "err");
              await refreshObjects();
              onClose();
            }}
          >
            创建
          </button>
        </div>
      </div>
    </div>
  );
}

function AboutDialog({ open, onClose }: { open: boolean; onClose: () => void }) {
  const { health } = useProject();
  if (!open) return null;
  return (
    <div className="dlg-mask" onClick={onClose}>
      <div className="dlg" onClick={(e) => e.stopPropagation()}>
        <h3>关于 CloudSim Web</h3>
        <p className="dlg-hint">
          CloudSim Web — 主程序三维工作区网页端
          <br />
          本地 Host：127.0.0.1:{health?.port ?? 8787}
          {health ? ` · pid ${health.pid}` : ""}
        </p>
        <div className="dlg-actions">
          <button type="button" className="primary" onClick={onClose}>
            确定
          </button>
        </div>
      </div>
    </div>
  );
}

function Shell() {
  const sceneRef = useRef<SceneViewportHandle>(null);
  const [frameDlg, setFrameDlg] = useState(false);
  const [mateDlg, setMateDlg] = useState(false);
  const [aboutDlg, setAboutDlg] = useState(false);
  const [wireframe, setWireframe] = useState(false);
  const { requestFocus, interactMode, setInteractMode } = useScene();
  const { mode } = useProject();
  const docks = useResizableDocks();
  const isGeomodeling = mode === "geomodeling";
  const showLeftDock = docks.leftVisible && !isGeomodeling;

  return (
    <div className={`shell${isGeomodeling ? " mode-geomodeling" : ""}`}>
      <MenuBar
        onInsertFrame={() => setFrameDlg(true)}
        onInsertMate={() => setMateDlg(true)}
        onAbout={() => setAboutDlg(true)}
        onFocus={() => {
          requestFocus();
          sceneRef.current?.focusAll();
        }}
        docks={{
          leftVisible: docks.leftVisible,
          rightVisible: docks.rightVisible,
          setLeftVisible: docks.setLeftVisible,
          setRightVisible: docks.setRightVisible,
          resetLayout: docks.resetLayout,
        }}
      />
      <div className="main" style={docks.gridStyle(showLeftDock, docks.rightVisible)}>
        {showLeftDock ? (
          <>
            <LeftDock />
            <DockSplitter
              title="拖动调整左侧栏宽度"
              onDragStart={docks.beginLeftDrag}
              onDrag={docks.onLeftDrag}
              onDragEnd={docks.persistLeftEnd}
            />
          </>
        ) : null}
        <section className="center">
          <DocTabs />
          <WorkspaceModeRouter
            sideVisible={docks.leftVisible}
            scene3dChildren={
              <div className="viewport">
                <SceneViewport ref={sceneRef} />
                <div className="view-toolbar">
                  <button
                    type="button"
                    title="对象选择"
                    className={interactMode === "select" ? "active" : ""}
                    onClick={() => setInteractMode(interactMode === "select" ? "view" : "select")}
                  >
                    ✥
                  </button>
                  <button type="button" title="聚焦" onClick={() => sceneRef.current?.focusAll()}>
                    ⌂
                  </button>
                  <button type="button" title="主视图" onClick={() => sceneRef.current?.homeView()}>
                    ◎
                  </button>
                  <button
                    type="button"
                    title="线框"
                    className={wireframe ? "active" : ""}
                    onClick={() => {
                      const next = !wireframe;
                      setWireframe(next);
                      sceneRef.current?.setWireframe(next);
                    }}
                  >
                    ▦
                  </button>
                  <button type="button" title="截图" onClick={() => sceneRef.current?.capturePng()}>
                    📷
                  </button>
                  <button
                    type="button"
                    title={isGeomodeling ? "特征树" : "左侧面板"}
                    className={docks.leftVisible ? "active" : ""}
                    onClick={() => docks.setLeftVisible(!docks.leftVisible)}
                  >
                    ◧
                  </button>
                  <button
                    type="button"
                    title={isGeomodeling ? "AI 助手" : "右侧面板"}
                    className={docks.rightVisible ? "active" : ""}
                    onClick={() => docks.setRightVisible(!docks.rightVisible)}
                  >
                    ◨
                  </button>
                </div>
              </div>
            }
          />
        </section>
        {docks.rightVisible ? (
          <>
            <DockSplitter
              title="拖动调整右侧栏宽度"
              onDragStart={docks.beginRightDrag}
              onDrag={docks.onRightDrag}
              onDragEnd={docks.persistRightEnd}
            />
            <RightDock />
          </>
        ) : null}
      </div>
      <StatusBar />
      <InsertFrameDialog open={frameDlg} onClose={() => setFrameDlg(false)} />
      <AssemblyMateDialog open={mateDlg} onClose={() => setMateDlg(false)} />
      <AboutDialog open={aboutDlg} onClose={() => setAboutDlg(false)} />
    </div>
  );
}

export default function App() {
  return (
    <AppProviders>
      <Shell />
    </AppProviders>
  );
}
