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
import { createCoordinateFrame } from "./api";
import { useStatus } from "./state/statusStore";
import { useScene } from "./state/sceneStore";
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

function Shell() {
  const sceneRef = useRef<SceneViewportHandle>(null);
  const [frameDlg, setFrameDlg] = useState(false);
  const { requestFocus } = useScene();
  const docks = useResizableDocks();

  return (
    <div className="shell">
      <MenuBar
        onInsertFrame={() => setFrameDlg(true)}
        onFocus={() => {
          requestFocus();
          sceneRef.current?.focusAll();
        }}
      />
      <div className="main" style={docks.mainStyle}>
        <LeftDock />
        <DockSplitter
          title="拖动调整左侧栏宽度"
          onDragStart={docks.beginLeftDrag}
          onDrag={docks.onLeftDrag}
          onDragEnd={docks.persistLeftEnd}
        />
        <section className="center">
          <DocTabs />
          <WorkspaceModeRouter
            scene3dChildren={
              <div className="viewport">
                <SceneViewport ref={sceneRef} />
                <div className="view-toolbar">
                  <button type="button" title="聚焦" onClick={() => sceneRef.current?.focusAll()}>
                    ⌂
                  </button>
                  <button type="button" title="主视图" onClick={() => sceneRef.current?.homeView()}>
                    ◎
                  </button>
                </div>
              </div>
            }
          />
        </section>
        <DockSplitter
          title="拖动调整右侧栏宽度"
          onDragStart={docks.beginRightDrag}
          onDrag={docks.onRightDrag}
          onDragEnd={docks.persistRightEnd}
        />
        <RightDock />
      </div>
      <StatusBar />
      <InsertFrameDialog open={frameDlg} onClose={() => setFrameDlg(false)} />
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
