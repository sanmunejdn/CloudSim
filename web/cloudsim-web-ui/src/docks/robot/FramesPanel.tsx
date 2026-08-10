import { useEffect, useState } from "react";
import {
  mutateFrames,
  putFrames,
  captureTool,
  resetTool,
  captureUser,
  type FrameSet,
  type ToolFrame,
  type UserFrame,
} from "../../api";
import { fetchRobotInstances } from "../../api/robot";
import { useFrames } from "../../state/frameStore";
import { useRobotProgram } from "../../state/robotProgramStore";
import { useStatus } from "../../state/statusStore";

export default function FramesPanel() {
  const { frames, linkNames, reloadFrames } = useFrames();
  const { activeRootId } = useRobotProgram();
  const { setStatus } = useStatus();
  const [instances, setInstances] = useState<{ sceneRootBackendId: string; name?: string; label?: string }[]>([]);
  const [root, setRoot] = useState(activeRootId);
  const [selTool, setSelTool] = useState("");
  const [selUser, setSelUser] = useState("");

  useEffect(() => {
    void fetchRobotInstances().then((r) => setInstances(r.instances || []));
  }, []);
  useEffect(() => {
    if (activeRootId) setRoot(activeRootId);
  }, [activeRootId]);

  const empty = !root || !instances.length;

  if (empty && !frames) {
    return (
      <div className="robot-pane" id="robotFrame">
        <label className="field">
          机器人实例
          <select value={root} onChange={(e) => setRoot(e.target.value)}>
            {instances.map((i) => (
              <option key={i.sceneRootBackendId} value={i.sceneRootBackendId}>
                {i.label || i.name || i.sceneRootBackendId}
              </option>
            ))}
          </select>
        </label>
        <div className="toolbar-row">
          <button type="button" className="btn-ghost" onClick={() => void reloadFrames()}>
            刷新
          </button>
        </div>
        <div className="hint">暂无机器人实例，请从设备库导入或打开含机器人的工程</div>
      </div>
    );
  }

  if (!frames) return <div className="robot-pane muted">加载坐标系…</div>;

  const tools = frames.toolFrames || [];
  const users = frames.userFrames || [];
  const tool = tools.find((t) => t.id === (selTool || frames.activeToolFrameId)) || tools[0];
  const user = users.find((u) => u.id === (selUser || frames.activeUserFrameId)) || users[0];
  const sceneRoot = root || activeRootId;

  const save = async (next: FrameSet) => {
    if (!sceneRoot) return;
    const r = await putFrames(sceneRoot, next);
    setStatus(r.ok ? "坐标系已保存" : r.error || "保存失败", r.ok ? "info" : "err");
    await reloadFrames();
  };

  const setSpin = (which: "tool" | "user", axis: "p" | "e", idx: number, value: number) => {
    if (which === "tool" && tool) {
      const T = tool.T_flange_tool || { positionMm: [0, 0, 0], eulerDeg: [0, 0, 0] };
      const field = axis === "p" ? "positionMm" : "eulerDeg";
      const arr = [...(T[field] || [0, 0, 0])];
      arr[idx] = value;
      const nextTool: ToolFrame = { ...tool, T_flange_tool: { ...T, [field]: arr } };
      void save({ ...frames, toolFrames: tools.map((t) => (t.id === tool.id ? nextTool : t)) });
    }
    if (which === "user" && user) {
      const T = user.T_base_user || { positionMm: [0, 0, 0], eulerDeg: [0, 0, 0] };
      const field = axis === "p" ? "positionMm" : "eulerDeg";
      const arr = [...(T[field] || [0, 0, 0])];
      arr[idx] = value;
      const nextUser: UserFrame = { ...user, T_base_user: { ...T, [field]: arr } };
      void save({ ...frames, userFrames: users.map((u) => (u.id === user.id ? nextUser : u)) });
    }
  };

  return (
    <div className="robot-pane" id="robotFrame">
      <label className="field">
        机器人实例
        <select value={sceneRoot} onChange={(e) => setRoot(e.target.value)}>
          {instances.map((i) => (
            <option key={i.sceneRootBackendId} value={i.sceneRootBackendId}>
              {i.label || i.name || i.sceneRootBackendId}
            </option>
          ))}
        </select>
      </label>
      <div className="toolbar-row">
        <button type="button" className="btn-ghost" onClick={() => void reloadFrames()}>
          刷新
        </button>
      </div>
      <div id="frameBody">
        <fieldset className="frame-group">
          <legend>工具坐标系</legend>
          <ul className="frame-list">
            {tools.map((t) => (
              <li
                key={t.id}
                className={t.id === tool?.id ? "sel" : ""}
                onClick={() => setSelTool(t.id)}
              >
                <span className="nm">{t.name || t.id}</span>
                <input
                  type="checkbox"
                  checked={!!t.showInScene}
                  onChange={(e) => {
                    void save({
                      ...frames,
                      toolFrames: tools.map((x) => (x.id === t.id ? { ...x, showInScene: e.target.checked } : x)),
                    });
                  }}
                />
              </li>
            ))}
          </ul>
          <div className="toolbar-row frame-btns">
            <button type="button" className="btn-ghost" onClick={() => void mutateFrames(sceneRoot, "addTool").then(reloadFrames)}>
              添加
            </button>
            <button type="button" className="btn-ghost" onClick={() => tool && void mutateFrames(sceneRoot, "duplicateTool", tool.id).then(reloadFrames)}>
              复制
            </button>
            <button type="button" className="btn-ghost danger" onClick={() => tool && void mutateFrames(sceneRoot, "deleteTool", tool.id).then(reloadFrames)}>
              删除
            </button>
            <button
              type="button"
              className="btn-ghost"
              onClick={() => tool && void save({ ...frames, activeToolFrameId: tool.id })}
            >
              设为当前
            </button>
          </div>
          <label className="field">
            法兰 link
            <select
              value={tool?.flangeLinkName || frames.flangeLinkName || ""}
              onChange={(e) => {
                if (!tool) return;
                void save({
                  ...frames,
                  flangeLinkName: e.target.value,
                  toolFrames: tools.map((t) => (t.id === tool.id ? { ...t, flangeLinkName: e.target.value } : t)),
                });
              }}
            >
              {linkNames.map((n) => (
                <option key={n} value={n}>
                  {n}
                </option>
              ))}
            </select>
          </label>
          <div className="frame-spins">
            {(["X", "Y", "Z"] as const).map((lab, i) => (
              <label key={lab}>
                {lab}
                <input
                  type="number"
                  step={0.1}
                  value={tool?.T_flange_tool?.positionMm?.[i] ?? 0}
                  onChange={(e) => setSpin("tool", "p", i, Number(e.target.value))}
                />
              </label>
            ))}
            {(["Rx", "Ry", "Rz"] as const).map((lab, i) => (
              <label key={lab}>
                {lab}
                <input
                  type="number"
                  step={0.1}
                  value={tool?.T_flange_tool?.eulerDeg?.[i] ?? 0}
                  onChange={(e) => setSpin("tool", "e", i, Number(e.target.value))}
                />
              </label>
            ))}
          </div>
          <div className="toolbar-row frame-btns">
            <button type="button" className="btn-ghost" onClick={() => void captureTool(sceneRoot).then(reloadFrames)}>
              从 TCP 捕获
            </button>
            <button type="button" className="btn-ghost" onClick={() => void resetTool(sceneRoot).then(reloadFrames)}>
              重置
            </button>
          </div>
        </fieldset>

        <fieldset className="frame-group">
          <legend>用户坐标系</legend>
          <ul className="frame-list">
            {users.map((u) => (
              <li key={u.id} className={u.id === user?.id ? "sel" : ""} onClick={() => setSelUser(u.id)}>
                <span className="nm">{u.name || u.id}</span>
                <input
                  type="checkbox"
                  checked={!!u.showInScene}
                  onChange={(e) => {
                    void save({
                      ...frames,
                      userFrames: users.map((x) => (x.id === u.id ? { ...x, showInScene: e.target.checked } : x)),
                    });
                  }}
                />
              </li>
            ))}
          </ul>
          <div className="toolbar-row frame-btns">
            <button type="button" className="btn-ghost" onClick={() => void mutateFrames(sceneRoot, "addUser").then(reloadFrames)}>
              添加
            </button>
            <button type="button" className="btn-ghost" onClick={() => user && void mutateFrames(sceneRoot, "duplicateUser", user.id).then(reloadFrames)}>
              复制
            </button>
            <button type="button" className="btn-ghost danger" onClick={() => user && void mutateFrames(sceneRoot, "deleteUser", user.id).then(reloadFrames)}>
              删除
            </button>
            <button type="button" className="btn-ghost" onClick={() => user && void save({ ...frames, activeUserFrameId: user.id })}>
              设为当前
            </button>
          </div>
          <div className="frame-spins">
            {(["X", "Y", "Z"] as const).map((lab, i) => (
              <label key={lab}>
                {lab}
                <input
                  type="number"
                  step={0.1}
                  value={user?.T_base_user?.positionMm?.[i] ?? 0}
                  onChange={(e) => setSpin("user", "p", i, Number(e.target.value))}
                />
              </label>
            ))}
            {(["Rx", "Ry", "Rz"] as const).map((lab, i) => (
              <label key={lab}>
                {lab}
                <input
                  type="number"
                  step={0.1}
                  value={user?.T_base_user?.eulerDeg?.[i] ?? 0}
                  onChange={(e) => setSpin("user", "e", i, Number(e.target.value))}
                />
              </label>
            ))}
          </div>
          <div className="toolbar-row frame-btns">
            <button type="button" className="btn-ghost" onClick={() => void captureUser(sceneRoot).then(reloadFrames)}>
              从 TCP 捕获
            </button>
          </div>
        </fieldset>

        <fieldset className="frame-group">
          <legend>三维显示</legend>
          <label className="inline">
            <input
              type="checkbox"
              checked={frames.showToolFrame !== false}
              onChange={(e) => void save({ ...frames, showToolFrame: e.target.checked })}
            />{" "}
            显示工具系
          </label>
          <label className="inline">
            <input
              type="checkbox"
              checked={frames.showUserFrames !== false}
              onChange={(e) => void save({ ...frames, showUserFrames: e.target.checked })}
            />{" "}
            显示用户系
          </label>
        </fieldset>
      </div>
    </div>
  );
}
