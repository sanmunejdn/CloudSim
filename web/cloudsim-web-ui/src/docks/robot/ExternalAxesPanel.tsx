import { useCallback, useEffect, useState } from "react";
import {
  fetchExternalAxes,
  putExternalAxes,
  type ExternalAxisConfig,
} from "../../api/robot";
import { useRobotProgram } from "../../state/robotProgramStore";
import { useStatus } from "../../state/statusStore";
import { eventHub } from "../../sse/EventHub";

function emptyAxis(): ExternalAxisConfig {
  return {
    enabled: true,
    displayName: "Rail",
    jointName: "rail_joint",
    kind: "LinearRail",
    motionType: "Translate",
    attachment: "RobotBase",
    isPrismatic: true,
    lower: 0,
    upper: 1000,
    home: 0,
    axis: [1, 0, 0],
    originMm: [0, 0, 0],
    boundBackendId: "",
    workingFrameId: "",
  };
}

export default function ExternalAxesPanel() {
  const { activeRootId } = useRobotProgram();
  const { setStatus } = useStatus();
  const [axes, setAxes] = useState<ExternalAxisConfig[]>([]);
  const [busy, setBusy] = useState(false);

  const load = useCallback(async () => {
    if (!activeRootId) {
      setAxes([]);
      return;
    }
    const r = await fetchExternalAxes(activeRootId);
    if (!r.ok) {
      setStatus(r.error || "外轴加载失败", "err");
      return;
    }
    setAxes([...(r.externalAxes?.axes || r.axes || [])]);
  }, [activeRootId, setStatus]);

  useEffect(() => {
    void load();
  }, [load]);

  useEffect(() => {
    const off = eventHub.on("RobotExternalAxesChanged", () => void load());
    return () => off();
  }, [load]);

  const save = async () => {
    if (!activeRootId) {
      setStatus("请先导入机器人", "warn");
      return;
    }
    setBusy(true);
    const r = await putExternalAxes({ sceneRootBackendId: activeRootId, axes });
    setBusy(false);
    setStatus(r.ok ? "外轴已保存" : r.error || "保存失败", r.ok ? "info" : "err");
    if (r.ok) await load();
  };

  const updateAt = (i: number, patch: Partial<ExternalAxisConfig>) => {
    setAxes((prev) => prev.map((a, idx) => (idx === i ? { ...a, ...patch } : a)));
  };

  if (!activeRootId) {
    return (
      <div className="robot-pane">
        <p className="hint muted">请先导入机器人后再配置外轴</p>
      </div>
    );
  }

  return (
    <div className="robot-pane" id="robotExtAxes">
      <div className="toolbar-row">
        <button type="button" className="btn-ghost" disabled={busy} onClick={() => void load()}>
          刷新
        </button>
        <button
          type="button"
          className="btn-ghost"
          disabled={busy}
          onClick={() => setAxes((prev) => [...prev, emptyAxis()])}
        >
          添加轴
        </button>
        <button type="button" className="btn-run" disabled={busy} onClick={() => void save()}>
          保存
        </button>
      </div>
      {!axes.length && <p className="hint muted">暂无外轴；点「添加轴」配置导轨/转台</p>}
      {axes.map((a, i) => (
        <fieldset key={i} disabled={busy} className="ext-axis-card">
          <legend>
            <label className="inline">
              <input
                type="checkbox"
                checked={!!a.enabled}
                onChange={(e) => updateAt(i, { enabled: e.target.checked })}
              />
              {a.displayName || `轴 ${i + 1}`}
            </label>
            <button
              type="button"
              className="btn-ghost danger"
              onClick={() => setAxes((prev) => prev.filter((_, j) => j !== i))}
            >
              删除
            </button>
          </legend>
          <label className="field compact">
            显示名
            <input value={a.displayName || ""} onChange={(e) => updateAt(i, { displayName: e.target.value })} />
          </label>
          <label className="field compact">
            关节名
            <input value={a.jointName || ""} onChange={(e) => updateAt(i, { jointName: e.target.value })} />
          </label>
          <label className="field compact">
            运动
            <select
              value={a.motionType || "Translate"}
              onChange={(e) =>
                updateAt(i, {
                  motionType: e.target.value,
                  isPrismatic: e.target.value === "Translate",
                  kind: e.target.value === "Translate" ? "LinearRail" : "Turntable",
                })
              }
            >
              <option value="Translate">平移</option>
              <option value="Rotate">旋转</option>
            </select>
          </label>
          <label className="field compact">
            附着
            <select
              value={a.attachment || "RobotBase"}
              onChange={(e) => updateAt(i, { attachment: e.target.value })}
            >
              <option value="RobotBase">机器人基座</option>
              <option value="Workpiece">工件</option>
            </select>
          </label>
          <div className="dlg-spins">
            <label>
              下限
              <input
                type="number"
                value={a.lower ?? 0}
                onChange={(e) => updateAt(i, { lower: Number(e.target.value) })}
              />
            </label>
            <label>
              上限
              <input
                type="number"
                value={a.upper ?? 0}
                onChange={(e) => updateAt(i, { upper: Number(e.target.value) })}
              />
            </label>
            <label>
              回零
              <input
                type="number"
                value={a.home ?? 0}
                onChange={(e) => updateAt(i, { home: Number(e.target.value) })}
              />
            </label>
          </div>
          {(a.attachment || "RobotBase") === "Workpiece" && (
            <label className="field compact">
              绑定 backendId
              <input
                value={a.boundBackendId || ""}
                onChange={(e) => updateAt(i, { boundBackendId: e.target.value })}
              />
            </label>
          )}
        </fieldset>
      ))}
    </div>
  );
}
