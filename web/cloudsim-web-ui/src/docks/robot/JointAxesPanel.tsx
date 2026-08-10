import { useCallback, useEffect, useState } from "react";
import { fetchJoints, postJoints, fetchRobotInstances } from "../../api";
import { useRobotProgram } from "../../state/robotProgramStore";
import { useStatus } from "../../state/statusStore";
import { eventHub } from "../../sse/EventHub";

type Joint = { name: string; angleRad: number; lowerRad?: number; upperRad?: number };

export default function JointAxesPanel() {
  const { activeRootId } = useRobotProgram();
  const { setStatus } = useStatus();
  const [instances, setInstances] = useState<{ sceneRootBackendId: string; name?: string; label?: string }[]>([]);
  const [root, setRoot] = useState(activeRootId);
  const [joints, setJoints] = useState<Joint[]>([]);
  const [csv, setCsv] = useState("");

  const reload = useCallback(async () => {
    const inst = await fetchRobotInstances();
    setInstances(inst.instances || []);
    const id = root || activeRootId || inst.instances?.[0]?.sceneRootBackendId || "";
    if (id && id !== root) setRoot(id);
    if (!id) return;
    const r = await fetchJoints(id);
    if (r.ok) {
      const js = r.joints || [];
      setJoints(js);
      setCsv(js.map((j) => j.angleRad.toFixed(4)).join(","));
    }
  }, [root, activeRootId]);

  useEffect(() => {
    void reload();
  }, [reload]);

  useEffect(() => {
    return eventHub.on("RobotKinematicsApplied", () => {
      void reload();
    });
  }, [reload]);

  const apply = async (next: Joint[]) => {
    setJoints(next);
    if (!root) return;
    const r = await postJoints(
      root,
      next.map((j) => j.angleRad),
    );
    if (!r.ok) setStatus(r.error || "关节应用失败", "err");
    setCsv(next.map((j) => j.angleRad.toFixed(4)).join(","));
  };

  return (
    <div className="robot-pane" id="robotJoint">
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
        <button type="button" className="btn-ghost" onClick={() => void reload()}>
          刷新
        </button>
        <button type="button" className="btn-ghost" onClick={() => void apply(joints.map((j) => ({ ...j, angleRad: 0 })))}>
          全部重置
        </button>
      </div>
      <div className="axis-joints">
        {joints.map((j, idx) => {
          const lo = j.lowerRad ?? -Math.PI;
          const hi = j.upperRad ?? Math.PI;
          const deg = (j.angleRad * 180) / Math.PI;
          return (
            <div key={j.name || idx} className="axis-row">
              <div className="axis-row-head">
                <span className="jn">{j.name || `J${idx + 1}`}</span>
                <span>{deg.toFixed(1)}°</span>
              </div>
              <input
                type="range"
                min={lo}
                max={hi}
                step={0.001}
                value={j.angleRad}
                onChange={(e) => {
                  const v = Number(e.target.value);
                  void apply(joints.map((x, i) => (i === idx ? { ...x, angleRad: v } : x)));
                }}
              />
              <div className="axis-row-vals">
                <input
                  type="number"
                  step={0.001}
                  value={Number(j.angleRad.toFixed(4))}
                  onChange={(e) => {
                    const v = Number(e.target.value);
                    void apply(joints.map((x, i) => (i === idx ? { ...x, angleRad: v } : x)));
                  }}
                />
                <input type="number" readOnly value={Number(deg.toFixed(2))} title="度" />
                <button type="button" onClick={() => void apply(joints.map((x, i) => (i === idx ? { ...x, angleRad: 0 } : x)))}>
                  复位
                </button>
              </div>
            </div>
          );
        })}
      </div>
      <details className="adv">
        <summary>高级：CSV</summary>
        <label className="field">
          关节角 rad (CSV)
          <input value={csv} onChange={(e) => setCsv(e.target.value)} placeholder="0,0,0,0,0,0" />
        </label>
        <button
          type="button"
          onClick={async () => {
            if (!root) return;
            const q = csv
              .split(",")
              .map((s) => Number(s.trim()))
              .filter((n) => !Number.isNaN(n));
            const r = await postJoints(root, q);
            setStatus(r.ok ? "关节已应用" : r.error || "失败", r.ok ? "info" : "err");
            await reload();
          }}
        >
          应用关节
        </button>
      </details>
    </div>
  );
}
