import { useCallback, useEffect, useState } from "react";
import { fetchJoints, postJoints, fetchRobotInstances } from "../../api";
import {
  applyCustomDeviceQ,
  fetchCustomDevice,
  fetchCustomDevices,
  type CustomDeviceSummary,
} from "../../api/customDevices";
import { useDeviceRuntime } from "../../state/deviceRuntimeStore";
import { useRobotProgram } from "../../state/robotProgramStore";
import { useStatus } from "../../state/statusStore";
import { eventHub } from "../../sse/EventHub";

type Joint = { name: string; angleRad: number; lowerRad?: number; upperRad?: number };

type TargetKind = "robot" | "device";

type Props = {
  /** 自定义设备 mode 下优先选中当前设备 */
  preferCustomDevice?: boolean;
};

export default function JointAxesPanel({ preferCustomDevice = false }: Props) {
  const { activeRootId } = useRobotProgram();
  const { selectedCustomDeviceId, setSelectedCustomDeviceId } = useDeviceRuntime();
  const { setStatus } = useStatus();
  const [instances, setInstances] = useState<{ sceneRootBackendId: string; name?: string; label?: string }[]>([]);
  const [devices, setDevices] = useState<CustomDeviceSummary[]>([]);
  const [targetKind, setTargetKind] = useState<TargetKind>(preferCustomDevice ? "device" : "robot");
  const [root, setRoot] = useState(activeRootId);
  const [deviceId, setDeviceId] = useState(selectedCustomDeviceId);
  const [joints, setJoints] = useState<Joint[]>([]);
  const [csv, setCsv] = useState("");

  useEffect(() => {
    if (preferCustomDevice) setTargetKind("device");
  }, [preferCustomDevice]);

  useEffect(() => {
    if (selectedCustomDeviceId && preferCustomDevice) setDeviceId(selectedCustomDeviceId);
  }, [selectedCustomDeviceId, preferCustomDevice]);

  const reload = useCallback(async () => {
    const [inst, devList] = await Promise.all([fetchRobotInstances(), fetchCustomDevices()]);
    setInstances(inst.instances || []);
    setDevices(devList.devices || []);

    if (targetKind === "device") {
      const id =
        deviceId ||
        selectedCustomDeviceId ||
        (devList.devices || [])[0]?.id ||
        "";
      if (id && id !== deviceId) setDeviceId(id);
      if (!id) {
        setJoints([]);
        setCsv("");
        return;
      }
      const r = await fetchCustomDevice(id);
      if (!r.ok) {
        setJoints([]);
        return;
      }
      const q = r.q || [];
      const js: Joint[] = q.map((angleRad, i) => ({
        name: `q${i}`,
        angleRad,
        lowerRad: -Math.PI,
        upperRad: Math.PI,
      }));
      setJoints(js);
      setCsv(js.map((j) => j.angleRad.toFixed(4)).join(","));
      return;
    }

    const id = root || activeRootId || inst.instances?.[0]?.sceneRootBackendId || "";
    if (id && id !== root) setRoot(id);
    if (!id) return;
    const r = await fetchJoints(id);
    if (r.ok) {
      const js = r.joints || [];
      setJoints(js);
      setCsv(js.map((j) => j.angleRad.toFixed(4)).join(","));
    }
  }, [root, activeRootId, targetKind, deviceId, selectedCustomDeviceId]);

  useEffect(() => {
    void reload();
  }, [reload]);

  useEffect(() => {
    return eventHub.on("RobotKinematicsApplied", () => {
      if (targetKind === "robot") void reload();
    });
  }, [reload, targetKind]);

  const apply = async (next: Joint[]) => {
    setJoints(next);
    const q = next.map((j) => j.angleRad);
    setCsv(q.map((v) => v.toFixed(4)).join(","));
    if (targetKind === "device") {
      if (!deviceId) return;
      const r = await applyCustomDeviceQ(deviceId, q);
      if (!r.ok) setStatus(r.error || "设备轴应用失败", "err");
      return;
    }
    if (!root) return;
    const r = await postJoints(root, q);
    if (!r.ok) setStatus(r.error || "关节应用失败", "err");
  };

  return (
    <div className="robot-pane" id="robotJoint">
      <label className="field">
        目标
        <select
          value={targetKind === "robot" ? `robot:${root}` : `device:${deviceId}`}
          onChange={(e) => {
            const v = e.target.value;
            if (v.startsWith("device:")) {
              setTargetKind("device");
              const id = v.slice("device:".length);
              setDeviceId(id);
              setSelectedCustomDeviceId(id);
            } else {
              setTargetKind("robot");
              setRoot(v.slice("robot:".length));
            }
          }}
        >
          <optgroup label="机器人">
            {instances.map((i) => (
              <option key={i.sceneRootBackendId} value={`robot:${i.sceneRootBackendId}`}>
                {i.label || i.name || i.sceneRootBackendId}
              </option>
            ))}
            {!instances.length && <option value="robot:">（无机器人）</option>}
          </optgroup>
          <optgroup label="自定义设备">
            {devices.map((d) => (
              <option key={d.id} value={`device:${d.id}`}>
                {d.name} ({d.axisCount} 轴)
              </option>
            ))}
            {!devices.length && <option value="device:">（无自定义设备）</option>}
          </optgroup>
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
            const q = csv
              .split(",")
              .map((s) => Number(s.trim()))
              .filter((n) => !Number.isNaN(n));
            if (targetKind === "device") {
              if (!deviceId) return;
              const r = await applyCustomDeviceQ(deviceId, q);
              setStatus(r.ok ? "设备轴已应用" : r.error || "失败", r.ok ? "info" : "err");
            } else {
              if (!root) return;
              const r = await postJoints(root, q);
              setStatus(r.ok ? "关节已应用" : r.error || "失败", r.ok ? "info" : "err");
            }
            await reload();
          }}
        >
          应用关节
        </button>
      </details>
    </div>
  );
}
