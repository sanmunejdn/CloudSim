import { useCallback, useEffect, useMemo, useState } from "react";
import {
  fetchCustomDevice,
  fetchCustomDevices,
  gotoCustomDevicePose,
  putCustomDevice,
  type CustomDeviceDetail,
  type CustomDeviceSummary,
} from "../../api/customDevices";
import { fetchIoNetwork } from "../../api/ioNetwork";
import type { IoSignalRow } from "../../api/ioSignals";
import { useDeviceRuntime } from "../../state/deviceRuntimeStore";
import { useProject } from "../../state/projectStore";
import { useScene } from "../../state/sceneStore";
import { useStatus } from "../../state/statusStore";

function newPoseId() {
  return `pose_${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 6)}`;
}

function newBindId() {
  return `bind_${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 6)}`;
}

export default function DeviceCommandPanel() {
  const { setStatus } = useStatus();
  const { onProjectChanged } = useProject();
  const { refreshObjects } = useScene();
  const { selectedCustomDeviceId, setSelectedCustomDeviceId } = useDeviceRuntime();
  const [list, setList] = useState<CustomDeviceSummary[]>([]);
  const [detail, setDetail] = useState<CustomDeviceDetail | null>(null);
  const [diSignals, setDiSignals] = useState<IoSignalRow[]>([]);
  const [goDurationSec, setGoDurationSec] = useState(1);
  const [busy, setBusy] = useState(false);

  const selectedId = selectedCustomDeviceId;

  const loadList = useCallback(async () => {
    const r = await fetchCustomDevices();
    if (!r.ok) {
      setStatus(r.error || "自定义设备列表失败", "err");
      return;
    }
    const devices = r.devices || [];
    setList(devices);
    if (!selectedId && devices.length) setSelectedCustomDeviceId(devices[0].id);
    if (selectedId && !devices.some((d) => d.id === selectedId) && devices.length) {
      setSelectedCustomDeviceId(devices[0].id);
    }
  }, [setStatus, selectedId, setSelectedCustomDeviceId]);

  const loadDetail = useCallback(async () => {
    if (!selectedId) {
      setDetail(null);
      setDiSignals([]);
      return;
    }
    const [dev, net] = await Promise.all([fetchCustomDevice(selectedId), fetchIoNetwork()]);
    if (!dev.ok) {
      setStatus(dev.error || "设备详情失败", "err");
      setDetail(null);
      return;
    }
    setDetail(dev);
    const ownerSigs = net.owners?.[selectedId]?.signals || [];
    const fromNet = ownerSigs.filter((s) => String(s.kind).toUpperCase() === "DI");
    if (fromNet.length) {
      setDiSignals(fromNet);
    } else {
      const raw = Array.isArray(dev.signals) ? (dev.signals as IoSignalRow[]) : [];
      setDiSignals(raw.filter((s) => String(s?.kind || "").toUpperCase() === "DI"));
    }
  }, [selectedId, setStatus]);

  useEffect(() => {
    void loadList();
  }, [onProjectChanged, loadList]);

  useEffect(() => {
    void loadDetail();
  }, [selectedId, loadDetail]);

  const poses = detail?.namedPoses || [];
  const bindings = detail?.poseSignalBindings || [];
  const q = detail?.q || [];

  const diOptions = useMemo(() => {
    const names = new Set(diSignals.map((s) => s.name).filter(Boolean));
    for (const b of bindings) {
      if (b.signalName) names.add(b.signalName);
    }
    return [...names];
  }, [diSignals, bindings]);

  const persist = async (next: Partial<CustomDeviceDetail> & { q?: number[] }) => {
    if (!selectedId || !detail?.ok) return false;
    setBusy(true);
    const r = await putCustomDevice(selectedId, {
      namedPoses: next.namedPoses ?? poses,
      poseSignalBindings: next.poseSignalBindings ?? bindings,
      q: next.q ?? q,
    });
    setBusy(false);
    if (!r.ok) {
      setStatus(r.error || "保存失败", "err");
      return false;
    }
    await loadDetail();
    return true;
  };

  const teachPose = async (renameId?: string) => {
    if (!detail?.ok) return;
    const defaultName = renameId
      ? poses.find((p) => p.id === renameId)?.name || "姿态"
      : `姿态${poses.length + 1}`;
    const name = window.prompt(renameId ? "重命名姿态" : "示教姿态名称", defaultName);
    if (name == null || !name.trim()) return;
    const snapshot = [...q];
    let nextPoses = [...poses];
    if (renameId) {
      nextPoses = nextPoses.map((p) =>
        p.id === renameId ? { ...p, name: name.trim(), q: snapshot } : p,
      );
    } else {
      nextPoses.push({ id: newPoseId(), name: name.trim(), q: snapshot });
    }
    if (await persist({ namedPoses: nextPoses })) setStatus(renameId ? "已更新姿态" : "已示教姿态");
  };

  return (
    <div className="robot-pane device-command-panel" id="deviceCommand">
      <label className="field">
        设备
        <select
          value={selectedId}
          onChange={(e) => setSelectedCustomDeviceId(e.target.value)}
        >
          {!list.length && <option value="">（无自定义设备）</option>}
          {list.map((d) => (
            <option key={d.id} value={d.id}>
              {d.name} ({d.axisCount} 轴)
            </option>
          ))}
        </select>
      </label>

      {!detail?.ok ? (
        <p className="hint">请先在左栏「设备」组装或打开含自定义设备的工程。</p>
      ) : (
        <>
          <div className="signal-toolbar">
            <strong>姿态库</strong>
            <button type="button" className="btn-ghost" disabled={busy} onClick={() => void teachPose()}>
              示教当前
            </button>
            <button
              type="button"
              className="btn-ghost"
              disabled={busy}
              onClick={async () => {
                const name = window.prompt("新姿态名称", `姿态${poses.length + 1}`);
                if (name == null || !name.trim()) return;
                const next = [...poses, { id: newPoseId(), name: name.trim(), q: [...q] }];
                if (await persist({ namedPoses: next })) setStatus("已新建姿态");
              }}
            >
              新建
            </button>
          </div>
          <label className="field compact">
            跳转时长 (s)
            <input
              type="number"
              min={0}
              step={0.1}
              value={goDurationSec}
              onChange={(e) => setGoDurationSec(Math.max(0, Number(e.target.value) || 0))}
            />
          </label>
          <div className="pose-list">
            {poses.map((p) => (
              <div key={p.id} className="pose-row">
                <span>{p.name}</span>
                <button
                  type="button"
                  className="btn-ghost"
                  disabled={busy}
                  onClick={async () => {
                    setBusy(true);
                    const r = await gotoCustomDevicePose(selectedId, p.id);
                    setBusy(false);
                    if (!r.ok) setStatus(r.error || "跳转失败", "err");
                    else {
                      await refreshObjects();
                      await loadDetail();
                      setStatus(`已跳转 ${p.name}`);
                    }
                  }}
                >
                  跳转
                </button>
                <button type="button" className="btn-ghost" disabled={busy} onClick={() => void teachPose(p.id)}>
                  重命名/更新
                </button>
                <button
                  type="button"
                  className="btn-ghost"
                  disabled={busy}
                  onClick={async () => {
                    if (!window.confirm(`删除姿态「${p.name}」？`)) return;
                    const nextPoses = poses.filter((x) => x.id !== p.id);
                    const nextBinds = bindings
                      .map((b) => (b.poseId === p.id ? { ...b, poseId: "" } : b))
                      .filter((b) => b.poseId);
                    if (await persist({ namedPoses: nextPoses, poseSignalBindings: nextBinds })) {
                      setStatus("已删除姿态");
                    }
                  }}
                >
                  删除
                </button>
              </div>
            ))}
            {!poses.length && <p className="hint">请先示教一个姿态。</p>}
          </div>

          <div className="signal-toolbar" style={{ marginTop: 12 }}>
            <strong>信号绑定（DI 上升沿）</strong>
          </div>
          {!diOptions.length && (
            <p className="hint">本设备尚无 DI。请到左栏「信号」为该设备 Owner 添加 DI。</p>
          )}
          <div className="pose-list bind-table">
            {bindings.map((b, idx) => (
              <div key={b.id || idx} className="pose-row bind-row">
                <label className="compact">
                  <input
                    type="checkbox"
                    checked={b.enabled !== false}
                    onChange={(e) => {
                      const next = [...bindings];
                      next[idx] = { ...b, enabled: e.target.checked };
                      setDetail({ ...detail, poseSignalBindings: next });
                    }}
                  />
                  启用
                </label>
                <select
                  value={b.signalName}
                  onChange={(e) => {
                    const next = [...bindings];
                    next[idx] = { ...b, signalName: e.target.value };
                    setDetail({ ...detail, poseSignalBindings: next });
                  }}
                >
                  <option value="">DI…</option>
                  {diOptions.map((n) => (
                    <option key={n} value={n}>
                      {n}
                    </option>
                  ))}
                </select>
                <select
                  value={b.poseId}
                  onChange={(e) => {
                    const next = [...bindings];
                    next[idx] = { ...b, poseId: e.target.value };
                    setDetail({ ...detail, poseSignalBindings: next });
                  }}
                >
                  <option value="">姿态…</option>
                  {poses.map((p) => (
                    <option key={p.id} value={p.id}>
                      {p.name}
                    </option>
                  ))}
                </select>
                <input
                  className="prop-input"
                  type="number"
                  min={0}
                  step={0.1}
                  title="durationSec"
                  value={b.durationSec ?? goDurationSec}
                  onChange={(e) => {
                    const next = [...bindings];
                    next[idx] = { ...b, durationSec: Math.max(0, Number(e.target.value) || 0) };
                    setDetail({ ...detail, poseSignalBindings: next });
                  }}
                />
                <button
                  type="button"
                  className="btn-ghost"
                  onClick={() => {
                    setDetail({
                      ...detail,
                      poseSignalBindings: bindings.filter((_, i) => i !== idx),
                    });
                  }}
                >
                  删
                </button>
              </div>
            ))}
          </div>
          <div className="toolbar-row">
            <button
              type="button"
              className="btn-ghost"
              onClick={() =>
                setDetail({
                  ...detail,
                  poseSignalBindings: [
                    ...bindings,
                    {
                      id: newBindId(),
                      signalName: diOptions[0] || "",
                      poseId: poses[0]?.id || "",
                      durationSec: goDurationSec,
                      enabled: true,
                    },
                  ],
                })
              }
            >
              添加绑定
            </button>
            <button
              type="button"
              className="btn-ghost"
              disabled={busy}
              onClick={async () => {
                if (await persist({ poseSignalBindings: bindings, namedPoses: poses })) {
                  setStatus("已保存信号绑定 / 姿态");
                }
              }}
            >
              保存
            </button>
            <button type="button" className="btn-ghost" onClick={() => void loadDetail()}>
              刷新
            </button>
          </div>
        </>
      )}
    </div>
  );
}
