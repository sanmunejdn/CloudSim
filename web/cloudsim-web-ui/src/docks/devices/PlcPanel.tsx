import { useCallback, useEffect, useState } from "react";
import { fetchPlcDevices, putPlcDevices, type DevicePanelEntry } from "../../api/devices";
import { useStatus } from "../../state/statusStore";
import { useProject } from "../../state/projectStore";

function newDevice(): DevicePanelEntry {
  return {
    id: `plc_${Date.now().toString(36)}`,
    name: "PLC",
    enabled: true,
    address: "192.168.0.1",
    note: "",
  };
}

export default function PlcPanel() {
  const { setStatus } = useStatus();
  const { onProjectChanged } = useProject();
  const [devices, setDevices] = useState<DevicePanelEntry[]>([]);
  const [note, setNote] = useState("");
  const [busy, setBusy] = useState(false);
  const [selected, setSelected] = useState(-1);

  const load = useCallback(async () => {
    const r = await fetchPlcDevices();
    if (!r.ok) {
      setStatus(r.error || "PLC 列表加载失败", "err");
      return;
    }
    setDevices(r.devices || []);
    setNote(r.note || "");
  }, [setStatus]);

  useEffect(() => {
    void load();
  }, [load, onProjectChanged]);

  const persist = async (next: DevicePanelEntry[]) => {
    setBusy(true);
    const r = await putPlcDevices({ devices: next });
    setBusy(false);
    if (!r.ok) {
      setStatus(r.error || "PLC 配置保存失败（后端可能仅支持 GET）", "warn");
      return;
    }
    setStatus("PLC 配置已保存");
    await load();
  };

  return (
    <div className="dock-body" id="leftPlc">
      <div className="signal-toolbar">
        <button
          type="button"
          className="btn-ghost"
          disabled={busy}
          onClick={() => {
            const next = [...devices, newDevice()];
            setDevices(next);
            setSelected(next.length - 1);
          }}
        >
          添加
        </button>
        <button
          type="button"
          className="btn-ghost"
          disabled={busy || selected < 0}
          onClick={() => {
            const next = devices.filter((_, i) => i !== selected);
            setSelected(-1);
            void persist(next);
          }}
        >
          删除
        </button>
        <button type="button" className="btn-ghost" disabled={busy} onClick={() => void persist(devices)}>
          保存
        </button>
        <button type="button" className="btn-ghost" disabled={busy} onClick={() => void load()}>
          刷新
        </button>
      </div>
      {note && <p className="hint">{note}</p>}
      <div className="device-stub-list">
        {devices.map((d, i) => (
          <div
            key={d.id || `plc-${i}`}
            className={`device-stub-row ${selected === i ? "sel" : ""}`}
            onClick={() => setSelected(i)}
          >
            <label className="field compact">
              名称
              <input
                className="prop-input"
                value={d.name || ""}
                disabled={busy}
                onChange={(e) => {
                  const name = e.target.value;
                  setDevices((prev) => prev.map((row, j) => (j === i ? { ...row, name } : row)));
                }}
              />
            </label>
            <label className="field compact">
              地址
              <input
                className="prop-input"
                value={d.address || ""}
                disabled={busy}
                onChange={(e) => {
                  const address = e.target.value;
                  setDevices((prev) => prev.map((row, j) => (j === i ? { ...row, address } : row)));
                }}
              />
            </label>
            <label className="inline">
              <input
                type="checkbox"
                checked={d.enabled !== false}
                disabled={busy}
                onChange={(e) => {
                  const enabled = e.target.checked;
                  setDevices((prev) => prev.map((row, j) => (j === i ? { ...row, enabled } : row)));
                }}
              />
              启用
            </label>
          </div>
        ))}
        {!devices.length && <p className="hint muted">暂无 PLC 设备（占位面板）</p>}
      </div>
    </div>
  );
}
