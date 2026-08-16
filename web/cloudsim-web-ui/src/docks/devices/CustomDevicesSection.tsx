import { useCallback, useEffect, useState } from "react";
import { fetchCustomDevices, type CustomDeviceSummary } from "../../api/customDevices";
import { useDockNav } from "../../state/dockNavStore";
import { useDeviceRuntime } from "../../state/deviceRuntimeStore";
import { useProject } from "../../state/projectStore";
import { useScene } from "../../state/sceneStore";
import { useStatus } from "../../state/statusStore";
import CustomDeviceAssemblyDialog from "./CustomDeviceAssemblyDialog";

type Props = { onOpenCatalog?: () => void };

/** 左栏目录入口：组装 / 跳转右栏设备指令，不含运行面 */
export default function CustomDevicesSection({ onOpenCatalog }: Props) {
  const { setStatus } = useStatus();
  const { onProjectChanged } = useProject();
  const { refreshObjects } = useScene();
  const { goDeviceCmd } = useDockNav();
  const { selectedCustomDeviceId, setSelectedCustomDeviceId } = useDeviceRuntime();
  const [list, setList] = useState<CustomDeviceSummary[]>([]);
  const [assemblyOpen, setAssemblyOpen] = useState(false);
  /** 空字符串 = 新建 */
  const [assemblyDeviceId, setAssemblyDeviceId] = useState<string | undefined>(undefined);

  const loadList = useCallback(async () => {
    const r = await fetchCustomDevices();
    if (!r.ok) {
      setStatus(r.error || "自定义设备列表失败", "err");
      return;
    }
    const devices = r.devices || [];
    setList(devices);
    if (!selectedCustomDeviceId && devices.length) setSelectedCustomDeviceId(devices[0].id);
  }, [setStatus, selectedCustomDeviceId, setSelectedCustomDeviceId]);

  useEffect(() => {
    void loadList();
  }, [onProjectChanged, loadList]);

  const openCreate = () => {
    setAssemblyDeviceId(undefined);
    setAssemblyOpen(true);
  };
  const openEdit = () => {
    if (!selectedCustomDeviceId) {
      setStatus("请先选择设备", "warn");
      return;
    }
    setAssemblyDeviceId(selectedCustomDeviceId);
    setAssemblyOpen(true);
  };

  return (
    <div className="custom-device-section">
      <div className="signal-toolbar">
        <strong>工程内自定义设备</strong>
        <button type="button" className="btn-ghost" onClick={openCreate}>
          新建/组装
        </button>
        <button type="button" className="btn-ghost" onClick={() => void loadList()}>
          刷新
        </button>
        {onOpenCatalog ? (
          <button type="button" className="btn-ghost" onClick={onOpenCatalog}>
            URDF 目录
          </button>
        ) : null}
      </div>
      <label className="field compact" style={{ padding: "0 8px" }}>
        设备
        <select
          value={selectedCustomDeviceId}
          onChange={(e) => setSelectedCustomDeviceId(e.target.value)}
        >
          {!list.length && <option value="">（无）</option>}
          {list.map((d) => (
            <option key={d.id} value={d.id}>
              {d.name} ({d.axisCount} 轴)
            </option>
          ))}
        </select>
      </label>
      <div className="toolbar-row" style={{ padding: "0 8px 8px" }}>
        <button
          type="button"
          className="btn-ghost"
          disabled={!selectedCustomDeviceId}
          onClick={() => goDeviceCmd()}
        >
          打开设备指令
        </button>
        <button type="button" className="btn-ghost" disabled={!selectedCustomDeviceId} onClick={openEdit}>
          编辑组装
        </button>
      </div>
      <p className="hint" style={{ padding: "0 8px" }}>
        姿态库与 DI→姿态绑定在右栏「设备 → 自定义设备 → 设备指令」。
      </p>
      <CustomDeviceAssemblyDialog
        open={assemblyOpen}
        deviceId={assemblyDeviceId}
        onClose={() => setAssemblyOpen(false)}
        onDone={async (id) => {
          setAssemblyOpen(false);
          await loadList();
          if (id) setSelectedCustomDeviceId(id);
          await refreshObjects();
        }}
      />
    </div>
  );
}
