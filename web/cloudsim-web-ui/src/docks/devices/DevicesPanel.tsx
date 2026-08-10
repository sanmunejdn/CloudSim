import { useEffect, useMemo, useState } from "react";
import { fetchDeviceCatalog, type DevicePackage } from "../../api";
import { importUrdf } from "../../api/robot";
import { useStatus } from "../../state/statusStore";
import { useScene } from "../../state/sceneStore";
import { useRobotProgram } from "../../state/robotProgramStore";

export default function DevicesPanel() {
  const { setStatus } = useStatus();
  const { refreshObjects } = useScene();
  const { reloadPrograms } = useRobotProgram();
  const [types, setTypes] = useState<string[]>([]);
  const [brandsByType, setBrands] = useState<Record<string, string[]>>({});
  const [packages, setPackages] = useState<DevicePackage[]>([]);
  const [type, setType] = useState("");
  const [brand, setBrand] = useState("");
  const [hint, setHint] = useState("加载设备库…");

  const load = async () => {
    setHint("加载设备库…");
    const r = await fetchDeviceCatalog();
    if (!r.ok) {
      setHint("设备库不可用");
      return;
    }
    setTypes(r.types || []);
    setBrands(r.brandsByType || {});
    setPackages(r.packages || []);
    setType((r.types || [])[0] || "");
    setHint(`${(r.packages || []).length} 个型号`);
  };

  useEffect(() => {
    void load();
  }, []);

  const brands = brandsByType[type] || [];
  useEffect(() => {
    setBrand(brands[0] || "");
  }, [type, brands]);

  const tiles = useMemo(
    () => packages.filter((p) => (!type || p.type === type) && (!brand || p.brand === brand)),
    [packages, type, brand],
  );

  return (
    <div className="dock-body" id="leftDevices">
      <div className="device-toolbar">
        <label className="field compact">
          类型
          <select value={type} onChange={(e) => setType(e.target.value)}>
            {types.map((t) => (
              <option key={t} value={t}>
                {t}
              </option>
            ))}
          </select>
        </label>
        <label className="field compact">
          品牌
          <select value={brand} onChange={(e) => setBrand(e.target.value)}>
            {brands.map((b) => (
              <option key={b} value={b}>
                {b}
              </option>
            ))}
          </select>
        </label>
        <button type="button" className="btn-ghost" onClick={() => void load()}>
          刷新
        </button>
      </div>
      <div className="device-grid">
        {tiles.map((p) => (
          <button
            key={`${p.brand}-${p.name}-${p.urdfPath}`}
            type="button"
            className="device-tile"
            onClick={async () => {
              const r = await importUrdf(p.urdfPath);
              setStatus(r.ok ? `已导入 ${p.name}` : r.error || "导入失败", r.ok ? "info" : "err");
              await refreshObjects();
              await reloadPrograms();
            }}
          >
            {p.thumbnailUrl ? <img src={p.thumbnailUrl} alt="" /> : <div className="ph">R</div>}
            <span className="name">{p.name}</span>
          </button>
        ))}
      </div>
      <p className="hint">{hint}</p>
    </div>
  );
}
