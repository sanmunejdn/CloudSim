import { useCallback, useEffect, useState } from "react";
import { apiJson } from "../api/client";
import { useStatus } from "../state/statusStore";

type BodyRow = {
  backendId?: string;
  name?: string;
  className?: string;
  hasGeometry?: boolean;
  featureCount?: number;
};

export default function GeomodelingShell() {
  const { setStatus } = useStatus();
  const [bodies, setBodies] = useState<BodyRow[]>([]);
  const [error, setError] = useState("");

  const reload = useCallback(async () => {
    try {
      const r = await apiJson<{ ok?: boolean; bodies?: BodyRow[]; error?: string }>("/api/geomodeling/summary");
      if (r.ok) {
        setBodies(Array.isArray(r.bodies) ? r.bodies : []);
        setError("");
      } else setError(r.error || "无几何建模数据");
    } catch {
      setError("无法连接几何建模 API");
      setBodies([]);
    }
  }, []);

  useEffect(() => {
    void reload();
  }, [reload]);

  return (
    <div className="workspace-shell geomodeling-shell">
      <header className="workspace-head">
        <h2>几何建模</h2>
        <p className="hint">GM-1 特征树 · {bodies.length} 个体</p>
      </header>
      <section className="workspace-toolbar">
        <button type="button" onClick={() => void reload()}>
          刷新
        </button>
        <button type="button" onClick={() => setStatus("请使用侧车 geometricModeling 或桌面端完整编辑")}>
          同步侧车
        </button>
      </section>
      <section className="workspace-body">
        {error ? <p className="hint">{error}</p> : null}
        <ul className="feature-tree workspace-list">
          {bodies.length ? (
            bodies.map((b) => (
              <li key={b.backendId || b.name}>
                <span className="feature-tree-name">{b.name || b.backendId}</span>
                <span className="feature-tree-meta">
                  {b.featureCount != null ? `${b.featureCount} 特征` : b.className || ""}
                  {b.hasGeometry ? " · 有几何" : ""}
                </span>
              </li>
            ))
          ) : (
            <li className="hint">暂无参数化体，导入或创建 BREP 后显示特征树</li>
          )}
        </ul>
      </section>
    </div>
  );
}
