import { useCallback, useMemo, useState } from "react";
import { discretizeGeometry, geometryOp } from "../../api/geometry";
import { dialogOpen } from "../../api/project";
import { useScene } from "../../state/sceneStore";
import { useStatus } from "../../state/statusStore";

type BooleanOp = "fuse" | "cut" | "common";

export default function GeometryPanel() {
  const { objects, selectedId, refreshObjects } = useScene();
  const { setStatus } = useStatus();
  const [busy, setBusy] = useState(false);
  const [targetEdgeLengthMm, setTargetEdgeLengthMm] = useState(2);
  const [targetTriangleCount, setTargetTriangleCount] = useState(5000);
  const [useTriangleCount, setUseTriangleCount] = useState(false);
  const [booleanOp, setBooleanOp] = useState<BooleanOp>("fuse");
  const [toolStepPath, setToolStepPath] = useState("");

  const meshes = useMemo(() => objects.filter((o) => o.hasGeometry && o.geometryKind !== 1), [objects]);
  const selected = objects.find((o) => o.id === selectedId) || null;
  const [targetId, setTargetId] = useState(selectedId || "");

  const runJob = useCallback(
    async (label: string, fn: () => Promise<{ ok: boolean; error?: string }>) => {
      setBusy(true);
      setStatus(`${label}…`);
      try {
        const r = await fn();
        setStatus(r.ok ? `${label} 完成` : r.error || `${label} 失败`, r.ok ? "info" : "err");
        if (r.ok) await refreshObjects();
        return r.ok;
      } finally {
        setBusy(false);
      }
    },
    [refreshObjects, setStatus],
  );

  const discretize = () => {
    const backendId = targetId || selectedId;
    if (!backendId) {
      setStatus("请选择目标模型", "warn");
      return;
    }
    const body: Record<string, unknown> = { backendId };
    if (useTriangleCount) body.targetTriangleCount = targetTriangleCount;
    else body.targetEdgeLengthMm = targetEdgeLengthMm;
    void runJob("离散化", () => discretizeGeometry(body));
  };

  const boolean = () => {
    const backendId = targetId || selectedId;
    if (!backendId) {
      setStatus("请选择目标模型", "warn");
      return;
    }
    if (!toolStepPath.trim()) {
      setStatus("请指定工具体 STEP 路径", "warn");
      return;
    }
    const body: Record<string, unknown> = {
      backendId,
      toolStepPath: toolStepPath.trim(),
      op: booleanOp,
      targetEdgeLengthMm,
    };
    void runJob("布尔运算", () => geometryOp(body));
  };

  return (
    <div className="dock-body" style={{ padding: 8, display: "flex", flexDirection: "column", gap: 8 }}>
      <details className="adv" open>
        <summary>离散化</summary>
        <label className="field">
          目标
          <select value={targetId || selectedId || ""} onChange={(e) => setTargetId(e.target.value)} disabled={busy}>
            <option value="">（选择场景对象）</option>
            {meshes.map((o) => (
              <option key={o.id} value={o.id}>
                {o.name || o.id}
              </option>
            ))}
          </select>
        </label>
        <label className="inline">
          <input type="checkbox" checked={useTriangleCount} onChange={(e) => setUseTriangleCount(e.target.checked)} />
          按三角面数
        </label>
        {useTriangleCount ? (
          <label className="field">
            目标三角面数
            <input
              type="number"
              min={100}
              value={targetTriangleCount}
              onChange={(e) => setTargetTriangleCount(Number(e.target.value) || 5000)}
            />
          </label>
        ) : (
          <label className="field">
            目标边长 mm
            <input
              type="number"
              min={0.1}
              step={0.1}
              value={targetEdgeLengthMm}
              onChange={(e) => setTargetEdgeLengthMm(Number(e.target.value) || 2)}
            />
          </label>
        )}
        <button type="button" disabled={busy} onClick={discretize}>
          离散化
        </button>
      </details>

      <details className="adv">
        <summary>布尔运算</summary>
        <label className="field">
          目标
          <select value={targetId || selectedId || ""} onChange={(e) => setTargetId(e.target.value)} disabled={busy}>
            <option value="">（选择场景对象）</option>
            {meshes.map((o) => (
              <option key={o.id} value={o.id}>
                {o.name || o.id}
              </option>
            ))}
          </select>
        </label>
        <label className="field">
          运算
          <select value={booleanOp} onChange={(e) => setBooleanOp(e.target.value as BooleanOp)}>
            <option value="fuse">并集 fuse</option>
            <option value="cut">差集 cut</option>
            <option value="common">交集 common</option>
          </select>
        </label>
        <label className="field">
          工具体 STEP
          <input value={toolStepPath} placeholder="路径" onChange={(e) => setToolStepPath(e.target.value)} />
        </label>
        <button
          type="button"
          className="btn-ghost"
          disabled={busy}
          onClick={async () => {
            const d = await dialogOpen({ purpose: "file", title: "选择工具体 STEP", filter: "STEP (*.step *.stp)" });
            if (d.ok && d.path) setToolStepPath(d.path);
          }}
        >
          浏览…
        </button>
        <button type="button" disabled={busy} onClick={boolean}>
          执行
        </button>
      </details>

      {selected && <div className="muted">当前选中：{selected.name || selected.id}</div>}
    </div>
  );
}
