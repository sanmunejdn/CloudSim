import { useCallback, useEffect, useState } from "react";
import { fetchAnnotations, putAnnotations, type AnnotationRow } from "../../api/sidecar";
import { useStatus } from "../../state/statusStore";

function rowId(row: AnnotationRow, index: number) {
  return row.id || `ann-${index}`;
}

export default function AnnotationsPanel() {
  const { setStatus } = useStatus();
  const [rows, setRows] = useState<AnnotationRow[]>([]);
  const [dirty, setDirty] = useState(false);

  const reload = useCallback(async () => {
    const data = await fetchAnnotations();
    setRows(Array.isArray(data) ? data : []);
    setDirty(false);
  }, []);

  useEffect(() => {
    void reload();
  }, [reload]);

  const save = async () => {
    const r = await putAnnotations(rows);
    if (r.ok) {
      setStatus("装配标注已保存");
      setDirty(false);
      await reload();
    } else setStatus(r.error || "保存失败", "err");
  };

  const updateRow = (index: number, patch: Partial<AnnotationRow>) => {
    setRows((prev) => prev.map((r, i) => (i === index ? { ...r, ...patch } : r)));
    setDirty(true);
  };

  const addRow = () => {
    setRows((prev) => [...prev, { id: `ann-${Date.now()}`, title: "新标注", note: "" }]);
    setDirty(true);
  };

  const removeRow = (index: number) => {
    setRows((prev) => prev.filter((_, i) => i !== index));
    setDirty(true);
  };

  return (
    <div className="dock-body annotations-panel">
      <div className="signal-toolbar">
        <button type="button" className="btn-ghost" onClick={() => void reload()}>
          刷新
        </button>
        <button type="button" className="btn-ghost" onClick={addRow}>
          添加
        </button>
        <button type="button" className="btn-primary" disabled={!dirty} onClick={() => void save()}>
          保存
        </button>
      </div>
      {rows.length === 0 ? (
        <p className="hint pad">暂无装配标注，点击「添加」创建</p>
      ) : (
        <ul className="workspace-list">
          {rows.map((row, i) => (
            <li key={rowId(row, i)} className="annotation-row">
              <input
                className="prop-input"
                value={row.title ?? ""}
                placeholder="标题"
                onChange={(e) => updateRow(i, { title: e.target.value })}
              />
              <input
                className="prop-input"
                value={row.note ?? ""}
                placeholder="说明"
                onChange={(e) => updateRow(i, { note: e.target.value })}
              />
              <button type="button" className="btn-ghost danger" onClick={() => removeRow(i)}>
                删除
              </button>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}
