import { useCallback, useEffect, useState } from "react";
import {
  fetchIoSignals,
  putIoSignals,
  patchIoSignalRuntime,
  resetIoSignalRuntime,
  type IoSignalRow,
} from "../../api";
import { eventHub } from "../../sse/EventHub";
import { useStatus } from "../../state/statusStore";
import { useProject } from "../../state/projectStore";

const KINDS = ["DI", "DO", "AI", "AO"] as const;

function newLocalId() {
  return `sig_${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 8)}`;
}

function defForPut(rows: IoSignalRow[]): IoSignalRow[] {
  return rows.map((r) => ({
    id: r.id || newLocalId(),
    name: r.name,
    kind: r.kind,
    port: Number(r.port) || 0,
    defaultBool: !!r.defaultBool,
    defaultAnalog: Number(r.defaultAnalog) || 0,
    description: r.description || "",
    simForceable: r.kind === "DI" ? r.simForceable !== false : false,
  }));
}

export default function SignalsPanel() {
  const { setStatus } = useStatus();
  const { onProjectChanged } = useProject();
  const [rows, setRows] = useState<IoSignalRow[]>([]);
  const [selected, setSelected] = useState(-1);
  const [busy, setBusy] = useState(false);

  const load = useCallback(async () => {
    const r = await fetchIoSignals();
    if (!r.ok) {
      setStatus(r.error || "信号表加载失败", "err");
      return;
    }
    setRows(r.signals || []);
  }, [setStatus]);

  useEffect(() => {
    void load();
  }, [load, onProjectChanged]);

  useEffect(() => {
    const off = eventHub.on("IoSignalsChanged", () => {
      void load();
    });
    return off;
  }, [load]);

  const persistDefs = async (next: IoSignalRow[]) => {
    setBusy(true);
    const r = await putIoSignals(defForPut(next));
    setBusy(false);
    if (!r.ok) {
      setStatus(r.error || "信号表保存失败", "err");
      await load();
      return;
    }
    await load();
  };

  const patchRuntime = async (row: IoSignalRow, value: string, forced?: boolean) => {
    setBusy(true);
    const body: { kind: string; port: number; value: string; forced?: boolean } = {
      kind: String(row.kind),
      port: Number(row.port) || 0,
      value,
    };
    if (forced !== undefined) body.forced = forced;
    const r = await patchIoSignalRuntime(body);
    setBusy(false);
    if (!r.ok) setStatus(r.error || "运行时更新失败", "err");
    else await load();
  };

  return (
    <div className="dock-body" id="leftSignals">
      <div className="signal-toolbar">
        <button
          type="button"
          className="btn-ghost"
          disabled={busy}
          onClick={() => {
            const next: IoSignalRow[] = [
              ...rows,
              {
                id: newLocalId(),
                name: `Signal${rows.length + 1}`,
                kind: "DI",
                port: rows.length,
                simForceable: true,
                value: "0",
                forced: false,
              },
            ];
            void persistDefs(next);
          }}
        >
          添加
        </button>
        <button
          type="button"
          className="btn-ghost"
          disabled={busy || selected < 0}
          onClick={() => {
            if (selected < 0) return;
            const next = rows.filter((_, i) => i !== selected);
            setSelected(-1);
            void persistDefs(next);
          }}
        >
          删除
        </button>
        <button
          type="button"
          className="btn-ghost"
          disabled={busy}
          onClick={async () => {
            setBusy(true);
            const r = await resetIoSignalRuntime();
            setBusy(false);
            if (!r.ok) setStatus(r.error || "重置失败", "err");
            else {
              setStatus("已重置为默认值");
              await load();
            }
          }}
        >
          重置默认
        </button>
        <button type="button" className="btn-ghost" disabled={busy} onClick={() => void load()}>
          刷新
        </button>
      </div>
      <div className="signal-table-wrap">
        <table className="signal-table">
          <thead>
            <tr>
              <th>名称</th>
              <th>类型</th>
              <th>端口</th>
              <th>值</th>
              <th>强制</th>
            </tr>
          </thead>
          <tbody>
            {rows.map((row, i) => {
              const canForce = row.kind === "DI" && row.simForceable !== false;
              return (
                <tr
                  key={row.id || `${row.name}-${i}`}
                  className={selected === i ? "sel" : ""}
                  onClick={() => setSelected(i)}
                >
                  <td>
                    <input
                      className="prop-input"
                      value={row.name}
                      disabled={busy}
                      onChange={(e) => {
                        const name = e.target.value;
                        setRows((prev) => prev.map((r, j) => (j === i ? { ...r, name } : r)));
                      }}
                      onBlur={() => void persistDefs(rows)}
                    />
                  </td>
                  <td>
                    <select
                      className="prop-input"
                      value={row.kind}
                      disabled={busy}
                      onChange={(e) => {
                        const kind = e.target.value;
                        const next = rows.map((r, j) =>
                          j === i
                            ? {
                                ...r,
                                kind,
                                simForceable: kind === "DI",
                              }
                            : r,
                        );
                        void persistDefs(next);
                      }}
                    >
                      {KINDS.map((k) => (
                        <option key={k} value={k}>
                          {k}
                        </option>
                      ))}
                    </select>
                  </td>
                  <td>
                    <input
                      className="prop-input"
                      value={String(row.port ?? 0)}
                      disabled={busy}
                      onChange={(e) => {
                        const port = Number(e.target.value) || 0;
                        setRows((prev) => prev.map((r, j) => (j === i ? { ...r, port } : r)));
                      }}
                      onBlur={() => void persistDefs(rows)}
                    />
                  </td>
                  <td>
                    <input
                      className="prop-input"
                      value={row.value ?? ""}
                      disabled={busy}
                      onChange={(e) => {
                        const value = e.target.value;
                        setRows((prev) => prev.map((r, j) => (j === i ? { ...r, value } : r)));
                      }}
                      onBlur={(e) => {
                        if (e.target.value !== (row.value ?? "")) {
                          void patchRuntime(row, e.target.value, canForce ? !!row.forced : undefined);
                        }
                      }}
                    />
                  </td>
                  <td className="signal-force">
                    <input
                      type="checkbox"
                      checked={!!row.forced}
                      disabled={busy || !canForce}
                      onChange={(e) => {
                        const forced = e.target.checked;
                        void patchRuntime(row, row.value ?? "0", forced);
                      }}
                    />
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
        {!rows.length && <p className="muted">无信号定义。点击「添加」创建。</p>}
      </div>
    </div>
  );
}
