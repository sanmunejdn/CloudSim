import { useCallback, useEffect, useState } from "react";
import {
  fetchIoNetwork,
  putOwnerSignals,
  patchIoNetworkRuntime,
  resetIoNetworkRuntime,
  type IoNetworkOwner,
  type IoSignalRow,
} from "../../api";
import { eventHub } from "../../sse/EventHub";
import { useStatus } from "../../state/statusStore";
import { useProject } from "../../state/projectStore";
import SignalsConnectionStationDialog from "./SignalsConnectionStationDialog";

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
  const [owners, setOwners] = useState<Record<string, IoNetworkOwner>>({});
  const [ownerId, setOwnerId] = useState("");
  const [rows, setRows] = useState<IoSignalRow[]>([]);
  const [selected, setSelected] = useState(-1);
  const [busy, setBusy] = useState(false);
  const [stationOpen, setStationOpen] = useState(false);

  const load = useCallback(async () => {
    const r = await fetchIoNetwork();
    if (!r.ok) {
      setStatus(r.error || "信号网加载失败", "err");
      return;
    }
    const own = r.owners || {};
    setOwners(own);
    const ids = Object.keys(own);
    const prefer = ownerId && own[ownerId] ? ownerId : r.primaryOwnerId || ids[0] || "";
    setOwnerId(prefer);
    setRows(prefer && own[prefer] ? own[prefer].signals || [] : []);
  }, [setStatus, ownerId]);

  useEffect(() => {
    void load();
  }, [onProjectChanged]);

  useEffect(() => {
    const off1 = eventHub.on("IoSignalsChanged", () => {
      void load();
    });
    const off2 = eventHub.on("IoNetworkChanged", () => {
      void load();
    });
    return () => {
      off1();
      off2();
    };
  }, [load]);

  useEffect(() => {
    if (ownerId && owners[ownerId]) setRows(owners[ownerId].signals || []);
  }, [ownerId, owners]);

  const persistDefs = async (next: IoSignalRow[]) => {
    if (!ownerId) {
      setStatus("无 Owner，请先导入机器人或自定义设备", "warn");
      return;
    }
    setBusy(true);
    const r = await putOwnerSignals(ownerId, defForPut(next));
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
    const body: { ownerId: string; kind: string; port: number; value: string; forced?: boolean } = {
      ownerId,
      kind: String(row.kind),
      port: Number(row.port) || 0,
      value,
    };
    if (forced !== undefined) body.forced = forced;
    const r = await patchIoNetworkRuntime(body);
    setBusy(false);
    if (!r.ok) setStatus(r.error || "运行时更新失败", "err");
    else await load();
  };

  const ownerEntries = Object.entries(owners);

  return (
    <div className="dock-body" id="leftSignals">
      <div className="signal-toolbar">
        <label className="field compact">
          Owner
          <select
            value={ownerId}
            disabled={busy || ownerEntries.length === 0}
            onChange={(e) => setOwnerId(e.target.value)}
          >
            {ownerEntries.map(([id, o]) => (
              <option key={id} value={id}>
                {o.displayName || id} ({o.kind})
              </option>
            ))}
          </select>
        </label>
        <button type="button" className="btn-ghost" disabled={busy} onClick={() => setStationOpen(true)}>
          信号连接站
        </button>
        <button
          type="button"
          className="btn-ghost"
          disabled={busy || !ownerId}
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
            const r = await resetIoNetworkRuntime(ownerId || undefined);
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
        <div className="signal-grid" role="table" aria-label="信号表">
          <div className="signal-grid-row head" role="row">
            <div className="col-name" role="columnheader">
              名称
            </div>
            <div className="col-kind" role="columnheader">
              类型
            </div>
            <div className="col-port" role="columnheader">
              端口
            </div>
            <div className="col-value" role="columnheader">
              值
            </div>
            <div className="col-force" role="columnheader" title="强制">
              强
            </div>
          </div>
          {rows.map((row, i) => {
            const canForce = row.kind === "DI" && row.simForceable !== false;
            return (
              <div
                key={row.id || `${row.name}-${i}`}
                className={`signal-grid-row ${selected === i ? "sel" : ""}`}
                role="row"
                onClick={() => setSelected(i)}
              >
                <div className="col-name" role="cell">
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
                </div>
                <div className="col-kind" role="cell">
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
                </div>
                <div className="col-port" role="cell">
                  <input
                    className="prop-input"
                    type="number"
                    value={row.port}
                    disabled={busy}
                    onChange={(e) => {
                      const port = Number(e.target.value) || 0;
                      setRows((prev) => prev.map((r, j) => (j === i ? { ...r, port } : r)));
                    }}
                    onBlur={() => void persistDefs(rows)}
                  />
                </div>
                <div className="col-value" role="cell">
                  {row.kind === "DI" || row.kind === "DO" ? (
                    <button
                      type="button"
                      className="btn-ghost"
                      disabled={busy}
                      onClick={() => void patchRuntime(row, row.value === "1" ? "0" : "1")}
                    >
                      {row.value ?? "-"}
                    </button>
                  ) : (
                    <input
                      className="prop-input"
                      value={row.value ?? ""}
                      disabled={busy}
                      onBlur={(e) => void patchRuntime(row, e.target.value)}
                      onChange={(e) => {
                        const value = e.target.value;
                        setRows((prev) => prev.map((r, j) => (j === i ? { ...r, value } : r)));
                      }}
                    />
                  )}
                </div>
                <div className="col-force signal-force" role="cell">
                  {canForce ? (
                    <input
                      type="checkbox"
                      checked={!!row.forced}
                      disabled={busy}
                      onChange={(e) =>
                        void patchRuntime(row, row.value === "1" ? "1" : "0", e.target.checked)
                      }
                    />
                  ) : (
                    "-"
                  )}
                </div>
              </div>
            );
          })}
        </div>
      </div>
      <SignalsConnectionStationDialog open={stationOpen} onClose={() => setStationOpen(false)} />
    </div>
  );
}
