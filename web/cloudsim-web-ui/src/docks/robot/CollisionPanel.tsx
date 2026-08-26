import { useCallback, useEffect, useMemo, useState } from "react";
import {
  fetchCollisionSettings,
  putCollisionSettings,
  postCollisionPlan,
  type CollisionSettings,
} from "../../api/robot";
import { useScene } from "../../state/sceneStore";
import { useStatus } from "../../state/statusStore";
import { useProject } from "../../state/projectStore";

function settingsBody(
  enabled: boolean,
  securityMarginMm: number,
  whiteListBackendIds: string[],
  blackListBackendIds: string[],
): CollisionSettings {
  return { enabled, securityMarginMm, whiteListBackendIds, blackListBackendIds };
}

export default function CollisionPanel() {
  const { objects } = useScene();
  const { setStatus } = useStatus();
  const { onProjectChanged } = useProject();
  const [enabled, setEnabled] = useState(false);
  const [marginMm, setMarginMm] = useState(1);
  const [whiteIds, setWhiteIds] = useState<string[]>([]);
  const [blackIds, setBlackIds] = useState<string[]>([]);
  const [poolSel, setPoolSel] = useState<string[]>([]);
  const [busy, setBusy] = useState(false);
  const [planStatus, setPlanStatus] = useState("");

  const pool = useMemo(
    () =>
      objects
        .filter((o) => o.hasGeometry)
        .map((o) => ({ id: o.id, label: o.name || o.id })),
    [objects],
  );

  const labelById = useMemo(() => {
    const m = new Map<string, string>();
    for (const o of pool) m.set(o.id, o.label);
    return m;
  }, [pool]);

  const load = useCallback(async () => {
    const r = await fetchCollisionSettings();
    if (!r.ok) {
      setStatus(r.error || "碰撞设置加载失败", "err");
      return;
    }
    setEnabled(!!r.enabled);
    setMarginMm(Number(r.securityMarginMm) || 1);
    setWhiteIds([...(r.whiteListBackendIds || [])]);
    setBlackIds([...(r.blackListBackendIds || [])]);
  }, [setStatus]);

  useEffect(() => {
    void load();
  }, [load, onProjectChanged]);

  const persist = async (next?: CollisionSettings) => {
    const body =
      next ??
      settingsBody(
        enabled,
        marginMm,
        whiteIds,
        blackIds,
      );
    setBusy(true);
    const r = await putCollisionSettings(body);
    setBusy(false);
    if (!r.ok) {
      setStatus(r.error || "碰撞设置保存失败", "err");
      return false;
    }
    await load();
    return true;
  };

  const moveToList = (toWhite: boolean) => {
    if (!poolSel.length) return;
    const add = new Set(toWhite ? whiteIds : blackIds);
    const other = new Set(toWhite ? blackIds : whiteIds);
    for (const id of poolSel) {
      add.add(id);
      other.delete(id);
    }
    if (toWhite) {
      setWhiteIds([...add]);
      setBlackIds([...other]);
    } else {
      setBlackIds([...add]);
      setWhiteIds([...other]);
    }
    void persist(
      settingsBody(
        enabled,
        marginMm,
        toWhite ? [...add] : [...other],
        toWhite ? [...other] : [...add],
      ),
    );
  };

  const removeFromList = (fromWhite: boolean) => {
    if (fromWhite) {
      const next = whiteIds.filter((id) => !poolSel.includes(id));
      setWhiteIds(next);
      void persist(settingsBody(enabled, marginMm, next, blackIds));
    } else {
      const next = blackIds.filter((id) => !poolSel.includes(id));
      setBlackIds(next);
      void persist(settingsBody(enabled, marginMm, whiteIds, next));
    }
  };

  const onPlan = async () => {
    const body = settingsBody(enabled, marginMm, whiteIds, blackIds);
    setBusy(true);
    const saved = await persist(body);
    if (!saved) {
      setBusy(false);
      return;
    }
    const r = await postCollisionPlan(body);
    setBusy(false);
    if (r.routeMissing) {
      setPlanStatus("规划 API 未就绪，已仅保存设置");
      setStatus("碰撞规划路由未上线，已保存设置", "warn");
      return;
    }
    if (!r.ok) {
      setPlanStatus(r.error || "规划失败");
      setStatus(r.error || "碰撞规划失败", "err");
      return;
    }
    setPlanStatus("规划完成");
    setStatus("碰撞路径规划完成");
  };

  return (
    <div className="robot-pane collision-pane" id="robotCollision">
      <fieldset disabled={busy}>
        <legend>碰撞检测</legend>
        <label className="inline">
          <input type="checkbox" checked={enabled} onChange={(e) => setEnabled(e.target.checked)} />
          启用碰撞检测
        </label>
        <label className="field compact">
          安全余量 (mm)
          <input
            type="number"
            min={0}
            max={500}
            step={0.5}
            value={marginMm}
            onChange={(e) => setMarginMm(Number(e.target.value) || 0)}
          />
        </label>
        <p className="hint">白名单组内互不检；黑名单组内互不检；仅黑白名单之间互检</p>

        <div className="section-title">场景对象池</div>
        <div
          className="collision-pool"
          onDragOver={(e) => {
            e.preventDefault();
            e.dataTransfer.dropEffect = "none";
          }}
        >
          {pool.map((o) => (
            <label key={o.id} className="collision-pool-item">
              <input
                type="checkbox"
                checked={poolSel.includes(o.id)}
                onChange={(e) => {
                  setPoolSel((prev) =>
                    e.target.checked ? [...prev, o.id] : prev.filter((id) => id !== o.id),
                  );
                }}
              />
              <span title={o.id}>{o.label}</span>
            </label>
          ))}
          {!pool.length && <p className="hint muted">无可检几何对象</p>}
        </div>

        <div className="toolbar-row">
          <button type="button" className="btn-ghost" disabled={!poolSel.length} onClick={() => moveToList(true)}>
            → 白名单
          </button>
          <button type="button" className="btn-ghost" disabled={!poolSel.length} onClick={() => moveToList(false)}>
            → 黑名单
          </button>
          <button type="button" className="btn-ghost" onClick={() => void load()}>
            刷新
          </button>
          <button type="button" className="btn-ghost" onClick={() => void persist()}>
            保存
          </button>
        </div>

        <div className="collision-lists">
          <div className="collision-list-col">
            <div className="section-title">白名单</div>
            <ul className="collision-id-list">
              {whiteIds.map((id) => (
                <li key={id}>
                  <label>
                    <input
                      type="checkbox"
                      checked={poolSel.includes(id)}
                      onChange={(e) => {
                        setPoolSel((prev) =>
                          e.target.checked ? [...prev, id] : prev.filter((x) => x !== id),
                        );
                      }}
                    />
                    {labelById.get(id) || id}
                  </label>
                </li>
              ))}
            </ul>
            <button type="button" className="btn-ghost" disabled={!poolSel.length} onClick={() => removeFromList(true)}>
              移除选中
            </button>
          </div>
          <div className="collision-list-col">
            <div className="section-title">黑名单</div>
            <ul className="collision-id-list">
              {blackIds.map((id) => (
                <li key={id}>
                  <label>
                    <input
                      type="checkbox"
                      checked={poolSel.includes(id)}
                      onChange={(e) => {
                        setPoolSel((prev) =>
                          e.target.checked ? [...prev, id] : prev.filter((x) => x !== id),
                        );
                      }}
                    />
                    {labelById.get(id) || id}
                  </label>
                </li>
              ))}
            </ul>
            <button type="button" className="btn-ghost" disabled={!poolSel.length} onClick={() => removeFromList(false)}>
              移除选中
            </button>
          </div>
        </div>
      </fieldset>

      <fieldset disabled={busy}>
        <legend>路径规划</legend>
        <p className="hint">规划前请先保存碰撞设置；若后端未提供规划路由则仅保存设置</p>
        <div className="toolbar-row">
          <button type="button" className="btn-run" onClick={() => void onPlan()}>
            规划
          </button>
        </div>
        {planStatus && <p className="hint">{planStatus}</p>}
      </fieldset>
    </div>
  );
}
