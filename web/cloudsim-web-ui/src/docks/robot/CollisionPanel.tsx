import { useCallback, useEffect, useMemo, useState } from "react";
import {
  fetchCollisionSettings,
  putCollisionSettings,
  postCollisionPlan,
  postCollisionConfirm,
  type CollisionSettings,
} from "../../api/robot";
import { useScene } from "../../state/sceneStore";
import { useStatus } from "../../state/statusStore";
import { useProject } from "../../state/projectStore";
import { useRobotProgram } from "../../state/robotProgramStore";

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
  const { activeProgram, activeRootId, reloadPrograms } = useRobotProgram();
  const [enabled, setEnabled] = useState(false);
  const [marginMm, setMarginMm] = useState(1);
  const [whiteIds, setWhiteIds] = useState<string[]>([]);
  const [blackIds, setBlackIds] = useState<string[]>([]);
  const [poolSel, setPoolSel] = useState<string[]>([]);
  const [startId, setStartId] = useState("");
  const [endId, setEndId] = useState("");
  const [canConfirm, setCanConfirm] = useState(false);
  const [busy, setBusy] = useState(false);
  const [planStatus, setPlanStatus] = useState("");

  const motionSteps = useMemo(
    () =>
      (activeProgram?.instructions || []).filter((s) => {
        const t = String(s.type || "").toLowerCase();
        return t === "ptp" || t === "line" || t === "arc";
      }),
    [activeProgram],
  );

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
    // 设置 API 只收名单字段，勿把 plan 的起终点一并 PUT
    const settingsOnly = settingsBody(
      !!body.enabled,
      Number(body.securityMarginMm) || 0,
      body.whiteListBackendIds || [],
      body.blackListBackendIds || [],
    );
    setBusy(true);
    const r = await putCollisionSettings(settingsOnly);
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
    if (!startId || !endId || startId === endId) {
      setPlanStatus("请选择不同的起点与终点路点");
      setStatus("请选择不同的起点与终点路点", "warn");
      return;
    }
    const body: CollisionSettings = {
      ...settingsBody(enabled, marginMm, whiteIds, blackIds),
      startInstructionId: startId,
      endInstructionId: endId,
      sceneRootBackendId: activeRootId || undefined,
    };
    setBusy(true);
    setCanConfirm(false);
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
    const detail = [
      r.plannerName || "plan",
      r.pointCount != null ? `${r.pointCount} 点` : "",
      r.pathLengthTcpMm != null ? `${r.pathLengthTcpMm.toFixed(1)} mm` : "",
    ]
      .filter(Boolean)
      .join(" · ");
    setPlanStatus(detail || "规划完成");
    setCanConfirm(true);
    setStatus("碰撞路径规划完成，可确认插入");
  };

  const onConfirm = async () => {
    setBusy(true);
    const r = await postCollisionConfirm({
      sceneRootBackendId: activeRootId || undefined,
      startInstructionId: startId,
      endInstructionId: endId,
    });
    setBusy(false);
    if (!r.ok) {
      setStatus(r.error || "确认插入失败", "err");
      return;
    }
    setCanConfirm(false);
    setPlanStatus(`已插入 ${r.insertedCount ?? "?"} 段`);
    setStatus("轨迹已插入程序");
    await reloadPrograms();
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
        <div className="collision-pool">
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
        <p className="hint">选程序中两运动路点；Headless 现为 JointLerp+FK（非 OMPL 绕障）</p>
        <label className="field compact">
          起点
          <select value={startId} onChange={(e) => setStartId(e.target.value)}>
            <option value="">选择…</option>
            {motionSteps.map((s) => (
              <option key={s.id} value={s.id}>
                {(s.name || s.type || s.id).toString()}
              </option>
            ))}
          </select>
        </label>
        <label className="field compact">
          终点
          <select value={endId} onChange={(e) => setEndId(e.target.value)}>
            <option value="">选择…</option>
            {motionSteps.map((s) => (
              <option key={s.id} value={s.id}>
                {(s.name || s.type || s.id).toString()}
              </option>
            ))}
          </select>
        </label>
        <div className="toolbar-row">
          <button type="button" className="btn-run" onClick={() => void onPlan()}>
            规划
          </button>
          <button type="button" className="btn-ghost" disabled={!canConfirm} onClick={() => void onConfirm()}>
            确认插入
          </button>
        </div>
        {planStatus && <p className="hint">{planStatus}</p>}
      </fieldset>
    </div>
  );
}
