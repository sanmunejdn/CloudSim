import { useEffect, useMemo, useRef, useState } from "react";
import {
  createPathPlan,
  trajDiscretize,
  trajMeshSpec,
  trajPreviewRaw,
  trajFeatureSchema,
} from "../../api";
import { useTrajectory } from "../../state/trajectoryStore";
import { useRobotProgram } from "../../state/robotProgramStore";
import { useScene } from "../../state/sceneStore";
import { useStatus } from "../../state/statusStore";
import {
  filterStrategies,
  isFaceStrategy,
  normalizeFeatureStrategy,
  resolveFeatureStrategy,
  strategyFilterForFeature,
  type StrategyInfo,
} from "./featureStrategy";
import { makeFeatureParams } from "./featureSchema";
import FeatureParamForm from "./FeatureParamForm";
import { publishRawPreview, type RawPreviewPayload } from "../../scene/rawPreview";

function parseCsv3(s: string): number[] {
  return String(s || "")
    .split(",")
    .map((x) => Number(x.trim()) || 0)
    .concat([0, 0, 0])
    .slice(0, 3);
}

function strategyLabel(id: string, catalog: StrategyInfo[]) {
  return catalog.find((s) => s.strategyId === id)?.displayNameZh || id || "—";
}

export default function TrajectoryGenPanel() {
  const {
    featureEditActive,
    beginEdit,
    cancelEdit,
    pathPlanId,
    pathPlans,
    reloadPathPlans,
    bindPlan,
    pickMode,
    setPickMode,
    features,
    setFeatures,
    workpieceId,
    setWorkpieceId,
    syncSession,
    editUiEpoch,
  } = useTrajectory();
  const { activeRootId, reloadPrograms, setSelectedInstrId } = useRobotProgram();
  const { selectedId, objects } = useScene();
  const { setStatus } = useStatus();
  const [sub, setSub] = useState<"cad" | "mesh">("cad");
  const [appendMode, setAppendMode] = useState(true);
  const [featSel, setFeatSel] = useState(-1);
  const [strategies, setStrategies] = useState<StrategyInfo[]>([]);
  const [strategyId, setStrategyId] = useState("");
  const [meshMethod, setMeshMethod] = useState("CrossSection");
  const [meshOrigin, setMeshOrigin] = useState("0,0,50");
  const [meshNormal, setMeshNormal] = useState("0,0,1");
  const [meshStep, setMeshStep] = useState(2);
  const [meshTri, setMeshTri] = useState("");
  const [axisX, setAxisX] = useState(true);
  const [axisY, setAxisY] = useState(true);
  const [axisZ, setAxisZ] = useState(true);
  const [axisInterval, setAxisInterval] = useState(0);
  const [lastRawPreview, setLastRawPreview] = useState<RawPreviewPayload | null>(null);
  const autoDiscTimer = useRef<number | null>(null);
  const featuresRef = useRef(features);
  const editActiveRef = useRef(featureEditActive);
  const workpieceRef = useRef(workpieceId);
  const strategyRef = useRef(strategyId);
  const strategiesRef = useRef(strategies);
  featuresRef.current = features;
  editActiveRef.current = featureEditActive;
  workpieceRef.current = workpieceId;
  strategyRef.current = strategyId;
  strategiesRef.current = strategies;

  const visibleStrategies = useMemo(() => {
    const filter =
      pickMode === "face"
        ? "Face"
        : pickMode === "edge"
          ? "Line"
          : strategyFilterForFeature(featSel >= 0 ? features[featSel] : undefined);
    const list = filterStrategies(strategies, filter);
    return list.length ? list : strategies;
  }, [strategies, pickMode, featSel, features]);

  useEffect(() => {
    if (activeRootId) void reloadPathPlans(activeRootId);
    void syncSession();
  }, [activeRootId, reloadPathPlans, syncSession]);

  // 应用/生成/取消后重置本地面板（特征表选择、追加模式、预览缓存）
  useEffect(() => {
    if (editUiEpoch === 0) return;
    if (autoDiscTimer.current) {
      window.clearTimeout(autoDiscTimer.current);
      autoDiscTimer.current = null;
    }
    setFeatSel(-1);
    setAppendMode(false);
    setLastRawPreview(null);
  }, [editUiEpoch]);

  useEffect(() => {
    if (!workpieceId && selectedId) setWorkpieceId(selectedId);
  }, [selectedId, workpieceId, setWorkpieceId]);

  useEffect(() => {
    void trajFeatureSchema().then((r) => {
      const cat = (r.strategies || []) as StrategyInfo[];
      if (!Array.isArray(cat) || !cat.length) return;
      setStrategies(cat);
      setStrategyId((prev) => prev || cat[0].strategyId);
    });
  }, [workpieceId]);

  useEffect(() => {
    if (featSel < 0 || !features[featSel]) return;
    const f = features[featSel];
    const sid = normalizeFeatureStrategy(f, strategies);
    if (sid !== f.strategyId) {
      const next = [...features];
      next[featSel] = { ...f, strategyId: sid };
      setFeatures(next);
    }
    setStrategyId(sid);
  }, [featSel]); // 仅行切换时同步下拉，避免与拾取写入打架

  const activatePick = (mode: "face" | "edge") => {
    const next = pickMode === mode ? null : mode;
    setPickMode(next);
    if (!next) return;
    const sid = resolveFeatureStrategy(next, strategyId, strategies);
    setStrategyId(sid);
    setStatus(next === "face" ? "拾取面：在视口点击 BREP 面" : "拾取线：在视口点击 BREP 边");
  };

  useEffect(() => {
    const onCommit = (ev: Event) => {
      if (!featureEditActive || !pickMode) return;
      const d = (ev as CustomEvent).detail as {
        workpieceBackendId?: string;
        result?: { faceIndex?: number; edgeIndex?: number };
      };
      if (d.workpieceBackendId) setWorkpieceId(d.workpieceBackendId);
      const faceIndex = d.result?.faceIndex;
      const edgeIndex = d.result?.edgeIndex;
      const strategy = resolveFeatureStrategy(pickMode, strategyId, strategies);
      setStrategyId(strategy);
      const geom = {
        faceIndices: pickMode === "face" && faceIndex != null && faceIndex >= 0 ? [faceIndex] : [],
        edgeIndices: pickMode === "edge" && edgeIndex != null && edgeIndex >= 0 ? [edgeIndex] : [],
      };
      const pickedKind = pickMode === "face" ? "面" : "线";
      if (appendMode && featSel >= 0 && features[featSel]) {
        const next = [...features];
        const cur = { ...next[featSel], geometry: { ...(next[featSel].geometry || {}) } };
        cur.geometry!.faceIndices = [...(cur.geometry!.faceIndices || []), ...geom.faceIndices];
        cur.geometry!.edgeIndices = [...(cur.geometry!.edgeIndices || []), ...geom.edgeIndices];
        if (pickMode === "face" && !isFaceStrategy(cur.strategyId, strategies)) cur.strategyId = strategy;
        if (pickMode === "edge" && isFaceStrategy(cur.strategyId, strategies)) cur.strategyId = strategy;
        next[featSel] = cur;
        setFeatures(next);
        setStatus(`拾取${pickedKind} ok · ${strategyLabel(strategy, strategies)}`);
      } else {
        void (async () => {
          const params = await makeFeatureParams(strategy);
          const feat = {
            featureId: `F${features.length + 1}`,
            strategyId: strategy,
            geometry: { ...geom, polylineXyz: [] as number[] },
            params,
          };
          setFeatures([...features, feat]);
          setFeatSel(features.length);
          setAppendMode(false);
          setStatus(`已添加特征 ${feat.featureId} · ${strategyLabel(strategy, strategies)}`);
          scheduleAutoDiscretize();
        })();
      }
      // 单次拾取后退出，避免连续误点
      setPickMode(null);
      if (appendMode && featSel >= 0) scheduleAutoDiscretize();
    };
    window.addEventListener("cloudsim-pick-commit", onCommit);
    return () => window.removeEventListener("cloudsim-pick-commit", onCommit);
  }, [
    featureEditActive,
    pickMode,
    appendMode,
    featSel,
    features,
    setFeatures,
    setWorkpieceId,
    setPickMode,
    setStatus,
    strategyId,
    strategies,
  ]);

  /** 离散前确保已绑定；已有则复用 */
  const ensurePlan = async () => {
    if (!activeRootId) {
      setStatus("请先选择机器人", "warn");
      return "";
    }
    if (pathPlanId) return pathPlanId;
    return createNewPathPlan();
  };

  /** 「+」始终新建（对齐旧版 createPathPlanFromUi） */
  const createNewPathPlan = async () => {
    if (!activeRootId) {
      setStatus("创建 PathPlan 需要机器人：请先导入或打开含机器人的工程", "err");
      return "";
    }
    const r = await createPathPlan(activeRootId);
    if (!r.ok || !r.pathPlanId) {
      setStatus(r.error || "创建 PathPlan 失败", "err");
      return "";
    }
    await reloadPrograms();
    await reloadPathPlans(activeRootId);
    await bindPlan(r.pathPlanId, activeRootId);
    setSelectedInstrId(r.pathPlanId);
    setStatus(`已创建 PathPlan ${r.pathPlanId}，请点「开始修改」再拾取/离散`);
    return r.pathPlanId;
  };

  const axisOpts = () => ({ x: axisX, y: axisY, z: axisZ, interval: Math.max(0, axisInterval | 0) });

  const showRawPreview = async (opts: { silent?: boolean } = {}) => {
    const r = (await trajPreviewRaw()) as RawPreviewPayload;
    if (!r.ok) {
      setLastRawPreview(null);
      publishRawPreview(null, axisOpts());
      if (!opts.silent) setStatus(String((r as { error?: string }).error || "Raw 预览失败"), "err");
      return;
    }
    setLastRawPreview(r);
    publishRawPreview(r, axisOpts());
    const n = r.pointCount || (r.pointsMm || []).length;
    const segs = Array.isArray(r.segmentEndExclusive) ? r.segmentEndExclusive.length : 0;
    if (!opts.silent) setStatus(`Raw 预览 ${n} 点${segs ? ` · ${segs} 段` : ""}`);
  };

  useEffect(() => {
    if (lastRawPreview) publishRawPreview(lastRawPreview, axisOpts());
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [axisX, axisY, axisZ, axisInterval]);

  const runDiscretize = async (opts: { silent?: boolean } = {}) => {
    const editOn = editActiveRef.current;
    const wp = workpieceRef.current;
    const feats = featuresRef.current;
    const cats = strategiesRef.current;
    const curSid = strategyRef.current;
    if (!editOn) {
      if (!opts.silent) setStatus("请先「开始修改」再离散", "warn");
      return;
    }
    await ensurePlan();
    if (!wp) {
      if (!opts.silent) setStatus("请选择工件", "warn");
      return;
    }
    if (!feats.length) {
      if (!opts.silent) setStatus("无特征可离散", "warn");
      return;
    }
    const normalized = feats.map((f) => {
      const sid = normalizeFeatureStrategy(f, cats);
      return { ...f, strategyId: sid };
    });
    if (normalized.some((f, i) => f.strategyId !== feats[i].strategyId)) {
      setFeatures(normalized);
    }
    const defaultSid = resolveFeatureStrategy(null, curSid, cats) || cats[0]?.strategyId || "EdgeChain";
    const r = await trajDiscretize({
      schemaVersion: 2,
      workpiece: { backendIdUtf8: wp, stepPathUtf8: "", frameId: "workpiece" },
      defaultStrategyId: defaultSid,
      features: normalized,
    });
    if (r.ok) {
      await syncSession();
      // 对齐桌面：离散后只预览 Raw
      await showRawPreview({ silent: opts.silent });
      if (!opts.silent) setStatus("离散完成");
    } else {
      setLastRawPreview(null);
      publishRawPreview(null, axisOpts());
      if (!opts.silent) setStatus(r.error || "离散失败", "err");
    }
  };

  const scheduleAutoDiscretize = () => {
    if (!editActiveRef.current) return;
    if (autoDiscTimer.current) window.clearTimeout(autoDiscTimer.current);
    autoDiscTimer.current = window.setTimeout(() => {
      autoDiscTimer.current = null;
      if (!editActiveRef.current || !featuresRef.current.length) return;
      void runDiscretize({ silent: true });
    }, 400);
  };

  const selectedParams =
    featSel >= 0 && features[featSel] ? (features[featSel].params as Record<string, unknown> | undefined) : undefined;

  const applyStrategyToSelection = async (sid: string) => {
    setStrategyId(sid);
    if (featSel < 0 || !features[featSel]) return;
    const params = await makeFeatureParams(sid);
    const next = [...features];
    next[featSel] = { ...next[featSel], strategyId: sid, params };
    setFeatures(next);
    scheduleAutoDiscretize();
  };

  const pickStatus =
    !featureEditActive
      ? "未开始修改：特征选取已锁定"
      : pickMode
        ? `拾取中：${pickMode === "edge" ? "线" : "面"}`
        : "3D 拾取未激活";

  return (
    <div className="robot-pane" id="robotTrajGen">
      <div className="toolbar-row">
        <label className="field compact">
          PathPlan
          <select
            value={pathPlanId}
            onChange={(e) => {
              if (e.target.value && activeRootId) void bindPlan(e.target.value, activeRootId);
            }}
          >
            <option value="">（无）</option>
            {pathPlans.map((p) => (
              <option key={p.id} value={p.id}>
                {p.name || p.id}
              </option>
            ))}
          </select>
        </label>
        <button type="button" className="btn-ghost" title="新建路径规划" onClick={() => void createNewPathPlan()}>
          +
        </button>
        <button
          type="button"
          className="btn-run"
          disabled={featureEditActive}
          onClick={async () => {
            const s = await beginEdit();
            if (!s) return;
            if (s.hasRaw) await showRawPreview({ silent: true });
            let n = 0;
            try {
              const raw = s.sourceFeatureJson;
              const doc = typeof raw === "string" ? JSON.parse(raw as string) : raw;
              n = Array.isArray((doc as { features?: unknown[] })?.features)
                ? (doc as { features: unknown[] }).features.length
                : 0;
            } catch {
              n = 0;
            }
            setFeatSel(n ? 0 : -1);
            setAppendMode(false);
          }}
        >
          开始修改
        </button>
        <button
          type="button"
          className="btn-ghost"
          disabled={!featureEditActive}
          onClick={() => void cancelEdit()}
        >
          取消修改
        </button>
      </div>
      <div id="trajEditGate" className="hint">
        {featureEditActive ? "修改中：可拾取特征" : "未开始修改：特征选取已锁定"}
      </div>
      <div className="dock-tabs tertiary">
        <button type="button" className={`tab ${sub === "cad" ? "active" : ""}`} onClick={() => setSub("cad")}>
          CAD/BREP
        </button>
        <button type="button" className={`tab ${sub === "mesh" ? "active" : ""}`} onClick={() => setSub("mesh")}>
          Mesh
        </button>
      </div>

      {sub === "cad" ? (
        <div className="traj-cad-pane">
          <label className="field compact">
            工件
            <select value={workpieceId} onChange={(e) => setWorkpieceId(e.target.value)}>
              <option value="">（选中或列表）</option>
              {objects
                .filter((o) => o.hasGeometry)
                .map((o) => (
                  <option key={o.id} value={o.id}>
                    {o.name || o.id}
                  </option>
                ))}
            </select>
          </label>

          <div className="feat-table-wrap">
            <table className="feat-table">
              <thead>
                <tr>
                  <th>#</th>
                  <th>特征ID</th>
                  <th>离散策略</th>
                  <th>几何摘要</th>
                  <th>状态</th>
                </tr>
              </thead>
              <tbody>
                {!features.length && (
                  <tr className="empty">
                    <td colSpan={5}>暂无特征</td>
                  </tr>
                )}
                {features.map((f, i) => {
                  const faces = f.geometry?.faceIndices || [];
                  const edges = f.geometry?.edgeIndices || [];
                  const summary = `F${faces.length}/E${edges.length}`;
                  const sid = normalizeFeatureStrategy(f, strategies);
                  return (
                    <tr
                      key={f.featureId}
                      className={i === featSel ? "sel" : ""}
                      onClick={() => {
                        setFeatSel(i);
                        setStrategyId(sid);
                      }}
                    >
                      <td>{i + 1}</td>
                      <td title={f.featureId}>{f.featureId}</td>
                      <td title={sid}>{strategyLabel(sid, strategies)}</td>
                      <td>{summary}</td>
                      <td className="st-draft">草稿</td>
                    </tr>
                  );
                })}
              </tbody>
            </table>
          </div>

          <div className="cmd-grid traj-write-row">
            <button
              type="button"
              className={`toggle-btn ${appendMode ? "active" : ""}`}
              disabled={!featureEditActive}
              onClick={() => setAppendMode(true)}
            >
              追加到选中
            </button>
            <button
              type="button"
              className={`toggle-btn ${!appendMode ? "active" : ""}`}
              disabled={!featureEditActive}
              onClick={() => setAppendMode(false)}
            >
              新建特征
            </button>
          </div>
          <div className="cmd-grid traj-pick-row">
            <button
              type="button"
              className={pickMode === "edge" ? "active" : ""}
              disabled={!featureEditActive}
              onClick={() => activatePick("edge")}
            >
              拾取线
            </button>
            <button
              type="button"
              className={pickMode === "face" ? "active" : ""}
              disabled={!featureEditActive}
              onClick={() => activatePick("face")}
            >
              拾取面
            </button>
            <button type="button" className="btn-ghost" disabled={!featureEditActive} onClick={() => setPickMode(null)}>
              取消拾取
            </button>
          </div>
          <div id="trajPickStatus" className="hint">
            {pickStatus}
          </div>

          <div className="toolbar-row strategy-row">
            <span className="lbl">离散策略</span>
            <select
              className="grow"
              value={
                visibleStrategies.some((s) => s.strategyId === strategyId)
                  ? strategyId
                  : visibleStrategies[0]?.strategyId || strategyId
              }
              disabled={!featureEditActive}
              onChange={(e) => void applyStrategyToSelection(e.target.value)}
            >
              {visibleStrategies.map((s) => (
                <option key={s.strategyId} value={s.strategyId}>
                  {s.displayNameZh || s.strategyId}
                </option>
              ))}
              {!visibleStrategies.length && <option value="EdgeChain">EdgeChain</option>}
            </select>
          </div>

          <FeatureParamForm
            strategyId={strategyId}
            params={selectedParams}
            disabled={!featureEditActive}
            editable={featSel >= 0}
            onEnsureParams={(defaults) => {
              if (featSel < 0 || !features[featSel]) return;
              if (features[featSel].params && Object.keys(features[featSel].params!).length) return;
              const next = [...features];
              next[featSel] = { ...next[featSel], params: defaults };
              setFeatures(next);
            }}
            onParamChange={(key, value) => {
              if (featSel < 0 || !features[featSel]) return;
              const next = [...features];
              next[featSel] = {
                ...next[featSel],
                params: { ...(next[featSel].params || {}), [key]: value },
              };
              setFeatures(next);
              scheduleAutoDiscretize();
            }}
          />

          <fieldset className="preview-group">
            <legend>预览</legend>
            <div className="preview-axis-row">
              <label className="inline">
                <input type="checkbox" checked={axisX} onChange={(e) => setAxisX(e.target.checked)} /> X 轴
              </label>
              <label className="inline">
                <input type="checkbox" checked={axisY} onChange={(e) => setAxisY(e.target.checked)} /> Y 轴
              </label>
              <label className="inline">
                <input type="checkbox" checked={axisZ} onChange={(e) => setAxisZ(e.target.checked)} /> Z 轴
              </label>
              <span className="lbl">轴间隔</span>
              <input
                type="number"
                min={0}
                step={1}
                value={axisInterval}
                title="0 = 自动（约 n/20）"
                onChange={(e) => setAxisInterval(Number(e.target.value) || 0)}
              />
            </div>
          </fieldset>
        </div>
      ) : (
        <div id="trajMeshPane">
          <label className="field">
            Mesh 工件
            <select value={workpieceId} onChange={(e) => setWorkpieceId(e.target.value)}>
              {objects
                .filter((o) => o.hasGeometry)
                .map((o) => (
                  <option key={o.id} value={o.id}>
                    {o.name || o.id}
                  </option>
                ))}
            </select>
          </label>
          <label className="field">
            方法
            <select value={meshMethod} onChange={(e) => setMeshMethod(e.target.value)}>
              <option value="CrossSection">截面</option>
              <option value="BsplineRegion">B样条区域</option>
            </select>
          </label>
          <label className="field">
            平面原点 mm (CSV)
            <input value={meshOrigin} onChange={(e) => setMeshOrigin(e.target.value)} />
          </label>
          <label className="field">
            平面法向 (CSV)
            <input value={meshNormal} onChange={(e) => setMeshNormal(e.target.value)} />
          </label>
          <label className="field">
            stepMm
            <input type="number" value={meshStep} step={0.1} onChange={(e) => setMeshStep(Number(e.target.value))} />
          </label>
          {meshMethod === "BsplineRegion" && (
            <label className="field">
              三角索引 CSV
              <input value={meshTri} onChange={(e) => setMeshTri(e.target.value)} placeholder="0,1,2,..." />
            </label>
          )}
          <button
            type="button"
            className="btn-run"
            disabled={!featureEditActive}
            onClick={async () => {
              const triCsv = meshTri
                .split(",")
                .map((x) => x.trim())
                .filter(Boolean)
                .map((x) => Number(x));
              if (meshMethod === "BsplineRegion" && !triCsv.length) {
                setStatus("B样条需要填写三角索引 CSV", "warn");
                return;
              }
              const spec = {
                schemaVersion: 1,
                workpiece: { backendIdUtf8: workpieceId || "", frameId: "workpiece" },
                method: meshMethod,
                crossSection: { planeOriginMm: parseCsv3(meshOrigin), planeNormal: parseCsv3(meshNormal) },
                discretize: { stepMm: meshStep || 2 },
                region: { triangleIndices: meshMethod === "BsplineRegion" ? triCsv : [] },
                bspline: meshMethod === "BsplineRegion" ? { uvCountU: 8, uvCountV: 8 } : {},
              };
              const r = await trajMeshSpec(spec);
              setStatus(r.ok ? "Mesh Raw 已生成" : r.error || "失败", r.ok ? "info" : "err");
              if (r.ok) {
                await syncSession();
                await showRawPreview({ silent: true });
              }
            }}
          >
            生成 Raw
          </button>
          <div className="hint">Mesh Raw 状态见会话</div>
        </div>
      )}
    </div>
  );
}
