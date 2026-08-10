import { useCallback, useEffect, useRef, useState } from "react";
import {
  trajGetPipeline,
  trajPutPipeline,
  trajOpPalette,
  trajUndo,
  trajRedo,
  trajReset,
  trajPreview,
  trajEmit,
  trajApply,
  trajSession,
} from "../../api";
import { useStatus } from "../../state/statusStore";
import { useTrajectory } from "../../state/trajectoryStore";
import { useRobotProgram } from "../../state/robotProgramStore";
import { useDockNav } from "../../state/dockNavStore";
import { normalizeOpPalette, OP_PALETTE_FALLBACK } from "./opPalette";
import OpParamForm from "./OpParamForm";
import { defaultScopeForNewOp, type PipelineOp } from "./opSchema";
import { publishRawPreview } from "../../scene/rawPreview";

export default function TrajectoryEditPanel() {
  const { featureEditActive, syncSession, exitEditAfterCommit, editUiEpoch } = useTrajectory();
  const { catalogs, activeRootId, reloadPrograms } = useRobotProgram();
  const { goCmd } = useDockNav();
  const { setStatus } = useStatus();
  const [ops, setOps] = useState<PipelineOp[]>([]);
  const [palette, setPalette] = useState(OP_PALETTE_FALLBACK);
  const [sel, setSel] = useState(0);
  const [recipe, setRecipe] = useState("weld");
  const [rawHint, setRawHint] = useState("请先在轨迹生成页离散");
  const [hasRaw, setHasRaw] = useState(false);
  const [rawPointCount, setRawPointCount] = useState(0);
  const [previewOn, setPreviewOn] = useState(true);
  const [progId, setProgId] = useState("");
  const [groupId, setGroupId] = useState("");
  const [paramsJson, setParamsJson] = useState("{}");
  const previewOnRef = useRef(previewOn);
  previewOnRef.current = previewOn;
  const saveTimer = useRef<number | null>(null);
  const opsRef = useRef<PipelineOp[]>([]);
  opsRef.current = ops;
  const persistGen = useRef(0);
  const selRef = useRef(sel);
  selRef.current = sel;

  const entry = catalogs.find((c) => c.sceneBackendId === activeRootId);
  const programs = entry?.programs || [];
  const prog = programs.find((p) => p.id === progId) || programs.find((p) => p.isMain) || programs[0];
  const groups = prog?.groups || [];

  const reload = useCallback(async () => {
    try {
      const pal = await trajOpPalette();
      setPalette(normalizeOpPalette(pal));
    } catch {
      setPalette(OP_PALETTE_FALLBACK);
    }
    try {
      const raw = await trajGetPipeline();
      const list = Array.isArray(raw)
        ? (raw as PipelineOp[])
        : ((raw as { pipeline?: PipelineOp[]; ops?: PipelineOp[] }).pipeline ||
            (raw as { ops?: PipelineOp[] }).ops ||
            []) as PipelineOp[];
      setOps(list);
      setSel((i) => (list.length ? Math.min(i, list.length - 1) : 0));
    } catch {
      /* 保留现有流水线 */
    }
    try {
      const s = await trajSession();
      const n = Math.floor(Number(s?.rawPointCount) || 0);
      const raw = !!s?.hasRaw && n > 0;
      setHasRaw(raw);
      setRawPointCount(raw ? n : 0);
      if (raw) setRawHint(`Raw ${n} 点 · 可编辑（新建算子默认 P 范围 1…${n}）`);
      else setRawHint("请先在轨迹生成页离散");
      await syncSession();
    } catch {
      /* ignore */
    }
  }, [syncSession]);

  useEffect(() => {
    void reload();
  }, [reload, featureEditActive]);

  // 应用/生成后：清空本地流水线选择与 Raw 提示（服务端会话已退出编辑）
  useEffect(() => {
    if (editUiEpoch === 0) return;
    if (featureEditActive) return;
    setOps([]);
    setSel(0);
    setHasRaw(false);
    setRawPointCount(0);
    setRawHint("请先在轨迹生成页离散");
    setParamsJson("{}");
  }, [editUiEpoch, featureEditActive]);

  useEffect(() => {
    if (prog?.id) setProgId(prog.id);
  }, [prog?.id]);

  useEffect(() => {
    const op = ops[sel];
    if (!op) {
      setParamsJson("{}");
      return;
    }
    setParamsJson(JSON.stringify({ scope: op.scope || {}, params: op.params || {} }, null, 2));
  }, [sel, ops]);

  const persist = useCallback(
    async (next: PipelineOp[], opts: { preview?: boolean; skipSetOps?: boolean } = {}) => {
      const gen = ++persistGen.current;
      if (!opts.skipSetOps) {
        opsRef.current = next;
        setOps(next);
      }
      const r = await trajPutPipeline(next);
      if (gen !== persistGen.current) return;
      if (!r.ok) {
        setStatus(r.error || "保存管线失败", "err");
        return;
      }
      if (opts.preview !== false && previewOnRef.current) {
        try {
          const prev = await trajPreview();
          if (gen !== persistGen.current) return;
          if (prev && (prev as { ok?: boolean }).ok !== false) {
            publishRawPreview(prev as never, { x: true, y: true, z: true, interval: 0 });
          }
        } catch {
          /* ignore */
        }
      }
    },
    [setStatus],
  );

  const saveOpAtSel = useCallback(
    (nextOp: PipelineOp, meta?: { immediate?: boolean }) => {
      const idx = selRef.current;
      setOps((prev) => {
        const next = prev.map((o, i) => (i === idx ? nextOp : o));
        opsRef.current = next;
        if (saveTimer.current) window.clearTimeout(saveTimer.current);
        const delay = meta?.immediate ? 0 : 200;
        saveTimer.current = window.setTimeout(() => {
          void persist(opsRef.current, { preview: true, skipSetOps: true });
        }, delay);
        return next;
      });
    },
    [persist],
  );

  const locked = !featureEditActive;
  const selected = ops[sel] || null;

  return (
    <div className="robot-pane traj-edit-pane" id="robotTrajEdit">
      <fieldset className="traj-edit-recipe">
        <legend>工艺模板</legend>
        <div className="toolbar-row traj-edit-recipe-row">
          <span className="traj-edit-raw-status grow">{rawHint}</span>
          <select value={recipe} onChange={(e) => setRecipe(e.target.value)} title="工艺">
            <option value="weld">焊缝</option>
            <option value="glue">涂胶</option>
            <option value="grind">打磨</option>
          </select>
          <button type="button" disabled={locked} onClick={() => setStatus(`已选择工艺：${recipe}`)}>
            填充
          </button>
          <button
            type="button"
            className="btn-run"
            disabled={locked}
            onClick={async () => {
              const r = await trajEmit();
              setStatus(r.ok ? "已生成 → LINE" : r.error || "生成失败", r.ok ? "info" : "err");
              if (!r.ok) return;
              await exitEditAfterCommit();
              await reloadPrograms();
              await reload();
              goCmd();
            }}
          >
            生成
          </button>
        </div>
      </fieldset>

      <div className="toolbar-row traj-edit-scope-row">
        <span className="lbl">程序</span>
        <select className="grow" value={prog?.id || ""} onChange={(e) => setProgId(e.target.value)}>
          {programs.map((p) => (
            <option key={p.id} value={p.id}>
              {p.name || p.id}
            </option>
          ))}
        </select>
        <span className="lbl">组</span>
        <select className="grow" value={groupId} onChange={(e) => setGroupId(e.target.value)}>
          <option value="">（无）</option>
          {groups.map((g) => (
            <option key={g.id} value={g.id}>
              {g.name || g.id}
            </option>
          ))}
        </select>
      </div>

      <div className="traj-edit-body">
        <div className="traj-edit-palette-col">
          <div className="section-title">算子</div>
          <div className={`traj-op-palette ${locked ? "disabled-pane" : ""}`}>
            {palette.map((p) => (
              <button
                key={p.kind}
                type="button"
                disabled={locked}
                title={p.kind}
                onClick={() => {
                  const scope = defaultScopeForNewOp(rawPointCount, groupId);
                  const next = [...ops, { kind: p.kind, enabled: true, scope, params: {} }];
                  setSel(next.length - 1);
                  void persist(next);
                }}
              >
                {p.displayNameZh || p.kind}
              </button>
            ))}
          </div>
        </div>
        <div className="traj-edit-pipeline-col">
          <div className="section-title">流水线</div>
          <div className={`traj-op-pipeline ${locked ? "disabled-pane" : ""} ${ops.length ? "" : "muted"}`}>
            {!ops.length && "空"}
            {ops.map((op, i) => (
              <div
                key={`${op.kind}-${i}`}
                className={`step ${i === sel ? "sel" : ""} ${op.enabled === false ? "disabled-op" : ""}`}
                onClick={() => setSel(i)}
              >
                <span className="t">{op.kind}</span>
                <label className="en">
                  <input
                    type="checkbox"
                    checked={op.enabled !== false}
                    disabled={locked}
                    onChange={(e) => {
                      const next = ops.map((o, idx) => (idx === i ? { ...o, enabled: e.target.checked } : o));
                      void persist(next);
                    }}
                  />
                </label>
                <button
                  type="button"
                  className="rm"
                  disabled={locked}
                  onClick={(e) => {
                    e.stopPropagation();
                    const next = ops.filter((_, idx) => idx !== i);
                    setSel((s) => Math.min(s, Math.max(0, next.length - 1)));
                    void persist(next);
                  }}
                >
                  ×
                </button>
              </div>
            ))}
          </div>
        </div>
      </div>

      {ops.length > 0 && (
        <>
          <details className="traj-edit-params" open>
            <summary>参数</summary>
            <OpParamForm
              op={selected}
              opIndex={sel}
              disabled={locked}
              hasRaw={hasRaw}
              rawPointCount={rawPointCount}
              groups={groups.map((g) => ({ id: g.id, name: g.name }))}
              onOpChange={(nextOp) => saveOpAtSel(nextOp)}
            />
            <details className="adv">
              <summary>高级：原始 JSON</summary>
              <textarea rows={3} value={paramsJson} onChange={(e) => setParamsJson(e.target.value)} placeholder="{}" />
              <button
                type="button"
                className="btn-ghost"
                disabled={locked || !selected}
                onClick={() => {
                  try {
                    const parsed = JSON.parse(paramsJson || "{}") as {
                      scope?: Record<string, unknown>;
                      params?: Record<string, unknown>;
                    };
                    const nextOp: PipelineOp = {
                      ...selected!,
                      scope: parsed.scope || selected!.scope || { kind: 0 },
                      params: parsed.params || {},
                    };
                    void persist(ops.map((o, i) => (i === sel ? nextOp : o)));
                    setStatus("参数已应用");
                  } catch {
                    setStatus("JSON 无效", "err");
                  }
                }}
              >
                应用 JSON
              </button>
            </details>
          </details>

          <div className="toolbar-row traj-edit-actions">
            <label className="inline">
              <input type="checkbox" checked={previewOn} onChange={(e) => setPreviewOn(e.target.checked)} /> 预览
            </label>
            <button
              type="button"
              className="btn-run"
              disabled={locked}
              onClick={async () => {
                if (previewOn) {
                  const prev = await trajPreview();
                  if (prev && (prev as { ok?: boolean }).ok !== false) {
                    publishRawPreview(prev as never, { x: true, y: true, z: true, interval: 0 });
                  }
                }
                const r = await trajApply();
                setStatus(r.ok ? "已应用 → LINE" : r.error || "应用失败", r.ok ? "info" : "err");
                if (!r.ok) return;
                await exitEditAfterCommit();
                setOps([]);
                setSel(0);
                await reloadPrograms();
                await reload();
                goCmd();
              }}
            >
              应用
            </button>
            <button type="button" className="btn-ghost" disabled={locked} onClick={() => void trajReset().then(reload)}>
              重置
            </button>
            <button type="button" className="btn-ghost" disabled={locked} onClick={() => void trajUndo().then(reload)}>
              撤销
            </button>
            <button type="button" className="btn-ghost" disabled={locked} onClick={() => void trajRedo().then(reload)}>
              重做
            </button>
          </div>
        </>
      )}
    </div>
  );
}
