import { useEffect, useMemo, useState } from "react";
import { useScene } from "../../state/sceneStore";
import {
  applySchemaValueToOp,
  ensurePointRangeScope,
  isExternalTcpBackendField,
  isOpFieldVisible,
  loadOpSchema,
  OpScopeKind,
  readSchemaValueFromOp,
  scopeKindToken,
  sortOpSchemaFields,
  type OpSchemaField,
  type PipelineOp,
} from "./opSchema";
import { fetchSceneCoordinateFrames, type SceneFrameOpt } from "./sceneFrames";

type GroupOpt = { id: string; name?: string };

type Props = {
  op: PipelineOp | null;
  opIndex: number;
  disabled?: boolean;
  rawPointCount?: number;
  hasRaw?: boolean;
  groups?: GroupOpt[];
  onOpChange: (next: PipelineOp, meta?: { immediate?: boolean }) => void;
};

function isIncompleteNumberDraft(raw: string) {
  return raw === "" || raw === "-" || raw === "." || raw === "-." || /^-?\d+\.$/.test(raw);
}

function scopeHintText(op: PipelineOp, hasRaw: boolean, rawPointCount: number): string {
  const token = scopeKindToken(op.scope?.kind);
  if (hasRaw && token === "Group") {
    return "离散点阶段请用「P 范围」；分组作用于生成程序后的指令组";
  }
  if (token === "PointIndexRange" && rawPointCount > 0) {
    return `P 范围为离散点序号 1…${rawPointCount}，算子仅作用于该区间`;
  }
  return "";
}

export default function OpParamForm({
  op,
  opIndex,
  disabled,
  rawPointCount = 0,
  hasRaw = false,
  groups = [],
  onOpChange,
}: Props) {
  const { objects } = useScene();
  const [fields, setFields] = useState<OpSchemaField[]>([]);
  const [serverValues, setServerValues] = useState<Record<string, unknown>>({});
  const [hint, setHint] = useState("选中算子后显示 schema 表单");
  const [loading, setLoading] = useState(false);
  const [drafts, setDrafts] = useState<Record<string, string>>({});
  const [frames, setFrames] = useState<SceneFrameOpt[]>([]);
  const [title, setTitle] = useState("");
  const needsFrames = fields.some(isExternalTcpBackendField);
  const pointMax = Math.max(1, Math.floor(rawPointCount) || 0);

  useEffect(() => {
    let cancelled = false;
    if (!op) {
      setFields([]);
      setHint("选中算子后显示 schema 表单");
      setTitle("");
      return;
    }
    setLoading(true);
    void (async () => {
      const schema = await loadOpSchema(op.kind || "Translate", opIndex);
      if (cancelled) return;
      setLoading(false);
      if (!schema?.ok) {
        setFields([]);
        setHint(schema?.error || "无 schema");
        return;
      }
      const list = sortOpSchemaFields(schema.fields || []);
      setFields(list);
      // 仅作展示兜底；禁止 hydrate 回写流水线（会覆盖用户刚改的 P 起/止）
      setServerValues(schema.values || {});
      setTitle(schema.displayNameZh || op.kind);
      setDrafts({});
      setHint("");
    })();
    return () => {
      cancelled = true;
    };
    // 仅 kind/index 变化时重拉 schema
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [op?.kind, opIndex]);

  useEffect(() => {
    if (!needsFrames) return;
    let cancelled = false;
    void fetchSceneCoordinateFrames(objects).then((opts) => {
      if (!cancelled) setFrames(opts);
    });
    return () => {
      cancelled = true;
    };
  }, [needsFrames, objects, op?.kind, opIndex]);

  // raw 点数就绪后：若 P 范围仍是默认 1…1，补成 1…N 并写回
  useEffect(() => {
    if (!op || rawPointCount <= 0) return;
    if (scopeKindToken(op.scope?.kind) !== "PointIndexRange") return;
    const next: PipelineOp = {
      ...op,
      scope: { ...(op.scope || {}) },
      params: { ...(op.params || {}) },
    };
    if (ensurePointRangeScope(next, rawPointCount)) onOpChange(next);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rawPointCount, op?.scope?.kind]);

  const valueOf = (key: string) => {
    if (!op) return undefined;
    if (serverValues[key] !== undefined && readSchemaValueFromOp(op, key) === undefined) return serverValues[key];
    const local = readSchemaValueFromOp(op, key);
    return local !== undefined ? local : serverValues[key];
  };

  const commit = (key: string, value: unknown) => {
    if (!op) return;
    const next: PipelineOp = {
      ...op,
      scope: { ...(op.scope || { kind: 0 }) },
      params: { ...(op.params || {}) },
    };
    applySchemaValueToOp(next, key, value);
    if (key === "scope.kind" && Number(value) === OpScopeKind.PointIndexRange) {
      // 切到 P 范围时强制写入 1…N，避免只带 kind、pointTo 仍为默认 1
      next.scope!.pointFrom = 1;
      next.scope!.pointTo = Math.max(1, rawPointCount || 1);
      ensurePointRangeScope(next, rawPointCount);
    }
    if ((key === "scope.pointFrom" || key === "scope.pointTo") && rawPointCount > 0) {
      let from = Number(next.scope!.pointFrom);
      let to = Number(next.scope!.pointTo);
      if (!Number.isFinite(from)) from = 1;
      if (!Number.isFinite(to)) to = rawPointCount;
      from = Math.min(Math.max(1, Math.floor(from)), pointMax);
      to = Math.min(Math.max(1, Math.floor(to)), pointMax);
      if (to < from) to = from;
      next.scope!.pointFrom = from;
      next.scope!.pointTo = to;
    }
    setServerValues((prev) => {
      const copy = { ...prev };
      if (key.startsWith("scope.")) {
        copy[key] = next.scope![key.slice("scope.".length)];
        if (key === "scope.kind") {
          copy["scope.pointFrom"] = next.scope!.pointFrom;
          copy["scope.pointTo"] = next.scope!.pointTo;
        }
      } else {
        copy[key] = value;
      }
      return copy;
    });
    onOpChange(next, { immediate: key.startsWith("scope.") });
  };

  const visibleFields = useMemo(() => {
    if (!op) return [];
    return fields.filter((f) => isOpFieldVisible(f, op));
  }, [fields, op]);

  const scopeHint = op ? scopeHintText(op, hasRaw, rawPointCount) : "";

  if (!op || hint || loading) {
    return (
      <div className={`op-params-form feat-param-form muted${disabled ? " disabled-pane" : ""}`}>
        {loading ? "加载参数…" : hint || "选中算子后显示 schema 表单"}
      </div>
    );
  }

  return (
    <div className={`op-params-form feat-param-form${disabled ? " disabled-pane" : ""}`}>
      <div className="op-field msg">{title || op.kind}</div>
      {scopeHint ? <div className="op-field op-scope-hint">{scopeHint}</div> : null}
      {visibleFields.map((field) => {
        const isTcp = isExternalTcpBackendField(field);
        if (isTcp) {
          let cur = String(valueOf(field.key) ?? "").trim();
          if (cur && !frames.some((f) => f.id === cur)) {
            const byName = frames.find((f) => f.name === cur);
            if (byName) cur = byName.id;
          }
          const radioName = `ext_tcp_${opIndex}_${field.key}`;
          return (
            <div key={field.key} className="op-field op-field-frame" data-field-key={field.key}>
              <label>外部 TCP 坐标系</label>
              <div className="op-frame-picker">
                <label className={`op-frame-opt${!cur ? " sel" : ""}`}>
                  <input
                    type="radio"
                    name={radioName}
                    disabled={disabled}
                    checked={!cur}
                    onChange={() => commit(field.key, "")}
                  />
                  <span>手动（填写下方六自由度）</span>
                </label>
                {frames.map((f) => (
                  <label key={f.id} className={`op-frame-opt${cur === f.id ? " sel" : ""}`}>
                    <input
                      type="radio"
                      name={radioName}
                      disabled={disabled}
                      checked={cur === f.id}
                      onChange={() => commit(field.key, f.id)}
                    />
                    <span title={f.id}>{f.name && f.name !== f.id ? `${f.name}  ·  ${f.id}` : f.id}</span>
                  </label>
                ))}
                {!frames.length && (
                  <div className="op-frame-empty">暂无场景坐标系。请菜单「插入 → 坐标系…」创建后再选。</div>
                )}
              </div>
            </div>
          );
        }
        if (field.type === "Message") {
          return (
            <div key={field.key} className="op-field msg" data-field-key={field.key}>
              {field.messageZh || field.messageEn || field.key}
            </div>
          );
        }
        const label =
          (field.labelZh || field.messageZh || field.labelEn || field.key) +
          (field.unit ? ` (${field.unit})` : "");
        const cur = valueOf(field.key);

        if (field.type === "Bool") {
          return (
            <div key={field.key} className="op-field" data-field-key={field.key}>
              <label>{label}</label>
              <input
                type="checkbox"
                disabled={disabled}
                checked={!!cur}
                onChange={(e) => commit(field.key, e.target.checked)}
              />
            </div>
          );
        }

        if (field.type === "Enum") {
          let opts = field.enumValues || [];
          let labels = field.enumLabelsZh || [];
          if (field.key === "scope.groupId" && groups.length) {
            opts = groups.map((g) => g.id);
            labels = groups.map((g) => g.name || g.id);
          }
          return (
            <div key={field.key} className="op-field" data-field-key={field.key}>
              <label>{label}</label>
              <select
                disabled={disabled}
                value={cur !== undefined && cur !== null ? String(cur) : opts[0] || ""}
                onChange={(e) => {
                  const n = Number(e.target.value);
                  commit(field.key, Number.isNaN(n) ? e.target.value : n);
                }}
              >
                {field.key === "scope.groupId" && !opts.length ? <option value="">（无分组）</option> : null}
                {opts.map((v, i) => (
                  <option key={v} value={v}>
                    {labels[i] || v}
                  </option>
                ))}
              </select>
            </div>
          );
        }

        if (field.type === "Vec3") {
          const sx = field.vec3SuffixX || ".x";
          const sy = field.vec3SuffixY || ".y";
          const sz = field.vec3SuffixZ || ".z";
          return (
            <div key={field.key} className="op-field" data-field-key={field.key}>
              <label>{label}</label>
              <div className="op-vec3">
                {[
                  [sx, "x"],
                  [sy, "y"],
                  [sz, "z"],
                ].map(([suf, axis]) => {
                  const k = field.key + suf;
                  const vv = valueOf(k);
                  return (
                    <input
                      key={k}
                      type="number"
                      disabled={disabled}
                      step={field.step || 0.1}
                      placeholder={axis}
                      value={vv !== undefined && vv !== null ? Number(vv) : 0}
                      onChange={(e) => commit(k, Number(e.target.value))}
                    />
                  );
                })}
              </div>
            </div>
          );
        }

        const isInt = field.type === "Int";
        const isPointRange = field.key === "scope.pointFrom" || field.key === "scope.pointTo";
        const min = isPointRange ? 1 : isInt ? field.minInt : field.min;
        const max = isPointRange && rawPointCount > 0 ? pointMax : isInt ? field.maxInt : field.max;
        const fallback = isInt ? (field.defaultInt ?? 0) : (field.defaultDouble ?? 0);
        const display =
          drafts[field.key] !== undefined
            ? drafts[field.key]
            : String(cur !== undefined && cur !== null ? cur : fallback);
        return (
          <div key={field.key} className="op-field" data-field-key={field.key}>
            <label>{label}</label>
            <input
              type="number"
              disabled={disabled}
              step={isInt ? 1 : field.step || 0.1}
              min={Number.isFinite(min as number) ? min : undefined}
              max={Number.isFinite(max as number) ? max : undefined}
              value={display}
              onChange={(e) => {
                const raw = e.target.value;
                if (isIncompleteNumberDraft(raw)) {
                  setDrafts((d) => ({ ...d, [field.key]: raw }));
                  return;
                }
                const n = isInt ? parseInt(raw, 10) : Number(raw);
                if (!Number.isFinite(n)) return;
                setDrafts((d) => {
                  const next = { ...d };
                  delete next[field.key];
                  return next;
                });
                commit(field.key, n);
              }}
              onBlur={() => {
                if (drafts[field.key] === undefined) return;
                const raw = drafts[field.key];
                const n = isInt ? parseInt(raw, 10) : Number(raw);
                setDrafts((d) => {
                  const next = { ...d };
                  delete next[field.key];
                  return next;
                });
                if (Number.isFinite(n)) commit(field.key, n);
              }}
            />
          </div>
        );
      })}
      <div className="op-field">
        <label>启用</label>
        <input
          type="checkbox"
          disabled={disabled}
          checked={op.enabled !== false}
          onChange={(e) => onOpChange({ ...op, enabled: e.target.checked })}
        />
      </div>
    </div>
  );
}
