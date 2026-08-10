import { useEffect, useState } from "react";
import { defaultsFromSchema, loadFeatureSchema, type SchemaField } from "./featureSchema";

type Props = {
  strategyId: string;
  params: Record<string, unknown> | undefined;
  disabled?: boolean;
  /** 无选中特征时仍可预览 schema，但不能改 */
  editable: boolean;
  onEnsureParams: (defaults: Record<string, unknown>) => void;
  onParamChange: (key: string, value: unknown) => void;
};

function isIncompleteNumberDraft(raw: string) {
  return raw === "" || raw === "-" || raw === "." || raw === "-." || /^-?\d+\.$/.test(raw);
}

export default function FeatureParamForm({
  strategyId,
  params,
  disabled,
  editable,
  onEnsureParams,
  onParamChange,
}: Props) {
  const [fields, setFields] = useState<SchemaField[]>([]);
  const [hint, setHint] = useState("选择策略后显示参数");
  const [loading, setLoading] = useState(false);
  /** 输入「-」等中间态，避免 Number('-')→NaN 被写成 0 */
  const [drafts, setDrafts] = useState<Record<string, string>>({});

  useEffect(() => {
    let cancelled = false;
    if (!strategyId) {
      setFields([]);
      setHint("选择策略后显示参数");
      return;
    }
    setLoading(true);
    void loadFeatureSchema(strategyId).then((schema) => {
      if (cancelled) return;
      setLoading(false);
      if (!schema) {
        setFields([]);
        setHint("无法加载策略参数");
        return;
      }
      const sorted = [...(schema.fields || [])].sort((a, b) => (a.order || 0) - (b.order || 0));
      setFields(sorted);
      setDrafts({});
      setHint("");
      const defs = defaultsFromSchema(schema);
      if (!params || !Object.keys(params).length) onEnsureParams(defs);
    });
    return () => {
      cancelled = true;
    };
    // params 刻意不进依赖：仅策略切换时拉 schema / 补默认
  }, [strategyId]);

  if (!strategyId || hint || loading) {
    return (
      <div className={`op-params-form feat-param-form muted${disabled ? " disabled-pane" : ""}`}>
        {loading ? "加载参数…" : hint || "选择策略后显示参数"}
      </div>
    );
  }

  if (!fields.length) {
    return (
      <div className={`op-params-form feat-param-form muted${disabled ? " disabled-pane" : ""}`}>
        该策略无参数
      </div>
    );
  }

  const values = params && Object.keys(params).length ? params : {};

  return (
    <div className={`op-params-form feat-param-form${disabled ? " disabled-pane" : ""}`}>
      {fields.map((field) => {
        if (field.type === "Message") {
          return (
            <div key={field.key || field.messageZh} className="op-field msg">
              {field.messageZh || field.key}
            </div>
          );
        }
        const label =
          (field.labelZh || field.labelEn || field.key) + (field.unit ? ` (${field.unit})` : "");
        const cur = values[field.key];
        const canEdit = editable && !disabled;

        if (field.type === "Bool") {
          return (
            <div key={field.key} className="op-field">
              <label>{label}</label>
              <input
                type="checkbox"
                disabled={!canEdit}
                checked={cur !== undefined ? !!cur : !!field.defaultBool}
                onChange={(e) => onParamChange(field.key, e.target.checked)}
              />
            </div>
          );
        }

        if (field.type === "Enum") {
          const opts = field.enumValues || [];
          return (
            <div key={field.key} className="op-field">
              <label>{label}</label>
              <select
                disabled={!canEdit}
                value={cur !== undefined && cur !== null ? String(cur) : opts[0] || ""}
                onChange={(e) => onParamChange(field.key, e.target.value)}
              >
                {opts.map((v, i) => (
                  <option key={v} value={v}>
                    {(field.enumLabelsZh || [])[i] || v}
                  </option>
                ))}
              </select>
            </div>
          );
        }

        const isInt = field.type === "Int";
        const min = isInt ? field.minInt : field.min;
        const max = isInt ? field.maxInt : field.max;
        const fallback = isInt ? (field.defaultInt ?? 0) : (field.defaultDouble ?? 0);
        const display =
          drafts[field.key] !== undefined
            ? drafts[field.key]
            : String(cur !== undefined && cur !== null ? cur : fallback);
        return (
          <div key={field.key} className="op-field">
            <label>{label}</label>
            <input
              type="number"
              disabled={!canEdit}
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
                onParamChange(field.key, n);
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
                if (Number.isFinite(n)) onParamChange(field.key, n);
              }}
            />
          </div>
        );
      })}
    </div>
  );
}
