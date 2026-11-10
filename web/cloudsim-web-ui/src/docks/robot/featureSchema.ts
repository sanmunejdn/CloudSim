import { trajFeatureSchema } from "../../api";

export type SchemaField = {
  key: string;
  type?: string;
  labelZh?: string;
  labelEn?: string;
  unit?: string;
  order?: number;
  min?: number;
  max?: number;
  step?: number;
  minInt?: number;
  maxInt?: number;
  defaultDouble?: number;
  defaultInt?: number;
  defaultBool?: boolean;
  enumValues?: string[];
  enumLabelsZh?: string[];
  messageZh?: string;
};

export type FeatureSchema = {
  ok?: boolean;
  strategyId?: string;
  fields?: SchemaField[];
  defaults?: Record<string, unknown> | { params?: Record<string, unknown> };
};

/** Enum 存字符串；兼容误写成下标的旧/后端默认 */
export function resolveEnumParamValue(
  cur: unknown,
  enumValues: string[] | undefined,
  defaultInt?: number,
): string {
  const opts = enumValues || [];
  const fallback = opts[defaultInt ?? 0] ?? opts[0] ?? "";
  if (cur === undefined || cur === null || cur === "") return fallback;
  if (typeof cur === "number" || (typeof cur === "string" && /^\d+$/.test(cur))) {
    const idx = Number(cur);
    if (Number.isInteger(idx) && idx >= 0 && idx < opts.length && !opts.includes(String(cur))) {
      return opts[idx] ?? fallback;
    }
  }
  const s = String(cur);
  return opts.includes(s) ? s : fallback;
}

export function defaultsFromSchema(schema: FeatureSchema | null | undefined): Record<string, unknown> {
  if (!schema) return {};
  const out: Record<string, unknown> = {};
  for (const f of schema.fields || []) {
    if (f.type === "Bool") out[f.key] = !!f.defaultBool;
    else if (f.type === "Int") out[f.key] = f.defaultInt ?? 0;
    else if (f.type === "Enum") out[f.key] = resolveEnumParamValue(undefined, f.enumValues, f.defaultInt);
    else if (f.type === "Message" || f.type === "Vec3") continue;
    else out[f.key] = f.defaultDouble ?? 0;
  }
  const d = schema.defaults;
  if (d && typeof d === "object") {
    const flat =
      "params" in d && d.params && typeof d.params === "object"
        ? (d.params as Record<string, unknown>)
        : (d as Record<string, unknown>);
    for (const [k, v] of Object.entries(flat)) {
      const field = (schema.fields || []).find((f) => f.key === k);
      out[k] = field?.type === "Enum" ? resolveEnumParamValue(v, field.enumValues, field.defaultInt) : v;
    }
  }
  return out;
}

export async function loadFeatureSchema(strategyId: string): Promise<FeatureSchema | null> {
  if (!strategyId) return null;
  const r = (await trajFeatureSchema(strategyId)) as FeatureSchema;
  if (!r?.ok) return null;
  return r;
}

export async function makeFeatureParams(strategyId: string): Promise<Record<string, unknown>> {
  return defaultsFromSchema(await loadFeatureSchema(strategyId));
}
