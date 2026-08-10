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

export function defaultsFromSchema(schema: FeatureSchema | null | undefined): Record<string, unknown> {
  if (!schema) return {};
  const out: Record<string, unknown> = {};
  for (const f of schema.fields || []) {
    if (f.type === "Bool") out[f.key] = !!f.defaultBool;
    else if (f.type === "Int") out[f.key] = f.defaultInt ?? 0;
    else if (f.type === "Enum") out[f.key] = (f.enumValues || [])[f.defaultInt || 0] ?? "";
    else if (f.type === "Message" || f.type === "Vec3") continue;
    else out[f.key] = f.defaultDouble ?? 0;
  }
  const d = schema.defaults;
  if (d && typeof d === "object") {
    const flat =
      "params" in d && d.params && typeof d.params === "object"
        ? (d.params as Record<string, unknown>)
        : (d as Record<string, unknown>);
    Object.assign(out, flat);
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
