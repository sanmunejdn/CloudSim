import type { OpPaletteEntry } from "../../api";

/** 与旧网页 OP_PALETTE_FALLBACK 一致：接口失败/空列表时仍可点选 */
export const OP_PALETTE_FALLBACK: OpPaletteEntry[] = [
  { kind: "Translate", displayNameZh: "平移" },
  { kind: "Rotate", displayNameZh: "旋转" },
  { kind: "Delete", displayNameZh: "删除" },
  { kind: "Duplicate", displayNameZh: "复制" },
  { kind: "Mirror", displayNameZh: "轴反向" },
  { kind: "Reorder", displayNameZh: "固定姿态" },
  { kind: "Resample", displayNameZh: "重采样" },
  { kind: "OffsetAlongNormal", displayNameZh: "法向偏移" },
  { kind: "OffsetLateral", displayNameZh: "横向偏移" },
  { kind: "SmoothPose", displayNameZh: "姿态平滑" },
  { kind: "AssignBlend", displayNameZh: "过渡半径" },
  { kind: "AssignSpeedZone", displayNameZh: "速度区" },
  { kind: "Weave", displayNameZh: "摆动" },
  { kind: "ReachabilityFilter", displayNameZh: "可达性过滤" },
  { kind: "ExternalAxisSearch", displayNameZh: "外部轴搜索" },
  { kind: "Approach", displayNameZh: "进刀" },
  { kind: "Retract", displayNameZh: "退刀" },
  { kind: "ProjectToGeometry", displayNameZh: "轨迹投影" },
  { kind: "NonRigidRegistration", displayNameZh: "非刚性配准纠正" },
  { kind: "ToWorkpieceInHand", displayNameZh: "转换工件型" },
];

export function normalizeOpPalette(r: {
  ok?: boolean;
  ops?: OpPaletteEntry[];
  entries?: OpPaletteEntry[];
} | null | undefined): OpPaletteEntry[] {
  const ops = r?.ops || r?.entries || [];
  if (r?.ok !== false && ops.length) {
    return ops.map((o) => ({
      kind: o.kind,
      displayNameZh: o.displayNameZh || o.kind,
    }));
  }
  return OP_PALETTE_FALLBACK.map((o) => ({ ...o }));
}
