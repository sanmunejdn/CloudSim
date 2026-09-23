/// @file geomodelRibbon.ts
/// @brief 几何建模 Ribbon 工具表（与 Host geomodel op 对齐）

export type ToolId =
  | "box"
  | "cylinder"
  | "polygon"
  | "slot"
  | "ellipse"
  | "sketch"
  | "pad"
  | "pocket"
  | "sweep"
  | "sweepCut"
  | "fillet"
  | "chamfer"
  | "revolve"
  | "revolveCut"
  | "linearPattern"
  | "circularPattern"
  | "mirror3d"
  | "loft"
  | "loftCut"
  | "shell"
  | "draft";

export const END_OPTIONS = ["Blind", "MidPlane", "TwoDirections", "ThroughAll"] as const;
export const PLANES = ["XY", "XZ", "YZ"] as const;

export const RIBBON: { group: string; tools: { id: ToolId; label: string }[] }[] = [
  {
    group: "草图",
    tools: [
      { id: "sketch", label: "轮廓" },
      { id: "pad", label: "拉伸" },
    ],
  },
  {
    group: "实体",
    tools: [
      { id: "box", label: "长方体" },
      { id: "cylinder", label: "圆柱" },
      { id: "polygon", label: "多边形" },
      { id: "slot", label: "槽口" },
      { id: "ellipse", label: "椭圆" },
    ],
  },
  {
    group: "特征",
    tools: [
      { id: "pocket", label: "切除" },
      { id: "sweep", label: "扫描" },
      { id: "sweepCut", label: "扫描切除" },
      { id: "fillet", label: "圆角" },
      { id: "chamfer", label: "倒角" },
      { id: "revolve", label: "旋转" },
      { id: "revolveCut", label: "旋转切除" },
    ],
  },
  {
    group: "阵列",
    tools: [
      { id: "linearPattern", label: "线性" },
      { id: "circularPattern", label: "圆周" },
      { id: "mirror3d", label: "镜像" },
      { id: "loft", label: "放样" },
      { id: "loftCut", label: "放样切除" },
      { id: "shell", label: "抽壳" },
      { id: "draft", label: "拔模" },
    ],
  },
];

export const TOOL_TITLE: Record<ToolId, string> = Object.fromEntries(
  RIBBON.flatMap((g) => g.tools.map((t) => [t.id, t.label])),
) as Record<ToolId, string>;

export const NEEDS_BODY = new Set<ToolId>([
  "pocket",
  "sweepCut",
  "fillet",
  "chamfer",
  "revolveCut",
  "linearPattern",
  "circularPattern",
  "mirror3d",
  "loftCut",
  "shell",
  "draft",
]);
