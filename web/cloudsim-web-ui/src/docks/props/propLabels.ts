/** 对齐桌面 MainWindow::propertyDisplayLabelForKey 中文标签 */
const LABEL_ZH: Record<string, string> = {
  "core.id": "标识",
  "core.name": "名称",
  "motion.pointIndex": "点位编号",
  "motion.target.frame": "目标坐标系",
  "motion.tool.frameId": "工具坐标系",
  "motion.user.frameId": "用户坐标系",
  "motion.target.pose.x": "目标 X (mm)",
  "motion.target.pose.y": "目标 Y (mm)",
  "motion.target.pose.z": "目标 Z (mm)",
  "motion.target.euler.rx": "欧拉角 RX (deg)",
  "motion.target.euler.ry": "欧拉角 RY (deg)",
  "motion.target.euler.rz": "欧拉角 RZ (deg)",
  "motion.speed": "速度",
  "motion.acc": "加速度",
  "motion.blendRadius": "平滑半径 (mm)",
  "motion.axisConfig": "轴配置预设",
  "motion.axisConfig.preset": "轴配置预设",
  "motion.axisConfig.elbow": "肘部姿态",
  "motion.axisConfig.wrist": "腕部姿态",
  "motion.axisConfig.arm": "臂形前后",
  "motion.axisConfig.turn.j1": "J1 转数",
  "motion.axisConfig.turn.j4": "J4 转数",
  "motion.axisConfig.turn.j6": "J6 转数",
  "pose.x": "位置 X",
  "pose.y": "位置 Y",
  "pose.z": "位置 Z",
  "rotation.x": "旋转 X (°)",
  "rotation.y": "旋转 Y (°)",
  "rotation.z": "旋转 Z (°)",
  "core.class": "类型",
  color: "颜色",
  "logic.io.signalName": "信号名",
  "logic.condition.signalName": "信号名",
  "logic.condition.kind": "等待方式",
  "logic.condition.port": "IO 端口",
  "logic.condition.equals": "目标值 (0/1)",
  "logic.io.port": "IO 端口",
  "logic.io.boolValue": "输出值 (0/1)",
  "logic.io.analogValue": "模拟量",
  "follow.enabled": "跟随启用",
  "follow.targetName": "跟随目标名称",
  "follow.targetId": "跟随目标 ID",
  "follow.localPosition": "局部位置 (mm)",
  "follow.localEulerDeg": "局部欧拉角 (deg)",
  "follow.hierarchyDriven": "层级驱动",
};

export function propertyDisplayLabel(key: string, fallback?: string) {
  return LABEL_ZH[key] || fallback || key;
}

export const AXIS_ENUM_OPTS: Record<string, string[]> = {
  "motion.axisConfig.preset": [
    "AUTO",
    "ELBOW_UP",
    "ELBOW_DOWN",
    "WRIST_FLIP",
    "WRIST_NO_FLIP",
    "ELBOW_UP_WRIST_NO_FLIP",
    "ELBOW_UP_WRIST_FLIP",
    "ELBOW_DOWN_WRIST_NO_FLIP",
    "ELBOW_DOWN_WRIST_FLIP",
    "CUSTOM",
  ],
  "motion.axisConfig.elbow": ["AUTO", "UP", "DOWN"],
  "motion.axisConfig.wrist": ["AUTO", "NO_FLIP", "FLIP"],
  "motion.axisConfig.arm": ["AUTO", "FRONT", "BACK"],
  "motion.axisConfig.turn.j1": ["AUTO", "-2", "-1", "0", "1", "2", "3"],
  "motion.axisConfig.turn.j4": ["AUTO", "-2", "-1", "0", "1", "2", "3"],
  "motion.axisConfig.turn.j6": ["AUTO", "-2", "-1", "0", "1", "2", "3"],
  "motion.target.frame": ["base", "user"],
  "logic.condition.equals": ["0", "1"],
  "logic.condition.kind": ["always", "never", "io", "compare"],
  "logic.io.boolValue": ["0", "1"],
  "follow.enabled": ["false", "true", "0", "1"],
};

export function enumOptionsForKey(key: string): string[] | null {
  return AXIS_ENUM_OPTS[key] || null;
}

/** 对齐桌面 InstructionPropertyPanel：条件用 DI，SET_DO 用 DO，SET_AO 用 AO */
export function ioSignalKindForInstrProp(key: string, instrType?: string): "DI" | "DO" | "AO" | null {
  if (key === "logic.condition.signalName") return "DI";
  if (key === "logic.io.signalName") {
    const t = String(instrType || "").toLowerCase();
    if (t === "set_ao" || t === "setao") return "AO";
    return "DO";
  }
  return null;
}
