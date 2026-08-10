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
};

export function enumOptionsForKey(key: string): string[] | null {
  return AXIS_ENUM_OPTS[key] || null;
}
