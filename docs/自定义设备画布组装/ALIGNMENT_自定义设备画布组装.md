# ALIGNMENT — 自定义设备画布组装

## 原始需求

选定/导入 STEP 后在画布形成 Link 块，通过连线定义旋转副/移动副；基座固定、子件相对运动；轴控可预览。

## 项目现状

- 已有 `CustomDeviceBackendData` + 扁平 `deviceAxes`，FK 为根上 `W0*ΠT(q)`，子件 Follow 整机
- 创建入口：设备页对话框向导（无图）
- TODO 已列「多 link」未做

## 边界

**做：** Link/Joint 图模型、树状 FK、桌面组装画布、旧 JSON 兼容、轴控打通  
**不做（MVP）：** 闭环机构、CAD Mate 求解、URDF 导出、联立 IK、网页画布

## 歧义澄清

- 「step」= STEP 几何件 → Link 块（非 6A 任务步）
- 平台：桌面优先；网页 React Flow 二期
