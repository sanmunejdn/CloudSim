# FINAL — 自定义设备导出 URDF

## 交付摘要

自定义设备可导出为带 `package.xml` 的 ROS 包；`.urdf` 按米写出（与现有导入对拍）；几何优先保留源 OBJ/STEP。

## 主要代码

- `RobotUrdf`: `CustomDeviceUrdfExporter.*`
- `CloudSimHost`: `exportCustomDeviceUrdfPackage`（`BackendFileImport`）
- `DevicePageWidget` + `MainWindowFileImport`：双入口 UI

## 文档

ALIGNMENT / CONSENSUS / DESIGN / TASK / ACCEPTANCE / TODO（本目录）
