# CONSENSUS — 自定义设备导出 URDF

## 需求描述

将已组装的自定义设备（Link/Joint 有向树）导出为 **可安装的 ROS 风格包**，并可用 **现有「导入 URDF」** 回灌验证。

## 已确认决策

1. **包形态**：含 `package.xml`；网格/CAD 用 `package://<pkg>/…` 引用
2. **几何策略**：优先拷贝子件 **源 OBJ / 源 STEP**（`sourcePath`）；无源或不可用时再导出网格（建议 STL/OBJ）
3. **验收底线**：导出包 → 现有 `importUrdfRobot` / 设备页导入 **回灌成功**（层级、轴控可动）
4. **UI**：设备页「导出 URDF…」+ 组装对话框「导出 URDF…」
5. **单位**：**方案 B** — `.urdf` 内长度按 **米** 写出（内部 mm÷1000），与现有 `UrdfRobotLoader` 一致；网格顶点保持工程 **mm**

## 验收标准

1. 对含 ≥2 Link、≥1 Revolute/Prismatic 的自定义设备，导出目录含：
   - `package.xml`
   - `urdf/<name>.urdf`
   - `meshes/` 或 `cad/` 下被引用的 OBJ/STEP/网格文件
2. 用现有导入入口打开该 `.urdf`，场景出现机器人层级，关节可在轴控拖动
3. 固定根、父子关系、运动类型、轴方向、限位与导出前设备一致（允许命名清洗与数值浮点误差）
4. 无 Link 图 / 无网格且无法落盘时，导出失败并中文提示，不写半包
5. Debug|x64 与 Release|x64：相关工程（至少 `RobotUrdf`/`CloudSimHost`/`RobotWidget`/`Widget`）编译通过

## 技术方案（摘要）

- 新增导出模块（建议 `RobotUrdf`：`CustomDeviceUrdfExporter` 或等价），只写文件，不改运行时设备身份
- 输入：`CustomDeviceBackendData` + `BackendDataManager`（解析 `geometryBackendId`）
- 映射：Link→`<link>`；Joint→`revolute`/`prismatic`/`fixed`；`parentToChildRest`→`<origin xyz rpy>`（米）；轴/限位写入 `<axis>`/`<limit>`
- 包名：由设备名清洗为合法 ROS package name
- UI：选目录或保存路径 → 调 Host/导出 API → 成功提示路径

## 不做

- 闭环、xacro、连续副、完整浮动基语义
- 网页导出
- 导出后把自定义设备运行时替换成 URDF 机器人（导出与内部模型并存）
- 联立 IK

## 开放点

- [x] 单位：**B（文件米）** — 2026-08-15 确认
