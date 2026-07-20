# CONSENSUS：坐标系后端对象

## 需求

- 菜单「插入 → 坐标系」：对话框定义名称与位姿，生成 `FrameBackendData` 后端
- 「转换工件型」外部 TCP：下拉选坐标系；选中后隐藏手动六参数并用该对象世界位姿；「手动」保留旧六参数

## 技术方案

| 项 | 约定 |
|----|------|
| className | `FrameBackendData` |
| catalogTypeName / sourceType | `CoordinateFrame` |
| 可视化 | `FrameBackendVisual`：外层 MT + RGB 三轴 |
| 注册 | `registerAdoptedFrameAndLoadScene` |
| 参数 | `toWorkpiece.externalTcpBackendId` |
| 运行时 | Engine 每步解析 Frame → `ctx.externalTcpInBase`；Builtins 不链 Data |

## 验收

- 创建后树可见、轴可见、属性可改位姿、工程存盘重开仍在
- 下拉仅列 Frame；选手动显示六数；选 Frame 隐藏六数且执行用世界位姿
- 旧工程无 backendId 行为不变
