# ACCEPTANCE — 自定义设备

## 一期

| 项 | 状态 |
|----|------|
| CustomDevice 聚合根 / Visual / FK / 轴控切换 | 完成 |
| Debug\|x64 + Release\|x64 | 通过 |

## 二期（多轴 UI + 3D 拾取）

| 项 | 状态 |
|----|------|
| CustomDeviceAxisEditorWidget 多轴 CRUD | 完成 |
| 向导嵌入编辑器 + 提前落场景 | 完成 |
| 面拾取旋转中心（W0 局部）+ 可选法向作轴 | 完成 |
| 轴控 enabled q 打包对齐 | 完成 |
| Debug\|x64 + Release\|x64 | 通过（RobotScene / RobotWidget / Widget） |

## 手工验收

1. 添加 ≥2 轴（平移+旋转），确认后轴控出现多滑条
2. 「应用模型到场景」后，旋转轴「拾取中心」点面，origin/轴方向更新
3. 关闭向导后拾取模式不残留；存盘重开轴配置保留
