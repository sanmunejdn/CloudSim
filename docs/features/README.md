# 已交付功能专题（docs/features）

跨多个工程、与当前主线一致的说明。单工程职责见各 `DEVELOPER_GUIDE.md`。

## 工作区模式

| 专题 | 说明 |
|------|------|
| [主程序](主程序/README.md) | 空 modeId；Widget / Host / RobotWidget / Data |
| [几何建模](几何建模/README.md) | `com.cloudsim.geomodeling`；FEATURES / ARCHITECTURE |
| [工艺流程](工艺流程/README.md) | `com.cloudsim.processflow` |
| [工程图](工程图/README.md) | `com.cloudsim.drawing` |
| [插件](插件/README.md) | 插件类型索引 |

## Host / 后端 / 空间

| 专题 | 说明 |
|------|------|
| [架构](架构/) | 桌面 / 网页架构 HTML |
| [Host优化](Host优化/) | 接口目录、backend 调用清单、Headless 运维 |
| [后端OSG同步](后端OSG同步/README.md) | 后端 ↔ OSG |
| [后端对象与软件模式](后端对象与软件模式/README.md) | 类型三键、侧车、工作区 vs Data |
| [Binding](Binding/README.md) | Data / 指令 / 插件属性 Binding |

## 机器人 / 轨迹

| 专题 | 说明 |
|------|------|
| [机器人路径规划](机器人路径规划/) | PLANNERS 算法原理 |
| [指令IK轨迹](指令IK轨迹/) | IK / 轨迹开发说明 |
| [运动副](运动副/) | 1-DOF FK 与架构图 |
| [仿真外置控制器](仿真外置控制器/README.md) | Webots 风格外置控制器（TCP 19620） |
| [视觉抓取](视觉抓取/README.md) | 工业相机 Tab：手眼 + 碰撞页轨迹规划抓取 |

## 网页端

| 专题 | 说明 |
|------|------|
| [网页端](网页端/README.md) | API 契约、交互、Host↔Headless 同步 |

## 领域资产

| 专题 | 说明 |
|------|------|
| [设计计算](设计计算/) | catalog / 计算表 |
| [标准件](标准件/) | 标准件目录 |
| [图片](图片/) | README 用截图 |

约定与布局见本目录上一级 [`docs/README.md`](../README.md)（`DIRECTORY_LAYOUT` / `SOURCE_CONVENTIONS` / `spatial_contract` / `MODULE_DEVELOPER_GUIDES`）。
