# ALIGNMENT — 脚本建模（JSON / Python）

## 项目上下文

几何建模已有两条非 UI 通路：`parametricHistory` JSON（与存盘同构）与 `feature.compose` ActionPlan v2。缺磁盘导入/导出、公开示例与几何 Python API。

## 原始需求

通过 JSON 文件或 Python 完成建模，脱离纯 UI 点选。

## 边界

| 做 | 不做 |
|----|------|
| 导出/导入 history；运行 compose 文件 | 第三套特征 DSL |
| 双格式自动判别 | Headless / 无窗口批处理 |
| 进程内 `cloudsim_geom` 四 API + 控制台 | FreeCAD 式 Part 对象模型 / pip RPC |
| 一期无 ABI bump | 用 compose 覆盖全部 history 能力 |

## 需求理解

- History：`set`/`queryParametricBodyHistoryJson`
- Compose：`IAiAssistantHost::executeActionPlan`
- Python：pybind 转发同一 Host API

## 疑问澄清（已锁定）

- 一期不 bump Host ABI
- Python 为二期薄封装，嵌入进程
