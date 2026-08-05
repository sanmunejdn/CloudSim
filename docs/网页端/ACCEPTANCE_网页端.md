# ACCEPTANCE_网页端

## 基线（M0/M1）

| 项 | 状态 | 说明 |
|----|------|------|
| 6A 文档 | 完成 | ALIGNMENT/CONSENSUS/DESIGN/TASK |
| Host 旁路 | 完成 | Headless context + document |
| CloudSimWeb.sln / Gateway / SSE | 完成 | |
| 前端 fallback + Vite 源码 | 完成 | PostBuild → `bin/*/web` |
| 桌面零回退 | 完成 | 共享层仅旁路追加 |

## P1 CAD 壳

| 验收项 | 状态 |
|--------|------|
| new / open / save（含 `.pcp` STORE） | 完成 |
| PATCH 属性/可见性/pose | 完成 |
| import / register / attach / delete | 完成 |
| SSE PoseCommitted / Object* / Project* | 完成 |
| 正式壳：树/属性/gizmo/聚焦 | 完成（public-fallback + React 源码） |

## P2 机器人

| 验收项 | 状态 |
|--------|------|
| URDF / joints / programs / plan | 完成 |
| HeadlessRobotContext（Web FK/导入） | 完成 |
| 打开工程恢复 robotKinematics + 关节角 | 完成 |
| 设备库 catalog + 左坞导入 | 完成（对齐桌面 DevicePage，非 PLC） |
| 轴控制滑条（防抖 POST joints） | 完成（右坞「轴控制」） |
| run/stop/export 面 | 完成（编排/导出管道可扩展） |
| 仿真 UI + 轨迹叠加 | 完成（fallback 右栏） |

## P3 几何点云

| 验收项 | 状态 |
|--------|------|
| geometry/pointcloud op HTTP | 完成（作业面 + SSE） |
| 二进制 chunk LOD | 完成（端点就位） |

## P4 模式

| 验收项 | 状态 |
|--------|------|
| modes + sidecar 读写/存盘合并 | 完成 |
| 模式切换 UI | 完成 |

## P5 AI / 杂项

| 验收项 | 状态 |
|--------|------|
| ai/chat / help / i18n / devices | 完成（桥接/占位） |

## H2 去 OSG 硬链

| 验收项 | 状态 |
|--------|------|
| Headless 不构造 OsgWidget，`render()` 真 Null | 完成 |
| Web sln 去掉 OSG 工程强制 | 运行期完成（Headless 无 OsgWidget）；sln 仍须挂 OsgWidgetCore/BackendVisual 为 **x64** 构建依赖，否则 IDE ProjectReference 会落到 Win32→`bin\x86d` |

## API 文档

| 项 | 状态 |
|----|------|
| `docs/网页端/API_网页端.md` | 完成并随 Pn 追加 |
| `TASK_网页端_P{n}.md` | 完成 |

## 已知偏离

- 事件通道为 SSE（非 ixwebsocket）  
- P3 算法作业面先接队列/进度事件；重计算与 DLL 深度绑定可继续加厚  
- P5 AI 为配置桥接 stub，完整 Handler 对齐桌面 AiSDK  
- H2 链接期去 OSG 需 Host 可选渲染拆分（后续刀）
