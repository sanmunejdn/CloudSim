# 资源生命周期契约（P1-8）

| 区域 | 创建者 | 释放者 | 异常路径 | 本轮修复 |
|------|--------|--------|----------|----------|
| SSE `eventQueue` | `WebGateway::pushEvent` | `/api/events` 消费；**`stop()` 清空** | 队列满丢弃最旧 + `EventsDropped` | `stop()` 清队列 |
| `m_faceHighlightSoup` | Headless 轨迹面高亮缓存 | **绑定/新建 PathPlan 时 `clear`** | 文档关闭随 Session 析构 | `createPathPlan` 清缓存 |
| `JobSystem` 超时弃池 | `MainWindow` 持有 | `shutdown` 等待 `kShutdownWaitMs` 后弃池 | 作业回调必须 `QPointer` | 注释固化契约 |
| Three.js unmount | `useViewportCore` | effect cleanup：`renderer.dispose()` + drain groups | 反复挂载依赖 cleanup | 已具备 dispose，文档确认 |

回归建议：SSE 停启风暴、轨迹新建后高亮不串味、关窗无挂死、前端反复进出场景页无 WebGL 泄漏。
