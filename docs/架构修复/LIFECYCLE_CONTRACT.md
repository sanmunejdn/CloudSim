# 资源生命周期契约（P1-8）

| 区域 | 创建者 | 释放者 | 异常路径 | 本轮修复 | CI |
|------|--------|--------|----------|----------|-----|
| SSE `eventQueue` | `WebGateway::pushEvent` | `/api/events` 消费；**`stop()` 清空** | 队列满丢弃最旧 + `EventsDropped` | `stop()` 清队列 | `tests/api/test_lifecycle_contract.py::test_sse_reconnect_storm_still_alive` |
| `m_faceHighlightSoup` | Headless 轨迹面高亮缓存 | **绑定/新建 PathPlan 时 `clear`** | 文档关闭随 Session 析构 | `createPathPlan` 清缓存 | `test_path_plan_create_twice_ok` + `test_trajectory_session_probe` |
| `JobSystem` 超时弃池 | `MainWindow` 持有 | `shutdown` 等待 `kShutdownWaitMs` 后弃池 | 作业回调必须 `QPointer` | 注释固化契约 | `JobSystemSelfTest.exe`（`runJobSystemLifecycleSelfTest`；挂 `run_selftest.ps1`，无 MainWindow） |
| Three.js unmount | `useViewportCore` | effect cleanup：`renderer.dispose()` + drain groups | 反复挂载依赖 cleanup | 已具备 dispose，文档确认 | **默认 CI 不阻塞**；dispose 路径见 `useViewportCore`（可选后续单独立项 Playwright/vitest） |

回归建议：SSE 停启风暴、轨迹新建后高亮不串味、关窗无挂死、前端反复进出场景页无 WebGL 泄漏。

默认 `scripts/run_tests.ps1` → `run_api_tests.ps1` 已包含 `tests/api/test_lifecycle_contract.py`。
