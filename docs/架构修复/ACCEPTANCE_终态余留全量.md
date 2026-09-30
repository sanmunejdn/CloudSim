# ACCEPTANCE：终态余留全量

BugTracker **#10**。窗口验收（Debug|x64 + Release|x64，除非另注）。

| 窗口 | 内容 | 结果 |
|------|------|------|
| W1 | 插件迁窄接口；geometry/document 追加 pointCloud/labelingHost | 通过 |
| W2 | `IPluginHostContext` 砍刀；ABI/IID `0x00013A00`；全插件重编 | 通过 |
| W3 | LabelingSDK 移出 sln 并删除目录 | 通过 |
| W4 | Config `ensureLoaded` → thread_local | 通过 |
| W5 | Viewport 去 `CloudSimHost.lib`；无双向依赖 | 通过 |
| W6 | `test_lifecycle_contract.py` 挂入 API/run_tests；3 passed | 通过 |
| W7 | 本 DESIGN/ACCEPTANCE + BugTracker | 通过 |

## 成功标准核对

1. 聚合无域业务虚函数；IID=`0x00013A00`；`check_plugin_iid_alignment.py` 绿
2. 无 LabelingSDK 工程
3. Config 无进程成员 `m_resourceBaseDir` 踩踏
4. Viewport dumpbin 无 `CloudSimHost.dll` 导入；WHOLEARCHIVE 仅 Core→Host/Headless
5. `tests/api/test_lifecycle_contract.py` 覆盖 SSE 风暴 / PathPlan / session 探活
6. 相关工程双配置绿

## 余留（非本轮）

- Core `/WHOLEARCHIVE` 仍保留（再导出契约）
- Three.js dispose 无独立 e2e（契约表已注明）
- JobSystem shutdown 依赖 SelfTest/桌面路径，无单独 pytest
