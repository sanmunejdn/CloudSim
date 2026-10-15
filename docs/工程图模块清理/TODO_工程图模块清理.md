# TODO — 工程图模块清理

| 项 | 说明 | 建议 |
|----|------|------|
| 安装包 dist 历史产物 | `Setup/packaging/dist` 可能仍含旧 `com.cloudsim.drawing` | 下次正式打包自动刷新；本地可选手动删 |
| 旧工程侧车字段 | `engineeringDrawing` 可残留于 JSON | 按决策忽略；若需瘦身工程文件可另做剥离工具 |
| CloudSim.exe / CloudSimWeb.exe | 本任务未强制重编顶层 App 工程 | 若本地 exe 过旧，按需编 Debug+Release 各一次做冒烟 |
| 其它插件 DLL | GeometricModeling 已按新 ABI 重编 | 若其它插件也直接链接 GeometryHost 虚表，按需重编 |
