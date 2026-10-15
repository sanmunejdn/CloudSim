# ACCEPTANCE — 网页端主程序裁剪

| 项 | 状态 | 说明 |
|----|------|------|
| 删除四工作区壳 + geomodeling API 客户端 | 完成 | `workspaces/*`、`api/geomodeling.ts` |
| Router / MenuBar / App / RightDock / projectStore / AiPanel | 完成 | 仅主程序 |
| Gateway 停注册专用路由 | 完成 | `WebGatewayParity.cpp` |
| modes + workspaceMode 归一 | 完成 | `WebGatewayApi.cpp` |
| 测试更新 | 完成 | `test_geomodeling`→404；probe/soak 改主程序路径 |
| 文档 | 完成 | README / API_CONTRACT / 08 / 本目录 CONSENSUS |
| Debug + Release 构建验证 | 完成 | `CloudSimWeb` Debug|x64 + Release|x64；`npm run build:debug` + `build:release` |
| 抽查已删 API | 完成 | geomodeling/processflow/drawing/labeling → 404；modes 仅 scene3d；workspaceMode 归一 |

## 回归要点

- 访问已删 API → 404
- `PUT /api/sidecar/workspaceMode` `{mode:geomodeling}` → 读回 `scene3d`
- 侧车 `geometricModeling` roundtrip 仍成功
