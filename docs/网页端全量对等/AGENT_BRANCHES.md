# 网页端全量对等 — Agent 分支命名

| Agent ID | 分支名 | 主要目录 |
|----------|--------|----------|
| W0 | `web-parity/w0-coordinator` | `docs/网页端全量对等/`、`WebGateway*.cpp` 拆分 |
| W1-R1 | `web-parity/w1-r1-playback` | `HeadlessRobotPlaybackBridge`、`WebGatewayRobotPlayback.cpp` |
| W1-R2 | `web-parity/w1-r2-export` | `HeadlessRobotExportBridge` |
| W1-G1 | `web-parity/w1-g1-geometry` | `HeadlessGeometryBridge`、`WebGatewayGeometry.cpp` |
| W1-AI | `web-parity/w1-ai-bridge` | `HeadlessAiBridge`、`WebGatewayAi.cpp` |
| W2-ROB | `web-parity/w2-rob-collision` | `HeadlessRobotCollisionBridge` |
| W2-PRG | `web-parity/w2-prg-edit` | `HeadlessProgramEditBridge` |
| W2-TRJ | `web-parity/w2-trj-ui` | `web/cloudsim-web-ui/src/docks/robot/` |
| W2-SIDE | `web-parity/w2-side-plugins` | `docks/geometry/`、`PlcPanel`、`CameraPanel` |
| W3-SHELL | `web-parity/w3-shell-router` | `workspaces/`、`WorkspaceModeRouter.tsx` |
| W3-PF | `web-parity/w3-pf-workspace` | `HeadlessProcessFlowBridge`、`workspaces/processflow/` |
| W3-ED | `web-parity/w3-ed-workspace` | `HeadlessDrawingBridge`、`workspaces/drawing/` |
| W3-GM | `web-parity/w3-gm-workspace` | `HeadlessGeomodelBridge`、`workspaces/geomodeling/` |
| W3-LB | `web-parity/w3-lb-workspace` | `HeadlessLabelingBridge`、`workspaces/labeling/` |
| W4-DOC | `web-parity/w4-doc-assembly` | `DocTabs`、assembly/annotations API |
| W4-UX | `web-parity/w4-ux-i18n` | i18n/theme/help |
| W4-INF | `web-parity/w4-inf-debt` | SSE、WebGL dispose |

合并顺序：W0 → W1 → W2/W3 → W4。Gateway 路由仅改对应 `WebGateway<Domain>.cpp`。
