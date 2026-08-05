# ACCEPTANCE — 网页端轨迹对等

| 阶段 | 标准 | 状态 |
|------|------|------|
| 1 | PathPlan 创建/绑定；开始修改门闩 | 通过（Host+Gateway+UI） |
| 2 | 视口 BREP 点选 → 特征 → 离散 → Raw | 通过 |
| 3 | Mesh 截面 → Raw；B样条需索引 | 通过（截面主路径） |
| 4 | 配方/管线预览/Apply→LINE | 通过 |
| 5 | 全算子调色板 + 模板 + undo | 通过 |
| 6 | 指令树 bind + API/TODO/6A | 通过 |

## 编译
- CloudSimHost / CloudSimWebGateway / TrajectoryAlgorithm：Debug\|x64 + Release\|x64
- `public-fallback` 部署至 `bin\x64d\web` 与 `bin\x64\web`

## 已知限制
- Mesh B样条区域依赖手工三角索引（无桌面 mesh 区域点选）
- 算子参数以 JSON 文本编辑为主（无桌面完整 schema 表单）
