# RobotKinematics 模块开发文档

## 1. 模块定位

`RobotKinematics` 提供：① **CircularArcGeometry**（圆弧采样，与 DH 无关）；② **SerialLinkKinematics** 修正 DH FK + **仅位置** DLS（**legacy**：仅无 URDF 时回退）。有 URDF 时生产路径保持空 `dhRows`，IK 走 `RobotUrdf::UrdfNumericalIk` / `RobotTeachIk`。

| 属性 | 说明 |
|------|------|
| 单位 | 长度 **mm**；角度 **rad**（关节变量） |
| 矩阵布局 | 4×4 齐次变换，**列主序** 16 double（与 OSG/`BackendMat4` 一致） |
| x64 输出 | `RobotKinematics.dll` |
| 导出 | `ROBOT_KINEMATICS_API`（`robot_kinematics_global.h`） |
| 构建定义 | x64：`ROBOT_KINEMATICS_LIB`；Win32：`ROBOT_KINEMATICS_STATIC` |

架构图：[`../../../docs/_archive/robot-kinematics-workspace/diagrams/target-architecture.html`](../../../docs/_archive/robot-kinematics-workspace/diagrams/target-architecture.html)

---

## 2. 核心类型

### 2.1 `struct DhRow`

**含义**：单节修正 DH 参数，对应  
\(A_i = R_z(\theta_i) \cdot T_z(d_i) \cdot T_x(a_i) \cdot R_x(\alpha_i)\)。

| 字段 | 类型 | 说明 |
|------|------|------|
| `a` | `double` | 连杆长度 |
| `alpha` | `double` | 扭角 |
| `d` | `double` | 偏距（棱柱时为名义 d） |
| `thetaOffset` | `double` | 固定偏置；旋转关节 θ = offset + q |
| `jointIndex` | `int` | 在关节向量 `q` 中的下标；`< 0` 无变量 |
| `isPrismatic` | `bool` | true：变量进 `d`（mm）；false：进 θ（rad） |

实现为**闭式单矩阵** FK；IK 使用**解析位置雅可比**（旋转列 `z×(p_ee-p_j)`，棱柱列 `z`），并对棱柱/旋转分别限幅步进。

---

## 3. 公共函数（`SerialLinkKinematics.h`，命名空间 `robot_kinematics`）

| 函数 | 输入 | 输出 | 作用 |
|------|------|------|------|
| `fkSerialDh` | `rows`, `q` | `T_end4x4_colMajor[16]` | 末端执行器 FK |
| `endEffectorPosition` | `rows`, `q` | `posOut[3]` | 仅取 FK 平移列 |
| `ikPositionDampedLeastSquares` | `rows`, `targetPos[3]`, `qInOut`, … | `bool` | 位置 DLS IK |
| `positionJacobianAnalytic` | `rows`, `q` | `J_3xn` | 解析位置雅可比 |
| `jointCountFromDhRows` | `rows` | `size_t` | `max(jointIndex)+1` |

### 3.1 IK 使用注意

- **仅位置**：不约束姿态；带欧拉角的 PTP/LINE 应优先走 URDF IK。
- **种子敏感**：`qInOut` 初值影响收敛。
- **外轴联动**：对象级配置与搜索在 RobotScene / 轨迹 Op；本库可对棱柱 DH 行参与联立。

---

## 4. 与上层集成

| 调用方 | 何时使用 |
|--------|----------|
| `Controller::setDhRows` | 仅无 URDF 时由上层注入；有 URDF 时 RobotWidget 跳过自动建表 |
| `PtpPlanner` / `LinePlanner` / `ArcPlanner` | `context.urdfPath` 为空且 `hasDhRows()` 时（ARC 优先 URDF 笛卡尔弧） |

### 3.2 圆弧几何（`CircularArcGeometry.h`）

| 函数 | 作用 |
|------|------|
| `fitCircle3Points` | 三点定圆；共线/半径过小 → false |
| `sampleArcByChord` | 按弦长采样（不含起点、含终点） |
| `pointOnArc` / `arcLengthMm` | 弧参数 u∈[0,1]、弧长 mm |

---

## 5. 扩展指南

- `isPrismatic` 默认 false，保持与旧 ABI 兼容。
- 7 轴/闭链建议新命名空间，勿破坏现有 `DhRow` 布局语义。
- 圆弧仅几何；IK 仍在 `RobotScene::ArcPlanner`。

---

## 6. 相关文档

- [`../RobotScene/DEVELOPER_GUIDE.md`](../RobotScene/DEVELOPER_GUIDE.md)
- [`../RobotUrdf/DEVELOPER_GUIDE.md`](../RobotUrdf/DEVELOPER_GUIDE.md)
- [`../../../docs/三点圆弧指令/CONSENSUS_三点圆弧指令.md`](../../../docs/_archive/三点圆弧指令/CONSENSUS_三点圆弧指令.md)
- [`../../../docs/外部轴联动求解/FINAL_外部轴联动求解.md`](../../../docs/_archive/外部轴联动求解/FINAL_外部轴联动求解.md)
