# SDF/DDF 混合非刚性配准

自研模块，与 SPARE **代码树分离**（不修改 `inc/spare/*`、`RegistrationSpare.*`）。

| 项 | 内容 |
|----|------|
| 入口 | `pclalgo::sdfRegister*`（`RegistrationSdf.h`） |
| 实现 | `Geometry/PointCloudAlgorithm/{inc,source}/sdf/` |
| 插件 | 点云侧栏 → 配准 → 方法「SDF/DDF 非刚性」 |
| Host | `IPluginPointCloudHost::nonRigidRegisterSdf`（**1.17.0+**，接口末尾） |

## 要解决什么

在允许局部变形的前提下，把源表面贴到目标表面。与 SPARE 的对称点-面不同，本方法粗阶段用 **目标距离场** 驱动：

- **DDF（默认）**：有向距离 \(\mathbf{d}(x)=x-\Pi(x)\)（点云路径为沿目标法线的有向投影）
- **SDF（可选）**：有符号距离 \(\phi(x)\)，残差 \(\phi\nabla\phi\) 方向

细阶段默认回落 **点-面**（混合方案），参数 `fineDataTerm` 可改回 DDF/SDF。

## 流水线

1. 可选体素预滤波、法线估计/定向、刚性点-面预对齐  
2. 构建目标 `DistanceField`（KdTree + 可选体素三线性缓存）  
3. FPS 变形图 + 蒙皮  
4. **粗**：场残差 + 节点平滑 / Welsch  
5. **细**：点-面（或场）+ 轻量 ARAP 邻域约束  
6. 写回变形 xyz / soup  

## 插件操作

1. 打开点云插件 → **配准**  
2. 方法下拉选 **SDF/DDF 非刚性**  
3. 选择非刚性源 / 目标（点云或网格）  
4. 可选：场模式、场体素、细阶段数据项、刚性预对齐、输出为新对象  
5. 点 **SDF/DDF 配准**  

须与宿主 **同时重编** Host 与 PointCloudPlugin（vtable）。

## 与 SPARE 对照

| | SPARE | SDF/DDF |
|--|-------|---------|
| 粗数据项 | 对称点-面 | DDF / SDF 场 |
| 细数据项 | 对称点-面 | 默认点-面 |
| 代码 | `spare/` | `sdf/`（独立） |
| 专利 | 原 SPARE 声明研究用途 | 自研配方 |

轨迹算子 `NonRigidRegistration` 本阶段仍走 SPARE，未改接 SDF。
