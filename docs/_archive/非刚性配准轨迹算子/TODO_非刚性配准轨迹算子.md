# TODO — 非刚性配准轨迹算子

## 可选后续

- [ ] HPL 式 `_deformed.bin` / PLY 导出缓存（「直接读取结果」模式）
- [ ] 绑定点刚性回退（HPL `applyBindingsToPath` fallback）
- [ ] 大 mesh 绑定的空间加速（BVH），当前为全三角面扫描

## 编译验证

在 VS x64 下编译 `TrajectoryAlgorithmBuiltins` + `RobotScene`，确认无链接错误。
