# TASK — 脚本建模

## T1 文档与示例

- 输入：无
- 输出：ALIGNMENT/CONSENSUS/DESIGN/TASK + `examples/*.json`
- 验收：两示例可被判别逻辑识别

## T2 插件导入导出

- 输入：T1；现有 Host/AI API
- 输出：Ribbon 四命令 + handlers
- 验收：导出再导入一致；compose 示例可跑

## T3 ROADMAP/FEATURES + ACCEPTANCE/FINAL

## T4 双配置编译 Plugin（依赖链按需）

## T5 Python 桥接

- 输入：T2
- 输出：`cloudsim_geom` 四 API + 控制台对话框
- 验收：控制台 `import cloudsim_geom; cloudsim_geom.list_bodies()`
