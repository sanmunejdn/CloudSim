# TASK — 网页端轨迹 UI 对齐

## T1 Host/Gateway feature-schema
- 输入：strategyId 可选
- 输出：JSON schema + defaults / 策略目录
- 验收：curl/浏览器返回 FaceParamSurface 全字段

## T2 HTML/CSS 桌面风 CAD 分区
- 特征表 5 列、拾取行、状态、参数面板、离散模板条
- 验收：布局接近桌面截图信息架构

## T3 JS 逻辑
- 策略中文名、参数表单、自动离散、删面/边、模板 CRUD
- 验收：FaceParamSurface 改行间距后 Raw 点数变化；删 F[] 索引生效

## T4 编译部署文档
- GeometryAlgorithm 无改则只编 Host+WebGateway（若只 Host）；双配置
- 更新 API_网页端.md 一行；ACCEPTANCE
