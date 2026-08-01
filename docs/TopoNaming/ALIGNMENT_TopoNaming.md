# ALIGNMENT — TopoNaming（弱命名起步）

> 状态：**范围锁定，本里程碑不实现代码**  
> 指针：`docs/几何建模/ROADMAP.md` §4

## 1. 原始动机

UpToFace / Fillet 边索引等依赖进程内索引，草图或上游特征变更后易失效。对标 Yi3D `generateShape(TopoNaming*)` 思路，自研渐进弱命名，**不抄源码、不做完整 FreeCAD/SW 命名引擎**。

## 2. 边界确认

| 做 | 不做 |
|----|------|
| P0 弱命名表进 history JSON | 完整拓扑命名引擎 |
| P0 消费方：UpToFace、Fillet 优先走命名表，失败降级索引 | 移植 Yi3D Transaction/Element |
| P1 rebuild 维护映射 + Host 查询 API（ABI bump） | 装配级命名 |
| 包 D（视口点选、Suppress）可并行 | 阻塞包 B+C 已交付项 |

## 3. 建议阶段

| 阶段 | 内容 |
|------|------|
| **P0** | `faceRef` / `edgeRef` = `{ afterFeatureId, localIndex }` 或稳定 hash；写入 Parametric history |
| **P0 消费** | UpToFace、Fillet 边：命名表优先，失败回退现索引 |
| **P1** | rebuild 时维护映射；Host 查询 API + ABI bump |
| **P2+** | 视需要扩展到 Pattern / 成角关联（另立项） |

## 4. 与现有系统对齐

- 落点：`ParametricBrepBackendData` rebuild、`FeatureDocument` JSON、Host pick/query
- 现有 `faceOwnerByIndex` / tipAfter 可作为过渡，不替代命名表
- 与包 D（点选硬化、Suppress）接口草图可同 6A 拆任务

## 5. 未决（启动专题时再问）

1. localIndex 相对「特征贡献体」还是「整 tip」？
2. hash 算法与冲突策略？
3. 是否与 Suppress 同一里程碑交付 P0？

## 6. 下一步

新建完整 6A（CONSENSUS → DESIGN → TASK）后再编码；本文件仅锁定范围。
