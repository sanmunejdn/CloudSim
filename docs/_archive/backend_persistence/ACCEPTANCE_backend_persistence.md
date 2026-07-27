# ACCEPTANCE_backend_persistence

## 当前执行进度

- [x] T1: `project.json v4` 版本切换与加载强校验
- [x] T2(首版): `BackendDataBase` 序列化模板方法骨架（`saveToJson/loadFromJson`）
- [x] T3(首版): `MeshBackendData/PointCloudBackendData` 派生序列化钩子实现
- [x] T4(首版): 通过 `BackendRegistry` 实现 `className -> create` 工厂加载
- [x] T5(首版): 组件统一序列化通路（先支持 FollowAttachment）
- [x] T6(首版): `MainWindowProjectIo` 对象读写切换到多态序列化主流程
- [~] T7: 回归测试与兼容性验证（环境阻塞，已产出执行清单）

## 本轮落地项

## 1) v4 强校验

- 保存版本号已从 `3` 切换到 `4`
- 打开工程时若 `version != 4`，直接中断并告警

## 2) 属性持久化首版（PropertyBag）

- 保存对象时新增 `propertyBag` 字段落盘
- 加载对象时按值类型恢复 `propertyBag`
  - 支持：`bool/int/double/string/[3]/[4]`
  - 不支持类型：记录 warning 并跳过

## 3) 后端基类多态骨架

- `BackendDataBase` 新增：
  - `saveToJson()`
  - `loadFromJson(...)`
  - `saveDerivedJson(...)`
  - `loadDerivedJson(...)`
- 基类统一处理：
  - `id/name/className`
  - `pose/rotation/color`
  - `poseReferenceFrame`
  - `worldMatrix`
  - `propertyBag`

## 4) 派生类型扩展

- `MeshBackendData`：
  - 保存/恢复 `geometry(kind=triangles, xyzBase64)`
  - 保存/恢复 `transformPivotAtOrigin`
- `PointCloudBackendData`：
  - 保存/恢复 `geometry(kind=points, xyzBase64, rgbaPerVertexBase64, pointCount)`

## 5) 对象工厂与主流程切换（T4/T6 首版）

- 加载对象时改为：
  - `ensureBackendBuiltinsRegistered()`
  - `BackendRegistry::instance().create(className)`
  - `BackendDataBase::loadFromJson(...)`
- 保存对象时改为：
  - `BackendDataBase::saveToJson()` 统一导出对象数据
- `MainWindowProjectIo` 不再维护点云/网格的几何解码分支（对象级逻辑下沉到后端类型）

## 6) 组件统一序列化（T5 首版）

- `BackendDataBase::saveToJson` 输出 `components` 数组
- `BackendDataBase::loadFromJson` 从 `components` 恢复组件
- 已接入 `FollowAttachmentComponent` 的读写
- 组件注册入口已独立为 `ensureBackendComponentCodecBuiltinsRegistered()`，风格与后端类型 builtins 对齐
- 组件注册表已接入告警钩子：重复注册、未知类型、编解码失败会通过 `RunLogger::warn` 输出
- 兼容兜底：若没有 `components` 但存在旧字段 `followAttachment`，仍可恢复
- `MainWindowProjectIo` 移除 `followAttachment` 手工拼装/回填逻辑

## 7) T7 当前状态

- 已完成：
  - 回归项拆解与执行脚本：`REGRESSION_CHECKLIST_backend_persistence.md`
  - 已在当前环境完成静态诊断（lints 无新增问题）
- 阻塞项：
  - 当前环境无法执行 `msbuild` / `devenv`（命令不存在），无法完成编译级回归
- 结论：
  - 代码层已具备进入回归阶段条件，待补齐构建工具后可完成 T7 收口

## 风险与备注

- 对象加载已切到 `BackendRegistry + loadFromJson`，但尚未抽离出独立 `BackendFactory` 门面类
- 组件序列化暂未统一注册化，`FollowAttachment` 仍走既有路径
- 工程文件不再写点云 sidecar，统一走对象内嵌几何（文件体积会增大）
- 尚未完成编译级回归（受构建环境限制）

## 下一步

1. 为组件读写引入显式注册表（已完成首版，当前仅注册 FollowAttachment）
2. 为 `BackendRegistry` 增加轻量 `BackendFactory` 门面（收敛日志与错误语义）
3. 安装/配置 MSBuild 后按清单执行 T7（对象/属性/层级/组件一致性）
