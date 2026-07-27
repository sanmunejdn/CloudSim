# FINAL：后端对象显示/隐藏

## 总结

将场景对象显示/隐藏从 OSG 会话态提升为 `BackendDataBase` 真源，经 `project.json` 的 `objects[].visible` 持久化；工程重开时恢复 NodeMask 与后端树勾选。

## 实现要点

1. Data：`m_visible`（默认 true）+ `saveToJson`/`loadFromJson`
2. Core：`BackendObjectDto.visible`、`IDataService::isVisible`/`setVisible`
3. 写：DocumentPage 与 Facade 均写 Data 再 sync OSG
4. 读：内嵌解码后 apply；文件导入路径从 JSON 补写 `visible`

## 兼容

旧工程无 `visible` → 全部显示（与现网一致）。
