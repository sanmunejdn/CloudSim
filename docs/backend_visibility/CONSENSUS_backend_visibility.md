# CONSENSUS：后端对象显示/隐藏持久化

## 需求

后端对象显示/隐藏以 Data 为真源，写入 `project.json`，重开工程后恢复树勾选与场景可见性。

## 范围

- 含：`BackendDataBase` 派生场景对象（点云/网格/Brep/坐标系等）
- 不含：注释（已有 `visible`）、仿真指令分组隐藏

## 验收标准

1. 隐藏若干后端 → 保存 → 重开：对象仍隐藏，树 Unchecked，场景不可见
2. 旧工程（无 `visible`）打开：全部显示
3. 新建/导入对象：默认 `visible=true`
4. Data 层无 OSG 依赖；字段经 `saveToJson`/`loadFromJson` 往返

## 技术约束

- 不 bump `project.json` version（仍 v4）；缺字段默认 `true`
- 写路径：先 Data，再 OSG NodeMask
- 契约：`IDataService::isVisible` / `setVisible`；`BackendObjectDto.visible`
