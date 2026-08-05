# TASK_网页端_P1 — CAD 壳

## 原子任务

1. `POST /api/project/new|save` + STORE pack → 验收：`.pcp` 互开  
2. `PATCH/DELETE/import/register/attach` 对象 API → 验收：属性/pose/可见性写回  
3. SSE：`PoseCommitted` / `BackendObject*` / `ProjectLoaded|Saved`  
4. 前端壳：菜单 + 树 + 属性 + TransformControls gizmo + 聚焦  

依赖：M0/M1 Gateway 骨架。后置：P2。
