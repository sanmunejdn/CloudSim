# ACCEPTANCE — 自定义设备导出 URDF

## 编译

| 工程 | Debug\|x64 | Release\|x64 |
|------|------------|--------------|
| RobotUrdf | ✅ | ✅ |
| CloudSimHost | ✅ | ✅ |
| RobotWidget | ✅ | ✅ |
| Widget | ✅ | ✅ |

## 功能清单

| # | 标准 | 状态 |
|---|------|------|
| 1 | 导出目录含 `package.xml`、`urdf/*.urdf`、`meshes/` 或 `cad/` | 已实现 |
| 2 | `.urdf` 长度单位为米（内部 mm÷1000） | 已实现 |
| 3 | 优先拷贝源 OBJ/STEP；否则 Mesh→PLY / Brep→STEP | 已实现 |
| 4 | 设备页「导出 URDF…」 | 已实现 |
| 5 | 组装对话框「导出 URDF…」（需先 Apply） | 已实现 |
| 6 | 导出 → 现有导入 URDF 回灌 | **待手工验收** |

## 手工回灌步骤

1. 组装自定义设备（≥2 Link、≥1 运动副）→ 应用  
2. 设备页或组装对话框 → 导出 URDF → 选父目录  
3. 确认生成 `<pkg>/package.xml`、`urdf/<pkg>.urdf`、几何文件  
4. 设备页导入该 `.urdf`（或资源扫描若拷入 models）  
5. 轴控拖动关节，确认运动方向与限位大致正确  

## 已知约束

- 组装对话框导出要求已 **Apply** 的 Link/Joint 图  
- 无源路径且非 Mesh/Brep 的几何会失败并提示  
- visual origin 当前为单位；依赖网格本地系与连杆系一致
