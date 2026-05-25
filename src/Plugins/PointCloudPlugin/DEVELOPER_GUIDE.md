# PointCloudPlugin 示例

点云处理插件，演示 **1.2.0+** SDK：`IPluginPointCloudHost` + 侧栏 UI。

## 构建与部署

| 项 | 说明 |
|----|------|
| 工程 | `PointCloudPlugin.vcxproj`（x64，v142，Qt 5.14.2） |
| 链接 | **仅** `CloudSimPluginSDK.lib` |
| 部署 | `bin/x64(d)/plugins/com.cloudsim.pointcloud/plugin.json` + `PointCloudPlugin.dll` |
| `minHostVersion` | `"1.2.0"` |

## 运行时

- 侧栏 Tab **点云** / **Point Cloud**：导入、列表、下采样、裁剪、预处理、ICP、重建
- 菜单 **Tools → Point Cloud**（中文下子菜单标题为 **点云**）
- 语言：默认中文；切换 **设置 → Language → 中文/English** 时侧栏与菜单同步更新
- **重建网格 → 导出 PLY**：侧栏「重建网格」区选网格对象，点 **导出 PLY…**（或菜单 **Tools → 点云 → 导出网格 PLY…**）

典型流程：

1. `importFileIntoActiveDocument(path, true)` 得 `backendId`
2. `doc->queryPointCloudInfo(id, info)` 显示点数
3. `host->pointCloudHost()->downsamplePointCloudVoxel(doc, id, params, onFinished)`
4. `onFinished` 中刷新列表；场景由宿主自动 `loadPointCloudFromBackendData`

## 相关文档

- SDK：[`../CloudSimPluginSDK/DEVELOPER_GUIDE.md`](../CloudSimPluginSDK/DEVELOPER_GUIDE.md)
- 宿主：[`../../UI/CloudSimPluginHost/DEVELOPER_GUIDE.md`](../../UI/CloudSimPluginHost/DEVELOPER_GUIDE.md)
