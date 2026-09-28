# ACCEPTANCE — 架构缺陷分级修复

Bug：#3

## P0

| 项 | 结果 |
|----|------|
| P0-1 物理迁移 OsgWidget/PluginHost → Host | 完成；Debug+Release Host/Headless/Widget 通过 |
| P0-2 CloudSimHostShared.items.props + filters Import | 完成；漂移检查 OK |
| P0-3 backend() 棘轮 | `scripts/check_backend_callsites.py` + baseline 147；hook 安装脚本 |

## P1

| 项 | 结果 |
|----|------|
| P1-4 v1 pose/rotation 迁移 | `BackendDataBase::loadFromJson` |
| P1-5 URDF 运行期开关 | `CLOUDSIM_URDF_MESH_BACKEND` |
| P1-6 revision 防护 | Debug 下 elementCount vs revision warn |
| P1-7 线程归属 | `BackendDataManager::assertOwnerThread` |
| P1-8 生命周期 | [LIFECYCLE_CONTRACT.md](LIFECYCLE_CONTRACT.md) + SSE/高亮清缓存 |

## P2

方向文档：[P2_方向_中级缺陷.md](P2_方向_中级缺陷.md)；`make-source-check.ps1` 已落盘。

## 编译验证

- CloudSimHost / Headless / Widget / Data / RobotUrdf / CloudSimWebGateway：Debug\|x64 + Release\|x64
