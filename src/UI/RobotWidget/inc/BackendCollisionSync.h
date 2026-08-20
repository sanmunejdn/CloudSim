#ifndef ROBOTWIDGET_BACKENDCOLLISIONSYNC_H
#define ROBOTWIDGET_BACKENDCOLLISIONSYNC_H

/// @file BackendCollisionSync.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 从文档后端填充 CollisionWorld；同实例连杆不做自碰（无 SRDF），检对场景

#include "robotwidget_global.h"

#include "CollisionWorld.h"
#include "RobotCollisionSettings.h"

#include <QHash>
#include <QString>
#include <QVector>
#include <string>
#include <vector>

class BackendDataManager;
class IRobotDocumentHost;
class IRobotOsgViewHost;

namespace BackendCollisionSync
{
/// 用当前文档几何重建 world（清空后写入）
/// osg 非空时优先用 OSG 组合世界矩阵（Gizmo/父子挂载后与画面一致；backend.worldMatrix 可能仍是导入初值）
ROBOTWIDGET_EXPORT void rebuildWorld(collision::CollisionWorld& world, IRobotDocumentHost* doc,
									 BackendDataManager& backend, const RobotCollision::Settings& settings,
									 IRobotOsgViewHost* osg = nullptr);

/// 仅刷新已有 body 的世界位姿（FK/Gizmo 后）
ROBOTWIDGET_EXPORT void updatePoses(collision::CollisionWorld& world, IRobotDocumentHost* doc,
									BackendDataManager& backend, IRobotOsgViewHost* osg = nullptr);

/// 对关节轨迹抽样碰撞；命中写 failSummary，返回 false
ROBOTWIDGET_EXPORT bool validateJointTrajectory(collision::CollisionWorld& world, IRobotDocumentHost* doc,
												BackendDataManager& backend, int instanceIndex,
												const QVector<double>& seedJointsBefore,
												const std::vector<std::vector<double>>& jointTrajectoryRad,
												const RobotCollision::Settings& settings, std::string* failSummary,
												IRobotOsgViewHost* osg = nullptr);

} // namespace BackendCollisionSync

#endif // ROBOTWIDGET_BACKENDCOLLISIONSYNC_H
