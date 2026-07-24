#ifndef ROBOTWIDGET_BACKENDCOLLISIONSYNC_H
#define ROBOTWIDGET_BACKENDCOLLISIONSYNC_H

/// @file BackendCollisionSync.h
/// @brief 从文档后端 Mesh/B-rep 填充 CollisionWorld，并做邻接 ACM

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

namespace BackendCollisionSync
{
/// 用当前文档几何重建 world（清空后写入）
ROBOTWIDGET_EXPORT void rebuildWorld(collision::CollisionWorld& world, IRobotDocumentHost* doc,
									 BackendDataManager& backend, const RobotCollision::Settings& settings);

/// 仅刷新已有 body 的世界位姿（FK/Gizmo 后）
ROBOTWIDGET_EXPORT void updatePoses(collision::CollisionWorld& world, IRobotDocumentHost* doc,
									BackendDataManager& backend);

/// 对关节轨迹抽样碰撞；命中写 failSummary，返回 false
ROBOTWIDGET_EXPORT bool validateJointTrajectory(collision::CollisionWorld& world, IRobotDocumentHost* doc,
												BackendDataManager& backend, int instanceIndex,
												const QVector<double>& seedJointsBefore,
												const std::vector<std::vector<double>>& jointTrajectoryRad,
												const RobotCollision::Settings& settings, std::string* failSummary);

} // namespace BackendCollisionSync

#endif // ROBOTWIDGET_BACKENDCOLLISIONSYNC_H
