#ifndef CLOUDSIMHOST_ROBOTPROJECTKINEMATICSRESTORE_H
#define CLOUDSIMHOST_ROBOTPROJECTKINEMATICSRESTORE_H

/// @file RobotProjectKinematicsRestore.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 收集连杆网格 id

#include "cloudsim_host_global.h"

#include <QHash>
#include <QJsonObject>
#include <QSet>
#include <QString>
#include <QVector>

class BackendDataManager;
class MeshBackendData;

namespace cloudsim::host
{
class IRobotUrdfImportContext;

/// perLink 连杆 URDF 回退加载（内嵌几何缺失时）
struct RobotLinkUrdfReloadHint
{
	QString urdfPath;
	QString linkName;
};

/// 收集连杆网格 id
CLOUDSIM_HOST_EXPORT QSet<QString> collectRobotLinkMeshBackendIds(const QJsonObject& projectRoot);

/// 收集机器人 scene 根 id（空壳 RobotURDF_*）
CLOUDSIM_HOST_EXPORT QSet<QString> collectRobotSceneRootBackendIds(const QJsonObject& projectRoot);

/// 连杆 backendId → URDF 网格回退
CLOUDSIM_HOST_EXPORT QHash<QString, RobotLinkUrdfReloadHint> collectRobotLinkUrdfReloadHints(
	const QJsonObject& projectRoot);

/// 内嵌几何为空时从 URDF 重载连杆 mesh
CLOUDSIM_HOST_EXPORT bool reloadRobotLinkMeshFromUrdfHint(MeshBackendData& mesh,
															const RobotLinkUrdfReloadHint& hint, QString* outError);

/// 按 URDF 父子表重建 Data 层级（edges 缺失/悬空时兜底）
CLOUDSIM_HOST_EXPORT void reapplyUrdfRobotHierarchyEdges(BackendDataManager& backend, const QString& urdfPath,
														 const QString& sceneRootBackendId,
														 const QHash<QString, QString>& linkNameToBackendId);

/// 遍历 robotKinematicsInstances 统一重建机器人层级
CLOUDSIM_HOST_EXPORT void reapplyAllRobotHierarchyFromProjectJson(class DocumentHost& host,
																  const QJsonObject& projectRoot);

/// 恢复 perLink 运动学
CLOUDSIM_HOST_EXPORT bool restorePerLinkRobotKinematicsFromProjectJson(IRobotUrdfImportContext& ctx,
																	   const QJsonObject& robotKinematicsJson,
																	   QVector<double>& outAllJointAnglesRad,
																	   QString* outWarning = nullptr);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_ROBOTPROJECTKINEMATICSRESTORE_H
