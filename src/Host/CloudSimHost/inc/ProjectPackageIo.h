#ifndef CLOUDSIMHOST_PROJECTPACKAGEIO_H
#define CLOUDSIMHOST_PROJECTPACKAGEIO_H

/// @file ProjectPackageIo.h
/// @brief 构建工程保存根

#include "cloudsim_host_global.h"

#include "BackendProjectObjectIo.h"
#include "CoreTypes.h"

#include <QJsonObject>
#include <QStringList>
#include <QVector>

namespace cloudsim::host
{
class DocumentHost;
class IRobotUrdfImportContext;

struct RobotKinematicsRestoreResult
{
	int restoredInstanceCount = 0;
	QVector<double> aggregatedJointAnglesRad; ///< 多机实例关节角拼接，供 Widget 写场景
	QStringList warnings;
};

struct ProjectSaveBuildResult
{
	QJsonObject root;
	QString abortMessage; ///< 非空则 Widget 应中止保存（如点云无坐标）
	QStringList warnings;
};

/// 构建工程保存根
CLOUDSIM_HOST_EXPORT ProjectSaveBuildResult buildProjectSaveRoot(DocumentHost& host, const QString& languageCode,
																 const QString& assetOutputDir = QString());

/// 恢复标注与视口
CLOUDSIM_HOST_EXPORT void applyProjectViewportFromJson(DocumentHost& host, const QJsonObject& root);

/// 工程加载收尾（OSG 由 Host 经 DocumentHost 解析，Widget 不传 OsgWidget）
CLOUDSIM_HOST_EXPORT void
finalizeProjectLoadFollowAndViewport(DocumentHost& host, const QJsonObject& root, bool useEdgesRelation,
									 const QVector<ProjectHierarchyEdge>& edges,
									 const cloudsim::core::FollowSolveContextDto* solveCtxDto = nullptr);

/// 恢复运动学实例
CLOUDSIM_HOST_EXPORT RobotKinematicsRestoreResult restoreRobotKinematicsFromProjectJson(IRobotUrdfImportContext& ctx,
																						const QJsonObject& projectRoot);

/// 工程恢复后把聚合关节角写入场景（Widget 不链 RobotScene）
CLOUDSIM_HOST_EXPORT bool applyRestoredJointAnglesToScene(IRobotUrdfImportContext& ctx,
														  const QVector<double>& aggregatedJointAnglesRad,
														  QString* outError = nullptr);

/// 加载机器人程序
CLOUDSIM_HOST_EXPORT bool loadRobotProgramsFromProjectJson(DocumentHost& host, const QJsonObject& projectRoot,
														   QString* outError = nullptr);

/// 合并程序到根
CLOUDSIM_HOST_EXPORT void mergeRobotProgramsIntoProjectRoot(DocumentHost& host, QJsonObject& root);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_PROJECTPACKAGEIO_H
