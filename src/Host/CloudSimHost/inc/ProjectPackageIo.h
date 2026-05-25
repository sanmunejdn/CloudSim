#pragma once

#include "BackendProjectObjectIo.h"
#include "cloudsim_host_global.h"

#include <QJsonObject>
#include <QStringList>
#include <QVector>

class OsgWidget;
class IRobotDocumentHost;

namespace cloudsim::host {

class DocumentHost;
class IRobotUrdfImportContext;
struct FollowSolveContext;

struct RobotKinematicsRestoreResult {
	int restoredInstanceCount = 0;
	QVector<double> aggregatedJointAnglesRad; ///< 多机实例关节角拼接，供 Widget 写场景
	QStringList warnings;
};

struct ProjectSaveBuildResult {
	QJsonObject root;
	QString abortMessage; ///< 非空则 Widget 应中止保存（如点云无坐标）
	QStringList warnings;
};

/// 构建工程保存根
CLOUDSIM_HOST_EXPORT ProjectSaveBuildResult buildProjectSaveRoot(DocumentHost& host, const QString& languageCode);

/// 合并运动学到根
CLOUDSIM_HOST_EXPORT void mergeRobotKinematicsIntoProjectRoot(::IRobotDocumentHost* robotDoc, QJsonObject& root,
	const QVector<double>* aggregatedJointAnglesRad = nullptr);

/// 恢复标注与视口
CLOUDSIM_HOST_EXPORT void applyProjectViewportFromJson(DocumentHost& host, const QJsonObject& root);

/// 工程加载收尾
CLOUDSIM_HOST_EXPORT void finalizeProjectLoadFollowAndViewport(DocumentHost& host, OsgWidget& osg,
	const QJsonObject& root, bool useEdgesRelation, const QVector<ProjectHierarchyEdge>& edges,
	const FollowSolveContext* solveCtx = nullptr);

/// 恢复运动学实例
CLOUDSIM_HOST_EXPORT RobotKinematicsRestoreResult restoreRobotKinematicsFromProjectJson(IRobotUrdfImportContext& ctx,
	const QJsonObject& projectRoot);

/// 加载机器人程序
CLOUDSIM_HOST_EXPORT bool loadRobotProgramsFromProjectJson(DocumentHost& host, const QJsonObject& projectRoot,
	QString* outError = nullptr);

/// 合并程序到根
CLOUDSIM_HOST_EXPORT void mergeRobotProgramsIntoProjectRoot(DocumentHost& host, QJsonObject& root);

} // namespace cloudsim::host
