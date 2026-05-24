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

/// 工程 v4 数据段（robotKinematics 由 mergeRobotKinematicsIntoProjectRoot 合并）
CLOUDSIM_HOST_EXPORT ProjectSaveBuildResult buildProjectSaveRoot(DocumentHost& host, const QString& languageCode);

/// 保存前写入 robotKinematics / robotKinematicsInstances（关节角由 Widget 采集）
CLOUDSIM_HOST_EXPORT void mergeRobotKinematicsIntoProjectRoot(::IRobotDocumentHost* robotDoc, QJsonObject& root,
	const QVector<double>* aggregatedJointAnglesRad = nullptr);

/// 从工程 JSON 恢复标注与相机跟随
CLOUDSIM_HOST_EXPORT void applyProjectViewportFromJson(DocumentHost& host, const QJsonObject& root);

/// 工程加载收尾：父链 → edges 跟随 → 视口 → 强制 Follow（顺序固定）
CLOUDSIM_HOST_EXPORT void finalizeProjectLoadFollowAndViewport(DocumentHost& host, OsgWidget& osg,
	const QJsonObject& root, bool useEdgesRelation, const QVector<ProjectHierarchyEdge>& edges,
	const FollowSolveContext* solveCtx = nullptr);

/// robotKinematicsInstances + 兼容单条 robotKinematics
CLOUDSIM_HOST_EXPORT RobotKinematicsRestoreResult restoreRobotKinematicsFromProjectJson(IRobotUrdfImportContext& ctx,
	const QJsonObject& projectRoot);

/// 非空 robotPrograms[] 时写入 RobotProgramStore
CLOUDSIM_HOST_EXPORT bool loadRobotProgramsFromProjectJson(DocumentHost& host, const QJsonObject& projectRoot,
	QString* outError = nullptr);

/// 保存根 JSON：写入或移除 robotPrograms
CLOUDSIM_HOST_EXPORT void mergeRobotProgramsIntoProjectRoot(DocumentHost& host, QJsonObject& root);

} // namespace cloudsim::host
