#pragma once

#include "cloudsim_host_global.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QSet>
#include <QStringList>
#include <QVector>

#include <functional>
#include <memory>
#include <string>

class BackendDataBase;
class PointCloudBackendData;
class OsgWidget;

namespace cloudsim::host {

class DocumentHost;

/// 工程 edges[] 一条父子边
struct ProjectHierarchyEdge {
	QString parentId;
	QString childId;
};

/// 单对象落盘：IDataService 序列化 + 旁路 sourcePath/sourceType/parentId
CLOUDSIM_HOST_EXPORT QJsonObject saveProjectObject(DocumentHost& host, const QString& objectId,
	const QString& sourcePath, const QString& sourceType, const QString& parentId);

/// 仅解码 JSON，不注册、不发事件
CLOUDSIM_HOST_EXPORT bool decodeBackendObjectFromProjectJson(const QJsonObject& objectJson,
	std::shared_ptr<BackendDataBase>& out, QString* outError = nullptr);

/// 内嵌几何：解码 → OSG → registerAdopted（发 BackendObjectRegistered）
CLOUDSIM_HOST_EXPORT bool registerEmbeddedProjectObject(DocumentHost& host, const QJsonObject& objectJson,
	const QString& persistedId, const QString& sourcePath, const QString& catalogTypeName, const QString& parentId,
	bool robotLinkMeshVisual, QString* outVisualError = nullptr, QString* outError = nullptr);

/// 工程文件回退：网格 importMeshFile；点云 ply/xyz（las/laz 失败由 Widget 兜底）
CLOUDSIM_HOST_EXPORT QString importProjectObjectFromFile(DocumentHost& host, const QString& loadPath,
	const QString& persistedId, const QString& catalogTypeName, bool isPointCloud, QString* outError = nullptr);

CLOUDSIM_HOST_EXPORT QVector<ProjectHierarchyEdge> parseProjectEdgesJson(const QJsonArray& edgesJson);

/// 将 edges 写入 BackendDataManager（不处理 Follow/OSG）
CLOUDSIM_HOST_EXPORT void applyProjectEdgesToBackend(DocumentHost& host, const QVector<ProjectHierarchyEdge>& edges,
	QStringList* outWarnings = nullptr);

/// 从 Data 父子关系同步 OSG 父链
CLOUDSIM_HOST_EXPORT void syncOsgBackendParentsFromBackend(DocumentHost& host);

/// 从 backendParentId 旁路表重建（工程加载 edges 后）
CLOUDSIM_HOST_EXPORT void rebuildBackendParentIdMirror(DocumentHost& host);

/// 文件回退点云：从工程 JSON 恢复 pose/rotation/color 到后端与 OSG 选中态
CLOUDSIM_HOST_EXPORT void applyPointCloudPoseFromProjectJson(PointCloudBackendData& pc, OsgWidget* osgWidget,
	const QJsonObject& objectJson);

struct ProjectObjectLoadOptions {
	QString projectDir;
	bool useEdgesRelation = false;
	QSet<QString> robotLinkMeshBackendIds;
};

/// 点云文件导入失败时的回退（通常走 DocumentImportFacade）
using ProjectPointCloudWidgetImportFn = std::function<bool(DocumentHost& host, const QString& loadPath,
	const QString& persistedId, QString& outImportedId, QString* outError)>;

/// 无 edges[] 时按 JSON parentId 写 Follow（旧工程）
using ProjectHierarchyFollowFn = std::function<void(const std::string& childId, const std::string& parentId)>;

struct ProjectObjectLoadCallbacks {
	ProjectPointCloudWidgetImportFn pointCloudWidgetImport;
	ProjectHierarchyFollowFn legacyParentFollow;
};

/// 遍历 objects[]：内嵌 / 文件回退；告警写入 outWarnings
CLOUDSIM_HOST_EXPORT void loadProjectObjectsFromJson(DocumentHost& host, const QJsonArray& objects,
	const ProjectObjectLoadOptions& options, const ProjectObjectLoadCallbacks& callbacks,
	QStringList* outWarnings = nullptr);

/// objects 加载后：edges → Data；重建 backendParentId 旁路表
CLOUDSIM_HOST_EXPORT void finalizeProjectHierarchyAfterObjects(DocumentHost& host, bool useEdgesRelation,
	const QVector<ProjectHierarchyEdge>& edges, QStringList* outWarnings = nullptr);

struct FollowSolveContext;

/// 工程 edges：补 hierarchyDriven Follow 并一次求解（annotations/机器人 UI 在 ProjectPackageIo）
CLOUDSIM_HOST_EXPORT void applyProjectEdgesFollowBindingAndSolve(DocumentHost& host, OsgWidget& osg,
	const QVector<ProjectHierarchyEdge>& edges, const FollowSolveContext* solveCtx = nullptr);

} // namespace cloudsim::host
