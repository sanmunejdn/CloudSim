#ifndef CLOUDSIMHOST_BACKENDPROJECTOBJECTIO_H
#define CLOUDSIMHOST_BACKENDPROJECTOBJECTIO_H

/// @file BackendProjectObjectIo.h
/// @brief 工程父子边

#include "cloudsim_host_global.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QSet>
#include <QStringList>
#include <QVector>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class BackendDataBase;
class PointCloudBackendData;
class OsgWidget;

namespace cloudsim::host
{
class DocumentHost;

/// 工程父子边
struct ProjectHierarchyEdge
{
	QString parentId;
	QString childId;
};

/// 单对象落盘
CLOUDSIM_HOST_EXPORT QJsonObject saveProjectObject(DocumentHost& host, const QString& objectId,
												   const QString& sourcePath, const QString& sourceType,
												   const QString& parentId);

/// 仅解码 JSON
CLOUDSIM_HOST_EXPORT bool decodeBackendObjectFromProjectJson(const QJsonObject& objectJson,
															 std::shared_ptr<BackendDataBase>& out,
															 QString* outError = nullptr);

/// 内嵌几何注册
CLOUDSIM_HOST_EXPORT bool registerEmbeddedProjectObject(DocumentHost& host, const QJsonObject& objectJson,
														const QString& persistedId, const QString& sourcePath,
														const QString& catalogTypeName, const QString& parentId,
														bool robotLinkMeshVisual, const QString& projectDir = QString(),
														QString* outVisualError = nullptr, QString* outError = nullptr);

/// 文件回退导入
CLOUDSIM_HOST_EXPORT QString importProjectObjectFromFile(DocumentHost& host, const QString& loadPath,
														 const QString& persistedId, const QString& catalogTypeName,
														 bool isPointCloud, QString* outError = nullptr);

CLOUDSIM_HOST_EXPORT QVector<ProjectHierarchyEdge> parseProjectEdgesJson(const QJsonArray& edgesJson);

/// edges 写 Data
CLOUDSIM_HOST_EXPORT void applyProjectEdgesToBackend(DocumentHost& host, const QVector<ProjectHierarchyEdge>& edges,
													 QStringList* outWarnings = nullptr);

/// Data 同步 OSG 父链
CLOUDSIM_HOST_EXPORT void syncOsgBackendParentsFromBackend(DocumentHost& host);

/// 重建 parentId 旁路
CLOUDSIM_HOST_EXPORT void rebuildBackendParentIdMirror(DocumentHost& host);

/// 点云 pose 回写
CLOUDSIM_HOST_EXPORT void applyPointCloudPoseFromProjectJson(PointCloudBackendData& pc, OsgWidget* osgWidget,
															 const QJsonObject& objectJson);

struct ProjectObjectLoadOptions
{
	QString projectDir;
	bool useEdgesRelation = false;
	QSet<QString> robotLinkMeshBackendIds;
};

/// 点云 Widget 回退
using ProjectPointCloudWidgetImportFn =
	std::function<bool(DocumentHost& host, const QString& loadPath, const QString& persistedId, QString& outImportedId,
					   QString* outError)>;

/// 旧工程 parent Follow
using ProjectHierarchyFollowFn = std::function<void(const std::string& childId, const std::string& parentId)>;

struct ProjectObjectLoadCallbacks
{
	ProjectPointCloudWidgetImportFn pointCloudWidgetImport;
	ProjectHierarchyFollowFn legacyParentFollow;
};

/// 加载 objects[]
CLOUDSIM_HOST_EXPORT void loadProjectObjectsFromJson(DocumentHost& host, const QJsonArray& objects,
													 const ProjectObjectLoadOptions& options,
													 const ProjectObjectLoadCallbacks& callbacks,
													 QStringList* outWarnings = nullptr);

/// objects 加载收尾
CLOUDSIM_HOST_EXPORT void finalizeProjectHierarchyAfterObjects(DocumentHost& host, bool useEdgesRelation,
															   const QVector<ProjectHierarchyEdge>& edges,
															   QStringList* outWarnings = nullptr);

struct FollowSolveContext;

/// edges Follow 绑定（OSG 由 Host 内部解析）
CLOUDSIM_HOST_EXPORT void applyProjectEdgesFollowBindingAndSolve(DocumentHost& host,
																 const QVector<ProjectHierarchyEdge>& edges,
																 const FollowSolveContext* solveCtx = nullptr);

/// Web：导出显示用 triangle soup（Mesh 直取；Brep 现场 Medium 离散）
CLOUDSIM_HOST_EXPORT bool exportBackendTriangleSoupMm(DocumentHost& host, const QString& backendId,
													  std::vector<float>& outSoup, QString* outError = nullptr);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_BACKENDPROJECTOBJECTIO_H
