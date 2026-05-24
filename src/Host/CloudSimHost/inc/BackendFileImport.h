#pragma once

#include "CoreTypes.h"
#include "cloudsim_host_global.h"

#include <memory>

class BackendDataBase;
class MeshBackendData;
class PointCloudBackendData;

namespace cloudsim::host {

class DocumentHost;

/// 单网格文件导入（CGAL 直读）；全格式路由见 DocumentImportFacade
CLOUDSIM_HOST_EXPORT core::ObjectId importMeshFile(DocumentHost& host, const QString& filePath,
	const core::ImportOptionsDto& options, QString* outError = nullptr);

/// 点云导入：ply/xyz（CGAL）或 las/laz（OsgWidget 解码 + capture）
CLOUDSIM_HOST_EXPORT core::ObjectId importPointCloudFile(DocumentHost& host, const QString& filePath,
	const core::ImportOptionsDto& options, QString* outError = nullptr);

/// 接管已构造 Backend（层级/分件）；Follow 绑定见 BackendHierarchyFollow
/// @param linkOsgSceneParent false 时仅 Data 父子 + 逻辑父链，OSG 仍扁平（DXF 世界坐标分件）
CLOUDSIM_HOST_EXPORT bool registerAdoptedBackendObject(DocumentHost& host, const std::shared_ptr<BackendDataBase>& object,
	const QString& sourcePath, const QString& catalogTypeName, const QString& parentId, QString* outError = nullptr,
	bool linkOsgSceneParent = true);

/// 注册已构造 mesh 并加载场景（层级分件导入用）
CLOUDSIM_HOST_EXPORT bool registerAdoptedMeshAndLoadScene(DocumentHost& host, const std::shared_ptr<MeshBackendData>& mesh,
	const QString& sourcePath, const QString& catalogTypeName, const QString& parentId, bool resetViewToHome,
	QString* outError = nullptr, bool skipInnerModelCenterRebase = false, bool linkOsgSceneParent = true);

/// 注册已构造点云并加载场景
CLOUDSIM_HOST_EXPORT bool registerAdoptedPointCloudAndLoadScene(DocumentHost& host,
	const std::shared_ptr<PointCloudBackendData>& pointCloud, const QString& sourcePath, const QString& catalogTypeName,
	bool resetViewToHome, QString* outError = nullptr);

/// 工程 persistedId：摘链后以新 id 再注册（Data 无原地改 id）
CLOUDSIM_HOST_EXPORT QString rekeyBackendObject(DocumentHost& host, const QString& fromId, const QString& toId,
	QString* outError = nullptr);

} // namespace cloudsim::host
