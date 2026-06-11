#pragma once

#include "CoreTypes.h"
#include "cloudsim_host_global.h"

#include <memory>

class BackendDataBase;
class BrepBackendData;
class MeshBackendData;
class PointCloudBackendData;

namespace cloudsim::host {

class DocumentHost;

/// 单网格文件导入
CLOUDSIM_HOST_EXPORT core::ObjectId importMeshFile(DocumentHost& host, const QString& filePath,
	const core::ImportOptionsDto& options, QString* outError = nullptr);

/// 点云文件导入
CLOUDSIM_HOST_EXPORT core::ObjectId importPointCloudFile(DocumentHost& host, const QString& filePath,
	const core::ImportOptionsDto& options, QString* outError = nullptr);

/// 接管已构造 Backend
/// @param linkOsgSceneParent false 时 OSG 扁平（DXF 世界坐标分件）
CLOUDSIM_HOST_EXPORT bool registerAdoptedBackendObject(DocumentHost& host, const std::shared_ptr<BackendDataBase>& object,
	const QString& sourcePath, const QString& catalogTypeName, const QString& parentId, QString* outError = nullptr,
	bool linkOsgSceneParent = true);

/// 注册 mesh 并加载
CLOUDSIM_HOST_EXPORT bool registerAdoptedMeshAndLoadScene(DocumentHost& host, const std::shared_ptr<MeshBackendData>& mesh,
	const QString& sourcePath, const QString& catalogTypeName, const QString& parentId, bool resetViewToHome,
	QString* outError = nullptr, bool linkOsgSceneParent = true);

CLOUDSIM_HOST_EXPORT bool registerAdoptedBrepAndLoadScene(DocumentHost& host, const std::shared_ptr<BrepBackendData>& brep,
	const QString& sourcePath, const QString& catalogTypeName, const QString& parentId, const bool resetViewToHome,
	QString* outError = nullptr, const bool linkOsgSceneParent = true, const bool loadScene = true);

/// 注册点云并加载
CLOUDSIM_HOST_EXPORT bool registerAdoptedPointCloudAndLoadScene(DocumentHost& host,
	const std::shared_ptr<PointCloudBackendData>& pointCloud, const QString& sourcePath, const QString& catalogTypeName,
	bool resetViewToHome, QString* outError = nullptr);

/// 工程 id 重注册
CLOUDSIM_HOST_EXPORT QString rekeyBackendObject(DocumentHost& host, const QString& fromId, const QString& toId,
	QString* outError = nullptr);

} // namespace cloudsim::host
