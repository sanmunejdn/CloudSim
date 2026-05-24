#pragma once

#include "CoreTypes.h"
#include "HierarchyMeshImport.h"
#include "cloudsim_host_global.h"

#include <QString>
#include <memory>

class MeshBackendData;
class PointCloudBackendData;

namespace cloudsim::host {

class DocumentHost;

enum class ImportFileKind { Mesh, PointCloud };

struct ImportFileResult {
	QString rootBackendId; ///< 层级导入多为 importParent id
	bool ok = false;
	bool hierarchyImport = false;
	bool skipFollowOnImport = false; ///< DXF/STEP 分件为世界坐标，导入期勿 Follow
	HierarchyMeshImportResult hierarchyDetail;
};

/// 菜单/插件/IDataService 统一导入路由；大 ply 异步仍由 Widget Job 先行
CLOUDSIM_HOST_EXPORT ImportFileResult importFileIntoDocument(DocumentHost& host, const QString& filePath,
	ImportFileKind kind, const cloudsim::core::ImportOptionsDto& options, QString* outError = nullptr);

struct AdoptMeshOptions {
	QString sourcePath;
	QString catalogTypeName = QStringLiteral("Model");
	QString parentId;
	bool resetViewToHome = true;
	bool skipInnerModelCenterRebase = false;
	bool linkOsgSceneParent = true;
};

struct AdoptPointCloudOptions {
	QString sourcePath;
	QString catalogTypeName = QStringLiteral("PointCloud");
	bool resetViewToHome = true;
};

struct AdoptRegistrationResult {
	QString backendId;
	bool ok = false;
};

/// 已构造 mesh/点云注册（AI、插件、ply Job 完成回调）
CLOUDSIM_HOST_EXPORT AdoptRegistrationResult registerAdoptedMesh(DocumentHost& host,
	const std::shared_ptr<MeshBackendData>& mesh, const AdoptMeshOptions& options, QString* outError = nullptr);
CLOUDSIM_HOST_EXPORT AdoptRegistrationResult registerAdoptedPointCloud(DocumentHost& host,
	const std::shared_ptr<PointCloudBackendData>& pointCloud, const AdoptPointCloudOptions& options,
	QString* outError = nullptr);

} // namespace cloudsim::host
