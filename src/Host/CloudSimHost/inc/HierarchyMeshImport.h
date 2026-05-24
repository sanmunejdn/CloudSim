#pragma once

#include "cloudsim_host_global.h"

#include "MeshBackendData.h"

#include <QString>

#include <functional>
#include <memory>
#include <string>
#include <vector>

class BackendDataBase;
class MeshBackendData;

namespace cloudsim::host {

class DocumentHost;

struct HierarchyMeshImportResult {
	bool ok = false;
	std::shared_ptr<BackendDataBase> importParent; ///< 空壳父，聚焦与树选中用
	std::shared_ptr<MeshBackendData> lastRegisteredMesh;
	int registeredPartCount = 0;
};

using HierarchyFollowBindingFn = std::function<void(const std::string& childBackendId, const std::string& parentBackendId)>;

/// 分件网格：空壳父 + registerAdoptedMeshAndLoadScene；世界坐标分件忽略 onParentFollow
/// @param importParentDisplayName 树顶显示名，空则用 defaultBaseName
CLOUDSIM_HOST_EXPORT bool importMeshHierarchyParts(DocumentHost& host, const QString& sourceFilePath,
	const QString& catalogTypeName, const std::vector<MeshHierarchyPart>& parts, const QString& defaultBaseName,
	const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out, QString* outError = nullptr,
	const QString& importParentDisplayName = QString());

/// DXF/STEP/OSG 层级或 CGAL/OSG 单件；true=已处理，false=调用方另寻路径
CLOUDSIM_HOST_EXPORT bool importMeshFileExtended(DocumentHost& host, const QString& filePath, const QString& catalogTypeName,
	bool quietUi, const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out,
	QString* outError = nullptr);

} // namespace cloudsim::host
