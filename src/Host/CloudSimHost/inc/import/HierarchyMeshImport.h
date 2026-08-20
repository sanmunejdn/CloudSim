#ifndef CLOUDSIMHOST_HIERARCHYMESHIMPORT_H
#define CLOUDSIMHOST_HIERARCHYMESHIMPORT_H

/// @file HierarchyMeshImport.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 分件网格导入

#include "cloudsim_host_global.h"

#include <QString>
#include <QStringList>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <ShapeHandle.h>

class BackendDataBase;
class BrepBackendData;
class MeshBackendData;
struct BrepHierarchyPart;
struct MeshHierarchyPart;

namespace cloudsim::host
{
class DocumentHost;

struct HierarchyMeshImportResult
{
	bool ok = false;
	std::shared_ptr<BackendDataBase> importParent; ///< 空壳父，聚焦与树选中用
	std::shared_ptr<MeshBackendData> lastRegisteredMesh;
	std::shared_ptr<BrepBackendData> lastRegisteredBrep;
	int registeredPartCount = 0;
	QStringList partBackendIds;
};

using HierarchyFollowBindingFn =
	std::function<void(const std::string& childBackendId, const std::string& parentBackendId)>;

/// 分件网格导入
/// @param importParentDisplayName 树顶显示名，空则用 defaultBaseName
CLOUDSIM_HOST_EXPORT bool
importMeshHierarchyParts(DocumentHost& host, const QString& sourceFilePath, const QString& catalogTypeName,
						 const std::vector<MeshHierarchyPart>& parts, const QString& defaultBaseName,
						 const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out,
						 QString* outError = nullptr, const QString& importParentDisplayName = QString());

CLOUDSIM_HOST_EXPORT bool
importBrepHierarchyParts(DocumentHost& host, const QString& sourceFilePath, const QString& catalogTypeName,
						 const std::vector<BrepHierarchyPart>& parts, const QString& defaultBaseName,
						 const HierarchyFollowBindingFn& onParentFollow, HierarchyMeshImportResult& out,
						 QString* outError = nullptr, const QString& importParentDisplayName = QString(),
						 const geoalgo::ShapeHandle& assemblyShape = {});

/// 装配 Phase1 一次再切片到各子 Shape 缓存；Worker / 同步导入共用
CLOUDSIM_HOST_EXPORT bool warmBrepHierarchyPartsDisplayFromAssembly(
	const geoalgo::ShapeHandle& assembly, const std::vector<BrepHierarchyPart>& parts,
	const std::function<void(double progress01, const QString& status)>& progress, QString* outError = nullptr);

/// 3D 点到的面所属 Solid 抽成独立 B-rep；仅一块时返回原对象
CLOUDSIM_HOST_EXPORT bool extractBrepSolidByFace(DocumentHost& host, const std::string& brepId, int faceIndex,
												 QString* outPartId, QString* outError = nullptr);

/// 扩展网格导入
CLOUDSIM_HOST_EXPORT bool importMeshFileExtended(DocumentHost& host, const QString& filePath,
												 const QString& catalogTypeName, bool quietUi, int meshImportQuality,
												 const HierarchyFollowBindingFn& onParentFollow,
												 HierarchyMeshImportResult& out, QString* outError = nullptr);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_HIERARCHYMESHIMPORT_H
