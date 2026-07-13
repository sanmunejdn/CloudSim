#pragma once

#include "PluginPointCloudTypes.h"
#include "PointCloudBackendOps.h"
#include "cloudsim_host_global.h"

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>

namespace geoalgo
{
class ShapeHandle;
}

namespace cloudsim::host {
class DocumentHost;
}

class IPluginMainWindowHost;
class BrepBackendData;
class MeshBackendData;
class PointCloudBackendData;

namespace document_point_cloud_ops
{

std::shared_ptr<PointCloudBackendData> resolvePointCloud(
	cloudsim::host::DocumentHost* page,
	const std::string& backendIdUtf8,
	std::string* outError = nullptr);

std::shared_ptr<MeshBackendData> resolveMesh(
	cloudsim::host::DocumentHost* page,
	const std::string& backendIdUtf8,
	std::string* outError = nullptr);

void commitPointCloudVisual(cloudsim::host::DocumentHost* page, const PointCloudBackendData& data);

std::string registerReconstructedMesh(
	cloudsim::host::DocumentHost* page,
	IPluginMainWindowHost* mainWindowHost,
	const std::shared_ptr<MeshBackendData>& meshPtr,
	const PluginMeshCreateOptions& options,
	std::string* outError = nullptr);

PluginPointCloudInfo toPluginInfo(const PointCloudBackendData& data);
PluginPointCloudMeasure toPluginMeasure(const point_cloud_backend_ops::PointCloudMeasureResult& m);
PluginMat4 toPluginMat4(const Eigen::Isometry3d& t);
Eigen::Isometry3d toEigenIsometry(const PluginMat4& m);
Eigen::AlignedBox3d toEigenBox(const PluginAxisAlignedBox& box);

bool queryPointCloudInfo(cloudsim::host::DocumentHost* page, const std::string& backendIdUtf8, PluginPointCloudInfo& out);
bool queryMeshInfo(cloudsim::host::DocumentHost* page, const std::string& backendIdUtf8, PluginMeshInfo& out);
bool measurePointCloud(cloudsim::host::DocumentHost* page, const std::string& backendIdUtf8, PluginPointCloudMeasure& out);
bool exportMeshToPly(
	cloudsim::host::DocumentHost* page,
	const std::string& backendIdUtf8,
	const std::string& pathUtf8,
	std::string* outError = nullptr);

CLOUDSIM_HOST_EXPORT bool exportPointCloudToPly(
	cloudsim::host::DocumentHost* page,
	const std::string& backendIdUtf8,
	const std::string& pathUtf8,
	std::string* outError = nullptr);

CLOUDSIM_HOST_EXPORT bool exportBrepToStep(
	cloudsim::host::DocumentHost* page,
	const std::string& backendIdUtf8,
	const std::string& pathUtf8,
	std::string* outError = nullptr);

/// 512 点采样：世界系 overlap（scan/template worldMatrix）
void logRegistrationOverlapDiagnostic(
	cloudsim::host::DocumentHost* page,
	const std::string& scanBackendIdUtf8,
	const std::string& templateBackendIdUtf8,
	const std::vector<float>& scanStoredXyz,
	const std::vector<float>& templateModelXyz,
	double gateMm);

/// 扫描/模板质心世界距离（诊断 3D 视图是否已手动对齐）
void logRegistrationCentroidDiagnostic(
	cloudsim::host::DocumentHost* page,
	const std::string& scanBackendIdUtf8,
	const std::string& templateBackendIdUtf8,
	const std::vector<float>& scanStoredXyz,
	double templateModelCenterX,
	double templateModelCenterY,
	double templateModelCenterZ);

bool queryTemplateModelToWorldIsometry(
	cloudsim::host::DocumentHost* page,
	const std::string& templateBackendIdUtf8,
	Eigen::Isometry3d& outModelToWorld,
	std::string* outError = nullptr);

bool queryScanStoredToWorldIsometry(
	cloudsim::host::DocumentHost* page,
	const std::string& scanBackendIdUtf8,
	Eigen::Isometry3d& outStoredToWorld,
	std::string* outError = nullptr);

/// 反向配准预览：仅更新模板 worldMatrix + OSG 同步（几何不变）
bool applyTemplateRegistrationToVisual(
	cloudsim::host::DocumentHost* page,
	const std::string& templateBackendIdUtf8,
	const Eigen::Isometry3d& icpDeltaWorld,
	std::string* outError = nullptr);

/// 曲面重构新 B-rep：内层质心与源网格对齐并继承 OSG 世界位姿（registerAdoptedBrepAndLoadScene 之后调用）
bool inheritBrepVisualPoseFromSourceMesh(
	cloudsim::host::DocumentHost* page,
	const std::string& sourceMeshBackendIdUtf8,
	const std::string& newBrepBackendIdUtf8,
	BrepBackendData& newBrep,
	std::string* outError = nullptr);

/// 面重构新工件：复制模板 worldMatrix 并同步 OSG（注册 loadScene 之后调用）
bool alignFaceUpdatedBrepWithTemplateVisual(
	cloudsim::host::DocumentHost* page,
	const std::string& templateBackendIdUtf8,
	const std::string& updatedBrepBackendIdUtf8,
	const BrepBackendData& templateBrep,
	BrepBackendData& updatedBrep,
	std::string* outError = nullptr);

/// 从 STEP 恢复模板原始几何并刷新显示（保留当前 OSG 位姿）
bool restoreTemplateShapeFromStep(
	cloudsim::host::DocumentHost* page,
	const std::string& templateBackendIdUtf8,
	const std::string& templateStepPathUtf8,
	std::string* outError = nullptr);

/// 校验扫描 backend（点云或 mesh）；输出几何系 xyz（mesh 为 soup 采样）
bool prepareScanForTemplateRegistration(
	cloudsim::host::DocumentHost* page,
	const std::string& scanBackendIdUtf8,
	std::vector<float>& outStoredXyz,
	std::vector<float>& outStoredNormals,
	std::size_t& outPointCount,
	bool& outIsMeshScan,
	std::string* outError = nullptr);

/// 校验点云 backend 缓冲；异常时从源 PLY 重载，输出 stored 系 xyz
bool prepareScanPointCloudForRegistration(
	cloudsim::host::DocumentHost* page,
	const std::string& scanBackendIdUtf8,
	std::vector<float>& outStoredXyz,
	std::size_t& outPointCount,
	std::string* outError = nullptr);

/// 点云 backend 存储坐标 → 世界 mm（与 OSG 外层 PAT 一致）
bool buildPointCloudModelToWorld(
	const PointCloudBackendData& data,
	PluginMat4& outModelToWorld);

} // namespace document_point_cloud_ops
