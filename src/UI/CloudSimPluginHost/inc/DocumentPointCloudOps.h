#pragma once

#include "PluginPointCloudTypes.h"
#include "PointCloudBackendOps.h"

#include <Eigen/Geometry>

#include <memory>
#include <string>

namespace cloudsim::host {
class DocumentHost;
}

class IPluginMainWindowHost;
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

/// 将扫描点云从 backend 存储坐标变换到 CAD 模板 STEP 模型坐标（与 B-rep 拾取同规则）
bool transformScanPointsToTemplateModelFrame(
	cloudsim::host::DocumentHost* page,
	const std::string& scanBackendIdUtf8,
	const std::string& templateBackendIdUtf8,
	std::vector<float>& inOutScanXyz,
	std::string* outError = nullptr);

/// 将 ICP 结果（CAD 模型坐标系）烘焙回点云 backend 存储坐标并刷新显示
bool applyScanIcpAlignmentToStoredPoints(
	cloudsim::host::DocumentHost* page,
	const std::string& scanBackendIdUtf8,
	const std::string& templateBackendIdUtf8,
	const Eigen::Isometry3d& scanToTemplateInModelFrame,
	PointCloudBackendData& inOutScan,
	std::string* outError = nullptr);

/// 点云 backend 存储坐标 → 世界 mm（与 OSG 外层 PAT 一致）
bool buildPointCloudModelToWorld(
	const PointCloudBackendData& data,
	PluginMat4& outModelToWorld);

} // namespace document_point_cloud_ops
