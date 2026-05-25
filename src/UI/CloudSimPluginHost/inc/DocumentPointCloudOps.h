#pragma once

#include "PluginPointCloudTypes.h"
#include "PointCloudBackendOps.h"

#include <Eigen/Geometry>

#include <memory>
#include <string>

class DocumentPage;
class MainWindow;
class MeshBackendData;
class PointCloudBackendData;

namespace document_point_cloud_ops
{

std::shared_ptr<PointCloudBackendData> resolvePointCloud(
	DocumentPage* page,
	const std::string& backendIdUtf8,
	std::string* outError = nullptr);

void commitPointCloudVisual(DocumentPage* page, const PointCloudBackendData& data);

std::string registerReconstructedMesh(
	DocumentPage* page,
	MainWindow* mainWindow,
	const std::shared_ptr<MeshBackendData>& meshPtr,
	const PluginMeshCreateOptions& options,
	std::string* outError = nullptr);

PluginPointCloudInfo toPluginInfo(const PointCloudBackendData& data);
PluginPointCloudMeasure toPluginMeasure(const point_cloud_backend_ops::PointCloudMeasureResult& m);
PluginMat4 toPluginMat4(const Eigen::Isometry3d& t);
Eigen::Isometry3d toEigenIsometry(const PluginMat4& m);
Eigen::AlignedBox3d toEigenBox(const PluginAxisAlignedBox& box);

bool queryPointCloudInfo(DocumentPage* page, const std::string& backendIdUtf8, PluginPointCloudInfo& out);
bool measurePointCloud(DocumentPage* page, const std::string& backendIdUtf8, PluginPointCloudMeasure& out);
bool exportMeshToPly(
	DocumentPage* page,
	const std::string& backendIdUtf8,
	const std::string& pathUtf8,
	std::string* outError = nullptr);

} // namespace document_point_cloud_ops
