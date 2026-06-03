#include "DocumentPointCloudOps.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendSceneDocumentFacade.h"
#include "DocumentImportFacade.h"
#include "DocumentHost.h"
#include "IPluginMainWindowHost.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PointCloudBackendData.h"
#include "WidgetDocumentAccess.h"

namespace document_point_cloud_ops
{

std::shared_ptr<PointCloudBackendData> resolvePointCloud(
	cloudsim::host::DocumentHost* page,
	const std::string& backendIdUtf8,
	std::string* outError)
{
	if (!page)
	{
		if (outError)
		{
			*outError = "invalid document page";
		}
		return nullptr;
	}
	const auto obj = page->backend().getData(backendIdUtf8);
	if (!obj)
	{
		if (outError)
		{
			*outError = "backend object not found";
		}
		return nullptr;
	}
	auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(obj);
	if (!pc)
	{
		if (outError)
		{
			*outError = "not a point cloud backend";
		}
		return nullptr;
	}
	return pc;
}

std::shared_ptr<MeshBackendData> resolveMesh(
	cloudsim::host::DocumentHost* page,
	const std::string& backendIdUtf8,
	std::string* outError)
{
	if (!page)
	{
		if (outError)
		{
			*outError = "invalid document page";
		}
		return nullptr;
	}
	const auto obj = page->backend().getData(backendIdUtf8);
	if (!obj)
	{
		if (outError)
		{
			*outError = "backend object not found";
		}
		return nullptr;
	}
	auto mesh = std::dynamic_pointer_cast<MeshBackendData>(obj);
	if (!mesh)
	{
		if (outError)
		{
			*outError = "not a mesh backend";
		}
		return nullptr;
	}
	if (mesh->triangleSoup().empty())
	{
		if (outError)
		{
			*outError = "mesh has no geometry";
		}
		return nullptr;
	}
	return mesh;
}

void commitPointCloudVisual(cloudsim::host::DocumentHost* page, const PointCloudBackendData& data)
{
	if (!page)
	{
		return;
	}
	OsgWidget* osg = widgetOsgFromPage(page);
	if (osg && !data.pointPositionsXyz().empty())
	{
		QString geomErr;
		(void)osg->loadPointCloudFromBackendData(data, &geomErr, false);
	}
}

std::string registerReconstructedMesh(
	cloudsim::host::DocumentHost* page,
	IPluginMainWindowHost* mainWindowHost,
	const std::shared_ptr<MeshBackendData>& meshPtr,
	const PluginMeshCreateOptions& options,
	std::string* outError)
{
	if (!page || !meshPtr)
	{
		if (outError)
		{
			*outError = "invalid document page or mesh";
		}
		return std::string();
	}
	const QString displayName =
		options.displayName.isEmpty() ? QStringLiteral("ReconstructedMesh") : options.displayName;
	meshPtr->setName(displayName.toStdString());

	BackendVec3 pos;
	pos.x = options.poseMm.x;
	pos.y = options.poseMm.y;
	pos.z = options.poseMm.z;
	meshPtr->setPose(pos);

	BackendVec3 rot;
	rot.x = options.rotationDeg.x;
	rot.y = options.rotationDeg.y;
	rot.z = options.rotationDeg.z;
	meshPtr->setRotation(rot);

	cloudsim::host::AdoptMeshOptions adoptOpt;
	adoptOpt.sourcePath =
		options.sourcePath.isEmpty() ? QStringLiteral("plugin://pointcloud/reconstruct") : options.sourcePath;
	adoptOpt.catalogTypeName = QStringLiteral("Model");
	adoptOpt.resetViewToHome = options.resetViewToHome;
	QString regErr;
	const cloudsim::host::AdoptRegistrationResult adopted =
		cloudsim::host::registerAdoptedMesh(*page, meshPtr, adoptOpt, &regErr);
	if (!adopted.ok)
	{
		if (outError)
		{
			*outError = regErr.toStdString();
		}
		return std::string();
	}
	if (mainWindowHost && options.selectInTree)
	{
		mainWindowHost->focusBackendInTreeAfterImport(adopted.backendId);
	}
	return adopted.backendId.toStdString();
}

PluginPointCloudInfo toPluginInfo(const PointCloudBackendData& data)
{
	PluginPointCloudInfo info;
	info.pointCount = data.geometryElementCount();
	info.hasPerVertexColors = data.hasPerVertexColors();
	info.hasPointNormals = data.hasPointNormals();
	const BackendBoundingBox b = data.geometryBounds();
	if (b.valid)
	{
		info.bounds.valid = true;
		info.bounds.minMm.x = b.min.x;
		info.bounds.minMm.y = b.min.y;
		info.bounds.minMm.z = b.min.z;
		info.bounds.maxMm.x = b.max.x;
		info.bounds.maxMm.y = b.max.y;
		info.bounds.maxMm.z = b.max.z;
	}
	return info;
}

PluginPointCloudMeasure toPluginMeasure(const point_cloud_backend_ops::PointCloudMeasureResult& m)
{
	PluginPointCloudMeasure out;
	out.centroidMm.x = m.centroidMm.x();
	out.centroidMm.y = m.centroidMm.y();
	out.centroidMm.z = m.centroidMm.z();
	out.averageSpacingMm = m.averageSpacingMm;
	if (m.boundingBoxMm.isEmpty())
	{
		return out;
	}
	out.bounds.valid = true;
	const Eigen::Vector3d minP = m.boundingBoxMm.min();
	const Eigen::Vector3d maxP = m.boundingBoxMm.max();
	out.bounds.minMm.x = minP.x();
	out.bounds.minMm.y = minP.y();
	out.bounds.minMm.z = minP.z();
	out.bounds.maxMm.x = maxP.x();
	out.bounds.maxMm.y = maxP.y();
	out.bounds.maxMm.z = maxP.z();
	return out;
}

PluginMat4 toPluginMat4(const Eigen::Isometry3d& t)
{
	PluginMat4 out;
	const Eigen::Matrix4d m = t.matrix();
	for (int col = 0; col < 4; ++col)
	{
		for (int row = 0; row < 4; ++row)
		{
			out.v[col * 4 + row] = m(row, col);
		}
	}
	return out;
}

Eigen::Isometry3d toEigenIsometry(const PluginMat4& m)
{
	Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
	for (int col = 0; col < 4; ++col)
	{
		for (int row = 0; row < 4; ++row)
		{
			mat(row, col) = m.v[col * 4 + row];
		}
	}
	Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
	iso.matrix() = mat;
	return iso;
}

Eigen::AlignedBox3d toEigenBox(const PluginAxisAlignedBox& box)
{
	Eigen::AlignedBox3d b;
	if (!box.valid)
	{
		return b;
	}
	b.extend(Eigen::Vector3d(box.minMm.x, box.minMm.y, box.minMm.z));
	b.extend(Eigen::Vector3d(box.maxMm.x, box.maxMm.y, box.maxMm.z));
	return b;
}

bool queryPointCloudInfo(cloudsim::host::DocumentHost* page, const std::string& backendIdUtf8, PluginPointCloudInfo& out)
{
	const auto pc = resolvePointCloud(page, backendIdUtf8);
	if (!pc)
	{
		return false;
	}
	out = toPluginInfo(*pc);
	return true;
}

bool measurePointCloud(cloudsim::host::DocumentHost* page, const std::string& backendIdUtf8, PluginPointCloudMeasure& out)
{
	const auto pc = resolvePointCloud(page, backendIdUtf8);
	if (!pc)
	{
		return false;
	}
	point_cloud_backend_ops::PointCloudMeasureResult m;
	std::string err;
	if (!point_cloud_backend_ops::measurePointCloud(*pc, m, &err))
	{
		return false;
	}
	out = toPluginMeasure(m);
	return true;
}

bool exportMeshToPly(
	cloudsim::host::DocumentHost* page,
	const std::string& backendIdUtf8,
	const std::string& pathUtf8,
	std::string* outError)
{
	std::string resolveErr;
	const auto mesh = resolveMesh(page, backendIdUtf8, &resolveErr);
	if (!mesh)
	{
		if (outError)
		{
			*outError = resolveErr;
		}
		return false;
	}
	if (!mesh->writeTriangleMeshPly(pathUtf8, outError))
	{
		return false;
	}
	return true;
}

} // namespace document_point_cloud_ops
