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

#include <osg/Matrixd>
#include <osg/Vec3d>

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

namespace
{

std::string templateTransformBackendId(OsgWidget* osg, const std::string& templateBackendIdUtf8)
{
	if (!osg || templateBackendIdUtf8.empty())
	{
		return templateBackendIdUtf8;
	}
	return osg->resolvePickScopeBackendId(templateBackendIdUtf8);
}

std::string scanTransformBackendId(OsgWidget* osg, const std::string& scanBackendIdUtf8)
{
	if (!osg || scanBackendIdUtf8.empty())
	{
		return scanBackendIdUtf8;
	}
	return osg->resolvePickScopeBackendId(scanBackendIdUtf8);
}

bool backendStoredPointToWorldMm(
	OsgWidget* osg,
	const std::string& backendIdUtf8,
	const double x,
	const double y,
	const double z,
	osg::Vec3d& outWorld)
{
	const std::string xformId = scanTransformBackendId(osg, backendIdUtf8);
	osg::Matrixd worldMat;
	if (!osg->getBackendRootWorldMatrix(xformId, worldMat))
	{
		return false;
	}
	double cx = 0.0;
	double cy = 0.0;
	double cz = 0.0;
	if (!osg->backendSkipsInnerModelCenterRebase(xformId))
	{
		(void)osg->tryGetBackendModelCenterMm(xformId, cx, cy, cz);
	}
	const osg::Vec3d pInner(x - cx, y - cy, z - cz);
	outWorld = pInner * worldMat;
	return true;
}

bool worldPointToTemplateModelMm(
	OsgWidget* osg,
	const std::string& templateBackendIdUtf8,
	const osg::Vec3d& worldMm,
	double& outX,
	double& outY,
	double& outZ)
{
	const std::string xformId = templateTransformBackendId(osg, templateBackendIdUtf8);
	osg::Matrixd worldMat;
	if (!osg->getBackendRootWorldMatrix(xformId, worldMat))
	{
		return false;
	}
	osg::Matrixd invMat;
	if (!invMat.invert(worldMat))
	{
		return false;
	}
	const osg::Vec3d pOuter = worldMm * invMat;
	double cx = 0.0;
	double cy = 0.0;
	double cz = 0.0;
	if (!osg->backendSkipsInnerModelCenterRebase(xformId))
	{
		(void)osg->tryGetBackendModelCenterMm(xformId, cx, cy, cz);
	}
	outX = pOuter.x() + cx;
	outY = pOuter.y() + cy;
	outZ = pOuter.z() + cz;
	return true;
}

bool templateModelPointToWorldMm(
	OsgWidget* osg,
	const std::string& templateBackendIdUtf8,
	const double modelX,
	const double modelY,
	const double modelZ,
	osg::Vec3d& outWorld)
{
	const std::string xformId = templateTransformBackendId(osg, templateBackendIdUtf8);
	osg::Matrixd worldMat;
	if (!osg->getBackendRootWorldMatrix(xformId, worldMat))
	{
		return false;
	}
	double cx = 0.0;
	double cy = 0.0;
	double cz = 0.0;
	if (!osg->backendSkipsInnerModelCenterRebase(xformId))
	{
		(void)osg->tryGetBackendModelCenterMm(xformId, cx, cy, cz);
	}
	const osg::Vec3d pOuter(modelX - cx, modelY - cy, modelZ - cz);
	outWorld = pOuter * worldMat;
	return true;
}

bool worldPointToScanStoredMm(
	OsgWidget* osg,
	const std::string& scanBackendIdUtf8,
	const osg::Vec3d& worldMm,
	double& outX,
	double& outY,
	double& outZ)
{
	const std::string xformId = scanTransformBackendId(osg, scanBackendIdUtf8);
	osg::Matrixd worldMat;
	if (!osg->getBackendRootWorldMatrix(xformId, worldMat))
	{
		return false;
	}
	osg::Matrixd invMat;
	if (!invMat.invert(worldMat))
	{
		return false;
	}
	const osg::Vec3d pInner = worldMm * invMat;
	double cx = 0.0;
	double cy = 0.0;
	double cz = 0.0;
	if (!osg->backendSkipsInnerModelCenterRebase(xformId))
	{
		(void)osg->tryGetBackendModelCenterMm(xformId, cx, cy, cz);
	}
	outX = pInner.x() + cx;
	outY = pInner.y() + cy;
	outZ = pInner.z() + cz;
	return true;
}

} // namespace

bool transformScanPointsToTemplateModelFrame(
	cloudsim::host::DocumentHost* page,
	const std::string& scanBackendIdUtf8,
	const std::string& templateBackendIdUtf8,
	std::vector<float>& inOutScanXyz,
	std::string* outError)
{
	if (inOutScanXyz.size() < 9U || (inOutScanXyz.size() % 3U) != 0U)
	{
		if (outError)
		{
			*outError = "invalid scan xyz buffer";
		}
		return false;
	}
	OsgWidget* osg = widgetOsgFromPage(page);
	if (!osg)
	{
		if (outError)
		{
			*outError = "no active 3D view";
		}
		return false;
	}
	for (std::size_t i = 0; i + 2U < inOutScanXyz.size(); i += 3U)
	{
		osg::Vec3d world;
		if (!backendStoredPointToWorldMm(
				osg,
				scanBackendIdUtf8,
				static_cast<double>(inOutScanXyz[i]),
				static_cast<double>(inOutScanXyz[i + 1U]),
				static_cast<double>(inOutScanXyz[i + 2U]),
				world))
		{
			if (outError)
			{
				*outError = "scan backend world transform unavailable";
			}
			return false;
		}
		double mx = 0.0;
		double my = 0.0;
		double mz = 0.0;
		if (!worldPointToTemplateModelMm(osg, templateBackendIdUtf8, world, mx, my, mz))
		{
			if (outError)
			{
				*outError = "template backend model transform unavailable";
			}
			return false;
		}
		inOutScanXyz[i] = static_cast<float>(mx);
		inOutScanXyz[i + 1U] = static_cast<float>(my);
		inOutScanXyz[i + 2U] = static_cast<float>(mz);
	}
	return true;
}

bool applyScanIcpAlignmentToStoredPoints(
	cloudsim::host::DocumentHost* page,
	const std::string& scanBackendIdUtf8,
	const std::string& templateBackendIdUtf8,
	const Eigen::Isometry3d& scanToTemplateInModelFrame,
	PointCloudBackendData& inOutScan,
	std::string* outError)
{
	OsgWidget* osg = widgetOsgFromPage(page);
	if (!osg)
	{
		if (outError)
		{
			*outError = "no active 3D view";
		}
		return false;
	}

	std::vector<float> xyz = inOutScan.pointPositionsXyz();
	if (xyz.size() < 9U || (xyz.size() % 3U) != 0U)
	{
		if (outError)
		{
			*outError = "invalid scan xyz buffer";
		}
		return false;
	}

	for (std::size_t i = 0; i + 2U < xyz.size(); i += 3U)
	{
		osg::Vec3d world;
		if (!backendStoredPointToWorldMm(
				osg,
				scanBackendIdUtf8,
				static_cast<double>(xyz[i]),
				static_cast<double>(xyz[i + 1U]),
				static_cast<double>(xyz[i + 2U]),
				world))
		{
			if (outError)
			{
				*outError = "scan backend world transform unavailable";
			}
			return false;
		}

		double mx = 0.0;
		double my = 0.0;
		double mz = 0.0;
		if (!worldPointToTemplateModelMm(osg, templateBackendIdUtf8, world, mx, my, mz))
		{
			if (outError)
			{
				*outError = "template backend model transform unavailable";
			}
			return false;
		}

		const Eigen::Vector3d aligned = scanToTemplateInModelFrame * Eigen::Vector3d(mx, my, mz);
		osg::Vec3d worldAligned;
		if (!templateModelPointToWorldMm(
				osg, templateBackendIdUtf8, aligned.x(), aligned.y(), aligned.z(), worldAligned))
		{
			if (outError)
			{
				*outError = "template world transform unavailable";
			}
			return false;
		}

		double sx = 0.0;
		double sy = 0.0;
		double sz = 0.0;
		if (!worldPointToScanStoredMm(osg, scanBackendIdUtf8, worldAligned, sx, sy, sz))
		{
			if (outError)
			{
				*outError = "scan stored transform unavailable";
			}
			return false;
		}

		xyz[i] = static_cast<float>(sx);
		xyz[i + 1U] = static_cast<float>(sy);
		xyz[i + 2U] = static_cast<float>(sz);
	}

	std::vector<float> rgba = inOutScan.pointVertexRgba();
	std::vector<float> normals = inOutScan.pointNormalsNxNyNz();
	inOutScan.setPointBuffers(std::move(xyz), std::move(rgba), std::move(normals));
	return true;
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

bool queryMeshInfo(cloudsim::host::DocumentHost* page, const std::string& backendIdUtf8, PluginMeshInfo& out)
{
	const auto mesh = resolveMesh(page, backendIdUtf8);
	if (!mesh)
	{
		return false;
	}
	const auto soup = mesh->triangleSoup();
	out.faceCount = soup.size() / 9;
	out.vertexCount = out.faceCount * 3; // triangle soup 无去重，近似值
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
