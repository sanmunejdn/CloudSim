#include "DocumentPointCloudOps.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendGeometryMetrics.h"
#include "BackendSceneDocumentFacade.h"
#include "BackendPoseOsg.h"
#include "BackendVisualMath.h"
#include "DocumentImportFacade.h"
#include "DocumentHost.h"
#include "IPluginMainWindowHost.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PointCloudBackendData.h"
#include "BrepBackendData.h"
#include "WidgetDocumentAccess.h"
#include "RunLogger.h"
#include "Adapters.h"
#include "TemplateBrepUpdate.h"
#include <RigidTransform.h>

#include <BrepImportArtifacts.h>
#include <ShapeHandle.h>
#include <ShapeIo.h>

#include <QByteArray>
#include <QFile>
#include <QString>

#include <osg/Matrixd>
#include <osg/Quat>
#include <osg/Vec3d>

#include <functional>
#include <sstream>
#include <cmath>

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

std::string nativePathFromUtf8Path(const std::string& pathUtf8)
{
	const QByteArray enc = QFile::encodeName(QString::fromUtf8(pathUtf8.c_str()));
	return std::string(enc.constData(), static_cast<std::size_t>(enc.size()));
}

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
	const osg::Vec3d pInner(x, y, z);
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
	outX = pOuter.x();
	outY = pOuter.y();
	outZ = pOuter.z();
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
	const osg::Vec3d pOuter(modelX, modelY, modelZ);
	outWorld = pOuter * worldMat;
	return true;
}

bool isometryFromThreeCalibrationPoints(
	const std::function<bool(double, double, double, osg::Vec3d&)>& modelToWorld,
	const double originX,
	const double originY,
	const double originZ,
	Eigen::Isometry3d& outModelToWorld,
	std::string* outError)
{
	osg::Vec3d wOrigin;
	osg::Vec3d wPlusX;
	osg::Vec3d wPlusY;
	if (!modelToWorld(originX, originY, originZ, wOrigin)
		|| !modelToWorld(originX + 100.0, originY, originZ, wPlusX)
		|| !modelToWorld(originX, originY + 100.0, originZ, wPlusY))
	{
		if (outError)
		{
			*outError = "calibration point world transform failed";
		}
		return false;
	}
	const Eigen::Vector3d o(wOrigin.x(), wOrigin.y(), wOrigin.z());
	Eigen::Vector3d ex(wPlusX.x() - wOrigin.x(), wPlusX.y() - wOrigin.y(), wPlusX.z() - wOrigin.z());
	Eigen::Vector3d ey(wPlusY.x() - wOrigin.x(), wPlusY.y() - wOrigin.y(), wPlusY.z() - wOrigin.z());
	if (ex.norm() < 1e-3 || ey.norm() < 1e-3)
	{
		if (outError)
		{
			*outError = "degenerate calibration for model-to-world isometry";
		}
		return false;
	}
	ex.normalize();
	ey -= ey.dot(ex) * ex;
	if (ey.norm() < 1e-3)
	{
		if (outError)
		{
			*outError = "collinear calibration for model-to-world isometry";
		}
		return false;
	}
	ey.normalize();
	Eigen::Matrix3d rot;
	rot.col(0) = ex;
	rot.col(1) = ey;
	rot.col(2) = ex.cross(ey);
	outModelToWorld.linear() = rot;
	outModelToWorld.translation() = o - rot * Eigen::Vector3d(originX, originY, originZ);
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
	outX = pInner.x();
	outY = pInner.y();
	outZ = pInner.z();
	return true;
}

} // namespace

bool captureRegistrationWorldFrameSnapshot(
	cloudsim::host::DocumentHost* page,
	const std::string& scanBackendIdUtf8,
	const std::string& templateBackendIdUtf8,
	geoalgo::RegistrationWorldFrameSnapshot& outSnapshot,
	std::string* outError)
{
	outSnapshot = {};
	OsgWidget* osg = widgetOsgFromPage(page);
	if (!osg)
	{
		if (outError)
		{
			*outError = "no active 3D view";
		}
		return false;
	}
	const auto copyMat = [](const osg::Matrixd& m, double out[16]) {
		for (int row = 0; row < 4; ++row)
		{
			for (int col = 0; col < 4; ++col)
			{
				out[row * 4 + col] = m(row, col);
			}
		}
	};
	const std::string scanXformId = scanTransformBackendId(osg, scanBackendIdUtf8);
	const std::string templateXformId = templateTransformBackendId(osg, templateBackendIdUtf8);
	osg::Matrixd scanMat;
	osg::Matrixd templateMat;
	if (!osg->getBackendRootWorldMatrix(scanXformId, scanMat)
		|| !osg->getBackendRootWorldMatrix(templateXformId, templateMat))
	{
		if (outError)
		{
			*outError = "backend world matrix unavailable for registration snapshot";
		}
		return false;
	}
	copyMat(scanMat, outSnapshot.scanRootWorldMat);
	copyMat(templateMat, outSnapshot.templateRootWorldMat);
	outSnapshot.valid = true;
	return true;
}

void logRegistrationOverlapDiagnostic(
	cloudsim::host::DocumentHost* page,
	const std::string& scanBackendIdUtf8,
	const std::string& templateBackendIdUtf8,
	const std::vector<float>& scanStoredXyz,
	const std::vector<float>& templateModelXyz,
	double gateMm)
{
	if (scanStoredXyz.size() < 9U || templateModelXyz.size() < 9U || gateMm <= 0.0)
	{
		return;
	}
	OsgWidget* osg = widgetOsgFromPage(page);
	if (!osg)
	{
		return;
	}
	const std::size_t nScan = scanStoredXyz.size() / 3U;
	const std::size_t nTpl = templateModelXyz.size() / 3U;
	const std::size_t scanStride = std::max<std::size_t>(1U, nScan / 512U);
	const std::size_t tplStride = std::max<std::size_t>(1U, nTpl / 512U);
	const double gateSq = gateMm * gateMm;
	std::size_t modelHits = 0U;
	std::size_t worldHits = 0U;
	for (std::size_t i = 0U; i < nScan; i += scanStride)
	{
		const std::size_t b = i * 3U;
		std::vector<float> scanOne = {
			scanStoredXyz[b],
			scanStoredXyz[b + 1U],
			scanStoredXyz[b + 2U]};
		std::vector<float> scanModel = scanOne;
		if (!transformScanPointsToTemplateModelFrame(
				page, scanBackendIdUtf8, templateBackendIdUtf8, scanModel, nullptr))
		{
			break;
		}
		osg::Vec3d scanWorld;
		if (!backendStoredPointToWorldMm(
				osg, scanBackendIdUtf8, scanOne[0], scanOne[1], scanOne[2], scanWorld))
		{
			break;
		}
		const Eigen::Vector3d qModel(scanModel[0], scanModel[1], scanModel[2]);
		const Eigen::Vector3d qWorld(scanWorld.x(), scanWorld.y(), scanWorld.z());
		double bestModelSq = gateSq;
		double bestWorldSq = gateSq;
		for (std::size_t j = 0U; j < nTpl; j += tplStride)
		{
			const std::size_t tb = j * 3U;
			const Eigen::Vector3d tModel(
				templateModelXyz[tb], templateModelXyz[tb + 1U], templateModelXyz[tb + 2U]);
			osg::Vec3d tWorld;
			if (!templateModelPointToWorldMm(
					osg, templateBackendIdUtf8, tModel.x(), tModel.y(), tModel.z(), tWorld))
			{
				continue;
			}
			const Eigen::Vector3d tWorldVec(tWorld.x(), tWorld.y(), tWorld.z());
			const double dModelSq = (qModel - tModel).squaredNorm();
			const double dWorldSq = (qWorld - tWorldVec).squaredNorm();
			if (dModelSq < bestModelSq)
			{
				bestModelSq = dModelSq;
			}
			if (dWorldSq < bestWorldSq)
			{
				bestWorldSq = dWorldSq;
			}
		}
		if (bestModelSq < gateSq)
		{
			++modelHits;
		}
		if (bestWorldSq < gateSq)
		{
			++worldHits;
		}
	}
	RunLogger::info(
		std::string("[TemplateBrepUpdate] overlap diagnostic modelHits=") + std::to_string(modelHits)
		+ "/512 worldHits=" + std::to_string(worldHits) + "/512 gateMm=" + std::to_string(gateMm));
}

void logRegistrationCentroidDiagnostic(
	cloudsim::host::DocumentHost* page,
	const std::string& scanBackendIdUtf8,
	const std::string& templateBackendIdUtf8,
	const std::vector<float>& scanStoredXyz,
	const double templateModelCenterX,
	const double templateModelCenterY,
	const double templateModelCenterZ)
{
	if (scanStoredXyz.size() < 9U)
	{
		return;
	}
	OsgWidget* osg = widgetOsgFromPage(page);
	if (!osg)
	{
		return;
	}
	const std::string scanXformId = scanTransformBackendId(osg, scanBackendIdUtf8);
	const std::string templateXformId = templateTransformBackendId(osg, templateBackendIdUtf8);
	const std::size_t nScan = scanStoredXyz.size() / 3U;
	const std::size_t stride = std::max<std::size_t>(1U, nScan / 10000U);
	double sumWx = 0.0;
	double sumWy = 0.0;
	double sumWz = 0.0;
	std::size_t count = 0U;
	for (std::size_t i = 0U; i < nScan; i += stride)
	{
		const std::size_t b = i * 3U;
		osg::Vec3d world;
		if (!backendStoredPointToWorldMm(
				osg,
				scanBackendIdUtf8,
				static_cast<double>(scanStoredXyz[b]),
				static_cast<double>(scanStoredXyz[b + 1U]),
				static_cast<double>(scanStoredXyz[b + 2U]),
				world))
		{
			break;
		}
		sumWx += world.x();
		sumWy += world.y();
		sumWz += world.z();
		++count;
	}
	if (count == 0U)
	{
		return;
	}
	osg::Vec3d templateWorld;
	if (!templateModelPointToWorldMm(
			osg, templateBackendIdUtf8, templateModelCenterX, templateModelCenterY, templateModelCenterZ, templateWorld))
	{
		return;
	}
	const double scanCx = sumWx / static_cast<double>(count);
	const double scanCy = sumWy / static_cast<double>(count);
	const double scanCz = sumWz / static_cast<double>(count);
	const double distMm = std::hypot(
		scanCx - templateWorld.x(),
		std::hypot(scanCy - templateWorld.y(), scanCz - templateWorld.z()));
	RunLogger::info(
		std::string("[TemplateBrepUpdate] centroid diagnostic worldDistMm=") + std::to_string(distMm)
		+ " scanXform=" + scanXformId + " templateXform=" + templateXformId
		+ " templateSkipRebase=" + (osg->backendSkipsInnerModelCenterRebase(templateXformId) ? "true" : "false"));
}

bool queryTemplateModelToWorldIsometry(
	cloudsim::host::DocumentHost* page,
	const std::string& templateBackendIdUtf8,
	Eigen::Isometry3d& outModelToWorld,
	std::string* outError)
{
	outModelToWorld = Eigen::Isometry3d::Identity();
	OsgWidget* osg = widgetOsgFromPage(page);
	if (!osg)
	{
		if (outError)
		{
			*outError = "no active 3D view";
		}
		return false;
	}
	const std::string xformId = templateTransformBackendId(osg, templateBackendIdUtf8);
	osg::Matrixd worldMat;
	if (!osg->getBackendRootWorldMatrix(xformId, worldMat))
	{
		if (outError)
		{
			*outError = "template backend world matrix unavailable";
		}
		return false;
	}
	outModelToWorld = engine::rigidTransformFromOsg(worldMat).isometry();
	return true;
}

bool queryScanStoredToWorldIsometry(
	cloudsim::host::DocumentHost* page,
	const std::string& scanBackendIdUtf8,
	Eigen::Isometry3d& outStoredToWorld,
	std::string* outError)
{
	outStoredToWorld = Eigen::Isometry3d::Identity();
	OsgWidget* osg = widgetOsgFromPage(page);
	if (!osg)
	{
		if (outError)
		{
			*outError = "no active 3D view";
		}
		return false;
	}
	const auto storedToWorld = [&](const double sx, const double sy, const double sz, osg::Vec3d& outWorld) {
		return backendStoredPointToWorldMm(osg, scanBackendIdUtf8, sx, sy, sz, outWorld);
	};
	return isometryFromThreeCalibrationPoints(storedToWorld, 0.0, 0.0, 0.0, outStoredToWorld, outError);
}

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

bool writeBackendPoseFromWorldMatrix(
	OsgWidget* osg,
	const std::string& visualId,
	BrepBackendData& brep,
	const osg::Matrixd& worldMat)
{
	(void)osg;
	(void)visualId;
	BackendVec3 pose{};
	BackendVec3 euler{};
	backend_pose_osg::backendPoseEulerFromWorldMatrix(worldMat, pose, euler);
	brep.setPose(pose);
	brep.setRotation(euler);
	return true;
}

bool tryLoadOriginalStepShape(
	cloudsim::host::DocumentHost* page,
	const std::string& templateBackendIdUtf8,
	geoalgo::ShapeHandle& outShape)
{
	outShape = geoalgo::ShapeHandle{};
	if (!page)
	{
		return false;
	}
	const QString stepPath =
		page->backendSourcePath().value(QString::fromStdString(templateBackendIdUtf8));
	if (stepPath.isEmpty())
	{
		return false;
	}
	std::string stepErr;
	return geoalgo::readStepIntoHandle(stepPath.toStdString(), outShape, &stepErr) && !outShape.isNull();
}

namespace
{

bool backendPoseNear(
	const BackendVec3& a,
	const BackendVec3& b,
	const double tolMm = 0.05)
{
	return std::abs(a.x - b.x) <= tolMm && std::abs(a.y - b.y) <= tolMm && std::abs(a.z - b.z) <= tolMm;
}

bool computeWorldMatrixForAlignedUpdatedShape(
	OsgWidget* osg,
	const std::string& templateVisualId,
	const BrepBackendData& templateBrep,
	const osg::Matrixd& templateWorld,
	const Eigen::Isometry3d& templateToScanInModelFrame,
	osg::Matrixd& outWorldMatrix)
{
	const bool hasIcpTransform =
		!templateToScanInModelFrame.matrix().isApprox(Eigen::Isometry3d::Identity().matrix());
	if (!hasIcpTransform)
	{
		outWorldMatrix = templateWorld;
		return true;
	}

	const engine::RigidTransform icpRt =
		engine::RigidTransform::fromIsometry(templateToScanInModelFrame);
	const osg::Matrixd icpOsg = engine::osgMatrixFromRigidTransform(icpRt);

	BrepBackendData worldPoseProbe;
	(void)writeBackendPoseFromWorldMatrix(osg, templateVisualId, worldPoseProbe, templateWorld);
	const bool templateUsesOriginalGeometryPose =
		backendPoseNear(templateBrep.pose(), worldPoseProbe.pose())
		&& backendPoseNear(templateBrep.rotation(), worldPoseProbe.rotation());

	// 模板若是「原始 STEP + ICP 外层矩阵」，面重构 aligned 几何应使用 worldBefore
	outWorldMatrix =
		templateUsesOriginalGeometryPose ? osg::Matrixd::inverse(icpOsg) * templateWorld : templateWorld;
	return true;
}

} // namespace

bool inheritBrepVisualPoseFromSourceMesh(
	cloudsim::host::DocumentHost* page,
	const std::string& sourceMeshBackendIdUtf8,
	const std::string& newBrepBackendIdUtf8,
	BrepBackendData& newBrep,
	std::string* outError)
{
	if (!page)
	{
		if (outError)
		{
			*outError = "invalid document page";
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
	const auto sourceObj = page->backend().getData(sourceMeshBackendIdUtf8);
	if (const auto* sourceMesh = dynamic_cast<const MeshBackendData*>(sourceObj.get()))
	{
		newBrep.setPose(sourceMesh->pose());
		newBrep.setRotation(sourceMesh->rotation());
	}
	(void)osg->syncOuterPatFromBackend(newBrep);
	osg->setBackendObjectVisible(newBrepBackendIdUtf8, true);
	return true;
}

bool alignFaceUpdatedBrepWithTemplateVisual(
	cloudsim::host::DocumentHost* page,
	const std::string& templateBackendIdUtf8,
	const std::string& updatedBrepBackendIdUtf8,
	const BrepBackendData& templateBrep,
	BrepBackendData& updatedBrep,
	const Eigen::Isometry3d& templateToScanInModelFrame,
	std::string* outError)
{
	if (!page)
	{
		if (outError)
		{
			*outError = "invalid document page";
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

	const std::string templateVisualId = templateTransformBackendId(osg, templateBackendIdUtf8);
	osg::Matrixd templateWorld;
	if (!osg->getBackendRootWorldMatrix(templateVisualId, templateWorld))
	{
		if (outError)
		{
			*outError = "template world transform unavailable";
		}
		return false;
	}

	osg::Matrixd worldForAligned;
	if (!computeWorldMatrixForAlignedUpdatedShape(
			osg, templateVisualId, templateBrep, templateWorld, templateToScanInModelFrame, worldForAligned))
	{
		if (outError)
		{
			*outError = "failed to derive aligned-shape world matrix";
		}
		return false;
	}

	(void)writeBackendPoseFromWorldMatrix(osg, templateVisualId, updatedBrep, worldForAligned);
	osg->setBackendRootWorldMatrixFromWorld(updatedBrepBackendIdUtf8, worldForAligned);
	osg->setBackendObjectVisible(updatedBrepBackendIdUtf8, true);

	return true;
}

bool applyTemplateRegistrationToVisual(
	cloudsim::host::DocumentHost* page,
	const std::string& templateBackendIdUtf8,
	const geoalgo::ShapeHandle& alignedTemplateShape,
	const Eigen::Isometry3d& templateToScanInModelFrame,
	std::string* outError)
{
	if (alignedTemplateShape.isNull())
	{
		if (outError)
		{
			*outError = "aligned template shape is null";
		}
		return false;
	}
	if (!page)
	{
		if (outError)
		{
			*outError = "invalid document page";
		}
		return false;
	}
	const auto obj = page->backend().getData(templateBackendIdUtf8);
	if (!obj)
	{
		if (outError)
		{
			*outError = "template backend object not found";
		}
		return false;
	}
	auto brep = std::dynamic_pointer_cast<BrepBackendData>(obj);
	if (!brep)
	{
		if (outError)
		{
			*outError = "template backend is not B-rep data";
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

	const std::string visualId = templateTransformBackendId(osg, templateBackendIdUtf8);
	std::shared_ptr<BrepBackendData> displayBrep = brep;
	if (visualId != templateBackendIdUtf8)
	{
		const auto visualObj = page->backend().getData(visualId);
		displayBrep = std::dynamic_pointer_cast<BrepBackendData>(visualObj);
	}
	if (!displayBrep)
	{
		if (outError)
		{
			*outError = "template visual backend not found";
		}
		return false;
	}

	const bool hasIcpTransform =
		!templateToScanInModelFrame.matrix().isApprox(Eigen::Isometry3d::Identity().matrix());

	osg::Matrixd worldBefore;
	if (!osg->getBackendRootWorldMatrix(visualId, worldBefore))
	{
		if (outError)
		{
			*outError = "template world transform unavailable";
		}
		return false;
	}

	osg::Matrixd icpOsg;
	icpOsg.makeIdentity();
	if (hasIcpTransform)
	{
		const engine::RigidTransform icpRt =
			engine::RigidTransform::fromIsometry(templateToScanInModelFrame);
		icpOsg = engine::osgMatrixFromRigidTransform(icpRt);
	}
	const osg::Matrixd worldAfter = hasIcpTransform ? icpOsg * worldBefore : worldBefore;

	geoalgo::ShapeHandle originalStepShape;
	const bool useOriginalPose =
		tryLoadOriginalStepShape(page, templateBackendIdUtf8, originalStepShape);

	if (useOriginalPose)
	{
		brep->setShape(originalStepShape.clone());
		if (displayBrep != brep)
		{
			displayBrep->setShape(originalStepShape.clone());
		}
	}
	else
	{
		RunLogger::info(
			"[TemplateBrepUpdate] applyVisual fallback=bakedAligned (STEP reload unavailable)");
		brep->setShape(alignedTemplateShape.clone());
		if (displayBrep != brep)
		{
			displayBrep->setShape(alignedTemplateShape.clone());
		}
	}

	geoalgo::clearBrepImportArtifactsCache();

	QString sceneErr;
	if (!osg->loadBackendFromBackendData(*displayBrep, &sceneErr, false, false, true))
	{
		if (outError)
		{
			*outError = sceneErr.isEmpty() ? "OSG B-rep display failed" : sceneErr.toStdString();
		}
		return false;
	}

	if (useOriginalPose)
	{
		osg->setBackendRootWorldMatrixFromWorld(visualId, worldAfter);
		(void)writeBackendPoseFromWorldMatrix(osg, visualId, *displayBrep, worldAfter);
		if (brep != displayBrep)
		{
			(void)writeBackendPoseFromWorldMatrix(osg, visualId, *brep, worldAfter);
		}
		const osg::Matrixd poseRebuild =
			backend_pose_osg::worldMatrixFromBackendPoseEuler(displayBrep->pose(), displayBrep->rotation());
		const double poseRoundTripMm = (poseRebuild.getTrans() - worldAfter.getTrans()).length();
		RunLogger::info(
			std::string("[TemplateBrepUpdate] assembly STEP: original geometry + ICP pose poseRoundTripMm=")
			+ std::to_string(poseRoundTripMm));
	}
	else
	{
		osg->setBackendRootWorldMatrixFromWorld(visualId, worldBefore);
	}

	osg->setBackendObjectVisible(visualId, true);
	return true;
}

bool restoreTemplateShapeFromStep(
	cloudsim::host::DocumentHost* page,
	const std::string& templateBackendIdUtf8,
	const std::string& templateStepPathUtf8,
	std::string* outError)
{
	if (templateStepPathUtf8.empty())
	{
		if (outError)
		{
			*outError = "template STEP path unavailable";
		}
		return false;
	}
	if (!page)
	{
		if (outError)
		{
			*outError = "invalid document page";
		}
		return false;
	}
	const auto obj = page->backend().getData(templateBackendIdUtf8);
	auto brep = std::dynamic_pointer_cast<BrepBackendData>(obj);
	if (!brep)
	{
		if (outError)
		{
			*outError = "template backend is not B-rep data";
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

	geoalgo::ShapeHandle originalShape;
	std::string stepErr;
	if (!geoalgo::readStepIntoHandle(templateStepPathUtf8, originalShape, &stepErr) || originalShape.isNull())
	{
		if (outError)
		{
			*outError = stepErr.empty() ? "failed to reload template STEP" : stepErr;
		}
		return false;
	}

	const std::string visualId = templateTransformBackendId(osg, templateBackendIdUtf8);
	std::shared_ptr<BrepBackendData> displayBrep = brep;
	if (visualId != templateBackendIdUtf8)
	{
		displayBrep = std::dynamic_pointer_cast<BrepBackendData>(page->backend().getData(visualId));
	}
	if (!displayBrep)
	{
		if (outError)
		{
			*outError = "template visual backend not found";
		}
		return false;
	}

	osg::Matrixd worldBefore;
	if (!osg->getBackendRootWorldMatrix(visualId, worldBefore))
	{
		if (outError)
		{
			*outError = "template world transform unavailable";
		}
		return false;
	}

	brep->setShape(originalShape.clone());
	if (displayBrep != brep)
	{
		displayBrep->setShape(originalShape.clone());
	}
	geoalgo::clearBrepImportArtifactsCache();

	QString sceneErr;
	if (!osg->loadBackendFromBackendData(*displayBrep, &sceneErr, false, false, true))
	{
		if (outError)
		{
			*outError = sceneErr.isEmpty() ? "OSG B-rep reload failed" : sceneErr.toStdString();
		}
		return false;
	}
	osg->setBackendRootWorldMatrixFromWorld(visualId, worldBefore);
	(void)writeBackendPoseFromWorldMatrix(osg, visualId, *displayBrep, worldBefore);
	if (brep != displayBrep)
	{
		(void)writeBackendPoseFromWorldMatrix(osg, visualId, *brep, worldBefore);
	}
	osg->setBackendObjectVisible(visualId, true);
	return true;
}

namespace
{
constexpr std::size_t kMinScanPointsForRegistration = 1000U;

bool scanPointCountSane(const std::size_t pointCount)
{
	return pointCount >= kMinScanPointsForRegistration;
}

enum class ScanBufferStatus
{
	Ok,
	Empty,
	TooFew,
	Inconsistent
};

ScanBufferStatus evaluateScanBuffer(const PointCloudBackendData& pc, std::size_t& outPointCount)
{
	outPointCount = 0U;
	const std::vector<float>& xyz = pc.pointPositionsXyz();
	if (xyz.empty())
	{
		return ScanBufferStatus::Empty;
	}
	if ((xyz.size() % 3U) != 0U)
	{
		return ScanBufferStatus::Inconsistent;
	}
	outPointCount = xyz.size() / 3U;
	if (outPointCount != pc.geometryElementCount())
	{
		return ScanBufferStatus::Inconsistent;
	}
	if (!scanPointCountSane(outPointCount))
	{
		return ScanBufferStatus::TooFew;
	}
	return ScanBufferStatus::Ok;
}
} // namespace

bool prepareScanPointCloudForRegistration(
	cloudsim::host::DocumentHost* page,
	const std::string& scanBackendIdUtf8,
	std::vector<float>& outStoredXyz,
	std::size_t& outPointCount,
	std::string* outError)
{
	outStoredXyz.clear();
	outPointCount = 0U;
	auto scan = resolvePointCloud(page, scanBackendIdUtf8, outError);
	if (!scan)
	{
		return false;
	}

	std::size_t pointCount = 0U;
	const ScanBufferStatus status = evaluateScanBuffer(*scan, pointCount);
	if (status == ScanBufferStatus::Ok)
	{
		outStoredXyz = scan->pointPositionsXyz();
		outPointCount = pointCount;
		if (pointCount > 1000000U)
		{
			RunLogger::info(
				std::string("[TemplateBrepUpdate] large scan for registration, pts=") + std::to_string(pointCount)
				+ " (voxel prefilter will reduce before ICP)");
		}
		return true;
	}

	const std::size_t badXyzPts = scan->pointPositionsXyz().size() / 3U;
	const std::size_t badElemCount = scan->geometryElementCount();

	if (status == ScanBufferStatus::TooFew && badXyzPts == badElemCount && badXyzPts > 0U)
	{
		if (outError)
		{
			*outError = "scan point count too low for registration: " + std::to_string(badXyzPts);
		}
		return false;
	}

	const QString sourcePath = page->backendSourcePath().value(QString::fromStdString(scanBackendIdUtf8));
	if (sourcePath.isEmpty())
	{
		if (outError)
		{
			std::ostringstream oss;
			oss << "scan point buffer invalid (xyzPts=" << (scan->pointPositionsXyz().size() / 3U)
				<< ", elementCount=" << scan->geometryElementCount()
				<< ") and source path unknown; re-import the point cloud file";
			*outError = oss.str();
		}
		return false;
	}

	PointCloudBackendData reloaded;
	std::string loadErr;
	if (!reloaded.readPointCloudFromPlyFile(sourcePath.toUtf8().constData(), &loadErr))
	{
		if (outError)
		{
			*outError = loadErr.empty() ? "failed to reload scan PLY" : loadErr;
		}
		return false;
	}
	const std::size_t reloadedCount = reloaded.geometryElementCount();
	std::size_t reloadedPts = 0U;
	if (evaluateScanBuffer(reloaded, reloadedPts) != ScanBufferStatus::Ok)
	{
		if (outError)
		{
			*outError = "reloaded scan buffer still invalid (pts=" + std::to_string(reloadedCount) + ")";
		}
		return false;
	}

	scan->setPointBuffers(
		reloaded.pointPositionsXyz(),
		reloaded.pointVertexRgba(),
		reloaded.pointNormalsNxNyNz());
	commitPointCloudVisual(page, *scan);
	RunLogger::info(
		std::string("[TemplateBrepUpdate] scan reloaded from PLY, pts=") + std::to_string(reloadedCount)
		+ " (was inconsistent xyzPts=" + std::to_string(badXyzPts)
		+ " elemCount=" + std::to_string(badElemCount) + ")");

	outStoredXyz = scan->pointPositionsXyz();
	outPointCount = reloadedPts;
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

bool exportPointCloudToPly(
	cloudsim::host::DocumentHost* page,
	const std::string& backendIdUtf8,
	const std::string& pathUtf8,
	std::string* outError)
{
	std::string resolveErr;
	const auto pc = resolvePointCloud(page, backendIdUtf8, &resolveErr);
	if (!pc)
	{
		if (outError)
		{
			*outError = resolveErr;
		}
		return false;
	}
	if (!pc->hasGeometry())
	{
		if (outError)
		{
			*outError = "point cloud has no geometry";
		}
		return false;
	}
	const std::string nativePath = nativePathFromUtf8Path(pathUtf8);
	if (!pc->writePointCloudPlySidecar(nativePath, outError))
	{
		return false;
	}
	return true;
}

bool exportBrepToStep(
	cloudsim::host::DocumentHost* page,
	const std::string& backendIdUtf8,
	const std::string& pathUtf8,
	std::string* outError)
{
	if (!page)
	{
		if (outError)
		{
			*outError = "invalid document page";
		}
		return false;
	}
	const auto obj = page->backend().getData(backendIdUtf8);
	if (!obj)
	{
		if (outError)
		{
			*outError = "backend object not found";
		}
		return false;
	}
	const auto brep = std::dynamic_pointer_cast<BrepBackendData>(obj);
	if (!brep)
	{
		if (outError)
		{
			*outError = "not a B-rep backend";
		}
		return false;
	}
	if (!brep->hasGeometry())
	{
		if (outError)
		{
			*outError = "B-rep has no geometry";
		}
		return false;
	}
	const std::string nativePath = nativePathFromUtf8Path(pathUtf8);
	if (!brep->writeStepFile(nativePath, outError))
	{
		return false;
	}
	return true;
}

bool buildPointCloudModelToWorld(const PointCloudBackendData& data, PluginMat4& outModelToWorld)
{
	const std::vector<float>& xyz = data.pointPositionsXyz();
	if (xyz.size() < 3U)
	{
		return false;
	}
	const osg::Vec3f center = backend_geometry_metrics::pointCloudCenterFromXyz(xyz);
	const BackendVec3 p = data.pose();
	const BackendVec3 r = data.rotation();
	const osg::Quat q = backendvisual_math::eulerDegToQuat(
		osg::Vec3f(static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.z)));
	const osg::Matrixd rot = osg::Matrixd::rotate(q);
	const osg::Matrixd negCenter = osg::Matrixd::translate(
		-static_cast<double>(center.x()),
		-static_cast<double>(center.y()),
		-static_cast<double>(center.z()));
	const osg::Matrixd pos = osg::Matrixd::translate(
		static_cast<double>(center.x()) + p.x,
		static_cast<double>(center.y()) + p.y,
		static_cast<double>(center.z()) + p.z);
	const osg::Matrixd modelToWorld = pos * rot * negCenter;
	for (int col = 0; col < 4; ++col)
	{
		for (int row = 0; row < 4; ++row)
		{
			outModelToWorld.v[col * 4 + row] = modelToWorld(row, col);
		}
	}
	return true;
}

} // namespace document_point_cloud_ops
