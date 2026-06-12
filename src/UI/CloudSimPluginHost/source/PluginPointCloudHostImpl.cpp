#include "PluginPointCloudHostImpl.h"

#include "BackendDataManager.h"
#include "BackendFileImport.h"
#include "BrepBackendData.h"
#include "DocumentImportFacade.h"
#include "DocumentPointCloudOps.h"
#include "DocumentHost.h"
#include "MeshBackendData.h"
#include "PluginDocumentAdapter.h"
#include "PluginHostContext.h"
#include "IPluginMainWindowHost.h"
#include "PointCloudBackendData.h"
#include "OsgWidget.h"
#include "WidgetDocumentAccess.h"
#include "TemplateBrepUpdate.h"

#include "PointCloudBackendOps.h"

#include <Adapters.h>
#include <BrepImportArtifacts.h>
#include <GeometryBackendOps.h>
#include <MeshSurfaceReconstruction.h>

#include <osg/Matrixd>
#include <osg/Quat>
#include <osg/Vec3f>

#include <functional>
#include <memory>
#include <stdexcept>

namespace
{

QString makeUniqueBrepDisplayName(cloudsim::host::DocumentHost& page, const QString& baseName)
{
	const QString root = baseName.isEmpty() ? QStringLiteral("BrepUpdated") : baseName;
	QString candidate = root;
	int suffix = 2;
	for (;;)
	{
		const std::vector<std::shared_ptr<BackendDataBase>> found =
			page.backend().findByName(candidate.toStdString());
		if (found.empty())
		{
			return candidate;
		}
		candidate = root + QStringLiteral("_%1").arg(suffix++);
	}
}

cloudsim::host::DocumentHost* pageFromDoc(IPluginDocument* doc)
{
	if (!doc)
	{
		return nullptr;
	}
	const auto* adapter = dynamic_cast<const PluginDocumentAdapter*>(doc);
	return adapter ? adapter->documentHost() : nullptr;
}

using MutateFn = std::function<bool(PointCloudBackendData&, std::string*)>;

void runMutateJob(
	PluginHostContext* host,
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const QString& jobTitle,
	MutateFn mutate,
	PluginPointCloudFinishedFn onFinished)
{
	if (!host || !onFinished)
	{
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	std::string resolveErr;
	const auto pc = document_point_cloud_ops::resolvePointCloud(page, backendIdUtf8, &resolveErr);
	if (!pc)
	{
		onFinished(false, QString::fromStdString(resolveErr), {});
		return;
	}

	struct WorkResult
	{
		std::vector<float> xyz;
		std::vector<float> rgba;
		std::vector<float> normals;
		std::string error;
		bool ok = false;
	};
	auto result = std::make_shared<WorkResult>();

	host->enqueueJob(
		jobTitle,
		[result, pc, mutate = std::move(mutate)](const PluginJobProgressFn& report) {
			PointCloudBackendData working;
			working.setPointBuffers(pc->pointPositionsXyz(), pc->pointVertexRgba(), pc->pointNormalsNxNyNz());
			report(0.2, QStringLiteral("Running..."));
			result->ok = mutate(working, &result->error);
			if (result->ok)
			{
				result->xyz = working.pointPositionsXyz();
				result->rgba = working.pointVertexRgba();
				result->normals = working.pointNormalsNxNyNz();
			}
			report(1.0, QStringLiteral("Done"));
		},
		[host, page, pc, backendIdUtf8, result, onFinished = std::move(onFinished)](
			const bool threw, const QString& throwMessage) {
			PluginPointCloudJobResult jobResult;
			if (threw)
			{
				onFinished(false, throwMessage, jobResult);
				return;
			}
			if (!result->ok)
			{
				onFinished(false, QString::fromStdString(result->error), jobResult);
				return;
			}
			pc->setPointBuffers(std::move(result->xyz), std::move(result->rgba), std::move(result->normals));
			document_point_cloud_ops::commitPointCloudVisual(page, *pc);
			jobResult.pointCountAfter = pc->geometryElementCount();
			(void)host;
			(void)backendIdUtf8;
			onFinished(true, QString(), jobResult);
		});
}

geoalgo::TemplateBrepUpdateParams buildTemplateBrepGeoParams(
	const PluginPointCloudTemplateBrepUpdateParams& params)
{
	geoalgo::TemplateBrepUpdateParams geoParams;
	geoParams.voxelPrefilterMm = params.voxelPrefilterMm;
	geoParams.faceBandMm = params.faceBandMm;
	geoParams.normalThresholdDeg = params.normalThresholdDeg;
	geoParams.minPointsPerFace = params.minPointsPerFace;
	geoParams.maxAllowedDeviationMm = params.maxAllowedDeviationMm;
	geoParams.selectedFaceIndices = params.selectedFaceIndices;
	geoParams.maxAssignPointsPerFace = params.maxAssignPointsPerFace;
	geoParams.bsplineUvGridCellsU = params.bsplineUvGridCellsU;
	geoParams.bsplineUvGridCellsV = params.bsplineUvGridCellsV;
	geoParams.bsplinePoleSmoothPasses = params.bsplinePoleSmoothPasses;
	geoParams.registrationStage =
		static_cast<geoalgo::TemplateBrepRegistrationStage>(static_cast<int>(params.registrationStage));
	return geoParams;
}

const char* faceUpdateActionName(const geoalgo::FaceUpdateAction action)
{
	switch (action)
	{
	case geoalgo::FaceUpdateAction::Unchanged:
		return "Unchanged";
	case geoalgo::FaceUpdateAction::PlaneRefit:
		return "PlaneRefit";
	case geoalgo::FaceUpdateAction::CylinderRefit:
		return "CylinderRefit";
	case geoalgo::FaceUpdateAction::FreeformRefit:
		return "FreeformRefit";
	case geoalgo::FaceUpdateAction::SkippedNoPoints:
		return "SkippedNoPoints";
	case geoalgo::FaceUpdateAction::ConeRefit:
		return "ConeRefit";
	case geoalgo::FaceUpdateAction::SphereRefit:
		return "SphereRefit";
	case geoalgo::FaceUpdateAction::ToroidRefit:
		return "ToroidRefit";
	case geoalgo::FaceUpdateAction::PlaneAdjusted:
		return "PlaneAdjusted";
	case geoalgo::FaceUpdateAction::CylinderAdjusted:
		return "CylinderAdjusted";
	case geoalgo::FaceUpdateAction::ConeAdjusted:
		return "ConeAdjusted";
	case geoalgo::FaceUpdateAction::SphereAdjusted:
		return "SphereAdjusted";
	case geoalgo::FaceUpdateAction::ToroidAdjusted:
		return "ToroidAdjusted";
	case geoalgo::FaceUpdateAction::BSplineAdjusted:
		return "BSplineAdjusted";
	default:
		return "Unknown";
	}
}

void fillPerFaceReports(
	const geoalgo::TemplateBrepUpdateResult& report,
	std::vector<PluginPointCloudFaceUpdateReport>& outPerFace)
{
	outPerFace.clear();
	outPerFace.reserve(report.perFace.size());
	for (const geoalgo::FaceUpdateReport& faceReport : report.perFace)
	{
		PluginPointCloudFaceUpdateReport sdkReport;
		sdkReport.faceIndex = faceReport.faceIndex;
		sdkReport.surfaceTypeName = faceReport.surfaceTypeName;
		sdkReport.action = faceUpdateActionName(faceReport.action);
		sdkReport.maxDeviationMm = faceReport.maxDeviationMm;
		outPerFace.push_back(std::move(sdkReport));
	}
}

std::shared_ptr<BrepBackendData> resolveBrep(
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
	auto brep = std::dynamic_pointer_cast<BrepBackendData>(obj);
	if (!brep)
	{
		if (outError)
		{
			*outError = "backend is not B-rep data";
		}
		return nullptr;
	}
	if (!brep->hasGeometry())
	{
		if (outError)
		{
			*outError = "B-rep backend has no shape geometry";
		}
		return nullptr;
	}
	return brep;
}

} // namespace

PluginPointCloudHostImpl::PluginPointCloudHostImpl(PluginHostContext* hostContext)
	: m_host(hostContext)
{
}

void PluginPointCloudHostImpl::downsamplePointCloudVoxel(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginPointCloudDownsampleVoxelParams& params,
	PluginPointCloudFinishedFn onFinished)
{
	runMutateJob(
		m_host,
		doc,
		backendIdUtf8,
		QStringLiteral("Voxel downsample"),
		[params](PointCloudBackendData& data, std::string* err) {
			return point_cloud_backend_ops::downsamplePointCloudVoxel(
				data, params.voxelSizeMm, params.minPointsPerCell, err);
		},
		std::move(onFinished));
}

void PluginPointCloudHostImpl::downsamplePointCloudRandom(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginPointCloudDownsampleRandomParams& params,
	PluginPointCloudFinishedFn onFinished)
{
	runMutateJob(
		m_host,
		doc,
		backendIdUtf8,
		QStringLiteral("Random downsample"),
		[params](PointCloudBackendData& data, std::string* err) {
			return point_cloud_backend_ops::downsamplePointCloudRandom(data, params.retainedFraction, err);
		},
		std::move(onFinished));
}

void PluginPointCloudHostImpl::cropPointCloudByBox(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginPointCloudCropBoxParams& params,
	PluginPointCloudFinishedFn onFinished)
{
	runMutateJob(
		m_host,
		doc,
		backendIdUtf8,
		QStringLiteral("Box crop"),
		[params](PointCloudBackendData& data, std::string* err) {
			return point_cloud_backend_ops::cropPointCloudByBox(
				data, document_point_cloud_ops::toEigenBox(params.box), err);
		},
		std::move(onFinished));
}

void PluginPointCloudHostImpl::cropPointCloudBySphere(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginPointCloudCropSphereParams& params,
	PluginPointCloudFinishedFn onFinished)
{
	runMutateJob(
		m_host,
		doc,
		backendIdUtf8,
		QStringLiteral("Sphere crop"),
		[params](PointCloudBackendData& data, std::string* err) {
			const Eigen::Vector3d center(params.centerMm.x, params.centerMm.y, params.centerMm.z);
			return point_cloud_backend_ops::cropPointCloudBySphere(data, center, params.radiusMm, err);
		},
		std::move(onFinished));
}

void PluginPointCloudHostImpl::applyRigidTransformToPointCloud(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginPointCloudRigidTransformParams& params,
	PluginPointCloudFinishedFn onFinished)
{
	runMutateJob(
		m_host,
		doc,
		backendIdUtf8,
		QStringLiteral("Rigid transform"),
		[params](PointCloudBackendData& data, std::string* err) {
			return point_cloud_backend_ops::applyRigidTransformToPointCloud(
				data, document_point_cloud_ops::toEigenIsometry(params.transform), err);
		},
		std::move(onFinished));
}

void PluginPointCloudHostImpl::removePointCloudOutliers(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginPointCloudOutlierParams& params,
	PluginPointCloudFinishedFn onFinished)
{
	runMutateJob(
		m_host,
		doc,
		backendIdUtf8,
		QStringLiteral("Remove outliers"),
		[params](PointCloudBackendData& data, std::string* err) {
			return point_cloud_backend_ops::removePointCloudOutliers(
				data, params.removalPercent, params.kNeighbors, err);
		},
		std::move(onFinished));
}

void PluginPointCloudHostImpl::smoothPointCloudBilateral(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	PluginPointCloudFinishedFn onFinished)
{
	runMutateJob(
		m_host,
		doc,
		backendIdUtf8,
		QStringLiteral("Bilateral smooth"),
		[](PointCloudBackendData& data, std::string* err) {
			return point_cloud_backend_ops::smoothPointCloudBilateral(data, err);
		},
		std::move(onFinished));
}

void PluginPointCloudHostImpl::estimatePointCloudNormalsPca(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginPointCloudNormalsParams& params,
	PluginPointCloudFinishedFn onFinished)
{
	runMutateJob(
		m_host,
		doc,
		backendIdUtf8,
		QStringLiteral("Estimate normals PCA"),
		[params](PointCloudBackendData& data, std::string* err) {
			return point_cloud_backend_ops::estimatePointCloudNormalsPca(data, params.kNeighbors, err);
		},
		std::move(onFinished));
}

void PluginPointCloudHostImpl::estimatePointCloudNormalsJet(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginPointCloudNormalsParams& params,
	PluginPointCloudFinishedFn onFinished)
{
	runMutateJob(
		m_host,
		doc,
		backendIdUtf8,
		QStringLiteral("Estimate normals Jet"),
		[params](PointCloudBackendData& data, std::string* err) {
			return point_cloud_backend_ops::estimatePointCloudNormalsJet(
				data, params.kNeighbors, params.jetDegreeFitting, err);
		},
		std::move(onFinished));
}

void PluginPointCloudHostImpl::orientPointCloudNormalsMst(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginPointCloudNormalsParams& params,
	PluginPointCloudFinishedFn onFinished)
{
	runMutateJob(
		m_host,
		doc,
		backendIdUtf8,
		QStringLiteral("Orient normals MST"),
		[params](PointCloudBackendData& data, std::string* err) {
			return point_cloud_backend_ops::orientPointCloudNormalsMst(data, params.kNeighbors, err);
		},
		std::move(onFinished));
}

void PluginPointCloudHostImpl::preprocessPointCloudForReconstruction(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginPointCloudPreprocessParams& params,
	PluginPointCloudFinishedFn onFinished)
{
	runMutateJob(
		m_host,
		doc,
		backendIdUtf8,
		QStringLiteral("Preprocess for reconstruction"),
		[params](PointCloudBackendData& data, std::string* err) {
			return point_cloud_backend_ops::preprocessPointCloudForReconstruction(
				data, params.voxelPrefilterMm, params.outlierRemovalPercent, err);
		},
		std::move(onFinished));
}

void PluginPointCloudHostImpl::rigidRegisterPointCloudsIcp(
	IPluginDocument* doc,
	const std::string& sourceBackendIdUtf8,
	const PluginPointCloudIcpParams& params,
	PluginPointCloudFinishedFn onFinished)
{
	if (!m_host || !onFinished)
	{
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	std::string resolveErr;
	const auto source = document_point_cloud_ops::resolvePointCloud(page, sourceBackendIdUtf8, &resolveErr);
	if (!source)
	{
		onFinished(false, QString::fromStdString(resolveErr), {});
		return;
	}
	const auto target = document_point_cloud_ops::resolvePointCloud(page, params.targetBackendIdUtf8, &resolveErr);
	if (!target)
	{
		onFinished(false, QString::fromStdString(resolveErr), {});
		return;
	}

	struct IcpResult
	{
		point_cloud_backend_ops::PointCloudIcpResult icp;
		std::string error;
		bool ok = false;
	};
	auto result = std::make_shared<IcpResult>();

	m_host->enqueueJob(
		QStringLiteral("ICP registration"),
		[source, target, params, result](const PluginJobProgressFn& report) {
			report(0.2, QStringLiteral("Running ICP..."));
			result->ok = point_cloud_backend_ops::rigidRegisterPointCloudsIcp(
				*source,
				*target,
				result->icp,
				params.maxIterations,
				params.convergenceTransMm,
				params.maxPairDistanceMm,
				params.icpMaxPoints,
				&result->error);
			report(1.0, QStringLiteral("Done"));
		},
		[page, source, params, result, onFinished = std::move(onFinished)](const bool threw, const QString& throwMessage) {
			PluginPointCloudJobResult jobResult;
			if (threw)
			{
				onFinished(false, throwMessage, jobResult);
				return;
			}
			if (!result->ok)
			{
				onFinished(false, QString::fromStdString(result->error), jobResult);
				return;
			}
			jobResult.icpTransform = document_point_cloud_ops::toPluginMat4(result->icp.sourceToTarget);
			jobResult.rmseMm = result->icp.rmseMm;
			if (params.applyTransformToSource)
			{
				std::string err;
				point_cloud_backend_ops::applyRigidTransformToPointCloud(
					*source, result->icp.sourceToTarget, &err);
				document_point_cloud_ops::commitPointCloudVisual(page, *source);
				jobResult.pointCountAfter = source->geometryElementCount();
			}
			onFinished(true, QString(), jobResult);
		});
}

void PluginPointCloudHostImpl::deformPointCloudTpsFromControls(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginPointCloudTpsControlParams& params,
	PluginPointCloudFinishedFn onFinished)
{
	runMutateJob(
		m_host,
		doc,
		backendIdUtf8,
		QStringLiteral("TPS deform"),
		[params](PointCloudBackendData& data, std::string* err) {
			return point_cloud_backend_ops::deformPointCloudTpsFromControls(
				data,
				params.controlPointIndices,
				params.controlDisplacementXyz,
				params.regularizationLambda,
				err);
		},
		std::move(onFinished));
}

void PluginPointCloudHostImpl::deformPointCloudTpsFitAndDeform(
	IPluginDocument* doc,
	const std::string& sourceBackendIdUtf8,
	const PluginPointCloudTpsFitParams& params,
	PluginPointCloudFinishedFn onFinished)
{
	if (!m_host || !onFinished)
	{
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	std::string resolveErr;
	const auto source = document_point_cloud_ops::resolvePointCloud(page, sourceBackendIdUtf8, &resolveErr);
	if (!source)
	{
		onFinished(false, QString::fromStdString(resolveErr), {});
		return;
	}
	const auto target = document_point_cloud_ops::resolvePointCloud(page, params.targetBackendIdUtf8, &resolveErr);
	if (!target)
	{
		onFinished(false, QString::fromStdString(resolveErr), {});
		return;
	}

	struct TpsResult
	{
		std::vector<float> deformedXyz;
		std::string error;
		bool ok = false;
	};
	auto result = std::make_shared<TpsResult>();

	m_host->enqueueJob(
		QStringLiteral("TPS fit and deform"),
		[source, target, params, result](const PluginJobProgressFn& report) {
			report(0.2, QStringLiteral("Running TPS..."));
			result->ok = point_cloud_backend_ops::deformPointCloudTpsFitAndDeform(
				*source,
				*target,
				params.correspondenceIndices,
				result->deformedXyz,
				params.regularizationLambda,
				&result->error);
			report(1.0, QStringLiteral("Done"));
		},
		[m_host = m_host, page, source, params, result, onFinished = std::move(onFinished)](
			const bool threw, const QString& throwMessage) {
			PluginPointCloudJobResult jobResult;
			if (threw)
			{
				onFinished(false, throwMessage, jobResult);
				return;
			}
			if (!result->ok)
			{
				onFinished(false, QString::fromStdString(result->error), jobResult);
				return;
			}
			if (params.createNewPointCloud)
			{
				auto newPc = std::make_shared<PointCloudBackendData>();
				newPc->setName(source->name() + "_tps");
				newPc->setPointBuffers(std::move(result->deformedXyz), {}, {});
				cloudsim::host::AdoptPointCloudOptions adoptOpt;
				adoptOpt.sourcePath = QStringLiteral("plugin://pointcloud/tps");
				QString regErr;
				const cloudsim::host::AdoptRegistrationResult adopted =
					cloudsim::host::registerAdoptedPointCloud(*page, newPc, adoptOpt, &regErr);
				if (!adopted.ok)
				{
					onFinished(false, regErr, jobResult);
					return;
				}
				jobResult.newBackendId = adopted.backendId.toStdString();
				jobResult.pointCountAfter = newPc->geometryElementCount();
			}
			else
			{
				source->setPointBuffers(std::move(result->deformedXyz), source->pointVertexRgba(), {});
				document_point_cloud_ops::commitPointCloudVisual(page, *source);
				jobResult.pointCountAfter = source->geometryElementCount();
			}
			(void)m_host;
			onFinished(true, QString(), jobResult);
		});
}

void PluginPointCloudHostImpl::reconstructMeshPoisson(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginPointCloudReconstructPoissonParams& params,
	PluginPointCloudFinishedFn onFinished)
{
	if (!m_host || !onFinished)
	{
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	std::string resolveErr;
	const auto pc = document_point_cloud_ops::resolvePointCloud(page, backendIdUtf8, &resolveErr);
	if (!pc)
	{
		onFinished(false, QString::fromStdString(resolveErr), {});
		return;
	}

	struct MeshResult
	{
		std::vector<float> triangleSoup;
		std::string error;
		bool ok = false;
	};
	auto result = std::make_shared<MeshResult>();

	m_host->enqueueJob(
		QStringLiteral("Poisson reconstruction"),
		[pc, params, result](const PluginJobProgressFn& report) {
			report(0.2, QStringLiteral("Reconstructing..."));
			MeshBackendData meshOut;
			result->ok = point_cloud_backend_ops::reconstructMeshPoisson(
				*pc,
				meshOut,
				params.spacingMm,
				params.smAngleDeg,
				params.smRadiusRel,
				params.smDistanceRel,
				&result->error);
			if (result->ok)
			{
				result->triangleSoup = meshOut.triangleSoup();
			}
			report(1.0, QStringLiteral("Done"));
		},
		[m_host = m_host, page, params, result, onFinished = std::move(onFinished)](
			const bool threw, const QString& throwMessage) {
			PluginPointCloudJobResult jobResult;
			if (threw)
			{
				onFinished(false, throwMessage, jobResult);
				return;
			}
			if (!result->ok)
			{
				onFinished(false, QString::fromStdString(result->error), jobResult);
				return;
			}
			auto meshPtr = std::make_shared<MeshBackendData>();
			meshPtr->setTriangleSoup(std::move(result->triangleSoup));
			std::string regErr;
			jobResult.newBackendId = document_point_cloud_ops::registerReconstructedMesh(
				page, m_host->mainWindowHost(), meshPtr, params.meshOptions, &regErr);
			if (jobResult.newBackendId.empty())
			{
				onFinished(false, QString::fromStdString(regErr), jobResult);
				return;
			}
			onFinished(true, QString(), jobResult);
		});
}

void PluginPointCloudHostImpl::reconstructMeshPoissonAuto(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginPointCloudReconstructPoissonAutoParams& params,
	PluginPointCloudFinishedFn onFinished)
{
	if (!m_host || !onFinished)
	{
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	std::string resolveErr;
	const auto pc = document_point_cloud_ops::resolvePointCloud(page, backendIdUtf8, &resolveErr);
	if (!pc)
	{
		onFinished(false, QString::fromStdString(resolveErr), {});
		return;
	}

	struct MeshResult
	{
		std::vector<float> triangleSoup;
		std::string error;
		bool ok = false;
	};
	auto result = std::make_shared<MeshResult>();

	m_host->enqueueJob(
		QStringLiteral("Poisson auto reconstruction"),
		[pc, params, result](const PluginJobProgressFn& report) {
			report(0.2, QStringLiteral("Reconstructing..."));
			MeshBackendData meshOut;
			result->ok = point_cloud_backend_ops::reconstructMeshFromPointCloudPoisson(
				*pc, meshOut, params.voxelPrefilterMm, &result->error);
			if (result->ok)
			{
				result->triangleSoup = meshOut.triangleSoup();
			}
			report(1.0, QStringLiteral("Done"));
		},
		[m_host = m_host, page, params, result, onFinished = std::move(onFinished)](
			const bool threw, const QString& throwMessage) {
			PluginPointCloudJobResult jobResult;
			if (threw)
			{
				onFinished(false, throwMessage, jobResult);
				return;
			}
			if (!result->ok)
			{
				onFinished(false, QString::fromStdString(result->error), jobResult);
				return;
			}
			auto meshPtr = std::make_shared<MeshBackendData>();
			meshPtr->setTriangleSoup(std::move(result->triangleSoup));
			std::string regErr;
			jobResult.newBackendId = document_point_cloud_ops::registerReconstructedMesh(
				page, m_host->mainWindowHost(), meshPtr, params.meshOptions, &regErr);
			if (jobResult.newBackendId.empty())
			{
				onFinished(false, QString::fromStdString(regErr), jobResult);
				return;
			}
			onFinished(true, QString(), jobResult);
		});
}

void PluginPointCloudHostImpl::reconstructMeshScaleSpace(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginPointCloudReconstructScaleSpaceParams& params,
	PluginPointCloudFinishedFn onFinished)
{
	if (!m_host || !onFinished)
	{
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	std::string resolveErr;
	const auto pc = document_point_cloud_ops::resolvePointCloud(page, backendIdUtf8, &resolveErr);
	if (!pc)
	{
		onFinished(false, QString::fromStdString(resolveErr), {});
		return;
	}

	struct MeshResult
	{
		std::vector<float> triangleSoup;
		std::string error;
		bool ok = false;
	};
	auto result = std::make_shared<MeshResult>();

	m_host->enqueueJob(
		QStringLiteral("Scale-space reconstruction"),
		[pc, params, result](const PluginJobProgressFn& report) {
			report(0.2, QStringLiteral("Reconstructing..."));
			MeshBackendData meshOut;
			result->ok = point_cloud_backend_ops::reconstructMeshScaleSpace(
				*pc, meshOut, params.smoothIterations, params.meshingRadiusMm, &result->error);
			if (result->ok)
			{
				result->triangleSoup = meshOut.triangleSoup();
			}
			report(1.0, QStringLiteral("Done"));
		},
		[m_host = m_host, page, params, result, onFinished = std::move(onFinished)](
			const bool threw, const QString& throwMessage) {
			PluginPointCloudJobResult jobResult;
			if (threw)
			{
				onFinished(false, throwMessage, jobResult);
				return;
			}
			if (!result->ok)
			{
				onFinished(false, QString::fromStdString(result->error), jobResult);
				return;
			}
			auto meshPtr = std::make_shared<MeshBackendData>();
			meshPtr->setTriangleSoup(std::move(result->triangleSoup));
			std::string regErr;
			jobResult.newBackendId = document_point_cloud_ops::registerReconstructedMesh(
				page, m_host->mainWindowHost(), meshPtr, params.meshOptions, &regErr);
			if (jobResult.newBackendId.empty())
			{
				onFinished(false, QString::fromStdString(regErr), jobResult);
				return;
			}
			onFinished(true, QString(), jobResult);
		});
}

bool PluginPointCloudHostImpl::cacheMatches(
	IPluginDocument* doc,
	const std::string& scanId,
	const std::string& templateId) const
{
	return m_templateBrepAlignCache.doc == doc
		&& m_templateBrepAlignCache.scanId == scanId
		&& m_templateBrepAlignCache.templateId == templateId
		&& m_templateBrepAlignCache.report.icpRmseGatePassed;
}

void PluginPointCloudHostImpl::registerScanToCadTemplate(
	IPluginDocument* doc,
	const std::string& scanBackendIdUtf8,
	const PluginPointCloudTemplateBrepUpdateParams& params,
	PluginPointCloudTemplateBrepRegisterFinishedFn onFinished)
{
	if (!m_host || !onFinished)
	{
		return;
	}
	if (params.templateBrepBackendIdUtf8.empty())
	{
		onFinished(false, QStringLiteral("CAD template B-rep not selected"), {});
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	std::string resolveErr;
	const auto scan = document_point_cloud_ops::resolvePointCloud(page, scanBackendIdUtf8, &resolveErr);
	if (!scan)
	{
		onFinished(false, QString::fromStdString(resolveErr), {});
		return;
	}
	const auto templateBrep = resolveBrep(page, params.templateBrepBackendIdUtf8, &resolveErr);
	if (!templateBrep)
	{
		onFinished(false, QString::fromStdString(resolveErr), {});
		return;
	}
	const QString templateStepPath =
		page->backendSourcePath().value(QString::fromStdString(params.templateBrepBackendIdUtf8));

	if (params.registrationStage == PluginPointCloudTemplateBrepRegistrationStage::FineOnly)
	{
		const bool idsMatch =
			m_templateBrepAlignCache.doc == doc
			&& m_templateBrepAlignCache.scanId == scanBackendIdUtf8
			&& m_templateBrepAlignCache.templateId == params.templateBrepBackendIdUtf8;
		if (!idsMatch || !m_templateBrepAlignCache.registrationCheckpoint.valid)
		{
			onFinished(
				false,
				QStringLiteral("Run coarse registration first (no checkpoint for fine match)"),
				{});
			return;
		}
	}
	else
	{
		m_templateBrepAlignCache.registrationCheckpoint.valid = false;
	}

	std::vector<float> scanXyzStored;
	std::size_t scanPointCount = 0U;
	std::string scanPrepErr;
	if (!document_point_cloud_ops::prepareScanPointCloudForRegistration(
			page, scanBackendIdUtf8, scanXyzStored, scanPointCount, &scanPrepErr))
	{
		onFinished(false, QString::fromStdString(scanPrepErr), {});
		return;
	}

	{
		std::vector<float> templateSoupXyz;
		std::vector<float> templateSoupNormals;
		std::size_t templateTriCount = 0U;
		std::string soupErr;
		if (geoalgo::extractDisplaySoupPointCloud(
				templateBrep->shapeRef(),
				templateSoupXyz,
				templateSoupNormals,
				40000U,
				&templateTriCount,
				&soupErr))
		{
			const double gateMm = std::max(params.faceBandMm * 6.0, 15.0);
			document_point_cloud_ops::logRegistrationOverlapDiagnostic(
				page,
				scanBackendIdUtf8,
				params.templateBrepBackendIdUtf8,
				scanXyzStored,
				templateSoupXyz,
				gateMm);
			const BackendBoundingBox tplBounds = templateBrep->geometryBounds();
			const double tplCx = 0.5 * (tplBounds.min.x + tplBounds.max.x);
			const double tplCy = 0.5 * (tplBounds.min.y + tplBounds.max.y);
			const double tplCz = 0.5 * (tplBounds.min.z + tplBounds.max.z);
			document_point_cloud_ops::logRegistrationCentroidDiagnostic(
				page,
				scanBackendIdUtf8,
				params.templateBrepBackendIdUtf8,
				scanXyzStored,
				tplCx,
				tplCy,
				tplCz);
		}
	}

	struct RegisterResult
	{
		geoalgo::TemplateBrepUpdateResult report;
		std::string error;
		bool registerOk = false;
	};
	auto result = std::make_shared<RegisterResult>();
	PluginPointCloudTemplateBrepRegisterFinishedFn onFinishedFinal = std::move(onFinished);
	geoalgo::TemplateBrepRegistrationCheckpoint* const registrationCheckpointPtr =
		&m_templateBrepAlignCache.registrationCheckpoint;
	const geoalgo::TemplateBrepRegistrationCheckpoint checkpointBeforeJob =
		m_templateBrepAlignCache.registrationCheckpoint;

	m_host->enqueueJob(
		QStringLiteral("Register scan to CAD template"),
		[scan, templateBrep, params, templateStepPath, scanXyzStored, result, registrationCheckpointPtr](
			const PluginJobProgressFn& report) {
			report(0.05, QStringLiteral("Preparing..."));

			PointCloudBackendData scanWork;
			std::vector<float> scanRgba = scan->pointVertexRgba();
			scanWork.setPointBuffers(scanXyzStored, std::move(scanRgba), {});
			scanWork.setWorldMatrix(scan->worldMatrix());

			const geoalgo::TemplateBrepUpdateParams geoParams = buildTemplateBrepGeoParams(params);

			report(0.2, QStringLiteral("ICP registration..."));
			result->registerOk = geometry_backend_ops::registerScanToCadTemplate(
				*templateBrep,
				scanWork,
				geoParams,
				result->report,
				&result->error,
				templateStepPath.toStdString(),
				registrationCheckpointPtr);
			report(1.0, QStringLiteral("Registration done"));
		},
		[this, m_host = m_host, page, scan, doc, params, scanBackendIdUtf8, templateStepPath, templateBrep, scanXyzStored, result, checkpointBeforeJob, onFinished = onFinishedFinal](
			const bool threw, const QString& throwMessage) {
			PluginPointCloudTemplateBrepRegisterResult registerResult;
			if (threw)
			{
				onFinished(false, throwMessage, registerResult);
				return;
			}

			if (result->registerOk)
			{
				const bool isCoarseStage =
					params.registrationStage == PluginPointCloudTemplateBrepRegistrationStage::CoarseOnly;
				const bool isFineStage =
					params.registrationStage == PluginPointCloudTemplateBrepRegistrationStage::FineOnly;
				const bool applyVisual =
					result->report.registrationPreviewOk || isCoarseStage;
				std::string applyErr;
				if (applyVisual)
				{
					Eigen::Isometry3d deltaToApply = result->report.icpDeltaWorld;
					if (isFineStage && checkpointBeforeJob.valid)
					{
						deltaToApply =
							result->report.icpDeltaWorld * checkpointBeforeJob.icpDeltaWorld.inverse();
					}
					if (document_point_cloud_ops::applyTemplateRegistrationToVisual(
							page,
							params.templateBrepBackendIdUtf8,
							deltaToApply,
							&applyErr))
					{
						const BackendBoundingBox tplBounds = templateBrep->geometryBounds();
						const double tplCx = 0.5 * (tplBounds.min.x + tplBounds.max.x);
						const double tplCy = 0.5 * (tplBounds.min.y + tplBounds.max.y);
						const double tplCz = 0.5 * (tplBounds.min.z + tplBounds.max.z);
						document_point_cloud_ops::logRegistrationCentroidDiagnostic(
							page,
							scanBackendIdUtf8,
							params.templateBrepBackendIdUtf8,
							scanXyzStored,
							tplCx,
							tplCy,
							tplCz);
						if (m_host)
						{
							const QString stageLabel = isCoarseStage
								? QStringLiteral("Coarse match applied to CAD preview")
								: QStringLiteral("CAD template aligned to scan (ICP RMSE %1 mm)");
							m_host->logInfo(
								isCoarseStage
									? stageLabel
									: stageLabel.arg(result->report.icpRmseMm, 0, 'f', 3));
							if (IPluginMainWindowHost* mw = m_host->mainWindowHost())
							{
								mw->focusBackendInTree(params.templateBrepBackendIdUtf8);
							}
						}
					}
					else if (m_host)
					{
						m_host->logWarn(QString::fromStdString(
							"ICP result could not be applied to template display: " + applyErr));
					}
				}
				else if (!isFineStage)
				{
					std::string restoreErr;
					if (document_point_cloud_ops::restoreTemplateShapeFromStep(
							page,
							params.templateBrepBackendIdUtf8,
							templateStepPath.toStdString(),
							&restoreErr))
					{
						if (m_host)
						{
							m_host->logWarn(
								QStringLiteral(
									"Registration overlap too poor (maxDev %1 mm); template restored. "
									"Drag the CAD workpiece to align with the scan in 3D view, then retry matching.")
									.arg(result->report.registrationOverlapMaxDevMm, 0, 'f', 1));
						}
					}
					else if (m_host)
					{
						m_host->logWarn(QString::fromStdString(
							"Registration overlap too poor and template restore failed: " + restoreErr));
					}
				}
				else if (m_host)
				{
					m_host->logWarn(
						QStringLiteral(
							"Fine match preview gate not met; coarse CAD pose kept. "
							"Adjust alignment in 3D view or retry coarse match."));
				}

				if (isCoarseStage)
				{
					m_templateBrepAlignCache.doc = doc;
					m_templateBrepAlignCache.scanId = scanBackendIdUtf8;
					m_templateBrepAlignCache.templateId = params.templateBrepBackendIdUtf8;
					m_templateBrepAlignCache.templateWorldMatrixAtRegister = templateBrep->worldMatrix();
					m_templateBrepAlignCache.icpRmseMm = result->report.icpRmseMm;
					m_templateBrepAlignCache.report = result->report;
					registerResult.icpRmseMm = result->report.icpRmseMm;
					if (m_host)
					{
						m_host->logInfo(
							QStringLiteral("Coarse checkpoint saved (valid=%1)")
								.arg(m_templateBrepAlignCache.registrationCheckpoint.valid
										 ? QStringLiteral("yes")
										 : QStringLiteral("no")));
					}
					onFinished(
						true,
						QStringLiteral(
							"Coarse match done (RMSE %1 mm). Run Fine match or adjust CAD in 3D view.")
							.arg(result->report.icpRmseMm, 0, 'f', 2),
						registerResult);
					return;
				}

				if (result->report.icpRmseGatePassed && result->report.registrationPreviewOk)
				{
					m_templateBrepAlignCache.doc = doc;
					m_templateBrepAlignCache.scanId = scanBackendIdUtf8;
					m_templateBrepAlignCache.templateId = params.templateBrepBackendIdUtf8;
					m_templateBrepAlignCache.templateWorldMatrixAtRegister = templateBrep->worldMatrix();
					m_templateBrepAlignCache.icpRmseMm = result->report.icpRmseMm;
					m_templateBrepAlignCache.report = result->report;
				}
			}

			registerResult.icpRmseMm = result->report.icpRmseMm;
			if (!result->registerOk)
			{
				onFinished(false, QString::fromStdString(result->error), registerResult);
				return;
			}
			const bool isFineStage =
				params.registrationStage == PluginPointCloudTemplateBrepRegistrationStage::FineOnly;
			if (!result->report.registrationPreviewOk)
			{
				const QString warn = result->error.empty()
					? QStringLiteral(
						  "Registration overlap too poor; drag CAD workpiece to align with scan in 3D view and retry")
					: QString::fromStdString(result->error);
				if (isFineStage)
				{
					onFinished(
						true,
						QStringLiteral(
							"Fine match gate not met; coarse CAD pose kept. Adjust in 3D view or retry."),
						registerResult);
					return;
				}
				onFinished(false, warn, registerResult);
				return;
			}
			if (!result->report.icpRmseGatePassed)
			{
				const QString warn = result->error.empty()
					? QStringLiteral("Registration preview updated; ICP RMSE above face-update gate")
					: QString::fromStdString(result->error);
				onFinished(true, warn, registerResult);
				return;
			}
			onFinished(true, QString(), registerResult);
		});
}

void PluginPointCloudHostImpl::updateTemplateBrepFromAlignedScan(
	IPluginDocument* doc,
	const std::string& scanBackendIdUtf8,
	const PluginPointCloudTemplateBrepUpdateParams& params,
	PluginPointCloudTemplateBrepUpdateFinishedFn onFinished)
{
	if (!m_host || !onFinished)
	{
		return;
	}
	if (params.templateBrepBackendIdUtf8.empty())
	{
		onFinished(false, QStringLiteral("CAD template B-rep not selected"), {});
		return;
	}
	if (!cacheMatches(doc, scanBackendIdUtf8, params.templateBrepBackendIdUtf8))
	{
		onFinished(
			false,
			QStringLiteral("Run scan-to-template registration first (matching step)"),
			{});
		return;
	}
	if (!m_templateBrepAlignCache.report.icpRmseGatePassed)
	{
		onFinished(
			false,
			QStringLiteral("Last registration RMSE is too high for face update; re-run matching after re-aligning the point cloud"),
			{});
		return;
	}

	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	std::string resolveErr;
	const auto templateBrep = resolveBrep(page, params.templateBrepBackendIdUtf8, &resolveErr);
	if (!templateBrep)
	{
		onFinished(false, QString::fromStdString(resolveErr), {});
		return;
	}
	const QString templateStepPath =
		page->backendSourcePath().value(QString::fromStdString(params.templateBrepBackendIdUtf8));

	struct BrepUpdateResult
	{
		geoalgo::TemplateBrepUpdateResult report;
		std::shared_ptr<BrepBackendData> brep;
		std::string error;
		bool ok = false;
	};
	auto result = std::make_shared<BrepUpdateResult>();
	result->brep = std::make_shared<BrepBackendData>();
	PluginPointCloudTemplateBrepUpdateFinishedFn onFinishedFinal = std::move(onFinished);

	const TemplateBrepAlignCache cacheCopy = m_templateBrepAlignCache;
	const auto scan = document_point_cloud_ops::resolvePointCloud(page, scanBackendIdUtf8, &resolveErr);
	if (!scan)
	{
		onFinished(false, QString::fromStdString(resolveErr), {});
		return;
	}

	m_host->enqueueJob(
		QStringLiteral("Update B-rep faces"),
		[templateBrep, scan, params, templateStepPath, cacheCopy, result](const PluginJobProgressFn& report) {
			report(0.1, QStringLiteral("Updating faces..."));
			const geoalgo::TemplateBrepUpdateParams geoParams = buildTemplateBrepGeoParams(params);
			result->report = cacheCopy.report;
			result->ok = geometry_backend_ops::updateBrepFromAlignedScan(
				*templateBrep,
				*scan,
				geoParams,
				*result->brep,
				result->report,
				&result->error,
				templateStepPath.toStdString());
			report(1.0, QStringLiteral("Done"));
		},
		[m_host = m_host,
			page,
			templateBrep,
			params,
			templateStepPath,
			scanBackendIdUtf8,
			cacheCopy,
			result,
			onFinished = onFinishedFinal](const bool threw, const QString& throwMessage) {
			PluginPointCloudTemplateBrepUpdateResult updateResult;
			if (threw)
			{
				onFinished(false, throwMessage, updateResult);
				return;
			}
			if (!result->ok)
			{
				onFinished(false, QString::fromStdString(result->error), updateResult);
				return;
			}
			if (result->report.updatedFaceCount == 0U)
			{
				updateResult.updatedFaceCount = 0;
				updateResult.skippedBadBboxFaceCount = result->report.skippedBadBboxFaceCount;
				updateResult.globalMaxDeviationMm = result->report.globalMaxDeviationMm;
				updateResult.qualityGatePassed = result->report.qualityPassed;
				fillPerFaceReports(result->report, updateResult.perFace);
				QString zeroFaceError;
				if (result->report.skippedBadBboxFaceCount > 0U)
				{
					zeroFaceError = QStringLiteral(
						"%1 face(s) skipped: model bounds guard. Try fewer faces or fix ICP/face band.")
						.arg(result->report.skippedBadBboxFaceCount);
				}
				else
				{
					zeroFaceError = QStringLiteral(
						"No template faces were updated. Check face selection, ICP alignment, "
						"min points per face, or BSpline adjust threshold.");
				}
				onFinished(false, zeroFaceError, updateResult);
				return;
			}

			// 注册新 B-rep 工件，保留原模板；几何在 ICP 对齐系，位姿在 loadScene 后单独同步
			result->brep->setColor(templateBrep->color());
			const QString templateName = QString::fromStdString(templateBrep->name());
			const QString displayBase = params.displayNameUtf8.empty()
				? (templateName.isEmpty() ? QStringLiteral("BrepUpdated")
										  : templateName + QStringLiteral("_updated"))
				: QString::fromStdString(params.displayNameUtf8);
			const QString displayName = makeUniqueBrepDisplayName(*page, displayBase);
			result->brep->setName(displayName.toStdString());
			geoalgo::clearBrepImportArtifactsCache();

			const QString sourcePath = templateStepPath.isEmpty()
				? QStringLiteral("plugin://template-brep-update")
				: templateStepPath;
			constexpr bool kResetViewToHome = false;
			QString regErr;
			const bool registerOk = cloudsim::host::registerAdoptedBrepAndLoadScene(
				*page,
				result->brep,
				sourcePath,
				QStringLiteral("BrepModel"),
				QString(),
				kResetViewToHome,
				&regErr);
			if (!registerOk)
			{
				onFinished(
					false,
					regErr.isEmpty() ? QStringLiteral("Register updated B-rep failed") : regErr,
					updateResult);
				return;
			}
			std::string alignVisualErr;
			if (!document_point_cloud_ops::alignFaceUpdatedBrepWithTemplateVisual(
					page,
					params.templateBrepBackendIdUtf8,
					result->brep->id(),
					*templateBrep,
					*result->brep,
					&alignVisualErr))
			{
				onFinished(
					false,
					QString::fromStdString(
						alignVisualErr.empty() ? "Updated B-rep visual placement failed" : alignVisualErr),
					updateResult);
				return;
			}
			if (OsgWidget* osg = widgetOsgFromPage(page))
			{
				osg->setBackendObjectVisible(templateBrep->id(), true);
				osg->setBackendObjectVisible(result->brep->id(), true);
				osg->focusCameraOnBackend(result->brep->id());
			}
			if (const auto scan = document_point_cloud_ops::resolvePointCloud(page, scanBackendIdUtf8, nullptr))
			{
				document_point_cloud_ops::commitPointCloudVisual(page, *scan);
				if (OsgWidget* osg = widgetOsgFromPage(page))
				{
					osg->setBackendObjectVisible(scan->id(), true);
				}
			}
			if (m_host)
			{
				m_host->logInfo(
					QStringLiteral("[TemplateBrepUpdate] new B-rep id=%1 name=%2 (template %3 kept)")
						.arg(QString::fromStdString(result->brep->id()))
						.arg(displayName)
						.arg(QString::fromStdString(templateBrep->id())));
			}

			updateResult.newBrepBackendId = result->brep->id();
			updateResult.updatedFaceCount = result->report.updatedFaceCount;
			updateResult.skippedBadBboxFaceCount = result->report.skippedBadBboxFaceCount;
			updateResult.globalMaxDeviationMm = result->report.globalMaxDeviationMm;
			updateResult.qualityGatePassed = result->report.qualityPassed;
			fillPerFaceReports(result->report, updateResult.perFace);
			onFinished(true, QString(), updateResult);
		});
}

// === 网格后处理（1.9.0+） ===

namespace
{

// 网格异步操作通用模式：读 soup → 后台处理 → 注册新 mesh
using MeshMutateFn = std::function<bool(const std::vector<float>& soupIn, std::vector<float>& soupOut, std::string* err)>;

void runMeshMutateJob(
	PluginHostContext* host,
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const QString& jobTitle,
	const PluginMeshCreateOptions& resultOptions,
	MeshMutateFn mutate,
	PluginMeshFinishedFn onFinished)
{
	if (!host || !onFinished)
	{
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	std::string resolveErr;
	const auto mesh = document_point_cloud_ops::resolveMesh(page, backendIdUtf8, &resolveErr);
	if (!mesh)
	{
		onFinished(false, QString::fromStdString(resolveErr), {});
		return;
	}

	struct WorkResult
	{
		std::vector<float> soupOut;
		std::string error;
		bool ok = false;
	};
	auto result = std::make_shared<WorkResult>();
	const std::vector<float> soupIn = mesh->triangleSoup();

	host->enqueueJob(
		jobTitle,
		[soupIn, result, mutate = std::move(mutate)](const PluginJobProgressFn& report) {
			report(0.2, QStringLiteral("Running..."));
			result->ok = mutate(soupIn, result->soupOut, &result->error);
			report(1.0, QStringLiteral("Done"));
		},
		[host, page, resultOptions, result, onFinished = std::move(onFinished)](
			const bool threw, const QString& throwMessage) {
			PluginPointCloudJobResult jobResult;
			if (threw)
			{
				onFinished(false, throwMessage, jobResult);
				return;
			}
			if (!result->ok)
			{
				onFinished(false, QString::fromStdString(result->error), jobResult);
				return;
			}
			auto meshPtr = std::make_shared<MeshBackendData>();
			meshPtr->setTriangleSoup(std::move(result->soupOut));
			std::string regErr;
			jobResult.newBackendId = document_point_cloud_ops::registerReconstructedMesh(
				page, host->mainWindowHost(), meshPtr, resultOptions, &regErr);
			if (jobResult.newBackendId.empty())
			{
				onFinished(false, QString::fromStdString(regErr), jobResult);
				return;
			}
			jobResult.pointCountAfter = meshPtr->triangleSoup().size() / 9;
			onFinished(true, QString(), jobResult);
		});
}

geoalgo::MeshSurfaceReconstructParams buildMeshSurfaceReconstructGeoParams(
	const PluginMeshSurfaceReconstructParams& params)
{
	geoalgo::MeshSurfaceReconstructParams geoParams;
	geoParams.normalSmoothIterations = params.normalSmoothIterations;
	geoParams.featureThresholdC0 = params.featureThresholdC0;
	geoParams.runVcgRepairFirst = params.runVcgRepairFirst;
	geoParams.patchCountHint = params.patchCountHint;
	geoParams.samplesPerPatchEdge = params.samplesPerPatchEdge;
	geoParams.blendStripWidth = params.blendStripWidth;
	geoParams.fairingEpsilon = params.fairingEpsilon;
	geoParams.fairingMaxIterations = params.fairingMaxIterations;
	return geoParams;
}

PluginMeshSurfaceReconstructReport toPluginMeshSurfaceReconstructReport(
	const geoalgo::MeshSurfaceReconstructReport& report,
	const std::string& newBrepBackendId)
{
	PluginMeshSurfaceReconstructReport out;
	out.patchCount = report.patchCount;
	out.junctionCount = report.junctionCount;
	out.maxDeviationMm = report.maxDeviationMm;
	out.globalFairingMetric = report.globalFairingMetric;
	out.normalSmoothGapVolume = report.normalSmoothGapVolume;
	out.c2BlendSucceeded = report.c2BlendSucceeded;
	out.newBrepBackendId = newBrepBackendId;
	return out;
}

void runMeshToBrepJob(
	PluginHostContext* host,
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginMeshSurfaceReconstructParams& params,
	PluginMeshSurfaceReconstructFinishedFn onFinished)
{
	if (!host || !onFinished)
	{
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	std::string resolveErr;
	const auto mesh = document_point_cloud_ops::resolveMesh(page, backendIdUtf8, &resolveErr);
	if (!mesh)
	{
		onFinished(false, QString::fromStdString(resolveErr), {});
		return;
	}

	struct WorkResult
	{
		std::shared_ptr<BrepBackendData> brep;
		geoalgo::MeshSurfaceReconstructReport report;
		std::string error;
		bool ok = false;
	};
	auto result = std::make_shared<WorkResult>();
	result->brep = std::make_shared<BrepBackendData>();
	const std::vector<float> soupIn = mesh->triangleSoup();
	const geoalgo::MeshSurfaceReconstructParams geoParams = buildMeshSurfaceReconstructGeoParams(params);

	host->enqueueJob(
		QStringLiteral("Surface reconstruct"),
		[soupIn, geoParams, result](const PluginJobProgressFn& report) {
			try
			{
				report(0.1, QStringLiteral("Preprocessing mesh..."));
				report(0.4, QStringLiteral("Reconstructing B-rep..."));
				result->ok = geometry_backend_ops::reconstructBrepFromMeshSoup(
					soupIn, geoParams, result->brep, result->report, &result->error);
				report(1.0, QStringLiteral("Done"));
			}
			catch (const std::exception& ex)
			{
				result->ok = false;
				result->error = ex.what();
			}
			catch (...)
			{
				result->ok = false;
				result->error = "surface reconstruction failed with internal error";
			}
		},
		[host,
			page,
			mesh,
			params,
			backendIdUtf8,
			result,
			onFinished = std::move(onFinished)](const bool threw, const QString& throwMessage) {
			if (threw)
			{
				onFinished(false, throwMessage, {});
				return;
			}
			if (!result->ok || !result->brep)
			{
				onFinished(false, QString::fromStdString(result->error), {});
				return;
			}

			result->brep->setColor(mesh->color());
			const QString meshName = QString::fromStdString(mesh->name());
			const QString displayBase = params.displayName.isEmpty()
				? (meshName.isEmpty() ? QStringLiteral("ReconstructedBrep")
									  : meshName + QStringLiteral("_brep"))
				: params.displayName;
			const QString displayName = makeUniqueBrepDisplayName(*page, displayBase);
			result->brep->setName(displayName.toStdString());
			geoalgo::clearBrepImportArtifactsCache();

			constexpr bool kResetViewToHome = false;
			QString regErr;
			const bool registerOk = cloudsim::host::registerAdoptedBrepAndLoadScene(
				*page,
				result->brep,
				QStringLiteral("plugin://pointcloud/surface-reconstruct"),
				QStringLiteral("BrepModel"),
				QString(),
				kResetViewToHome,
				&regErr);
			if (!registerOk)
			{
				onFinished(
					false,
					regErr.isEmpty() ? QStringLiteral("Register reconstructed B-rep failed") : regErr,
					{});
				return;
			}

			std::string alignErr;
			if (!document_point_cloud_ops::inheritBrepVisualPoseFromSourceMesh(
					page, backendIdUtf8, result->brep->id(), *result->brep, &alignErr))
			{
				onFinished(
					false,
					QString::fromStdString(
						alignErr.empty() ? "Reconstructed B-rep visual placement failed" : alignErr),
					{});
				return;
			}

			if (OsgWidget* osg = widgetOsgFromPage(page))
			{
				osg->setBackendObjectVisible(mesh->id(), true);
				osg->focusCameraOnBackend(result->brep->id());
			}
			if (params.selectInTree && host->mainWindowHost())
			{
				host->mainWindowHost()->focusBackendInTreeAfterImport(
					QString::fromStdString(result->brep->id()));
			}
			if (host)
			{
				host->logInfo(
					QStringLiteral("[SurfaceReconstruct] new B-rep id=%1 patches=%2 maxDev=%3 mm")
						.arg(QString::fromStdString(result->brep->id()))
						.arg(result->report.patchCount)
						.arg(result->report.maxDeviationMm, 0, 'f', 4));
			}

			onFinished(
				true,
				QString(),
				toPluginMeshSurfaceReconstructReport(result->report, result->brep->id()));
		});
}

osg::Vec3f meshSoupCenter(const std::vector<float>& soup)
{
	if (soup.size() < 9U)
	{
		return osg::Vec3f(0.0f, 0.0f, 0.0f);
	}
	osg::Vec3f sum(0.0f, 0.0f, 0.0f);
	const std::size_t vertCount = soup.size() / 3U;
	for (std::size_t i = 0; i + 2U < soup.size(); i += 3U)
	{
		sum.x() += soup[i];
		sum.y() += soup[i + 1U];
		sum.z() += soup[i + 2U];
	}
	const float inv = 1.0f / static_cast<float>(vertCount);
	sum *= inv;
	return sum;
}

osg::Matrixd meshBackendWorldMatrix(const MeshBackendData& mesh)
{
	const osg::Vec3f center = meshSoupCenter(mesh.triangleSoup());
	const BackendVec3 p = mesh.pose();
	const BackendVec3 r = mesh.rotation();
	const osg::Vec3d trans(
		static_cast<double>(center.x()) + p.x,
		static_cast<double>(center.y()) + p.y,
		static_cast<double>(center.z()) + p.z);
	const osg::Quat q = engine::eulerDegToQuat(r.x, r.y, r.z);
	return osg::Matrixd::translate(trans) * osg::Matrixd::rotate(q);
}

std::vector<osg::Vec3f> defectFacesToWorldVerts(
	const MeshBackendData& mesh,
	const std::vector<int>& faceIndices)
{
	const std::vector<float>& soup = mesh.triangleSoup();
	const osg::Vec3f center = meshSoupCenter(soup);
	const osg::Matrixd world = meshBackendWorldMatrix(mesh);
	std::vector<osg::Vec3f> verts;
	verts.reserve(faceIndices.size() * 3U);
	for (const int faceIndex : faceIndices)
	{
		if (faceIndex < 0)
		{
			continue;
		}
		const std::size_t base = static_cast<std::size_t>(faceIndex) * 9U;
		if (base + 8U >= soup.size())
		{
			continue;
		}
		for (int v = 0; v < 3; ++v)
		{
			const std::size_t i = base + static_cast<std::size_t>(v) * 3U;
			const osg::Vec3d local(
				static_cast<double>(soup[i]) - static_cast<double>(center.x()),
				static_cast<double>(soup[i + 1U]) - static_cast<double>(center.y()),
				static_cast<double>(soup[i + 2U]) - static_cast<double>(center.z()));
			verts.push_back(osg::Vec3f(world.preMult(local)));
		}
	}
	return verts;
}

PluginMeshDefectReport toPluginDefectReport(
	const std::vector<int>& faceIndices,
	const std::vector<float>& scores,
	const std::vector<int>& kinds,
	const int totalFaces,
	const int defectFaceCount,
	const double defectAreaRatio,
	const int needleCount,
	const int protrusionCount,
	const int boundarySpikeCount)
{
	PluginMeshDefectReport report;
	report.totalFaces = totalFaces;
	report.defectFaceCount = defectFaceCount;
	report.defectAreaRatio = defectAreaRatio;
	report.needleCount = needleCount;
	report.protrusionCount = protrusionCount;
	report.boundarySpikeCount = boundarySpikeCount;
	const std::size_t n = faceIndices.size();
	report.defects.reserve(n);
	for (std::size_t i = 0; i < n; ++i)
	{
		PluginMeshDefectFace face;
		face.faceIndex = faceIndices[i];
		face.kind = kinds[i];
		face.score = static_cast<double>(scores[i]);
		report.defects.push_back(face);
	}
	return report;
}

void runMeshAnalyzeJob(
	PluginHostContext* host,
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginMeshDefectParams& params,
	PluginMeshDefectFinishedFn onFinished)
{
	if (!host || !onFinished)
	{
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	std::string resolveErr;
	const auto mesh = document_point_cloud_ops::resolveMesh(page, backendIdUtf8, &resolveErr);
	if (!mesh)
	{
		onFinished(false, QString::fromStdString(resolveErr), {});
		return;
	}

	struct WorkResult
	{
		std::vector<int> faceIndices;
		std::vector<float> scores;
		std::vector<int> kinds;
		int totalFaces = 0;
		int defectFaceCount = 0;
		double defectAreaRatio = 0.0;
		int needleCount = 0;
		int protrusionCount = 0;
		int boundarySpikeCount = 0;
		std::string error;
		bool ok = false;
	};
	auto result = std::make_shared<WorkResult>();
	const std::vector<float> soupIn = mesh->triangleSoup();

	host->enqueueJob(
		QStringLiteral("Mesh defect analysis"),
		[soupIn, params, result](const PluginJobProgressFn& report) {
			report(0.2, QStringLiteral("Preparing mesh..."));
			report(0.3, QStringLiteral("Computing curvature..."));
			result->ok = point_cloud_backend_ops::analyzeMeshDefects(
				soupIn,
				result->faceIndices,
				result->scores,
				result->kinds,
				result->totalFaces,
				result->defectFaceCount,
				result->defectAreaRatio,
				result->needleCount,
				result->protrusionCount,
				result->boundarySpikeCount,
				params.sensitivity,
				params.minClusterFaces,
				params.detectNeedle,
				params.detectProtrusion,
				params.detectBoundarySpike,
				&result->error);
			report(1.0, QStringLiteral("Done"));
		},
		[host, page, mesh, backendIdUtf8, result, onFinished = std::move(onFinished)](
			const bool threw, const QString& throwMessage) {
			if (threw)
			{
				onFinished(false, throwMessage, {});
				return;
			}
			if (!result->ok)
			{
				onFinished(false, QString::fromStdString(result->error), {});
				return;
			}
			const PluginMeshDefectReport report = toPluginDefectReport(
				result->faceIndices,
				result->scores,
				result->kinds,
				result->totalFaces,
				result->defectFaceCount,
				result->defectAreaRatio,
				result->needleCount,
				result->protrusionCount,
				result->boundarySpikeCount);
			if (OsgWidget* osg = widgetOsgFromPage(page))
			{
				const std::vector<osg::Vec3f> verts = defectFacesToWorldVerts(*mesh, result->faceIndices);
				if (!verts.empty())
				{
					osg->showMeshFaceHighlight(verts);
				}
				else
				{
					osg->hideMeshElementHighlight();
				}
			}
			if (host)
			{
				host->logInfo(
					QStringLiteral("[MeshDefect] %1 faces=%2 defects=%3 (%.1f%%) needle=%4 protrusion=%5 boundary=%6")
						.arg(QString::fromStdString(backendIdUtf8))
						.arg(report.totalFaces)
						.arg(report.defectFaceCount)
						.arg(report.defectAreaRatio * 100.0, 0, 'f', 1)
						.arg(report.needleCount)
						.arg(report.protrusionCount)
						.arg(report.boundarySpikeCount));
			}
			onFinished(true, QString(), report);
		});
}

} // namespace

bool PluginPointCloudHostImpl::queryMeshInfo(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	PluginMeshInfo& out) const
{
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	return document_point_cloud_ops::queryMeshInfo(page, backendIdUtf8, out);
}

void PluginPointCloudHostImpl::simplifyMesh(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginMeshSimplifyParams& params,
	PluginMeshFinishedFn onFinished)
{
	runMeshMutateJob(
		m_host,
		doc,
		backendIdUtf8,
		QStringLiteral("Mesh simplify"),
		params.resultOptions,
		[params](const std::vector<float>& soupIn, std::vector<float>& soupOut, std::string* err) {
			return point_cloud_backend_ops::simplifyMesh(
				soupIn, soupOut, params.targetFaceCount, params.qualityThreshold, err);
		},
		std::move(onFinished));
}

void PluginPointCloudHostImpl::smoothMesh(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginMeshSmoothParams& params,
	PluginMeshFinishedFn onFinished)
{
	runMeshMutateJob(
		m_host,
		doc,
		backendIdUtf8,
		params.useImplicitFairing ? QStringLiteral("Mesh implicit fairing") : QStringLiteral("Mesh Laplacian smooth"),
		params.resultOptions,
		[params](const std::vector<float>& soupIn, std::vector<float>& soupOut, std::string* err) {
			return point_cloud_backend_ops::smoothMesh(
				soupIn, soupOut, params.iterations, params.useImplicitFairing, err);
		},
		std::move(onFinished));
}

void PluginPointCloudHostImpl::repairMesh(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginMeshRepairParams& params,
	PluginMeshFinishedFn onFinished)
{
	runMeshMutateJob(
		m_host,
		doc,
		backendIdUtf8,
		QStringLiteral("Mesh repair"),
		params.resultOptions,
		[params](const std::vector<float>& soupIn, std::vector<float>& soupOut, std::string* err) {
			return point_cloud_backend_ops::repairMesh(
				soupIn, soupOut, params.removeDegenerate, params.removeDuplicate, params.removeNonManifold, err);
		},
		std::move(onFinished));
}

void PluginPointCloudHostImpl::reconstructSurfaceFromMesh(
	IPluginDocument* doc,
	const std::string& meshBackendIdUtf8,
	const PluginMeshSurfaceReconstructParams& params,
	PluginMeshSurfaceReconstructFinishedFn onFinished)
{
	runMeshToBrepJob(m_host, doc, meshBackendIdUtf8, params, std::move(onFinished));
}

void PluginPointCloudHostImpl::remeshMeshIsotropic(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginMeshRemeshParams& params,
	PluginMeshFinishedFn onFinished)
{
	runMeshMutateJob(
		m_host,
		doc,
		backendIdUtf8,
		QStringLiteral("Isotropic remesh"),
		params.resultOptions,
		[params](const std::vector<float>& soupIn, std::vector<float>& soupOut, std::string* err) {
			return point_cloud_backend_ops::remeshMeshIsotropic(
				soupIn, soupOut, params.targetEdgeLengthMm, params.iterations, err);
		},
		std::move(onFinished));
}

void PluginPointCloudHostImpl::analyzeMeshDefects(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginMeshDefectParams& params,
	PluginMeshDefectFinishedFn onFinished)
{
	runMeshAnalyzeJob(m_host, doc, backendIdUtf8, params, std::move(onFinished));
}

void PluginPointCloudHostImpl::clearMeshDefectHighlight(IPluginDocument* doc)
{
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (OsgWidget* osg = widgetOsgFromPage(page))
	{
		osg->hideMeshElementHighlight();
	}
}

void PluginPointCloudHostImpl::pickPolylineFromViewport(
	IPluginDocument* doc,
	PluginPointCloudPolylinePickFinishedFn onFinished)
{
	if (!onFinished)
	{
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page || !m_host)
	{
		onFinished(false, QStringLiteral("No active document"), {});
		return;
	}
	OsgWidget* osg = widgetOsgFromPage(page);
	if (!osg)
	{
		onFinished(false, QStringLiteral("3D viewport unavailable"), {});
		return;
	}

	struct PickState
	{
		bool done = false;
		QMetaObject::Connection connCommitted;
		QMetaObject::Connection connCanceled;
	};
	const auto state = std::make_shared<PickState>();
	const auto complete = [state, osg, onFinished](
							  const bool ok, const QString& err, const PluginPointCloudPolylinePickResult& result) {
		if (state->done)
		{
			return;
		}
		state->done = true;
		QObject::disconnect(state->connCommitted);
		QObject::disconnect(state->connCanceled);
		if (osg)
		{
			osg->setPolylinePickMode(false);
		}
		onFinished(ok, err, result);
	};

	osg->setPolylinePickMode(true);

	state->connCommitted = QObject::connect(
		osg,
		&OsgWidget::polylinePickCommitted,
		m_host,
		[=](const QVector<float>& polylineScreenXy, const QVector<double>& mvpMatrix, const int viewportWidth, const int viewportHeight) {
			PluginPointCloudPolylinePickResult result;
			result.polylineScreenXy.assign(polylineScreenXy.begin(), polylineScreenXy.end());
			const int n = std::min(16, mvpMatrix.size());
			for (int i = 0; i < n; ++i)
			{
				result.mvpMatrix[i] = mvpMatrix[i];
			}
			result.viewportWidth = viewportWidth;
			result.viewportHeight = viewportHeight;
			complete(true, QString(), result);
		});

	state->connCanceled = QObject::connect(osg, &OsgWidget::polylinePickCanceled, m_host, [=]() {
		complete(false, QStringLiteral("Polyline pick canceled"), {});
	});
}

void PluginPointCloudHostImpl::cropPointCloudByPolyline(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginPointCloudCropPolylineParams& params,
	PluginPointCloudFinishedFn onFinished)
{
	if (params.polylineScreenXy.size() < 6U || params.viewportWidth <= 0 || params.viewportHeight <= 0)
	{
		if (onFinished)
		{
			onFinished(false, QStringLiteral("Invalid polyline crop parameters"), {});
		}
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	std::string resolveErr;
	const auto pc = document_point_cloud_ops::resolvePointCloud(page, backendIdUtf8, &resolveErr);
	if (!pc)
	{
		if (onFinished)
		{
			onFinished(false, QString::fromStdString(resolveErr), {});
		}
		return;
	}
	PluginPointCloudCropPolylineParams jobParams = params;
	bool haveModelToWorld = false;
	if (OsgWidget* osg = widgetOsgFromPage(page))
	{
		haveModelToWorld = osg->tryGetBackendPointLocalToWorldMatrix(backendIdUtf8, jobParams.modelToWorld);
	}
	if (!haveModelToWorld)
	{
		PluginMat4 modelToWorld;
		if (!document_point_cloud_ops::buildPointCloudModelToWorld(*pc, modelToWorld))
		{
			if (onFinished)
			{
				onFinished(false, QStringLiteral("Failed to build point cloud world transform"), {});
			}
			return;
		}
		for (int i = 0; i < 16; ++i)
		{
			jobParams.modelToWorld[i] = modelToWorld.v[i];
		}
	}
	runMutateJob(
		m_host,
		doc,
		backendIdUtf8,
		QStringLiteral("Polyline crop"),
		[jobParams](PointCloudBackendData& data, std::string* err) {
			return point_cloud_backend_ops::cropPointCloudByPolyline2D(
				data,
				jobParams.polylineScreenXy,
				jobParams.mvpMatrix,
				jobParams.modelToWorld,
				jobParams.viewportWidth,
				jobParams.viewportHeight,
				jobParams.keepInside,
				err);
		},
		std::move(onFinished));
}
