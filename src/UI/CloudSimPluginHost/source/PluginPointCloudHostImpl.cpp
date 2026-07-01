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
#include <TubularGrinding.h>

#include <osg/Matrixd>
#include <osg/Quat>
#include <osg/Vec3f>

#include <atomic>
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
	geoParams.runIsotropicRemesh = params.runIsotropicRemesh;
	geoParams.remeshTargetEdgeLengthMm = params.remeshTargetEdgeLengthMm;
	geoParams.remeshIterations = params.remeshIterations;
	geoParams.remeshFeatureAngleDeg = params.remeshFeatureAngleDeg;
	geoParams.patchCountHint = params.patchCountHint;
	switch (params.partitionMode)
	{
	case PluginMeshSurfacePartitionMode::HybridNormalCvt:
		geoParams.partitionMode = geoalgo::MeshSurfacePartitionMode::HybridNormalCvt;
		break;
	case PluginMeshSurfacePartitionMode::GeodesicVoronoiV3:
	default:
		geoParams.partitionMode = geoalgo::MeshSurfacePartitionMode::GeodesicVoronoiV3;
		break;
	}
	geoParams.partitionNormalSmoothIters = params.partitionNormalSmoothIters;
	geoParams.featureAnglePercentile = params.featureAnglePercentile;
	geoParams.hybridFeatureAngleDeg = params.hybridFeatureAngleDeg;
	geoParams.hybridClusterMaxIters = params.hybridClusterMaxIters;
	geoParams.hybridSecondarySampleScale = params.hybridSecondarySampleScale;
	geoParams.hybridMergeCosHigh = params.hybridMergeCosHigh;
	geoParams.hybridMergeCosLowBase = params.hybridMergeCosLowBase;
	geoParams.hybridMergeCosLowScale = params.hybridMergeCosLowScale;
	geoParams.hybridSmallRegionRatio = params.hybridSmallRegionRatio;
	geoParams.hybridSmallRegionMin = params.hybridSmallRegionMin;
	geoParams.hybridSmallRegionMax = params.hybridSmallRegionMax;
	geoParams.hybridEnableRegionAdjust = params.hybridEnableRegionAdjust;
	geoParams.hybridCollapseValenceSumMax = params.hybridCollapseValenceSumMax;
	geoParams.hybridCollapseLengthRatio = params.hybridCollapseLengthRatio;
	geoParams.hybridRegionAdjustMaxPasses = params.hybridRegionAdjustMaxPasses;
	geoParams.samplesPerPatchEdge = params.samplesPerPatchEdge;
	geoParams.targetUvSpacingMm = params.targetUvSpacingMm;
	geoParams.minSamplesPerEdge = params.minSamplesPerEdge;
	geoParams.maxSamplesPerEdge = params.maxSamplesPerEdge;
	geoParams.maxFitGridPerEdge = params.maxFitGridPerEdge;
	geoParams.fitUvSpacingMm = params.fitUvSpacingMm;
	geoParams.sampleRateFactor = params.sampleRateFactor;
	geoParams.sampleGridMin = params.sampleGridMin;
	geoParams.sampleGridMax = params.sampleGridMax;
	geoParams.controlPointDensityFactor = params.controlPointDensityFactor;
	geoParams.minControlPointsPerDirection = params.minControlPointsPerDirection;
	geoParams.nurbsDegreeU = params.nurbsDegreeU;
	geoParams.nurbsDegreeV = params.nurbsDegreeV;
	switch (params.fitMode)
	{
	case PluginMeshSurfaceNurbsFitMode::Interpolate:
		geoParams.fitMode = geoalgo::MeshSurfaceNurbsFitMode::Interpolate;
		break;
	case PluginMeshSurfaceNurbsFitMode::ApproxCentripetal:
		geoParams.fitMode = geoalgo::MeshSurfaceNurbsFitMode::ApproxCentripetal;
		break;
	case PluginMeshSurfaceNurbsFitMode::ApproxCentripetalFixedCtrlpts:
		geoParams.fitMode = geoalgo::MeshSurfaceNurbsFitMode::ApproxCentripetalFixedCtrlpts;
		break;
	case PluginMeshSurfaceNurbsFitMode::ApproxFixedCtrlpts:
	default:
		geoParams.fitMode = geoalgo::MeshSurfaceNurbsFitMode::ApproxFixedCtrlpts;
		break;
	}
	geoParams.parameterGridMode = params.parameterGridMode;
	geoParams.fitEvaluationDelta = params.fitEvaluationDelta;
	geoParams.blendStripWidth = params.blendStripWidth;
	geoParams.fairingEpsilon = params.fairingEpsilon;
	geoParams.fairingMaxIterations = params.fairingMaxIterations;
	geoParams.tessellateLinearDeflectionMm = params.tessellateLinearDeflectionMm;
	return geoParams;
}

void copyGeoReportFields(PluginMeshSurfaceReconstructReport& out, const geoalgo::MeshSurfaceReconstructReport& report)
{
	out.patchCount = report.patchCount;
	out.junctionCount = report.junctionCount;
	out.maxDeviationMm = report.maxDeviationMm;
	out.globalFairingMetric = report.globalFairingMetric;
	out.normalSmoothGapVolume = report.normalSmoothGapVolume;
	out.c2BlendSucceeded = report.c2BlendSucceeded;
	out.boundaryBlendPairCount = report.boundaryBlendPairCount;
	out.boundaryBlendCtrlPtCount = report.boundaryBlendCtrlPtCount;
	out.boundaryBlendMaxMoveMm = report.boundaryBlendMaxMoveMm;
	out.junctionBlendAppliedCount = report.junctionBlendAppliedCount;
	out.junctionBlendMaxMoveMm = report.junctionBlendMaxMoveMm;
	out.inputTriangleCount = report.inputTriangleCount;
	out.repairedTriangleCount = report.repairedTriangleCount;
	out.remeshedTriangleCount = report.remeshedTriangleCount;
	out.remeshTargetEdgeLengthUsedMm = report.remeshTargetEdgeLengthUsedMm;
	out.totalSamplePoints = report.totalSamplePoints;
	out.bsplinePatchCount = report.bsplinePatchCount;
	out.nurbsPatchCount = report.nurbsPatchCount;
	out.planeFallbackCount = report.planeFallbackCount;
	out.amrtoHarmonicSampleCount = report.amrtoHarmonicSampleCount;
	out.outputFaceCount = report.outputFaceCount;
	out.avgFacesPerPatch = report.avgFacesPerPatch;
	out.minFacesPerPatch = report.minFacesPerPatch;
	out.maxFacesPerPatch = report.maxFacesPerPatch;
	out.smallPatchCount = report.smallPatchCount;
	out.initialRegionCount = report.initialRegionCount;
	out.quadPatchCount = report.quadPatchCount;
	out.triPatchCount = report.triPatchCount;
	out.pentPatchCount = report.pentPatchCount;
	out.hexPatchCount = report.hexPatchCount;
	out.gridN = report.gridN;
	out.gridNuMax = report.gridNuMax;
	out.gridNvMax = report.gridNvMax;
	out.fitRejectApprox = report.fitRejectApprox;
	out.fitRejectPole = report.fitRejectPole;
	out.fitRejectFitGrid = report.fitRejectFitGrid;
	out.fitRejectFullGrid = report.fitRejectFullGrid;
	out.fitRejectMakeFace = report.fitRejectMakeFace;
}

QString formatSurfaceReconStageSummaryZh(
	const PluginMeshSurfaceReconstructStage stage,
	const PluginMeshSurfaceReconstructReport& report,
	const int samplesPerPatchEdge)
{
	switch (stage)
	{
	case PluginMeshSurfaceReconstructStage::Preprocess:
		if (report.remeshedTriangleCount > 0)
		{
			return QStringLiteral(
				"预处理完成：输入三角 %1 个，修复后 %2 个，均匀化后 %3 个（目标边长 %4 mm），光顺间隙体积 %5")
				.arg(report.inputTriangleCount)
				.arg(report.repairedTriangleCount)
				.arg(report.remeshedTriangleCount)
				.arg(report.remeshTargetEdgeLengthUsedMm, 0, 'f', 4)
				.arg(report.normalSmoothGapVolume, 0, 'g', 6);
		}
		return QStringLiteral("预处理完成：输入三角 %1 个，修复后 %2 个，光顺间隙体积 %3")
			.arg(report.inputTriangleCount)
			.arg(report.repairedTriangleCount)
			.arg(report.normalSmoothGapVolume, 0, 'g', 6);
	case PluginMeshSurfaceReconstructStage::Partition:
		if (report.initialRegionCount > 0 || report.quadPatchCount > 0)
		{
			return QStringLiteral(
				"分块完成：%1 片，交汇 %2 处，三角 %3~%4（平均 %5），初始区 %6，四边 %7 片（场景已按片着色）")
				.arg(report.patchCount)
				.arg(report.junctionCount)
				.arg(report.minFacesPerPatch)
				.arg(report.maxFacesPerPatch)
				.arg(report.avgFacesPerPatch, 0, 'f', 1)
				.arg(report.initialRegionCount)
				.arg(report.quadPatchCount);
		}
		return QStringLiteral("分块完成：%1 片，交汇 %2 处，三角 %3~%4（平均 %5），合并前碎片 %6 个（场景已按片着色）")
			.arg(report.patchCount)
			.arg(report.junctionCount)
			.arg(report.minFacesPerPatch)
			.arg(report.maxFacesPerPatch)
			.arg(report.avgFacesPerPatch, 0, 'f', 1)
			.arg(report.smallPatchCount);
	case PluginMeshSurfaceReconstructStage::Sample:
		if (report.amrtoHarmonicSampleCount > 0)
		{
			return QStringLiteral("栅格采样完成：AMRTO 调和 %1 片，UV 最大 %2×%3，总采样点 %4 个")
				.arg(report.amrtoHarmonicSampleCount)
				.arg(report.gridNuMax > 0 ? report.gridNuMax + 1 : samplesPerPatchEdge + 1)
				.arg(report.gridNvMax > 0 ? report.gridNvMax + 1 : samplesPerPatchEdge + 1)
				.arg(report.totalSamplePoints);
		}
		if (report.gridNuMax > 0 || report.gridNvMax > 0)
		{
			return QStringLiteral("栅格采样完成：UV 最大 %1×%2，总采样点 %3 个")
				.arg(report.gridNuMax > 0 ? report.gridNuMax + 1 : samplesPerPatchEdge + 1)
				.arg(report.gridNvMax > 0 ? report.gridNvMax + 1 : samplesPerPatchEdge + 1)
				.arg(report.totalSamplePoints);
		}
		return QStringLiteral("栅格采样完成：每边 %1 点，总采样点 %2 个")
			.arg(report.gridN > 0 ? report.gridN : samplesPerPatchEdge)
			.arg(report.totalSamplePoints);
	case PluginMeshSurfaceReconstructStage::Fit:
	{
		const int rejectTotal = report.fitRejectApprox + report.fitRejectPole + report.fitRejectFitGrid
			+ report.fitRejectFullGrid + report.fitRejectMakeFace;
		if (rejectTotal > 0)
		{
			return QStringLiteral(
				"拟合完成：NURBS %1 片，三角回退 %2 片；拒因 approxFail=%3 pole=%4 fitGrid=%5 fullGrid=%6 makeFace=%7")
				.arg(report.nurbsPatchCount > 0 ? report.nurbsPatchCount : report.bsplinePatchCount)
				.arg(report.planeFallbackCount)
				.arg(report.fitRejectApprox)
				.arg(report.fitRejectPole)
				.arg(report.fitRejectFitGrid)
				.arg(report.fitRejectFullGrid)
				.arg(report.fitRejectMakeFace);
		}
		return QStringLiteral("拟合完成：NURBS %1 片，三角回退 %2 片")
			.arg(report.nurbsPatchCount > 0 ? report.nurbsPatchCount : report.bsplinePatchCount)
			.arg(report.planeFallbackCount);
	}
	case PluginMeshSurfaceReconstructStage::BoundaryBlend:
		if (report.c2BlendSucceeded)
		{
			return QStringLiteral("边界混合完成：%1 对相邻片，%2 个控制点，最大移动 %3 mm")
				.arg(report.boundaryBlendPairCount)
				.arg(report.boundaryBlendCtrlPtCount)
				.arg(report.boundaryBlendMaxMoveMm, 0, 'f', 4);
		}
		return QStringLiteral("边界混合完成：失败（%1 对相邻片，%2 个控制点，最大移动 %3 mm）")
			.arg(report.boundaryBlendPairCount)
			.arg(report.boundaryBlendCtrlPtCount)
			.arg(report.boundaryBlendMaxMoveMm, 0, 'f', 4);
	case PluginMeshSurfaceReconstructStage::JunctionBlend:
		return QStringLiteral("交汇混合完成：处理交汇 %1/%2 处，最大移动 %3 mm")
			.arg(report.junctionBlendAppliedCount)
			.arg(report.junctionCount)
			.arg(report.junctionBlendMaxMoveMm, 0, 'f', 4);
	case PluginMeshSurfaceReconstructStage::Fair:
		return QStringLiteral("光顺完成：全局指标 %1").arg(report.globalFairingMetric, 0, 'g', 6);
	case PluginMeshSurfaceReconstructStage::Assemble:
		return QStringLiteral("装配完成：最大偏差 %1 mm，新 B-rep id=%2，面数 %3")
			.arg(report.maxDeviationMm, 0, 'f', 4)
			.arg(QString::fromStdString(report.newBrepBackendId))
			.arg(report.outputFaceCount);
	default:
		return QStringLiteral("会话已重置");
	}
}

QString translateSurfaceReconErrorZh(const QString& error)
{
	if (error.contains(QStringLiteral("stage out of order"), Qt::CaseInsensitive))
	{
		return QStringLiteral("请先完成上一阶段");
	}
	if (error.contains(QStringLiteral("mesh soup too small"), Qt::CaseInsensitive))
	{
		return QStringLiteral("网格三角数过少");
	}
	if (error.contains(QStringLiteral("Missing Component"), Qt::CaseInsensitive)
		|| error.contains(QStringLiteral("missing required VCG component"), Qt::CaseInsensitive))
	{
		return QStringLiteral("均匀化重网格失败：网格数据结构不兼容");
	}
	if (error.contains(QStringLiteral("isotropic remesh"), Qt::CaseInsensitive))
	{
		return QStringLiteral("均匀化重网格失败：%1").arg(error);
	}
	return error;
}

geoalgo::MeshSurfaceReconstructStage mapPluginStageToGeo(const PluginMeshSurfaceReconstructStage stage)
{
	switch (stage)
	{
	case PluginMeshSurfaceReconstructStage::Partition:
		return geoalgo::MeshSurfaceReconstructStage::Partition;
	case PluginMeshSurfaceReconstructStage::Sample:
		return geoalgo::MeshSurfaceReconstructStage::Sample;
	case PluginMeshSurfaceReconstructStage::Fit:
		return geoalgo::MeshSurfaceReconstructStage::Fit;
	case PluginMeshSurfaceReconstructStage::BoundaryBlend:
		return geoalgo::MeshSurfaceReconstructStage::BoundaryBlend;
	case PluginMeshSurfaceReconstructStage::JunctionBlend:
		return geoalgo::MeshSurfaceReconstructStage::JunctionBlend;
	case PluginMeshSurfaceReconstructStage::Fair:
		return geoalgo::MeshSurfaceReconstructStage::Fair;
	case PluginMeshSurfaceReconstructStage::Assemble:
		return geoalgo::MeshSurfaceReconstructStage::Assemble;
	default:
		return geoalgo::MeshSurfaceReconstructStage::None;
	}
}

bool isNextPluginStage(
	const PluginMeshSurfaceReconstructStage lastCompleted,
	const PluginMeshSurfaceReconstructStage want)
{
	if (want == PluginMeshSurfaceReconstructStage::Preprocess
		&& lastCompleted == PluginMeshSurfaceReconstructStage::None)
	{
		return true;
	}
	return static_cast<int>(want) == static_cast<int>(lastCompleted) + 1;
}

std::string makeSurfaceReconSessionId()
{
	static std::atomic<uint64_t> counter{0U};
	return "sr_" + std::to_string(counter.fetch_add(1U) + 1U);
}

std::string makeTubularGrindingSessionId()
{
	static std::atomic<uint64_t> counter{0U};
	return "tg_" + std::to_string(counter.fetch_add(1U) + 1U);
}

bool isNextTubularGrindingStage(
	const PluginTubularGrindingStage lastCompleted,
	const PluginTubularGrindingStage want)
{
	// Centerline is now the first stage (Segment stage has been removed)
	if (want == PluginTubularGrindingStage::Centerline
		&& lastCompleted == PluginTubularGrindingStage::None)
	{
		return true;
	}
	return static_cast<int>(want) == static_cast<int>(lastCompleted) + 1
		|| want == lastCompleted;
}

geoalgo::TubularGrindingParams buildTubularGrindingGeoParams(const PluginTubularGrindingParams& params)
{
	geoalgo::TubularGrindingParams geoParams;
	geoParams.ringCenterClusterEpsMm = params.ringCenterClusterEpsMm;
	geoParams.minRingFaces = params.minRingFaces;
	geoParams.regionGrowAxisAngleDeg = params.regionGrowAxisAngleDeg;
	geoParams.axisMergeAngleDeg = params.axisMergeAngleDeg;
	geoParams.junctionAxisSpreadDeg = params.junctionAxisSpreadDeg;
	geoParams.minSegmentFaces = params.minSegmentFaces;
	geoParams.ringRayConvergenceEpsMm = params.ringRayConvergenceEpsMm;
	geoParams.faceNormalAxisLengthMm = params.faceNormalAxisLengthMm;
	geoParams.sectionSpacingMm = params.sectionSpacingMm;
	geoParams.minSectionPoints = params.minSectionPoints;
	geoParams.helicalCoils = params.helicalCoils;
	geoParams.circumferentialRings = params.circumferentialRings;
	geoParams.axialMeridians = params.axialMeridians;
	geoParams.zigzagPasses = params.zigzagPasses;
	geoParams.projectionMaxDistMm = params.projectionMaxDistMm;
	geoParams.centerlineIterations = params.centerlineIterations;
	geoParams.laplacianLambda = params.laplacianLambda;
	geoParams.laplacianAttraction = params.laplacianAttraction;
	geoParams.laplacianKNeighbors = params.laplacianKNeighbors;
	geoParams.otSampleRate = params.otSampleRate;
	geoParams.otCostBeta = params.otCostBeta;
	geoParams.otcPreSteps = params.otcPreSteps;
	geoParams.otcOuterLoops = params.otcOuterLoops;
	geoParams.otLcOuterMaxIters = params.otLcOuterMaxIters;
	geoParams.minRootsBySamples = params.minRootsBySamples;
	geoParams.pointCloudKnnK = params.pointCloudKnnK;
	switch (params.centerlineMethod)
	{
	case PluginTubularGrindingCenterlineMethod::OtLc:
		geoParams.centerlineMethod = geoalgo::TubularGrindingCenterlineMethod::OtLc;
		break;
	default:
		geoParams.centerlineMethod = geoalgo::TubularGrindingCenterlineMethod::Laplacian;
		break;
	}
	switch (params.templateKind)
	{
	case PluginTubularGrindingTemplateKind::Helical:
		geoParams.templateKind = geoalgo::TubularGrindingTemplateKind::Helical;
		break;
	case PluginTubularGrindingTemplateKind::Circumferential:
		geoParams.templateKind = geoalgo::TubularGrindingTemplateKind::Circumferential;
		break;
	case PluginTubularGrindingTemplateKind::AxialParallel:
		geoParams.templateKind = geoalgo::TubularGrindingTemplateKind::AxialParallel;
		break;
	case PluginTubularGrindingTemplateKind::Zigzag:
		geoParams.templateKind = geoalgo::TubularGrindingTemplateKind::Zigzag;
		break;
	default:
		geoParams.templateKind = geoalgo::TubularGrindingTemplateKind::Auto;
		break;
	}
	return geoParams;
}

geoalgo::TubularGrindingStage mapPluginTubularStageToGeo(const PluginTubularGrindingStage stage)
{
	switch (stage)
	{
	case PluginTubularGrindingStage::Segment:
		return geoalgo::TubularGrindingStage::Segment;
	case PluginTubularGrindingStage::Centerline:
		return geoalgo::TubularGrindingStage::Centerline;
	case PluginTubularGrindingStage::TemplatePoints:
		return geoalgo::TubularGrindingStage::TemplatePoints;
	case PluginTubularGrindingStage::Project:
		return geoalgo::TubularGrindingStage::Project;
	default:
		return geoalgo::TubularGrindingStage::None;
	}
}

QString formatTubularGrindingStageSummaryZh(
	const PluginTubularGrindingStage stage,
	const PluginTubularGrindingReport& report)
{
	switch (stage)
	{
	case PluginTubularGrindingStage::Segment:
		return QStringLiteral("管段分割完成：%1 个管段，%2 个环，交汇面 %3，环心簇 %4")
			.arg(report.pipeCount)
			.arg(report.ringCount)
			.arg(report.junctionFaceCount)
			.arg(report.regionCountBeforeFilter);
	case PluginTubularGrindingStage::Centerline:
	{
		QString summary = QStringLiteral("中心线提取完成：%1 个采样点，%2 个截面拟合失败")
			.arg(report.centerlinePointCount)
			.arg(report.sectionFitFailCount);
		if (report.centerlineOtLcExtraction)
		{
			switch (report.centerlineOtPathKind)
			{
			case 1:
				summary += QStringLiteral("；提线路径：OT 分簇链");
				break;
			case 0:
				summary += QStringLiteral("；提线路径：收缩点云截面质心");
				break;
			default:
				summary += QStringLiteral("；提线路径：有序折线兜底");
				break;
			}
			summary += QStringLiteral("；sample 根 %1，边 %2，连通分量 %3")
				.arg(report.centerlineOtRootCount)
				.arg(report.centerlineOtEdgeCount)
				.arg(report.centerlineOtComponentCount);
			if (report.centerlineOtKnnFallbackEdges)
			{
				summary += QStringLiteral("（KNN 补边）");
			}
		}
		return summary;
	}
	case PluginTubularGrindingStage::TemplatePoints:
		return QStringLiteral("模板点位生成完成：%1 个点").arg(report.templatePointCount);
	case PluginTubularGrindingStage::Project:
		return QStringLiteral("表面投影完成：%1 个点，命中率 %2%")
			.arg(report.projectedPointCount)
			.arg(report.projectionHitRate * 100.0, 0, 'f', 1);
	default:
		return QString();
	}
}

bool registerTubularGrindingColoredMeshFromSoup(
	PluginHostContext* host,
	cloudsim::host::DocumentHost* page,
	const std::shared_ptr<MeshBackendData>& sourceMesh,
	const std::vector<float>& soup,
	const std::vector<float>& rgbPerVertex,
	const QString& displaySuffix,
	std::string& outBackendId,
	std::string* errMsg)
{
	if (!page || !sourceMesh || soup.empty() || rgbPerVertex.size() != soup.size())
	{
		if (errMsg)
		{
			*errMsg = "invalid tubular grinding colored mesh";
		}
		return false;
	}
	auto meshPtr = std::make_shared<MeshBackendData>();
	meshPtr->setTriangleSoupWithVertexColors(soup, rgbPerVertex);
	PluginMeshCreateOptions options;
	const QString baseName = QString::fromStdString(sourceMesh->name());
	options.displayName =
		baseName.isEmpty() ? displaySuffix : baseName + displaySuffix;
	options.selectInTree = true;
	options.sourcePath = QStringLiteral("plugin://pointcloud/tubular-grinding-segment");
	outBackendId = document_point_cloud_ops::registerReconstructedMesh(
		page, host ? host->mainWindowHost() : nullptr, meshPtr, options, errMsg);
	return !outBackendId.empty();
}

bool registerTubularGrindingNormalAxisLines(
	PluginHostContext* host,
	cloudsim::host::DocumentHost* page,
	const std::shared_ptr<MeshBackendData>& sourceMesh,
	const std::vector<float>& lineXyz,
	std::string& outBackendId,
	std::string* errMsg)
{
	if (!page || !sourceMesh || lineXyz.size() < 6U || (lineXyz.size() % 6U) != 0U)
	{
		if (errMsg)
		{
			*errMsg = "invalid tubular grinding normal axis lines";
		}
		return false;
	}
	auto meshPtr = std::make_shared<MeshBackendData>();
	meshPtr->setOverlayLineSegments(lineXyz);
	BackendColor c;
	c.r = 0.25f;
	c.g = 0.85f;
	c.b = 1.0f;
	c.a = 1.0f;
	meshPtr->setColor(c);
	meshPtr->setPose(sourceMesh->pose());
	meshPtr->setRotation(sourceMesh->rotation());
	PluginMeshCreateOptions options;
	const QString baseName = QString::fromStdString(sourceMesh->name());
	options.displayName = baseName.isEmpty() ? QStringLiteral("_法向") : baseName + QStringLiteral("_法向");
	options.selectInTree = false;
	options.sourcePath = QStringLiteral("plugin://pointcloud/tubular-grinding-normals");
	outBackendId = document_point_cloud_ops::registerReconstructedMesh(
		page, host ? host->mainWindowHost() : nullptr, meshPtr, options, errMsg);
	return !outBackendId.empty();
}

bool registerTubularGrindingLocalAxisLines(
	PluginHostContext* host,
	cloudsim::host::DocumentHost* page,
	const std::shared_ptr<MeshBackendData>& sourceMesh,
	const std::vector<float>& lineXyz,
	std::string& outBackendId,
	std::string* errMsg)
{
	if (!page || !sourceMesh || lineXyz.size() < 6U || (lineXyz.size() % 6U) != 0U)
	{
		if (errMsg)
		{
			*errMsg = "invalid tubular grinding local axis lines";
		}
		return false;
	}
	auto meshPtr = std::make_shared<MeshBackendData>();
	meshPtr->setOverlayLineSegments(lineXyz);
	BackendColor c;
	c.r = 1.0f;
	c.g = 0.6f;
	c.b = 0.1f;
	c.a = 1.0f;
	meshPtr->setColor(c);
	meshPtr->setPose(sourceMesh->pose());
	meshPtr->setRotation(sourceMesh->rotation());
	PluginMeshCreateOptions options;
	const QString baseName = QString::fromStdString(sourceMesh->name());
	options.displayName = baseName.isEmpty() ? QStringLiteral("_局部轴线") : baseName + QStringLiteral("_局部轴线");
	options.selectInTree = false;
	options.sourcePath = QStringLiteral("plugin://pointcloud/tubular-grinding-local-axes");
	outBackendId = document_point_cloud_ops::registerReconstructedMesh(
		page, host ? host->mainWindowHost() : nullptr, meshPtr, options, errMsg);
	return !outBackendId.empty();
}

bool registerTubularGrindingCenterlineLines(
	PluginHostContext* host,
	cloudsim::host::DocumentHost* page,
	const std::shared_ptr<MeshBackendData>& sourceMesh,
	const std::vector<float>& polylineXyz,
	std::string& outBackendId,
	std::string* errMsg)
{
	if (!page || !sourceMesh || polylineXyz.size() < 6U || (polylineXyz.size() % 3U) != 0U)
	{
		if (errMsg)
		{
			*errMsg = "invalid tubular grinding centerline polyline";
		}
		return false;
	}
	std::vector<float> lineXyz;
	lineXyz.reserve((polylineXyz.size() / 3U - 1U) * 6U);
	for (std::size_t i = 0; i + 5U < polylineXyz.size(); i += 3U)
	{
		lineXyz.push_back(polylineXyz[i]);
		lineXyz.push_back(polylineXyz[i + 1U]);
		lineXyz.push_back(polylineXyz[i + 2U]);
		lineXyz.push_back(polylineXyz[i + 3U]);
		lineXyz.push_back(polylineXyz[i + 4U]);
		lineXyz.push_back(polylineXyz[i + 5U]);
	}
	auto meshPtr = std::make_shared<MeshBackendData>();
	meshPtr->setOverlayLineSegments(std::move(lineXyz));
	meshPtr->setOverlayLinesAlwaysOnTop(true);
	BackendColor c;
	c.r = 1.0f;
	c.g = 0.15f;
	c.b = 0.1f;
	c.a = 1.0f;
	meshPtr->setColor(c);
	meshPtr->setPose(sourceMesh->pose());
	meshPtr->setRotation(sourceMesh->rotation());
	PluginMeshCreateOptions options;
	const QString baseName = QString::fromStdString(sourceMesh->name());
	options.displayName = baseName.isEmpty() ? QStringLiteral("中心线") : baseName + QStringLiteral("_中心线");
	options.selectInTree = false;
	options.sourcePath = QStringLiteral("plugin://pointcloud/tubular-grinding-centerline");
	outBackendId = document_point_cloud_ops::registerReconstructedMesh(
		page, host ? host->mainWindowHost() : nullptr, meshPtr, options, errMsg);
	return !outBackendId.empty();
}

bool registerTubularGrindingCenterlineLines(
	PluginHostContext* host,
	cloudsim::host::DocumentHost* page,
	const std::shared_ptr<PointCloudBackendData>& sourcePointCloud,
	const std::vector<float>& polylineXyz,
	std::string& outBackendId,
	std::string* errMsg)
{
	if (!page || !sourcePointCloud || polylineXyz.size() < 6U || (polylineXyz.size() % 3U) != 0U)
	{
		if (errMsg)
		{
			*errMsg = "invalid tubular grinding centerline polyline";
		}
		return false;
	}
	std::vector<float> lineXyz;
	lineXyz.reserve((polylineXyz.size() / 3U - 1U) * 6U);
	for (std::size_t i = 0; i + 5U < polylineXyz.size(); i += 3U)
	{
		lineXyz.push_back(polylineXyz[i]);
		lineXyz.push_back(polylineXyz[i + 1U]);
		lineXyz.push_back(polylineXyz[i + 2U]);
		lineXyz.push_back(polylineXyz[i + 3U]);
		lineXyz.push_back(polylineXyz[i + 4U]);
		lineXyz.push_back(polylineXyz[i + 5U]);
	}
	auto meshPtr = std::make_shared<MeshBackendData>();
	meshPtr->setOverlayLineSegments(std::move(lineXyz));
	meshPtr->setOverlayLinesAlwaysOnTop(true);
	BackendColor c;
	c.r = 1.0f;
	c.g = 0.15f;
	c.b = 0.1f;
	c.a = 1.0f;
	meshPtr->setColor(c);
	meshPtr->setPose(sourcePointCloud->pose());
	meshPtr->setRotation(sourcePointCloud->rotation());
	PluginMeshCreateOptions options;
	const QString baseName = QString::fromStdString(sourcePointCloud->name());
	options.displayName = baseName.isEmpty() ? QStringLiteral("中心线") : baseName + QStringLiteral("_中心线");
	options.selectInTree = false;
	options.sourcePath = QStringLiteral("plugin://pointcloud/tubular-grinding-centerline");
	outBackendId = document_point_cloud_ops::registerReconstructedMesh(
		page, host ? host->mainWindowHost() : nullptr, meshPtr, options, errMsg);
	return !outBackendId.empty();
}

bool registerTubularGrindingPcaAxisArrowLines(
	PluginHostContext* host,
	cloudsim::host::DocumentHost* page,
	const std::shared_ptr<MeshBackendData>& sourceMesh,
	const std::vector<float>& lineXyz,
	std::string& outBackendId,
	std::string* errMsg)
{
	if (!page || !sourceMesh || lineXyz.size() < 6U || (lineXyz.size() % 6U) != 0U)
	{
		if (errMsg)
		{
			*errMsg = "invalid tubular grinding PCA axis arrow lines";
		}
		return false;
	}
	auto meshPtr = std::make_shared<MeshBackendData>();
	meshPtr->setOverlayLineSegments(lineXyz);
	meshPtr->setOverlayLinesAlwaysOnTop(true);
	BackendColor c;
	c.r = 0.15f;
	c.g = 0.95f;
	c.b = 0.25f;
	c.a = 1.0f;
	meshPtr->setColor(c);
	meshPtr->setPose(sourceMesh->pose());
	meshPtr->setRotation(sourceMesh->rotation());
	PluginMeshCreateOptions options;
	const QString baseName = QString::fromStdString(sourceMesh->name());
	options.displayName = baseName.isEmpty() ? QStringLiteral("PCA轴") : baseName + QStringLiteral("_PCA轴");
	options.selectInTree = false;
	options.sourcePath = QStringLiteral("plugin://pointcloud/tubular-grinding-pca-axis");
	outBackendId = document_point_cloud_ops::registerReconstructedMesh(
		page, host ? host->mainWindowHost() : nullptr, meshPtr, options, errMsg);
	return !outBackendId.empty();
}

bool registerTubularGrindingPcaAxisArrowLines(
	PluginHostContext* host,
	cloudsim::host::DocumentHost* page,
	const std::shared_ptr<PointCloudBackendData>& sourcePointCloud,
	const std::vector<float>& lineXyz,
	std::string& outBackendId,
	std::string* errMsg)
{
	if (!page || !sourcePointCloud || lineXyz.size() < 6U || (lineXyz.size() % 6U) != 0U)
	{
		if (errMsg)
		{
			*errMsg = "invalid tubular grinding PCA axis arrow lines";
		}
		return false;
	}
	auto meshPtr = std::make_shared<MeshBackendData>();
	meshPtr->setOverlayLineSegments(lineXyz);
	meshPtr->setOverlayLinesAlwaysOnTop(true);
	BackendColor c;
	c.r = 0.15f;
	c.g = 0.95f;
	c.b = 0.25f;
	c.a = 1.0f;
	meshPtr->setColor(c);
	meshPtr->setPose(sourcePointCloud->pose());
	meshPtr->setRotation(sourcePointCloud->rotation());
	PluginMeshCreateOptions options;
	const QString baseName = QString::fromStdString(sourcePointCloud->name());
	options.displayName = baseName.isEmpty() ? QStringLiteral("PCA轴") : baseName + QStringLiteral("_PCA轴");
	options.selectInTree = false;
	options.sourcePath = QStringLiteral("plugin://pointcloud/tubular-grinding-pca-axis");
	outBackendId = document_point_cloud_ops::registerReconstructedMesh(
		page, host ? host->mainWindowHost() : nullptr, meshPtr, options, errMsg);
	return !outBackendId.empty();
}

bool registerTubularGrindingPointCloud(
	cloudsim::host::DocumentHost* page,
	const std::shared_ptr<MeshBackendData>& sourceMesh,
	const QString& displaySuffix,
	const QString& sourcePath,
	std::vector<float> xyz,
	std::vector<float> rgba,
	std::string& outBackendId,
	QString* errMsg)
{
	if (!page || !sourceMesh || xyz.empty())
	{
		if (errMsg)
		{
			*errMsg = QStringLiteral("invalid tubular grinding point cloud");
		}
		return false;
	}
	auto pcPtr = std::make_shared<PointCloudBackendData>();
	const QString baseName = QString::fromStdString(sourceMesh->name());
	pcPtr->setName((baseName.isEmpty() ? displaySuffix.mid(1) : baseName + displaySuffix).toStdString());
	pcPtr->setPointBuffers(std::move(xyz), std::move(rgba));
	pcPtr->setPose(sourceMesh->pose());
	pcPtr->setRotation(sourceMesh->rotation());
	cloudsim::host::AdoptPointCloudOptions adoptOpt;
	adoptOpt.sourcePath = sourcePath;
	adoptOpt.resetViewToHome = false;
	const cloudsim::host::AdoptRegistrationResult adopted =
		cloudsim::host::registerAdoptedPointCloud(*page, pcPtr, adoptOpt, errMsg);
	if (!adopted.ok)
	{
		return false;
	}
	outBackendId = adopted.backendId.toStdString();
	return true;
}

bool registerTubularGrindingPointCloud(
	cloudsim::host::DocumentHost* page,
	const std::shared_ptr<PointCloudBackendData>& sourcePointCloud,
	const QString& displaySuffix,
	const QString& sourcePath,
	std::vector<float> xyz,
	std::vector<float> rgba,
	std::string& outBackendId,
	QString* errMsg)
{
	if (!page || !sourcePointCloud || xyz.empty())
	{
		if (errMsg)
		{
			*errMsg = QStringLiteral("invalid tubular grinding point cloud");
		}
		return false;
	}
	auto pcPtr = std::make_shared<PointCloudBackendData>();
	const QString baseName = QString::fromStdString(sourcePointCloud->name());
	pcPtr->setName((baseName.isEmpty() ? displaySuffix.mid(1) : baseName + displaySuffix).toStdString());
	pcPtr->setPointBuffers(std::move(xyz), std::move(rgba));
	pcPtr->setPose(sourcePointCloud->pose());
	pcPtr->setRotation(sourcePointCloud->rotation());
	cloudsim::host::AdoptPointCloudOptions adoptOpt;
	adoptOpt.sourcePath = sourcePath;
	adoptOpt.resetViewToHome = false;
	const cloudsim::host::AdoptRegistrationResult adopted =
		cloudsim::host::registerAdoptedPointCloud(*page, pcPtr, adoptOpt, errMsg);
	if (!adopted.ok)
	{
		return false;
	}
	outBackendId = adopted.backendId.toStdString();
	return true;
}

PluginMeshSurfaceReconstructReport toPluginMeshSurfaceReconstructReport(
	const geoalgo::MeshSurfaceReconstructReport& report,
	const std::string& newBrepBackendId,
	const PluginMeshSurfaceReconstructStage lastStage)
{
	PluginMeshSurfaceReconstructReport out;
	copyGeoReportFields(out, report);
	out.lastCompletedStage = lastStage;
	out.newBrepBackendId = newBrepBackendId;
	return out;
}

bool registerPreprocessedMeshFromSoup(
	PluginHostContext* host,
	cloudsim::host::DocumentHost* page,
	const std::shared_ptr<MeshBackendData>& sourceMesh,
	const std::vector<float>& soup,
	const PluginMeshSurfaceReconstructParams& params,
	std::string& outBackendId,
	std::string* errMsg)
{
	if (!page || !sourceMesh)
	{
		if (errMsg)
		{
			*errMsg = "invalid mesh session";
		}
		return false;
	}
	auto meshPtr = std::make_shared<MeshBackendData>();
	meshPtr->setTriangleSoup(soup);
	meshPtr->setColor(sourceMesh->color());
	PluginMeshCreateOptions options;
	const QString baseName = QString::fromStdString(sourceMesh->name());
	options.displayName =
		baseName.isEmpty() ? QStringLiteral("预处理后") : baseName + QStringLiteral("_预处理后");
	options.selectInTree = false;
	options.sourcePath = QStringLiteral("plugin://pointcloud/surface-reconstruct-preprocess");
	outBackendId = document_point_cloud_ops::registerReconstructedMesh(
		page, host ? host->mainWindowHost() : nullptr, meshPtr, options, errMsg);
	return !outBackendId.empty();
}

bool registerPartitionColoredMeshFromSoup(
	PluginHostContext* host,
	cloudsim::host::DocumentHost* page,
	const std::shared_ptr<MeshBackendData>& sourceMesh,
	const std::vector<float>& soup,
	const std::vector<float>& rgbPerVertex,
	std::string& outBackendId,
	std::string* errMsg)
{
	if (!page || !sourceMesh || soup.empty() || rgbPerVertex.size() != soup.size())
	{
		if (errMsg)
		{
			*errMsg = "invalid partition colored mesh";
		}
		return false;
	}
	auto meshPtr = std::make_shared<MeshBackendData>();
	meshPtr->setTriangleSoupWithVertexColors(soup, rgbPerVertex);
	PluginMeshCreateOptions options;
	const QString baseName = QString::fromStdString(sourceMesh->name());
	options.displayName =
		baseName.isEmpty() ? QStringLiteral("分块着色") : baseName + QStringLiteral("_分块着色");
	options.selectInTree = true;
	options.sourcePath = QStringLiteral("plugin://pointcloud/surface-reconstruct-partition");
	outBackendId = document_point_cloud_ops::registerReconstructedMesh(
		page, host ? host->mainWindowHost() : nullptr, meshPtr, options, errMsg);
	return !outBackendId.empty();
}

bool registerReconstructedBrepFromShape(
	PluginHostContext* host,
	cloudsim::host::DocumentHost* page,
	const std::shared_ptr<MeshBackendData>& sourceMesh,
	const std::string& sourceMeshBackendId,
	const std::shared_ptr<BrepBackendData>& brep,
	const PluginMeshSurfaceReconstructParams& params,
	std::string* errMsg)
{
	if (!page || !sourceMesh || !brep)
	{
		if (errMsg)
		{
			*errMsg = "invalid brep registration context";
		}
		return false;
	}
	brep->setColor(sourceMesh->color());
	const QString meshName = QString::fromStdString(sourceMesh->name());
	const QString displayBase = params.displayName.isEmpty()
		? (meshName.isEmpty() ? QStringLiteral("ReconstructedBrep") : meshName + QStringLiteral("_brep"))
		: params.displayName;
	const QString displayName = makeUniqueBrepDisplayName(*page, displayBase);
	brep->setName(displayName.toStdString());
	geoalgo::clearBrepImportArtifactsCache();

	constexpr bool kResetViewToHome = false;
	QString regErr;
	const bool registerOk = cloudsim::host::registerAdoptedBrepAndLoadScene(
		*page,
		brep,
		QStringLiteral("plugin://pointcloud/surface-reconstruct"),
		QStringLiteral("BrepModel"),
		QString(),
		kResetViewToHome,
		&regErr);
	if (!registerOk)
	{
		if (errMsg)
		{
			*errMsg = regErr.toStdString();
		}
		return false;
	}

	std::string alignErr;
	if (!document_point_cloud_ops::inheritBrepVisualPoseFromSourceMesh(
			page, sourceMeshBackendId, brep->id(), *brep, &alignErr))
	{
		if (errMsg)
		{
			*errMsg = alignErr.empty() ? "Reconstructed B-rep visual placement failed" : alignErr;
		}
		return false;
	}

	if (OsgWidget* osg = widgetOsgFromPage(page))
	{
		osg->setBackendObjectVisible(sourceMesh->id(), true);
		osg->focusCameraOnBackend(brep->id());
	}
	if (params.selectInTree && host && host->mainWindowHost())
	{
		host->mainWindowHost()->focusBackendInTreeAfterImport(QString::fromStdString(brep->id()));
	}
	return true;
}

bool registerFitPreviewBrepFromShape(
	PluginHostContext* host,
	cloudsim::host::DocumentHost* page,
	const std::shared_ptr<MeshBackendData>& sourceMesh,
	const std::string& sourceMeshBackendId,
	const std::shared_ptr<BrepBackendData>& brep,
	std::string* errMsg,
	const QString& displayNameSuffix = QStringLiteral("拟合曲面"),
	const QString& sourceUri = QStringLiteral("plugin://pointcloud/surface-reconstruct-fit"))
{
	if (!page || !sourceMesh || !brep)
	{
		if (errMsg)
		{
			*errMsg = "invalid fit preview brep context";
		}
		return false;
	}
	brep->setColor(sourceMesh->color());
	const QString meshName = QString::fromStdString(sourceMesh->name());
	const QString displayBase =
		meshName.isEmpty() ? displayNameSuffix : meshName + QStringLiteral("_") + displayNameSuffix;
	brep->setName(makeUniqueBrepDisplayName(*page, displayBase).toStdString());
	geoalgo::clearBrepImportArtifactsCache();

	constexpr bool kResetViewToHome = false;
	QString regErr;
	const bool registerOk = cloudsim::host::registerAdoptedBrepAndLoadScene(
		*page,
		brep,
		sourceUri,
		QStringLiteral("BrepModel"),
		QString(),
		kResetViewToHome,
		&regErr);
	if (!registerOk)
	{
		if (errMsg)
		{
			*errMsg = regErr.toStdString();
		}
		return false;
	}

	std::string alignErr;
	if (!document_point_cloud_ops::inheritBrepVisualPoseFromSourceMesh(
			page, sourceMeshBackendId, brep->id(), *brep, &alignErr))
	{
		if (errMsg)
		{
			*errMsg = alignErr.empty() ? "Fit preview B-rep visual placement failed" : alignErr;
		}
		return false;
	}

	if (OsgWidget* osg = widgetOsgFromPage(page))
	{
		osg->setBackendObjectVisible(sourceMesh->id(), true);
		osg->focusCameraOnBackend(brep->id());
	}
	if (host && host->mainWindowHost())
	{
		host->mainWindowHost()->focusBackendInTreeAfterImport(QString::fromStdString(brep->id()));
	}
	return true;
}

bool registerSurfaceReconStagePreviewBrep(
	PluginHostContext* host,
	cloudsim::host::DocumentHost* page,
	IPluginDocument* doc,
	const std::shared_ptr<MeshBackendData>& sourceMesh,
	const std::string& sourceMeshBackendId,
	geoalgo::MeshSurfaceReconstructSession& geoSession,
	std::string& sessionPreviewBackendId,
	const QString& displayNameSuffix,
	const QString& sourceUri,
	std::string* outPreviewBackendId,
	std::string* errMsg)
{
	geoalgo::ShapeHandle previewShape;
	std::string shapeErr;
	if (!geometry_backend_ops::buildFitPreviewShape(geoSession, previewShape, &shapeErr))
	{
		if (errMsg)
		{
			*errMsg = shapeErr;
		}
		return false;
	}

	auto previewBrep = std::make_shared<BrepBackendData>();
	std::string brepErr;
	if (!geometry_backend_ops::meshSurfaceReconstructShapeToBrep(previewShape, previewBrep, &brepErr))
	{
		if (errMsg)
		{
			*errMsg = brepErr;
		}
		return false;
	}

	if (!sessionPreviewBackendId.empty() && doc)
	{
		std::string removeErr;
		(void)doc->removeBackendObject(sessionPreviewBackendId, &removeErr);
	}

	std::string regErr;
	if (!registerFitPreviewBrepFromShape(
			host,
			page,
			sourceMesh,
			sourceMeshBackendId,
			previewBrep,
			&regErr,
			displayNameSuffix,
			sourceUri))
	{
		if (errMsg)
		{
			*errMsg = regErr;
		}
		return false;
	}

	sessionPreviewBackendId = previewBrep->id();
	if (outPreviewBackendId)
	{
		*outPreviewBackendId = sessionPreviewBackendId;
	}
	return true;
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

			PluginMeshSurfaceReconstructReport pluginReport =
				toPluginMeshSurfaceReconstructReport(
					result->report, result->brep->id(), PluginMeshSurfaceReconstructStage::Assemble);
			pluginReport.stageSummaryZh =
				formatSurfaceReconStageSummaryZh(PluginMeshSurfaceReconstructStage::Assemble, pluginReport, 0);
			onFinished(true, QString(), pluginReport);
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

PluginPointCloudHostImpl::SurfaceReconHostSession* PluginPointCloudHostImpl::findSurfaceReconSession(
	const std::string& sessionId,
	IPluginDocument* doc)
{
	if (sessionId.empty() || !doc)
	{
		return nullptr;
	}
	const auto it = m_surfaceReconSessions.find(sessionId);
	if (it == m_surfaceReconSessions.end() || it->second.docId != doc->documentId())
	{
		return nullptr;
	}
	return &it->second;
}

void PluginPointCloudHostImpl::eraseSurfaceReconSession(const std::string& sessionId, IPluginDocument* doc)
{
	SurfaceReconHostSession* session = findSurfaceReconSession(sessionId, doc);
	if (!session)
	{
		m_surfaceReconSessions.erase(sessionId);
		return;
	}
	if (!session->preprocessedMeshBackendId.empty() && doc)
	{
		std::string removeErr;
		(void)doc->removeBackendObject(session->preprocessedMeshBackendId, &removeErr);
	}
	if (!session->partitionColoredMeshBackendId.empty() && doc)
	{
		std::string removeErr;
		(void)doc->removeBackendObject(session->partitionColoredMeshBackendId, &removeErr);
	}
	if (!session->samplePointsBackendId.empty() && doc)
	{
		std::string removeErr;
		(void)doc->removeBackendObject(session->samplePointsBackendId, &removeErr);
	}
	if (!session->fitPreviewBrepBackendId.empty() && doc)
	{
		std::string removeErr;
		(void)doc->removeBackendObject(session->fitPreviewBrepBackendId, &removeErr);
	}
	if (!session->boundaryBlendPreviewBrepBackendId.empty() && doc)
	{
		std::string removeErr;
		(void)doc->removeBackendObject(session->boundaryBlendPreviewBrepBackendId, &removeErr);
	}
	if (!session->junctionBlendPreviewBrepBackendId.empty() && doc)
	{
		std::string removeErr;
		(void)doc->removeBackendObject(session->junctionBlendPreviewBrepBackendId, &removeErr);
	}
	m_surfaceReconSessions.erase(sessionId);
}

PluginMeshSurfaceReconstructSessionId PluginPointCloudHostImpl::beginMeshSurfaceReconstructSession(
	IPluginDocument* doc,
	const std::string& meshBackendIdUtf8)
{
	PluginMeshSurfaceReconstructSessionId out;
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page || meshBackendIdUtf8.empty())
	{
		return out;
	}
	std::string resolveErr;
	const auto mesh = document_point_cloud_ops::resolveMesh(page, meshBackendIdUtf8, &resolveErr);
	if (!mesh)
	{
		return out;
	}

	const std::string newSessionId = makeSurfaceReconSessionId();
	SurfaceReconHostSession session;
	session.sessionId = newSessionId;
	session.docId = doc->documentId();
	session.meshBackendId = meshBackendIdUtf8;
	session.rawSoup = mesh->triangleSoup();
	session.lastCompleted = PluginMeshSurfaceReconstructStage::None;
	m_surfaceReconSessions[newSessionId] = std::move(session);
	out.value = newSessionId;
	return out;
}

void PluginPointCloudHostImpl::clearMeshSurfaceReconstructSession(
	IPluginDocument* doc,
	const PluginMeshSurfaceReconstructSessionId& sessionId)
{
	eraseSurfaceReconSession(sessionId.value, doc);
}

void PluginPointCloudHostImpl::runMeshSurfaceReconstructStage(
	IPluginDocument* doc,
	const PluginMeshSurfaceReconstructSessionId& sessionId,
	const PluginMeshSurfaceReconstructStage stage,
	const PluginMeshSurfaceReconstructParams& params,
	PluginMeshSurfaceReconstructFinishedFn onFinished)
{
	if (!m_host || !onFinished || !sessionId.valid())
	{
		return;
	}
	SurfaceReconHostSession* session = findSurfaceReconSession(sessionId.value, doc);
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!session || !page)
	{
		onFinished(false, QStringLiteral("曲面重构会话无效"), {});
		return;
	}
	if (!isNextPluginStage(session->lastCompleted, stage))
	{
		onFinished(false, QStringLiteral("请先完成上一阶段"), {});
		return;
	}

	std::string resolveErr;
	const auto mesh = document_point_cloud_ops::resolveMesh(page, session->meshBackendId, &resolveErr);
	if (!mesh)
	{
		onFinished(false, QString::fromStdString(resolveErr), {});
		return;
	}

	const geoalgo::MeshSurfaceReconstructParams geoParams = buildMeshSurfaceReconstructGeoParams(params);
	struct WorkResult
	{
		geoalgo::MeshSurfaceReconstructReport report;
		std::vector<float> workingSoup;
		std::shared_ptr<BrepBackendData> brep;
		std::string preprocessedMeshBackendId;
		std::string partitionColoredMeshBackendId;
		std::string samplePointsBackendId;
		std::string fitPreviewBrepBackendId;
		std::string boundaryBlendPreviewBrepBackendId;
		std::string junctionBlendPreviewBrepBackendId;
		std::string newBrepBackendId;
		std::string error;
		bool ok = false;
	};
	auto result = std::make_shared<WorkResult>();

	const QString jobTitle = QStringLiteral("曲面重构");
	m_host->enqueueJob(
		jobTitle,
		[session, stage, geoParams, result](const PluginJobProgressFn& progress) {
			try
			{
				progress(0.1, QStringLiteral("执行中..."));
				if (stage == PluginMeshSurfaceReconstructStage::Preprocess)
				{
					result->ok = geometry_backend_ops::preprocessMeshSoupForSurfaceReconstruct(
						session->rawSoup, geoParams, result->workingSoup, result->report, &result->error);
				}
				else
				{
					if (!session->geoSession)
					{
						result->error = "surface reconstruction session not preprocessed";
						result->ok = false;
					}
					else
					{
						const geoalgo::MeshSurfaceReconstructStage geoStage = mapPluginStageToGeo(stage);
						geoalgo::ShapeHandle shape;
						geoalgo::ShapeHandle* shapeOut =
							(stage == PluginMeshSurfaceReconstructStage::Assemble) ? &shape : nullptr;
						result->ok = geometry_backend_ops::runMeshSurfaceReconstructStage(
							*session->geoSession, geoStage, geoParams, shapeOut, result->report, &result->error);
						if (result->ok && stage == PluginMeshSurfaceReconstructStage::Assemble)
						{
							result->brep = std::make_shared<BrepBackendData>();
							result->ok = geometry_backend_ops::meshSurfaceReconstructShapeToBrep(
								shape, result->brep, &result->error);
						}
					}
				}
				progress(1.0, QStringLiteral("完成"));
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
		[this,
			doc,
			session,
			stage,
			params,
			mesh,
			page,
			result,
			onFinished = std::move(onFinished)](const bool threw, const QString& throwMessage) {
			if (threw)
			{
				onFinished(false, translateSurfaceReconErrorZh(throwMessage), {});
				return;
			}
			if (!result->ok)
			{
				onFinished(false, translateSurfaceReconErrorZh(QString::fromStdString(result->error)), {});
				return;
			}

			if (stage == PluginMeshSurfaceReconstructStage::Preprocess)
			{
				session->workingSoup = std::move(result->workingSoup);
				session->geoSession =
					geometry_backend_ops::createMeshSurfaceReconstructSession(session->workingSoup);
				if (params.exportPreprocessedMeshToScene)
				{
					if (!session->preprocessedMeshBackendId.empty() && doc)
					{
						std::string removeErr;
						(void)doc->removeBackendObject(session->preprocessedMeshBackendId, &removeErr);
					}
					std::string regErr;
					if (!registerPreprocessedMeshFromSoup(
							m_host,
							page,
							mesh,
							session->workingSoup,
							params,
							result->preprocessedMeshBackendId,
							&regErr))
					{
						onFinished(false, translateSurfaceReconErrorZh(QString::fromStdString(regErr)), {});
						return;
					}
					session->preprocessedMeshBackendId = result->preprocessedMeshBackendId;
				}
			}
			else if (stage == PluginMeshSurfaceReconstructStage::Partition && session->geoSession)
			{
				std::vector<float> coloredSoup;
				std::vector<float> coloredRgb;
				std::string colorErr;
				if (geometry_backend_ops::buildPartitionColoredMeshSoup(
						*session->geoSession, coloredSoup, coloredRgb, &colorErr))
				{
					if (!session->partitionColoredMeshBackendId.empty() && doc)
					{
						std::string removeErr;
						(void)doc->removeBackendObject(session->partitionColoredMeshBackendId, &removeErr);
					}
					std::string regErr;
					if (!registerPartitionColoredMeshFromSoup(
							m_host,
							page,
							mesh,
							coloredSoup,
							coloredRgb,
							result->partitionColoredMeshBackendId,
							&regErr))
					{
						onFinished(false, translateSurfaceReconErrorZh(QString::fromStdString(regErr)), {});
						return;
					}
					session->partitionColoredMeshBackendId = result->partitionColoredMeshBackendId;
				}
			}
			else if (stage == PluginMeshSurfaceReconstructStage::Sample && session->geoSession)
			{
				std::vector<float> sampleXyz;
				std::vector<float> sampleRgba;
				std::string sampleErr;
				if (geometry_backend_ops::buildSamplePointsCloud(
						*session->geoSession, sampleXyz, sampleRgba, &sampleErr))
				{
					if (!session->samplePointsBackendId.empty() && doc)
					{
						std::string removeErr;
						(void)doc->removeBackendObject(session->samplePointsBackendId, &removeErr);
					}
					auto pcPtr = std::make_shared<PointCloudBackendData>();
					const QString baseName = QString::fromStdString(mesh->name());
					pcPtr->setName((baseName.isEmpty()
						? QStringLiteral("采样栅格") : baseName + QStringLiteral("_采样栅格")).toStdString());
					pcPtr->setPointBuffers(std::move(sampleXyz), std::move(sampleRgba));
					pcPtr->setPose(mesh->pose());
					pcPtr->setRotation(mesh->rotation());
					cloudsim::host::AdoptPointCloudOptions adoptOpt;
					adoptOpt.sourcePath = QStringLiteral("plugin://pointcloud/surface-reconstruct-sample");
					adoptOpt.resetViewToHome = false;
					QString adoptErr;
					const cloudsim::host::AdoptRegistrationResult adopted =
						cloudsim::host::registerAdoptedPointCloud(*page, pcPtr, adoptOpt, &adoptErr);
					if (!adopted.ok)
					{
						onFinished(false, adoptErr, {});
						return;
					}
					result->samplePointsBackendId = adopted.backendId.toStdString();
					session->samplePointsBackendId = result->samplePointsBackendId;
				}
			}
			else if (stage == PluginMeshSurfaceReconstructStage::Fit && session->geoSession)
			{
				std::string regErr;
				if (!registerSurfaceReconStagePreviewBrep(
						m_host,
						page,
						doc,
						mesh,
						session->meshBackendId,
						*session->geoSession,
						session->fitPreviewBrepBackendId,
						QStringLiteral("拟合曲面"),
						QStringLiteral("plugin://pointcloud/surface-reconstruct-fit"),
						&result->fitPreviewBrepBackendId,
						&regErr))
				{
					if (m_host)
					{
						m_host->logWarn(
							QStringLiteral("[曲面重构] 拟合预览未写入场景: %1")
								.arg(QString::fromStdString(regErr)));
					}
				}
			}
			else if (stage == PluginMeshSurfaceReconstructStage::BoundaryBlend && session->geoSession)
			{
				std::string regErr;
				if (!registerSurfaceReconStagePreviewBrep(
						m_host,
						page,
						doc,
						mesh,
						session->meshBackendId,
						*session->geoSession,
						session->boundaryBlendPreviewBrepBackendId,
						QStringLiteral("边界混合"),
						QStringLiteral("plugin://pointcloud/surface-reconstruct-boundary-blend"),
						&result->boundaryBlendPreviewBrepBackendId,
						&regErr))
				{
					if (m_host)
					{
						m_host->logWarn(
							QStringLiteral("[曲面重构] 边界混合预览未写入场景: %1")
								.arg(QString::fromStdString(regErr)));
					}
				}
			}
			else if (stage == PluginMeshSurfaceReconstructStage::JunctionBlend && session->geoSession)
			{
				std::string regErr;
				if (!registerSurfaceReconStagePreviewBrep(
						m_host,
						page,
						doc,
						mesh,
						session->meshBackendId,
						*session->geoSession,
						session->junctionBlendPreviewBrepBackendId,
						QStringLiteral("交汇混合"),
						QStringLiteral("plugin://pointcloud/surface-reconstruct-junction-blend"),
						&result->junctionBlendPreviewBrepBackendId,
						&regErr))
				{
					if (m_host)
					{
						m_host->logWarn(
							QStringLiteral("[曲面重构] 交汇混合预览未写入场景: %1")
								.arg(QString::fromStdString(regErr)));
					}
				}
			}
			else if (stage == PluginMeshSurfaceReconstructStage::Assemble && result->brep)
			{
				std::string regErr;
				if (!registerReconstructedBrepFromShape(
						m_host,
						page,
						mesh,
						session->meshBackendId,
						result->brep,
						params,
						&regErr))
				{
					onFinished(false, translateSurfaceReconErrorZh(QString::fromStdString(regErr)), {});
					return;
				}
				result->newBrepBackendId = result->brep->id();
				if (m_host)
				{
					m_host->logInfo(
						QStringLiteral("[曲面重构] 装配完成 id=%1 分块=%2 最大偏差=%3 mm")
							.arg(QString::fromStdString(result->newBrepBackendId))
							.arg(result->report.patchCount)
							.arg(result->report.maxDeviationMm, 0, 'f', 4));
				}
			}

			session->lastCompleted = stage;
			PluginMeshSurfaceReconstructReport pluginReport =
				toPluginMeshSurfaceReconstructReport(result->report, result->newBrepBackendId, stage);
			pluginReport.preprocessedMeshBackendId = session->preprocessedMeshBackendId;
			pluginReport.partitionColoredMeshBackendId = session->partitionColoredMeshBackendId;
			pluginReport.fitPreviewBrepBackendId = session->fitPreviewBrepBackendId;
			pluginReport.boundaryBlendPreviewBrepBackendId = session->boundaryBlendPreviewBrepBackendId;
			pluginReport.junctionBlendPreviewBrepBackendId = session->junctionBlendPreviewBrepBackendId;
			pluginReport.stageSummaryZh =
				formatSurfaceReconStageSummaryZh(stage, pluginReport, params.samplesPerPatchEdge);
			if (stage == PluginMeshSurfaceReconstructStage::Fit
				&& !session->fitPreviewBrepBackendId.empty())
			{
				pluginReport.stageSummaryZh += QStringLiteral("（场景已显示拟合曲面）");
			}
			if (stage == PluginMeshSurfaceReconstructStage::BoundaryBlend
				&& !session->boundaryBlendPreviewBrepBackendId.empty())
			{
				pluginReport.stageSummaryZh += QStringLiteral("（场景已显示边界混合曲面）");
			}
			if (stage == PluginMeshSurfaceReconstructStage::JunctionBlend
				&& !session->junctionBlendPreviewBrepBackendId.empty())
			{
				pluginReport.stageSummaryZh += QStringLiteral("（场景已显示交汇混合曲面）");
			}
			else if (stage == PluginMeshSurfaceReconstructStage::JunctionBlend)
			{
				pluginReport.stageSummaryZh += QStringLiteral("（交汇混合场景预览未生成）");
			}
			if (m_host)
			{
				m_host->logInfo(QStringLiteral("[曲面重构] %1").arg(pluginReport.stageSummaryZh));
			}
			onFinished(true, QString(), pluginReport);
		});
}

PluginPointCloudHostImpl::TubularGrindingHostSession* PluginPointCloudHostImpl::findTubularGrindingSession(
	const std::string& sessionId,
	IPluginDocument* doc)
{
	const auto it = m_tubularGrindingSessions.find(sessionId);
	if (it == m_tubularGrindingSessions.end())
	{
		return nullptr;
	}
	if (doc && it->second.docId != doc->documentId())
	{
		return nullptr;
	}
	return &it->second;
}

void PluginPointCloudHostImpl::eraseTubularGrindingSession(
	const std::string& sessionId,
	IPluginDocument* doc)
{
	TubularGrindingHostSession* session = findTubularGrindingSession(sessionId, doc);
	if (!session)
	{
		m_tubularGrindingSessions.erase(sessionId);
		return;
	}
	const auto removeId = [&](const std::string& backendId) {
		if (!backendId.empty() && doc)
		{
			std::string removeErr;
			(void)doc->removeBackendObject(backendId, &removeErr);
		}
	};
	removeId(session->normalAxisLinesBackendId);
	removeId(session->localAxisLinesBackendId);
	removeId(session->centerlinePointsBackendId);
	removeId(session->centerlinePcaAxisBackendId);
	removeId(session->templatePointsBackendId);
	removeId(session->projectedPointsBackendId);
	for (const auto& bid : session->iterationSnapshotBackendIds)
	{
		removeId(bid);
	}
	session->iterationSnapshotBackendIds.clear();
	m_tubularGrindingSessions.erase(sessionId);
}

PluginTubularGrindingSessionId PluginPointCloudHostImpl::beginTubularGrindingSession(
	IPluginDocument* doc,
	const std::string& meshBackendIdUtf8)
{
	PluginTubularGrindingSessionId out;
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page || meshBackendIdUtf8.empty())
	{
		return out;
	}
	std::string resolveErr;
	const auto mesh = document_point_cloud_ops::resolveMesh(page, meshBackendIdUtf8, &resolveErr);
	if (mesh)
	{
		const std::string newSessionId = makeTubularGrindingSessionId();
		TubularGrindingHostSession session;
		session.sessionId = newSessionId;
		session.docId = doc->documentId();
		session.meshBackendId = meshBackendIdUtf8;
		session.rawSoup = mesh->triangleSoup();
		session.inputKind = 0; // mesh
		session.geoSession = geometry_backend_ops::createTubularGrindingSession(session.rawSoup);
		session.lastCompleted = PluginTubularGrindingStage::None;
		m_tubularGrindingSessions[newSessionId] = std::move(session);
		out.value = newSessionId;
		return out;
	}
	// 点云 fallback
	const auto pc = document_point_cloud_ops::resolvePointCloud(page, meshBackendIdUtf8, &resolveErr);
	if (!pc)
	{
		return out;
	}
	const std::string newSessionId = makeTubularGrindingSessionId();
	TubularGrindingHostSession session;
	session.sessionId = newSessionId;
	session.docId = doc->documentId();
	session.meshBackendId = meshBackendIdUtf8;
	session.rawPointXyz = pc->pointPositionsXyz();
	session.inputKind = 1; // point cloud
	session.geoSession = geometry_backend_ops::createTubularGrindingSessionFromPointCloud(session.rawPointXyz);
	session.lastCompleted = PluginTubularGrindingStage::None;
	m_tubularGrindingSessions[newSessionId] = std::move(session);
	out.value = newSessionId;
	return out;
}

void PluginPointCloudHostImpl::clearTubularGrindingSession(
	IPluginDocument* doc,
	const PluginTubularGrindingSessionId& sessionId)
{
	eraseTubularGrindingSession(sessionId.value, doc);
}

void PluginPointCloudHostImpl::runTubularGrindingStage(
	IPluginDocument* doc,
	const PluginTubularGrindingSessionId& sessionId,
	const PluginTubularGrindingStage stage,
	const PluginTubularGrindingParams& params,
	PluginTubularGrindingFinishedFn onFinished)
{
	if (!m_host || !onFinished || !sessionId.valid())
	{
		return;
	}
	TubularGrindingHostSession* session = findTubularGrindingSession(sessionId.value, doc);
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!session || !page || !session->geoSession)
	{
		onFinished(false, QStringLiteral("特征构建会话无效"), {});
		return;
	}
	if (!isNextTubularGrindingStage(session->lastCompleted, stage))
	{
		onFinished(false, QStringLiteral("请先完成上一阶段"), {});
		return;
	}

	const bool usePointCloud = session->inputKind == 1;
	std::shared_ptr<MeshBackendData> mesh;
	std::shared_ptr<PointCloudBackendData> sourcePointCloud;
	std::string resolveErr;
	if (usePointCloud)
	{
		sourcePointCloud =
			document_point_cloud_ops::resolvePointCloud(page, session->meshBackendId, &resolveErr);
		if (!sourcePointCloud)
		{
			onFinished(
				false,
				resolveErr.empty() ? QStringLiteral("点云对象无效") : QString::fromStdString(resolveErr),
				{});
			return;
		}
	}
	else
	{
		mesh = document_point_cloud_ops::resolveMesh(page, session->meshBackendId, &resolveErr);
		if (!mesh)
		{
			onFinished(false, QString::fromStdString(resolveErr), {});
			return;
		}
	}

	const geoalgo::TubularGrindingParams geoParams = buildTubularGrindingGeoParams(params);
	struct WorkResult
	{
		geoalgo::TubularGrindingReport report;
		std::string normalAxisLinesBackendId;
		std::string localAxisLinesBackendId;
		std::string ellipseResidualSummary;
		std::string centerlinePointsBackendId;
		std::string centerlinePcaAxisBackendId;
		std::string templatePointsBackendId;
		std::string projectedPointsBackendId;
		std::string error;
		bool ok = false;
	};
	auto result = std::make_shared<WorkResult>();

	m_host->enqueueJob(
		QStringLiteral("特征构建"),
		[session, stage, geoParams, result](const PluginJobProgressFn& progress) {
			try
			{
				progress(0.1, QStringLiteral("执行中..."));
				const geoalgo::TubularGrindingStage geoStage = mapPluginTubularStageToGeo(stage);
				result->ok = geometry_backend_ops::runTubularGrindingStage(
					*session->geoSession, geoStage, geoParams, result->report, &result->error);
				progress(1.0, QStringLiteral("完成"));
			}
			catch (const std::exception& ex)
			{
				result->ok = false;
				result->error = ex.what();
			}
			catch (...)
			{
				result->ok = false;
				result->error = "tubular grinding failed with internal error";
			}
		},
		[this, doc, session, stage, mesh, sourcePointCloud, usePointCloud, page, geoParams, result, onFinished = std::move(onFinished)](
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

			if (stage == PluginTubularGrindingStage::Centerline)
			{
				std::vector<float> polylineXyz;
				std::string polyErr;
				if (geometry_backend_ops::buildTubularGrindingCenterlinePolylineXyz(
						*session->geoSession, polylineXyz, &polyErr))
				{
					if (!session->centerlinePointsBackendId.empty() && doc)
					{
						std::string removeErr;
						(void)doc->removeBackendObject(session->centerlinePointsBackendId, &removeErr);
					}
					std::string adoptErr;
					const bool centerlineRegistered = usePointCloud
						? registerTubularGrindingCenterlineLines(
							m_host,
							page,
							sourcePointCloud,
							polylineXyz,
							result->centerlinePointsBackendId,
							&adoptErr)
						: registerTubularGrindingCenterlineLines(
							m_host,
							page,
							mesh,
							polylineXyz,
							result->centerlinePointsBackendId,
							&adoptErr);
					if (!centerlineRegistered)
					{
						onFinished(false, QString::fromStdString(adoptErr), {});
						return;
					}
					session->centerlinePointsBackendId = result->centerlinePointsBackendId;
				}

				std::vector<float> pcaLineXyz;
				std::string pcaErr;
				if (geometry_backend_ops::buildTubularGrindingCenterlinePcaAxisArrowLineSegments(
						*session->geoSession, pcaLineXyz, &pcaErr))
				{
					if (!session->centerlinePcaAxisBackendId.empty() && doc)
					{
						std::string removeErr;
						(void)doc->removeBackendObject(session->centerlinePcaAxisBackendId, &removeErr);
					}
					std::string pcaAdoptErr;
					const bool pcaRegistered = usePointCloud
						? registerTubularGrindingPcaAxisArrowLines(
							m_host,
							page,
							sourcePointCloud,
							pcaLineXyz,
							result->centerlinePcaAxisBackendId,
							&pcaAdoptErr)
						: registerTubularGrindingPcaAxisArrowLines(
							m_host,
							page,
							mesh,
							pcaLineXyz,
							result->centerlinePcaAxisBackendId,
							&pcaAdoptErr);
					if (!pcaRegistered)
					{
						onFinished(false, QString::fromStdString(pcaAdoptErr), {});
						return;
					}
					session->centerlinePcaAxisBackendId = result->centerlinePcaAxisBackendId;
				}

				for (const auto& bid : session->iterationSnapshotBackendIds)
				{
					if (!bid.empty() && doc)
					{
						(void)doc->removeBackendObject(bid, nullptr);
					}
				}
				session->iterationSnapshotBackendIds.clear();
				const int snapCount =
					geometry_backend_ops::tubularGrindingIterationSnapshotCount(*session->geoSession);
				for (int si = 0; si < snapCount; ++si)
				{
					const int iteration =
						geometry_backend_ops::tubularGrindingIterationSnapshotIteration(
							*session->geoSession, si);
					std::vector<float> snapXyz;
					std::vector<float> snapRgba;
					std::string snapErr;
					if (!geometry_backend_ops::buildTubularGrindingIterationSnapshotPointsCloud(
							*session->geoSession, si, snapXyz, snapRgba, &snapErr))
					{
						continue;
					}
					const QString suffix = QStringLiteral("_迭代%1").arg(iteration);
					std::string backendId;
					QString adoptErr;
					const bool registered = usePointCloud
						? registerTubularGrindingPointCloud(
							page, sourcePointCloud, suffix,
							QStringLiteral("plugin://pointcloud/tubular-grinding-iteration"),
							std::move(snapXyz), std::move(snapRgba),
							backendId, &adoptErr)
						: registerTubularGrindingPointCloud(
							page, mesh, suffix,
							QStringLiteral("plugin://pointcloud/tubular-grinding-iteration"),
							std::move(snapXyz), std::move(snapRgba),
							backendId, &adoptErr);
					if (registered && !backendId.empty())
					{
						session->iterationSnapshotBackendIds.push_back(backendId);
					}
					std::vector<float> contractedXyz;
					std::vector<float> contractedRgba;
					if (geometry_backend_ops::buildTubularGrindingIterationSnapshotContractedPointsCloud(
							*session->geoSession, si, contractedXyz, contractedRgba, &snapErr))
					{
						const QString contractedSuffix =
							QStringLiteral("_迭代%1_收缩").arg(iteration);
						std::string contractedBackendId;
						const bool contractedRegistered = usePointCloud
							? registerTubularGrindingPointCloud(
								page, sourcePointCloud, contractedSuffix,
								QStringLiteral("plugin://pointcloud/tubular-grinding-iteration-contracted"),
								std::move(contractedXyz), std::move(contractedRgba),
								contractedBackendId, &adoptErr)
							: registerTubularGrindingPointCloud(
								page, mesh, contractedSuffix,
								QStringLiteral("plugin://pointcloud/tubular-grinding-iteration-contracted"),
								std::move(contractedXyz), std::move(contractedRgba),
								contractedBackendId, &adoptErr);
						if (contractedRegistered && !contractedBackendId.empty())
						{
							session->iterationSnapshotBackendIds.push_back(contractedBackendId);
						}
					}
				}
			}
			else if (stage == PluginTubularGrindingStage::TemplatePoints)
			{
				std::vector<float> xyz;
				std::vector<float> rgba;
				std::string pcErr;
				if (geometry_backend_ops::buildTubularGrindingTemplatePointsCloud(
						*session->geoSession, xyz, rgba, &pcErr))
				{
					if (!session->templatePointsBackendId.empty() && doc)
					{
						std::string removeErr;
						(void)doc->removeBackendObject(session->templatePointsBackendId, &removeErr);
					}
					QString adoptErr;
					const bool templateRegistered = usePointCloud
						? registerTubularGrindingPointCloud(
							page,
							sourcePointCloud,
							QStringLiteral("_模板点位"),
							QStringLiteral("plugin://pointcloud/tubular-grinding-template"),
							std::move(xyz),
							std::move(rgba),
							result->templatePointsBackendId,
							&adoptErr)
						: registerTubularGrindingPointCloud(
							page,
							mesh,
							QStringLiteral("_模板点位"),
							QStringLiteral("plugin://pointcloud/tubular-grinding-template"),
							std::move(xyz),
							std::move(rgba),
							result->templatePointsBackendId,
							&adoptErr);
					if (!templateRegistered)
					{
						onFinished(false, adoptErr, {});
						return;
					}
					session->templatePointsBackendId = result->templatePointsBackendId;
				}
			}
			else if (stage == PluginTubularGrindingStage::Project)
			{
				std::vector<float> xyz;
				std::vector<float> rgba;
				std::string pcErr;
				if (geometry_backend_ops::buildTubularGrindingProjectedPointsCloud(
						*session->geoSession, xyz, rgba, &pcErr))
				{
					if (!session->projectedPointsBackendId.empty() && doc)
					{
						std::string removeErr;
						(void)doc->removeBackendObject(session->projectedPointsBackendId, &removeErr);
					}
					QString adoptErr;
					const bool projectRegistered = usePointCloud
						? registerTubularGrindingPointCloud(
							page,
							sourcePointCloud,
							QStringLiteral("_投影点位"),
							QStringLiteral("plugin://pointcloud/tubular-grinding-project"),
							std::move(xyz),
							std::move(rgba),
							result->projectedPointsBackendId,
							&adoptErr)
						: registerTubularGrindingPointCloud(
							page,
							mesh,
							QStringLiteral("_投影点位"),
							QStringLiteral("plugin://pointcloud/tubular-grinding-project"),
							std::move(xyz),
							std::move(rgba),
							result->projectedPointsBackendId,
							&adoptErr);
					if (!projectRegistered)
					{
						onFinished(false, adoptErr, {});
						return;
					}
					session->projectedPointsBackendId = result->projectedPointsBackendId;
				}
			}

			session->lastCompleted = stage;
			PluginTubularGrindingReport pluginReport;
			pluginReport.lastCompletedStage = stage;
			pluginReport.pipeCount = result->report.pipeCount;
			pluginReport.ringCount = result->report.ringCount;
			pluginReport.junctionFaceCount = result->report.junctionFaceCount;
			pluginReport.regionCountBeforeFilter = result->report.regionCountBeforeFilter;
			pluginReport.centerlinePointCount = result->report.centerlinePointCount;
			pluginReport.templatePointCount = result->report.templatePointCount;
			pluginReport.projectedPointCount = result->report.projectedPointCount;
			pluginReport.sectionFitFailCount = result->report.sectionFitFailCount;
			pluginReport.projectionHitRate = result->report.projectionHitRate;
			pluginReport.centerlinePcaFallback = result->report.centerlinePcaFallback;
			pluginReport.centerlineOtLcExtraction = result->report.centerlineOtLcExtraction;
			pluginReport.centerlineOtRootCount = result->report.centerlineOtRootCount;
			pluginReport.centerlineOtEdgeCount = result->report.centerlineOtEdgeCount;
			pluginReport.centerlineOtComponentCount = result->report.centerlineOtComponentCount;
			pluginReport.centerlineOtKnnFallbackEdges = result->report.centerlineOtKnnFallbackEdges;
			pluginReport.centerlineOtPathKind = result->report.centerlineOtPathKind;
			pluginReport.normalAxisLinesBackendId = session->normalAxisLinesBackendId;
			pluginReport.centerlinePointsBackendId = session->centerlinePointsBackendId;
			pluginReport.templatePointsBackendId = session->templatePointsBackendId;
			pluginReport.projectedPointsBackendId = session->projectedPointsBackendId;
			pluginReport.stageSummaryZh = formatTubularGrindingStageSummaryZh(stage, pluginReport);
			if (m_host)
			{
				m_host->logInfo(QStringLiteral("[特征构建] %1").arg(pluginReport.stageSummaryZh));
				// 输出椭圆拟合残差摘要
				if (!result->ellipseResidualSummary.empty())
				{
					m_host->logInfo(QStringLiteral("[特征构建] %1")
						.arg(QString::fromStdString(result->ellipseResidualSummary)));
				}
			}
			onFinished(true, QString(), pluginReport);
		});
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
