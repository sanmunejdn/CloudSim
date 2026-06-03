#include "PluginPointCloudHostImpl.h"

#include "DocumentImportFacade.h"
#include "DocumentPointCloudOps.h"
#include "DocumentHost.h"
#include "MeshBackendData.h"
#include "PluginDocumentAdapter.h"
#include "PluginHostContext.h"
#include "PointCloudBackendData.h"

#include <functional>
#include <memory>
#include <stdexcept>

namespace
{

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
