/// @file GeometryBackendOps.cpp
/// @brief 仅统计距模板 soup 在 inlierGateMm 内的扫描点（与 ICP 重叠区一致，避免全云离群点拉高 maxDev）

#include "pch.h"

#include "GeometryBackendOps.h"

#include "BackendDataManager.h"
#include "BackendFollowMath.h"
#include "BackendSpatial.h"
#include "BrepBackendData.h"
#include "GeometryRef.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"

#include <BrepImportArtifacts.h>
#include <Discretize.h>
#include <Downsample.h>
#include <FeatureDiscretizerBridge.h>
#include <KdTreePointSet.h>
#include <Measure.h>
#include <MeshDiscretize.h>
#include <MeshSurfaceReconstruction.h>
#include <Preprocess.h>
#include <RegistrationGlobal.h>
#include <RegistrationRigid.h>
#include <ShapeQuery.h>
#include <TemplateBrepRegistration.h>
#include <TemplateBrepUpdate.h>
#include <TubularGrinding.h>
#if defined(_WIN64)
#include <MeshNormalSmooth.h>
#include <MeshRemesh.h>
#include <MeshRepair.h>
#endif
#include "RunLogger.h"

#include <algorithm>
#include <array>
#include <limits>
#include <sstream>
#include <unordered_map>

#include <Adapters.h>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <RigidTransform.h>
#include <ShapeIo.h>
#include <ShapeQuery.h>

namespace geometry_backend_ops
{
namespace
{
void applyIsometryInPlace(std::vector<float>& xyz, const Eigen::Isometry3d& transform)
{
	const std::size_t n = xyz.size() / 3U;
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		Eigen::Vector3d p(xyz[b], xyz[b + 1U], xyz[b + 2U]);
		p = transform * p;
		xyz[b] = static_cast<float>(p.x());
		xyz[b + 1U] = static_cast<float>(p.y());
		xyz[b + 2U] = static_cast<float>(p.z());
	}
}

void applyIsometryInPlaceNormals(std::vector<float>& normals, const Eigen::Isometry3d& transform)
{
	const std::size_t n = normals.size() / 3U;
	const Eigen::Matrix3d rot = transform.linear();
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		Eigen::Vector3d nrm(normals[b], normals[b + 1U], normals[b + 2U]);
		nrm = rot * nrm;
		if (nrm.norm() > 1e-9)
		{
			nrm.normalize();
		}
		normals[b] = static_cast<float>(nrm.x());
		normals[b + 1U] = static_cast<float>(nrm.y());
		normals[b + 2U] = static_cast<float>(nrm.z());
	}
}

double boundingBoxDiagonalMm(const std::vector<float>& xyz)
{
	if (xyz.size() < 9U)
	{
		return 0.0;
	}
	return pclalgo::computeBoundingBox(xyz).diagonal().norm();
}

constexpr std::size_t kRegistrationPairTargetPoints = 12000U;
constexpr std::size_t kFineRegistrationPairTargetPoints = 30000U;
constexpr double kFineMaxMatchVoxelMm = 2.0;
constexpr double kFineTargetInlierAvgMm = 5.0;

double fineOverlapInlierGateMm(const geoalgo::TemplateBrepUpdateParams& params, const double baselineInlierMaxMm)
{
	const double fb = std::max(params.faceBandMm, 1.0);
	const double vox = params.registrationMatchVoxelMm > 0.0 ? params.registrationMatchVoxelMm : fb;
	const double fromBaseline = baselineInlierMaxMm > 0.0 ? baselineInlierMaxMm * 1.15 : 0.0;
	return std::min(10.0, std::max({vox * 2.5, fromBaseline, 6.0}));
}

double voxelDownsampleXyzToTargetCount(std::vector<float>& xyz, const std::size_t targetCount)
{
	const std::size_t pointCount = xyz.size() / 3U;
	const double diag = boundingBoxDiagonalMm(xyz);
	if (pointCount == 0U)
	{
		return std::max(diag, 1e-9);
	}
	if (pointCount <= targetCount || diag < 1e-15)
	{
		return std::max(diag / std::max(std::cbrt(static_cast<double>(pointCount)), 1.0), 1e-9);
	}

	double lo = std::max(diag * 1e-7, 1e-9);
	double hi = diag;
	double bestVoxel = lo;
	std::size_t bestDiff = pointCount - targetCount;
	std::vector<float> bestXyz = xyz;

	for (int iter = 0; iter < 40; ++iter)
	{
		const double mid = (lo + hi) * 0.5;
		std::vector<float> trial = xyz;
		if (!pclalgo::downsampleVoxelGrid(trial, mid))
		{
			hi = mid;
			continue;
		}
		const std::size_t trialCount = trial.size() / 3U;
		if (trialCount == 0U)
		{
			hi = mid;
			continue;
		}
		const std::size_t diff = trialCount > targetCount ? trialCount - targetCount : targetCount - trialCount;
		if (diff < bestDiff)
		{
			bestDiff = diff;
			bestXyz = std::move(trial);
			bestVoxel = mid;
		}
		if (trialCount > targetCount)
		{
			lo = mid;
		}
		else
		{
			hi = mid;
		}
		if (hi - lo < std::max(diag * 1e-10, 1e-12))
		{
			break;
		}
	}
	xyz = std::move(bestXyz);
	return bestVoxel;
}

double downsampleRegistrationPairToTarget(std::vector<float>& scanXyz, std::vector<float>& scanNormals,
										  std::vector<float>& templateXyz, std::vector<float>& templateNormals,
										  const std::size_t targetCount = kRegistrationPairTargetPoints,
										  const double maxVoxelMm = 0.0)
{
	double scanVoxel = voxelDownsampleXyzToTargetCount(scanXyz, targetCount);
	double templateVoxel = voxelDownsampleXyzToTargetCount(templateXyz, targetCount);
	if (maxVoxelMm > 0.0)
	{
		if (scanVoxel > maxVoxelMm)
		{
			scanVoxel = maxVoxelMm;
			(void)pclalgo::downsampleVoxelGrid(scanXyz, scanVoxel);
		}
		if (templateVoxel > maxVoxelMm)
		{
			templateVoxel = maxVoxelMm;
			(void)pclalgo::downsampleVoxelGrid(templateXyz, templateVoxel);
		}
	}
	scanNormals.clear();
	templateNormals.clear();
	(void)pclalgo::estimateNormalsPca(scanXyz, scanNormals, 12U, nullptr);
	(void)pclalgo::estimateNormalsPca(templateXyz, templateNormals, 12U, nullptr);
	const double matchVoxel = std::max({scanVoxel, templateVoxel, 1e-9});
	const std::size_t scanPts = scanXyz.size() / 3U;
	const std::size_t templatePts = templateXyz.size() / 3U;
	const std::size_t minPts = std::min(scanPts, templatePts);
	const double countRatio =
		minPts > 0U ? static_cast<double>(std::max(scanPts, templatePts)) / static_cast<double>(minPts) : 0.0;
	RunLogger::info(std::string("[TemplateBrepUpdate] paired downsample scanPts=") + std::to_string(scanPts) +
					" templatePts=" + std::to_string(templatePts) + " scanVoxelMm=" + std::to_string(scanVoxel) +
					" templateVoxelMm=" + std::to_string(templateVoxel) +
					" matchVoxelMm=" + std::to_string(matchVoxel) + " countRatio=" + std::to_string(countRatio));
	return matchVoxel;
}

double ransacTranslationCapMm(const double modelDiag, const double preAlignGateMm, const double initialMaxDevMm)
{
	const double baseCap = std::max(modelDiag * 0.25, preAlignGateMm * 2.0);
	if (initialMaxDevMm > modelDiag * 0.12)
	{
		return std::max({modelDiag * 0.85, initialMaxDevMm * 1.15, baseCap});
	}
	return baseCap;
}

engine::RigidTransform rigidFromBackendWorldMat(const BackendMat4& world)
{
	engine::ColMajorMat4 cm{};
	for (int i = 0; i < 16; ++i)
	{
		cm[static_cast<size_t>(i)] = world.v[i];
	}
	return engine::rigidTransformFromColMajor(cm);
}

void transformXyzByEngineWorldMatrix(std::vector<float>& xyz, const BackendMat4& world)
{
	const Eigen::Isometry3d iso = rigidFromBackendWorldMat(world).isometry();
	const std::size_t n = xyz.size() / 3U;
	for (std::size_t i = 0U; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		Eigen::Vector3d p(static_cast<double>(xyz[b]), static_cast<double>(xyz[b + 1U]),
						  static_cast<double>(xyz[b + 2U]));
		p = iso * p;
		xyz[b] = static_cast<float>(p.x());
		xyz[b + 1U] = static_cast<float>(p.y());
		xyz[b + 2U] = static_cast<float>(p.z());
	}
}

void transformNormalsByEngineWorldMatrix(std::vector<float>& normals, const BackendMat4& world)
{
	const Eigen::Matrix3d rot = rigidFromBackendWorldMat(world).isometry().linear();
	const std::size_t n = normals.size() / 3U;
	for (std::size_t i = 0U; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		Eigen::Vector3d nrm(static_cast<double>(normals[b]), static_cast<double>(normals[b + 1U]),
							static_cast<double>(normals[b + 2U]));
		nrm = rot * nrm;
		if (nrm.norm() > 1e-9)
		{
			nrm.normalize();
		}
		normals[b] = static_cast<float>(nrm.x());
		normals[b + 1U] = static_cast<float>(nrm.y());
		normals[b + 2U] = static_cast<float>(nrm.z());
	}
}

BackendMat4 backendMat4FromRigid(const engine::RigidTransform& rt)
{
	const engine::ColMajorMat4 cm = engine::colMajorFromRigidTransform(rt);
	BackendMat4 out{};
	for (int i = 0; i < 16; ++i)
	{
		out.v[i] = cm[static_cast<size_t>(i)];
	}
	return out;
}

bool scanPointsToTemplateModelFrame(const PointCloudBackendData& scanCloud, const BrepBackendData& templateBrep,
									std::vector<float>& inOutXyz, std::vector<float>& inOutNormals)
{
	if (inOutXyz.size() < 9U || (inOutXyz.size() % 3U) != 0U)
	{
		return false;
	}
	const BackendMat4 scanWorld = scanCloud.worldMatrix();
	const BackendMat4 invTemplateWorld =
		backendMat4FromRigid(rigidFromBackendWorldMat(templateBrep.worldMatrix()).inverse());
	transformXyzByEngineWorldMatrix(inOutXyz, scanWorld);
	transformXyzByEngineWorldMatrix(inOutXyz, invTemplateWorld);
	if (!inOutNormals.empty() && inOutNormals.size() == inOutXyz.size())
	{
		transformNormalsByEngineWorldMatrix(inOutNormals, scanWorld);
		transformNormalsByEngineWorldMatrix(inOutNormals, invTemplateWorld);
	}
	return true;
}

void subsampleXyzUniform(std::vector<float>& xyz, const std::size_t maxPoints)
{
	const std::size_t n = xyz.size() / 3U;
	if (n <= maxPoints || maxPoints < 3U)
	{
		return;
	}
	std::vector<float> reduced;
	reduced.reserve(maxPoints * 3U);
	const std::size_t step = std::max<std::size_t>(1U, n / maxPoints);
	for (std::size_t i = 0; i < n; i += step)
	{
		if ((reduced.size() / 3U) >= maxPoints)
		{
			break;
		}
		const std::size_t b = i * 3U;
		reduced.push_back(xyz[b]);
		reduced.push_back(xyz[b + 1U]);
		reduced.push_back(xyz[b + 2U]);
	}
	xyz.swap(reduced);
}

void subsampleXyzWithNormalsUniform(std::vector<float>& xyz, std::vector<float>& normals, const std::size_t maxPoints)
{
	const std::size_t n = xyz.size() / 3U;
	if (n <= maxPoints || maxPoints < 3U)
	{
		return;
	}
	const bool hasNormals = normals.size() == xyz.size();
	std::vector<float> reducedXyz;
	std::vector<float> reducedNormals;
	reducedXyz.reserve(maxPoints * 3U);
	if (hasNormals)
	{
		reducedNormals.reserve(maxPoints * 3U);
	}
	const std::size_t step = std::max<std::size_t>(1U, n / maxPoints);
	for (std::size_t i = 0; i < n; i += step)
	{
		if ((reducedXyz.size() / 3U) >= maxPoints)
		{
			break;
		}
		const std::size_t b = i * 3U;
		reducedXyz.push_back(xyz[b]);
		reducedXyz.push_back(xyz[b + 1U]);
		reducedXyz.push_back(xyz[b + 2U]);
		if (hasNormals)
		{
			reducedNormals.push_back(normals[b]);
			reducedNormals.push_back(normals[b + 1U]);
			reducedNormals.push_back(normals[b + 2U]);
		}
	}
	xyz.swap(reducedXyz);
	if (hasNormals)
	{
		normals.swap(reducedNormals);
	}
	else
	{
		normals.clear();
	}
}

geoalgo::ShapeHandle resolveTemplateShapeForUpdate(const BrepBackendData& templateBrep,
												   const std::string& templateStepPathUtf8, std::string* errMsg)
{
	if (!templateStepPathUtf8.empty())
	{
		geoalgo::ShapeHandle fromStep;
		std::string stepErr;
		if (geoalgo::readStepIntoHandle(templateStepPathUtf8, fromStep, &stepErr) && !fromStep.isNull())
		{
			return fromStep;
		}
		if (errMsg && !stepErr.empty())
		{
			*errMsg = "STEP template load failed: " + stepErr;
		}
	}

	const geoalgo::ShapeHandle& inMemory = templateBrep.shapeRef();
	if (!inMemory.isNull())
	{
		return inMemory;
	}
	if (errMsg)
	{
		*errMsg = "template B-rep has no shape";
	}
	return geoalgo::ShapeHandle{};
}

geoalgo::ShapeHandle resolveTemplateShapeForIcp(const BrepBackendData& templateBrep,
												const std::string& templateStepPathUtf8, std::string* errMsg)
{
	if (!templateStepPathUtf8.empty())
	{
		geoalgo::ShapeHandle fromStep;
		std::string stepErr;
		if (geoalgo::readStepIntoHandle(templateStepPathUtf8, fromStep, &stepErr) && !fromStep.isNull())
		{
			return fromStep;
		}
		if (errMsg && !stepErr.empty())
		{
			*errMsg = "STEP template load failed: " + stepErr;
		}
	}

	const geoalgo::ShapeHandle& inMemory = templateBrep.shapeRef();
	if (!inMemory.isNull())
	{
		return inMemory;
	}
	if (errMsg)
	{
		*errMsg = "template B-rep has no shape";
	}
	return geoalgo::ShapeHandle{};
}

std::size_t countPointsWithinPairDistance(const std::vector<float>& srcXyz, const std::vector<float>& tgtXyz,
										  const double maxPairMm, const std::size_t maxSamples)
{
	if (srcXyz.size() < 9U || tgtXyz.size() < 9U || maxPairMm <= 0.0)
	{
		return 0U;
	}
	const pclalgo::KdTreePointSet tgtTree(tgtXyz);
	if (tgtTree.empty())
	{
		return 0U;
	}
	const double maxPairDistSq = maxPairMm * maxPairMm;
	const std::size_t nSrc = srcXyz.size() / 3U;
	const std::size_t sampleCount = std::min(nSrc, maxSamples);
	const std::size_t srcStride = std::max<std::size_t>(1U, nSrc / sampleCount);
	std::size_t hits = 0U;
	for (std::size_t i = 0; i < nSrc; i += srcStride)
	{
		const std::size_t b = i * 3U;
		double bestSq = maxPairDistSq;
		const std::size_t nnIdx =
			tgtTree.findNearest(static_cast<double>(srcXyz[b]), static_cast<double>(srcXyz[b + 1U]),
								static_cast<double>(srcXyz[b + 2U]), maxPairDistSq, bestSq);
		if (nnIdx != static_cast<std::size_t>(-1))
		{
			++hits;
		}
	}
	return hits;
}

void measureScanToCloudDistance(const std::vector<float>& scanXyz, const std::vector<float>& cloudXyz, double& outMaxMm,
								double& outAvgMm, const std::size_t maxScanSamples = 512U)
{
	outMaxMm = 0.0;
	outAvgMm = 0.0;
	if (scanXyz.size() < 9U || cloudXyz.size() < 9U)
	{
		return;
	}
	const pclalgo::KdTreePointSet cloudTree(cloudXyz);
	if (cloudTree.empty())
	{
		return;
	}
	const double unlimitedSq = std::numeric_limits<double>::max();
	const std::size_t nScan = scanXyz.size() / 3U;
	const std::size_t sampleCount = std::min(nScan, maxScanSamples);
	const std::size_t scanStride = std::max<std::size_t>(1U, nScan / std::max<std::size_t>(1U, sampleCount));
	double sumDist = 0.0;
	std::size_t count = 0U;
	for (std::size_t i = 0U; i < nScan; i += scanStride)
	{
		const std::size_t b = i * 3U;
		double bestSq = unlimitedSq;
		const std::size_t nnIdx =
			cloudTree.findNearest(static_cast<double>(scanXyz[b]), static_cast<double>(scanXyz[b + 1U]),
								  static_cast<double>(scanXyz[b + 2U]), unlimitedSq, bestSq);
		if (nnIdx == static_cast<std::size_t>(-1))
		{
			continue;
		}
		const double dist = std::sqrt(bestSq);
		outMaxMm = std::max(outMaxMm, dist);
		sumDist += dist;
		++count;
	}
	outAvgMm = count > 0U ? (sumDist / static_cast<double>(count)) : 0.0;
}

/// 仅统计距模板 soup 在 inlierGateMm 内的扫描点（与 ICP 重叠区一致，避免全云离群点拉高 maxDev）
void measureScanToCloudInlierStats(const std::vector<float>& scanXyz, const std::vector<float>& cloudXyz,
								   const double inlierGateMm, double& outInlierMaxMm, double& outInlierAvgMm,
								   std::size_t& outInlierHits, const std::size_t maxScanSamples = 512U)
{
	outInlierMaxMm = 0.0;
	outInlierAvgMm = 0.0;
	outInlierHits = 0U;
	if (scanXyz.size() < 9U || cloudXyz.size() < 9U || inlierGateMm <= 0.0)
	{
		return;
	}
	const double gateSq = inlierGateMm * inlierGateMm;
	const std::size_t nScan = scanXyz.size() / 3U;
	const std::size_t nCloud = cloudXyz.size() / 3U;
	const std::size_t sampleCount = std::min(nScan, maxScanSamples);
	const std::size_t scanStride = std::max<std::size_t>(1U, nScan / std::max<std::size_t>(1U, sampleCount));
	const std::size_t cloudStride = std::max<std::size_t>(1U, nCloud / 512U);
	double sumInlier = 0.0;
	for (std::size_t i = 0U; i < nScan; i += scanStride)
	{
		const std::size_t b = i * 3U;
		const Eigen::Vector3d query(scanXyz[b], scanXyz[b + 1U], scanXyz[b + 2U]);
		double bestSq = std::numeric_limits<double>::max();
		for (std::size_t j = 0U; j < nCloud; j += cloudStride)
		{
			const std::size_t cb = j * 3U;
			const double d2 =
				(query - Eigen::Vector3d(cloudXyz[cb], cloudXyz[cb + 1U], cloudXyz[cb + 2U])).squaredNorm();
			if (d2 < bestSq)
			{
				bestSq = d2;
			}
		}
		if (bestSq > gateSq)
		{
			continue;
		}
		const double dist = std::sqrt(bestSq);
		outInlierMaxMm = std::max(outInlierMaxMm, dist);
		sumInlier += dist;
		++outInlierHits;
	}
	outInlierAvgMm = outInlierHits > 0U ? (sumInlier / static_cast<double>(outInlierHits)) : 0.0;
}

constexpr std::size_t kMinRegistrationOverlapHits = 32U;
// 重叠采样命中足够时保留 globalPreAlign 结果，避免 reset 丢弃 PCA/RANSAC 偏航修正
constexpr std::size_t kMinResetOverlapHits = 96U;

enum class RegistrationAlignMode
{
	ManualTrusted,
	AutoRecover,
	ManualPartial,
	ColdStart,
};

const char* registrationAlignModeName(const RegistrationAlignMode mode)
{
	switch (mode)
	{
	case RegistrationAlignMode::ManualTrusted:
		return "manualTrusted";
	case RegistrationAlignMode::AutoRecover:
		return "autoRecover";
	case RegistrationAlignMode::ManualPartial:
		return "manualPartial";
	case RegistrationAlignMode::ColdStart:
		return "coldStart";
	default:
		return "unknown";
	}
}

RegistrationAlignMode resolveRegistrationAlignMode(const geoalgo::TemplateBrepUpdateParams& /*params*/,
												   const std::size_t initialOverlapHits, const double initialMaxDevMm,
												   const double preAlignGateMm)
{
	if (initialOverlapHits >= kMinRegistrationOverlapHits)
	{
		return RegistrationAlignMode::ManualTrusted;
	}
	if (initialOverlapHits == 0U)
	{
		return RegistrationAlignMode::AutoRecover;
	}
	if (initialMaxDevMm > preAlignGateMm * 2.0)
	{
		return RegistrationAlignMode::AutoRecover;
	}
	return RegistrationAlignMode::ManualPartial;
}

Eigen::Vector3d pointAtXyz(const std::vector<float>& xyz, const std::size_t i)
{
	const std::size_t b = i * 3U;
	return Eigen::Vector3d(xyz[b], xyz[b + 1U], xyz[b + 2U]);
}

double computeMatchedRmseMm(const std::vector<float>& srcXyz, const std::vector<float>& srcNormals,
							const std::vector<float>& tgtXyz, const std::vector<float>& tgtNormals,
							const Eigen::Isometry3d& transform, const double maxPairMm, const double maxNormalAngleDeg)
{
	if (srcXyz.size() < 9U || tgtXyz.size() < 9U || maxPairMm <= 0.0)
	{
		return 0.0;
	}

	const double maxPairDistSq = maxPairMm * maxPairMm;
	const double minNormalDot =
		maxNormalAngleDeg > 0.0 ? std::cos(maxNormalAngleDeg * 3.14159265358979323846 / 180.0) : -1.0;
	const bool useNormals = srcNormals.size() == srcXyz.size() && tgtNormals.size() == tgtXyz.size();
	const Eigen::Matrix3d rot = transform.linear();

	const std::size_t nSrc = srcXyz.size() / 3U;
	const std::size_t nTgt = tgtXyz.size() / 3U;
	const std::size_t srcStride = std::max<std::size_t>(1U, nSrc / 4000U);
	const std::size_t tgtStride = std::max<std::size_t>(1U, nTgt / 4000U);

	double sumSq = 0.0;
	std::size_t pairs = 0U;
	for (std::size_t i = 0; i < nSrc; i += srcStride)
	{
		const std::size_t b = i * 3U;
		const Eigen::Vector3d ps = transform * Eigen::Vector3d(srcXyz[b], srcXyz[b + 1U], srcXyz[b + 2U]);
		Eigen::Vector3d sn = Eigen::Vector3d::Zero();
		if (useNormals)
		{
			sn = rot * Eigen::Vector3d(srcNormals[b], srcNormals[b + 1U], srcNormals[b + 2U]);
			if (sn.norm() > 1e-12)
			{
				sn.normalize();
			}
		}

		double bestSq = maxPairDistSq;
		std::size_t bestJ = static_cast<std::size_t>(-1);
		Eigen::Vector3d tn = Eigen::Vector3d::Zero();
		for (std::size_t j = 0; j < nTgt; j += tgtStride)
		{
			const std::size_t tb = j * 3U;
			const Eigen::Vector3d qt(tgtXyz[tb], tgtXyz[tb + 1U], tgtXyz[tb + 2U]);
			const double d2 = (ps - qt).squaredNorm();
			if (d2 >= bestSq)
			{
				continue;
			}
			if (useNormals && minNormalDot > -0.999)
			{
				tn = Eigen::Vector3d(tgtNormals[tb], tgtNormals[tb + 1U], tgtNormals[tb + 2U]);
				if (tn.norm() > 1e-12)
				{
					tn.normalize();
				}
				if (sn.dot(tn) < minNormalDot)
				{
					continue;
				}
			}
			bestSq = d2;
			bestJ = j;
			if (useNormals)
			{
				tn = Eigen::Vector3d(tgtNormals[tb], tgtNormals[tb + 1U], tgtNormals[tb + 2U]);
			}
		}
		if (bestJ == static_cast<std::size_t>(-1))
		{
			continue;
		}
		if (useNormals && tn.norm() > 1e-12)
		{
			tn.normalize();
			const double d = std::abs(tn.dot(ps - pointAtXyz(tgtXyz, bestJ)));
			sumSq += d * d;
		}
		else
		{
			sumSq += bestSq;
		}
		++pairs;
	}
	return pairs > 0U ? std::sqrt(sumSq / static_cast<double>(pairs)) : 0.0;
}

std::unordered_map<int, BackendColor>
buildRefittedFaceHighlightColors(const std::vector<geoalgo::FaceUpdateReport>& perFace)
{
	static const BackendColor palette[] = {
		{0.92f, 0.25f, 0.22f, 1.0f}, {0.22f, 0.72f, 0.35f, 1.0f}, {0.25f, 0.45f, 0.92f, 1.0f},
		{0.95f, 0.62f, 0.12f, 1.0f}, {0.62f, 0.28f, 0.86f, 1.0f}, {0.18f, 0.78f, 0.78f, 1.0f},
		{0.88f, 0.38f, 0.62f, 1.0f}, {0.45f, 0.45f, 0.20f, 1.0f},
	};
	constexpr std::size_t paletteCount = sizeof(palette) / sizeof(palette[0]);

	std::unordered_map<int, BackendColor> out;
	std::size_t colorSlot = 0U;
	for (const geoalgo::FaceUpdateReport& report : perFace)
	{
		if (report.action == geoalgo::FaceUpdateAction::PlaneRefit ||
			report.action == geoalgo::FaceUpdateAction::CylinderRefit ||
			report.action == geoalgo::FaceUpdateAction::FreeformRefit)
		{
			out[report.faceIndex] = palette[colorSlot % paletteCount];
			++colorSlot;
		}
	}
	return out;
}

bool runReverseIcpStage(const char* stageLabel, std::vector<float>& templateSampleXyz,
						std::vector<float>& templateSampleNormals, const std::vector<float>& scanXyz,
						const std::vector<float>& scanNormals, const geoalgo::TemplateBrepUpdateParams& params,
						const double maxPairMm, const int maxIterations, Eigen::Isometry3d& outTemplateToScanStep,
						double& outRmseMm, std::string* errMsg, const double normalGateDegOverride = -1.0)
{
	outTemplateToScanStep = Eigen::Isometry3d::Identity();
	outRmseMm = 0.0;
	std::string stageErr;
	const std::vector<float>* tplNormalsPtr =
		(templateSampleNormals.size() == templateSampleXyz.size() && !templateSampleNormals.empty())
			? &templateSampleNormals
			: nullptr;
	const std::vector<float>* scanNormalsPtr =
		(scanNormals.size() == scanXyz.size() && !scanNormals.empty()) ? &scanNormals : nullptr;
	const double normalGateDeg = normalGateDegOverride >= 0.0
									 ? normalGateDegOverride
									 : (params.normalThresholdDeg > 0.0 ? params.normalThresholdDeg : 0.0);
	if (!pclalgo::rigidRegisterIcp(templateSampleXyz, scanXyz, outTemplateToScanStep, &outRmseMm, maxIterations, 0.01,
								   maxPairMm, params.icpMaxPoints, &stageErr, tplNormalsPtr, scanNormalsPtr,
								   normalGateDeg))
	{
		if (errMsg)
		{
			const std::size_t pairHits = countPointsWithinPairDistance(scanXyz, templateSampleXyz, maxPairMm, 512U);
			std::ostringstream oss;
			oss << stageLabel << ": " << (stageErr.empty() ? "reverse ICP failed" : stageErr)
				<< " [maxPairMm=" << maxPairMm << " pairHits=" << pairHits << "/512]";
			*errMsg = oss.str();
		}
		return false;
	}
	applyIsometryInPlace(templateSampleXyz, outTemplateToScanStep);
	if (!templateSampleNormals.empty())
	{
		applyIsometryInPlaceNormals(templateSampleNormals, outTemplateToScanStep);
	}
	return true;
}

bool runReversePointToPlaneIcpStage(const char* stageLabel, std::vector<float>& templateSampleXyz,
									std::vector<float>& templateSampleNormals, const std::vector<float>& scanXyz,
									const std::vector<float>& scanNormals,
									const geoalgo::TemplateBrepUpdateParams& params, const double maxPairMm,
									const int maxIterations, Eigen::Isometry3d& outTemplateToScanStep,
									double& outRmseMm, std::string* errMsg, const double normalGateDegOverride = -1.0)
{
	outTemplateToScanStep = Eigen::Isometry3d::Identity();
	outRmseMm = 0.0;
	if (templateSampleNormals.empty() || scanNormals.empty())
	{
		return runReverseIcpStage(stageLabel, templateSampleXyz, templateSampleNormals, scanXyz, scanNormals, params,
								  maxPairMm, maxIterations, outTemplateToScanStep, outRmseMm, errMsg,
								  normalGateDegOverride >= 0.0 ? normalGateDegOverride : 0.0);
	}
	std::string stageErr;
	const double normalGateDeg = normalGateDegOverride >= 0.0
									 ? normalGateDegOverride
									 : (params.normalThresholdDeg > 0.0 ? params.normalThresholdDeg : 0.0);
	if (!pclalgo::rigidRegisterPointToPlaneIcp(templateSampleXyz, templateSampleNormals, scanXyz, scanNormals,
											   outTemplateToScanStep, &outRmseMm, maxIterations, 0.005, maxPairMm,
											   params.icpMaxPoints, &stageErr, normalGateDeg))
	{
		if (errMsg)
		{
			*errMsg =
				std::string(stageLabel) + ": " + (stageErr.empty() ? "reverse point-to-plane ICP failed" : stageErr);
		}
		return false;
	}
	applyIsometryInPlace(templateSampleXyz, outTemplateToScanStep);
	applyIsometryInPlaceNormals(templateSampleNormals, outTemplateToScanStep);
	return true;
}

bool runIcpStage(const char* stageLabel, std::vector<float>& workXyz, std::vector<float>& workNormals,
				 const std::vector<float>& templateSampleXyz, const std::vector<float>* templateSampleNormals,
				 const geoalgo::TemplateBrepUpdateParams& params, const double maxPairMm, const int maxIterations,
				 Eigen::Isometry3d& outStep, double& outRmseMm, std::string* errMsg,
				 const double normalGateDegOverride = -1.0)
{
	outStep = Eigen::Isometry3d::Identity();
	outRmseMm = 0.0;
	std::string stageErr;
	const std::vector<float>* srcNormalsPtr =
		(workNormals.size() == workXyz.size() && !workNormals.empty()) ? &workNormals : nullptr;
	const std::vector<float>* tgtNormalsPtr =
		(templateSampleNormals != nullptr && templateSampleNormals->size() == templateSampleXyz.size() &&
		 !templateSampleNormals->empty())
			? templateSampleNormals
			: nullptr;
	const double normalGateDeg = normalGateDegOverride >= 0.0
									 ? normalGateDegOverride
									 : (params.normalThresholdDeg > 0.0 ? params.normalThresholdDeg : 0.0);
	if (!pclalgo::rigidRegisterIcp(workXyz, templateSampleXyz, outStep, &outRmseMm, maxIterations, 0.01, maxPairMm,
								   params.icpMaxPoints, &stageErr, srcNormalsPtr, tgtNormalsPtr, normalGateDeg))
	{
		if (errMsg)
		{
			const std::size_t pairHits = countPointsWithinPairDistance(workXyz, templateSampleXyz, maxPairMm, 512U);
			std::ostringstream oss;
			oss << stageLabel << ": " << (stageErr.empty() ? "ICP failed" : stageErr) << " [maxPairMm=" << maxPairMm
				<< " pairHits=" << pairHits << "/512]";
			*errMsg = oss.str();
		}
		return false;
	}
	applyIsometryInPlace(workXyz, outStep);
	if (!workNormals.empty())
	{
		applyIsometryInPlaceNormals(workNormals, outStep);
	}
	return true;
}

bool runPointToPlaneIcpStage(const char* stageLabel, std::vector<float>& workXyz, std::vector<float>& workNormals,
							 const std::vector<float>& templateSampleXyz,
							 const std::vector<float>& templateSampleNormals,
							 const geoalgo::TemplateBrepUpdateParams& params, const double maxPairMm,
							 const int maxIterations, Eigen::Isometry3d& outStep, double& outRmseMm,
							 std::string* errMsg, const double normalGateDegOverride = -1.0)
{
	outStep = Eigen::Isometry3d::Identity();
	outRmseMm = 0.0;
	if (workNormals.empty() || templateSampleNormals.empty())
	{
		return runIcpStage(stageLabel, workXyz, workNormals, templateSampleXyz, nullptr, params, maxPairMm,
						   maxIterations, outStep, outRmseMm, errMsg,
						   normalGateDegOverride >= 0.0 ? normalGateDegOverride : 0.0);
	}
	std::string stageErr;
	const double normalGateDeg = normalGateDegOverride >= 0.0
									 ? normalGateDegOverride
									 : (params.normalThresholdDeg > 0.0 ? params.normalThresholdDeg : 0.0);
	if (!pclalgo::rigidRegisterPointToPlaneIcp(workXyz, workNormals, templateSampleXyz, templateSampleNormals, outStep,
											   &outRmseMm, maxIterations, 0.005, maxPairMm, params.icpMaxPoints,
											   &stageErr, normalGateDeg))
	{
		if (errMsg)
		{
			*errMsg = std::string(stageLabel) + ": " + (stageErr.empty() ? "point-to-plane ICP failed" : stageErr);
		}
		return false;
	}
	applyIsometryInPlace(workXyz, outStep);
	applyIsometryInPlaceNormals(workNormals, outStep);
	return true;
}

bool computeCloudPcaFrame(const std::vector<float>& xyz, Eigen::Vector3d& outCentroid, Eigen::Matrix3d& outAxes)
{
	const std::size_t n = xyz.size() / 3U;
	if (n < 3U)
	{
		return false;
	}
	outCentroid = Eigen::Vector3d::Zero();
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		outCentroid += Eigen::Vector3d(xyz[b], xyz[b + 1U], xyz[b + 2U]);
	}
	outCentroid /= static_cast<double>(n);

	Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
	for (std::size_t i = 0; i < n; i += std::max<std::size_t>(1U, n / 8000U))
	{
		const std::size_t b = i * 3U;
		const Eigen::Vector3d q(xyz[b], xyz[b + 1U], xyz[b + 2U]);
		const Eigen::Vector3d d = q - outCentroid;
		cov += d * d.transpose();
	}
	const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
	if (es.info() != Eigen::Success)
	{
		return false;
	}
	outAxes.col(0) = es.eigenvectors().col(2).normalized();
	outAxes.col(1) = es.eigenvectors().col(1).normalized();
	outAxes.col(2) = outAxes.col(0).cross(outAxes.col(1)).normalized();
	return true;
}

double meanNearestDistanceAfterTransform(const std::vector<float>& srcXyz, const std::vector<float>& tgtXyz,
										 const Eigen::Isometry3d& transform, const std::size_t maxSamples)
{
	const std::size_t nSrc = srcXyz.size() / 3U;
	if (nSrc == 0U || tgtXyz.size() < 9U)
	{
		return std::numeric_limits<double>::max();
	}
	const pclalgo::KdTreePointSet tgtTree(tgtXyz);
	if (tgtTree.empty())
	{
		return std::numeric_limits<double>::max();
	}
	const double unlimitedSq = std::numeric_limits<double>::max();
	const std::size_t step = std::max<std::size_t>(1U, nSrc / std::max<std::size_t>(1U, maxSamples));
	double sum = 0.0;
	std::size_t count = 0U;
	for (std::size_t i = 0; i < nSrc; i += step)
	{
		const std::size_t b = i * 3U;
		const Eigen::Vector3d p = transform * Eigen::Vector3d(srcXyz[b], srcXyz[b + 1U], srcXyz[b + 2U]);
		double bestSq = unlimitedSq;
		const std::size_t nnIdx = tgtTree.findNearest(p.x(), p.y(), p.z(), unlimitedSq, bestSq);
		if (nnIdx == static_cast<std::size_t>(-1))
		{
			continue;
		}
		sum += std::sqrt(bestSq);
		++count;
	}
	return count > 0U ? (sum / static_cast<double>(count)) : std::numeric_limits<double>::max();
}

double rotationAngleDeg(const Eigen::Isometry3d& transform);

struct PcaAlignmentScore
{
	double maxDevMm = std::numeric_limits<double>::max();
	double meanDistMm = std::numeric_limits<double>::max();
	std::size_t pairHits = 0U;
};

PcaAlignmentScore scoreTemplatePcaCandidate(const std::vector<float>& templateXyz, const std::vector<float>& scanXyz,
											const Eigen::Isometry3d& candidate, const double gateMm)
{
	PcaAlignmentScore score;
	std::vector<float> trial = templateXyz;
	applyIsometryInPlace(trial, candidate);
	measureScanToCloudDistance(scanXyz, trial, score.maxDevMm, score.meanDistMm);
	score.pairHits = countPointsWithinPairDistance(scanXyz, trial, gateMm, 512U);
	return score;
}

bool pcaAlignmentScoreBetter(const PcaAlignmentScore& candidate, const PcaAlignmentScore& best)
{
	if (best.maxDevMm >= std::numeric_limits<double>::max() * 0.5)
	{
		return true;
	}
	if (candidate.maxDevMm + 3.0 < best.maxDevMm)
	{
		return true;
	}
	if (best.maxDevMm + 3.0 < candidate.maxDevMm)
	{
		return false;
	}
	if (candidate.pairHits >= kMinRegistrationOverlapHits && best.pairHits < kMinRegistrationOverlapHits)
	{
		return true;
	}
	if (best.pairHits >= kMinRegistrationOverlapHits && candidate.pairHits < kMinRegistrationOverlapHits)
	{
		return false;
	}
	if (candidate.pairHits != best.pairHits)
	{
		return candidate.pairHits > best.pairHits;
	}
	return candidate.meanDistMm < best.meanDistMm;
}

bool pcaStrongOverlapAccept(const PcaAlignmentScore& pcaScore, const PcaAlignmentScore& priorScore,
							const double initialMaxDevMm)
{
	return pcaScore.pairHits >= kMinRegistrationOverlapHits && pcaScore.pairHits >= priorScore.pairHits + 32U &&
		   pcaScore.maxDevMm < initialMaxDevMm * 0.2;
}

bool tryCoarseQuarterTurnAlignment(const std::vector<float>& workXyz, std::vector<float>& templateSoupXyz,
								   std::vector<float>& templateSoupNormals, const geoalgo::ShapeHandle& templateShape,
								   geoalgo::ShapeHandle& workingTemplate, const double preAlignGateMm,
								   Eigen::Isometry3d& inOutTemplateToScan, PcaAlignmentScore& ioScore)
{
	const Eigen::AlignedBox3d scanBox = pclalgo::computeBoundingBox(workXyz);
	if (scanBox.isEmpty())
	{
		return false;
	}
	const Eigen::Vector3d pivot = scanBox.center();

	auto rotationAboutPivot = [&](const Eigen::Vector3d& axisUnit, const double deg)
	{
		Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
		step.linear() =
			Eigen::AngleAxisd(deg * 3.14159265358979323846 / 180.0, axisUnit.normalized()).toRotationMatrix();
		step.translation() = pivot - step.linear() * pivot;
		return step;
	};

	PcaAlignmentScore bestCandidate = ioScore;
	Eigen::Isometry3d bestStep = Eigen::Isometry3d::Identity();
	bool found = false;

	const Eigen::Vector3d axes[] = {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()};
	const double degs[] = {90.0, 180.0, 270.0};

	for (const Eigen::Vector3d& axis : axes)
	{
		for (const double deg : degs)
		{
			const Eigen::Isometry3d step = rotationAboutPivot(axis, deg);
			const PcaAlignmentScore candidate =
				scoreTemplatePcaCandidate(templateSoupXyz, workXyz, step, preAlignGateMm);
			if (pcaAlignmentScoreBetter(candidate, bestCandidate))
			{
				bestCandidate = candidate;
				bestStep = step;
				found = true;
			}
		}
	}

	if (!found || !pcaAlignmentScoreBetter(bestCandidate, ioScore))
	{
		if (found)
		{
			RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=quarterTurn rejected maxDevMm=") +
							std::to_string(bestCandidate.maxDevMm) +
							" pairHits=" + std::to_string(bestCandidate.pairHits) + "/512");
		}
		return false;
	}

	const std::vector<float> soupBefore = templateSoupXyz;
	const std::vector<float> normalsBefore = templateSoupNormals;
	const Eigen::Isometry3d transformBefore = inOutTemplateToScan;
	const geoalgo::ShapeHandle templateBefore = workingTemplate.clone();

	applyIsometryInPlace(templateSoupXyz, bestStep);
	if (!templateSoupNormals.empty())
	{
		applyIsometryInPlaceNormals(templateSoupNormals, bestStep);
	}
	inOutTemplateToScan = bestStep * inOutTemplateToScan;
	std::string transformErr;
	if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, bestStep, workingTemplate, &transformErr))
	{
		templateSoupXyz = soupBefore;
		templateSoupNormals = normalsBefore;
		inOutTemplateToScan = transformBefore;
		workingTemplate = templateBefore;
		return false;
	}

	ioScore = bestCandidate;
	const double rotDeg = rotationAngleDeg(bestStep);
	RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=quarterTurn ok pairHits=") +
					std::to_string(bestCandidate.pairHits) + "/512 maxDevMm=" + std::to_string(bestCandidate.maxDevMm) +
					" rotDeg=" + std::to_string(rotDeg));
	return true;
}

bool coarseOverlapQualityPoor(const std::size_t pairHits, const double maxDevMm, const double maxDevGateMm)
{
	return pairHits < kMinRegistrationOverlapHits || maxDevMm > maxDevGateMm;
}

bool tryCoarseFeatureRansacAlign(const char* stageLabel, const std::vector<float>& workXyz,
								 const std::vector<float>& workNormals, std::vector<float>& templateSoupXyz,
								 std::vector<float>& templateSoupNormals, const geoalgo::ShapeHandle& templateShape,
								 geoalgo::ShapeHandle& workingTemplate, const geoalgo::TemplateBrepUpdateParams& params,
								 const double modelDiag, const double preAlignGateMm, const double baselineMaxDevMm,
								 Eigen::Isometry3d& inOutTemplateToScan, PcaAlignmentScore& ioScore,
								 std::string* errMsg)
{
	if (!params.enableRansacCoarseMatch)
	{
		return false;
	}

	const double fb = std::max(params.faceBandMm, 1.0);
	pclalgo::RigidRegisterRansacParams ransacParams;
	ransacParams.maxFeaturePoints = params.icpMaxPoints;
	ransacParams.modelDiagMm = modelDiag;
	ransacParams.faceBandMm = params.faceBandMm;
	ransacParams.maxIterations = params.ransacMaxIterations;
	ransacParams.inlierDistanceMm =
		std::min(std::max({preAlignGateMm * 1.25, fb * 4.0, baselineMaxDevMm * 0.45}), modelDiag * 0.12);
	ransacParams.featureVoxelMm = std::max(modelDiag * 0.015, fb * 2.0);
	ransacParams.skipTranslationCap = true;
	const double transCapMm = std::max(modelDiag * 0.25, preAlignGateMm * 2.0);

	struct RansacCandidate
	{
		Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
		PcaAlignmentScore score;
		double inlierRatio = 0.0;
		bool reverse = false;
	};

	auto scoreRansacStep = [&](const Eigen::Isometry3d& step, PcaAlignmentScore& outScore)
	{
		std::vector<float> trialSoup = templateSoupXyz;
		std::vector<float> trialNormals = templateSoupNormals;
		applyIsometryInPlace(trialSoup, step);
		if (!trialNormals.empty())
		{
			applyIsometryInPlaceNormals(trialNormals, step);
		}
		measureScanToCloudDistance(workXyz, trialSoup, outScore.maxDevMm, outScore.meanDistMm);
		outScore.pairHits = countPointsWithinPairDistance(workXyz, trialSoup, preAlignGateMm, 512U);
	};

	std::vector<RansacCandidate> candidates;
	auto tryDirection = [&](const bool reverse)
	{
		Eigen::Isometry3d rawStep = Eigen::Isometry3d::Identity();
		double inlierRatio = 0.0;
		std::string ransacErr;
		const bool ok =
			reverse ? pclalgo::rigidRegisterFeatureRansac(workXyz, workNormals, templateSoupXyz, templateSoupNormals,
														  rawStep, &inlierRatio, ransacParams, &ransacErr)
					: pclalgo::rigidRegisterFeatureRansac(templateSoupXyz, templateSoupNormals, workXyz, workNormals,
														  rawStep, &inlierRatio, ransacParams, &ransacErr);
		if (!ok)
		{
			RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=") + stageLabel +
							(reverse ? " reverse ransac failed: " : " ransac failed: ") +
							(ransacErr.empty() ? "unknown" : ransacErr));
			return;
		}
		const Eigen::Isometry3d step = reverse ? rawStep.inverse() : rawStep;
		RansacCandidate candidate;
		candidate.step = step;
		candidate.inlierRatio = inlierRatio;
		candidate.reverse = reverse;
		scoreRansacStep(step, candidate.score);
		candidates.push_back(std::move(candidate));
	};

	tryDirection(false);
	tryDirection(true);
	if (candidates.empty())
	{
		return false;
	}

	const RansacCandidate* bestCandidate = &candidates.front();
	for (const RansacCandidate& candidate : candidates)
	{
		if (pcaAlignmentScoreBetter(candidate.score, bestCandidate->score))
		{
			bestCandidate = &candidate;
		}
	}
	if (!pcaAlignmentScoreBetter(bestCandidate->score, ioScore))
	{
		return false;
	}
	if (bestCandidate->step.translation().norm() > transCapMm && bestCandidate->score.maxDevMm > preAlignGateMm * 2.0)
	{
		RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=") + stageLabel +
						" ransac best rejected transMm=" + std::to_string(bestCandidate->step.translation().norm()) +
						" maxDevMm=" + std::to_string(bestCandidate->score.maxDevMm));
		return false;
	}

	const std::vector<float> soupBefore = templateSoupXyz;
	const std::vector<float> normalsBefore = templateSoupNormals;
	const Eigen::Isometry3d transformBefore = inOutTemplateToScan;
	const geoalgo::ShapeHandle templateBefore = workingTemplate.clone();

	applyIsometryInPlace(templateSoupXyz, bestCandidate->step);
	applyIsometryInPlaceNormals(templateSoupNormals, bestCandidate->step);
	inOutTemplateToScan = bestCandidate->step * inOutTemplateToScan;
	std::string transformErr;
	if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, bestCandidate->step, workingTemplate, &transformErr))
	{
		templateSoupXyz = soupBefore;
		templateSoupNormals = normalsBefore;
		inOutTemplateToScan = transformBefore;
		workingTemplate = templateBefore;
		if (errMsg)
		{
			*errMsg = transformErr.empty() ? "RANSAC template transform failed" : transformErr;
		}
		return false;
	}

	ioScore = bestCandidate->score;
	const double rotDeg = rotationAngleDeg(bestCandidate->step);
	RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=") + stageLabel +
					(bestCandidate->reverse ? " reverse ransac ok " : " ransac ok ") + "inlierRatio=" +
					std::to_string(bestCandidate->inlierRatio) + " pairHits=" + std::to_string(ioScore.pairHits) +
					"/512 maxDevMm=" + std::to_string(ioScore.maxDevMm) + " transMm=" +
					std::to_string(bestCandidate->step.translation().norm()) + " rotDeg=" + std::to_string(rotDeg));
	return true;
}

void normalizeCoarseTemplateToScanMatrix(const std::vector<float>& originalSoupXyz,
										 const std::vector<float>& finalSoupXyz, const std::vector<float>& scanXyz,
										 const double modelDiag, const double preAlignGateMm,
										 Eigen::Isometry3d& inOutTemplateToScan)
{
	const double accumTransMm = inOutTemplateToScan.translation().norm();
	const double transCapMm = ransacTranslationCapMm(modelDiag, preAlignGateMm, accumTransMm);
	if (accumTransMm <= std::max(modelDiag * 0.25, preAlignGateMm * 2.0))
	{
		return;
	}
	if (originalSoupXyz.size() >= 9U && originalSoupXyz.size() == finalSoupXyz.size() && scanXyz.size() >= 9U)
	{
		Eigen::Isometry3d estimated = Eigen::Isometry3d::Identity();
		double icpRmse = 0.0;
		const double maxPairMm = std::max({modelDiag * 0.6, accumTransMm * 1.25, preAlignGateMm * 4.0, 50.0});
		if (pclalgo::rigidRegisterIcp(originalSoupXyz, finalSoupXyz, estimated, &icpRmse, 25, 0.01, maxPairMm, 60000U,
									  nullptr, nullptr, nullptr, 0.0))
		{
			std::vector<float> verifyXyz = originalSoupXyz;
			applyIsometryInPlace(verifyXyz, estimated);
			double scanMaxMm = 0.0;
			double scanAvgMm = 0.0;
			measureScanToCloudDistance(scanXyz, verifyXyz, scanMaxMm, scanAvgMm);
			const std::size_t scanHits = countPointsWithinPairDistance(scanXyz, verifyXyz, preAlignGateMm, 512U);
			const double estimatedTransMm = estimated.translation().norm();
			const double verifyGateMm = std::max({preAlignGateMm * 1.5, modelDiag * 0.03, 20.0});
			const bool strictOverlapOk =
				scanHits >= kMinRegistrationOverlapHits / 2U && scanMaxMm <= verifyGateMm * 2.0;
			const bool relaxedOverlapOk =
				scanHits >= kMinRegistrationOverlapHits / 2U &&
				scanMaxMm <= std::max({verifyGateMm * 3.0, preAlignGateMm * 5.0, verifyGateMm * 2.0 + 25.0});
			const bool overlapOk = strictOverlapOk || relaxedOverlapOk;
			if (overlapOk && estimatedTransMm <= transCapMm)
			{
				inOutTemplateToScan = estimated;
				RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=templateToScan normalized transMm=") +
								std::to_string(estimatedTransMm) + " scanMaxMm=" + std::to_string(scanMaxMm));
				return;
			}
		}
	}
	// originalPose 写回：显示层用 pose 承载 ICP，不能钳为 Identity
	RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=templateToScan keep pipeline matrix (was ") +
					std::to_string(accumTransMm) + "mm); normalize verify failed");
}

void reconcileTemplateToScanWithSoup(const geoalgo::ShapeHandle& templateShape,
									 const std::vector<float>& originalSoupXyz, const std::vector<float>& finalSoupXyz,
									 const std::vector<float>& scanXyz, const double modelDiag,
									 const double preAlignGateMm, Eigen::Isometry3d& inOutTemplateToScan,
									 geoalgo::ShapeHandle& inOutAlignedTemplateShape)
{
	(void)templateShape;
	(void)inOutAlignedTemplateShape;
	if (originalSoupXyz.size() < 9U || originalSoupXyz.size() != finalSoupXyz.size() || scanXyz.size() < 9U)
	{
		return;
	}
	Eigen::Isometry3d estimated = Eigen::Isometry3d::Identity();
	double icpRmse = 0.0;
	const double accumTransMm = inOutTemplateToScan.translation().norm();
	const double maxPairMm = std::max({modelDiag * 0.6, accumTransMm * 1.25, preAlignGateMm * 4.0, 50.0});
	if (!pclalgo::rigidRegisterIcp(originalSoupXyz, finalSoupXyz, estimated, &icpRmse, 25, 0.01, maxPairMm, 60000U,
								   nullptr, nullptr, nullptr, 0.0))
	{
		return;
	}
	std::vector<float> verifyXyz = originalSoupXyz;
	applyIsometryInPlace(verifyXyz, estimated);
	double scanMaxMm = 0.0;
	double scanAvgMm = 0.0;
	measureScanToCloudDistance(scanXyz, verifyXyz, scanMaxMm, scanAvgMm);
	const std::size_t scanHits = countPointsWithinPairDistance(scanXyz, verifyXyz, preAlignGateMm, 512U);
	const double estimatedTransMm = estimated.translation().norm();
	const double transDeltaMm = (inOutTemplateToScan.translation() - estimated.translation()).norm();
	const double estimatedRotDeg = rotationAngleDeg(estimated);
	const double verifyGateMm = std::max({preAlignGateMm * 1.5, modelDiag * 0.03, 20.0});
	const double transCapMm = ransacTranslationCapMm(modelDiag, preAlignGateMm, accumTransMm);
	const bool strictOverlapOk = scanHits >= kMinRegistrationOverlapHits / 2U && scanMaxMm <= verifyGateMm * 2.0;
	// 粗配后 soup 已贴近 scan 时，original→final 的 ICP 估计比逐步累乘更可信
	const bool relaxedOverlapOk =
		scanHits >= kMinRegistrationOverlapHits / 2U &&
		scanMaxMm <= std::max({verifyGateMm * 3.0, preAlignGateMm * 5.0, verifyGateMm * 2.0 + 25.0});
	const bool overlapOk = strictOverlapOk || relaxedOverlapOk;
	if (!overlapOk || estimatedTransMm > transCapMm)
	{
		RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=reconcile rejected scanMaxMm=") +
						std::to_string(scanMaxMm) + " scanHits=" + std::to_string(scanHits) +
						"/512 transMm=" + std::to_string(estimatedTransMm) + " (cap=" + std::to_string(transCapMm) +
						"mm); keep pipeline templateToScan");
		return;
	}
	RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=reconcile templateToScan accumTransMm=") +
					std::to_string(accumTransMm) + " estimatedTransMm=" + std::to_string(estimatedTransMm) +
					" deltaMm=" + std::to_string(transDeltaMm) + " scanMaxMm=" + std::to_string(scanMaxMm) +
					" scanHits=" + std::to_string(scanHits) + "/512 rotDeg=" + std::to_string(estimatedRotDeg) +
					(relaxedOverlapOk && !strictOverlapOk ? " (relaxed overlap gate)" : ""));
	inOutTemplateToScan = estimated;
}

Eigen::Isometry3d alignTemplatePcaToScan(std::vector<float>& templateSampleXyz,
										 std::vector<float>& templateSampleNormals, const std::vector<float>& scanXyz)
{
	Eigen::Vector3d scanCentroid = Eigen::Vector3d::Zero();
	Eigen::Vector3d templateCentroid = Eigen::Vector3d::Zero();
	Eigen::Matrix3d scanAxes = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d templateAxes = Eigen::Matrix3d::Identity();
	if (!computeCloudPcaFrame(scanXyz, scanCentroid, scanAxes) ||
		!computeCloudPcaFrame(templateSampleXyz, templateCentroid, templateAxes))
	{
		return Eigen::Isometry3d::Identity();
	}

	const double modelDiag = std::max(boundingBoxDiagonalMm(scanXyz), boundingBoxDiagonalMm(templateSampleXyz));
	const double gateMm = std::max({modelDiag * 0.02, 15.0});

	Eigen::Isometry3d best = Eigen::Isometry3d::Identity();
	PcaAlignmentScore bestScore;
	const int axisPermutations[6][3] = {
		{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0},
	};
	for (const int(&perm)[3] : axisPermutations)
	{
		Eigen::Matrix3d permutedTemplateAxes;
		for (int col = 0; col < 3; ++col)
		{
			permutedTemplateAxes.col(col) = templateAxes.col(perm[col]);
		}
		if (permutedTemplateAxes.determinant() < 0.0)
		{
			continue;
		}
		for (const int sx : {-1, 1})
		{
			for (const int sy : {-1, 1})
			{
				for (const int sz : {-1, 1})
				{
					Eigen::Matrix3d signedTemplateAxes =
						permutedTemplateAxes * Eigen::DiagonalMatrix<double, 3>(sx, sy, sz);
					Eigen::Matrix3d rot = scanAxes * signedTemplateAxes.transpose();
					if (rot.determinant() < 0.0)
					{
						continue;
					}
					Eigen::Isometry3d candidate = Eigen::Isometry3d::Identity();
					candidate.linear() = rot;
					candidate.translation() = scanCentroid - rot * templateCentroid;
					const PcaAlignmentScore candidateScore =
						scoreTemplatePcaCandidate(templateSampleXyz, scanXyz, candidate, gateMm);
					if (pcaAlignmentScoreBetter(candidateScore, bestScore))
					{
						bestScore = candidateScore;
						best = candidate;
					}
				}
			}
		}
	}

	const double rotDeg = rotationAngleDeg(best);
	RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=pca selected pairHits=") +
					std::to_string(bestScore.pairHits) + "/512 maxDevMm=" + std::to_string(bestScore.maxDevMm) +
					" meanDistMm=" + std::to_string(bestScore.meanDistMm) + " rotDeg=" + std::to_string(rotDeg));

	applyIsometryInPlace(templateSampleXyz, best);
	if (!templateSampleNormals.empty())
	{
		applyIsometryInPlaceNormals(templateSampleNormals, best);
	}
	return best;
}

Eigen::Isometry3d alignScanPcaToTemplate(std::vector<float>& workXyz, std::vector<float>& workNormals,
										 const std::vector<float>& templateSampleXyz)
{
	Eigen::Vector3d scanCentroid = Eigen::Vector3d::Zero();
	Eigen::Vector3d templateCentroid = Eigen::Vector3d::Zero();
	Eigen::Matrix3d scanAxes = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d templateAxes = Eigen::Matrix3d::Identity();
	if (!computeCloudPcaFrame(workXyz, scanCentroid, scanAxes) ||
		!computeCloudPcaFrame(templateSampleXyz, templateCentroid, templateAxes))
	{
		return Eigen::Isometry3d::Identity();
	}

	Eigen::Isometry3d best = Eigen::Isometry3d::Identity();
	double bestScore = std::numeric_limits<double>::max();
	for (const int sx : {-1, 1})
	{
		for (const int sy : {-1, 1})
		{
			for (const int sz : {-1, 1})
			{
				Eigen::Matrix3d signedScanAxes = scanAxes * Eigen::DiagonalMatrix<double, 3>(sx, sy, sz);
				Eigen::Matrix3d rot = templateAxes * signedScanAxes.transpose();
				if (rot.determinant() < 0.0)
				{
					continue;
				}
				Eigen::Isometry3d candidate = Eigen::Isometry3d::Identity();
				candidate.linear() = rot;
				candidate.translation() = templateCentroid - rot * scanCentroid;
				const double score = meanNearestDistanceAfterTransform(workXyz, templateSampleXyz, candidate, 600U);
				if (score < bestScore)
				{
					bestScore = score;
					best = candidate;
				}
			}
		}
	}

	applyIsometryInPlace(workXyz, best);
	if (!workNormals.empty())
	{
		applyIsometryInPlaceNormals(workNormals, best);
	}
	return best;
}

double rotationAngleDeg(const Eigen::Isometry3d& transform)
{
	Eigen::AngleAxisd aa(transform.linear());
	return aa.angle() * 180.0 / 3.14159265358979323846;
}

bool runReverseSoupMultiStageIcp(const std::vector<float>& workXyz, const std::vector<float>& workNormals,
								 std::vector<float>& templateSoupXyz, std::vector<float>& templateSoupNormals,
								 geoalgo::ShapeHandle& workingTemplate, const geoalgo::TemplateBrepUpdateParams& params,
								 const double modelDiag, Eigen::Isometry3d& outTemplateToScan,
								 geoalgo::ShapeHandle& outAlignedTemplateShape, double& outRmseMm,
								 const bool allowLargeCorrection, const bool fineRefinementStage, bool& outSoupApplied)
{
	const Eigen::Isometry3d priorStep = outTemplateToScan;
	outTemplateToScan = Eigen::Isometry3d::Identity();
	outRmseMm = 0.0;
	outSoupApplied = false;
	const double fb = std::max(params.faceBandMm, 1.0);

	const geoalgo::ShapeHandle originalTemplate = workingTemplate.clone();
	const std::vector<float> originalSoupXyz = templateSoupXyz;
	const std::vector<float> originalSoupNormals = templateSoupNormals;

	double baselineAvgDist = 0.0;
	double baselineMaxDist = 0.0;
	measureScanToCloudDistance(workXyz, templateSoupXyz, baselineMaxDist, baselineAvgDist);
	const double coarseInlierGateMm = params.registrationMatchVoxelMm > 0.0
										  ? std::max({params.registrationMatchVoxelMm * 2.5, fb * 3.0, 15.0})
										  : std::max({fb * 6.0, modelDiag * 0.02, 15.0});
	double baselineInlierMax = 0.0;
	double baselineInlierAvg = 0.0;
	std::size_t baselineInlierHits = 0U;
	double inlierGateMm = coarseInlierGateMm;
	if (fineRefinementStage)
	{
		measureScanToCloudInlierStats(workXyz, templateSoupXyz, coarseInlierGateMm, baselineInlierMax,
									  baselineInlierAvg, baselineInlierHits);
		inlierGateMm = fineOverlapInlierGateMm(params, baselineInlierMax);
		if (baselineInlierHits > 0U)
		{
			measureScanToCloudInlierStats(workXyz, templateSoupXyz, inlierGateMm, baselineInlierMax, baselineInlierAvg,
										  baselineInlierHits);
		}
	}
	RunLogger::info(std::string("[TemplateBrepUpdate] reverse pre-align baseline maxDevMm=") +
					std::to_string(baselineMaxDist) + " avgDevMm=" + std::to_string(baselineAvgDist) +
					(fineRefinementStage ? " inlierHits=" + std::to_string(baselineInlierHits) +
											   "/512 inlierAvgMm=" + std::to_string(baselineInlierAvg)
										 : ""));

	std::vector<double> maxPairStages;
	if (fineRefinementStage && params.registrationMatchVoxelMm > 0.0)
	{
		const double v = params.registrationMatchVoxelMm;
		const double finePairCap = baselineInlierMax > 0.0 ? std::min({baselineInlierMax * 1.8, v * 3.0, fb * 5.0})
														   : std::min({baselineMaxDist * 0.4, v * 3.0, fb * 5.0});
		for (const double mult : {1.0, 1.5, 2.0, 2.5})
		{
			maxPairStages.push_back(std::min(std::max(v * mult, 1e-9), finePairCap));
		}
	}
	else if (params.registrationMatchVoxelMm > 0.0)
	{
		const double v = params.registrationMatchVoxelMm;
		maxPairStages.push_back(std::max(v * 5.0, 1e-9));
		maxPairStages.push_back(std::max(v * 2.5, 1e-9));
		maxPairStages.push_back(std::max(v * 1.5, 1e-9));
		maxPairStages.push_back(std::max(v * 1.0, 1e-9));
	}
	else if (allowLargeCorrection && baselineMaxDist > modelDiag * 0.1)
	{
		maxPairStages.push_back(std::min(baselineMaxDist * 0.95, modelDiag * 0.4));
	}
	else if (allowLargeCorrection && baselineMaxDist > fb * 10.0)
	{
		maxPairStages.push_back(std::min(baselineMaxDist * 0.85, modelDiag * 0.25));
	}
	if (params.registrationMatchVoxelMm <= 0.0)
	{
		if (baselineMaxDist > fb * 6.0)
		{
			maxPairStages.push_back(std::max(fb * 10.0, 8.0));
		}
		maxPairStages.push_back(std::max(fb * 6.0, 3.0));
		maxPairStages.push_back(std::max(fb * 3.0, 2.0));
		maxPairStages.push_back(std::max(fb * 1.5, 1.5));
	}

	Eigen::Isometry3d cumulative = Eigen::Isometry3d::Identity();
	double bestRmse = baselineMaxDist;
	bool anyStageOk = false;

	for (std::size_t stage = 0; stage < maxPairStages.size(); ++stage)
	{
		Eigen::Isometry3d stageStep = Eigen::Isometry3d::Identity();
		double stageRmse = 0.0;
		std::string stageErr;
		const int maxIter = fineRefinementStage ? 60 : ((stage + 1U == maxPairStages.size()) ? 50 : 35);
		const double normalGate = fineRefinementStage && params.normalThresholdDeg > 0.0
									  ? params.normalThresholdDeg
									  : ((stage + 1U >= maxPairStages.size()) ? params.normalThresholdDeg : 0.0);
		if (!runReversePointToPlaneIcpStage(fineRefinementStage ? "fine soup ICP" : "pre-aligned soup ICP",
											templateSoupXyz, templateSoupNormals, workXyz, workNormals, params,
											maxPairStages[stage], maxIter, stageStep, stageRmse, &stageErr, normalGate))
		{
			RunLogger::info(std::string("[TemplateBrepUpdate] soup ICP stage ") + std::to_string(stage) +
							" failed: " + (stageErr.empty() ? "unknown" : stageErr));
			continue;
		}

		std::string transformErr;
		if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, stageStep, workingTemplate, &transformErr))
		{
			RunLogger::info(std::string("[TemplateBrepUpdate] soup ICP stage ") + std::to_string(stage) +
							" shape transform failed: " + (transformErr.empty() ? "unknown" : transformErr));
			continue;
		}

		cumulative = stageStep * cumulative;
		bestRmse = stageRmse;
		anyStageOk = true;
	}

	double trialAvgDist = 0.0;
	double trialMaxDist = 0.0;
	measureScanToCloudDistance(workXyz, templateSoupXyz, trialMaxDist, trialAvgDist);
	const double transMm = cumulative.translation().norm();
	const double rotDeg = rotationAngleDeg(cumulative);
	// autoRecover 初距大时 cumulative 平移可达数百 mm，旧 cap(modelDiag×0.5) 会误拒有效解
	const double maxTransMm =
		fineRefinementStage ? std::max({params.registrationMatchVoxelMm * 12.0, baselineMaxDist * 0.85, fb * 5.0})
							: (allowLargeCorrection ? std::max({modelDiag * 0.85, baselineMaxDist * 2.5, fb * 25.0})
													: std::max({fb * 5.0, modelDiag * 0.01, 8.0}));
	const double maxRotDeg = fineRefinementStage ? 12.0 : (allowLargeCorrection ? 60.0 : 8.0);
	double trialInlierMax = 0.0;
	double trialInlierAvg = 0.0;
	std::size_t trialInlierHits = 0U;
	if (fineRefinementStage)
	{
		measureScanToCloudInlierStats(workXyz, templateSoupXyz, inlierGateMm, trialInlierMax, trialInlierAvg,
									  trialInlierHits);
	}
	bool improved = false;
	if (fineRefinementStage && anyStageOk)
	{
		const bool inlierMaxNotWorse = baselineInlierMax <= 0.0 || trialInlierMax <= baselineInlierMax + 0.2;
		const bool inlierMaxBetter = baselineInlierMax > 0.0 && trialInlierMax + 0.1 < baselineInlierMax;
		const bool reachesFineTarget = trialInlierAvg > 0.0 && trialInlierAvg + 0.05 < kFineTargetInlierAvgMm;
		const bool avgBetter =
			trialInlierAvg > 0.0 && baselineInlierAvg > 0.0 &&
			(reachesFineTarget || trialInlierAvg + 0.15 < baselineInlierAvg ||
			 (baselineInlierAvg > kFineTargetInlierAvgMm && trialInlierAvg < baselineInlierAvg * 0.97));
		// 未扫描表面会抬高全局 maxDev，精配只看重叠区 inlier
		const bool globalExploded = trialMaxDist > baselineMaxDist + 25.0;
		improved = inlierMaxNotWorse && (inlierMaxBetter || avgBetter) && !globalExploded;
	}
	else
	{
		improved = anyStageOk && trialMaxDist + 0.1 < baselineMaxDist;
	}
	const bool saneTrans = transMm <= maxTransMm || (improved && transMm <= maxTransMm * 1.25 && rotDeg <= 8.0);
	const bool sane = saneTrans && rotDeg <= maxRotDeg;

	if (improved && sane)
	{
		outTemplateToScan = cumulative * priorStep;
		outAlignedTemplateShape = workingTemplate.clone();
		outRmseMm = bestRmse > 0.0 ? bestRmse : trialMaxDist;
		outSoupApplied = true;
		RunLogger::info(std::string("[TemplateBrepUpdate] ") +
						(fineRefinementStage ? "fineStage=soupMulti" : "coarseStage=soupMulti") +
						" reverse soup ICP applied: maxDevMm=" + std::to_string(trialMaxDist) + " (was " +
						std::to_string(baselineMaxDist) + ") transMm=" + std::to_string(transMm) +
						" rotDeg=" + std::to_string(rotDeg) +
						(fineRefinementStage ? " inlierAvg=" + std::to_string(trialInlierAvg) +
												   " hits=" + std::to_string(trialInlierHits)
											 : ""));
	}
	else
	{
		workingTemplate = originalTemplate.clone();
		templateSoupXyz = originalSoupXyz;
		templateSoupNormals = originalSoupNormals;
		outAlignedTemplateShape = originalTemplate.clone();
		outTemplateToScan = priorStep;
		outRmseMm = baselineMaxDist;
		outSoupApplied = false;
		RunLogger::info(std::string("[TemplateBrepUpdate] ") +
						(fineRefinementStage ? "fineStage=soupMulti" : "coarseStage=soupMulti") +
						" reverse soup ICP skipped (rollback): baselineMaxDevMm=" + std::to_string(baselineMaxDist) +
						" trialMaxDevMm=" + std::to_string(trialMaxDist) + " transMm=" + std::to_string(transMm) +
						" rotDeg=" + std::to_string(rotDeg) + " maxTransMm=" + std::to_string(maxTransMm) +
						" sane=" + (sane ? "true" : "false") +
						(fineRefinementStage ? " baselineInlierAvg=" + std::to_string(baselineInlierAvg) +
												   " trialInlierAvg=" + std::to_string(trialInlierAvg) +
												   " improved=" + (improved ? "true" : "false")
											 : ""));
	}

	if (outRmseMm > 0.0)
	{
		RunLogger::info(std::string("[TemplateBrepUpdate] overlapRmseMm=") + std::to_string(outRmseMm));
	}
	return true;
}

bool runCoarseGlobalPreAlign(const std::vector<float>& workXyz, const std::vector<float>& workNormals,
							 std::vector<float>& templateSoupXyz, std::vector<float>& templateSoupNormals,
							 const geoalgo::ShapeHandle& templateShape, geoalgo::ShapeHandle& workingTemplate,
							 const geoalgo::TemplateBrepUpdateParams& params, const double modelDiag,
							 const double initialMaxDevMm, const double preAlignGateMm,
							 Eigen::Isometry3d& inOutTemplateToScan, std::string* errMsg)
{
	double frameMaxDevMm = 0.0;
	double frameAvgDevMm = 0.0;
	measureScanToCloudDistance(workXyz, templateSoupXyz, frameMaxDevMm, frameAvgDevMm);
	std::size_t overlapHits = countPointsWithinPairDistance(workXyz, templateSoupXyz, preAlignGateMm, 512U);
	PcaAlignmentScore bestScore{frameMaxDevMm, frameAvgDevMm, overlapHits};

	if (params.enableRansacCoarseMatch)
	{
		pclalgo::RigidRegisterRansacParams ransacParams;
		ransacParams.maxFeaturePoints = params.icpMaxPoints;
		ransacParams.modelDiagMm = modelDiag;
		ransacParams.faceBandMm = params.faceBandMm;
		ransacParams.maxIterations = params.ransacMaxIterations;
		const double fb = std::max(params.faceBandMm, 1.0);
		if (params.registrationMatchVoxelMm > 0.0)
		{
			const double baseDist = params.registrationMatchVoxelMm * 2.5;
			ransacParams.inlierDistanceMm = baseDist * 2.4;
			ransacParams.featureVoxelMm = params.registrationMatchVoxelMm;
		}
		else
		{
			ransacParams.inlierDistanceMm =
				std::min(std::max({initialMaxDevMm * 0.6, modelDiag * 0.04, fb * 3.0}), modelDiag * 0.15);
			ransacParams.featureVoxelMm = std::max(modelDiag * 0.025, fb * 4.0);
		}
		ransacParams.skipTranslationCap = true;
		RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=ransac globalPreAlign inlierDistanceMm=") +
						std::to_string(ransacParams.inlierDistanceMm) +
						" featureVoxelMm=" + std::to_string(ransacParams.featureVoxelMm) +
						" (initialMaxDevMm=" + std::to_string(initialMaxDevMm) + ")");

		Eigen::Isometry3d ransacStep = Eigen::Isometry3d::Identity();
		double inlierRatio = 0.0;
		std::string ransacErr;
		if (pclalgo::rigidRegisterFeatureRansac(templateSoupXyz, templateSoupNormals, workXyz, workNormals, ransacStep,
												&inlierRatio, ransacParams, &ransacErr))
		{
			const std::vector<float> originalSoupBeforeRansac = templateSoupXyz;
			const std::vector<float> normalsBeforeRansac = templateSoupNormals;
			const Eigen::Isometry3d transformBeforeRansac = inOutTemplateToScan;
			applyIsometryInPlace(templateSoupXyz, ransacStep);
			applyIsometryInPlaceNormals(templateSoupNormals, ransacStep);
			inOutTemplateToScan = ransacStep * inOutTemplateToScan;
			std::string transformErr;
			if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, ransacStep, workingTemplate, &transformErr))
			{
				if (errMsg)
				{
					*errMsg = transformErr.empty() ? "RANSAC template transform failed" : transformErr;
				}
				return false;
			}
			measureScanToCloudDistance(workXyz, templateSoupXyz, frameMaxDevMm, frameAvgDevMm);
			overlapHits = countPointsWithinPairDistance(workXyz, templateSoupXyz, preAlignGateMm, 512U);
			const double ransacTransMm = ransacStep.translation().norm();
			const double ransacTransCapMm = ransacTranslationCapMm(modelDiag, preAlignGateMm, initialMaxDevMm);
			if (ransacTransMm > ransacTransCapMm)
			{
				templateSoupXyz = originalSoupBeforeRansac;
				templateSoupNormals = normalsBeforeRansac;
				inOutTemplateToScan = transformBeforeRansac;
				workingTemplate = templateShape.clone();
				if (!transformBeforeRansac.matrix().isApprox(Eigen::Isometry3d::Identity().matrix()))
				{
					if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, transformBeforeRansac, workingTemplate,
															 &transformErr))
					{
						if (errMsg)
						{
							*errMsg = transformErr.empty() ? "RANSAC rollback transform failed" : transformErr;
						}
						return false;
					}
				}
				measureScanToCloudDistance(workXyz, templateSoupXyz, frameMaxDevMm, frameAvgDevMm);
				overlapHits = countPointsWithinPairDistance(workXyz, templateSoupXyz, preAlignGateMm, 512U);
				RunLogger::info(
					std::string("[TemplateBrepUpdate] coarseStage=ransac globalPreAlign rejected transMm=") +
					std::to_string(ransacTransMm) + " (cap=" + std::to_string(ransacTransCapMm) +
					"mm); try PCA fallback");
			}
			else
			{
				bestScore = {frameMaxDevMm, frameAvgDevMm, overlapHits};
				RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=ransac globalPreAlign ok inlierRatio=") +
								std::to_string(inlierRatio) + " pairHits=" + std::to_string(overlapHits) +
								"/512 maxDevMm=" + std::to_string(frameMaxDevMm) +
								" transMm=" + std::to_string(ransacTransMm));
			}
		}
		else
		{
			RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=ransac globalPreAlign failed: ") +
							(ransacErr.empty() ? "unknown" : ransacErr));
		}
	}

	if (params.enableRansacCoarseMatch && overlapHits < kMinRegistrationOverlapHits)
	{
		pclalgo::RigidRegisterRansacParams reverseParams;
		reverseParams.maxFeaturePoints = params.icpMaxPoints;
		reverseParams.modelDiagMm = modelDiag;
		reverseParams.faceBandMm = params.faceBandMm;
		reverseParams.maxIterations = params.ransacMaxIterations;
		const double fb = std::max(params.faceBandMm, 1.0);
		if (params.registrationMatchVoxelMm > 0.0)
		{
			const double baseDist = params.registrationMatchVoxelMm * 2.5;
			reverseParams.inlierDistanceMm = baseDist * 2.4;
			reverseParams.featureVoxelMm = params.registrationMatchVoxelMm;
		}
		else
		{
			reverseParams.inlierDistanceMm =
				std::min(std::max({initialMaxDevMm * 0.6, modelDiag * 0.04, fb * 3.0}), modelDiag * 0.15);
			reverseParams.featureVoxelMm = std::max(modelDiag * 0.025, fb * 4.0);
		}
		reverseParams.skipTranslationCap = true;

		Eigen::Isometry3d scanToTemplate = Eigen::Isometry3d::Identity();
		double reverseInlierRatio = 0.0;
		std::string reverseErr;
		if (pclalgo::rigidRegisterFeatureRansac(workXyz, workNormals, templateSoupXyz, templateSoupNormals,
												scanToTemplate, &reverseInlierRatio, reverseParams, &reverseErr))
		{
			const std::vector<float> soupBeforeReverse = templateSoupXyz;
			const std::vector<float> normalsBeforeReverse = templateSoupNormals;
			const Eigen::Isometry3d transformBeforeReverse = inOutTemplateToScan;
			const PcaAlignmentScore scoreBeforeReverse{bestScore.maxDevMm, bestScore.meanDistMm, overlapHits};

			const Eigen::Isometry3d ransacStep = scanToTemplate.inverse();
			applyIsometryInPlace(templateSoupXyz, ransacStep);
			applyIsometryInPlaceNormals(templateSoupNormals, ransacStep);
			inOutTemplateToScan = ransacStep * inOutTemplateToScan;
			std::string transformErr;
			if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, ransacStep, workingTemplate, &transformErr))
			{
				if (errMsg)
				{
					*errMsg = transformErr.empty() ? "reverse RANSAC template transform failed" : transformErr;
				}
				return false;
			}

			measureScanToCloudDistance(workXyz, templateSoupXyz, frameMaxDevMm, frameAvgDevMm);
			overlapHits = countPointsWithinPairDistance(workXyz, templateSoupXyz, preAlignGateMm, 512U);
			const PcaAlignmentScore reverseScore{frameMaxDevMm, frameAvgDevMm, overlapHits};
			const double ransacTransMm = ransacStep.translation().norm();
			const double ransacTransCapMm = ransacTranslationCapMm(modelDiag, preAlignGateMm, initialMaxDevMm);
			if (ransacTransMm <= ransacTransCapMm && pcaAlignmentScoreBetter(reverseScore, scoreBeforeReverse))
			{
				bestScore = reverseScore;
				RunLogger::info(
					std::string("[TemplateBrepUpdate] coarseStage=ransac reverse globalPreAlign ok inlierRatio=") +
					std::to_string(reverseInlierRatio) + " pairHits=" + std::to_string(overlapHits) +
					"/512 maxDevMm=" + std::to_string(frameMaxDevMm) + " transMm=" + std::to_string(ransacTransMm));
			}
			else
			{
				templateSoupXyz = soupBeforeReverse;
				templateSoupNormals = normalsBeforeReverse;
				inOutTemplateToScan = transformBeforeReverse;
				workingTemplate = templateShape.clone();
				if (!transformBeforeReverse.matrix().isApprox(Eigen::Isometry3d::Identity().matrix()))
				{
					if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, transformBeforeReverse, workingTemplate,
															 &transformErr))
					{
						if (errMsg)
						{
							*errMsg = transformErr.empty() ? "reverse RANSAC rollback failed" : transformErr;
						}
						return false;
					}
				}
				measureScanToCloudDistance(workXyz, templateSoupXyz, frameMaxDevMm, frameAvgDevMm);
				overlapHits = countPointsWithinPairDistance(workXyz, templateSoupXyz, preAlignGateMm, 512U);
				RunLogger::info(
					std::string("[TemplateBrepUpdate] coarseStage=ransac reverse globalPreAlign rejected transMm=") +
					std::to_string(ransacTransMm));
			}
		}
		else
		{
			RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=ransac reverse globalPreAlign failed: ") +
							(reverseErr.empty() ? "unknown" : reverseErr));
		}
	}

	if (overlapHits < kMinRegistrationOverlapHits)
	{
		const std::vector<float> soupBeforePca = templateSoupXyz;
		const std::vector<float> normalsBeforePca = templateSoupNormals;
		const Eigen::Isometry3d transformBeforePca = inOutTemplateToScan;
		const PcaAlignmentScore scoreBeforePca = bestScore;

		RunLogger::info("[TemplateBrepUpdate] coarseStage=pca globalPreAlign fallback");
		const Eigen::Isometry3d pcaStep = alignTemplatePcaToScan(templateSoupXyz, templateSoupNormals, workXyz);
		inOutTemplateToScan = pcaStep * inOutTemplateToScan;
		std::string transformErr;
		if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, pcaStep, workingTemplate, &transformErr))
		{
			if (errMsg)
			{
				*errMsg = transformErr.empty() ? "PCA template transform failed" : transformErr;
			}
			return false;
		}

		double pcaMaxDevMm = 0.0;
		double pcaAvgDevMm = 0.0;
		measureScanToCloudDistance(workXyz, templateSoupXyz, pcaMaxDevMm, pcaAvgDevMm);
		const std::size_t pcaHits = countPointsWithinPairDistance(workXyz, templateSoupXyz, preAlignGateMm, 512U);
		const PcaAlignmentScore pcaScore{pcaMaxDevMm, pcaAvgDevMm, pcaHits};
		const double pcaAcceptMaxDevMm = preAlignGateMm * 2.0;
		if ((!pcaAlignmentScoreBetter(pcaScore, scoreBeforePca) || pcaMaxDevMm > pcaAcceptMaxDevMm) &&
			!pcaStrongOverlapAccept(pcaScore, scoreBeforePca, initialMaxDevMm))
		{
			templateSoupXyz = soupBeforePca;
			templateSoupNormals = normalsBeforePca;
			inOutTemplateToScan = transformBeforePca;
			workingTemplate = templateShape.clone();
			if (!transformBeforePca.matrix().isApprox(Eigen::Isometry3d::Identity().matrix()))
			{
				if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, transformBeforePca, workingTemplate,
														 &transformErr))
				{
					if (errMsg)
					{
						*errMsg = transformErr.empty() ? "PCA rollback transform failed" : transformErr;
					}
					return false;
				}
			}
			RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=pca rejected (keep prior pairHits=") +
							std::to_string(scoreBeforePca.pairHits) + " vs " + std::to_string(pcaHits) + " maxDevMm=" +
							std::to_string(pcaMaxDevMm) + " need<=" + std::to_string(pcaAcceptMaxDevMm) + ")");
		}
	}
	else
	{
		RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=pca skip (globalPreAlign sufficient pairHits=") +
						std::to_string(overlapHits) + "/512 maxDevMm=" + std::to_string(frameMaxDevMm) + ")");
	}

	measureScanToCloudDistance(workXyz, templateSoupXyz, frameMaxDevMm, frameAvgDevMm);
	overlapHits = countPointsWithinPairDistance(workXyz, templateSoupXyz, preAlignGateMm, 512U);
	const Eigen::AlignedBox3d scanBox = pclalgo::computeBoundingBox(workXyz);
	const Eigen::AlignedBox3d templateBox = pclalgo::computeBoundingBox(templateSoupXyz);
	const double centroidDistMm =
		(scanBox.isEmpty() || templateBox.isEmpty()) ? 0.0 : (scanBox.center() - templateBox.center()).norm();
	if (centroidDistMm > preAlignGateMm * 0.5)
	{
		const Eigen::Vector3d shift = scanBox.center() - templateBox.center();
		const double shiftMm = shift.norm();
		const double shiftCapMm = std::max(modelDiag * 0.25, preAlignGateMm * 2.0);
		if (shiftMm > 0.0 && shiftMm <= shiftCapMm)
		{
			const std::vector<float> soupBeforeSnap = templateSoupXyz;
			const std::vector<float> normalsBeforeSnap = templateSoupNormals;
			const Eigen::Isometry3d transformBeforeSnap = inOutTemplateToScan;
			const PcaAlignmentScore scoreBeforeSnap{frameMaxDevMm, frameAvgDevMm, overlapHits};

			Eigen::Isometry3d snapStep = Eigen::Isometry3d::Identity();
			snapStep.translation() = shift;
			applyIsometryInPlace(templateSoupXyz, snapStep);
			applyIsometryInPlaceNormals(templateSoupNormals, snapStep);
			inOutTemplateToScan = snapStep * inOutTemplateToScan;
			std::string transformErr;
			if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, snapStep, workingTemplate, &transformErr))
			{
				if (errMsg)
				{
					*errMsg = transformErr.empty() ? "centroid snap transform failed" : transformErr;
				}
				return false;
			}

			double snapMaxDevMm = 0.0;
			double snapAvgDevMm = 0.0;
			measureScanToCloudDistance(workXyz, templateSoupXyz, snapMaxDevMm, snapAvgDevMm);
			const std::size_t snapHits = countPointsWithinPairDistance(workXyz, templateSoupXyz, preAlignGateMm, 512U);
			const PcaAlignmentScore snapScore{snapMaxDevMm, snapAvgDevMm, snapHits};
			if (pcaAlignmentScoreBetter(snapScore, scoreBeforeSnap))
			{
				frameMaxDevMm = snapMaxDevMm;
				frameAvgDevMm = snapAvgDevMm;
				overlapHits = snapHits;
				RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=centroidSnap ok shiftMm=") +
								std::to_string(shiftMm) + " pairHits=" + std::to_string(snapHits) +
								"/512 maxDevMm=" + std::to_string(snapMaxDevMm));
			}
			else
			{
				templateSoupXyz = soupBeforeSnap;
				templateSoupNormals = normalsBeforeSnap;
				inOutTemplateToScan = transformBeforeSnap;
				workingTemplate = templateShape.clone();
				if (!transformBeforeSnap.matrix().isApprox(Eigen::Isometry3d::Identity().matrix()))
				{
					if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, transformBeforeSnap, workingTemplate,
															 &transformErr))
					{
						if (errMsg)
						{
							*errMsg = transformErr.empty() ? "centroid snap rollback failed" : transformErr;
						}
						return false;
					}
				}
				RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=centroidSnap rejected shiftMm=") +
								std::to_string(shiftMm));
			}
		}
	}

	if (overlapHits >= kMinRegistrationOverlapHits && frameMaxDevMm > preAlignGateMm * 1.2)
	{
		const std::vector<float> soupBeforeRefine = templateSoupXyz;
		const std::vector<float> normalsBeforeRefine = templateSoupNormals;
		const Eigen::Isometry3d transformBeforeRefine = inOutTemplateToScan;
		const geoalgo::ShapeHandle templateBeforeRefine = workingTemplate.clone();
		const PcaAlignmentScore scoreBeforeRefine{frameMaxDevMm, frameAvgDevMm, overlapHits};

		const double maxPairMm = std::min({frameMaxDevMm * 0.9, modelDiag * 0.2, preAlignGateMm * 8.0});
		Eigen::Isometry3d refineStep = Eigen::Isometry3d::Identity();
		double refineRmse = 0.0;
		std::string refineErr;
		if (runReversePointToPlaneIcpStage("globalPreAlign refine", templateSoupXyz, templateSoupNormals, workXyz,
										   workNormals, params, maxPairMm, 40, refineStep, refineRmse, &refineErr, 0.0))
		{
			std::string transformErr;
			if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, refineStep, workingTemplate, &transformErr))
			{
				if (errMsg)
				{
					*errMsg = transformErr.empty() ? "globalPreAlign refine shape failed" : transformErr;
				}
				return false;
			}
			inOutTemplateToScan = refineStep * inOutTemplateToScan;

			double refineMaxDevMm = 0.0;
			double refineAvgDevMm = 0.0;
			measureScanToCloudDistance(workXyz, templateSoupXyz, refineMaxDevMm, refineAvgDevMm);
			const std::size_t refineHits =
				countPointsWithinPairDistance(workXyz, templateSoupXyz, preAlignGateMm, 512U);
			const PcaAlignmentScore refineScore{refineMaxDevMm, refineAvgDevMm, refineHits};
			const double refineTransMm = refineStep.translation().norm();
			const double refineRotDeg = rotationAngleDeg(refineStep);
			const double refineTransCapMm = std::max(modelDiag * 0.25, preAlignGateMm * 2.0);
			if (pcaAlignmentScoreBetter(refineScore, scoreBeforeRefine) && refineTransMm <= refineTransCapMm &&
				refineRotDeg <= 45.0)
			{
				frameMaxDevMm = refineMaxDevMm;
				frameAvgDevMm = refineAvgDevMm;
				overlapHits = refineHits;
				RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=globalPreAlign refine ok maxPairMm=") +
								std::to_string(maxPairMm) + " pairHits=" + std::to_string(refineHits) +
								"/512 maxDevMm=" + std::to_string(refineMaxDevMm));
			}
			else
			{
				templateSoupXyz = soupBeforeRefine;
				templateSoupNormals = normalsBeforeRefine;
				inOutTemplateToScan = transformBeforeRefine;
				workingTemplate = templateBeforeRefine;
				RunLogger::info(
					std::string("[TemplateBrepUpdate] coarseStage=globalPreAlign refine rejected maxDevMm=") +
					std::to_string(refineMaxDevMm) + " transMm=" + std::to_string(refineTransMm));
			}
		}
		else
		{
			RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=globalPreAlign refine failed: ") +
							(refineErr.empty() ? "unknown" : refineErr));
		}
	}
	return true;
}

std::vector<double> buildCoarseIcpLadderMaxPairsMm(const double postCoarseMaxDevMm, const double centroidDistMm,
												   const double modelDiag, const double preAlignGateMm,
												   const double faceBandMm)
{
	const double fb = std::max(faceBandMm, 1.0);
	const double gapMm = std::max(postCoarseMaxDevMm, centroidDistMm);
	std::vector<double> out;
	const auto add = [&](const double v)
	{
		if (v > 0.0)
		{
			out.push_back(v);
		}
	};
	for (const double factor : {0.55, 0.75, 0.95, 1.05})
	{
		add(gapMm * factor);
	}
	for (const double factor : {0.25, 0.5})
	{
		add(modelDiag * factor);
	}
	add(preAlignGateMm);
	for (const double factor : {10.0, 15.0})
	{
		add(fb * factor);
	}
	for (const double factor : {8.0, 5.0, 3.0, 2.0, 1.25})
	{
		add(preAlignGateMm * factor);
	}
	std::sort(out.begin(), out.end());
	out.erase(std::unique(out.begin(), out.end()), out.end());
	return out;
}

bool applyCoarseCentroidSnap(const std::vector<float>& workXyz, std::vector<float>& templateSoupXyz,
							 std::vector<float>& templateSoupNormals, const geoalgo::ShapeHandle& templateShape,
							 geoalgo::ShapeHandle& workingTemplate, Eigen::Isometry3d& inOutTemplateToScan,
							 std::string* errMsg)
{
	const Eigen::AlignedBox3d scanBox = pclalgo::computeBoundingBox(workXyz);
	const Eigen::AlignedBox3d templateBox = pclalgo::computeBoundingBox(templateSoupXyz);
	if (scanBox.isEmpty() || templateBox.isEmpty())
	{
		return false;
	}
	const Eigen::Vector3d shift = scanBox.center() - templateBox.center();
	if (shift.norm() < 1e-3)
	{
		return false;
	}
	Eigen::Isometry3d snapStep = Eigen::Isometry3d::Identity();
	snapStep.translation() = shift;
	applyIsometryInPlace(templateSoupXyz, snapStep);
	if (!templateSoupNormals.empty())
	{
		applyIsometryInPlaceNormals(templateSoupNormals, snapStep);
	}
	inOutTemplateToScan = snapStep * inOutTemplateToScan;
	std::string transformErr;
	if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, snapStep, workingTemplate, &transformErr))
	{
		if (errMsg)
		{
			*errMsg = transformErr.empty() ? "centroid snap transform failed" : transformErr;
		}
		return false;
	}
	RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=centroidSnap shiftMm=") +
					std::to_string(shift.norm()));
	return true;
}

bool runCoarseTightIcpRefinement(const std::vector<float>& workXyz, const std::vector<float>& workNormals,
								 std::vector<float>& templateSoupXyz, std::vector<float>& templateSoupNormals,
								 geoalgo::ShapeHandle& workingTemplate, const geoalgo::TemplateBrepUpdateParams& params,
								 const double preAlignGateMm, const double baselineMaxDevMm,
								 Eigen::Isometry3d& inOutTemplateToScan, double& outRmseMm)
{
	const double maxPairMm = std::max(preAlignGateMm * 2.0, baselineMaxDevMm * 0.45);
	const std::vector<float> soupBefore = templateSoupXyz;
	const std::vector<float> normalsBefore = templateSoupNormals;
	const geoalgo::ShapeHandle shapeBefore = workingTemplate.clone();
	const Eigen::Isometry3d transformBefore = inOutTemplateToScan;

	Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
	double rmse = 0.0;
	std::string stageErr;
	if (!runReversePointToPlaneIcpStage("tight ICP refine", templateSoupXyz, templateSoupNormals, workXyz, workNormals,
										params, maxPairMm, 50, step, rmse, &stageErr,
										params.normalThresholdDeg > 0.0 ? params.normalThresholdDeg : 0.0))
	{
		return false;
	}

	std::string transformErr;
	if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, step, workingTemplate, &transformErr))
	{
		templateSoupXyz = soupBefore;
		templateSoupNormals = normalsBefore;
		workingTemplate = shapeBefore;
		return false;
	}
	inOutTemplateToScan = step * inOutTemplateToScan;

	double maxDevMm = 0.0;
	double avgDevMm = 0.0;
	measureScanToCloudDistance(workXyz, templateSoupXyz, maxDevMm, avgDevMm);
	const std::size_t hits = countPointsWithinPairDistance(workXyz, templateSoupXyz, preAlignGateMm, 512U);
	if (maxDevMm + 0.5 >= baselineMaxDevMm)
	{
		templateSoupXyz = soupBefore;
		templateSoupNormals = normalsBefore;
		workingTemplate = shapeBefore;
		inOutTemplateToScan = transformBefore;
		RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=tightIcpRefine rejected maxDevMm=") +
						std::to_string(maxDevMm) + " (baseline=" + std::to_string(baselineMaxDevMm) + ")");
		return false;
	}

	outRmseMm = rmse > 0.0 ? rmse : maxDevMm;
	RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=tightIcpRefine ok maxPairMm=") +
					std::to_string(maxPairMm) + " pairHits=" + std::to_string(hits) +
					"/512 maxDevMm=" + std::to_string(maxDevMm));
	return true;
}

bool runCoarseIcpLadderFallback(const std::vector<float>& workXyz, const std::vector<float>& workNormals,
								std::vector<float>& templateSoupXyz, std::vector<float>& templateSoupNormals,
								geoalgo::ShapeHandle& workingTemplate, const geoalgo::TemplateBrepUpdateParams& params,
								const double modelDiag, const double preAlignGateMm, const double postCoarseMaxDevMm,
								const double centroidDistMm, const double baselineMaxDevMm,
								Eigen::Isometry3d& inOutTemplateToScan, double& outRmseMm, std::string* errMsg)
{
	RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=coarseIcpLadder postMaxDevMm=") +
					std::to_string(postCoarseMaxDevMm) + " centroidDistMm=" + std::to_string(centroidDistMm));

	const std::vector<double> maxPairCandidatesMm = buildCoarseIcpLadderMaxPairsMm(
		postCoarseMaxDevMm, centroidDistMm, modelDiag, preAlignGateMm, params.faceBandMm);

	const double ladderQualityGateMm = std::max(preAlignGateMm * 2.0, 30.0);
	Eigen::Isometry3d ladderCumulative = Eigen::Isometry3d::Identity();
	double coarseRmse = 0.0;
	bool coarseOk = false;
	std::string coarseErr;
	double bestMaxDev = baselineMaxDevMm;
	std::size_t bestHits = 0U;

	for (std::size_t li = maxPairCandidatesMm.size(); li > 0U; --li)
	{
		const double maxPairMm = maxPairCandidatesMm[li - 1U];
		const std::vector<float> soupBeforeStage = templateSoupXyz;
		const std::vector<float> normalsBeforeStage = templateSoupNormals;
		const geoalgo::ShapeHandle shapeBeforeStage = workingTemplate.clone();
		const Eigen::Isometry3d cumulativeBeforeStage = ladderCumulative;

		coarseErr.clear();
		Eigen::Isometry3d stageStep = Eigen::Isometry3d::Identity();
		if (!runReverseIcpStage("coarse ICP ladder", templateSoupXyz, templateSoupNormals, workXyz, workNormals, params,
								maxPairMm, 30, stageStep, coarseRmse, &coarseErr, 0.0))
		{
			continue;
		}

		std::string transformErr;
		if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, stageStep, workingTemplate, &transformErr))
		{
			if (errMsg)
			{
				*errMsg = transformErr.empty() ? "coarse ICP ladder shape transform failed" : transformErr;
			}
			return false;
		}

		ladderCumulative = stageStep * ladderCumulative;

		const std::size_t hits = countPointsWithinPairDistance(workXyz, templateSoupXyz, preAlignGateMm, 512U);
		double maxDevMm = 0.0;
		double avgDevMm = 0.0;
		measureScanToCloudDistance(workXyz, templateSoupXyz, maxDevMm, avgDevMm);
		RunLogger::info(std::string("[TemplateBrepUpdate] coarse ICP ladder stage ok maxPairMm=") +
						std::to_string(maxPairMm) + " pairHits=" + std::to_string(hits) +
						"/512 maxDevMm=" + std::to_string(maxDevMm));

		if (maxDevMm > bestMaxDev + 2.0 && maxDevMm > ladderQualityGateMm)
		{
			templateSoupXyz = soupBeforeStage;
			templateSoupNormals = normalsBeforeStage;
			workingTemplate = shapeBeforeStage;
			ladderCumulative = cumulativeBeforeStage;
			RunLogger::info(std::string("[TemplateBrepUpdate] coarse ICP ladder stage rollback maxDevMm=") +
							std::to_string(maxDevMm) + " (best=" + std::to_string(bestMaxDev) + ")");
			continue;
		}

		coarseOk = true;
		bestMaxDev = maxDevMm;
		bestHits = hits;
		if (hits >= kMinRegistrationOverlapHits && maxDevMm <= ladderQualityGateMm)
		{
			break;
		}
	}

	if (!coarseOk)
	{
		if (errMsg)
		{
			*errMsg = coarseErr.empty() ? "coarse ICP ladder failed" : coarseErr;
		}
		return false;
	}

	inOutTemplateToScan = ladderCumulative * inOutTemplateToScan;
	outRmseMm = coarseRmse > 0.0 ? coarseRmse : bestMaxDev;
	RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=coarseIcpLadder done pairHits=") +
					std::to_string(bestHits) + "/512 maxDevMm=" + std::to_string(bestMaxDev));
	return true;
}

bool runColdStartCoarseIcpPath(const std::vector<float>& workXyz, const std::vector<float>& workNormals,
							   std::vector<float>& templateSoupXyz, std::vector<float>& templateSoupNormals,
							   geoalgo::ShapeHandle& workingTemplate, const geoalgo::TemplateBrepUpdateParams& params,
							   const RegistrationAlignMode alignMode, const double modelDiag,
							   const double preAlignGateMm, const double initialMaxDevMm, const double centroidDistMm,
							   Eigen::Isometry3d& inOutTemplateToScan, geoalgo::ShapeHandle& outAlignedTemplateShape,
							   double& outRmseMm, std::string* errMsg)
{
	std::vector<double> coarseMaxPairCandidatesMm;
	if (params.icpMaxPairDistanceMm > 0.0)
	{
		coarseMaxPairCandidatesMm.push_back(std::max(params.icpMaxPairDistanceMm, modelDiag * 0.25));
	}
	else
	{
		for (const double frac : {0.5, 0.75, 1.0, 1.25})
		{
			coarseMaxPairCandidatesMm.push_back(std::max(modelDiag * frac, 10.0));
		}
		if (alignMode == RegistrationAlignMode::AutoRecover && initialMaxDevMm > preAlignGateMm * 2.0)
		{
			const double gapMm = std::max(initialMaxDevMm, centroidDistMm);
			coarseMaxPairCandidatesMm.push_back(gapMm * 0.55);
			coarseMaxPairCandidatesMm.push_back(gapMm * 0.75);
			coarseMaxPairCandidatesMm.push_back(gapMm * 0.95);
			coarseMaxPairCandidatesMm.push_back(gapMm * 1.05);
			RunLogger::info(std::string("[TemplateBrepUpdate] extend coarse maxPair for large gap, gapMm=") +
							std::to_string(gapMm));
		}
	}

	std::sort(coarseMaxPairCandidatesMm.begin(), coarseMaxPairCandidatesMm.end());

	Eigen::Isometry3d coarseStep = Eigen::Isometry3d::Identity();
	double coarseRmse = 0.0;
	bool coarseOk = false;
	std::string coarseErr;
	for (std::size_t ci = coarseMaxPairCandidatesMm.size(); ci > 0U; --ci)
	{
		const double maxPairMm = coarseMaxPairCandidatesMm[ci - 1U];
		coarseErr.clear();
		if (runReverseIcpStage("coarse reverse ICP", templateSoupXyz, templateSoupNormals, workXyz, workNormals, params,
							   maxPairMm, 30, coarseStep, coarseRmse, &coarseErr, 0.0))
		{
			coarseOk = true;
			RunLogger::info(std::string("[TemplateBrepUpdate] coarse reverse ICP ok, maxPairMm=") +
							std::to_string(maxPairMm) + " rmseMm=" + std::to_string(coarseRmse));
			break;
		}
	}
	if (!coarseOk)
	{
		if (errMsg)
		{
			std::ostringstream oss;
			oss << coarseErr << " [scanPts=" << (workXyz.size() / 3U)
				<< " templatePts=" << (templateSoupXyz.size() / 3U) << " modelDiagMm=" << modelDiag
				<< " centroidDistMm=" << centroidDistMm << "]";
			*errMsg = oss.str();
		}
		return false;
	}
	inOutTemplateToScan = coarseStep * inOutTemplateToScan;
	std::string transformErr;
	if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, coarseStep, workingTemplate, &transformErr))
	{
		if (errMsg)
		{
			*errMsg = transformErr.empty() ? "coarse template transform failed" : transformErr;
		}
		return false;
	}

	double fineMaxPairMm = params.icpMaxPairDistanceMm;
	if (fineMaxPairMm <= 0.0)
	{
		fineMaxPairMm = std::max(modelDiag * 0.05, 2.0);
	}
	if (!coarseMaxPairCandidatesMm.empty())
	{
		fineMaxPairMm = std::min(fineMaxPairMm, coarseMaxPairCandidatesMm.back());
	}

	const double icpGateHintMm =
		std::max(params.faceBandMm * params.maxIcpRmseToFaceBandRatio, params.minIcpRmseGateMm);
	const bool skipFineIcp =
		coarseOk && coarseRmse > 0.0 && (params.maxIcpRmseToFaceBandRatio <= 0.0 || coarseRmse <= icpGateHintMm);

	if (skipFineIcp)
	{
		RunLogger::info(std::string("[TemplateBrepUpdate] skip fine ICP (coarse rmseMm=") + std::to_string(coarseRmse) +
						" <= gate " + std::to_string(icpGateHintMm) + ")");
		outRmseMm = coarseRmse;
	}
	else
	{
		RunLogger::info("[TemplateBrepUpdate] fine reverse soup point-to-plane ICP...");
		Eigen::Isometry3d fineStep = Eigen::Isometry3d::Identity();
		double fineRmse = coarseRmse;
		std::string fineErr;
		const int fineIterations = 20;
		if (runReversePointToPlaneIcpStage("fine reverse soup ICP", templateSoupXyz, templateSoupNormals, workXyz,
										   workNormals, params, fineMaxPairMm, fineIterations, fineStep, fineRmse,
										   &fineErr))
		{
			inOutTemplateToScan = fineStep * inOutTemplateToScan;
			if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, fineStep, workingTemplate, &transformErr))
			{
				if (errMsg)
				{
					*errMsg = transformErr.empty() ? "fine template transform failed" : transformErr;
				}
				return false;
			}
			outRmseMm = fineRmse;
			RunLogger::info(std::string("[TemplateBrepUpdate] fine reverse soup ICP ok, rmseMm=") +
							std::to_string(fineRmse));
		}
		else
		{
			RunLogger::info(std::string("[TemplateBrepUpdate] fine reverse soup ICP skipped: ") +
							(fineErr.empty() ? "unknown" : fineErr));
			outRmseMm = coarseRmse;
		}
	}

	outAlignedTemplateShape = std::move(workingTemplate);

	if (!skipFineIcp)
	{
		double avgDist = 0.0;
		double maxDist = 0.0;
		measureScanToCloudDistance(workXyz, templateSoupXyz, maxDist, avgDist);
		if (maxDist > 0.0)
		{
			RunLogger::info(std::string("[TemplateBrepUpdate] overlap maxDevMm=") + std::to_string(maxDist) +
							" avgDevMm=" + std::to_string(avgDist));
			if (outRmseMm <= 0.0)
			{
				outRmseMm = maxDist;
			}
		}
	}
	return true;
}

void saveRegistrationCheckpoint(geoalgo::TemplateBrepRegistrationCheckpoint* checkpoint,
								const std::vector<float>& templateSoupXyz,
								const std::vector<float>& templateSoupNormals, const Eigen::Isometry3d& icpDeltaWorld)
{
	if (!checkpoint)
	{
		return;
	}
	checkpoint->templateSoupXyz = templateSoupXyz;
	checkpoint->templateSoupNormals = templateSoupNormals;
	checkpoint->icpDeltaWorld = icpDeltaWorld;
	checkpoint->valid = !templateSoupXyz.empty();
}

bool runFineInlierIcpRefinement(const std::vector<float>& workXyz, const std::vector<float>& workNormals,
								std::vector<float>& templateSoupXyz, std::vector<float>& templateSoupNormals,
								geoalgo::ShapeHandle& workingTemplate, const geoalgo::TemplateBrepUpdateParams& params,
								const double modelDiag, Eigen::Isometry3d& inOutFineStep, double& outRmseMm)
{
	const double fb = std::max(params.faceBandMm, 1.0);
	const double coarseInlierGateMm = params.registrationMatchVoxelMm > 0.0
										  ? std::max({params.registrationMatchVoxelMm * 2.5, fb * 3.0, 15.0})
										  : std::max({fb * 6.0, modelDiag * 0.02, 15.0});

	double baseInlierMax = 0.0;
	double baseInlierAvg = 0.0;
	std::size_t baseInlierHits = 0U;
	measureScanToCloudInlierStats(workXyz, templateSoupXyz, coarseInlierGateMm, baseInlierMax, baseInlierAvg,
								  baseInlierHits);
	const double inlierGateMm = fineOverlapInlierGateMm(params, baseInlierMax);
	if (baseInlierHits > 0U)
	{
		measureScanToCloudInlierStats(workXyz, templateSoupXyz, inlierGateMm, baseInlierMax, baseInlierAvg,
									  baseInlierHits);
	}
	double baseMax = 0.0;
	double baseAvg = 0.0;
	measureScanToCloudDistance(workXyz, templateSoupXyz, baseMax, baseAvg);

	const std::vector<float> soupBefore = templateSoupXyz;
	const std::vector<float> normalsBefore = templateSoupNormals;
	const geoalgo::ShapeHandle shapeBefore = workingTemplate.clone();
	const Eigen::Isometry3d stepBefore = inOutFineStep;

	const bool tightBaseline = baseInlierAvg > 0.0 && baseInlierAvg < 15.0;
	const double vox = params.registrationMatchVoxelMm > 0.0 ? params.registrationMatchVoxelMm : fb;

	std::vector<double> maxPairCandidates;
	if (tightBaseline)
	{
		const double pairCap = std::min(std::max(baseInlierMax * 1.6, 8.0), 15.0);
		maxPairCandidates = {std::max(vox * 1.5, 3.0), std::max(vox * 2.5, 5.0), std::max(vox * 3.5, 7.0), pairCap};
	}
	else
	{
		const double cap = std::min({baseInlierMax > 0.0 ? baseInlierMax * 2.2 : baseMax * 0.35, vox * 3.0, fb * 5.0});
		maxPairCandidates.push_back(std::max(3.0, cap));
	}

	Eigen::Isometry3d bestStep = Eigen::Isometry3d::Identity();
	double bestTrialInlierMax = baseInlierMax;
	double bestTrialInlierAvg = baseInlierAvg;
	double bestIcpRmse = 0.0;
	bool foundBetter = false;

	for (const double maxPairMm : maxPairCandidates)
	{
		templateSoupXyz = soupBefore;
		templateSoupNormals = normalsBefore;
		workingTemplate = shapeBefore.clone();

		Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
		double icpRmse = 0.0;
		std::string icpErr;
		bool icpOk = runReversePointToPlaneIcpStage("fine inlier P2PL", templateSoupXyz, templateSoupNormals, workXyz,
													workNormals, params, maxPairMm, 100, step, icpRmse, &icpErr, 0.0);
		if (!icpOk)
		{
			templateSoupXyz = soupBefore;
			templateSoupNormals = normalsBefore;
			icpOk = runReverseIcpStage("fine inlier P2P", templateSoupXyz, templateSoupNormals, workXyz, workNormals,
									   params, maxPairMm, 100, step, icpRmse, &icpErr, 0.0);
		}
		if (!icpOk)
		{
			continue;
		}

		std::string transformErr;
		if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, step, workingTemplate, &transformErr))
		{
			continue;
		}

		double trialInlierMax = 0.0;
		double trialInlierAvg = 0.0;
		std::size_t trialInlierHits = 0U;
		measureScanToCloudInlierStats(workXyz, templateSoupXyz, inlierGateMm, trialInlierMax, trialInlierAvg,
									  trialInlierHits);
		double trialMax = 0.0;
		double trialAvg = 0.0;
		measureScanToCloudDistance(workXyz, templateSoupXyz, trialMax, trialAvg);

		const bool inlierMaxBetter =
			baseInlierMax > 0.0 && trialInlierMax + (tightBaseline ? 0.03 : 0.15) < baseInlierMax;
		const bool reachesFineTarget = trialInlierAvg > 0.0 && trialInlierAvg + 0.05 < kFineTargetInlierAvgMm;
		const bool avgBetter =
			baseInlierAvg > 0.0 && trialInlierAvg > 0.0 &&
			(reachesFineTarget || trialInlierAvg + 0.15 < baseInlierAvg ||
			 (tightBaseline && baseInlierAvg > kFineTargetInlierAvgMm && trialInlierAvg < baseInlierAvg * 0.97) ||
			 (!tightBaseline && trialInlierAvg < baseInlierAvg * 0.985));
		const bool globalOk = trialMax <= baseMax + 2.0;

		if ((inlierMaxBetter || avgBetter) && globalOk && (!foundBetter || trialInlierAvg < bestTrialInlierAvg))
		{
			foundBetter = true;
			bestStep = step;
			bestTrialInlierMax = trialInlierMax;
			bestTrialInlierAvg = trialInlierAvg;
			bestIcpRmse = icpRmse;
		}
	}

	if (!foundBetter)
	{
		templateSoupXyz = soupBefore;
		templateSoupNormals = normalsBefore;
		workingTemplate = shapeBefore;
		inOutFineStep = stepBefore;
		RunLogger::info(std::string("[TemplateBrepUpdate] fineStage=inlierIcp no improvement (baseline inlierAvg=") +
						std::to_string(baseInlierAvg) + "mm)");
		outRmseMm = baseInlierAvg > 0.0 ? baseInlierAvg : baseMax;
		return false;
	}

	templateSoupXyz = soupBefore;
	templateSoupNormals = normalsBefore;
	workingTemplate = shapeBefore.clone();
	applyIsometryInPlace(templateSoupXyz, bestStep);
	applyIsometryInPlaceNormals(templateSoupNormals, bestStep);
	std::string shapeErr;
	if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, bestStep, workingTemplate, &shapeErr))
	{
		templateSoupXyz = soupBefore;
		templateSoupNormals = normalsBefore;
		workingTemplate = shapeBefore;
		inOutFineStep = stepBefore;
		return false;
	}

	inOutFineStep = bestStep * inOutFineStep;
	outRmseMm = bestTrialInlierAvg > 0.0 ? bestTrialInlierAvg : bestIcpRmse;
	RunLogger::info(std::string("[TemplateBrepUpdate] fineStage=inlierIcp ok inlierMaxMm=") +
					std::to_string(bestTrialInlierMax) + " (was " + std::to_string(baseInlierMax) +
					") inlierAvgMm=" + std::to_string(bestTrialInlierAvg) + " (was " + std::to_string(baseInlierAvg) +
					") transMm=" + std::to_string(bestStep.translation().norm()));
	return true;
}

bool runFineRegistrationStage(const std::vector<float>& workXyz, const std::vector<float>& workNormals,
							  std::vector<float>& templateSoupXyz, std::vector<float>& templateSoupNormals,
							  geoalgo::ShapeHandle& workingTemplate, const geoalgo::TemplateBrepUpdateParams& params,
							  const double modelDiag, Eigen::Isometry3d& inOutTemplateToScan,
							  geoalgo::ShapeHandle& outAlignedTemplateShape, double& outRmseMm, std::string* errMsg)
{
	(void)errMsg;
	RunLogger::info("[TemplateBrepUpdate] fineStage=soupMulti point-to-plane ICP (PointCloudMatch.py multiscale)");

	bool soupApplied = false;
	if (!runReverseSoupMultiStageIcp(workXyz, workNormals, templateSoupXyz, templateSoupNormals, workingTemplate,
									 params, modelDiag, inOutTemplateToScan, outAlignedTemplateShape, outRmseMm, false,
									 true, soupApplied))
	{
		return false;
	}
	if (!soupApplied)
	{
		RunLogger::info("[TemplateBrepUpdate] fineStage=soupMulti skipped (no improvement)");
		outAlignedTemplateShape = workingTemplate.clone();
	}

	(void)runFineInlierIcpRefinement(workXyz, workNormals, templateSoupXyz, templateSoupNormals, workingTemplate,
									 params, modelDiag, inOutTemplateToScan, outRmseMm);
	outAlignedTemplateShape = workingTemplate.clone();
	return true;
}

bool runCoarseAlignmentPipeline(const std::vector<float>& workXyz, const std::vector<float>& workNormals,
								const geoalgo::ShapeHandle& templateShape, std::vector<float>& templateSoupXyz,
								std::vector<float>& templateSoupNormals,
								const geoalgo::TemplateBrepUpdateParams& params, const RegistrationAlignMode alignMode,
								const double modelDiag, const double preAlignGateMm,
								const std::size_t initialOverlapHits, const double initialMaxDevMm,
								Eigen::Isometry3d& outTemplateToScan, geoalgo::ShapeHandle& outAlignedTemplateShape,
								double& outRmseMm, std::string* errMsg)
{
	outTemplateToScan = Eigen::Isometry3d::Identity();
	outRmseMm = 0.0;
	outAlignedTemplateShape = templateShape.clone();
	geoalgo::ShapeHandle workingTemplate = templateShape.clone();

	std::vector<float> originalSoupXyz;
	std::vector<float> originalSoupNormalsAtStart;
	bool coarsePreTransformApplied = false;

	if (alignMode == RegistrationAlignMode::ManualTrusted)
	{
		RunLogger::info("[TemplateBrepUpdate] coarseStage=globalPreAlign skip (CAD pre-aligned in 3D view)");
	}
	else if (alignMode == RegistrationAlignMode::ManualPartial)
	{
		RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=globalPreAlign skip (partial overlap pairHits=") +
						std::to_string(initialOverlapHits) + "/512); drag CAD workpiece to improve match");
	}
	else
	{
		originalSoupXyz = templateSoupXyz;
		originalSoupNormalsAtStart = templateSoupNormals;
		coarsePreTransformApplied = true;
		RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=globalPreAlign ransac+pca (mode=") +
						registrationAlignModeName(alignMode) + " initialMaxDevMm=" + std::to_string(initialMaxDevMm) +
						")");
		if (!runCoarseGlobalPreAlign(workXyz, workNormals, templateSoupXyz, templateSoupNormals, templateShape,
									 workingTemplate, params, modelDiag, initialMaxDevMm, preAlignGateMm,
									 outTemplateToScan, errMsg))
		{
			return false;
		}
	}

	std::size_t overlapHits = countPointsWithinPairDistance(workXyz, templateSoupXyz, preAlignGateMm, 512U);
	const Eigen::AlignedBox3d scanBox = pclalgo::computeBoundingBox(workXyz);
	const Eigen::AlignedBox3d templateBox = pclalgo::computeBoundingBox(templateSoupXyz);
	const double centroidDistMm =
		(scanBox.isEmpty() || templateBox.isEmpty()) ? 0.0 : (scanBox.center() - templateBox.center()).norm();

	double frameMaxDevMm = 0.0;
	double frameAvgDevMm = 0.0;
	measureScanToCloudDistance(workXyz, templateSoupXyz, frameMaxDevMm, frameAvgDevMm);
	RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=frameCheck pairHits=") + std::to_string(overlapHits) +
					"/512 centroidDistMm=" + std::to_string(centroidDistMm) +
					" maxDevMm=" + std::to_string(frameMaxDevMm) + " avgDevMm=" + std::to_string(frameAvgDevMm));

	const double ransacMaxDevGateMm = std::max(preAlignGateMm * 2.0, std::max(params.faceBandMm * 8.0, 25.0));
	const bool overlapQualityPoor = coarseOverlapQualityPoor(overlapHits, frameMaxDevMm, ransacMaxDevGateMm);
	const bool globalPreAlignInsufficient = coarsePreTransformApplied && overlapQualityPoor &&
											!originalSoupXyz.empty() && overlapHits < kMinResetOverlapHits;
	bool skipSoupForLadder = false;
	if (globalPreAlignInsufficient)
	{
		templateSoupXyz = originalSoupXyz;
		templateSoupNormals = originalSoupNormalsAtStart;
		workingTemplate = templateShape.clone();
		outTemplateToScan = Eigen::Isometry3d::Identity();
		if (centroidDistMm > preAlignGateMm * 2.0)
		{
			std::string snapErr;
			if (!applyCoarseCentroidSnap(workXyz, templateSoupXyz, templateSoupNormals, templateShape, workingTemplate,
										 outTemplateToScan, &snapErr))
			{
				RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=centroidSnap skipped: ") +
								(snapErr.empty() ? "unknown" : snapErr));
			}
		}
		measureScanToCloudDistance(workXyz, templateSoupXyz, frameMaxDevMm, frameAvgDevMm);
		overlapHits = countPointsWithinPairDistance(workXyz, templateSoupXyz, preAlignGateMm, 512U);
		PcaAlignmentScore resetScore{frameMaxDevMm, frameAvgDevMm, overlapHits};
		if (coarseOverlapQualityPoor(overlapHits, frameMaxDevMm, ransacMaxDevGateMm))
		{
			(void)tryCoarseQuarterTurnAlignment(workXyz, templateSoupXyz, templateSoupNormals, templateShape,
												workingTemplate, preAlignGateMm, outTemplateToScan, resetScore);
			frameMaxDevMm = resetScore.maxDevMm;
			frameAvgDevMm = resetScore.meanDistMm;
			overlapHits = resetScore.pairHits;
		}
		if (coarseOverlapQualityPoor(overlapHits, frameMaxDevMm, ransacMaxDevGateMm))
		{
			std::string ransacErr;
			(void)tryCoarseFeatureRansacAlign("postSnapRansac", workXyz, workNormals, templateSoupXyz,
											  templateSoupNormals, templateShape, workingTemplate, params, modelDiag,
											  preAlignGateMm, frameMaxDevMm, outTemplateToScan, resetScore, &ransacErr);
			frameMaxDevMm = resetScore.maxDevMm;
			frameAvgDevMm = resetScore.meanDistMm;
			overlapHits = resetScore.pairHits;
		}
		if (coarseOverlapQualityPoor(overlapHits, frameMaxDevMm, ransacMaxDevGateMm))
		{
			const std::vector<float> soupBeforePca = templateSoupXyz;
			const std::vector<float> normalsBeforePca = templateSoupNormals;
			const Eigen::Isometry3d transformBeforePca = outTemplateToScan;
			const geoalgo::ShapeHandle templateBeforePca = workingTemplate.clone();
			const PcaAlignmentScore scoreBeforePca = resetScore;

			RunLogger::info("[TemplateBrepUpdate] coarseStage=pca after postSnapRansac");
			const Eigen::Isometry3d pcaStep = alignTemplatePcaToScan(templateSoupXyz, templateSoupNormals, workXyz);
			std::string transformErr;
			if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, pcaStep, workingTemplate, &transformErr))
			{
				if (errMsg)
				{
					*errMsg = transformErr.empty() ? "PCA after snap failed" : transformErr;
				}
				return false;
			}
			outTemplateToScan = pcaStep * outTemplateToScan;

			double pcaMaxDevMm = 0.0;
			double pcaAvgDevMm = 0.0;
			measureScanToCloudDistance(workXyz, templateSoupXyz, pcaMaxDevMm, pcaAvgDevMm);
			const std::size_t pcaHits = countPointsWithinPairDistance(workXyz, templateSoupXyz, preAlignGateMm, 512U);
			const PcaAlignmentScore pcaScore{pcaMaxDevMm, pcaAvgDevMm, pcaHits};
			const double pcaAcceptMaxDevMm = ransacMaxDevGateMm;
			if ((!pcaAlignmentScoreBetter(pcaScore, scoreBeforePca) || pcaMaxDevMm > pcaAcceptMaxDevMm) &&
				!pcaStrongOverlapAccept(pcaScore, scoreBeforePca, initialMaxDevMm))
			{
				templateSoupXyz = soupBeforePca;
				templateSoupNormals = normalsBeforePca;
				outTemplateToScan = transformBeforePca;
				workingTemplate = templateBeforePca;
				measureScanToCloudDistance(workXyz, templateSoupXyz, frameMaxDevMm, frameAvgDevMm);
				overlapHits = countPointsWithinPairDistance(workXyz, templateSoupXyz, preAlignGateMm, 512U);
				RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=pca after snap rejected maxDevMm=") +
								std::to_string(pcaMaxDevMm));
			}
			else
			{
				frameMaxDevMm = pcaMaxDevMm;
				frameAvgDevMm = pcaAvgDevMm;
				overlapHits = pcaHits;
			}
		}
		skipSoupForLadder = true;
		RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=globalPreAlign reset (maxDevMm=") +
						std::to_string(frameMaxDevMm) + " pairHits=" + std::to_string(overlapHits) +
						"/512); run coarse ICP ladder from identity");
	}
	else if (coarsePreTransformApplied && overlapQualityPoor)
	{
		RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=globalPreAlign keep (pairHits=") +
						std::to_string(overlapHits) + "/512 maxDevMm=" + std::to_string(frameMaxDevMm) +
						"); refine in place");
		PcaAlignmentScore refineScore{frameMaxDevMm, frameAvgDevMm, overlapHits};
		if (tryCoarseQuarterTurnAlignment(workXyz, templateSoupXyz, templateSoupNormals, templateShape, workingTemplate,
										  preAlignGateMm, outTemplateToScan, refineScore))
		{
			frameMaxDevMm = refineScore.maxDevMm;
			frameAvgDevMm = refineScore.meanDistMm;
			overlapHits = refineScore.pairHits;
		}
	}
	const bool runRansacCoarse = params.enableRansacCoarseMatch && !coarsePreTransformApplied &&
								 (alignMode == RegistrationAlignMode::ColdStart ||
								  (alignMode == RegistrationAlignMode::AutoRecover && overlapQualityPoor));
	const bool useSoupMultiStage =
		alignMode == RegistrationAlignMode::ManualTrusted || alignMode == RegistrationAlignMode::ManualPartial ||
		alignMode == RegistrationAlignMode::AutoRecover ||
		(alignMode == RegistrationAlignMode::ColdStart && overlapHits >= kMinRegistrationOverlapHits);
	const bool allowLargeSoupIcp =
		alignMode == RegistrationAlignMode::AutoRecover || alignMode == RegistrationAlignMode::ColdStart ||
		((alignMode == RegistrationAlignMode::ManualTrusted || alignMode == RegistrationAlignMode::ManualPartial) &&
		 frameMaxDevMm > ransacMaxDevGateMm);
	const bool allowLadderFallback =
		alignMode == RegistrationAlignMode::AutoRecover || alignMode == RegistrationAlignMode::ColdStart;

	if (!coarsePreTransformApplied && !outTemplateToScan.matrix().isApprox(Eigen::Isometry3d::Identity().matrix()))
	{
		std::string transformErr;
		if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, outTemplateToScan, workingTemplate, &transformErr))
		{
			if (errMsg)
			{
				*errMsg = transformErr.empty() ? "failed to apply coarse template transform" : transformErr;
			}
			return false;
		}
	}

	double postCoarseMaxDevMm = frameMaxDevMm;
	if (runRansacCoarse)
	{
		pclalgo::RigidRegisterRansacParams ransacParams;
		ransacParams.maxFeaturePoints = params.icpMaxPoints;
		ransacParams.modelDiagMm = modelDiag;
		ransacParams.faceBandMm = params.faceBandMm;
		ransacParams.maxIterations = params.ransacMaxIterations;
		ransacParams.skipTranslationCap = true;
		if (overlapHits == 0U || frameMaxDevMm > ransacMaxDevGateMm)
		{
			const double fb = std::max(params.faceBandMm, 1.0);
			const double defaultInlier = std::max({modelDiag * 0.04, fb * 3.0, 3.0});
			ransacParams.inlierDistanceMm = std::min(std::max(frameMaxDevMm * 0.6, defaultInlier), modelDiag * 0.15);
			ransacParams.featureVoxelMm = std::max(modelDiag * 0.025, fb * 4.0);
			RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=ransac enlarged inlierDistanceMm=") +
							std::to_string(ransacParams.inlierDistanceMm) +
							" featureVoxelMm=" + std::to_string(ransacParams.featureVoxelMm) +
							" (maxDevMm=" + std::to_string(frameMaxDevMm) + ")");
		}

		Eigen::Isometry3d ransacStep = Eigen::Isometry3d::Identity();
		double inlierRatio = 0.0;
		std::string ransacErr;
		if (pclalgo::rigidRegisterFeatureRansac(templateSoupXyz, templateSoupNormals, workXyz, workNormals, ransacStep,
												&inlierRatio, ransacParams, &ransacErr))
		{
			applyIsometryInPlace(templateSoupXyz, ransacStep);
			applyIsometryInPlaceNormals(templateSoupNormals, ransacStep);
			outTemplateToScan = ransacStep * outTemplateToScan;
			std::string transformErr;
			if (!geoalgo::applyIsometryToShapeHandle(workingTemplate, ransacStep, workingTemplate, &transformErr))
			{
				if (errMsg)
				{
					*errMsg = transformErr.empty() ? "RANSAC template transform failed" : transformErr;
				}
				return false;
			}
			RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=ransac ok inlierRatio=") +
							std::to_string(inlierRatio));
			measureScanToCloudDistance(workXyz, templateSoupXyz, postCoarseMaxDevMm, frameAvgDevMm);
		}
		else
		{
			RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=ransac skipped/failed: ") +
							(ransacErr.empty() ? "unknown" : ransacErr));
		}
	}
	else if (params.enableRansacCoarseMatch && coarsePreTransformApplied)
	{
		RunLogger::info("[TemplateBrepUpdate] coarseStage=ransac skip (globalPreAlign already applied)");
	}
	else if (params.enableRansacCoarseMatch && alignMode == RegistrationAlignMode::ManualTrusted)
	{
		RunLogger::info("[TemplateBrepUpdate] coarseStage=ransac skip (CAD pre-aligned in 3D view)");
	}
	else if (params.enableRansacCoarseMatch && alignMode == RegistrationAlignMode::AutoRecover && !overlapQualityPoor)
	{
		RunLogger::info("[TemplateBrepUpdate] coarseStage=ransac skip (autoRecover overlap quality sufficient)");
	}

	if (useSoupMultiStage)
	{
		bool soupApplied = false;
		if (!skipSoupForLadder)
		{
			(void)runReverseSoupMultiStageIcp(
				workXyz, workNormals, templateSoupXyz, templateSoupNormals, workingTemplate, params, modelDiag,
				outTemplateToScan, outAlignedTemplateShape, outRmseMm, allowLargeSoupIcp, false, soupApplied);
		}

		const std::size_t postSoupHits = countPointsWithinPairDistance(workXyz, templateSoupXyz, preAlignGateMm, 512U);
		double postSoupMaxDevMm = postCoarseMaxDevMm;
		double postSoupAvgDevMm = 0.0;
		measureScanToCloudDistance(workXyz, templateSoupXyz, postSoupMaxDevMm, postSoupAvgDevMm);
		const Eigen::AlignedBox3d postScanBox = pclalgo::computeBoundingBox(workXyz);
		const Eigen::AlignedBox3d postTplBox = pclalgo::computeBoundingBox(templateSoupXyz);
		const double postCentroidDistMm =
			(postScanBox.isEmpty() || postTplBox.isEmpty()) ? 0.0 : (postScanBox.center() - postTplBox.center()).norm();
		const bool preAlignGoodEnough =
			postSoupHits >= 128U && postSoupMaxDevMm <= std::max(ransacMaxDevGateMm * 2.5, preAlignGateMm * 4.0) &&
			!soupApplied && (!skipSoupForLadder || postSoupHits >= kMinRegistrationOverlapHits);
		const bool needLadder = allowLadderFallback && !preAlignGoodEnough &&
								(skipSoupForLadder || !soupApplied ||
								 coarseOverlapQualityPoor(postSoupHits, postSoupMaxDevMm, ransacMaxDevGateMm));
		if (preAlignGoodEnough)
		{
			RunLogger::info(
				std::string("[TemplateBrepUpdate] coarseStage=coarseIcpLadder skip (preAlign ok pairHits=") +
				std::to_string(postSoupHits) + "/512 maxDevMm=" + std::to_string(postSoupMaxDevMm) + ")");
		}
		if (needLadder)
		{
			const double ladderBaselineMaxDev = postCoarseMaxDevMm;
			std::string ladderErr;
			if (runCoarseIcpLadderFallback(workXyz, workNormals, templateSoupXyz, templateSoupNormals, workingTemplate,
										   params, modelDiag, preAlignGateMm, postSoupMaxDevMm, postCentroidDistMm,
										   ladderBaselineMaxDev, outTemplateToScan, outRmseMm, &ladderErr))
			{
				outAlignedTemplateShape = workingTemplate.clone();

				const std::size_t postLadderHits =
					countPointsWithinPairDistance(workXyz, templateSoupXyz, preAlignGateMm, 512U);
				double postLadderMaxDevMm = postSoupMaxDevMm;
				double postLadderAvgDevMm = 0.0;
				measureScanToCloudDistance(workXyz, templateSoupXyz, postLadderMaxDevMm, postLadderAvgDevMm);
				const bool needPostLadderRefine =
					postLadderHits >= kMinRegistrationOverlapHits && postLadderMaxDevMm > ransacMaxDevGateMm;
				if (needPostLadderRefine)
				{
					RunLogger::info("[TemplateBrepUpdate] coarseStage=soupRefine after ladder");
					bool refineApplied = false;
					(void)runReverseSoupMultiStageIcp(workXyz, workNormals, templateSoupXyz, templateSoupNormals,
													  workingTemplate, params, modelDiag, outTemplateToScan,
													  outAlignedTemplateShape, outRmseMm, false, false, refineApplied);
					measureScanToCloudDistance(workXyz, templateSoupXyz, postLadderMaxDevMm, postLadderAvgDevMm);
				}
				if (postLadderMaxDevMm > ransacMaxDevGateMm)
				{
					const double tightBaseline = postLadderMaxDevMm;
					(void)runCoarseTightIcpRefinement(workXyz, workNormals, templateSoupXyz, templateSoupNormals,
													  workingTemplate, params, preAlignGateMm, tightBaseline,
													  outTemplateToScan, outRmseMm);
				}
			}
			else
			{
				RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=coarseIcpLadder failed: ") +
								(ladderErr.empty() ? "unknown" : ladderErr));
			}
		}
		if (coarsePreTransformApplied)
		{
			outAlignedTemplateShape = workingTemplate.clone();
			reconcileTemplateToScanWithSoup(templateShape, originalSoupXyz, templateSoupXyz, workXyz, modelDiag,
											preAlignGateMm, outTemplateToScan, outAlignedTemplateShape);
			normalizeCoarseTemplateToScanMatrix(originalSoupXyz, templateSoupXyz, workXyz, modelDiag, preAlignGateMm,
												outTemplateToScan);
		}
		return true;
	}

	if (!runColdStartCoarseIcpPath(workXyz, workNormals, templateSoupXyz, templateSoupNormals, workingTemplate, params,
								   alignMode, modelDiag, preAlignGateMm, initialMaxDevMm, centroidDistMm,
								   outTemplateToScan, outAlignedTemplateShape, outRmseMm, errMsg))
	{
		return false;
	}
	if (coarsePreTransformApplied)
	{
		outAlignedTemplateShape = workingTemplate.clone();
		reconcileTemplateToScanWithSoup(templateShape, originalSoupXyz, templateSoupXyz, workXyz, modelDiag,
										preAlignGateMm, outTemplateToScan, outAlignedTemplateShape);
		normalizeCoarseTemplateToScanMatrix(originalSoupXyz, templateSoupXyz, workXyz, modelDiag, preAlignGateMm,
											outTemplateToScan);
	}
	return true;
}

bool alignScanToTemplateRegistration(const std::vector<float>& workXyz, const std::vector<float>& workNormals,
									 const geoalgo::ShapeHandle& templateShape, std::vector<float>& templateSampleXyz,
									 std::vector<float>& templateSampleNormals,
									 const geoalgo::TemplateBrepUpdateParams& params,
									 Eigen::Isometry3d& outTemplateToScan,
									 geoalgo::ShapeHandle& outAlignedTemplateShape, double& outRmseMm,
									 std::string* errMsg)
{
	outTemplateToScan = Eigen::Isometry3d::Identity();
	outRmseMm = 0.0;
	outAlignedTemplateShape = templateShape.clone();

	const double scanDiag = boundingBoxDiagonalMm(workXyz);
	const double templateDiag = boundingBoxDiagonalMm(templateSampleXyz);
	const double modelDiag = std::max(scanDiag, templateDiag);
	if (modelDiag <= 0.0)
	{
		if (errMsg)
		{
			*errMsg = "invalid scan or template bounds for registration";
		}
		return false;
	}

	if (templateSampleNormals.size() != templateSampleXyz.size() || templateSampleNormals.empty())
	{
		templateSampleNormals.clear();
		if (!pclalgo::estimateNormalsPca(templateSampleXyz, templateSampleNormals, 16U, nullptr))
		{
			templateSampleNormals.clear();
		}
		else
		{
			std::vector<float> templateForOrient = templateSampleXyz;
			(void)pclalgo::orientNormalsMst(templateForOrient, templateSampleNormals, 16U, nullptr, nullptr);
		}
	}

	const double preAlignGateMm = std::max({params.faceBandMm * 6.0, modelDiag * 0.02, 15.0});
	const std::size_t initialOverlapHits =
		countPointsWithinPairDistance(workXyz, templateSampleXyz, preAlignGateMm, 512U);
	double initialMaxDevMm = 0.0;
	double initialAvgDevMm = 0.0;
	measureScanToCloudDistance(workXyz, templateSampleXyz, initialMaxDevMm, initialAvgDevMm);
	const RegistrationAlignMode alignMode =
		resolveRegistrationAlignMode(params, initialOverlapHits, initialMaxDevMm, preAlignGateMm);
	RunLogger::info(std::string("[TemplateBrepUpdate] coarseStage=modeSelect mode=") +
					registrationAlignModeName(alignMode) + " pairHits=" + std::to_string(initialOverlapHits) +
					"/512 maxDevMm=" + std::to_string(initialMaxDevMm) +
					" avgDevMm=" + std::to_string(initialAvgDevMm));

	if (!runCoarseAlignmentPipeline(workXyz, workNormals, templateShape, templateSampleXyz, templateSampleNormals,
									params, alignMode, modelDiag, preAlignGateMm, initialOverlapHits, initialMaxDevMm,
									outTemplateToScan, outAlignedTemplateShape, outRmseMm, errMsg))
	{
		return false;
	}
	return true;
}

bool runRegistrationCoarsePipelineSelfTestImpl(std::string* errMsg)
{
	geoalgo::TemplateBrepUpdateParams modeParams;
	if (resolveRegistrationAlignMode(modeParams, 0U, 50.0, 25.0) != RegistrationAlignMode::AutoRecover)
	{
		if (errMsg)
		{
			*errMsg = "pairHits=0 must select autoRecover";
		}
		return false;
	}
	if (resolveRegistrationAlignMode(modeParams, 16U, 50.0, 25.0) != RegistrationAlignMode::ManualPartial)
	{
		if (errMsg)
		{
			*errMsg = "partial overlap with moderate maxDev must stay manualPartial";
		}
		return false;
	}
	if (resolveRegistrationAlignMode(modeParams, 0U, 200.0, 25.0) != RegistrationAlignMode::AutoRecover)
	{
		if (errMsg)
		{
			*errMsg = "pairHits=0 with large maxDev must stay autoRecover";
		}
		return false;
	}
	if (resolveRegistrationAlignMode(modeParams, 40U, 10.0, 25.0) != RegistrationAlignMode::ManualTrusted)
	{
		if (errMsg)
		{
			*errMsg = "pairHits>=32 must select manualTrusted";
		}
		return false;
	}

	const std::vector<double> ladderPairs = buildCoarseIcpLadderMaxPairsMm(350.0, 400.0, 500.0, 30.0, 2.0);
	if (ladderPairs.empty() || ladderPairs.front() >= ladderPairs.back())
	{
		if (errMsg)
		{
			*errMsg = "coarse ICP ladder maxPair candidates must be non-empty and ascending";
		}
		return false;
	}
	return true;
}

bool alignScanToTemplateIcp(std::vector<float>& workXyz, std::vector<float>& workNormals,
							const geoalgo::ShapeHandle& templateShape, std::vector<float>& templateSampleXyz,
							std::vector<float>& templateSampleNormals, const geoalgo::TemplateBrepUpdateParams& params,
							Eigen::Isometry3d& outTemplateToScan, geoalgo::ShapeHandle& outAlignedTemplateShape,
							double& outRmseMm, std::string* errMsg)
{
	return alignScanToTemplateRegistration(workXyz, workNormals, templateShape, templateSampleXyz,
										   templateSampleNormals, params, outTemplateToScan, outAlignedTemplateShape,
										   outRmseMm, errMsg);
}

bool preparePointCloudWork(std::vector<float>& workXyz, std::vector<float>& workNormals,
						   const std::size_t inputPointCount, std::size_t& outWorkPointCount, std::string* errMsg,
						   const double voxelPrefilterMm, const double outlierRemovalPercent)
{
	outWorkPointCount = 0U;

	if (workXyz.size() < 9U)
	{
		if (errMsg)
		{
			*errMsg = "too few points";
		}
		return false;
	}

	if (voxelPrefilterMm > 0.0)
	{
		if (!pclalgo::downsampleVoxelGrid(workXyz, voxelPrefilterMm))
		{
			if (errMsg)
			{
				*errMsg = "voxel prefilter failed";
			}
			return false;
		}
		workNormals.clear();
	}

	if (outlierRemovalPercent > 0.0)
	{
		if (!pclalgo::removeOutliers(workXyz, outlierRemovalPercent, 24U, nullptr, nullptr, errMsg))
		{
			return false;
		}
		workNormals.clear();
	}

	if (workNormals.size() != workXyz.size())
	{
		workNormals.clear();
	}

	if (workNormals.empty())
	{
		if (!pclalgo::estimateNormalsPca(workXyz, workNormals, 12U, errMsg))
		{
			return false;
		}
		if (!pclalgo::orientNormalsMst(workXyz, workNormals, 12U, nullptr, errMsg))
		{
			return false;
		}
	}

	outWorkPointCount = workXyz.size() / 3U;
	if (outWorkPointCount < 3U)
	{
		if (errMsg)
		{
			std::ostringstream oss;
			oss << "too few points after prefilter (input=" << inputPointCount << " work=" << outWorkPointCount << ")";
			*errMsg = oss.str();
		}
		return false;
	}

	return true;
}

double effectiveFaceBandMmForAssignment(const double userFaceBandMm, const geoalgo::TemplateBrepUpdateResult& report)
{
	double bandMm = std::max(userFaceBandMm, 0.5);
	if (report.icpRmseMm > 0.0)
	{
		bandMm = std::max(bandMm, report.icpRmseMm * 1.25);
	}
	if (report.registrationOverlapMaxDevMm > 0.0)
	{
		bandMm = std::max(bandMm, std::min(report.registrationOverlapMaxDevMm * 0.75, 25.0));
	}
	return bandMm;
}

} // namespace

bool discretizeStepToMesh(const std::string& stepPathUtf8, const geoalgo::MeshDiscretizeParams& params,
						  std::vector<float>& soup, geoalgo::MeshDiscretizeReport& report, std::string* errMsg)
{
	if (!geoalgo::tessellateStepFileToMesh(stepPathUtf8, params, soup, report, errMsg))
	{
		return false;
	}

	if (params.densityControl != geoalgo::MeshDensityControl::TargetEdgeLength || !(params.targetEdgeLengthMm > 0.0))
	{
		return true;
	}

#if defined(_WIN64)
	// 单次 remesh：塌缩过密圆角 + 加密大面到目标边长；失败保留 refine 结果
	constexpr std::size_t kRemeshMaxTris = 800000U;
	if (soup.size() / 9U > kRemeshMaxTris)
	{
		geoalgo::fillMeshReport(soup, report);
		return !soup.empty();
	}

	std::vector<float> working = soup;
	{
		vcgalgo::RepairParams repairParams;
		std::vector<float> repaired;
		if (vcgalgo::repairMesh(working, repaired, repairParams, nullptr, nullptr) && !repaired.empty())
		{
			working = std::move(repaired);
		}
	}

	std::vector<float> remeshed;
	if (vcgalgo::isotropicRemesh(working, params.targetEdgeLengthMm, remeshed, 3, 30.0, nullptr) && !remeshed.empty())
	{
		soup = std::move(remeshed);
	}
	else
	{
		soup = std::move(working);
	}
	geoalgo::fillMeshReport(soup, report);
	return !soup.empty();
#else
	if (errMsg)
	{
		*errMsg = "target edge length remesh requires Win64 VcgAlgorithms";
	}
	return false;
#endif
}

bool discretizeStepFaceToMesh(const std::string& stepPathUtf8, int faceIndex,
							  const geoalgo::MeshDiscretizeParams& params, std::vector<float>& soup,
							  geoalgo::MeshDiscretizeReport& report, std::string* errMsg)
{
	return geoalgo::discretizeStepFaceToMesh(stepPathUtf8, faceIndex, params, soup, report, errMsg);
}

bool discretizePolylineToMesh(const std::vector<float>& polylineXyz, const geoalgo::MeshDiscretizeParams& params,
							  std::vector<float>& soup, std::string* errMsg)
{
	geoalgo::Polyline3d poly;
	poly.xyz = polylineXyz;
	return geoalgo::discretizePolylineToMesh(poly, params, soup, errMsg);
}

bool discretizeStepEdgesToPolylines(const std::string& stepPathUtf8, const geoalgo::TessellateParams& params,
									std::vector<geoalgo::Polyline3d>& outPolylines, std::string* errMsg)
{
	return geoalgo::discretizeStepEdgesToPolylines(stepPathUtf8, params, outPolylines, errMsg);
}

bool intersectStepEdges(const std::string& stepPathUtf8, int edgeIndex1, int edgeIndex2,
						const geoalgo::IntersectionParams& params, geoalgo::IntersectionResult& result,
						std::string* errMsg)
{
	return geoalgo::intersectStepEdges(stepPathUtf8, edgeIndex1, edgeIndex2, params, result, errMsg);
}

bool intersectStepEdgeFace(const std::string& stepPathUtf8, int edgeIndex, int faceIndex,
						   const geoalgo::IntersectionParams& params, geoalgo::IntersectionResult& result,
						   std::string* errMsg)
{
	return geoalgo::intersectStepEdgeFace(stepPathUtf8, edgeIndex, faceIndex, params, result, errMsg);
}

bool intersectStepFaces(const std::string& stepPathUtf8, int faceIndex1, int faceIndex2,
						const geoalgo::IntersectionParams& params, geoalgo::IntersectionResult& result,
						std::string* errMsg)
{
	return geoalgo::intersectStepFaces(stepPathUtf8, faceIndex1, faceIndex2, params, result, errMsg);
}

bool intersectStepFiles(const std::string& targetStepPathUtf8, const std::string& toolStepPathUtf8,
						const geoalgo::IntersectionParams& params, geoalgo::IntersectionResult& result,
						std::string* errMsg)
{
	return geoalgo::intersectStepFiles(targetStepPathUtf8, toolStepPathUtf8, params, result, errMsg);
}

bool brepBooleanStepFilesToMesh(const std::string& targetStepPathUtf8, const std::string& toolStepPathUtf8,
								geoalgo::BrepBooleanOp op, const geoalgo::MeshDiscretizeParams& meshParams,
								std::vector<float>& outSoup, std::string* errMsg)
{
	return geoalgo::brepBooleanStepFilesToMesh(targetStepPathUtf8, toolStepPathUtf8, op, meshParams, outSoup, errMsg);
}

bool fuseStepEdgesToPolyline(const std::string& stepPathUtf8, const std::vector<int>& edgeIndices,
							 const geoalgo::TessellateParams& disc, geoalgo::Polyline3d& out, std::string* errMsg)
{
	return geoalgo::fuseStepEdgesToPolyline(stepPathUtf8, edgeIndices, disc, out, errMsg);
}

bool sewStepFacesToMesh(const std::string& stepPathUtf8, const std::vector<int>& faceIndices, double toleranceMm,
						const geoalgo::MeshDiscretizeParams& meshParams, std::vector<float>& outSoup,
						std::string* errMsg)
{
	return geoalgo::sewStepFacesToMesh(stepPathUtf8, faceIndices, toleranceMm, meshParams, outSoup, errMsg);
}

void applyQualityPreset(geoalgo::MeshDiscretizeParams& params)
{
	geoalgo::applyQualityPreset(params);
}

void fillMeshReport(const std::vector<float>& soup, geoalgo::MeshDiscretizeReport& report)
{
	geoalgo::fillMeshReport(soup, report);
}

bool resolveGeometryRef(const GeometryRef& ref, geoalgo::WorkpieceRef& out, std::string* errMsg)
{
	if (ref.backendIdUtf8.empty() && ref.stepPathUtf8.empty())
	{
		if (errMsg)
		{
			*errMsg = "geometry ref missing backendIdUtf8 and stepPathUtf8";
		}
		return false;
	}
	out.backendIdUtf8 = ref.backendIdUtf8;
	out.stepPathUtf8 = ref.stepPathUtf8;
	out.frameId = ref.frameId.empty() ? "workpiece" : ref.frameId;
	return true;
}

WorkpieceShapeSource resolveWorkpieceShape(const std::string& backendIdUtf8, BackendDataManager& mgr,
										   const std::string& stepPathUtf8Optional, geoalgo::ShapeHandle& outShape,
										   geoalgo::WorkpieceRef& outRef, std::string* errMsg)
{
	outShape = geoalgo::ShapeHandle{};
	outRef = geoalgo::WorkpieceRef{};
	outRef.backendIdUtf8 = backendIdUtf8;
	outRef.frameId = "workpiece";
	outRef.stepPathUtf8 = stepPathUtf8Optional;

	const auto data = mgr.getData(backendIdUtf8);
	if (!data)
	{
		if (errMsg)
		{
			*errMsg = "backend not found";
		}
		return WorkpieceShapeSource::Unavailable;
	}

	if (auto brep = std::dynamic_pointer_cast<BrepBackendData>(data))
	{
		if (!brep->hasGeometry())
		{
			if (errMsg)
			{
				*errMsg = "BrepModel has no shape";
			}
			return WorkpieceShapeSource::Unavailable;
		}
		outShape = brep->shapeRef();
		return WorkpieceShapeSource::InMemoryBrep;
	}

	if (!stepPathUtf8Optional.empty())
	{
		if (!geoalgo::readStepIntoHandle(stepPathUtf8Optional, outShape, errMsg))
		{
			return WorkpieceShapeSource::Unavailable;
		}
		return WorkpieceShapeSource::StepFileFallback;
	}

	if (errMsg)
	{
		*errMsg = "workpiece has no in-memory B-rep and no STEP path";
	}
	return WorkpieceShapeSource::Unavailable;
}

bool discretizeFeatureList(const geoalgo::FeatureListDocument& doc, geoalgo::RawPath& out, std::string* errMsg)
{
	return geoalgo::discretizeFeatureList(doc, out, errMsg);
}

bool discretizeFeatureList(const geoalgo::FeatureListDocument& doc, const geoalgo::ShapeHandle& shape,
						   geoalgo::RawPath& out, std::string* errMsg)
{
	return geoalgo::discretizeFeatureList(doc, shape, out, errMsg);
}

bool featureListFromJson(const std::string& jsonUtf8, geoalgo::FeatureListDocument& out, std::string* errMsg)
{
	return geoalgo::featureListFromJson(jsonUtf8, out, errMsg);
}

std::string featureListToJson(const geoalgo::FeatureListDocument& doc)
{
	return geoalgo::featureListToJson(doc);
}

bool validateFeatureListDocument(const geoalgo::FeatureListDocument& doc, std::string* errMsg)
{
	return geoalgo::validateFeatureListDocument(doc, errMsg);
}

bool enumerateFeatureCatalog(const geoalgo::WorkpieceRef& workpiece, geoalgo::FeatureCatalog& out, std::string* errMsg)
{
	return geoalgo::enumerateFeatureCatalog(workpiece, out, errMsg);
}

bool enumerateFeatureCatalog(const geoalgo::WorkpieceRef& workpiece, const geoalgo::ShapeHandle& shape,
							 geoalgo::FeatureCatalog& out, std::string* errMsg)
{
	return geoalgo::enumerateFeatureCatalog(workpiece, shape, out, errMsg);
}

std::string featureCatalogToJson(const geoalgo::FeatureCatalog& catalog)
{
	return geoalgo::featureCatalogToJson(catalog);
}

bool suggestFeaturesFromCatalog(const geoalgo::FeatureCatalog& catalog, const std::string& intentUtf8,
								geoalgo::FeatureListDocument& out, std::string* errMsg)
{
	return geoalgo::suggestFeaturesFromCatalog(catalog, intentUtf8, out, errMsg);
}

bool computeFeatureAnchor(const geoalgo::WorkpieceRef& workpiece, const geoalgo::FeatureGeometry& geometry,
						  geoalgo::FeatureAnchor& out, std::string* errMsg)
{
	return geoalgo::computeFeatureAnchor(workpiece, geometry, out, errMsg);
}

bool computeFeatureAnchor(const geoalgo::WorkpieceRef& workpiece, const geoalgo::ShapeHandle& shape,
						  const geoalgo::FeatureGeometry& geometry, geoalgo::FeatureAnchor& out, std::string* errMsg)
{
	return geoalgo::computeFeatureAnchor(workpiece, shape, geometry, out, errMsg);
}

void ensureFeatureDiscretizersRegistered()
{
	geoalgo::ensureFeatureDiscretizersRegistered();
}

bool ensureFeatureDiscretizerConfigsLoaded(const std::string& resourceBaseDir, std::string* errMsg)
{
	return geoalgo::ensureFeatureDiscretizerConfigsLoaded(resourceBaseDir, errMsg);
}

std::vector<std::string> featureDiscretizerListStrategyIds()
{
	return geoalgo::featureDiscretizerListStrategyIds();
}

std::vector<geoalgo::FeatureDiscretizerParamField> featureDiscretizerAllParamFields(const std::string& strategyId)
{
	return geoalgo::featureDiscretizerAllParamFields(strategyId);
}

std::string featureDiscretizerDisplayNameZh(const std::string& strategyId)
{
	if (const geoalgo::IFeatureDiscretizer* algo = geoalgo::featureDiscretizerGet(strategyId))
	{
		return algo->displayNameZh();
	}
	return strategyId;
}

geoalgo::GeometryAffinity featureDiscretizerAffinity(const std::string& strategyId)
{
	if (const geoalgo::IFeatureDiscretizer* algo = geoalgo::featureDiscretizerGet(strategyId))
	{
		return algo->affinity();
	}
	return geoalgo::GeometryAffinity::Any;
}

nlohmann::json featureDiscretizerDefaultParams(const std::string& strategyId)
{
	return geoalgo::featureDiscretizerConfigRegistry().defaultParamsForStrategy(strategyId);
}

bool buildFeatureEntryFromModelPick(const geoalgo::WorkpieceRef& workpiece, const geoalgo::ShapeHandle& shape,
									const std::string& strategyId, const bool pickFace,
									const geoalgo::Point3d& modelPointA, const geoalgo::Point3d& modelPointB,
									geoalgo::FeatureEntry& out, std::string* errMsg, const int knownFaceIndex,
									const int knownEdgeIndex)
{
	out = geoalgo::FeatureEntry{};
	out.strategyId = strategyId;
	out.params = geoalgo::featureDiscretizerConfigRegistry().defaultParamsForStrategy(strategyId);
	if (pickFace)
	{
		int faceIdx = knownFaceIndex;
		if (faceIdx < 0)
		{
			if (!geoalgo::resolveFaceIndexFromModelPoint(shape, modelPointA, faceIdx, 2.0, errMsg))
			{
				return false;
			}
		}
		else if (!geoalgo::validateShapeFaceIndex(shape, faceIdx, errMsg))
		{
			return false;
		}
		out.featureId = "face_" + std::to_string(faceIdx);
		out.geometry.faceIndices = {faceIdx};
	}
	else
	{
		int edgeIdx = knownEdgeIndex;
		if (edgeIdx < 0)
		{
			if (!geoalgo::resolveEdgeIndexFromModelPoints(shape, modelPointA, modelPointB, edgeIdx, 2.0, errMsg))
			{
				return false;
			}
		}
		else if (!geoalgo::validateShapeEdgeIndex(shape, edgeIdx, errMsg))
		{
			return false;
		}
		out.featureId = "edge_" + std::to_string(edgeIdx);
		out.geometry.edgeIndices = {edgeIdx};
	}
	(void)workpiece;
	return true;
}

bool sampleTriangleSoupToPointBuffers(const std::vector<float>& soup, const std::vector<float>& soupNormals,
									  std::vector<float>& outXyz, std::vector<float>& outNormals,
									  const std::size_t maxPoints, std::string* errMsg)
{
	outXyz.clear();
	outNormals.clear();
	if (soup.size() < 9U || soup.size() % 9U != 0U)
	{
		if (errMsg)
		{
			*errMsg = "sampleTriangleSoupToPointBuffers: invalid triangle soup";
		}
		return false;
	}
	if (maxPoints < 3U)
	{
		if (errMsg)
		{
			*errMsg = "sampleTriangleSoupToPointBuffers: maxPoints too small";
		}
		return false;
	}

	const bool hasNormals = soupNormals.size() == soup.size();
	const std::size_t triCount = soup.size() / 9U;
	const std::size_t triStep = std::max<std::size_t>(1U, triCount / std::max<std::size_t>(1U, maxPoints / 3U));
	outXyz.reserve(std::min(maxPoints, triCount * 3U) * 3U);
	if (hasNormals)
	{
		outNormals.reserve(outXyz.capacity());
	}

	for (std::size_t tri = 0U; tri < triCount; tri += triStep)
	{
		if ((outXyz.size() / 3U) >= maxPoints)
		{
			break;
		}
		const std::size_t base = tri * 9U;
		for (std::size_t corner = 0U; corner < 3U; ++corner)
		{
			if ((outXyz.size() / 3U) >= maxPoints)
			{
				break;
			}
			const std::size_t vb = base + corner * 3U;
			outXyz.push_back(soup[vb]);
			outXyz.push_back(soup[vb + 1U]);
			outXyz.push_back(soup[vb + 2U]);
			if (hasNormals)
			{
				outNormals.push_back(soupNormals[vb]);
				outNormals.push_back(soupNormals[vb + 1U]);
				outNormals.push_back(soupNormals[vb + 2U]);
			}
			else
			{
				const float* p0 = &soup[base];
				const float* p1 = &soup[base + 3U];
				const float* p2 = &soup[base + 6U];
				const double e1x = static_cast<double>(p1[0] - p0[0]);
				const double e1y = static_cast<double>(p1[1] - p0[1]);
				const double e1z = static_cast<double>(p1[2] - p0[2]);
				const double e2x = static_cast<double>(p2[0] - p0[0]);
				const double e2y = static_cast<double>(p2[1] - p0[1]);
				const double e2z = static_cast<double>(p2[2] - p0[2]);
				double nx = e1y * e2z - e1z * e2y;
				double ny = e1z * e2x - e1x * e2z;
				double nz = e1x * e2y - e1y * e2x;
				const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
				if (len > 1e-12)
				{
					nx /= len;
					ny /= len;
					nz /= len;
				}
				else
				{
					nx = 0.0;
					ny = 0.0;
					nz = 1.0;
				}
				outNormals.push_back(static_cast<float>(nx));
				outNormals.push_back(static_cast<float>(ny));
				outNormals.push_back(static_cast<float>(nz));
			}
		}
	}

	if (outXyz.size() < 9U)
	{
		if (errMsg)
		{
			*errMsg = "sampleTriangleSoupToPointBuffers: no points sampled";
		}
		return false;
	}
	return true;
}

bool buildPointCloudFromMeshForTemplateBrep(const MeshBackendData& mesh, PointCloudBackendData& outScan,
											const std::size_t maxPoints, std::string* errMsg)
{
	outScan.clearGeometry();
	std::vector<float> xyz;
	std::vector<float> normals;
	if (!sampleTriangleSoupToPointBuffers(mesh.triangleSoup(), mesh.triangleVertexNormals(), xyz, normals, maxPoints,
										  errMsg))
	{
		return false;
	}
	outScan.setPointBuffers(std::move(xyz), {}, std::move(normals));
	outScan.setWorldMatrix(mesh.worldMatrix());
	return true;
}

bool registerScanToCadTemplate(const BrepBackendData& templateBrep, const PointCloudBackendData& scanCloud,
							   geoalgo::TemplateBrepUpdateParams params, geoalgo::TemplateBrepUpdateResult& outReport,
							   std::string* errMsg, const std::string& templateStepPathUtf8,
							   geoalgo::TemplateBrepRegistrationCheckpoint* registrationCheckpoint)
{
	outReport = {};

	const geoalgo::ShapeHandle templateShapeForIcp =
		resolveTemplateShapeForIcp(templateBrep, templateStepPathUtf8, errMsg);
	if (templateShapeForIcp.isNull())
	{
		return false;
	}

	std::vector<float> workXyz = scanCloud.pointPositionsXyz();
	std::vector<float> workNormals = scanCloud.pointNormalsNxNyNz();
	const std::size_t inputPointCount = workXyz.size() / 3U;

	std::size_t workPointCount = 0U;
	if (!preparePointCloudWork(workXyz, workNormals, inputPointCount, workPointCount, errMsg, params.voxelPrefilterMm,
							   params.outlierRemovalPercent))
	{
		return false;
	}
	RunLogger::info("[TemplateBrepUpdate] preprocess done, workPts=" + std::to_string(workPointCount));

	const BackendMat4 scanWorld = scanCloud.worldMatrix();
	const BackendMat4 templateWorld = templateBrep.worldMatrix();
	transformXyzByEngineWorldMatrix(workXyz, scanWorld);
	if (!workNormals.empty())
	{
		transformNormalsByEngineWorldMatrix(workNormals, scanWorld);
	}
	RunLogger::info("[TemplateBrepUpdate] registration in world frame (scan/template worldMatrix)");

	const bool fineFromCheckpoint = params.registrationStage == geoalgo::TemplateBrepRegistrationStage::FineOnly &&
									registrationCheckpoint != nullptr && registrationCheckpoint->valid;

	std::vector<float> templateSampleXyz;
	std::vector<float> templateSampleNormals;
	std::size_t templateTriCount = 0U;
	constexpr std::size_t kMaxTemplateSoupPoints = 40000U;
	if (fineFromCheckpoint)
	{
		if (!geoalgo::extractDisplaySoupPointCloud(templateShapeForIcp, templateSampleXyz, templateSampleNormals,
												   kMaxTemplateSoupPoints, &templateTriCount, errMsg))
		{
			return false;
		}
		transformXyzByEngineWorldMatrix(templateSampleXyz, templateWorld);
		if (!templateSampleNormals.empty())
		{
			transformNormalsByEngineWorldMatrix(templateSampleNormals, templateWorld);
		}
		RunLogger::info(std::string("[TemplateBrepUpdate] template soup pts=") +
						std::to_string(templateSampleXyz.size() / 3U) + " (fine: rebuilt from template.worldMatrix)");
	}
	else
	{
		if (!geoalgo::extractDisplaySoupPointCloud(templateShapeForIcp, templateSampleXyz, templateSampleNormals,
												   kMaxTemplateSoupPoints, &templateTriCount, errMsg))
		{
			return false;
		}
		RunLogger::info(std::string("[TemplateBrepUpdate] template soup pts=") +
						std::to_string(templateSampleXyz.size() / 3U) + " (from " + std::to_string(templateTriCount) +
						" tris)");
		transformXyzByEngineWorldMatrix(templateSampleXyz, templateWorld);
		if (!templateSampleNormals.empty())
		{
			transformNormalsByEngineWorldMatrix(templateSampleNormals, templateWorld);
		}
	}

	std::vector<float> icpWorkXyz = workXyz;
	std::vector<float> icpWorkNormals = workNormals;
	constexpr std::size_t kMaxIcpScanPoints = 60000U;
	subsampleXyzWithNormalsUniform(icpWorkXyz, icpWorkNormals, kMaxIcpScanPoints);
	if (icpWorkXyz.size() / 3U < workXyz.size() / 3U)
	{
		RunLogger::info(std::string("[TemplateBrepUpdate] ICP scan subsampled to ") +
						std::to_string(icpWorkXyz.size() / 3U) + " pts");
	}

	if (fineFromCheckpoint)
	{
		params.registrationMatchVoxelMm =
			downsampleRegistrationPairToTarget(icpWorkXyz, icpWorkNormals, templateSampleXyz, templateSampleNormals,
											   kFineRegistrationPairTargetPoints, kFineMaxMatchVoxelMm);
	}
	else
	{
		params.registrationMatchVoxelMm = downsampleRegistrationPairToTarget(
			icpWorkXyz, icpWorkNormals, templateSampleXyz, templateSampleNormals, kRegistrationPairTargetPoints);
	}

	const double modelDiag = std::max(boundingBoxDiagonalMm(icpWorkXyz), boundingBoxDiagonalMm(templateSampleXyz));

	Eigen::Isometry3d icpDeltaWorld = Eigen::Isometry3d::Identity();
	geoalgo::ShapeHandle unusedAlignedShape;

	if (params.registrationStage == geoalgo::TemplateBrepRegistrationStage::FineOnly)
	{
		if (!fineFromCheckpoint)
		{
			if (errMsg)
			{
				*errMsg = "run coarse registration first (FineOnly requires checkpoint)";
			}
			return false;
		}
		const Eigen::Isometry3d coarseTotalDelta = registrationCheckpoint->icpDeltaWorld;
		icpDeltaWorld = Eigen::Isometry3d::Identity();
		geoalgo::ShapeHandle workingTemplate = templateShapeForIcp.clone();
		if (!runFineRegistrationStage(icpWorkXyz, icpWorkNormals, templateSampleXyz, templateSampleNormals,
									  workingTemplate, params, modelDiag, icpDeltaWorld, unusedAlignedShape,
									  outReport.icpRmseMm, errMsg))
		{
			return false;
		}
		const Eigen::Isometry3d fineIncrement = icpDeltaWorld;
		icpDeltaWorld = fineIncrement * coarseTotalDelta;
	}
	else if (!alignScanToTemplateRegistration(icpWorkXyz, icpWorkNormals, templateShapeForIcp, templateSampleXyz,
											  templateSampleNormals, params, icpDeltaWorld, unusedAlignedShape,
											  outReport.icpRmseMm, errMsg))
	{
		return false;
	}

	saveRegistrationCheckpoint(registrationCheckpoint, templateSampleXyz, templateSampleNormals, icpDeltaWorld);

	if (params.registrationStage == geoalgo::TemplateBrepRegistrationStage::CoarseOnly)
	{
		RunLogger::info("[TemplateBrepUpdate] coarse-only registration stage complete");
	}

	outReport.icpDeltaWorld = icpDeltaWorld;
	outReport.templateToScan = icpDeltaWorld;

	RunLogger::info(std::string("[TemplateBrepUpdate] reverse registration done, icpRmseMm=") +
					std::to_string(outReport.icpRmseMm));

	const double previewOverlapGateMm = std::max(params.faceBandMm * 10.0, 25.0);
	const double overlapModelDiag =
		std::max(boundingBoxDiagonalMm(icpWorkXyz), boundingBoxDiagonalMm(templateSampleXyz));
	const double coarseReportGateMm = std::max({params.faceBandMm * 6.0, overlapModelDiag * 0.02, 15.0});
	const bool fineReportStage = params.registrationStage == geoalgo::TemplateBrepRegistrationStage::FineOnly;

	double overlapMaxAll = 0.0;
	double overlapAvgAll = 0.0;
	measureScanToCloudDistance(icpWorkXyz, templateSampleXyz, overlapMaxAll, overlapAvgAll);

	double inlierMax = 0.0;
	double inlierAvg = 0.0;
	std::size_t inlierHits = 0U;
	double reportInlierGateMm = coarseReportGateMm;
	if (fineReportStage)
	{
		double preInlierMax = 0.0;
		double preInlierAvg = 0.0;
		std::size_t preInlierHits = 0U;
		measureScanToCloudInlierStats(icpWorkXyz, templateSampleXyz, coarseReportGateMm, preInlierMax, preInlierAvg,
									  preInlierHits);
		reportInlierGateMm = fineOverlapInlierGateMm(params, preInlierMax);
	}
	measureScanToCloudInlierStats(icpWorkXyz, templateSampleXyz, reportInlierGateMm, inlierMax, inlierAvg, inlierHits);

	const double pipelineIcpRmseMm = outReport.icpRmseMm;
	if (inlierHits >= kMinRegistrationOverlapHits && inlierAvg > 0.0)
	{
		outReport.icpRmseMm = inlierAvg;
		RunLogger::info(std::string("[TemplateBrepUpdate] icpRmseMm from inlier overlap avg=") +
						std::to_string(inlierAvg) + "mm (pipeline reported " + std::to_string(pipelineIcpRmseMm) +
						"mm)");
	}

	outReport.registrationOverlapMaxDevMm = inlierHits >= kMinRegistrationOverlapHits ? inlierMax : overlapMaxAll;
	RunLogger::info(
		std::string("[TemplateBrepUpdate] registration overlap inlierHits=") + std::to_string(inlierHits) +
		"/512 inlierMaxDevMm=" + std::to_string(inlierMax) + " inlierAvgDevMm=" + std::to_string(inlierAvg) +
		" globalMaxDevMm=" + std::to_string(overlapMaxAll) + " reportGateMm=" + std::to_string(reportInlierGateMm));

	const double icpGateMm = std::max(params.faceBandMm * params.maxIcpRmseToFaceBandRatio, params.minIcpRmseGateMm);
	outReport.icpRmseGatePassed = params.maxIcpRmseToFaceBandRatio <= 0.0 || outReport.icpRmseMm <= icpGateMm;

	const double effectiveOverlapMaxMm = inlierHits >= kMinRegistrationOverlapHits ? inlierMax : overlapMaxAll;
	outReport.registrationPreviewOk = inlierHits >= kMinRegistrationOverlapHits && effectiveOverlapMaxMm > 0.0 &&
									  effectiveOverlapMaxMm <= previewOverlapGateMm;
	if (params.registrationStage != geoalgo::TemplateBrepRegistrationStage::CoarseOnly)
	{
		outReport.registrationPreviewOk = outReport.registrationPreviewOk && outReport.icpRmseGatePassed;
	}

	if (outReport.registrationPreviewOk)
	{
		RunLogger::info(std::string("[TemplateBrepUpdate] registration preview ok, inlierMaxDevMm=") +
						std::to_string(inlierMax));
	}

	if (!outReport.icpRmseGatePassed)
	{
		RunLogger::info(std::string("[TemplateBrepUpdate] ICP RMSE gate failed: rmse=") +
						std::to_string(outReport.icpRmseMm) + "mm, need <= " + std::to_string(icpGateMm) + "mm");
		if (errMsg)
		{
			std::ostringstream oss;
			oss << "ICP RMSE above face-update gate: rmse=" << outReport.icpRmseMm << "mm, need <= " << icpGateMm
				<< "mm (faceBand=" << params.faceBandMm << "mm). "
				<< "Template preview may still update; drag CAD to align with scan in 3D view or increase face band.";
			*errMsg = oss.str();
		}
	}

	if (!outReport.registrationPreviewOk)
	{
		RunLogger::info(std::string("[TemplateBrepUpdate] registration preview rejected: inlierHits=") +
						std::to_string(inlierHits) + "/512 effectiveMaxDevMm=" + std::to_string(effectiveOverlapMaxMm) +
						" (gate<=" + std::to_string(previewOverlapGateMm) +
						"mm; inlierMax=" + std::to_string(inlierMax) + " globalMax=" + std::to_string(overlapMaxAll) +
						"). Drag CAD workpiece to align with scan in 3D view until overlap is good, then retry.");
		if (errMsg && errMsg->empty())
		{
			std::ostringstream oss;
			oss << "Registration overlap too poor (inlierMaxDev=" << outReport.registrationOverlapMaxDevMm
				<< "mm, inlierHits=" << inlierHits
				<< "/512). Drag CAD workpiece to align with scan in 3D view, then retry matching.";
			*errMsg = oss.str();
		}
	}

	return true;
}

bool updateBrepFromAlignedScan(const BrepBackendData& templateBrep, const PointCloudBackendData& scanCloud,
							   geoalgo::TemplateBrepUpdateParams params, BrepBackendData& brepOut,
							   geoalgo::TemplateBrepUpdateResult& inOutReport, std::string* errMsg,
							   const std::string& templateStepPathUtf8)
{
	brepOut.clearGeometry();

	const geoalgo::ShapeHandle templateShape =
		resolveTemplateShapeForUpdate(templateBrep, templateStepPathUtf8, errMsg);
	if (templateShape.isNull())
	{
		return false;
	}
	RunLogger::info("[TemplateBrepUpdate] faceUpdate shape=originalSTEP (scan via worldMatrix → model)");

	std::vector<float> modelXyz = scanCloud.pointPositionsXyz();
	std::vector<float> modelNormals = scanCloud.pointNormalsNxNyNz();
	const std::size_t inputPointCount = modelXyz.size() / 3U;
	std::size_t workPointCount = 0U;
	if (!preparePointCloudWork(modelXyz, modelNormals, inputPointCount, workPointCount, errMsg, params.voxelPrefilterMm,
							   params.outlierRemovalPercent))
	{
		return false;
	}
	if (!scanPointsToTemplateModelFrame(scanCloud, templateBrep, modelXyz, modelNormals))
	{
		if (errMsg)
		{
			*errMsg = "scan to template model frame transform failed";
		}
		return false;
	}

	const double effectiveFaceBandMm = effectiveFaceBandMmForAssignment(params.faceBandMm, inOutReport);
	if (effectiveFaceBandMm > params.faceBandMm + 1e-6)
	{
		RunLogger::info(std::string("[TemplateBrepUpdate] faceBandMm elevated for assignment: ") +
						std::to_string(params.faceBandMm) + " -> " + std::to_string(effectiveFaceBandMm) +
						" (icpRmse=" + std::to_string(inOutReport.icpRmseMm) +
						"mm overlapMax=" + std::to_string(inOutReport.registrationOverlapMaxDevMm) + "mm)");
		params.faceBandMm = effectiveFaceBandMm;
	}

	RunLogger::info("[TemplateBrepUpdate] assigning scan points to template faces...");
	geoalgo::TemplateBrepUpdateResult faceReport = inOutReport;
	if (!geoalgo::updateShapeFromPointCloud(templateShape, modelXyz, modelNormals, params, faceReport, errMsg))
	{
		return false;
	}

	inOutReport.updatedShape = faceReport.updatedShape;
	inOutReport.perFace = std::move(faceReport.perFace);
	inOutReport.globalMaxDeviationMm = faceReport.globalMaxDeviationMm;
	inOutReport.globalAvgDeviationMm = faceReport.globalAvgDeviationMm;
	inOutReport.updatedFaceCount = faceReport.updatedFaceCount;
	inOutReport.qualityPassed = faceReport.qualityPassed;

	brepOut.setShape(inOutReport.updatedShape);
	brepOut.setFaceHighlightColors(buildRefittedFaceHighlightColors(inOutReport.perFace));
	RunLogger::info("[TemplateBrepUpdate] face update done, updatedFaces=" +
					std::to_string(inOutReport.updatedFaceCount));
	return true;
}

bool updateBrepFromCadTemplate(const BrepBackendData& templateBrep, const PointCloudBackendData& scanCloud,
							   geoalgo::TemplateBrepUpdateParams params, BrepBackendData& brepOut,
							   geoalgo::TemplateBrepUpdateResult& outReport, std::string* errMsg,
							   const std::string& templateStepPathUtf8)
{
	outReport = {};
	brepOut.clearGeometry();

	if (!registerScanToCadTemplate(templateBrep, scanCloud, params, outReport, errMsg, templateStepPathUtf8))
	{
		return false;
	}

	return updateBrepFromAlignedScan(templateBrep, scanCloud, params, brepOut, outReport, errMsg, templateStepPathUtf8);
}

namespace
{
double soupBBoxDiagonalMm(const std::vector<float>& soup)
{
	if (soup.size() < 9U)
	{
		return 0.0;
	}
	double xmin = soup[0];
	double ymin = soup[1];
	double zmin = soup[2];
	double xmax = xmin;
	double ymax = ymin;
	double zmax = zmin;
	for (std::size_t i = 0U; i + 2U < soup.size(); i += 3U)
	{
		xmin = std::min(xmin, static_cast<double>(soup[i]));
		ymin = std::min(ymin, static_cast<double>(soup[i + 1U]));
		zmin = std::min(zmin, static_cast<double>(soup[i + 2U]));
		xmax = std::max(xmax, static_cast<double>(soup[i]));
		ymax = std::max(ymax, static_cast<double>(soup[i + 1U]));
		zmax = std::max(zmax, static_cast<double>(soup[i + 2U]));
	}
	const double dx = xmax - xmin;
	const double dy = ymax - ymin;
	const double dz = zmax - zmin;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double resolveRemeshFeatureAngleDeg(const geoalgo::MeshSurfaceReconstructParams& params)
{
	if (params.remeshFeatureAngleDeg > 0.0)
	{
		return params.remeshFeatureAngleDeg;
	}
	if (params.featureThresholdC0 > 0.0)
	{
		return params.featureThresholdC0 * 180.0 / 3.141592653589793;
	}
	return 30.0;
}

double clampRemeshTargetEdgeLengthMm(const double targetLen, const std::vector<float>& soup)
{
	constexpr double kMinEdgeMm = 0.05;
	const double bboxDiag = soupBBoxDiagonalMm(soup);
	const double maxEdge = bboxDiag > 1e-6 ? bboxDiag / 50.0 : 1000.0;
	return std::max(kMinEdgeMm, std::min(targetLen, std::max(kMinEdgeMm, maxEdge)));
}

} // namespace

bool preprocessMeshSoupForSurfaceReconstruct(const std::vector<float>& soup,
											 const geoalgo::MeshSurfaceReconstructParams& params,
											 std::vector<float>& outSoup, geoalgo::MeshSurfaceReconstructReport& report,
											 std::string* errMsg)
{
	outSoup.clear();
	report.inputTriangleCount = static_cast<int>(soup.size() / 9U);
	if (soup.size() < 9U)
	{
		if (errMsg)
		{
			*errMsg = "mesh soup too small";
		}
		return false;
	}

	std::vector<float> working = soup;
#if defined(_WIN64)
	if (params.runVcgRepairFirst)
	{
		std::vector<float> repaired;
		vcgalgo::RepairParams repairParams;
		if (!vcgalgo::repairMesh(working, repaired, repairParams, nullptr, errMsg))
		{
			return false;
		}
		working = std::move(repaired);
	}
	report.repairedTriangleCount = static_cast<int>(working.size() / 9U);

	if (params.runIsotropicRemesh)
	{
		double targetLen = params.remeshTargetEdgeLengthMm;
		if (targetLen <= 0.0)
		{
			if (!vcgalgo::computeMedianEdgeLengthMm(working, targetLen, errMsg))
			{
				return false;
			}
		}
		targetLen = clampRemeshTargetEdgeLengthMm(targetLen, working);
		const double featureAngleDeg = resolveRemeshFeatureAngleDeg(params);
		std::vector<float> remeshed;
		if (!vcgalgo::isotropicRemesh(working, targetLen, remeshed, params.remeshIterations, featureAngleDeg, errMsg))
		{
			return false;
		}
		working = std::move(remeshed);
		report.remeshedTriangleCount = static_cast<int>(working.size() / 9U);
		report.remeshTargetEdgeLengthUsedMm = targetLen;
	}

	if (params.normalSmoothIterations > 0)
	{
		vcgalgo::MeshNormalSmoothParams smoothParams;
		smoothParams.iterations = params.normalSmoothIterations;
		smoothParams.featureThresholdC0 = params.featureThresholdC0;
		std::vector<float> smoothed;
		if (!vcgalgo::smoothMeshByNormalAdjustment(working, smoothed, smoothParams, &report.normalSmoothGapVolume,
												   errMsg))
		{
			return false;
		}
		working = std::move(smoothed);
	}
#else
	if (errMsg)
	{
		*errMsg = "mesh surface reconstruction requires x64";
	}
	return false;
#endif

	outSoup = std::move(working);
	if (report.repairedTriangleCount <= 0)
	{
		report.repairedTriangleCount = static_cast<int>(outSoup.size() / 9U);
	}
	return true;
}

geoalgo::MeshSurfaceReconstructSessionPtr createMeshSurfaceReconstructSession(std::vector<float> preprocessedSoup)
{
	return geoalgo::createMeshSurfaceReconstructSession(std::move(preprocessedSoup));
}

bool runMeshSurfaceReconstructStage(geoalgo::MeshSurfaceReconstructSession& session,
									const geoalgo::MeshSurfaceReconstructStage stage,
									const geoalgo::MeshSurfaceReconstructParams& params, geoalgo::ShapeHandle* outShape,
									geoalgo::MeshSurfaceReconstructReport& report, std::string* errMsg)
{
	if (!geoalgo::runMeshSurfaceReconstructStage(session, stage, params, outShape, errMsg))
	{
		return false;
	}
	report = session.report();
	return true;
}

bool buildPartitionColoredMeshSoup(const geoalgo::MeshSurfaceReconstructSession& session, std::vector<float>& outSoup,
								   std::vector<float>& outRgbPerVertex, std::string* errMsg)
{
	return geoalgo::buildPartitionColoredMeshSoup(session, outSoup, outRgbPerVertex, errMsg);
}

bool buildSamplePointsCloud(const geoalgo::MeshSurfaceReconstructSession& session, std::vector<float>& outXyz,
							std::vector<float>& outRgba, std::string* errMsg)
{
	return geoalgo::buildSamplePointsCloud(session, outXyz, outRgba, errMsg);
}

bool buildFitPreviewShape(const geoalgo::MeshSurfaceReconstructSession& session, geoalgo::ShapeHandle& outShape,
						  std::string* errMsg)
{
	return geoalgo::buildFitPreviewShape(session, outShape, errMsg);
}

bool meshSurfaceReconstructShapeToBrep(const geoalgo::ShapeHandle& shape, std::shared_ptr<BrepBackendData>& outBrep,
									   std::string* errMsg)
{
	outBrep.reset();
	if (shape.isNull())
	{
		if (errMsg)
		{
			*errMsg = "reconstruction produced null shape";
		}
		return false;
	}
	try
	{
		outBrep = std::make_shared<BrepBackendData>();
		outBrep->setShape(shape);
		return true;
	}
	catch (const std::exception& ex)
	{
		if (errMsg)
		{
			*errMsg = ex.what();
		}
		return false;
	}
	catch (...)
	{
		if (errMsg)
		{
			*errMsg = "mesh surface reconstruction internal error";
		}
		return false;
	}
}

bool reconstructBrepFromMeshSoup(const std::vector<float>& soup, const geoalgo::MeshSurfaceReconstructParams& params,
								 std::shared_ptr<BrepBackendData>& outBrep,
								 geoalgo::MeshSurfaceReconstructReport& report, std::string* errMsg)
{
	report = {};
	outBrep.reset();

	std::vector<float> working;
	if (!preprocessMeshSoupForSurfaceReconstruct(soup, params, working, report, errMsg))
	{
		return false;
	}

	try
	{
		geoalgo::ShapeHandle shape;
		if (!geoalgo::reconstructBrepFromMeshSoup(working, params, shape, report, errMsg))
		{
			return false;
		}
		return meshSurfaceReconstructShapeToBrep(shape, outBrep, errMsg);
	}
	catch (const std::exception& ex)
	{
		if (errMsg)
		{
			*errMsg = ex.what();
		}
		return false;
	}
	catch (...)
	{
		if (errMsg)
		{
			*errMsg = "mesh surface reconstruction internal error";
		}
		return false;
	}
}

geoalgo::TubularGrindingSessionPtr createTubularGrindingSession(std::vector<float> sourceSoup)
{
	return geoalgo::createTubularGrindingSession(std::move(sourceSoup));
}

geoalgo::TubularGrindingSessionPtr createTubularGrindingSessionFromPointCloud(std::vector<float> pointXyz)
{
	return geoalgo::createTubularGrindingSessionFromPointCloud(std::move(pointXyz));
}

bool runTubularGrindingStage(geoalgo::TubularGrindingSession& session, const geoalgo::TubularGrindingStage stage,
							 const geoalgo::TubularGrindingParams& params, geoalgo::TubularGrindingReport& report,
							 std::string* errMsg)
{
	if (!geoalgo::runTubularGrindingStage(session, stage, params, errMsg))
	{
		return false;
	}
	report = session.report();
	return true;
}

bool buildTubularGrindingSegmentColoredMeshSoup(const geoalgo::TubularGrindingSession& session,
												std::vector<float>& outSoup, std::vector<float>& outRgbPerVertex,
												std::string* errMsg)
{
	return geoalgo::buildSegmentColoredMeshSoup(session, outSoup, outRgbPerVertex, errMsg);
}

bool buildTubularGrindingFpfhRegionColoredMeshSoup(const geoalgo::TubularGrindingSession& session,
												   std::vector<float>& outSoup, std::vector<float>& outRgbPerVertex,
												   std::string* errMsg)
{
	return geoalgo::buildFpfhRegionColoredMeshSoup(session, outSoup, outRgbPerVertex, errMsg);
}

bool buildTubularGrindingRingColoredMeshSoup(const geoalgo::TubularGrindingSession& session,
											 std::vector<float>& outSoup, std::vector<float>& outRgbPerVertex,
											 std::string* errMsg)
{
	return geoalgo::buildRingColoredMeshSoup(session, outSoup, outRgbPerVertex, errMsg);
}

bool buildTubularGrindingRingCenterPointsCloud(const geoalgo::TubularGrindingSession& session,
											   std::vector<float>& outXyz, std::vector<float>& outRgba,
											   std::string* errMsg)
{
	return geoalgo::buildRingCenterPointsCloud(session, outXyz, outRgba, errMsg);
}

bool buildTubularGrindingFaceNormalAxisLineSegments(const geoalgo::TubularGrindingSession& session,
													const geoalgo::TubularGrindingParams& params,
													std::vector<float>& outLineXyz, std::string* errMsg)
{
	return geoalgo::buildFaceNormalAxisLineSegments(session, params, outLineXyz, errMsg);
}

bool buildTubularGrindingLocalAxisLineSegments(const geoalgo::TubularGrindingSession& session,
											   const geoalgo::TubularGrindingParams& params,
											   std::vector<float>& outLineXyz, std::string* errMsg)
{
	return geoalgo::buildLocalAxisLineSegments(session, params, outLineXyz, errMsg);
}

bool computeTubularGrindingEllipseResidualReport(const geoalgo::TubularGrindingSession& session,
												 const geoalgo::TubularGrindingParams& params,
												 std::vector<double>& outPerRingRmsResiduals,
												 std::string& outSummaryText, std::string* errMsg)
{
	return geoalgo::computeEllipseFittingResidualReport(session, params, outPerRingRmsResiduals, outSummaryText,
														errMsg);
}

bool buildTubularGrindingCenterlinePointsCloud(const geoalgo::TubularGrindingSession& session,
											   std::vector<float>& outXyz, std::vector<float>& outRgba,
											   std::string* errMsg)
{
	return geoalgo::buildCenterlinePointsCloud(session, outXyz, outRgba, errMsg);
}

bool buildTubularGrindingCenterlinePolylineXyz(const geoalgo::TubularGrindingSession& session,
											   std::vector<float>& outXyz, std::string* errMsg)
{
	return geoalgo::buildCenterlinePolylineXyz(session, outXyz, errMsg);
}

bool buildTubularGrindingCenterlinePcaAxisArrowLineSegments(const geoalgo::TubularGrindingSession& session,
															std::vector<float>& outLineXyz, std::string* errMsg)
{
	return geoalgo::buildCenterlinePcaAxisArrowLineSegments(session, outLineXyz, errMsg);
}

bool buildTubularGrindingTemplatePointsCloud(const geoalgo::TubularGrindingSession& session, std::vector<float>& outXyz,
											 std::vector<float>& outRgba, std::string* errMsg)
{
	return geoalgo::buildTemplatePointsCloud(session, outXyz, outRgba, errMsg);
}

bool buildTubularGrindingProjectedPointsCloud(const geoalgo::TubularGrindingSession& session,
											  std::vector<float>& outXyz, std::vector<float>& outRgba,
											  std::string* errMsg)
{
	return geoalgo::buildProjectedPointsCloud(session, outXyz, outRgba, errMsg);
}

int tubularGrindingIterationSnapshotCount(const geoalgo::TubularGrindingSession& session)
{
	return geoalgo::iterationSnapshotCount(session);
}

int tubularGrindingIterationSnapshotIteration(const geoalgo::TubularGrindingSession& session, int snapshotIndex)
{
	return geoalgo::iterationSnapshotIteration(session, snapshotIndex);
}

bool buildTubularGrindingIterationSnapshotPointsCloud(const geoalgo::TubularGrindingSession& session, int snapshotIndex,
													  std::vector<float>& outXyz, std::vector<float>& outRgba,
													  std::string* errMsg)
{
	return geoalgo::buildIterationSnapshotPointsCloud(session, snapshotIndex, outXyz, outRgba, errMsg);
}

bool buildTubularGrindingIterationSnapshotContractedPointsCloud(const geoalgo::TubularGrindingSession& session,
																int snapshotIndex, std::vector<float>& outXyz,
																std::vector<float>& outRgba, std::string* errMsg)
{
	return geoalgo::buildIterationSnapshotContractedPointsCloud(session, snapshotIndex, outXyz, outRgba, errMsg);
}

bool runWorldMatrixV2SelfTestImpl(std::string* errMsg)
{
	PointCloudBackendData pc;
	const BackendMat4 world = composeWorldMatrix(BackendVec3{100.0, -50.0, 250.0}, BackendVec3{15.0, -30.0, 45.0});
	pc.setWorldMatrix(world);

	const BackendVec3 stored{10.0, 20.0, 30.0};
	const BackendVec3 worldPt = transformPointToWorld(pc, stored);
	const BackendVec3 roundTrip = transformPointToStored(pc, worldPt);
	const auto near = [](double a, double b) { return std::abs(a - b) < 1e-5; };
	if (!near(roundTrip.x, stored.x) || !near(roundTrip.y, stored.y) || !near(roundTrip.z, stored.z))
	{
		if (errMsg)
		{
			*errMsg = "transformPointToWorld/Stored round-trip failed";
		}
		return false;
	}

	BackendMat4 inc = BackendMat4::translate(5.0, 0.0, 0.0);
	pc.applyWorldMatrixIncrement(inc);
	BackendMat4 expected{};
	backend_mat4_multiply(inc, world, expected);
	const BackendMat4 actual = pc.worldMatrix();
	for (std::size_t i = 0; i < 16U; ++i)
	{
		if (std::abs(actual.v[i] - expected.v[i]) > 1e-6)
		{
			if (errMsg)
			{
				*errMsg = "applyWorldMatrixIncrement mismatch";
			}
			return false;
		}
	}

	const nlohmann::json saved = pc.saveToJson();
	if (!saved.contains("worldMatrix") || !saved["worldMatrix"].is_array() || saved["worldMatrix"].size() != 16)
	{
		if (errMsg)
		{
			*errMsg = "saveToJson must emit 16-element worldMatrix";
		}
		return false;
	}

	PointCloudBackendData legacy;
	nlohmann::json oldJson = saved;
	oldJson.erase("worldMatrix");
	std::string loadErr;
	if (legacy.loadFromJson(oldJson, &loadErr))
	{
		if (errMsg)
		{
			*errMsg = "loadFromJson must reject json without worldMatrix";
		}
		return false;
	}
	if (loadErr.find("worldMatrix") == std::string::npos)
	{
		if (errMsg)
		{
			*errMsg = "loadFromJson error must mention worldMatrix";
		}
		return false;
	}
	return true;
}

bool registrationCoarsePipelineSelfTest(std::string* errMsg)
{
	if (!runRegistrationCoarsePipelineSelfTestImpl(errMsg))
	{
		return false;
	}
	return runWorldMatrixV2SelfTestImpl(errMsg);
}

} // namespace geometry_backend_ops