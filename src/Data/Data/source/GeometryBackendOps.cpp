#include "pch.h"
#include "GeometryBackendOps.h"
#include "GeometryRef.h"
#include "BrepBackendData.h"
#include "BackendDataManager.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"

#include <FeatureSpec.h>
#include <MeshDiscretize.h>
#include <Discretize.h>
#include <Downsample.h>
#include <Preprocess.h>
#include <Measure.h>
#include <RegistrationRigid.h>
#include <RegistrationGlobal.h>
#include <TemplateBrepUpdate.h>
#include <ShapeIo.h>
#include <ShapeQuery.h>
#include "RunLogger.h"

#include <sstream>

#include <limits>
#include <unordered_map>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

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

geoalgo::ShapeHandle resolveTemplateShapeForUpdate(
	const BrepBackendData& templateBrep,
	const std::string& templateStepPathUtf8,
	std::string* errMsg)
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

geoalgo::ShapeHandle resolveTemplateShapeForIcp(
	const BrepBackendData& templateBrep,
	const std::string& templateStepPathUtf8,
	std::string* errMsg)
{
	const geoalgo::ShapeHandle& inMemory = templateBrep.shapeRef();
	if (!inMemory.isNull())
	{
		return inMemory;
	}
	return resolveTemplateShapeForUpdate(templateBrep, templateStepPathUtf8, errMsg);
}

std::size_t countPointsWithinPairDistance(
	const std::vector<float>& srcXyz,
	const std::vector<float>& tgtXyz,
	const double maxPairMm,
	const std::size_t maxSamples)
{
	if (srcXyz.size() < 9U || tgtXyz.size() < 9U || maxPairMm <= 0.0)
	{
		return 0U;
	}
	const double maxPairDistSq = maxPairMm * maxPairMm;
	const std::size_t nSrc = srcXyz.size() / 3U;
	const std::size_t nTgt = tgtXyz.size() / 3U;
	const std::size_t sampleCount = std::min(nSrc, maxSamples);
	const std::size_t srcStride = std::max<std::size_t>(1U, nSrc / sampleCount);
	const std::size_t tgtStride = std::max<std::size_t>(1U, nTgt / 512U);
	std::size_t hits = 0U;
	for (std::size_t i = 0; i < nSrc; i += srcStride)
	{
		const std::size_t b = i * 3U;
		const Eigen::Vector3d query(srcXyz[b], srcXyz[b + 1U], srcXyz[b + 2U]);
		double bestSq = maxPairDistSq;
		for (std::size_t j = 0; j < nTgt; j += tgtStride)
		{
			const std::size_t tb = j * 3U;
			const double d2 = (query - Eigen::Vector3d(tgtXyz[tb], tgtXyz[tb + 1U], tgtXyz[tb + 2U])).squaredNorm();
			if (d2 < bestSq)
			{
				bestSq = d2;
			}
		}
		if (bestSq < maxPairDistSq)
		{
			++hits;
		}
	}
	return hits;
}

Eigen::Vector3d pointAtXyz(const std::vector<float>& xyz, const std::size_t i)
{
	const std::size_t b = i * 3U;
	return Eigen::Vector3d(xyz[b], xyz[b + 1U], xyz[b + 2U]);
}

double computeMatchedRmseMm(
	const std::vector<float>& srcXyz,
	const std::vector<float>& srcNormals,
	const std::vector<float>& tgtXyz,
	const std::vector<float>& tgtNormals,
	const Eigen::Isometry3d& transform,
	const double maxPairMm,
	const double maxNormalAngleDeg)
{
	if (srcXyz.size() < 9U || tgtXyz.size() < 9U || maxPairMm <= 0.0)
	{
		return 0.0;
	}

	const double maxPairDistSq = maxPairMm * maxPairMm;
	const double minNormalDot = maxNormalAngleDeg > 0.0
		? std::cos(maxNormalAngleDeg * 3.14159265358979323846 / 180.0)
		: -1.0;
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
		const Eigen::Vector3d ps =
			transform * Eigen::Vector3d(srcXyz[b], srcXyz[b + 1U], srcXyz[b + 2U]);
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

std::unordered_map<int, BackendColor> buildRefittedFaceHighlightColors(
	const std::vector<geoalgo::FaceUpdateReport>& perFace)
{
	static const BackendColor palette[] = {
		{0.92f, 0.25f, 0.22f, 1.0f},
		{0.22f, 0.72f, 0.35f, 1.0f},
		{0.25f, 0.45f, 0.92f, 1.0f},
		{0.95f, 0.62f, 0.12f, 1.0f},
		{0.62f, 0.28f, 0.86f, 1.0f},
		{0.18f, 0.78f, 0.78f, 1.0f},
		{0.88f, 0.38f, 0.62f, 1.0f},
		{0.45f, 0.45f, 0.20f, 1.0f},
	};
	constexpr std::size_t paletteCount = sizeof(palette) / sizeof(palette[0]);

	std::unordered_map<int, BackendColor> out;
	std::size_t colorSlot = 0U;
	for (const geoalgo::FaceUpdateReport& report : perFace)
	{
		if (report.action == geoalgo::FaceUpdateAction::PlaneRefit
			|| report.action == geoalgo::FaceUpdateAction::CylinderRefit
			|| report.action == geoalgo::FaceUpdateAction::FreeformRefit)
		{
			out[report.faceIndex] = palette[colorSlot % paletteCount];
			++colorSlot;
		}
	}
	return out;
}

bool runIcpStage(
	const char* stageLabel,
	std::vector<float>& workXyz,
	std::vector<float>& workNormals,
	const std::vector<float>& templateSampleXyz,
	const std::vector<float>* templateSampleNormals,
	const geoalgo::TemplateBrepUpdateParams& params,
	const double maxPairMm,
	const int maxIterations,
	Eigen::Isometry3d& outStep,
	double& outRmseMm,
	std::string* errMsg,
	const double normalGateDegOverride = -1.0)
{
	outStep = Eigen::Isometry3d::Identity();
	outRmseMm = 0.0;
	std::string stageErr;
	const std::vector<float>* srcNormalsPtr =
		(workNormals.size() == workXyz.size() && !workNormals.empty()) ? &workNormals : nullptr;
	const std::vector<float>* tgtNormalsPtr =
		(templateSampleNormals != nullptr && templateSampleNormals->size() == templateSampleXyz.size()
			&& !templateSampleNormals->empty())
			? templateSampleNormals
			: nullptr;
	const double normalGateDeg = normalGateDegOverride >= 0.0
		? normalGateDegOverride
		: (params.normalThresholdDeg > 0.0 ? params.normalThresholdDeg : 0.0);
	if (!pclalgo::rigidRegisterIcp(
			workXyz,
			templateSampleXyz,
			outStep,
			&outRmseMm,
			maxIterations,
			0.01,
			maxPairMm,
			params.icpMaxPoints,
			&stageErr,
			srcNormalsPtr,
			tgtNormalsPtr,
			normalGateDeg))
	{
		if (errMsg)
		{
			const std::size_t pairHits =
				countPointsWithinPairDistance(workXyz, templateSampleXyz, maxPairMm, 512U);
			std::ostringstream oss;
			oss << stageLabel << ": " << (stageErr.empty() ? "ICP failed" : stageErr)
				<< " [maxPairMm=" << maxPairMm << " pairHits=" << pairHits << "/512]";
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

bool runPointToPlaneIcpStage(
	const char* stageLabel,
	std::vector<float>& workXyz,
	std::vector<float>& workNormals,
	const std::vector<float>& templateSampleXyz,
	const std::vector<float>& templateSampleNormals,
	const geoalgo::TemplateBrepUpdateParams& params,
	const double maxPairMm,
	const int maxIterations,
	Eigen::Isometry3d& outStep,
	double& outRmseMm,
	std::string* errMsg,
	const double normalGateDegOverride = -1.0)
{
	outStep = Eigen::Isometry3d::Identity();
	outRmseMm = 0.0;
	if (workNormals.empty() || templateSampleNormals.empty())
	{
		return runIcpStage(
			stageLabel,
			workXyz,
			workNormals,
			templateSampleXyz,
			nullptr,
			params,
			maxPairMm,
			maxIterations,
			outStep,
			outRmseMm,
			errMsg,
			normalGateDegOverride >= 0.0 ? normalGateDegOverride : 0.0);
	}
	std::string stageErr;
	const double normalGateDeg = normalGateDegOverride >= 0.0
		? normalGateDegOverride
		: (params.normalThresholdDeg > 0.0 ? params.normalThresholdDeg : 0.0);
	if (!pclalgo::rigidRegisterPointToPlaneIcp(
			workXyz,
			workNormals,
			templateSampleXyz,
			templateSampleNormals,
			outStep,
			&outRmseMm,
			maxIterations,
			0.005,
			maxPairMm,
			params.icpMaxPoints,
			&stageErr,
			normalGateDeg))
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

bool computeCloudPcaFrame(
	const std::vector<float>& xyz,
	Eigen::Vector3d& outCentroid,
	Eigen::Matrix3d& outAxes)
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

double meanNearestDistanceAfterTransform(
	const std::vector<float>& srcXyz,
	const std::vector<float>& tgtXyz,
	const Eigen::Isometry3d& transform,
	const std::size_t maxSamples)
{
	const std::size_t nSrc = srcXyz.size() / 3U;
	const std::size_t nTgt = tgtXyz.size() / 3U;
	if (nSrc == 0U || nTgt == 0U)
	{
		return std::numeric_limits<double>::max();
	}
	const std::size_t step = std::max<std::size_t>(1U, nSrc / std::max<std::size_t>(1U, maxSamples));
	double sum = 0.0;
	std::size_t count = 0U;
	for (std::size_t i = 0; i < nSrc; i += step)
	{
		const std::size_t b = i * 3U;
		const Eigen::Vector3d p =
			transform * Eigen::Vector3d(srcXyz[b], srcXyz[b + 1U], srcXyz[b + 2U]);
		double bestSq = std::numeric_limits<double>::max();
		const std::size_t tgtStep = std::max<std::size_t>(1U, nTgt / 4000U);
		for (std::size_t j = 0; j < nTgt; j += tgtStep)
		{
			const std::size_t tb = j * 3U;
			const double d2 = (p - Eigen::Vector3d(tgtXyz[tb], tgtXyz[tb + 1U], tgtXyz[tb + 2U])).squaredNorm();
			if (d2 < bestSq)
			{
				bestSq = d2;
			}
		}
		sum += std::sqrt(bestSq);
		++count;
	}
	return count > 0U ? (sum / static_cast<double>(count)) : std::numeric_limits<double>::max();
}

Eigen::Isometry3d alignScanPcaToTemplate(
	std::vector<float>& workXyz,
	std::vector<float>& workNormals,
	const std::vector<float>& templateSampleXyz)
{
	Eigen::Vector3d scanCentroid = Eigen::Vector3d::Zero();
	Eigen::Vector3d templateCentroid = Eigen::Vector3d::Zero();
	Eigen::Matrix3d scanAxes = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d templateAxes = Eigen::Matrix3d::Identity();
	if (!computeCloudPcaFrame(workXyz, scanCentroid, scanAxes)
		|| !computeCloudPcaFrame(templateSampleXyz, templateCentroid, templateAxes))
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

bool alignScanPreAlignedInTemplateFrame(
	std::vector<float>& workXyz,
	std::vector<float>& workNormals,
	const std::vector<float>& templateSampleXyz,
	const std::vector<float>& templateSampleNormals,
	const geoalgo::TemplateBrepUpdateParams& params,
	Eigen::Isometry3d& outScanToTemplate,
	double& outRmseMm)
{
	outScanToTemplate = Eigen::Isometry3d::Identity();
	const double fb = std::max(params.faceBandMm, 1.0);
	const double evalPairMm = std::max(fb * 2.5, 2.0);
	const double baselineRmse = computeMatchedRmseMm(
		workXyz,
		workNormals,
		templateSampleXyz,
		templateSampleNormals,
		Eigen::Isometry3d::Identity(),
		evalPairMm,
		0.0);

	std::vector<float> trialXyz = workXyz;
	std::vector<float> trialNormals = workNormals;
	Eigen::Isometry3d refineStep = Eigen::Isometry3d::Identity();
	double icpRmse = 0.0;
	const double fineMaxPairMm = std::max(fb * 2.0, 1.5);
	const bool icpRan = runPointToPlaneIcpStage(
		"fine ICP",
		trialXyz,
		trialNormals,
		templateSampleXyz,
		templateSampleNormals,
		params,
		fineMaxPairMm,
		25,
		refineStep,
		icpRmse,
		nullptr,
		0.0);

	const double trialOverlapRmse = computeMatchedRmseMm(
		trialXyz,
		trialNormals,
		templateSampleXyz,
		templateSampleNormals,
		Eigen::Isometry3d::Identity(),
		evalPairMm,
		0.0);
	const double transMm = refineStep.translation().norm();
	const double rotDeg = rotationAngleDeg(refineStep);
	const double maxTransMm = std::max(fb * 5.0, 8.0);
	const bool improved = icpRan && trialOverlapRmse > 0.0 && trialOverlapRmse + 0.02 < baselineRmse;
	const bool sane = transMm <= maxTransMm && rotDeg <= 5.0;

	if (improved && sane)
	{
		workXyz = std::move(trialXyz);
		workNormals = std::move(trialNormals);
		outScanToTemplate = refineStep;
		outRmseMm = trialOverlapRmse;
		RunLogger::info(
			std::string("[TemplateBrepUpdate] pre-aligned ICP applied: overlapRmseMm=")
			+ std::to_string(trialOverlapRmse)
			+ " (was " + std::to_string(baselineRmse)
			+ ") transMm=" + std::to_string(transMm)
			+ " rotDeg=" + std::to_string(rotDeg));
	}
	else
	{
		outRmseMm = baselineRmse > 0.0 ? baselineRmse : icpRmse;
		RunLogger::info(
			std::string("[TemplateBrepUpdate] pre-aligned ICP skipped (manual alignment kept): baselineRmseMm=")
			+ std::to_string(baselineRmse)
			+ " trialRmseMm=" + std::to_string(trialOverlapRmse)
			+ " transMm=" + std::to_string(transMm)
			+ " rotDeg=" + std::to_string(rotDeg));
	}

	if (outRmseMm > 0.0)
	{
		RunLogger::info(
			std::string("[TemplateBrepUpdate] overlapRmseMm=") + std::to_string(outRmseMm)
			+ " (pair<=" + std::to_string(evalPairMm) + "mm)");
	}
	return true;
}

Eigen::Isometry3d alignScanToTemplateBBoxOverlap(
	std::vector<float>& workXyz,
	std::vector<float>& workNormals,
	const std::vector<float>& templateSampleXyz)
{
	const Eigen::AlignedBox3d scanBox = pclalgo::computeBoundingBox(workXyz);
	const Eigen::AlignedBox3d templateBox = pclalgo::computeBoundingBox(templateSampleXyz);
	Eigen::Isometry3d overlap = Eigen::Isometry3d::Identity();
	if (scanBox.isEmpty() || templateBox.isEmpty())
	{
		return overlap;
	}
	overlap.translation() = templateBox.center() - scanBox.center();
	applyIsometryInPlace(workXyz, overlap);
	if (!workNormals.empty())
	{
		applyIsometryInPlaceNormals(workNormals, overlap);
	}
	return overlap;
}

bool alignScanToTemplateRegistration(
	std::vector<float>& workXyz,
	std::vector<float>& workNormals,
	const std::vector<float>& templateSampleXyz,
	const geoalgo::TemplateBrepUpdateParams& params,
	Eigen::Isometry3d& outScanToTemplate,
	double& outRmseMm,
	std::string* errMsg)
{
	outScanToTemplate = Eigen::Isometry3d::Identity();
	outRmseMm = 0.0;

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

	if (params.scanAlreadyInTemplateFrame)
	{
		RunLogger::info("[TemplateBrepUpdate] skip bbox centering (scan pre-aligned in 3D view)");
	}
	else
	{
		const Eigen::Isometry3d bboxStep =
			alignScanToTemplateBBoxOverlap(workXyz, workNormals, templateSampleXyz);
		outScanToTemplate = bboxStep;
		const Eigen::Isometry3d pcaStep =
			alignScanPcaToTemplate(workXyz, workNormals, templateSampleXyz);
		outScanToTemplate = pcaStep * outScanToTemplate;
	}

	std::vector<float> templateSampleNormals;
	if (!pclalgo::estimateNormalsPca(templateSampleXyz, templateSampleNormals, 16U, nullptr))
	{
		templateSampleNormals.clear();
	}
	else
	{
		std::vector<float> templateForOrient = templateSampleXyz;
		(void)pclalgo::orientNormalsMst(templateForOrient, templateSampleNormals, 16U, nullptr, nullptr);
	}

	const double evalPairMm = std::max(params.faceBandMm * 2.5, 2.0);
	const std::size_t overlapHits =
		countPointsWithinPairDistance(workXyz, templateSampleXyz, evalPairMm, 512U);
	const Eigen::AlignedBox3d scanBox = pclalgo::computeBoundingBox(workXyz);
	const Eigen::AlignedBox3d templateBox = pclalgo::computeBoundingBox(templateSampleXyz);
	const double centroidDistMm =
		(scanBox.isEmpty() || templateBox.isEmpty())
			? 0.0
			: (scanBox.center() - templateBox.center()).norm();
	RunLogger::info(
		std::string("[TemplateBrepUpdate] frameCheck pairHits=") + std::to_string(overlapHits)
		+ "/512 centroidDistMm=" + std::to_string(centroidDistMm));

	constexpr std::size_t kMinPreAlignedOverlapHits = 32U;
	const bool usePreAlignedFineOnly =
		params.scanAlreadyInTemplateFrame && overlapHits >= kMinPreAlignedOverlapHits;
	bool useRansacInRecovery = false;
	if (params.scanAlreadyInTemplateFrame && !usePreAlignedFineOnly)
	{
		RunLogger::info(
			"[TemplateBrepUpdate] recovery: overlap insufficient, running full coarse alignment");
		const Eigen::Isometry3d bboxStep =
			alignScanToTemplateBBoxOverlap(workXyz, workNormals, templateSampleXyz);
		outScanToTemplate = bboxStep * outScanToTemplate;
		const Eigen::Isometry3d pcaStep =
			alignScanPcaToTemplate(workXyz, workNormals, templateSampleXyz);
		outScanToTemplate = pcaStep * outScanToTemplate;
		useRansacInRecovery = true;
	}

	if (params.enableRansacCoarseMatch && (!params.scanAlreadyInTemplateFrame || useRansacInRecovery))
	{
		pclalgo::RigidRegisterRansacParams ransacParams;
		ransacParams.maxFeaturePoints = params.icpMaxPoints;
		ransacParams.modelDiagMm = modelDiag;
		ransacParams.faceBandMm = params.faceBandMm;
		ransacParams.maxIterations = params.ransacMaxIterations;

		Eigen::Isometry3d ransacStep = Eigen::Isometry3d::Identity();
		double inlierRatio = 0.0;
		std::string ransacErr;
		if (pclalgo::rigidRegisterFeatureRansac(
				workXyz,
				workNormals,
				templateSampleXyz,
				templateSampleNormals,
				ransacStep,
				&inlierRatio,
				ransacParams,
				&ransacErr))
		{
			applyIsometryInPlace(workXyz, ransacStep);
			applyIsometryInPlaceNormals(workNormals, ransacStep);
			outScanToTemplate = ransacStep * outScanToTemplate;
			RunLogger::info(
				std::string("[TemplateBrepUpdate] RANSAC coarse: inlierRatio=")
				+ std::to_string(inlierRatio));
		}
		else
		{
			RunLogger::info(
				std::string("[TemplateBrepUpdate] RANSAC coarse skipped/failed: ")
				+ (ransacErr.empty() ? "unknown" : ransacErr));
		}
	}
	else if (params.enableRansacCoarseMatch && params.scanAlreadyInTemplateFrame && !useRansacInRecovery)
	{
		RunLogger::info("[TemplateBrepUpdate] skip RANSAC (scan pre-aligned, overlap sufficient)");
	}

	if (usePreAlignedFineOnly)
	{
		return alignScanPreAlignedInTemplateFrame(
			workXyz,
			workNormals,
			templateSampleXyz,
			templateSampleNormals,
			params,
			outScanToTemplate,
			outRmseMm);
	}

	std::vector<double> coarseMaxPairCandidatesMm;
	if (params.icpMaxPairDistanceMm > 0.0)
	{
		coarseMaxPairCandidatesMm.push_back(std::max(params.icpMaxPairDistanceMm, modelDiag * 0.25));
	}
	else if (params.scanAlreadyInTemplateFrame)
	{
		if (useRansacInRecovery)
		{
			for (const double frac : {0.5, 0.75, 1.0, 1.25})
			{
				coarseMaxPairCandidatesMm.push_back(std::max(modelDiag * frac, 10.0));
			}
		}
		else
		{
			const double fb = std::max(params.faceBandMm, 1.0);
			for (const double mm : {fb * 1.5, fb * 2.5, fb * 4.0, fb * 6.0, std::max(fb * 10.0, 8.0)})
			{
				coarseMaxPairCandidatesMm.push_back(mm);
			}
		}
	}
	else
	{
		for (const double frac : {0.5, 0.75, 1.0, 1.25})
		{
			coarseMaxPairCandidatesMm.push_back(std::max(modelDiag * frac, 10.0));
		}
	}

	Eigen::Isometry3d coarseStep = Eigen::Isometry3d::Identity();
	double coarseRmse = 0.0;
	bool coarseOk = false;
	std::string coarseErr;
	const std::vector<float>* tplNormalsForIcp =
		templateSampleNormals.empty() ? nullptr : &templateSampleNormals;

	for (const double maxPairMm : coarseMaxPairCandidatesMm)
	{
		coarseErr.clear();
		if (runIcpStage(
				"coarse ICP",
				workXyz,
				workNormals,
				templateSampleXyz,
				tplNormalsForIcp,
				params,
				maxPairMm,
				30,
				coarseStep,
				coarseRmse,
				&coarseErr,
				0.0))
		{
			coarseOk = true;
			break;
		}
	}
	if (!coarseOk)
	{
		if (errMsg)
		{
			const Eigen::AlignedBox3d scanBox = pclalgo::computeBoundingBox(workXyz);
			const Eigen::AlignedBox3d templateBox = pclalgo::computeBoundingBox(templateSampleXyz);
			const double centroidDistMm = (scanBox.center() - templateBox.center()).norm();
			std::ostringstream oss;
			oss << coarseErr << " [scanPts=" << (workXyz.size() / 3U)
				<< " templatePts=" << (templateSampleXyz.size() / 3U)
				<< " modelDiagMm=" << modelDiag << " centroidDistMm=" << centroidDistMm << "]";
			*errMsg = oss.str();
		}
		return false;
	}
	outScanToTemplate = coarseStep * outScanToTemplate;

	double fineMaxPairMm = params.icpMaxPairDistanceMm;
	if (fineMaxPairMm <= 0.0)
	{
		if (params.scanAlreadyInTemplateFrame)
		{
			if (useRansacInRecovery)
			{
				fineMaxPairMm = std::max(modelDiag * 0.05, 2.0);
			}
			else
			{
				fineMaxPairMm = std::max(params.faceBandMm * 2.0, 1.5);
			}
		}
		else
		{
			fineMaxPairMm = std::max(modelDiag * 0.05, 2.0);
		}
	}
	if (!coarseMaxPairCandidatesMm.empty())
	{
		fineMaxPairMm = std::min(fineMaxPairMm, coarseMaxPairCandidatesMm.back());
	}

	Eigen::Isometry3d fineStep = Eigen::Isometry3d::Identity();
	double fineRmse = coarseRmse;
	if (runPointToPlaneIcpStage(
			"fine ICP",
			workXyz,
			workNormals,
			templateSampleXyz,
			templateSampleNormals,
			params,
			fineMaxPairMm,
			50,
			fineStep,
			fineRmse,
			nullptr))
	{
		outScanToTemplate = fineStep * outScanToTemplate;
		outRmseMm = fineRmse;
	}
	else
	{
		outRmseMm = coarseRmse;
	}

	const double overlapPairMm = std::max(params.faceBandMm * 2.5, 2.0);
	const double overlapRmseMm = computeMatchedRmseMm(
		workXyz,
		workNormals,
		templateSampleXyz,
		templateSampleNormals,
		Eigen::Isometry3d::Identity(),
		overlapPairMm,
		params.normalThresholdDeg);
	if (overlapRmseMm > 0.0)
	{
		RunLogger::info(
			std::string("[TemplateBrepUpdate] overlapRmseMm=") + std::to_string(overlapRmseMm)
			+ " (pair<=" + std::to_string(overlapPairMm) + "mm)");
	}

	return true;
}

bool alignScanToTemplateIcp(
	std::vector<float>& workXyz,
	std::vector<float>& workNormals,
	const std::vector<float>& templateSampleXyz,
	const geoalgo::TemplateBrepUpdateParams& params,
	Eigen::Isometry3d& outScanToTemplate,
	double& outRmseMm,
	std::string* errMsg)
{
	return alignScanToTemplateRegistration(
		workXyz, workNormals, templateSampleXyz, params, outScanToTemplate, outRmseMm, errMsg);
}

bool preparePointCloudWork(
	std::vector<float>& workXyz,
	std::vector<float>& workNormals,
	const std::size_t inputPointCount,
	std::size_t& outWorkPointCount,
	std::string* errMsg,
	const double voxelPrefilterMm,
	const double outlierRemovalPercent)
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
			oss << "too few points after prefilter (input=" << inputPointCount
				<< " work=" << outWorkPointCount << ")";
			*errMsg = oss.str();
		}
		return false;
	}

	return true;
}

} // namespace

bool discretizeStepToMesh(
	const std::string& stepPathUtf8,
	const geoalgo::MeshDiscretizeParams& params,
	std::vector<float>& soup,
	geoalgo::MeshDiscretizeReport& report,
	std::string* errMsg)
{
	return geoalgo::tessellateStepFileToMesh(stepPathUtf8, params, soup, report, errMsg);
}

bool discretizeStepFaceToMesh(
	const std::string& stepPathUtf8,
	int faceIndex,
	const geoalgo::MeshDiscretizeParams& params,
	std::vector<float>& soup,
	geoalgo::MeshDiscretizeReport& report,
	std::string* errMsg)
{
	return geoalgo::discretizeStepFaceToMesh(stepPathUtf8, faceIndex, params, soup, report, errMsg);
}

bool discretizePolylineToMesh(
	const std::vector<float>& polylineXyz,
	const geoalgo::MeshDiscretizeParams& params,
	std::vector<float>& soup,
	std::string* errMsg)
{
	geoalgo::Polyline3d poly;
	poly.xyz = polylineXyz;
	return geoalgo::discretizePolylineToMesh(poly, params, soup, errMsg);
}

bool discretizeStepEdgesToPolylines(
	const std::string& stepPathUtf8,
	const geoalgo::TessellateParams& params,
	std::vector<geoalgo::Polyline3d>& outPolylines,
	std::string* errMsg)
{
	return geoalgo::discretizeStepEdgesToPolylines(stepPathUtf8, params, outPolylines, errMsg);
}

bool intersectStepEdges(
	const std::string& stepPathUtf8,
	int edgeIndex1,
	int edgeIndex2,
	const geoalgo::IntersectionParams& params,
	geoalgo::IntersectionResult& result,
	std::string* errMsg)
{
	return geoalgo::intersectStepEdges(stepPathUtf8, edgeIndex1, edgeIndex2, params, result, errMsg);
}

bool intersectStepEdgeFace(
	const std::string& stepPathUtf8,
	int edgeIndex,
	int faceIndex,
	const geoalgo::IntersectionParams& params,
	geoalgo::IntersectionResult& result,
	std::string* errMsg)
{
	return geoalgo::intersectStepEdgeFace(stepPathUtf8, edgeIndex, faceIndex, params, result, errMsg);
}

bool intersectStepFaces(
	const std::string& stepPathUtf8,
	int faceIndex1,
	int faceIndex2,
	const geoalgo::IntersectionParams& params,
	geoalgo::IntersectionResult& result,
	std::string* errMsg)
{
	return geoalgo::intersectStepFaces(stepPathUtf8, faceIndex1, faceIndex2, params, result, errMsg);
}

bool intersectStepFiles(
	const std::string& targetStepPathUtf8,
	const std::string& toolStepPathUtf8,
	const geoalgo::IntersectionParams& params,
	geoalgo::IntersectionResult& result,
	std::string* errMsg)
{
	return geoalgo::intersectStepFiles(targetStepPathUtf8, toolStepPathUtf8, params, result, errMsg);
}

bool brepBooleanStepFilesToMesh(
	const std::string& targetStepPathUtf8,
	const std::string& toolStepPathUtf8,
	geoalgo::BrepBooleanOp op,
	const geoalgo::MeshDiscretizeParams& meshParams,
	std::vector<float>& outSoup,
	std::string* errMsg)
{
	return geoalgo::brepBooleanStepFilesToMesh(targetStepPathUtf8, toolStepPathUtf8, op, meshParams, outSoup, errMsg);
}

bool fuseStepEdgesToPolyline(
	const std::string& stepPathUtf8,
	const std::vector<int>& edgeIndices,
	const geoalgo::TessellateParams& disc,
	geoalgo::Polyline3d& out,
	std::string* errMsg)
{
	return geoalgo::fuseStepEdgesToPolyline(stepPathUtf8, edgeIndices, disc, out, errMsg);
}

bool sewStepFacesToMesh(
	const std::string& stepPathUtf8,
	const std::vector<int>& faceIndices,
	double toleranceMm,
	const geoalgo::MeshDiscretizeParams& meshParams,
	std::vector<float>& outSoup,
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

WorkpieceShapeSource resolveWorkpieceShape(
	const std::string& backendIdUtf8,
	BackendDataManager& mgr,
	const std::string& stepPathUtf8Optional,
	geoalgo::ShapeHandle& outShape,
	geoalgo::WorkpieceRef& outRef,
	std::string* errMsg)
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

bool discretizeFeature(const geoalgo::FeatureSpec& spec, geoalgo::RawPath& out, std::string* errMsg)
{
	return geoalgo::discretizeFeature(spec, out, errMsg);
}

bool discretizeFeature(
	const geoalgo::FeatureSpec& spec,
	const geoalgo::ShapeHandle& shape,
	geoalgo::RawPath& out,
	std::string* errMsg)
{
	return geoalgo::discretizeFeature(spec, shape, out, errMsg);
}

bool discretizeFeatures(
	const std::vector<geoalgo::FeatureSpec>& specs,
	std::vector<geoalgo::RawPath>& out,
	std::string* errMsg)
{
	return geoalgo::discretizeFeatures(specs, out, errMsg);
}

bool validateFeatureSpec(const geoalgo::FeatureSpec& spec, std::string* errMsg)
{
	return geoalgo::validateFeatureSpecWithShape(spec, errMsg);
}

bool enumerateFeatureCatalog(
	const geoalgo::WorkpieceRef& workpiece,
	geoalgo::FeatureCatalog& out,
	std::string* errMsg)
{
	return geoalgo::enumerateFeatureCatalog(workpiece, out, errMsg);
}

bool enumerateFeatureCatalog(
	const geoalgo::WorkpieceRef& workpiece,
	const geoalgo::ShapeHandle& shape,
	geoalgo::FeatureCatalog& out,
	std::string* errMsg)
{
	return geoalgo::enumerateFeatureCatalog(workpiece, shape, out, errMsg);
}

bool featureSpecFromJson(const std::string& jsonUtf8, geoalgo::FeatureSpec& out, std::string* errMsg)
{
	return geoalgo::featureSpecFromJson(jsonUtf8, out, errMsg);
}

std::string featureSpecToJson(const geoalgo::FeatureSpec& spec)
{
	return geoalgo::featureSpecToJson(spec);
}

std::string featureCatalogToJson(const geoalgo::FeatureCatalog& catalog)
{
	return geoalgo::featureCatalogToJson(catalog);
}

bool suggestFeaturesFromCatalog(
	const geoalgo::FeatureCatalog& catalog,
	const std::string& intentUtf8,
	std::vector<geoalgo::FeatureSpec>& out,
	std::string* errMsg)
{
	return geoalgo::suggestFeaturesFromCatalog(catalog, intentUtf8, out, errMsg);
}

bool computeFeatureAnchor(
	const geoalgo::WorkpieceRef& workpiece,
	const geoalgo::FeatureRefs& refs,
	geoalgo::FeatureAnchor& out,
	std::string* errMsg)
{
	return geoalgo::computeFeatureAnchor(workpiece, refs, out, errMsg);
}

bool buildFeatureSpecFromModelPick(
	const geoalgo::WorkpieceRef& workpiece,
	const geoalgo::ShapeHandle& shape,
	const bool pickFace,
	const geoalgo::FeatureKind faceKindForPick,
	const geoalgo::Point3d& modelPointA,
	const geoalgo::Point3d& modelPointB,
	geoalgo::FeatureSpec& out,
	std::string* errMsg,
	const int knownFaceIndex,
	const int knownEdgeIndex)
{
	out = geoalgo::FeatureSpec{};
	out.workpiece = workpiece;
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
		out.kind = faceKindForPick;
		out.refs.faceIndices = {faceIdx};
		if (faceKindForPick == geoalgo::FeatureKind::FaceUVGrid)
		{
			out.refs.uvCountU = 16;
			out.refs.uvCountV = 16;
			out.discretize.stepMm = 0.0;
		}
		else
		{
			out.discretize.stepMm = 2.0;
		}
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
		out.kind = geoalgo::FeatureKind::EdgeChain;
		out.refs.edgeIndices = {edgeIdx};
		out.discretize.stepMm = 2.0;
	}
	return geoalgo::validateFeatureSpec(out, errMsg);
}

bool buildFeatureSpecFromModelPick(
	const geoalgo::WorkpieceRef& workpiece,
	const bool pickFace,
	const geoalgo::FeatureKind faceKindForPick,
	const geoalgo::Point3d& modelPointA,
	const geoalgo::Point3d& modelPointB,
	geoalgo::FeatureSpec& out,
	std::string* errMsg,
	const int knownFaceIndex,
	const int knownEdgeIndex)
{
	if (!workpiece.stepPathUtf8.empty())
	{
		geoalgo::ShapeHandle shape;
		if (!geoalgo::readStepIntoHandle(workpiece.stepPathUtf8, shape, errMsg))
		{
			return false;
		}
		return buildFeatureSpecFromModelPick(
			workpiece, shape, pickFace, faceKindForPick, modelPointA, modelPointB, out, errMsg,
			knownFaceIndex, knownEdgeIndex);
	}
	if (errMsg)
	{
		*errMsg = "buildFeatureSpecFromModelPick requires shape or stepPath";
	}
	return false;
}

bool registerScanToCadTemplate(
	const BrepBackendData& templateBrep,
	const PointCloudBackendData& scanCloud,
	geoalgo::TemplateBrepUpdateParams params,
	geoalgo::TemplateBrepUpdateResult& outReport,
	std::vector<float>& outAlignedWorkXyz,
	std::vector<float>& outAlignedWorkNormals,
	std::string* errMsg,
	const std::string& templateStepPathUtf8)
{
	outReport = {};
	outAlignedWorkXyz.clear();
	outAlignedWorkNormals.clear();

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
	if (!preparePointCloudWork(
			workXyz,
			workNormals,
			inputPointCount,
			workPointCount,
			errMsg,
			params.voxelPrefilterMm,
			params.outlierRemovalPercent))
	{
		return false;
	}
	RunLogger::info("[TemplateBrepUpdate] preprocess done, workPts=" + std::to_string(workPointCount));

	std::vector<float> templateSampleXyz;
	if (!geoalgo::sampleShapeSurfacePoints(
			templateShapeForIcp, params.sampleSpacingMm, templateSampleXyz, errMsg))
	{
		return false;
	}
	subsampleXyzUniform(templateSampleXyz, 40000U);
	RunLogger::info(
		"[TemplateBrepUpdate] template samples=" + std::to_string(templateSampleXyz.size() / 3U));

	if (!alignScanToTemplateRegistration(
			workXyz, workNormals, templateSampleXyz, params, outReport.scanToTemplate, outReport.icpRmseMm, errMsg))
	{
		return false;
	}
	RunLogger::info(
		"[TemplateBrepUpdate] registration done, icpRmseMm=" + std::to_string(outReport.icpRmseMm));

	outAlignedWorkXyz = std::move(workXyz);
	outAlignedWorkNormals = std::move(workNormals);

	const double icpGateMm = std::max(
		params.faceBandMm * params.maxIcpRmseToFaceBandRatio,
		params.minIcpRmseGateMm);
	if (params.maxIcpRmseToFaceBandRatio > 0.0 && outReport.icpRmseMm > icpGateMm)
	{
		if (errMsg)
		{
			std::ostringstream oss;
			oss << "ICP alignment insufficient: rmse=" << outReport.icpRmseMm << "mm, need <= " << icpGateMm
				<< "mm (faceBand=" << params.faceBandMm << "mm). "
				<< "RMSE is averaged over all scan points; partial scans often need a larger face band. "
				<< "ICP transform will still be applied to the point cloud for preview.";
			*errMsg = oss.str();
		}
		return false;
	}

	return true;
}

bool updateBrepFromAlignedScan(
	const BrepBackendData& templateBrep,
	const std::vector<float>& alignedWorkXyz,
	const std::vector<float>& alignedWorkNormals,
	geoalgo::TemplateBrepUpdateParams params,
	BrepBackendData& brepOut,
	geoalgo::TemplateBrepUpdateResult& inOutReport,
	std::string* errMsg,
	const std::string& templateStepPathUtf8)
{
	brepOut.clearGeometry();

	const geoalgo::ShapeHandle templateShape =
		resolveTemplateShapeForUpdate(templateBrep, templateStepPathUtf8, errMsg);
	if (templateShape.isNull())
	{
		return false;
	}

	RunLogger::info("[TemplateBrepUpdate] assigning scan points to template faces...");
	geoalgo::TemplateBrepUpdateResult faceReport = inOutReport;
	if (!geoalgo::updateShapeFromPointCloud(
			templateShape, alignedWorkXyz, alignedWorkNormals, params, faceReport, errMsg))
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
	RunLogger::info(
		"[TemplateBrepUpdate] face update done, updatedFaces=" + std::to_string(inOutReport.updatedFaceCount));
	return true;
}

bool updateBrepFromCadTemplate(
	const BrepBackendData& templateBrep,
	const PointCloudBackendData& scanCloud,
	geoalgo::TemplateBrepUpdateParams params,
	BrepBackendData& brepOut,
	geoalgo::TemplateBrepUpdateResult& outReport,
	std::string* errMsg,
	const std::string& templateStepPathUtf8)
{
	outReport = {};
	brepOut.clearGeometry();

	std::vector<float> alignedWorkXyz;
	std::vector<float> alignedWorkNormals;
	if (!registerScanToCadTemplate(
			templateBrep,
			scanCloud,
			params,
			outReport,
			alignedWorkXyz,
			alignedWorkNormals,
			errMsg,
			templateStepPathUtf8))
	{
		return false;
	}

	return updateBrepFromAlignedScan(
		templateBrep,
		alignedWorkXyz,
		alignedWorkNormals,
		params,
		brepOut,
		outReport,
		errMsg,
		templateStepPathUtf8);
}

} // namespace geometry_backend_ops