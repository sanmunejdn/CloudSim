#include "RegistrationGlobal.h"

#include "Downsample.h"
#include "KdTreePointSet.h"
#include "Measure.h"
#include "PointCloudBuffer.h"
#include "PointFeatures.h"
#include "Preprocess.h"
#include "RegistrationRigid.h"
#include "Transform.h"

#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <sstream>

namespace pclalgo
{

namespace
{

constexpr std::size_t kFpfhDim = 33U;

Eigen::Vector3d pointAt(const std::vector<float>& xyz, const std::size_t i)
{
	const std::size_t b = i * 3U;
	return Eigen::Vector3d(xyz[b], xyz[b + 1U], xyz[b + 2U]);
}

Eigen::Vector3d normalAt(const std::vector<float>& nrm, const std::size_t i)
{
	const std::size_t b = i * 3U;
	Eigen::Vector3d n(nrm[b], nrm[b + 1U], nrm[b + 2U]);
	const double len = n.norm();
	if (len > 1e-12)
	{
		n /= len;
	}
	return n;
}

double boundingBoxDiagonalMm(const std::vector<float>& xyz)
{
	const Eigen::AlignedBox3d box = computeBoundingBox(xyz);
	if (box.isEmpty())
	{
		return 0.0;
	}
	return box.diagonal().norm();
}

// 原始暴力搜索版本（保留用于小点云或 KD-tree 不可用时）
void findKNearestBruteForce(
	const std::vector<float>& xyz,
	const std::size_t queryIndex,
	const unsigned int k,
	std::vector<std::size_t>& outIndices,
	std::vector<double>& outDistSq)
{
	const std::size_t n = pointCountFromXyz(xyz);
	const Eigen::Vector3d query = pointAt(xyz, queryIndex);
	const unsigned int kk = (std::min)(k, static_cast<unsigned int>(n - 1U));

	struct Neighbor
	{
		std::size_t index = 0U;
		double distSq = 0.0;
	};

	std::vector<Neighbor> neighbors;
	neighbors.reserve(n);
	for (std::size_t j = 0; j < n; ++j)
	{
		if (j == queryIndex)
		{
			continue;
		}
		const double d2 = (query - pointAt(xyz, j)).squaredNorm();
		neighbors.push_back({j, d2});
	}

	const auto cmp = [](const Neighbor& a, const Neighbor& b) { return a.distSq < b.distSq; };
	if (neighbors.size() > kk)
	{
		std::nth_element(neighbors.begin(), neighbors.begin() + kk, neighbors.end(), cmp);
		neighbors.resize(kk);
	}
	std::sort(neighbors.begin(), neighbors.end(), cmp);

	outIndices.clear();
	outDistSq.clear();
	outIndices.reserve(neighbors.size());
	outDistSq.reserve(neighbors.size());
	for (const Neighbor& nb : neighbors)
	{
		outIndices.push_back(nb.index);
		outDistSq.push_back(nb.distSq);
	}
}

void buildFeatureCorrespondences(
	const std::vector<float>& srcFpfh,
	const std::vector<float>& tgtFpfh,
	std::vector<std::size_t>& outSrcToTgt)
{
	const std::size_t nSrc = srcFpfh.size() / kFpfhDim;
	const std::size_t nTgt = tgtFpfh.size() / kFpfhDim;
	outSrcToTgt.assign(nSrc, static_cast<std::size_t>(-1));

	std::vector<std::size_t> tgtToSrc(nTgt, static_cast<std::size_t>(-1));
	for (std::size_t i = 0; i < nTgt; ++i)
	{
		float best = std::numeric_limits<float>::max();
		float second = std::numeric_limits<float>::max();
		for (std::size_t j = 0; j < nSrc; ++j)
		{
			const float d = fpfhL2Distance(
				tgtFpfh.data() + i * kFpfhDim,
				srcFpfh.data() + j * kFpfhDim);
			if (d < best)
			{
				second = best;
				best = d;
				tgtToSrc[i] = j;
			}
			else if (d < second)
			{
				second = d;
			}
		}
		(void)second;
	}

	const float ratioThreshold = 0.85f;
	for (std::size_t i = 0; i < nSrc; ++i)
	{
		float best = std::numeric_limits<float>::max();
		float second = std::numeric_limits<float>::max();
		std::size_t bestTgt = static_cast<std::size_t>(-1);
		for (std::size_t j = 0; j < nTgt; ++j)
		{
			const float d = fpfhL2Distance(
				srcFpfh.data() + i * kFpfhDim,
				tgtFpfh.data() + j * kFpfhDim);
			if (d < best)
			{
				second = best;
				best = d;
				bestTgt = j;
			}
			else if (d < second)
			{
				second = d;
			}
		}
		if (bestTgt == static_cast<std::size_t>(-1))
		{
			continue;
		}
		const bool ratioOk = second <= 1e-6f || (best / second) < ratioThreshold;
		const bool reciprocalOk = tgtToSrc[bestTgt] == i;
		if (reciprocalOk && ratioOk)
		{
			outSrcToTgt[i] = bestTgt;
		}
	}
}

bool kabsch(
	const std::vector<Eigen::Vector3d>& src,
	const std::vector<Eigen::Vector3d>& tgt,
	Eigen::Isometry3d& transform)
{
	if (src.size() != tgt.size() || src.size() < 3U)
	{
		return false;
	}

	Eigen::Vector3d srcCentroid = Eigen::Vector3d::Zero();
	Eigen::Vector3d tgtCentroid = Eigen::Vector3d::Zero();
	for (std::size_t i = 0; i < src.size(); ++i)
	{
		srcCentroid += src[i];
		tgtCentroid += tgt[i];
	}
	srcCentroid /= static_cast<double>(src.size());
	tgtCentroid /= static_cast<double>(tgt.size());

	Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
	for (std::size_t i = 0; i < src.size(); ++i)
	{
		h += (src[i] - srcCentroid) * (tgt[i] - tgtCentroid).transpose();
	}

	const Eigen::JacobiSVD<Eigen::Matrix3d> svd(h, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d r = svd.matrixV() * svd.matrixU().transpose();
	if (r.determinant() < 0.0)
	{
		Eigen::Matrix3d v = svd.matrixV();
		v.col(2) *= -1.0;
		r = v * svd.matrixU().transpose();
	}

	transform = Eigen::Isometry3d::Identity();
	transform.linear() = r;
	transform.translation() = tgtCentroid - r * srcCentroid;
	return true;
}

std::size_t evaluateInliers(
	const KdTreePointSet& tgtTree,
	const std::vector<float>& srcXyz,
	const std::vector<float>& srcNormals,
	const std::vector<float>& tgtXyz,
	const std::vector<float>& tgtNormals,
	const Eigen::Isometry3d& transform,
	const double inlierDistMm,
	const double minNormalDot,
	std::vector<std::size_t>& outInlierSrcIndices)
{
	const std::size_t nSrc = pointCountFromXyz(srcXyz);
	const double inlierDistSq = inlierDistMm * inlierDistMm;
	const Eigen::Matrix3d rot = transform.linear();

	outInlierSrcIndices.clear();
	for (std::size_t i = 0; i < nSrc; ++i)
	{
		const Eigen::Vector3d ps = transform * pointAt(srcXyz, i);
		Eigen::Vector3d ns = rot * normalAt(srcNormals, i);
		if (ns.norm() > 1e-12)
		{
			ns.normalize();
		}

		// 使用 KD-tree 加速最近邻搜索
		double distSq = 0.0;
		const std::size_t bestJ = tgtTree.findNearest(ps.x(), ps.y(), ps.z(), inlierDistSq, distSq);
		if (bestJ == static_cast<std::size_t>(-1))
		{
			continue;
		}
		const Eigen::Vector3d nt = normalAt(tgtNormals, bestJ);
		if (ns.dot(nt) < minNormalDot)
		{
			continue;
		}
		outInlierSrcIndices.push_back(i);
	}
	return outInlierSrcIndices.size();
}

// 原始暴力搜索版本（保留用于小点云或 KD-tree 不可用时）
std::size_t evaluateInliersBruteForce(
	const std::vector<float>& srcXyz,
	const std::vector<float>& srcNormals,
	const std::vector<float>& tgtXyz,
	const std::vector<float>& tgtNormals,
	const Eigen::Isometry3d& transform,
	const double inlierDistMm,
	const double minNormalDot,
	std::vector<std::size_t>& outInlierSrcIndices)
{
	const std::size_t nSrc = pointCountFromXyz(srcXyz);
	const std::size_t nTgt = pointCountFromXyz(tgtXyz);
	const double inlierDistSq = inlierDistMm * inlierDistMm;
	const Eigen::Matrix3d rot = transform.linear();

	outInlierSrcIndices.clear();
	for (std::size_t i = 0; i < nSrc; ++i)
	{
		const Eigen::Vector3d ps = transform * pointAt(srcXyz, i);
		Eigen::Vector3d ns = rot * normalAt(srcNormals, i);
		if (ns.norm() > 1e-12)
		{
			ns.normalize();
		}

		double bestSq = std::numeric_limits<double>::max();
		std::size_t bestJ = 0U;
		for (std::size_t j = 0; j < nTgt; ++j)
		{
			const double d2 = (ps - pointAt(tgtXyz, j)).squaredNorm();
			if (d2 < bestSq)
			{
				bestSq = d2;
				bestJ = j;
			}
		}
		if (bestSq > inlierDistSq)
		{
			continue;
		}
		const Eigen::Vector3d nt = normalAt(tgtNormals, bestJ);
		if (ns.dot(nt) < minNormalDot)
		{
			continue;
		}
		outInlierSrcIndices.push_back(i);
	}
	return outInlierSrcIndices.size();
}

bool prepareFeatureCloud(
	std::vector<float>& xyz,
	std::vector<float>& normalsOut,
	const double voxelMm,
	const std::size_t maxPoints,
	std::string* errMsg)
{
	if (!validXyzLength(xyz) || pointCountFromXyz(xyz) < 20U)
	{
		if (errMsg)
		{
			*errMsg = "too few points for feature cloud";
		}
		return false;
	}

	if (voxelMm > 0.0)
	{
		(void)downsampleVoxelGrid(xyz, voxelMm, 1U, nullptr);
	}

	if (pointCountFromXyz(xyz) > maxPoints)
	{
		const double frac = static_cast<double>(maxPoints) / static_cast<double>(pointCountFromXyz(xyz));
		(void)downsampleRandom(xyz, frac, nullptr);
	}

	normalsOut.clear();
	if (!estimateNormalsPca(xyz, normalsOut, 16U, errMsg))
	{
		return false;
	}
	(void)orientNormalsMst(xyz, normalsOut, 16U, nullptr, errMsg);
	return pointCountFromXyz(xyz) >= 20U;
}

void resolveAutoParams(
	const std::vector<float>& srcXyz,
	const std::vector<float>& tgtXyz,
	RigidRegisterRansacParams& params)
{
	const double srcDiag = boundingBoxDiagonalMm(srcXyz);
	const double tgtDiag = boundingBoxDiagonalMm(tgtXyz);
	const double modelDiag = params.modelDiagMm > 0.0
		? params.modelDiagMm
		: (std::max)(srcDiag, tgtDiag);

	if (params.featureVoxelMm <= 0.0)
	{
		const double avgSpacing = computeAverageSpacingMm(srcXyz, 6U);
		params.featureVoxelMm = (std::max)(modelDiag * 0.015, avgSpacing * 2.0);
		if (params.featureVoxelMm <= 0.0)
		{
			params.featureVoxelMm = (std::max)(modelDiag * 0.015, 1.0);
		}
	}

	if (params.inlierDistanceMm <= 0.0)
	{
		params.inlierDistanceMm = (std::max)({modelDiag * 0.04, params.faceBandMm * 3.0, 3.0});
	}

	if (params.modelDiagMm <= 0.0)
	{
		params.modelDiagMm = modelDiag;
	}
}

} // namespace

bool rigidRegisterFeatureRansac(
	const std::vector<float>& sourceXyz,
	const std::vector<float>& sourceNormalsNxNyNz,
	const std::vector<float>& targetXyz,
	const std::vector<float>& targetNormalsNxNyNz,
	Eigen::Isometry3d& sourceToTarget,
	double* inlierRatio,
	RigidRegisterRansacParams params,
	std::string* errMsg)
{
	sourceToTarget = Eigen::Isometry3d::Identity();
	if (inlierRatio)
	{
		*inlierRatio = 0.0;
	}
	(void)sourceNormalsNxNyNz;
	(void)targetNormalsNxNyNz;

	if (!validXyzLength(sourceXyz) || !validXyzLength(targetXyz))
	{
		if (errMsg)
		{
			*errMsg = "invalid xyz buffers";
		}
		return false;
	}

	resolveAutoParams(sourceXyz, targetXyz, params);

	std::vector<float> srcXyz = sourceXyz;
	std::vector<float> tgtXyz = targetXyz;
	std::vector<float> srcNormals;
	std::vector<float> tgtNormals;
	std::string prepErr;
	if (!prepareFeatureCloud(srcXyz, srcNormals, params.featureVoxelMm, params.maxFeaturePoints, &prepErr)
		|| !prepareFeatureCloud(tgtXyz, tgtNormals, params.featureVoxelMm, params.maxFeaturePoints, &prepErr))
	{
		if (errMsg)
		{
			*errMsg = prepErr.empty() ? "feature cloud preparation failed" : prepErr;
		}
		return false;
	}

	std::vector<float> srcSpfh;
	std::vector<float> tgtSpfh;
	std::vector<float> srcFpfh;
	std::vector<float> tgtFpfh;
	computeSpfhForCloud(srcXyz, srcNormals, params.fpfhNeighbors, srcSpfh);
	computeSpfhForCloud(tgtXyz, tgtNormals, params.fpfhNeighbors, tgtSpfh);
	computeFpfhForCloud(srcXyz, srcNormals, srcSpfh, params.fpfhNeighbors, srcFpfh);
	computeFpfhForCloud(tgtXyz, tgtNormals, tgtSpfh, params.fpfhNeighbors, tgtFpfh);

	std::vector<std::size_t> srcToTgt;
	buildFeatureCorrespondences(srcFpfh, tgtFpfh, srcToTgt);

	std::vector<std::size_t> validPairs;
	validPairs.reserve(srcToTgt.size());
	for (std::size_t i = 0; i < srcToTgt.size(); ++i)
	{
		if (srcToTgt[i] != static_cast<std::size_t>(-1))
		{
			validPairs.push_back(i);
		}
	}
	if (validPairs.size() < 3U)
	{
		if (errMsg)
		{
			std::ostringstream oss;
			oss << "too few reciprocal feature matches: " << validPairs.size();
			*errMsg = oss.str();
		}
		return false;
	}

	const double minNormalDot = std::cos(params.maxNormalAngleDeg * 3.14159265358979323846 / 180.0);
	std::mt19937 rng{std::random_device{}()};

	// 构建目标点云的 KD-tree 加速 RANSAC 内点评估
	const KdTreePointSet tgtTree(tgtXyz);

	Eigen::Isometry3d bestTransform = Eigen::Isometry3d::Identity();
	std::size_t bestInliers = 0U;
	std::vector<std::size_t> bestInlierIndices;

	for (int iter = 0; iter < params.maxIterations; ++iter)
	{
		std::array<std::size_t, 3U> sample{};
		for (int s = 0; s < 3; ++s)
		{
			std::uniform_int_distribution<std::size_t> dist(0U, validPairs.size() - 1U);
			sample[static_cast<std::size_t>(s)] = validPairs[dist(rng)];
		}
		if (sample[0] == sample[1] || sample[0] == sample[2] || sample[1] == sample[2])
		{
			continue;
		}

		std::vector<Eigen::Vector3d> srcPts;
		std::vector<Eigen::Vector3d> tgtPts;
		srcPts.reserve(3U);
		tgtPts.reserve(3U);
		for (const std::size_t si : sample)
		{
			srcPts.push_back(pointAt(srcXyz, si));
			tgtPts.push_back(pointAt(tgtXyz, srcToTgt[si]));
		}

		Eigen::Isometry3d hypothesis = Eigen::Isometry3d::Identity();
		if (!kabsch(srcPts, tgtPts, hypothesis))
		{
			continue;
		}

		std::vector<std::size_t> inlierIndices;
		// 使用 KD-tree 加速内点评估
		const std::size_t inlierCount = evaluateInliers(
			tgtTree,
			srcXyz,
			srcNormals,
			tgtXyz,
			tgtNormals,
			hypothesis,
			params.inlierDistanceMm,
			minNormalDot,
			inlierIndices);

		if (inlierCount > bestInliers)
		{
			bestInliers = inlierCount;
			bestTransform = hypothesis;
			bestInlierIndices = std::move(inlierIndices);
		}

		if (bestInliers >= params.minInliers
			&& static_cast<double>(bestInliers) / static_cast<double>(pointCountFromXyz(srcXyz)) > 0.35)
		{
			break;
		}
	}

	if (bestInliers < params.minInliers)
	{
		if (errMsg)
		{
			std::ostringstream oss;
			oss << "RANSAC inliers below threshold: " << bestInliers << " < " << params.minInliers;
			*errMsg = oss.str();
		}
		return false;
	}

	if (bestInlierIndices.size() >= 3U)
	{
		std::vector<Eigen::Vector3d> srcPts;
		std::vector<Eigen::Vector3d> tgtPts;
		srcPts.reserve(bestInlierIndices.size());
		tgtPts.reserve(bestInlierIndices.size());
		for (const std::size_t si : bestInlierIndices)
		{
			const Eigen::Vector3d ps = bestTransform * pointAt(srcXyz, si);
			// 使用 KD-tree 加速最近邻搜索
			double distSq = 0.0;
			const std::size_t bestJ = tgtTree.findNearest(
				ps.x(), ps.y(), ps.z(),
				std::numeric_limits<double>::max(),
				distSq);
			if (bestJ != static_cast<std::size_t>(-1))
			{
				srcPts.push_back(ps);
				tgtPts.push_back(pointAt(tgtXyz, bestJ));
			}
		}
		if (srcPts.size() >= 3U)
		{
			Eigen::Isometry3d refined = Eigen::Isometry3d::Identity();
			if (kabsch(srcPts, tgtPts, refined))
			{
				const Eigen::Isometry3d composed = refined * bestTransform;
				std::vector<std::size_t> refinedInliers;
				// 使用 KD-tree 加速内点评估
				const std::size_t refinedCount = evaluateInliers(
					tgtTree,
					srcXyz,
					srcNormals,
					tgtXyz,
					tgtNormals,
					composed,
					params.inlierDistanceMm,
					minNormalDot,
					refinedInliers);
				if (refinedCount >= bestInliers)
				{
					bestTransform = composed;
					bestInliers = refinedCount;
				}
			}
		}
	}

	sourceToTarget = bestTransform;

	std::vector<float> alignedSrcXyz = srcXyz;
	std::vector<float> alignedSrcNormals = srcNormals;
	transformXyzInPlace(alignedSrcXyz, sourceToTarget);
	const Eigen::Matrix3d rot = sourceToTarget.linear();
	for (std::size_t i = 0; i < pointCountFromXyz(alignedSrcNormals); ++i)
	{
		const std::size_t b = i * 3U;
		const Eigen::Vector3d n = rot * normalAt(alignedSrcNormals, i);
		alignedSrcNormals[b] = static_cast<float>(n.x());
		alignedSrcNormals[b + 1U] = static_cast<float>(n.y());
		alignedSrcNormals[b + 2U] = static_cast<float>(n.z());
	}
	if (params.refineWithIcp)
	{
		Eigen::Isometry3d icpRefineStep = Eigen::Isometry3d::Identity();
		double icpRefineRmse = 0.0;
		if (rigidRegisterPointToPlaneIcp(
				alignedSrcXyz,
				alignedSrcNormals,
				tgtXyz,
				tgtNormals,
				icpRefineStep,
				&icpRefineRmse,
				25,
				0.005,
				params.inlierDistanceMm,
				params.maxFeaturePoints,
				nullptr,
				params.maxNormalAngleDeg))
		{
			sourceToTarget = icpRefineStep * sourceToTarget;
			std::vector<std::size_t> icpInliers;
			// 使用 KD-tree 加速内点评估
			const std::size_t icpInliersCount = evaluateInliers(
				tgtTree,
				srcXyz,
				srcNormals,
				tgtXyz,
				tgtNormals,
				sourceToTarget,
				params.inlierDistanceMm,
				minNormalDot,
				icpInliers);
			if (icpInliersCount > bestInliers)
			{
				bestInliers = icpInliersCount;
			}
		}
	}

	if (inlierRatio)
	{
		*inlierRatio = static_cast<double>(bestInliers) / static_cast<double>(pointCountFromXyz(srcXyz));
	}

	const double transCapMm = std::max(params.modelDiagMm * 0.25, params.inlierDistanceMm * 2.0);
	const double transMm = sourceToTarget.translation().norm();
	if (!params.skipTranslationCap && transMm > transCapMm)
	{
		if (errMsg)
		{
			std::ostringstream oss;
			oss << "RANSAC translation exceeds cap: " << transMm << "mm > " << transCapMm << "mm";
			*errMsg = oss.str();
		}
		sourceToTarget = Eigen::Isometry3d::Identity();
		if (inlierRatio)
		{
			*inlierRatio = 0.0;
		}
		return false;
	}
	return true;
}

} // namespace pclalgo
