#include "RegistrationRigid.h"

#include "Measure.h"
#include "PointCloudBuffer.h"
#include "Transform.h"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <random>
#include <limits>

namespace pclalgo
{

namespace
{

Eigen::Vector3d pointAt(const std::vector<float>& xyz, const std::size_t i)
{
	const std::size_t b = i * 3U;
	return Eigen::Vector3d(xyz[b], xyz[b + 1U], xyz[b + 2U]);
}

std::vector<std::size_t> subsampleIndices(const std::size_t count, const std::size_t maxPoints)
{
	std::vector<std::size_t> indices(count);
	for (std::size_t i = 0; i < count; ++i)
	{
		indices[i] = i;
	}
	if (count <= maxPoints)
	{
		return indices;
	}
	std::shuffle(indices.begin(), indices.end(), std::mt19937{std::random_device{}()});
	indices.resize(maxPoints);
	return indices;
}

std::size_t nearestIndex(
	const Eigen::Vector3d& query,
	const std::vector<float>& tgtXyz,
	const std::vector<std::size_t>& tgtIndices,
	const double maxDistSq)
{
	std::size_t best = tgtIndices[0];
	double bestSq = std::numeric_limits<double>::max();
	for (const std::size_t j : tgtIndices)
	{
		const double d2 = (query - pointAt(tgtXyz, j)).squaredNorm();
		if (d2 < bestSq)
		{
			bestSq = d2;
			best = j;
		}
	}
	if (bestSq > maxDistSq)
	{
		return static_cast<std::size_t>(-1);
	}
	return best;
}

bool kabsch(
	const std::vector<Eigen::Vector3d>& src,
	const std::vector<Eigen::Vector3d>& tgt,
	Eigen::Isometry3d& transform)
{
	if (src.size() != tgt.size() || src.empty())
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

} // namespace

bool rigidRegisterIcp(
	const std::vector<float>& sourceXyz,
	const std::vector<float>& targetXyz,
	Eigen::Isometry3d& sourceToTarget,
	double* rmseMm,
	const int maxIterations,
	const double convergenceTransMm,
	double maxPairDistanceMm,
	const std::size_t icpMaxPoints,
	std::string* errMsg)
{
	if (!validXyzLength(sourceXyz) || !validXyzLength(targetXyz))
	{
		if (errMsg != nullptr)
		{
			*errMsg = "invalid xyz buffer length";
		}
		return false;
	}

	const std::size_t nSrc = pointCountFromXyz(sourceXyz);
	const std::size_t nTgt = pointCountFromXyz(targetXyz);
	if (nSrc < 3U || nTgt < 3U)
	{
		if (errMsg != nullptr)
		{
			*errMsg = "need at least 3 points in source and target";
		}
		return false;
	}

	if (maxPairDistanceMm <= 0.0)
	{
		const Eigen::AlignedBox3d box = computeBoundingBox(targetXyz);
		maxPairDistanceMm = box.diagonal().norm() * 0.05;
		if (maxPairDistanceMm <= 0.0)
		{
			maxPairDistanceMm = 1.0;
		}
	}
	const double maxPairDistSq = maxPairDistanceMm * maxPairDistanceMm;

	const std::vector<std::size_t> srcIdx = subsampleIndices(nSrc, icpMaxPoints);
	const std::vector<std::size_t> tgtIdx = subsampleIndices(nTgt, icpMaxPoints);

	sourceToTarget = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d cumulative = Eigen::Isometry3d::Identity();

	for (int iter = 0; iter < maxIterations; ++iter)
	{
		std::vector<Eigen::Vector3d> pairedSrc;
		std::vector<Eigen::Vector3d> pairedTgt;
		pairedSrc.reserve(srcIdx.size());
		pairedTgt.reserve(srcIdx.size());

		for (const std::size_t i : srcIdx)
		{
			const Eigen::Vector3d ps = cumulative * pointAt(sourceXyz, i);
			const std::size_t j = nearestIndex(ps, targetXyz, tgtIdx, maxPairDistSq);
			if (j == static_cast<std::size_t>(-1))
			{
				continue;
			}
			pairedSrc.push_back(pointAt(sourceXyz, i));
			pairedTgt.push_back(pointAt(targetXyz, j));
		}

		if (pairedSrc.size() < 3U)
		{
			if (errMsg != nullptr)
			{
				*errMsg = "too few ICP correspondences";
			}
			return false;
		}

		Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
		if (!kabsch(pairedSrc, pairedTgt, step))
		{
			if (errMsg != nullptr)
			{
				*errMsg = "kabsch failed";
			}
			return false;
		}

		const double delta = (step.translation() - Eigen::Vector3d::Zero()).norm();
		cumulative = step * cumulative;

		if (delta < convergenceTransMm)
		{
			break;
		}
	}

	sourceToTarget = cumulative;

	if (rmseMm != nullptr)
	{
		double sumSq = 0.0;
		std::size_t pairs = 0U;
		for (const std::size_t i : srcIdx)
		{
			const Eigen::Vector3d ps = sourceToTarget * pointAt(sourceXyz, i);
			const std::size_t j = nearestIndex(ps, targetXyz, tgtIdx, maxPairDistSq);
			if (j == static_cast<std::size_t>(-1))
			{
				continue;
			}
			const double d = (ps - pointAt(targetXyz, j)).norm();
			sumSq += d * d;
			++pairs;
		}
		*rmseMm = pairs > 0U ? std::sqrt(sumSq / static_cast<double>(pairs)) : 0.0;
	}

	return true;
}

} // namespace pclalgo
