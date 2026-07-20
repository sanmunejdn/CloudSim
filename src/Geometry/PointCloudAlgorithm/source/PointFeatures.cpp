/// @file PointFeatures.cpp
/// @brief PointFeatures 实现

#include "PointFeatures.h"

#include "KdTreePointSet.h"
#include "PointCloudBuffer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Geometry>

namespace pclalgo
{
namespace
{
constexpr std::size_t kFpfhBinsPerFeature = 11U;

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

void findKNearest(const KdTreePointSet& tree, const std::vector<float>& xyz, const std::size_t queryIndex,
				  const unsigned int k, std::vector<std::size_t>& outIndices, std::vector<double>& outDistSq)
{
	const Eigen::Vector3d query = pointAt(xyz, queryIndex);
	tree.findKNearest(query.x(), query.y(), query.z(), k, outIndices, outDistSq);
}

std::size_t histogramBin(const double value)
{
	const double clamped = (std::max)(-1.0, (std::min)(1.0, value));
	const double scaled = (clamped + 1.0) * 0.5 * static_cast<double>(kFpfhBinsPerFeature);
	std::size_t bin = static_cast<std::size_t>(scaled);
	if (bin >= kFpfhBinsPerFeature)
	{
		bin = kFpfhBinsPerFeature - 1U;
	}
	return bin;
}

void accumulatePairFeatures(const Eigen::Vector3d& p1, const Eigen::Vector3d& n1, const Eigen::Vector3d& p2,
							const Eigen::Vector3d& n2, std::vector<float>& hist)
{
	const Eigen::Vector3d dp = p2 - p1;
	const double dist = dp.norm();
	if (dist < 1e-9)
	{
		return;
	}
	const Eigen::Vector3d dpN = dp / dist;
	const double f1 = n1.dot(dpN);
	const double f2 = dpN.dot(n2);
	const double f3 = n1.dot(n2);

	hist[histogramBin(f1)] += 100.0f;
	hist[kFpfhBinsPerFeature + histogramBin(f2)] += 100.0f;
	hist[2U * kFpfhBinsPerFeature + histogramBin(f3)] += 100.0f;
}

float featureDistanceImpl(const float* a, const float* b)
{
	float sum = 0.0f;
	for (std::size_t d = 0; d < kFpfhDim; ++d)
	{
		const float diff = a[d] - b[d];
		sum += diff * diff;
	}
	return sum;
}

} // namespace

void computeSpfhForCloud(const std::vector<float>& xyz, const std::vector<float>& normals,
						 const unsigned int kNeighbors, std::vector<float>& outSpfh)
{
	const std::size_t n = pointCountFromXyz(xyz);
	outSpfh.assign(n * kFpfhDim, 0.0f);

	const KdTreePointSet tree(xyz);

	std::vector<std::size_t> nnIdx;
	std::vector<double> nnDistSq;
	for (std::size_t i = 0; i < n; ++i)
	{
		findKNearest(tree, xyz, i, kNeighbors, nnIdx, nnDistSq);
		std::vector<float> hist(kFpfhDim, 0.0f);
		const Eigen::Vector3d pi = pointAt(xyz, i);
		const Eigen::Vector3d ni = normalAt(normals, i);
		for (const std::size_t j : nnIdx)
		{
			accumulatePairFeatures(pi, ni, pointAt(xyz, j), normalAt(normals, j), hist);
		}
		float sum = 0.0f;
		for (const float v : hist)
		{
			sum += v;
		}
		if (sum > 1e-6f)
		{
			for (float& v : hist)
			{
				v /= sum;
			}
		}
		std::copy(hist.begin(), hist.end(), outSpfh.begin() + i * kFpfhDim);
	}
}

void computeFpfhForCloud(const std::vector<float>& xyz, const std::vector<float>& normals,
						 const std::vector<float>& spfh, const unsigned int kNeighbors, std::vector<float>& outFpfh)
{
	const std::size_t n = pointCountFromXyz(xyz);
	outFpfh.assign(n * kFpfhDim, 0.0f);

	const KdTreePointSet tree(xyz);

	std::vector<std::size_t> nnIdx;
	std::vector<double> nnDistSq;
	for (std::size_t i = 0; i < n; ++i)
	{
		findKNearest(tree, xyz, i, kNeighbors, nnIdx, nnDistSq);
		std::vector<float> feat(kFpfhDim, 0.0f);
		float weightSum = 0.0f;
		for (std::size_t k = 0; k < nnIdx.size(); ++k)
		{
			const std::size_t j = nnIdx[k];
			const float w = 1.0f / static_cast<float>(std::sqrt((std::max)(nnDistSq[k], 1e-12)));
			weightSum += w;
			for (std::size_t d = 0; d < kFpfhDim; ++d)
			{
				feat[d] += w * spfh[j * kFpfhDim + d];
			}
		}
		for (std::size_t d = 0; d < kFpfhDim; ++d)
		{
			const float self = spfh[i * kFpfhDim + d];
			const float neighborPart = weightSum > 1e-6f ? (feat[d] / weightSum) : 0.0f;
			outFpfh[i * kFpfhDim + d] = self + neighborPart;
		}
		float sum = 0.0f;
		for (std::size_t d = 0; d < kFpfhDim; ++d)
		{
			sum += outFpfh[i * kFpfhDim + d];
		}
		if (sum > 1e-6f)
		{
			for (std::size_t d = 0; d < kFpfhDim; ++d)
			{
				outFpfh[i * kFpfhDim + d] /= sum;
			}
		}
	}
}

float fpfhL2Distance(const float* a, const float* b)
{
	return featureDistanceImpl(a, b);
}

} // namespace pclalgo
