#include "MeshFpfhRegionPartition.h"

#include "KdTreePointSet.h"
#include "PointFeatures.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <queue>
#include <set>
#include <vector>

namespace geoalgo
{
namespace tg
{

namespace
{

double bboxDiagonal(const IndexedMeshLite& mesh)
{
	const double dx = mesh.bboxMax[0] - mesh.bboxMin[0];
	const double dy = mesh.bboxMax[1] - mesh.bboxMin[1];
	const double dz = mesh.bboxMax[2] - mesh.bboxMin[2];
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double bboxDiagonalForXyz(const std::vector<float>& xyz)
{
	if (xyz.size() < 3U)
	{
		return 1.0;
	}
	double minx = xyz[0], maxx = xyz[0];
	double miny = xyz[1], maxy = xyz[1];
	double minz = xyz[2], maxz = xyz[2];
	for (std::size_t i = 3; i < xyz.size(); i += 3)
	{
		minx = (std::min)(minx, static_cast<double>(xyz[i]));
		maxx = (std::max)(maxx, static_cast<double>(xyz[i]));
		miny = (std::min)(miny, static_cast<double>(xyz[i + 1U]));
		maxy = (std::max)(maxy, static_cast<double>(xyz[i + 1U]));
		minz = (std::min)(minz, static_cast<double>(xyz[i + 2U]));
		maxz = (std::max)(maxz, static_cast<double>(xyz[i + 2U]));
	}
	return std::sqrt((maxx - minx) * (maxx - minx) + (maxy - miny) * (maxy - miny) + (maxz - minz) * (maxz - minz));
}

std::vector<float> buildSampleXyz(const IndexedMeshLite& mesh)
{
	std::vector<float> xyz;
	xyz.reserve(static_cast<std::size_t>(mesh.faceCount) * 3U);
	for (int f = 0; f < mesh.faceCount; ++f)
	{
		const Vec3& c = mesh.faceCentroids[static_cast<std::size_t>(f)];
		xyz.push_back(static_cast<float>(c.x));
		xyz.push_back(static_cast<float>(c.y));
		xyz.push_back(static_cast<float>(c.z));
	}
	return xyz;
}

std::vector<float> buildSampleNormals(const IndexedMeshLite& mesh)
{
	std::vector<float> nrm;
	nrm.reserve(static_cast<std::size_t>(mesh.faceCount) * 3U);
	for (int f = 0; f < mesh.faceCount; ++f)
	{
		const Vec3& n = mesh.faceNormals[static_cast<std::size_t>(f)];
		nrm.push_back(static_cast<float>(n.x));
		nrm.push_back(static_cast<float>(n.y));
		nrm.push_back(static_cast<float>(n.z));
	}
	return nrm;
}

std::vector<std::size_t> voxelDownsampleIndices(const std::vector<float>& xyz, double voxelMm, int maxPoints)
{
	const std::size_t n = xyz.size() / 3U;
	if (n == 0U)
	{
		return {};
	}
	if (voxelMm <= 0.0)
	{
		voxelMm = bboxDiagonalForXyz(xyz) * 0.02;
	}
	struct Key
	{
		int x, y, z;
		bool operator<(const Key& o) const
		{
			if (x != o.x) return x < o.x;
			if (y != o.y) return y < o.y;
			return z < o.z;
		}
	};
	std::map<Key, std::size_t> buckets;
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		Key k{
			static_cast<int>(std::floor(xyz[b] / voxelMm)),
			static_cast<int>(std::floor(xyz[b + 1U] / voxelMm)),
			static_cast<int>(std::floor(xyz[b + 2U] / voxelMm))
		};
		if (buckets.find(k) == buckets.end())
		{
			buckets[k] = i;
		}
	}
	std::vector<std::size_t> idx;
	idx.reserve(buckets.size());
	for (const auto& kv : buckets)
	{
		idx.push_back(kv.second);
	}
	if (maxPoints > 0 && static_cast<int>(idx.size()) > maxPoints)
	{
		idx.resize(static_cast<std::size_t>(maxPoints));
	}
	return idx;
}

std::vector<std::size_t> selectKeypoints(
	const std::vector<float>& fpfh,
	const std::vector<std::size_t>& faceSampleIndices,
	int desiredCount,
	double minSepMm,
	const std::vector<float>& sampleXyz,
	unsigned int saliencyNeighbors)
{
	const std::size_t m = faceSampleIndices.size();
	if (m == 0U)
	{
		return {};
	}
	const unsigned int salK = (std::max)(2U, saliencyNeighbors);
	std::vector<double> saliency(m, 0.0);
	for (std::size_t i = 0; i < m; ++i)
	{
		std::vector<double> dists;
		dists.reserve(m);
		for (std::size_t j = 0; j < m; ++j)
		{
			if (i == j)
			{
				continue;
			}
			dists.push_back(static_cast<double>(pclalgo::fpfhL2Distance(
				fpfh.data() + i * pclalgo::kFpfhDim,
				fpfh.data() + j * pclalgo::kFpfhDim)));
		}
		if (dists.empty())
		{
			continue;
		}
		const std::size_t kk = (std::min)(static_cast<std::size_t>(salK), dists.size());
		std::nth_element(dists.begin(), dists.begin() + static_cast<std::ptrdiff_t>(kk - 1U), dists.end());
		double sum = 0.0;
		for (std::size_t k = 0; k < kk; ++k)
		{
			sum += dists[k];
		}
		saliency[i] = sum / static_cast<double>(kk);
	}
	std::vector<std::size_t> order(m);
	for (std::size_t i = 0; i < m; ++i)
	{
		order[i] = i;
	}
	std::sort(order.begin(), order.end(), [&](const std::size_t a, const std::size_t b) {
		return saliency[a] > saliency[b];
	});
	std::vector<std::size_t> seeds;
	for (const std::size_t oi : order)
	{
		const std::size_t fi = faceSampleIndices[oi];
		bool ok = true;
		for (const std::size_t s : seeds)
		{
			const double dx = sampleXyz[fi * 3U] - sampleXyz[s * 3U];
			const double dy = sampleXyz[fi * 3U + 1U] - sampleXyz[s * 3U + 1U];
			const double dz = sampleXyz[fi * 3U + 2U] - sampleXyz[s * 3U + 2U];
			if (std::sqrt(dx * dx + dy * dy + dz * dz) < minSepMm)
			{
				ok = false;
				break;
			}
		}
		if (ok)
		{
			seeds.push_back(fi);
			if (desiredCount > 0 && static_cast<int>(seeds.size()) >= desiredCount)
			{
				break;
			}
		}
	}
	return seeds;
}

MeshFpfhPartitionParams resolveParams(const MeshFpfhPartitionParams& params)
{
	return params;
}

} // namespace

bool runMeshFpfhRegionPartition(
	const IndexedMeshLite& mesh,
	const MeshFpfhPartitionParams& paramsIn,
	std::vector<int>& outFaceRegionId,
	int& outRegionCount,
	int& outKeypointCount,
	std::string* errMsg)
{
	const MeshFpfhPartitionParams params = resolveParams(paramsIn);
	outFaceRegionId.assign(static_cast<std::size_t>(mesh.faceCount), -1);
	outRegionCount = 0;
	outKeypointCount = 0;
	if (mesh.faceCount < 3)
	{
		if (errMsg)
		{
			*errMsg = "mesh too small";
		}
		return false;
	}

	IndexedMeshLite work = mesh;
	if (work.faceCentroids.empty() || work.faceNeighbors.empty())
	{
		if (!buildIndexedMeshLite(work.soup, work, errMsg))
		{
			return false;
		}
	}
	orientMeshFaceNormals(work);

	const std::vector<float> sampleXyz = buildSampleXyz(work);
	const std::vector<float> sampleNrm = buildSampleNormals(work);
	const int maxPts = params.maxSamplePoints > 0 ? params.maxSamplePoints : 4000;
	const double voxel =
		params.featureVoxelMm > 0.0 ? params.featureVoxelMm : bboxDiagonal(work) * 0.02;
	const std::vector<std::size_t> sampleIdx = voxelDownsampleIndices(sampleXyz, voxel, maxPts);
	if (sampleIdx.empty())
	{
		if (errMsg)
		{
			*errMsg = "no samples";
		}
		return false;
	}

	std::vector<float> subXyz;
	std::vector<float> subNrm;
	subXyz.reserve(sampleIdx.size() * 3U);
	subNrm.reserve(sampleIdx.size() * 3U);
	for (const std::size_t fi : sampleIdx)
	{
		subXyz.push_back(sampleXyz[fi * 3U]);
		subXyz.push_back(sampleXyz[fi * 3U + 1U]);
		subXyz.push_back(sampleXyz[fi * 3U + 2U]);
		subNrm.push_back(sampleNrm[fi * 3U]);
		subNrm.push_back(sampleNrm[fi * 3U + 1U]);
		subNrm.push_back(sampleNrm[fi * 3U + 2U]);
	}

	std::vector<float> spfh;
	std::vector<float> fpfh;
	pclalgo::computeSpfhForCloud(subXyz, subNrm, params.fpfhNeighbors, spfh);
	pclalgo::computeFpfhForCloud(subXyz, subNrm, spfh, params.fpfhNeighbors, fpfh);

	const int desiredKp = params.keypointCount > 0
		? params.keypointCount
		: std::clamp(
			static_cast<int>(std::sqrt(static_cast<double>(sampleIdx.size())) * 2.0),
			8,
			64);
	const double minSep =
		params.keypointMinSeparationMm > 0.0 ? params.keypointMinSeparationMm : voxel * 2.0;
	const std::vector<std::size_t> keySeeds =
		selectKeypoints(fpfh, sampleIdx, desiredKp, minSep, sampleXyz, params.saliencyNeighbors);
	outKeypointCount = static_cast<int>(keySeeds.size());
	if (keySeeds.empty())
	{
		if (errMsg)
		{
			*errMsg = "no keypoints";
		}
		return false;
	}

	std::map<std::size_t, std::size_t> faceToSubIdx;
	for (std::size_t si = 0; si < sampleIdx.size(); ++si)
	{
		faceToSubIdx[sampleIdx[si]] = si;
	}

	std::vector<float> kpXyz;
	kpXyz.reserve(keySeeds.size() * 3U);
	for (const std::size_t fi : keySeeds)
	{
		kpXyz.push_back(sampleXyz[fi * 3U]);
		kpXyz.push_back(sampleXyz[fi * 3U + 1U]);
		kpXyz.push_back(sampleXyz[fi * 3U + 2U]);
	}
	const pclalgo::KdTreePointSet kpTree(kpXyz);

	std::vector<int> faceLabel(static_cast<std::size_t>(work.faceCount), -1);
	for (std::size_t ki = 0; ki < keySeeds.size(); ++ki)
	{
		const std::size_t seedFace = keySeeds[ki];
		const auto subIt = faceToSubIdx.find(seedFace);
		if (subIt == faceToSubIdx.end())
		{
			continue;
		}
		const std::size_t seedSub = subIt->second;
		const float* seedFpfh = fpfh.data() + seedSub * pclalgo::kFpfhDim;

		faceLabel[seedFace] = static_cast<int>(ki);
		std::queue<int> q;
		q.push(static_cast<int>(seedFace));
		std::vector<uint8_t> visited(static_cast<std::size_t>(work.faceCount), 0U);
		visited[seedFace] = 1U;

		const double growDist = params.regionGrowDist > 0.0 ? params.regionGrowDist : 0.35;
		const double cosAngle =
			std::cos(params.regionGrowNormalAngleDeg * 3.14159265358979323846 / 180.0);

		while (!q.empty())
		{
			const int cur = q.front();
			q.pop();
			const Vec3& n1 = work.faceNormals[static_cast<std::size_t>(cur)];
			for (const int nb : work.faceNeighbors[static_cast<std::size_t>(cur)])
			{
				if (nb < 0 || visited[static_cast<std::size_t>(nb)] != 0U)
				{
					continue;
				}
				const Vec3& n2 = work.faceNormals[static_cast<std::size_t>(nb)];
				const double dotn = n1.x * n2.x + n1.y * n2.y + n1.z * n2.z;
				if (dotn < cosAngle)
				{
					continue;
				}
				const auto nbSubIt = faceToSubIdx.find(static_cast<std::size_t>(nb));
				if (nbSubIt == faceToSubIdx.end())
				{
					continue;
				}
				const float d = pclalgo::fpfhL2Distance(
					seedFpfh,
					fpfh.data() + nbSubIt->second * pclalgo::kFpfhDim);
				if (d >= growDist)
				{
					continue;
				}
				faceLabel[static_cast<std::size_t>(nb)] = static_cast<int>(ki);
				visited[static_cast<std::size_t>(nb)] = 1U;
				q.push(nb);
			}
		}
	}

	// 未生长到的面：按最近关键点 seed 分配
	for (int f = 0; f < work.faceCount; ++f)
	{
		if (faceLabel[static_cast<std::size_t>(f)] >= 0)
		{
			continue;
		}
		std::vector<std::size_t> nn;
		std::vector<double> dsq;
		kpTree.findKNearest(
			sampleXyz[static_cast<std::size_t>(f) * 3U],
			sampleXyz[static_cast<std::size_t>(f) * 3U + 1U],
			sampleXyz[static_cast<std::size_t>(f) * 3U + 2U],
			1U,
			nn,
			dsq);
		if (!nn.empty())
		{
			faceLabel[static_cast<std::size_t>(f)] = static_cast<int>(nn[0]);
		}
	}

	std::map<int, std::vector<int>> regionFaces;
	for (int f = 0; f < work.faceCount; ++f)
	{
		const int lbl = faceLabel[static_cast<std::size_t>(f)];
		if (lbl >= 0)
		{
			regionFaces[lbl].push_back(f);
		}
	}

	for (const auto& kv : regionFaces)
	{
		if (static_cast<int>(kv.second.size()) < params.minRegionFaces)
		{
			for (const int f : kv.second)
			{
				faceLabel[static_cast<std::size_t>(f)] = -1;
			}
		}
	}

	regionFaces.clear();
	for (int f = 0; f < work.faceCount; ++f)
	{
		const int lbl = faceLabel[static_cast<std::size_t>(f)];
		if (lbl >= 0)
		{
			regionFaces[lbl].push_back(f);
		}
	}

	std::map<int, int> labelRemap;
	int nextId = 0;
	for (const auto& kv : regionFaces)
	{
		labelRemap[kv.first] = nextId++;
	}

	outRegionCount = nextId;
	outFaceRegionId.assign(static_cast<std::size_t>(work.faceCount), -1);
	for (int f = 0; f < work.faceCount; ++f)
	{
		const int lbl = faceLabel[static_cast<std::size_t>(f)];
		if (lbl >= 0)
		{
			const auto it = labelRemap.find(lbl);
			if (it != labelRemap.end())
			{
				outFaceRegionId[static_cast<std::size_t>(f)] = it->second;
			}
		}
	}
	return outRegionCount > 0;
}

} // namespace tg
} // namespace geoalgo
