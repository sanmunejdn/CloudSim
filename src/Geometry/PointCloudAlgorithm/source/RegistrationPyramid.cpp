/// @file RegistrationPyramid.cpp
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com

#include "RegistrationPyramid.h"

#include "KdTreePointSet.h"

#include <MeshRemesh.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace pclalgo
{
namespace
{

constexpr double kVertexWeldEpsMm = 1e-4;
constexpr int kDefaultLayers = 3;
constexpr int kCoarseMaxOuter = 15;
constexpr std::size_t kCoarseAlignSample = 2000U;

struct WeldedSoup
{
	std::vector<float> xyz;				 ///< 3*V
	std::vector<std::size_t> cornerToVert; ///< 每角点 → 顶点
};

std::uint64_t quantizeKey(float x, float y, float z, double eps)
{
	const auto qx = static_cast<std::int64_t>(std::llround(static_cast<double>(x) / eps));
	const auto qy = static_cast<std::int64_t>(std::llround(static_cast<double>(y) / eps));
	const auto qz = static_cast<std::int64_t>(std::llround(static_cast<double>(z) / eps));
	const auto ux = static_cast<std::uint64_t>(qx) & 0x1FFFFFULL;
	const auto uy = static_cast<std::uint64_t>(qy) & 0x1FFFFFULL;
	const auto uz = static_cast<std::uint64_t>(qz) & 0x1FFFFFULL;
	return (ux << 42) | (uy << 21) | uz;
}

bool weldSoup(const std::vector<float>& soup, WeldedSoup& out, std::string* errMsg)
{
	if (soup.size() < 9U || (soup.size() % 9U) != 0U)
	{
		if (errMsg)
		{
			*errMsg = "Pyramid: invalid triangle soup";
		}
		return false;
	}
	out.xyz.clear();
	out.cornerToVert.clear();
	out.cornerToVert.reserve(soup.size() / 3U);
	std::unordered_map<std::uint64_t, std::size_t> map;
	map.reserve(soup.size() / 9U);
	for (std::size_t i = 0; i + 2U < soup.size(); i += 3U)
	{
		const float x = soup[i];
		const float y = soup[i + 1U];
		const float z = soup[i + 2U];
		const std::uint64_t key = quantizeKey(x, y, z, kVertexWeldEpsMm);
		const auto it = map.find(key);
		if (it != map.end())
		{
			out.cornerToVert.push_back(it->second);
			continue;
		}
		const std::size_t vid = out.xyz.size() / 3U;
		map.emplace(key, vid);
		out.xyz.push_back(x);
		out.xyz.push_back(y);
		out.xyz.push_back(z);
		out.cornerToVert.push_back(vid);
	}
	return !out.xyz.empty();
}

bool writeSoupFromVerts(const WeldedSoup& welded, const std::vector<float>& deformedXyz, std::vector<float>& soupOut,
						std::string* errMsg)
{
	if (deformedXyz.size() != welded.xyz.size() || welded.cornerToVert.empty())
	{
		if (errMsg)
		{
			*errMsg = "Pyramid: deformed vertex count mismatch";
		}
		return false;
	}
	soupOut.resize(welded.cornerToVert.size() * 3U);
	for (std::size_t c = 0; c < welded.cornerToVert.size(); ++c)
	{
		const std::size_t v = welded.cornerToVert[c];
		const std::size_t b = v * 3U;
		const std::size_t ob = c * 3U;
		soupOut[ob] = deformedXyz[b];
		soupOut[ob + 1U] = deformedXyz[b + 1U];
		soupOut[ob + 2U] = deformedXyz[b + 2U];
	}
	return true;
}

/// 用 rest 焊点索引聚合 deformed 角点，保证 rest/def 顶点一一对应
bool deformedVertsFromRestWeld(const WeldedSoup& restWeld, const std::vector<float>& defSoup,
							   std::vector<float>& defXyzOut, std::string* errMsg)
{
	if (defSoup.size() != restWeld.cornerToVert.size() * 3U)
	{
		if (errMsg)
		{
			*errMsg = "Pyramid: coarse rest/def soup size mismatch";
		}
		return false;
	}
	const std::size_t nVert = restWeld.xyz.size() / 3U;
	defXyzOut.assign(restWeld.xyz.size(), 0.0f);
	std::vector<int> counts(nVert, 0);
	for (std::size_t c = 0; c < restWeld.cornerToVert.size(); ++c)
	{
		const std::size_t v = restWeld.cornerToVert[c];
		const std::size_t cb = c * 3U;
		const std::size_t vb = v * 3U;
		defXyzOut[vb] += defSoup[cb];
		defXyzOut[vb + 1U] += defSoup[cb + 1U];
		defXyzOut[vb + 2U] += defSoup[cb + 2U];
		++counts[v];
	}
	for (std::size_t v = 0; v < nVert; ++v)
	{
		if (counts[v] <= 0)
		{
			continue;
		}
		const float inv = 1.0f / static_cast<float>(counts[v]);
		const std::size_t vb = v * 3U;
		defXyzOut[vb] *= inv;
		defXyzOut[vb + 1U] *= inv;
		defXyzOut[vb + 2U] *= inv;
	}
	return true;
}

/// 粗层 rest→def 位移经 NN 抬到细层 rest，作 warm-start
bool prolongateByNearestNeighbor(const std::vector<float>& coarseRestSoup, const std::vector<float>& coarseDefSoup,
								 const std::vector<float>& fineRestSoup, std::vector<float>& fineWarmSoupOut,
								 std::string* errMsg)
{
	WeldedSoup coarseRest;
	WeldedSoup fineRest;
	if (!weldSoup(coarseRestSoup, coarseRest, errMsg) || !weldSoup(fineRestSoup, fineRest, errMsg))
	{
		return false;
	}
	std::vector<float> coarseDefXyz;
	if (!deformedVertsFromRestWeld(coarseRest, coarseDefSoup, coarseDefXyz, errMsg))
	{
		return false;
	}

	KdTreePointSet tree(coarseRest.xyz);
	std::vector<float> fineDefXyz = fineRest.xyz;
	for (std::size_t v = 0; v * 3U + 2U < fineRest.xyz.size(); ++v)
	{
		const std::size_t b = v * 3U;
		double distSq = 0.0;
		const std::size_t nn =
			tree.findNearest(fineRest.xyz[b], fineRest.xyz[b + 1U], fineRest.xyz[b + 2U],
							 std::numeric_limits<double>::max(), distSq);
		if (nn == static_cast<std::size_t>(-1))
		{
			continue;
		}
		const std::size_t cb = nn * 3U;
		fineDefXyz[b] = fineRest.xyz[b] + (coarseDefXyz[cb] - coarseRest.xyz[cb]);
		fineDefXyz[b + 1U] = fineRest.xyz[b + 1U] + (coarseDefXyz[cb + 1U] - coarseRest.xyz[cb + 1U]);
		fineDefXyz[b + 2U] = fineRest.xyz[b + 2U] + (coarseDefXyz[cb + 2U] - coarseRest.xyz[cb + 2U]);
	}
	return writeSoupFromVerts(fineRest, fineDefXyz, fineWarmSoupOut, errMsg);
}

double layerEdgeLengthMm(double baseH, double layerScale, int layers, int level)
{
	double edge = baseH;
	for (int i = 0; i < (layers - 1 - level); ++i)
	{
		edge *= layerScale;
	}
	return edge;
}

} // namespace

bool pyramidRegisterMeshSoupToMeshSoup(const std::vector<float>& sourceSoup, const std::vector<float>& targetSoup,
									   std::vector<float>& sourceSoupDeformedOut, const PyramidRegisterParams& params,
									   PyramidRegisterResult* stats, std::string* errMsg)
{
	sourceSoupDeformedOut.clear();
	if (stats)
	{
		*stats = PyramidRegisterResult{};
	}
	if (sourceSoup.size() < 9U || targetSoup.size() < 9U || (sourceSoup.size() % 9U) != 0U ||
		(targetSoup.size() % 9U) != 0U)
	{
		if (errMsg)
		{
			*errMsg = "Pyramid: source/target soup invalid";
		}
		return false;
	}

	const int layers = params.layers > 0 ? params.layers : kDefaultLayers;
	if (layers < 1 || layers > 8)
	{
		if (errMsg)
		{
			*errMsg = "Pyramid: layers out of range";
		}
		return false;
	}
	const double scale = params.layerScale > 1.0 ? params.layerScale : 2.0;

	double baseH = params.baseEdgeLengthMm;
	if (!(baseH > 0.0))
	{
		if (!vcgalgo::computeMedianEdgeLengthMm(sourceSoup, baseH, errMsg) || !(baseH > 0.0))
		{
			if (errMsg && errMsg->empty())
			{
				*errMsg = "Pyramid: failed to estimate base edge length";
			}
			return false;
		}
	}

	std::ostringstream dbg;
	dbg << "[Pyramid] h=" << baseH << " mm layers=" << layers << " solver="
		<< (params.solver == PyramidSolver::Spare ? "SPARE" : "SDF") << " mode=prolongate\n";

	std::vector<float> prevRest;
	std::vector<float> prevDef;
	std::vector<float> lastDef;
	double lastMeanErr = 0.0;
	int lastNodes = 0;

	for (int level = 0; level < layers; ++level)
	{
		const double edge = layerEdgeLengthMm(baseH, scale, layers, level);
		dbg << "[Pyramid] L" << level << " edge=" << edge << " mm\n";

		// 每层从原始几何 remesh；层间只传位移场，不对变形结果整网 remesh
		std::vector<float> srcLevel;
		std::vector<float> tgtLevel;
		if (!vcgalgo::isotropicRemesh(sourceSoup, edge, srcLevel, params.remeshIterations, 30.0, errMsg))
		{
			if (errMsg && errMsg->empty())
			{
				*errMsg = "Pyramid: source remesh failed";
			}
			return false;
		}
		if (!vcgalgo::isotropicRemesh(targetSoup, edge, tgtLevel, params.remeshIterations, 30.0, errMsg))
		{
			if (errMsg && errMsg->empty())
			{
				*errMsg = "Pyramid: target remesh failed";
			}
			return false;
		}

		std::vector<float> srcWarm = srcLevel;
		if (level > 0)
		{
			if (!prolongateByNearestNeighbor(prevRest, prevDef, srcLevel, srcWarm, errMsg))
			{
				return false;
			}
			dbg << "[Pyramid] L" << level << " prolongate ok\n";
		}

		const bool lastLayer = (level + 1 == layers);
		const int layerOuter =
			lastLayer ? std::max(params.sdf.maxOuterIters, params.spare.maxOuterIters)
					  : (level == 0 ? kCoarseMaxOuter : kCoarseMaxOuter + 5);

		std::vector<float> srcDef;
		if (params.solver == PyramidSolver::Spare)
		{
			SpareRegisterParams layerParams = params.spare;
			layerParams.rigidPreAlign = (level == 0) && params.rigidPreAlign;
			layerParams.useFineReg = lastLayer && params.useFineRegOnLastLayer;
			layerParams.useCoarseReg = true;
			layerParams.maxOuterIters = layerOuter;
			if (!lastLayer)
			{
				layerParams.alignSampleCount = kCoarseAlignSample;
			}
			SpareRegisterResult spareStats;
			if (!spareRegisterMeshSoupToMeshSoup(srcWarm, tgtLevel, srcDef, layerParams, &spareStats, errMsg))
			{
				return false;
			}
			lastMeanErr = spareStats.meanErrorMm;
			lastNodes = spareStats.deformationNodeCount;
		}
		else
		{
			SdfRegisterParams layerParams = params.sdf;
			layerParams.rigidPreAlign = (level == 0) && params.rigidPreAlign;
			layerParams.useFineReg = lastLayer && params.useFineRegOnLastLayer;
			layerParams.useCoarseReg = true;
			layerParams.maxOuterIters = layerOuter;
			if (!lastLayer)
			{
				layerParams.alignSampleCount = kCoarseAlignSample;
			}
			SdfRegisterResult sdfStats;
			if (!sdfRegisterMeshSoupToMeshSoup(srcWarm, tgtLevel, srcDef, layerParams, &sdfStats, errMsg))
			{
				return false;
			}
			lastMeanErr = sdfStats.meanErrorMm;
			lastNodes = sdfStats.deformationNodeCount;
			if (!sdfStats.debugSummary.empty())
			{
				dbg << sdfStats.debugSummary << "\n";
			}
		}

		prevRest = std::move(srcLevel);
		prevDef = srcDef;
		lastDef = std::move(srcDef);
	}

	sourceSoupDeformedOut = std::move(lastDef);
	if (stats)
	{
		stats->meanErrorMm = lastMeanErr;
		stats->deformationNodeCount = lastNodes;
		stats->baseEdgeLengthMmUsed = baseH;
		stats->layersRun = layers;
		stats->debugSummary = dbg.str();
	}
	return true;
}

} // namespace pclalgo
