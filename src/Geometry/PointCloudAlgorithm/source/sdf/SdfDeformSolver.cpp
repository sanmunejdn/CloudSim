#include "sdf/SdfDeformSolver.h"

#include "KdTreePointSet.h"
#include "Measure.h"
#include "Preprocess.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <tuple>
#include <vector>

namespace pclalgo
{
namespace sdf
{
namespace
{

Eigen::Vector3d readP(const std::vector<float>& xyz, std::size_t i)
{
	return Eigen::Vector3d(xyz[i * 3U], xyz[i * 3U + 1U], xyz[i * 3U + 2U]);
}

void writeP(std::vector<float>& xyz, std::size_t i, const Eigen::Vector3d& p)
{
	xyz[i * 3U] = static_cast<float>(p.x());
	xyz[i * 3U + 1U] = static_cast<float>(p.y());
	xyz[i * 3U + 2U] = static_cast<float>(p.z());
}

Eigen::Vector3d skinnedDelta(const DeformGraph& g, std::size_t vi, const std::vector<Eigen::Vector3d>& nodeT)
{
	Eigen::Vector3d d = Eigen::Vector3d::Zero();
	for (const SkinWeight& sw : g.vertSkin[vi])
	{
		if (sw.node >= 0 && sw.node < static_cast<int>(nodeT.size()))
		{
			d += sw.w * nodeT[static_cast<std::size_t>(sw.node)];
		}
	}
	return d;
}

double welsch(double r2, double nu2)
{
	return std::exp(-r2 / std::max(1e-12, 2.0 * nu2));
}

void clampVec(Eigen::Vector3d& v, double maxLen)
{
	const double len = v.norm();
	if (len > maxLen && len > 1e-12)
	{
		v *= (maxLen / len);
	}
}

Eigen::Vector3d residualAt(const Eigen::Vector3d& x, const FieldSample& s, bool coarse, SdfFieldMode mode,
						   SdfFineDataTerm fineTerm)
{
	if (!s.valid)
	{
		return Eigen::Vector3d::Zero();
	}
	const Eigen::Vector3d d = coarse ? s.directed : (x - s.closest);
	const double vn = d.dot(s.normal);
	const Eigen::Vector3d nComp = vn * s.normal;

	if (coarse)
	{
		if (mode == SdfFieldMode::SignedDistance)
		{
			return nComp;
		}
		const Eigen::Vector3d tComp = d - nComp;
		return nComp + 0.02 * tComp;
	}
	if (fineTerm == SdfFineDataTerm::DdfVector)
	{
		const Eigen::Vector3d tComp = d - nComp;
		return nComp + 0.02 * tComp;
	}
	return nComp;
}

double bboxDiagOf(const std::vector<float>& xyz)
{
	Eigen::AlignedBox3d box;
	for (std::size_t i = 0; i + 2U < xyz.size(); i += 3U)
	{
		box.extend(Eigen::Vector3d(xyz[i], xyz[i + 1U], xyz[i + 2U]));
	}
	return std::max(1e-6, box.diagonal().norm());
}

} // namespace

bool runSdfDeform(std::vector<float>& xyzInOut, std::vector<float>& normalsInOut, DistanceField& field,
				  const SdfRegisterParams& params, SdfRegisterResult* stats, std::string* errMsg,
				  const std::vector<std::pair<int, int>>* meshEdges)
{
	const std::size_t n = xyzInOut.size() / 3U;
	if (n < 100U || normalsInOut.size() != xyzInOut.size())
	{
		if (errMsg)
		{
			*errMsg = "SdfDeform: need >=100 points with normals";
		}
		return false;
	}

	const double diag = bboxDiagOf(xyzInOut);
	const double avgSpacing = std::max(1e-4, computeAverageSpacingMm(xyzInOut, 6U));
	double sampleR = params.sampleRadiusRatio;
	if (sampleR <= 0.0)
	{
		// 节点过稀时，曲面法向拉力在节点域内互抵（cancelRatio≈0.5），加密是主要手段
		sampleR = std::max(avgSpacing * 4.0, diag * 0.015);
	}
	else
	{
		sampleR = avgSpacing * sampleR;
	}
	sampleR = std::max(avgSpacing * 2.0, sampleR);

	// 从边建邻接，供网格拓扑蒙皮
	std::vector<std::vector<int>> vertAdj;
	const std::vector<std::vector<int>>* adjPtr = nullptr;
	if (meshEdges != nullptr && !meshEdges->empty())
	{
		vertAdj.assign(n, {});
		for (const auto& e : *meshEdges)
		{
			if (e.first < 0 || e.second < 0)
			{
				continue;
			}
			const std::size_t a = static_cast<std::size_t>(e.first);
			const std::size_t b = static_cast<std::size_t>(e.second);
			if (a >= n || b >= n || a == b)
			{
				continue;
			}
			vertAdj[a].push_back(static_cast<int>(b));
			vertAdj[b].push_back(static_cast<int>(a));
		}
		adjPtr = &vertAdj;
	}

	DeformGraph graph;
	if (!buildDeformGraph(xyzInOut, sampleR, 4, 8, graph, errMsg, adjPtr))
	{
		return false;
	}
	if (stats)
	{
		stats->deformationNodeCount = static_cast<int>(graph.nodeRest.size());
		stats->fieldVoxelMmUsed = field.fieldVoxelMmUsed();
	}
	if (graph.nodeRest.size() < 4U)
	{
		if (errMsg)
		{
			*errMsg = "SdfDeform: too few deformation nodes";
		}
		return false;
	}

	std::vector<Eigen::Vector3d> nodeT(graph.nodeRest.size(), Eigen::Vector3d::Zero());
	const std::vector<float> restXyz = xyzInOut;
	const std::vector<float> restNormals = normalsInOut;
	// 单步过小会导致 30 轮仍动不了 mm 级误差
	double maxNodeStep = std::max(avgSpacing * 4.0, diag * 0.025);
	const double maxPair = std::min(diag * 0.10, std::max(avgSpacing * 40.0, sampleR * 4.0));
	const double maxPair2 = maxPair * maxPair;
	const double maxTang = std::max(avgSpacing * 12.0, sampleR * 0.8);
	const double maxTang2 = maxTang * maxTang;
	constexpr double minNormalDot = 0.4;
	// 平滑按节点邻接归一，避免 wSmo 下限 8 压死数据项
	const double wSmo = std::max(0.1, params.wSmo) / 8.0;
	double wEdge = 0.0;
	if (meshEdges != nullptr && !meshEdges->empty())
	{
		const double edgeCount = static_cast<double>(meshEdges->size());
		const double sampleN = static_cast<double>(std::max<std::size_t>(1U, params.alignSampleCount));
		wEdge = std::max(0.5, params.wArapCoarse) * (sampleN / edgeCount);
		wEdge = std::min(wEdge, 3.0);
	}
	// 数据项放大：2500 个采样点驱动 511 节点，需足够大梯度才能看到非刚性位移
	double dataGain = 20.0;

	auto applyNodes = [&]() {
		for (std::size_t i = 0; i < n; ++i)
		{
			writeP(xyzInOut, i, readP(restXyz, i) + skinnedDelta(graph, i, nodeT));
		}
	};

	auto smoothNodes = [&]() {
		std::vector<Eigen::Vector3d> smoothed = nodeT;
		for (std::size_t j = 0; j < nodeT.size(); ++j)
		{
			if (graph.nodeNeighbors[j].empty())
			{
				continue;
			}
			Eigen::Vector3d avg = Eigen::Vector3d::Zero();
			for (int nb : graph.nodeNeighbors[j])
			{
				if (nb >= 0)
				{
					avg += nodeT[static_cast<std::size_t>(nb)];
				}
			}
			avg /= static_cast<double>(graph.nodeNeighbors[j].size());
			smoothed[j] = 0.85 * nodeT[j] + 0.15 * avg;
		}
		nodeT.swap(smoothed);
	};

	auto optimizeNodes = [&](bool coarse, int* outSampled, int* outAccepted, int* outRejPair, int* outRejN,
							 int* outRejTang, double* outCancelRatio) -> double {
		const int iters = std::max(1, params.maxOuterIters);
		double nu2 = std::max(sampleR * sampleR, (diag * 0.015) * (diag * 0.015));
		double meanErr = 0.0;
		const double edgeW = coarse ? wEdge : wEdge * 0.5;
		int lastSampled = 0;
		int lastAccepted = 0;
		int lastRejPair = 0;
		int lastRejN = 0;
		int lastRejTang = 0;
		double lastCancel = 1.0;
		for (int it = 0; it < iters; ++it)
		{
			applyNodes();
			std::vector<Eigen::Vector3d> grad(nodeT.size(), Eigen::Vector3d::Zero());
			std::vector<double> mass(nodeT.size(), std::max(1e-3, params.wRot));
			meanErr = 0.0;
			std::size_t errCount = 0;
			lastSampled = 0;
			lastAccepted = 0;
			lastRejPair = 0;
			lastRejN = 0;
			lastRejTang = 0;
			Eigen::Vector3d sumRes = Eigen::Vector3d::Zero();
			double sumResMag = 0.0;
			const std::size_t step = std::max<std::size_t>(1U, n / std::max<std::size_t>(1U, params.alignSampleCount));
			for (std::size_t i = 0; i < n; i += step)
			{
				++lastSampled;
				const Eigen::Vector3d x = readP(xyzInOut, i);
				const FieldSample s = field.query(x);
				if (!s.valid || s.directed.squaredNorm() > maxPair2)
				{
					++lastRejPair;
					continue;
				}
				Eigen::Vector3d sn(restNormals[i * 3U], restNormals[i * 3U + 1U], restNormals[i * 3U + 2U]);
				const double snLen = sn.norm();
				if (snLen > 1e-12)
				{
					sn /= snLen;
					if (sn.dot(s.normal) < minNormalDot)
					{
						++lastRejN;
						continue;
					}
				}
				const Eigen::Vector3d d = s.directed;
				const double vn = d.dot(s.normal);
				const double tang2 = (d - vn * s.normal).squaredNorm();
				if (tang2 > maxTang2)
				{
					++lastRejTang;
					continue;
				}
				const Eigen::Vector3d residual = residualAt(x, s, coarse, params.fieldMode, params.fineDataTerm) * dataGain;
				const double r2 = residual.squaredNorm();
				const double alpha = welsch(r2, nu2);
				const double rMag = std::sqrt(r2);
				meanErr += rMag / dataGain;
				sumRes += residual;
				sumResMag += rMag;
				++errCount;
				++lastAccepted;
				for (const SkinWeight& sw : graph.vertSkin[i])
				{
					if (sw.node < 0)
					{
						continue;
					}
					const std::size_t j = static_cast<std::size_t>(sw.node);
					grad[j] += (alpha * sw.w) * residual;
					mass[j] += alpha * sw.w * sw.w;
				}
			}
			if (errCount == 0U)
			{
				break;
			}
			meanErr /= static_cast<double>(errCount);
			lastCancel = (sumResMag > 1e-12) ? (sumRes.norm() / sumResMag) : 0.0;

			for (std::size_t j = 0; j < nodeT.size(); ++j)
			{
				for (int nb : graph.nodeNeighbors[j])
				{
					if (nb < 0)
					{
						continue;
					}
					const Eigen::Vector3d diff = nodeT[j] - nodeT[static_cast<std::size_t>(nb)];
					grad[j] += wSmo * diff;
					mass[j] += wSmo;
				}
			}

			if (edgeW > 0.0 && meshEdges != nullptr)
			{
				for (const auto& e : *meshEdges)
				{
					if (e.first < 0 || e.second < 0)
					{
						continue;
					}
					const std::size_t a = static_cast<std::size_t>(e.first);
					const std::size_t b = static_cast<std::size_t>(e.second);
					if (a >= n || b >= n)
					{
						continue;
					}
					const Eigen::Vector3d strain = skinnedDelta(graph, b, nodeT) - skinnedDelta(graph, a, nodeT);
					for (const SkinWeight& sw : graph.vertSkin[a])
					{
						if (sw.node < 0)
						{
							continue;
						}
						const std::size_t j = static_cast<std::size_t>(sw.node);
						grad[j] += (-edgeW * sw.w) * strain;
						mass[j] += edgeW * sw.w * sw.w;
					}
					for (const SkinWeight& sw : graph.vertSkin[b])
					{
						if (sw.node < 0)
						{
							continue;
						}
						const std::size_t j = static_cast<std::size_t>(sw.node);
						grad[j] += (edgeW * sw.w) * strain;
						mass[j] += edgeW * sw.w * sw.w;
					}
				}
			}

			double maxStep = 0.0;
			for (std::size_t j = 0; j < nodeT.size(); ++j)
			{
				Eigen::Vector3d delta = -grad[j] / std::max(1e-9, mass[j]);
				clampVec(delta, maxNodeStep);
				nodeT[j] += delta;
				maxStep = std::max(maxStep, delta.norm());
			}
			// 隔轮轻度平滑，避免每步把数据项位移抹掉
			if ((it % 2) == 1)
			{
				smoothNodes();
			}
			nu2 = std::max(avgSpacing * avgSpacing, 0.5 * (nu2 + meanErr * meanErr));
			const double stop = coarse ? params.stopCoarse : params.stopFine;
			if (maxStep < stop)
			{
				break;
			}
		}
		if (outSampled)
		{
			*outSampled = lastSampled;
		}
		if (outAccepted)
		{
			*outAccepted = lastAccepted;
		}
		if (outRejPair)
		{
			*outRejPair = lastRejPair;
		}
		if (outRejN)
		{
			*outRejN = lastRejN;
		}
		if (outRejTang)
		{
			*outRejTang = lastRejTang;
		}
		if (outCancelRatio)
		{
			*outCancelRatio = lastCancel;
		}
		applyNodes();
		return meanErr;
	};

	int corrSampled = 0;
	int corrAccepted = 0;
	int corrRejPair = 0;
	int corrRejN = 0;
	int corrRejTang = 0;
	double cancelRatio = 1.0;

	// 残差分解：总残差 / 法向分量 / 切向分量（normalize 单位）
	auto errDecomp = [&]() {
		double sumR = 0.0;
		double sumN = 0.0;
		double sumT = 0.0;
		int cnt = 0;
		const std::size_t step = std::max<std::size_t>(1U, n / std::max<std::size_t>(1U, params.alignSampleCount));
		for (std::size_t i = 0; i < n; i += step)
		{
			const Eigen::Vector3d x = readP(xyzInOut, i);
			const FieldSample s = field.query(x);
			if (!s.valid || s.directed.squaredNorm() > maxPair2)
			{
				continue;
			}
			const Eigen::Vector3d d = s.directed;
			const double vn = d.dot(s.normal);
			sumR += d.norm();
			sumN += std::abs(vn);
			sumT += (d - vn * s.normal).norm();
			++cnt;
		}
		const double inv = cnt > 0 ? 1.0 / cnt : 0.0;
		return std::make_tuple(sumR * inv, sumN * inv, sumT * inv);
	};

	applyNodes();
	const auto errInit = errDecomp();
	double meanErr = 0.0;
	if (params.useCoarseReg)
	{
		meanErr = optimizeNodes(true, &corrSampled, &corrAccepted, &corrRejPair, &corrRejN, &corrRejTang, &cancelRatio);
	}
	else
	{
		applyNodes();
	}
	// 粗阶段没动起来（位移远小于误差）→ 关边约束，细阶段全力贴目标
	if (params.useFineReg)
	{
		if (params.useCoarseReg && stats->nodeDispMaxMm < meanErr * 0.1)
		{
			// 强制 fine 阶段 dataGain 翻倍，且边权清零，做最后收敛
			wEdge = 0.0;
			const double savedGain = dataGain;
			dataGain *= 3.0;
			meanErr = optimizeNodes(false, &corrSampled, &corrAccepted, &corrRejPair, &corrRejN, &corrRejTang, &cancelRatio);
			dataGain = savedGain;
		}
		else
		{
			meanErr = optimizeNodes(false, &corrSampled, &corrAccepted, &corrRejPair, &corrRejN, &corrRejTang, &cancelRatio);
		}
	}
	applyNodes();

	// 逐顶点阻尼点-面精修：平移节点图无法表达曲面径向收放（拉力互抵），
	// 直接对每顶点消残余法向误差，邻接位移平滑保拓扑
	double vertDispMean = 0.0;
	double vertDispMax = 0.0;
	if (params.useFineReg)
	{
		std::vector<std::vector<int>> nb;
		if (adjPtr != nullptr)
		{
			nb = *adjPtr;
		}
		else
		{
			nb.assign(n, {});
			KdTreePointSet restTree(restXyz);
			for (std::size_t i = 0; i < n; ++i)
			{
				std::vector<std::size_t> idx;
				std::vector<double> d2;
				restTree.findKNearest(restXyz[i * 3U], restXyz[i * 3U + 1U], restXyz[i * 3U + 2U], 9U, idx, d2);
				for (std::size_t k = 0; k < idx.size(); ++k)
				{
					if (idx[k] != i)
					{
						nb[i].push_back(static_cast<int>(idx[k]));
					}
				}
			}
		}

		std::vector<Eigen::Vector3d> disp(n, Eigen::Vector3d::Zero());
		for (std::size_t i = 0; i < n; ++i)
		{
			disp[i] = readP(xyzInOut, i) - readP(restXyz, i);
		}
		const double maxVertStep = std::max(avgSpacing * 3.0, diag * 0.01);
		for (int pass = 0; pass < 8; ++pass)
		{
			for (std::size_t i = 0; i < n; ++i)
			{
				const Eigen::Vector3d x = readP(restXyz, i) + disp[i];
				const FieldSample s = field.query(x);
				if (!s.valid || s.directed.squaredNorm() > maxPair2)
				{
					continue;
				}
				Eigen::Vector3d sn(restNormals[i * 3U], restNormals[i * 3U + 1U], restNormals[i * 3U + 2U]);
				const double snLen = sn.norm();
				if (snLen > 1e-12)
				{
					sn /= snLen;
					if (sn.dot(s.normal) < minNormalDot)
					{
						continue;
					}
				}
				const double vn = s.directed.dot(s.normal);
				Eigen::Vector3d step = (-0.5 * vn) * s.normal;
				clampVec(step, maxVertStep);
				disp[i] += step;
			}
			for (int sm = 0; sm < 2; ++sm)
			{
				std::vector<Eigen::Vector3d> smoothed = disp;
				for (std::size_t i = 0; i < n; ++i)
				{
					if (nb[i].empty())
					{
						continue;
					}
					Eigen::Vector3d avg = Eigen::Vector3d::Zero();
					for (int j : nb[i])
					{
						if (j >= 0 && static_cast<std::size_t>(j) < n)
						{
							avg += disp[static_cast<std::size_t>(j)];
						}
					}
					avg /= static_cast<double>(nb[i].size());
					smoothed[i] = 0.5 * disp[i] + 0.5 * avg;
				}
				disp.swap(smoothed);
			}
		}
		for (std::size_t i = 0; i < n; ++i)
		{
			writeP(xyzInOut, i, readP(restXyz, i) + disp[i]);
			const double len = disp[i].norm();
			vertDispMean += len;
			vertDispMax = std::max(vertDispMax, len);
		}
		vertDispMean = n > 0U ? vertDispMean / static_cast<double>(n) : 0.0;
	}

	{
		// 末尾法线重估不能调 orientNormalsMst（会重排 xyz 打乱网格索引）；
		// PCA 保持逐点顺序，再按变形前法线定向符号即可
		std::vector<float> newN;
		if (estimateNormalsPca(xyzInOut, newN, 12U, nullptr) && newN.size() == normalsInOut.size())
		{
			for (std::size_t i = 0; i < n; ++i)
			{
				const std::size_t b = i * 3U;
				const double dot = static_cast<double>(newN[b]) * restNormals[b] +
								   static_cast<double>(newN[b + 1U]) * restNormals[b + 1U] +
								   static_cast<double>(newN[b + 2U]) * restNormals[b + 2U];
				if (dot < 0.0)
				{
					newN[b] = -newN[b];
					newN[b + 1U] = -newN[b + 1U];
					newN[b + 2U] = -newN[b + 2U];
				}
			}
			normalsInOut.swap(newN);
		}
	}

	const auto errFinal = errDecomp();

	if (stats)
	{
		stats->errInitNormal = std::get<1>(errInit);
		stats->errInitTangential = std::get<2>(errInit);
		stats->errFinalMean = std::get<0>(errFinal);
		stats->errFinalNormal = std::get<1>(errFinal);
		stats->errFinalTangential = std::get<2>(errFinal);
		stats->meanErrorMm = meanErr;
		stats->sourceVertexCount = static_cast<int>(n);
		stats->meshEdgeCount = meshEdges ? static_cast<int>(meshEdges->size()) : 0;
		stats->usedMeshTopology = adjPtr != nullptr;
		stats->corrSampled = corrSampled;
		stats->corrAccepted = corrAccepted;
		stats->corrRejectMaxPair = corrRejPair;
		stats->corrRejectNormal = corrRejN;
		stats->corrRejectTangential = corrRejTang;

		int singleSkin = 0;
		double skinSum = 0.0;
		for (std::size_t i = 0; i < n; ++i)
		{
			const int k = static_cast<int>(graph.vertSkin[i].size());
			skinSum += static_cast<double>(k);
			if (k <= 1)
			{
				++singleSkin;
			}
		}
		stats->skinSingleNodeVerts = singleSkin;
		stats->skinAvgNodesPerVert = n > 0U ? skinSum / static_cast<double>(n) : 0.0;

		double dispSum = 0.0;
		double dispMax = 0.0;
		for (const Eigen::Vector3d& t : nodeT)
		{
			const double len = t.norm();
			dispSum += len;
			dispMax = std::max(dispMax, len);
		}
		stats->nodeDispMeanMm = nodeT.empty() ? 0.0 : dispSum / static_cast<double>(nodeT.size());
		stats->nodeDispMaxMm = dispMax;

		if (meshEdges != nullptr && !meshEdges->empty())
		{
			std::vector<double> ratios;
			ratios.reserve(meshEdges->size());
			int over2 = 0;
			int over5 = 0;
			int over10 = 0;
			double sumR = 0.0;
			double maxR = 1.0;
			for (const auto& e : *meshEdges)
			{
				if (e.first < 0 || e.second < 0)
				{
					continue;
				}
				const std::size_t a = static_cast<std::size_t>(e.first);
				const std::size_t b = static_cast<std::size_t>(e.second);
				if (a >= n || b >= n)
				{
					continue;
				}
				const double restLen = (readP(restXyz, b) - readP(restXyz, a)).norm();
				const double curLen = (readP(xyzInOut, b) - readP(xyzInOut, a)).norm();
				if (restLen < 1e-12)
				{
					continue;
				}
				const double r = curLen / restLen;
				ratios.push_back(r);
				sumR += r;
				maxR = std::max(maxR, r);
				if (r > 2.0)
				{
					++over2;
				}
				if (r > 5.0)
				{
					++over5;
				}
				if (r > 10.0)
				{
					++over10;
				}
			}
			stats->edgeStretchMean = ratios.empty() ? 1.0 : sumR / static_cast<double>(ratios.size());
			stats->edgeStretchMax = maxR;
			stats->edgeStretchOver2 = over2;
			stats->edgeStretchOver5 = over5;
			stats->edgeStretchOver10 = over10;
			if (!ratios.empty())
			{
				std::nth_element(ratios.begin(), ratios.begin() + static_cast<std::ptrdiff_t>(ratios.size() * 95U / 100U),
								 ratios.end());
				stats->edgeStretchP95 = ratios[ratios.size() * 95U / 100U];
			}
		}

		std::ostringstream oss;
		oss << "[SDF-debug] verts=" << n << " nodes=" << graph.nodeRest.size()
			<< " meshTopo=" << (stats->usedMeshTopology ? 1 : 0) << " edges=" << stats->meshEdgeCount << "\n";
		oss << "[SDF-debug] sampleR=" << sampleR << " maxPair=" << maxPair << " maxTang=" << maxTang
			<< " wSmo=" << wSmo << " wEdge=" << wEdge << " maxStep=" << maxNodeStep << "\n";
		oss << "[SDF-debug] corr sampled=" << corrSampled << " accepted=" << corrAccepted
			<< " rejPair=" << corrRejPair << " rejNormal=" << corrRejN << " rejTang=" << corrRejTang;
		if (corrSampled > 0)
		{
			oss << " acceptRate=" << (100.0 * corrAccepted / corrSampled) << "%";
		}
		oss << " cancelRatio=" << cancelRatio << "\n";
		oss << "[SDF-debug] skin avgNodes=" << stats->skinAvgNodesPerVert << " singleNodeVerts=" << singleSkin;
		if (n > 0U)
		{
			oss << " (" << (100.0 * singleSkin / static_cast<double>(n)) << "%)";
		}
		oss << "\n";
		oss << "[SDF-debug] vertFine dispMean=" << vertDispMean << " dispMax=" << vertDispMax << "\n";
		stats->vertDispMeanMm = vertDispMean;
		stats->vertDispMaxMm = vertDispMax;
		if (stats->meshEdgeCount > 0)
		{
			oss << "[SDF-debug] edgeStretch mean=" << stats->edgeStretchMean << " p95=" << stats->edgeStretchP95
				<< " max=" << stats->edgeStretchMax << " | >2x:" << stats->edgeStretchOver2
				<< " >5x:" << stats->edgeStretchOver5 << " >10x:" << stats->edgeStretchOver10 << "\n";
		}
		if (stats->meshEdgeCount > 0 && stats->edgeStretchOver10 > 0)
		{
			oss << "[SDF-WARN] 边长拉伸>10x 数量=" << stats->edgeStretchOver10
				<< " → 典型蜘蛛网/长三角，优先查蒙皮跨瓣或边约束不足\n";
		}
		else if (stats->meshEdgeCount > 0 && stats->edgeStretchOver5 > 0)
		{
			oss << "[SDF-WARN] 边长拉伸>5x 数量=" << stats->edgeStretchOver5 << " → 局部拓扑已撕裂\n";
		}
		if (corrSampled > 0 && corrAccepted * 5 < corrSampled)
		{
			oss << "[SDF-WARN] 对应接受率过低 → 预对齐/尺度/法线门控过严，变形可能乱滑\n";
		}
		if (n > 0U && singleSkin * 2 > static_cast<int>(n))
		{
			oss << "[SDF-WARN] 单节点蒙皮顶点过多 → 节间断缝/尖刺风险\n";
		}
		if (meanErr > 1e-6 && stats->nodeDispMaxMm < meanErr * 0.05)
		{
			oss << "[SDF-WARN] nodeDisp≪meanErr → 非刚性几乎未改几何，输出≈刚性预对齐后的源；"
				   "若画面呈蜘蛛网，请先打开源网格边显示核对是否原本即有长三角，或检查源/目标 worldMatrix\n";
		}
		if (cancelRatio < 0.15 && meanErr > 1e-6)
		{
			oss << "[SDF-WARN] cancelRatio过低 → 残差矢量相互抵消，局部法向误差难合成节点位移\n";
		}
		if (!stats->usedMeshTopology && stats->meshEdgeCount == 0)
		{
			oss << "[SDF-INFO] 无网格边(点云路径) → 无边长诊断；错乱时请用 mesh 源\n";
		}
		if (stats->meshEdgeCount > 0 && !stats->usedMeshTopology)
		{
			oss << "[SDF-WARN] 有边但未走网格拓扑蒙皮 → 欧氏 kNN 可能跨凹槽\n";
		}
		stats->debugSummary = oss.str();
	}
	return true;
}

} // namespace sdf
} // namespace pclalgo
