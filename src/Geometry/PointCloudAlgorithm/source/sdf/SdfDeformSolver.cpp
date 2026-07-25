#include "sdf/SdfDeformSolver.h"

#include "KdTreePointSet.h"
#include "Measure.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

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

} // namespace

bool runSdfDeform(std::vector<float>& xyzInOut, std::vector<float>& normalsInOut, DistanceField& field,
				  const SdfRegisterParams& params, SdfRegisterResult* stats, std::string* errMsg)
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

	double sampleR = params.sampleRadiusRatio;
	if (sampleR <= 0.0)
	{
		sampleR = std::max(1e-3, computeAverageSpacingMm(xyzInOut, 6U) * 10.0);
	}
	else
	{
		sampleR = std::max(1e-3, computeAverageSpacingMm(xyzInOut, 6U) * sampleR);
	}

	DeformGraph graph;
	if (!buildDeformGraph(xyzInOut, sampleR, 4, 6, graph, errMsg))
	{
		return false;
	}
	if (stats)
	{
		stats->deformationNodeCount = static_cast<int>(graph.nodeRest.size());
		stats->fieldVoxelMmUsed = field.fieldVoxelMmUsed();
	}

	std::vector<Eigen::Vector3d> nodeT(graph.nodeRest.size(), Eigen::Vector3d::Zero());
	std::vector<float> restXyz = xyzInOut;

	auto applyNodes = [&]() {
		for (std::size_t i = 0; i < n; ++i)
		{
			writeP(xyzInOut, i, readP(restXyz, i) + skinnedDelta(graph, i, nodeT));
		}
	};

	double meanErr = 0.0;
	if (params.useCoarseReg)
	{
		const int iters = std::max(1, params.maxOuterIters);
		double nu2 = sampleR * sampleR;
		for (int it = 0; it < iters; ++it)
		{
			applyNodes();
			std::vector<Eigen::Vector3d> grad(nodeT.size(), Eigen::Vector3d::Zero());
			std::vector<double> mass(nodeT.size(), params.wRot);
			meanErr = 0.0;
			std::size_t errCount = 0;
			const std::size_t step = std::max<std::size_t>(1U, n / std::max<std::size_t>(1U, params.alignSampleCount));
			for (std::size_t i = 0; i < n; i += step)
			{
				const Eigen::Vector3d x = readP(xyzInOut, i);
				const FieldSample s = field.query(x);
				if (!s.valid)
				{
					continue;
				}
				Eigen::Vector3d residual;
				if (params.fieldMode == SdfFieldMode::SignedDistance)
				{
					residual = s.signedDistance * s.normal;
				}
				else
				{
					residual = s.directed;
				}
				const double r2 = residual.squaredNorm();
				const double alpha = welsch(r2, nu2);
				meanErr += std::sqrt(r2);
				++errCount;
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
			if (errCount > 0)
			{
				meanErr /= static_cast<double>(errCount);
			}
			// 节点平滑
			for (std::size_t j = 0; j < nodeT.size(); ++j)
			{
				for (int nb : graph.nodeNeighbors[j])
				{
					if (nb < 0)
					{
						continue;
					}
					const Eigen::Vector3d diff = nodeT[j] - nodeT[static_cast<std::size_t>(nb)];
					grad[j] += params.wSmo * diff;
					mass[j] += params.wSmo;
				}
			}
			double maxStep = 0.0;
			for (std::size_t j = 0; j < nodeT.size(); ++j)
			{
				const Eigen::Vector3d delta = -grad[j] / std::max(1e-9, mass[j] + params.wArapCoarse * 1e-4);
				nodeT[j] += delta;
				maxStep = std::max(maxStep, delta.norm());
			}
			nu2 = std::max(1e-6, 0.5 * (nu2 + meanErr * meanErr));
			if (maxStep < params.stopCoarse)
			{
				break;
			}
		}
		applyNodes();
	}

	if (params.useFineReg)
	{
		KdTreePointSet tgtTree;
		// 细阶段点-面：用场的 closest/normal 即可，无需另建树
		const int iters = std::max(1, params.maxOuterIters);
		for (int it = 0; it < iters; ++it)
		{
			double maxMove = 0.0;
			meanErr = 0.0;
			std::size_t errCount = 0;
			std::vector<Eigen::Vector3d> newPos(n);
			for (std::size_t i = 0; i < n; ++i)
			{
				const Eigen::Vector3d x = readP(xyzInOut, i);
				const FieldSample s = field.query(x);
				Eigen::Vector3d delta = Eigen::Vector3d::Zero();
				if (s.valid)
				{
					if (params.fineDataTerm == SdfFineDataTerm::PointToPlane ||
						params.fineDataTerm == SdfFineDataTerm::SignedDistance)
					{
						const double phi = (params.fineDataTerm == SdfFineDataTerm::PointToPlane)
											   ? (x - s.closest).dot(s.normal)
											   : s.signedDistance;
						delta = -phi * s.normal;
						meanErr += std::abs(phi);
					}
					else
					{
						delta = -s.directed;
						meanErr += s.directed.norm();
					}
					++errCount;
				}
				// ARAP 邻域：向邻居平均靠拢（轻量）
				Eigen::Vector3d lap = Eigen::Vector3d::Zero();
				int lapN = 0;
				for (const SkinWeight& sw : graph.vertSkin[i])
				{
					for (const SkinWeight& sw2 : graph.vertSkin[i])
					{
						(void)sw2;
					}
					(void)sw;
				}
				// 用节点邻接近似：拉向 rest 相对位移一致
				const Eigen::Vector3d rest = readP(restXyz, i);
				for (const SkinWeight& sw : graph.vertSkin[i])
				{
					if (sw.node < 0)
					{
						continue;
					}
					const Eigen::Vector3d nr = graph.nodeRest[static_cast<std::size_t>(sw.node)];
					const Eigen::Vector3d nd = nr + nodeT[static_cast<std::size_t>(sw.node)];
					lap += sw.w * ((nd - nr) - (x - rest));
					++lapN;
				}
				if (lapN > 0)
				{
					delta += (params.wArapFine / (params.wArapFine + 1.0)) * lap;
				}
				newPos[i] = x + 0.5 * delta;
				maxMove = std::max(maxMove, (0.5 * delta).norm());
			}
			for (std::size_t i = 0; i < n; ++i)
			{
				writeP(xyzInOut, i, newPos[i]);
			}
			if (errCount > 0)
			{
				meanErr /= static_cast<double>(errCount);
			}
			if (maxMove < params.stopFine)
			{
				break;
			}
		}
		(void)tgtTree;
	}

	// 用法线：对每个点重新用场法线近似变形后朝向
	for (std::size_t i = 0; i < n; ++i)
	{
		const FieldSample s = field.query(readP(xyzInOut, i));
		if (s.valid)
		{
			normalsInOut[i * 3U] = static_cast<float>(s.normal.x());
			normalsInOut[i * 3U + 1U] = static_cast<float>(s.normal.y());
			normalsInOut[i * 3U + 2U] = static_cast<float>(s.normal.z());
		}
	}

	if (stats)
	{
		stats->meanErrorMm = meanErr;
	}
	return true;
}

} // namespace sdf
} // namespace pclalgo
