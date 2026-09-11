/// @file RegistrationSpare.cpp
/// @brief RegistrationSpare 实现

#include "RegistrationSpare.h"

#include "Downsample.h"
#include "KdTreePointSet.h"
#include "Measure.h"
#include "PointCloudBuffer.h"
#include "Preprocess.h"
#include "RegistrationGlobal.h"
#include "RegistrationGlobalPcl.h"
#include "RegistrationRigid.h"
#include "Transform.h"
#include "spare/SpareSolver.h"
#include "spare/SpareSurface.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace pclalgo
{
namespace
{
void transformNormalsInPlace(std::vector<float>& normals, const Eigen::Isometry3d& t)
{
	if (normals.size() < 3U)
	{
		return;
	}
	const Eigen::Matrix3d rot = t.linear();
	const std::size_t n = normals.size() / 3U;
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		Eigen::Vector3d nr(normals[b], normals[b + 1U], normals[b + 2U]);
		nr = rot * nr;
		const double len = nr.norm();
		if (len > 1e-12)
		{
			nr /= len;
		}
		normals[b] = static_cast<float>(nr.x());
		normals[b + 1U] = static_cast<float>(nr.y());
		normals[b + 2U] = static_cast<float>(nr.z());
	}
}

bool copyNormalsOrEstimate(std::vector<float>& xyz, const std::vector<float>& normalsIn, std::vector<float>& normalsOut,
						   std::string* errMsg)
{
	if (normalsIn.size() == xyz.size() && !normalsIn.empty())
	{
		normalsOut = normalsIn;
		return true;
	}
	normalsOut.clear();
	if (!estimateNormalsPca(xyz, normalsOut, 12U, errMsg))
	{
		return false;
	}
	// MST 会裁掉无法定向的点；必须写回 xyz，否则后续点-面 ICP 报 normal buffer length mismatch
	return orientNormalsMst(xyz, normalsOut, 12U, nullptr, errMsg);
}

// 与 SDF / ICP 默认配对半径同量级；内点过少时仍做质心平移
constexpr double kOverlapNnFrac = 0.05;
constexpr double kOverlapInlierRatio = 0.5;
constexpr std::size_t kMinFpfhPoints = 50U;

bool alreadyOverlapping(const std::vector<float>& srcXyz, const std::vector<float>& tgtXyz, const double diag)
{
	if (diag <= 1e-9 || srcXyz.size() < 3U || tgtXyz.size() < 3U)
	{
		return false;
	}
	const KdTreePointSet tree(tgtXyz);
	if (tree.empty())
	{
		return false;
	}
	const std::size_t n = pointCountFromXyz(srcXyz);
	const std::size_t step = std::max<std::size_t>(1U, n / 400U);
	std::size_t sampled = 0;
	std::size_t closeHits = 0;
	const double closeDistSq = (diag * kOverlapNnFrac) * (diag * kOverlapNnFrac);
	for (std::size_t i = 0; i < n; i += step)
	{
		++sampled;
		const std::size_t b = i * 3U;
		double d2 = 0.0;
		if (tree.findNearest(static_cast<double>(srcXyz[b]), static_cast<double>(srcXyz[b + 1U]),
							 static_cast<double>(srcXyz[b + 2U]), closeDistSq, d2) != static_cast<std::size_t>(-1))
		{
			++closeHits;
		}
	}
	if (sampled < 16U)
	{
		return false;
	}
	return static_cast<double>(closeHits) / static_cast<double>(sampled) >= kOverlapInlierRatio;
}

spare::SpareInternalParams toInternalParams(const SpareRegisterParams& params)
{
	spare::SpareInternalParams out;
	out.maxOuterIters = params.maxOuterIters;
	out.wSmo = params.wSmo;
	out.wRot = params.wRot;
	out.wArapCoarse = params.wArapCoarse;
	out.wArapFine = params.wArapFine;
	out.useSymmetricPointToPlane = params.useSymmetricPointToPlane;
	out.useCoarseReg = params.useCoarseReg;
	out.useFineReg = params.useFineReg;
	out.stopCoarse = params.stopCoarse;
	out.stopFine = params.stopFine;
	out.alignSampleCount = params.alignSampleCount;
	out.uniSampleRatio = params.sampleRadiusRatio;
	return out;
}

bool applyRigidIcpPreAlign(std::vector<float>& sourceXyz, std::vector<float>& sourceNormals,
						   const std::vector<float>& targetXyz, const std::vector<float>& targetNormals,
						   const SpareRegisterParams& params, std::string* errMsg)
{
	const double tgtDiag = std::max(1e-6, computeBoundingBox(targetXyz).diagonal().norm());
	if (!alreadyOverlapping(sourceXyz, targetXyz, tgtDiag))
	{
		Eigen::Isometry3d shift = Eigen::Isometry3d::Identity();
		shift.translation() = computeCentroid(targetXyz) - computeCentroid(sourceXyz);
		transformXyzInPlace(sourceXyz, shift);
	}

	Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
	double rmse = 0.0;
	if (!rigidRegisterPointToPlaneIcp(sourceXyz, sourceNormals, targetXyz, targetNormals, transform, &rmse,
									  params.rigidPreAlignMaxIterations, 0.01, params.rigidPreAlignMaxPairDistanceMm,
									  params.rigidPreAlignMaxPoints, errMsg))
	{
		return false;
	}
	transformXyzInPlace(sourceXyz, transform);
	transformNormalsInPlace(sourceNormals, transform);
	return true;
}

bool applyPreAlign(std::vector<float>& sourceXyz, std::vector<float>& sourceNormals,
				   const std::vector<float>& targetXyz, const std::vector<float>& targetNormals,
				   const SpareRegisterParams& params, std::string* errMsg, std::string* preAlignNote)
{
	auto setNote = [&](const std::string& note) {
		if (preAlignNote)
		{
			*preAlignNote = note;
		}
	};

	if (params.coarseGlobalAlign)
	{
		const bool enoughForFpfh =
			pointCountFromXyz(sourceXyz) >= kMinFpfhPoints && pointCountFromXyz(targetXyz) >= kMinFpfhPoints;
		if (enoughForFpfh)
		{
			Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
			double inlierRatio = 0.0;
			bool ok = false;
			const char* stage = nullptr;
			std::string stageErr;
#ifdef CLOUDSIM_HAS_PCL
			{
				PclGlobalAlignParams pclParams;
				pclParams.refineWithIcp = true;
				ok = rigidRegisterFeatureRansacPcl(sourceXyz, sourceNormals, targetXyz, targetNormals, transform,
												   &inlierRatio, pclParams, &stageErr);
				if (ok)
				{
					stage = "pcl";
				}
			}
			// 有 PCL 时不再回退自研 FPFH：后者常以 ~20% 内点“成功”却留下大转角误差
#else
			{
				RigidRegisterRansacParams ransacParams;
				ransacParams.refineWithIcp = true;
				ransacParams.skipTranslationCap = true;
				ok = rigidRegisterFeatureRansac(sourceXyz, sourceNormals, targetXyz, targetNormals, transform,
												&inlierRatio, ransacParams, &stageErr);
				if (ok && inlierRatio >= 0.30)
				{
					stage = "selfRansac";
				}
				else if (ok)
				{
					ok = false;
					std::ostringstream oss;
					oss << "selfRansac inlier too low: " << inlierRatio;
					stageErr = oss.str();
				}
			}
#endif
			if (ok)
			{
				transformXyzInPlace(sourceXyz, transform);
				transformNormalsInPlace(sourceNormals, transform);
				std::ostringstream oss;
				oss << (stage ? stage : "ransac") << " inlier=" << inlierRatio;
				setNote(oss.str());
				return true;
			}
			if (preAlignNote && !stageErr.empty())
			{
				*preAlignNote = std::string("ransacFail→icp: ") + stageErr;
			}
		}
		else
		{
			setNote("tooFewPts→icp");
		}
		// FPFH 点数不够或 RANSAC 失败时退回质心+点-面 ICP，避免整次 SPARE 中断
		if (errMsg)
		{
			errMsg->clear();
		}
		const bool icpOk = applyRigidIcpPreAlign(sourceXyz, sourceNormals, targetXyz, targetNormals, params, errMsg);
		if (icpOk && preAlignNote && preAlignNote->empty())
		{
			setNote("icp");
		}
		return icpOk;
	}

	if (!params.rigidPreAlign)
	{
		setNote("none");
		return true;
	}

	const bool icpOk = applyRigidIcpPreAlign(sourceXyz, sourceNormals, targetXyz, targetNormals, params, errMsg);
	if (icpOk)
	{
		setNote("icp");
	}
	return icpOk;
}

bool resolveSampleRadius(const spare::SpareSurface& source, const SpareRegisterParams& params,
						 spare::SpareInternalParams& internal)
{
	(void)source;
	if (params.sampleRadiusRatio > 0.0)
	{
		internal.uniSampleRatio = params.sampleRadiusRatio;
		return true;
	}
	// 无量纲：采样半径 = 比 × 平均边长；越小节点越密（旧 spacing×10 量纲错，大网格常只剩几十节点）
	internal.uniSampleRatio = 3.0;
	return true;
}

bool runSpareCore(spare::SpareSurface& source, spare::SpareSurface& target, const SpareRegisterParams& params,
				  SpareRegisterResult* stats, std::string* errMsg)
{
	spare::SpareInternalParams internal = toInternalParams(params);
	internal.useGeodesicDist = source.hasFaces();
	if (!resolveSampleRadius(source, params, internal))
	{
		if (errMsg)
		{
			*errMsg = "failed to resolve sample radius";
		}
		return false;
	}

	double meshScale = 1.0;
	if (params.normalizeScale)
	{
		meshScale = static_cast<double>(spare::normalizeSpareSurfaces(source, target));
	}

	spare::SpareSolver solver;
	if (!solver.init(source, target, internal))
	{
		if (errMsg)
		{
			*errMsg = "SPARE solver init failed";
		}
		return false;
	}
	if (!solver.run())
	{
		if (errMsg)
		{
			*errMsg = "SPARE solver run failed";
		}
		return false;
	}

	if (params.normalizeScale && meshScale > 1e-12)
	{
		spare::applyScaleToSpareSurface(source, static_cast<spare::Scalar>(1.0 / meshScale));
	}

	if (stats != nullptr)
	{
		stats->meanErrorMm = static_cast<double>(solver.meanError());
		stats->meshScale = meshScale;
		stats->deformationNodeCount = internal.numSampleNodes;
		// 求解在单位包围盒内；反缩放到 mm（与 SDF 一致）
		if (params.normalizeScale && meshScale > 1e-12)
		{
			stats->meanErrorMm /= meshScale;
		}
	}
	return true;
}

} // namespace

bool spareRegisterPointClouds(const std::vector<float>& sourceXyz, const std::vector<float>& sourceNormals,
							  const std::vector<float>& targetXyz, const std::vector<float>& targetNormals,
							  std::vector<float>& sourceXyzDeformedOut, std::vector<float>& sourceNormalsDeformedOut,
							  const SpareRegisterParams& params, SpareRegisterResult* stats, std::string* errMsg)
{
	if (!validXyzLength(sourceXyz) || !validXyzLength(targetXyz))
	{
		if (errMsg)
		{
			*errMsg = "invalid source or target xyz";
		}
		return false;
	}
	if (pointCountFromXyz(sourceXyz) < 100U || pointCountFromXyz(targetXyz) < 100U)
	{
		if (errMsg)
		{
			*errMsg = "SPARE requires at least 100 points";
		}
		return false;
	}

	std::vector<float> srcXyz = sourceXyz;
	std::vector<float> tgtXyz = targetXyz;
	std::vector<float> srcNormals;
	std::vector<float> tgtNormals;
	if (!copyNormalsOrEstimate(srcXyz, sourceNormals, srcNormals, errMsg))
	{
		return false;
	}
	if (!copyNormalsOrEstimate(tgtXyz, targetNormals, tgtNormals, errMsg))
	{
		return false;
	}

	if (params.voxelPrefilterMm > 0.0)
	{
		(void)downsampleVoxelGrid(srcXyz, params.voxelPrefilterMm, 1U, nullptr);
		(void)downsampleVoxelGrid(tgtXyz, params.voxelPrefilterMm, 1U, nullptr);
		if (!copyNormalsOrEstimate(srcXyz, {}, srcNormals, errMsg))
		{
			return false;
		}
		if (!copyNormalsOrEstimate(tgtXyz, {}, tgtNormals, errMsg))
		{
			return false;
		}
	}

	std::string preAlignNote;
	if (!applyPreAlign(srcXyz, srcNormals, tgtXyz, tgtNormals, params, errMsg, &preAlignNote))
	{
		return false;
	}
	if (stats)
	{
		stats->preAlignNote = preAlignNote;
	}

	spare::SpareSurface source;
	spare::SpareSurface target;
	if (!spare::buildSpareSurfaceFromXyz(source, srcXyz, &srcNormals, true, errMsg))
	{
		return false;
	}
	if (!spare::buildSpareSurfaceFromXyz(target, tgtXyz, &tgtNormals, false, errMsg))
	{
		return false;
	}

	if (!runSpareCore(source, target, params, stats, errMsg))
	{
		return false;
	}
	if (stats && stats->preAlignNote.empty())
	{
		stats->preAlignNote = preAlignNote;
	}

	spare::spareSurfaceToXyz(source, sourceXyzDeformedOut, sourceNormalsDeformedOut);
	return true;
}

bool spareRegisterMeshSoupToTarget(const std::vector<float>& sourceSoup, const std::vector<float>& targetXyz,
								   const std::vector<float>& targetNormals, std::vector<float>& sourceSoupDeformedOut,
								   const SpareRegisterParams& params, SpareRegisterResult* stats, std::string* errMsg)
{
	if (sourceSoup.size() < 9U || (sourceSoup.size() % 9U) != 0U || !validXyzLength(targetXyz))
	{
		if (errMsg)
		{
			*errMsg = "invalid mesh soup or target xyz";
		}
		return false;
	}

	spare::SpareSurface source;
	if (!spare::buildSpareSurfaceFromMeshSoup(source, sourceSoup, errMsg))
	{
		return false;
	}
	if (!spare::ensureSpareSurfaceNormals(source, errMsg))
	{
		return false;
	}

	std::vector<float> srcXyz;
	std::vector<float> srcNormals;
	spare::spareSurfaceToXyz(source, srcXyz, srcNormals);

	std::vector<float> tgtXyz = targetXyz;
	std::vector<float> tgtNormals;
	if (!copyNormalsOrEstimate(tgtXyz, targetNormals, tgtNormals, errMsg))
	{
		return false;
	}

	// 网格 SPARE 原先忽略体素；大网格会极慢。体素>0 时在简化点云上求位移再映射回全顶点
	if (params.voxelPrefilterMm > 0.0)
	{
		std::vector<float> srcWork = srcXyz;
		std::vector<float> tgtWork = tgtXyz;
		(void)downsampleVoxelGrid(srcWork, params.voxelPrefilterMm, 1U, nullptr);
		(void)downsampleVoxelGrid(tgtWork, params.voxelPrefilterMm, 1U, nullptr);
		std::vector<float> srcWorkN;
		std::vector<float> tgtWorkN;
		if (!copyNormalsOrEstimate(srcWork, {}, srcWorkN, errMsg) ||
			!copyNormalsOrEstimate(tgtWork, {}, tgtWorkN, errMsg))
		{
			return false;
		}
		const std::vector<float> srcWorkBefore = srcWork;
		std::vector<float> srcWorkDef;
		std::vector<float> srcWorkDefN;
		SpareRegisterParams pcParams = params;
		pcParams.voxelPrefilterMm = 0.0;
		if (!spareRegisterPointClouds(srcWork, srcWorkN, tgtWork, tgtWorkN, srcWorkDef, srcWorkDefN, pcParams, stats,
									  errMsg))
		{
			return false;
		}
		if (srcWorkDef.size() != srcWorkBefore.size())
		{
			if (errMsg)
			{
				*errMsg = "SPARE voxel remap size mismatch";
			}
			return false;
		}
		KdTreePointSet tree(srcWorkBefore);
		const double maxDistSq = std::numeric_limits<double>::max();
		for (std::size_t i = 0; i < source.vertexCount(); ++i)
		{
			const std::size_t b = i * 3U;
			double distSq = 0.0;
			const std::size_t nn = tree.findNearest(static_cast<double>(srcXyz[b]), static_cast<double>(srcXyz[b + 1U]),
													static_cast<double>(srcXyz[b + 2U]), maxDistSq, distSq);
			if (nn == static_cast<std::size_t>(-1) || nn * 3U + 2U >= srcWorkDef.size())
			{
				continue;
			}
			const std::size_t nb = nn * 3U;
			const float dx = srcWorkDef[nb] - srcWorkBefore[nb];
			const float dy = srcWorkDef[nb + 1U] - srcWorkBefore[nb + 1U];
			const float dz = srcWorkDef[nb + 2U] - srcWorkBefore[nb + 2U];
			source.positions[i] = spare::Vector3(srcXyz[b] + dx, srcXyz[b + 1U] + dy, srcXyz[b + 2U] + dz);
			if (nb + 2U < srcWorkDefN.size())
			{
				source.normals[i] = spare::Vector3(srcWorkDefN[nb], srcWorkDefN[nb + 1U], srcWorkDefN[nb + 2U]);
			}
		}
		spare::spareSurfaceToMeshSoup(source, sourceSoup, sourceSoupDeformedOut);
		return true;
	}

	std::string preAlignNote;
	if (!applyPreAlign(srcXyz, srcNormals, tgtXyz, tgtNormals, params, errMsg, &preAlignNote))
	{
		return false;
	}
	if (stats)
	{
		stats->preAlignNote = preAlignNote;
	}

	spare::SpareSurface target;
	if (!spare::buildSpareSurfaceFromXyz(target, tgtXyz, &tgtNormals, false, errMsg))
	{
		return false;
	}

	for (std::size_t i = 0; i < source.vertexCount(); ++i)
	{
		const std::size_t b = i * 3U;
		source.positions[i] = spare::Vector3(srcXyz[b], srcXyz[b + 1U], srcXyz[b + 2U]);
		source.normals[i] = spare::Vector3(srcNormals[b], srcNormals[b + 1U], srcNormals[b + 2U]);
	}

	if (!runSpareCore(source, target, params, stats, errMsg))
	{
		return false;
	}
	if (stats && stats->preAlignNote.empty())
	{
		stats->preAlignNote = preAlignNote;
	}

	spare::spareSurfaceToMeshSoup(source, sourceSoup, sourceSoupDeformedOut);
	return true;
}

bool spareRegisterMeshSoupToMeshSoup(const std::vector<float>& sourceSoup, const std::vector<float>& targetSoup,
									 std::vector<float>& sourceSoupDeformedOut, const SpareRegisterParams& params,
									 SpareRegisterResult* stats, std::string* errMsg)
{
	spare::SpareSurface targetMesh;
	if (!spare::buildSpareSurfaceFromMeshSoup(targetMesh, targetSoup, errMsg))
	{
		return false;
	}
	if (!spare::ensureSpareSurfaceNormals(targetMesh, errMsg))
	{
		return false;
	}

	std::vector<float> tgtXyz;
	std::vector<float> tgtNormals;
	spare::spareSurfaceToXyz(targetMesh, tgtXyz, tgtNormals);
	return spareRegisterMeshSoupToTarget(sourceSoup, tgtXyz, tgtNormals, sourceSoupDeformedOut, params, stats, errMsg);
}

} // namespace pclalgo
