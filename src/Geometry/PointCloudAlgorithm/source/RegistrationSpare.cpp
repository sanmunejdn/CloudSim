#include "RegistrationSpare.h"

#include "Downsample.h"
#include "KdTreePointSet.h"
#include "Measure.h"
#include "PointCloudBuffer.h"
#include "Preprocess.h"
#include "RegistrationGlobal.h"
#include "RegistrationRigid.h"
#include "Transform.h"
#include "spare/SpareSolver.h"
#include "spare/SpareSurface.h"

#include <algorithm>
#include <cmath>
#include <limits>

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

bool copyNormalsOrEstimate(
	const std::vector<float>& xyz,
	const std::vector<float>& normalsIn,
	std::vector<float>& normalsOut,
	std::string* errMsg)
{
	if (normalsIn.size() == xyz.size() && !normalsIn.empty())
	{
		normalsOut = normalsIn;
		return true;
	}
	normalsOut.clear();
	std::vector<float> xyzMutable = xyz;
	if (!estimateNormalsPca(xyzMutable, normalsOut, 12U, errMsg))
	{
		return false;
	}
	return orientNormalsMst(xyzMutable, normalsOut, 12U, nullptr, errMsg);
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

bool applyPreAlign(
	std::vector<float>& sourceXyz,
	std::vector<float>& sourceNormals,
	const std::vector<float>& targetXyz,
	const std::vector<float>& targetNormals,
	const SpareRegisterParams& params,
	std::string* errMsg)
{
	if (params.coarseGlobalAlign)
	{
		RigidRegisterRansacParams ransacParams;
		ransacParams.refineWithIcp = true;
		Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
		double inlierRatio = 0.0;
		if (!rigidRegisterFeatureRansac(
				sourceXyz,
				sourceNormals,
				targetXyz,
				targetNormals,
				transform,
				&inlierRatio,
				ransacParams,
				errMsg))
		{
			return false;
		}
		transformXyzInPlace(sourceXyz, transform);
		transformNormalsInPlace(sourceNormals, transform);
		return true;
	}

	if (!params.rigidPreAlign)
	{
		return true;
	}

	Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
	double rmse = 0.0;
	if (!rigidRegisterPointToPlaneIcp(
			sourceXyz,
			sourceNormals,
			targetXyz,
			targetNormals,
			transform,
			&rmse,
			params.rigidPreAlignMaxIterations,
			0.01,
			params.rigidPreAlignMaxPairDistanceMm,
			params.rigidPreAlignMaxPoints,
			errMsg))
	{
		return false;
	}
	transformXyzInPlace(sourceXyz, transform);
	transformNormalsInPlace(sourceNormals, transform);
	return true;
}

bool resolveSampleRadius(
	const spare::SpareSurface& source,
	const SpareRegisterParams& params,
	spare::SpareInternalParams& internal)
{
	if (params.sampleRadiusRatio > 0.0)
	{
		internal.uniSampleRatio = params.sampleRadiusRatio;
		return true;
	}

	std::vector<float> xyz;
	std::vector<float> nrm;
	spare::spareSurfaceToXyz(source, xyz, nrm);
	const double spacing = computeAverageSpacingMm(xyz, 6U);
	if (spacing <= 1e-9)
	{
		return false;
	}
	internal.uniSampleRatio = spacing * 10.0;
	return true;
}

bool runSpareCore(
	spare::SpareSurface& source,
	spare::SpareSurface& target,
	const SpareRegisterParams& params,
	SpareRegisterResult* stats,
	std::string* errMsg)
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
	}
	return true;
}

} // namespace

bool spareRegisterPointClouds(
	const std::vector<float>& sourceXyz,
	const std::vector<float>& sourceNormals,
	const std::vector<float>& targetXyz,
	const std::vector<float>& targetNormals,
	std::vector<float>& sourceXyzDeformedOut,
	std::vector<float>& sourceNormalsDeformedOut,
	const SpareRegisterParams& params,
	SpareRegisterResult* stats,
	std::string* errMsg)
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

	if (!applyPreAlign(srcXyz, srcNormals, tgtXyz, tgtNormals, params, errMsg))
	{
		return false;
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

	spare::spareSurfaceToXyz(source, sourceXyzDeformedOut, sourceNormalsDeformedOut);
	return true;
}

bool spareRegisterMeshSoupToTarget(
	const std::vector<float>& sourceSoup,
	const std::vector<float>& targetXyz,
	const std::vector<float>& targetNormals,
	std::vector<float>& sourceSoupDeformedOut,
	const SpareRegisterParams& params,
	SpareRegisterResult* stats,
	std::string* errMsg)
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
		if (!copyNormalsOrEstimate(srcWork, {}, srcWorkN, errMsg)
			|| !copyNormalsOrEstimate(tgtWork, {}, tgtWorkN, errMsg))
		{
			return false;
		}
		const std::vector<float> srcWorkBefore = srcWork;
		std::vector<float> srcWorkDef;
		std::vector<float> srcWorkDefN;
		SpareRegisterParams pcParams = params;
		pcParams.voxelPrefilterMm = 0.0;
		if (!spareRegisterPointClouds(
				srcWork,
				srcWorkN,
				tgtWork,
				tgtWorkN,
				srcWorkDef,
				srcWorkDefN,
				pcParams,
				stats,
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
			const std::size_t nn = tree.findNearest(
				static_cast<double>(srcXyz[b]),
				static_cast<double>(srcXyz[b + 1U]),
				static_cast<double>(srcXyz[b + 2U]),
				maxDistSq,
				distSq);
			if (nn == static_cast<std::size_t>(-1) || nn * 3U + 2U >= srcWorkDef.size())
			{
				continue;
			}
			const std::size_t nb = nn * 3U;
			const float dx = srcWorkDef[nb] - srcWorkBefore[nb];
			const float dy = srcWorkDef[nb + 1U] - srcWorkBefore[nb + 1U];
			const float dz = srcWorkDef[nb + 2U] - srcWorkBefore[nb + 2U];
			source.positions[i] = spare::Vector3(
				srcXyz[b] + dx,
				srcXyz[b + 1U] + dy,
				srcXyz[b + 2U] + dz);
			if (nb + 2U < srcWorkDefN.size())
			{
				source.normals[i] = spare::Vector3(
					srcWorkDefN[nb],
					srcWorkDefN[nb + 1U],
					srcWorkDefN[nb + 2U]);
			}
		}
		spare::spareSurfaceToMeshSoup(source, sourceSoup, sourceSoupDeformedOut);
		return true;
	}

	if (!applyPreAlign(srcXyz, srcNormals, tgtXyz, tgtNormals, params, errMsg))
	{
		return false;
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

	spare::spareSurfaceToMeshSoup(source, sourceSoup, sourceSoupDeformedOut);
	return true;
}

bool spareRegisterMeshSoupToMeshSoup(
	const std::vector<float>& sourceSoup,
	const std::vector<float>& targetSoup,
	std::vector<float>& sourceSoupDeformedOut,
	const SpareRegisterParams& params,
	SpareRegisterResult* stats,
	std::string* errMsg)
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
	return spareRegisterMeshSoupToTarget(
		sourceSoup,
		tgtXyz,
		tgtNormals,
		sourceSoupDeformedOut,
		params,
		stats,
		errMsg);
}

} // namespace pclalgo
