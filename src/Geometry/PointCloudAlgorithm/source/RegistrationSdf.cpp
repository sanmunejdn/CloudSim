#include "RegistrationSdf.h"

#include "Downsample.h"
#include "Preprocess.h"
#include "RegistrationRigid.h"
#include "Transform.h"
#include "sdf/DistanceField.h"
#include "sdf/SdfDeformSolver.h"

#include <Eigen/Geometry>

#include <cmath>
#include <map>

namespace pclalgo
{
namespace
{

bool ensureNormals(std::vector<float>& xyz, std::vector<float>& normals, std::string* errMsg)
{
	if (normals.size() == xyz.size() && !normals.empty())
	{
		return true;
	}
	if (!estimateNormalsPca(xyz, normals, 12U, errMsg))
	{
		return false;
	}
	return orientNormalsMst(xyz, normals, 12U, nullptr, errMsg);
}

bool prepareCloud(std::vector<float>& xyz, std::vector<float>& normals, const SdfRegisterParams& params,
				  std::string* errMsg)
{
	if (params.voxelPrefilterMm > 0.0)
	{
		std::vector<float>* nPtr = normals.empty() ? nullptr : &normals;
		if (!downsampleVoxelGrid(xyz, params.voxelPrefilterMm, 1U, nPtr))
		{
			if (errMsg)
			{
				*errMsg = "SDF: voxel prefilter failed";
			}
			return false;
		}
	}
	return ensureNormals(xyz, normals, errMsg);
}

bool maybeRigidPreAlign(std::vector<float>& srcXyz, std::vector<float>& srcNormals, const std::vector<float>& tgtXyz,
						const std::vector<float>& tgtNormals, const SdfRegisterParams& params, std::string* errMsg)
{
	if (!params.rigidPreAlign)
	{
		return true;
	}
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	double rmse = 0.0;
	if (!rigidRegisterPointToPlaneIcp(srcXyz, srcNormals, tgtXyz, tgtNormals, T, &rmse,
									  params.rigidPreAlignMaxIterations, 0.01, params.rigidPreAlignMaxPairDistanceMm,
									  params.rigidPreAlignMaxPoints, errMsg))
	{
		return false;
	}
	transformXyzInPlace(srcXyz, T);
	// 旋转法线
	const Eigen::Matrix3d R = T.linear();
	for (std::size_t i = 0; i + 2U < srcNormals.size(); i += 3U)
	{
		Eigen::Vector3d n(srcNormals[i], srcNormals[i + 1U], srcNormals[i + 2U]);
		n = R * n;
		const double len = n.norm();
		if (len > 1e-12)
		{
			n /= len;
		}
		srcNormals[i] = static_cast<float>(n.x());
		srcNormals[i + 1U] = static_cast<float>(n.y());
		srcNormals[i + 2U] = static_cast<float>(n.z());
	}
	return true;
}

double bboxDiag(const std::vector<float>& xyz)
{
	if (xyz.size() < 3U)
	{
		return 1.0;
	}
	Eigen::AlignedBox3d box;
	for (std::size_t i = 0; i + 2U < xyz.size(); i += 3U)
	{
		box.extend(Eigen::Vector3d(xyz[i], xyz[i + 1U], xyz[i + 2U]));
	}
	return std::max(1e-6, box.diagonal().norm());
}

void scaleCloud(std::vector<float>& xyz, double s)
{
	for (float& v : xyz)
	{
		v = static_cast<float>(static_cast<double>(v) * s);
	}
}

bool soupToUniqueVertices(const std::vector<float>& soup, std::vector<float>& xyz, std::vector<int>& cornerToVert,
						  std::string* errMsg)
{
	if (soup.size() % 9U != 0U || soup.empty())
	{
		if (errMsg)
		{
			*errMsg = "SDF: invalid soup";
		}
		return false;
	}
	xyz.clear();
	cornerToVert.assign(soup.size() / 3U, -1);
	// 简单量化焊点
	struct Key
	{
		int x, y, z;
		bool operator<(const Key& o) const
		{
			if (x != o.x)
				return x < o.x;
			if (y != o.y)
				return y < o.y;
			return z < o.z;
		}
	};
	std::map<Key, int> map;
	const double q = 1e-4;
	int next = 0;
	for (std::size_t c = 0; c < soup.size() / 3U; ++c)
	{
		const double px = soup[c * 3U];
		const double py = soup[c * 3U + 1U];
		const double pz = soup[c * 3U + 2U];
		Key k{static_cast<int>(std::llround(px / q)), static_cast<int>(std::llround(py / q)),
			  static_cast<int>(std::llround(pz / q))};
		auto it = map.find(k);
		if (it == map.end())
		{
			map.emplace(k, next);
			cornerToVert[c] = next;
			xyz.push_back(static_cast<float>(px));
			xyz.push_back(static_cast<float>(py));
			xyz.push_back(static_cast<float>(pz));
			++next;
		}
		else
		{
			cornerToVert[c] = it->second;
		}
	}
	return true;
}

void writeSoupFromVerts(const std::vector<float>& xyz, const std::vector<int>& cornerToVert,
						std::vector<float>& soupOut)
{
	soupOut.resize(cornerToVert.size() * 3U);
	for (std::size_t c = 0; c < cornerToVert.size(); ++c)
	{
		const int vi = cornerToVert[c];
		soupOut[c * 3U] = xyz[static_cast<std::size_t>(vi) * 3U];
		soupOut[c * 3U + 1U] = xyz[static_cast<std::size_t>(vi) * 3U + 1U];
		soupOut[c * 3U + 2U] = xyz[static_cast<std::size_t>(vi) * 3U + 2U];
	}
}

bool runCore(std::vector<float>& srcXyz, std::vector<float>& srcNormals, std::vector<float> tgtXyz,
			 std::vector<float> tgtNormals, const SdfRegisterParams& params, SdfRegisterResult* stats,
			 std::string* errMsg)
{
	if (!prepareCloud(srcXyz, srcNormals, params, errMsg))
	{
		return false;
	}
	if (!prepareCloud(tgtXyz, tgtNormals, params, errMsg))
	{
		return false;
	}
	if (!maybeRigidPreAlign(srcXyz, srcNormals, tgtXyz, tgtNormals, params, errMsg))
	{
		return false;
	}

	double scale = 1.0;
	if (params.normalizeScale)
	{
		scale = 1.0 / bboxDiag(tgtXyz);
		scaleCloud(srcXyz, scale);
		scaleCloud(tgtXyz, scale);
	}

	sdf::DistanceField field;
	SdfRegisterParams p = params;
	if (params.normalizeScale && params.fieldVoxelMm > 0.0)
	{
		p.fieldVoxelMm = params.fieldVoxelMm * scale;
	}
	if (!field.buildFromPointCloud(tgtXyz, tgtNormals, p.fieldVoxelMm, errMsg))
	{
		return false;
	}
	if (!sdf::runSdfDeform(srcXyz, srcNormals, field, p, stats, errMsg))
	{
		return false;
	}
	if (params.normalizeScale)
	{
		scaleCloud(srcXyz, 1.0 / scale);
	}
	if (stats)
	{
		stats->meshScale = scale;
		if (params.normalizeScale)
		{
			stats->meanErrorMm /= scale;
			stats->fieldVoxelMmUsed /= scale;
		}
	}
	return true;
}

} // namespace

bool sdfRegisterPointClouds(const std::vector<float>& sourceXyz, const std::vector<float>& sourceNormals,
							const std::vector<float>& targetXyz, const std::vector<float>& targetNormals,
							std::vector<float>& sourceXyzDeformedOut, std::vector<float>& sourceNormalsDeformedOut,
							const SdfRegisterParams& params, SdfRegisterResult* stats, std::string* errMsg)
{
	sourceXyzDeformedOut = sourceXyz;
	sourceNormalsDeformedOut = sourceNormals;
	return runCore(sourceXyzDeformedOut, sourceNormalsDeformedOut, targetXyz, targetNormals, params, stats, errMsg);
}

bool sdfRegisterMeshSoupToTarget(const std::vector<float>& sourceSoup, const std::vector<float>& targetXyz,
								 const std::vector<float>& targetNormals, std::vector<float>& sourceSoupDeformedOut,
								 const SdfRegisterParams& params, SdfRegisterResult* stats, std::string* errMsg)
{
	std::vector<float> xyz;
	std::vector<int> cornerToVert;
	if (!soupToUniqueVertices(sourceSoup, xyz, cornerToVert, errMsg))
	{
		return false;
	}
	std::vector<float> normals;
	if (!ensureNormals(xyz, normals, errMsg))
	{
		return false;
	}
	if (!runCore(xyz, normals, targetXyz, targetNormals, params, stats, errMsg))
	{
		return false;
	}
	writeSoupFromVerts(xyz, cornerToVert, sourceSoupDeformedOut);
	return true;
}

bool sdfRegisterMeshSoupToMeshSoup(const std::vector<float>& sourceSoup, const std::vector<float>& targetSoup,
								   std::vector<float>& sourceSoupDeformedOut, const SdfRegisterParams& params,
								   SdfRegisterResult* stats, std::string* errMsg)
{
	std::vector<float> tgtXyz;
	std::vector<float> tgtNormals;
	std::vector<int> corners;
	if (!soupToUniqueVertices(targetSoup, tgtXyz, corners, errMsg))
	{
		return false;
	}
	if (!ensureNormals(tgtXyz, tgtNormals, errMsg))
	{
		return false;
	}
	return sdfRegisterMeshSoupToTarget(sourceSoup, tgtXyz, tgtNormals, sourceSoupDeformedOut, params, stats, errMsg);
}

} // namespace pclalgo
