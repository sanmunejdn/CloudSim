#include "PointCloudBackendOps.h"

#include "MeshBackendData.h"
#include "PointCloudBackendData.h"

#include "Crop.h"
#include "Downsample.h"
#include "Measure.h"
#include "Preprocess.h"
#include "Reconstruction.h"
#include "RegistrationNonRigid.h"
#include "RegistrationRigid.h"
#include "Transform.h"

namespace point_cloud_backend_ops
{

namespace
{

void writeBack(
	PointCloudBackendData& data,
	std::vector<float> xyz,
	std::vector<float> rgba,
	std::vector<float> normals)
{
	data.setPointBuffers(std::move(xyz), std::move(rgba), std::move(normals));
}

std::vector<float>* rgbaPtr(PointCloudBackendData& data, std::vector<float>& rgba)
{
	return data.hasPerVertexColors() ? &rgba : nullptr;
}

std::vector<float>* normalsPtr(PointCloudBackendData& data, std::vector<float>& normals)
{
	return data.hasPointNormals() ? &normals : nullptr;
}

} // namespace

bool downsamplePointCloudVoxel(
	PointCloudBackendData& data,
	const double voxelSizeMm,
	const unsigned int minPointsPerCell,
	std::string* errMsg)
{
	std::vector<float> xyz = data.pointPositionsXyz();
	std::vector<float> rgba = data.pointVertexRgba();
	if (!pclalgo::downsampleVoxelGrid(xyz, voxelSizeMm, minPointsPerCell, rgbaPtr(data, rgba)))
	{
		if (errMsg != nullptr)
		{
			*errMsg = "downsampleVoxelGrid failed";
		}
		return false;
	}
	std::vector<float> clearedNormals;
	writeBack(data, std::move(xyz), std::move(rgba), std::move(clearedNormals));
	return true;
}

bool downsamplePointCloudRandom(PointCloudBackendData& data, const double retainedFraction, std::string* errMsg)
{
	std::vector<float> xyz = data.pointPositionsXyz();
	std::vector<float> rgba = data.pointVertexRgba();
	if (!pclalgo::downsampleRandom(xyz, retainedFraction, rgbaPtr(data, rgba)))
	{
		if (errMsg != nullptr)
		{
			*errMsg = "downsampleRandom failed";
		}
		return false;
	}
	std::vector<float> normals;
	if (data.hasPointNormals())
	{
		normals.clear();
	}
	writeBack(data, std::move(xyz), std::move(rgba), std::move(normals));
	return true;
}

bool applyRigidTransformToPointCloud(
	PointCloudBackendData& data,
	const Eigen::Isometry3d& transform,
	std::string* errMsg)
{
	(void)errMsg;
	std::vector<float> xyz = data.pointPositionsXyz();
	pclalgo::transformXyzInPlace(xyz, transform);
	std::vector<float> rgba = data.pointVertexRgba();
	std::vector<float> normals = data.pointNormalsNxNyNz();
	if (data.hasPointNormals() && !normals.empty())
	{
		const Eigen::Matrix3d rot = transform.linear();
		for (std::size_t i = 0; i + 2 < normals.size(); i += 3)
		{
			const Eigen::Vector3d n(normals[i], normals[i + 1], normals[i + 2]);
			const Eigen::Vector3d out = rot * n;
			normals[i] = static_cast<float>(out.x());
			normals[i + 1] = static_cast<float>(out.y());
			normals[i + 2] = static_cast<float>(out.z());
		}
	}
	writeBack(data, std::move(xyz), std::move(rgba), std::move(normals));
	return true;
}

bool cropPointCloudByBox(PointCloudBackendData& data, const Eigen::AlignedBox3d& box, std::string* errMsg)
{
	(void)errMsg;
	const std::vector<float>& srcXyz = data.pointPositionsXyz();
	std::vector<float> outXyz;
	std::vector<float> outRgba;
	std::vector<float> rgba = data.pointVertexRgba();
	if (data.hasPerVertexColors())
	{
		pclalgo::cropXyzByBox(srcXyz, rgba, box, outXyz, outRgba, nullptr);
	}
	else
	{
		pclalgo::cropXyzByBox(srcXyz, box, outXyz, nullptr);
	}
	std::vector<float> normals;
	writeBack(data, std::move(outXyz), std::move(outRgba), std::move(normals));
	return !data.pointPositionsXyz().empty();
}

bool cropPointCloudBySphere(
	PointCloudBackendData& data,
	const Eigen::Vector3d& centerMm,
	const double radiusMm,
	std::string* errMsg)
{
	(void)errMsg;
	const std::vector<float>& srcXyz = data.pointPositionsXyz();
	std::vector<float> outXyz;
	std::vector<std::size_t> kept;
	pclalgo::cropXyzBySphere(srcXyz, centerMm, radiusMm, outXyz, &kept);
	std::vector<float> outRgba;
	if (data.hasPerVertexColors())
	{
		const std::vector<float>& rgba = data.pointVertexRgba();
		outRgba.reserve(kept.size() * 4U);
		for (const std::size_t idx : kept)
		{
			const std::size_t base = idx * 4U;
			outRgba.push_back(rgba[base]);
			outRgba.push_back(rgba[base + 1U]);
			outRgba.push_back(rgba[base + 2U]);
			outRgba.push_back(rgba[base + 3U]);
		}
	}
	std::vector<float> outNormals;
	if (data.hasPointNormals())
	{
		const std::vector<float>& normals = data.pointNormalsNxNyNz();
		outNormals.reserve(kept.size() * 3U);
		for (const std::size_t idx : kept)
		{
			const std::size_t base = idx * 3U;
			outNormals.push_back(normals[base]);
			outNormals.push_back(normals[base + 1U]);
			outNormals.push_back(normals[base + 2U]);
		}
	}
	writeBack(data, std::move(outXyz), std::move(outRgba), std::move(outNormals));
	return !data.pointPositionsXyz().empty();
}

bool measurePointCloud(const PointCloudBackendData& data, PointCloudMeasureResult& out, std::string* errMsg)
{
	(void)errMsg;
	const std::vector<float>& xyz = data.pointPositionsXyz();
	if (xyz.size() < 3U)
	{
		return false;
	}
	out.boundingBoxMm = pclalgo::computeBoundingBox(xyz);
	out.centroidMm = pclalgo::computeCentroid(xyz);
	out.averageSpacingMm = pclalgo::computeAverageSpacingMm(xyz);
	return true;
}

bool removePointCloudOutliers(
	PointCloudBackendData& data,
	const double removalPercent,
	const unsigned int kNeighbors,
	std::string* errMsg)
{
	std::vector<float> xyz = data.pointPositionsXyz();
	std::vector<float> rgba = data.pointVertexRgba();
	std::vector<float> normals = data.pointNormalsNxNyNz();
	if (!pclalgo::removeOutliers(
			xyz,
			removalPercent,
			kNeighbors,
			normalsPtr(data, normals),
			rgbaPtr(data, rgba),
			errMsg))
	{
		return false;
	}
	writeBack(data, std::move(xyz), std::move(rgba), std::move(normals));
	return true;
}

bool smoothPointCloudBilateral(PointCloudBackendData& data, std::string* errMsg)
{
	std::vector<float> xyz = data.pointPositionsXyz();
	std::vector<float> normals = data.pointNormalsNxNyNz();
	if (!pclalgo::smoothBilateral(xyz, normalsPtr(data, normals), errMsg))
	{
		return false;
	}
	std::vector<float> rgba = data.pointVertexRgba();
	writeBack(data, std::move(xyz), std::move(rgba), std::move(normals));
	return true;
}

bool estimatePointCloudNormalsPca(PointCloudBackendData& data, const unsigned int kNeighbors, std::string* errMsg)
{
	const std::vector<float>& xyz = data.pointPositionsXyz();
	std::vector<float> normals;
	if (!pclalgo::estimateNormalsPca(xyz, normals, kNeighbors, errMsg))
	{
		return false;
	}
	data.setPointNormals(std::move(normals));
	return true;
}

bool estimatePointCloudNormalsJet(
	PointCloudBackendData& data,
	const unsigned int kNeighbors,
	const unsigned int degreeFitting,
	std::string* errMsg)
{
	const std::vector<float>& xyz = data.pointPositionsXyz();
	std::vector<float> normals;
	if (!pclalgo::estimateNormalsJet(xyz, normals, kNeighbors, degreeFitting, errMsg))
	{
		return false;
	}
	data.setPointNormals(std::move(normals));
	return true;
}

bool orientPointCloudNormalsMst(PointCloudBackendData& data, const unsigned int kNeighbors, std::string* errMsg)
{
	std::vector<float> xyz = data.pointPositionsXyz();
	std::vector<float> normals = data.pointNormalsNxNyNz();
	std::vector<float> rgba = data.pointVertexRgba();
	if (!pclalgo::orientNormalsMst(xyz, normals, kNeighbors, rgbaPtr(data, rgba), errMsg))
	{
		return false;
	}
	writeBack(data, std::move(xyz), std::move(rgba), std::move(normals));
	return true;
}

bool preprocessPointCloudForReconstruction(
	PointCloudBackendData& data,
	const double voxelPrefilterMm,
	const double outlierRemovalPercent,
	std::string* errMsg)
{
	std::vector<float> xyz = data.pointPositionsXyz();
	std::vector<float> normals;
	if (!pclalgo::preprocessForReconstruction(xyz, normals, voxelPrefilterMm, outlierRemovalPercent, errMsg))
	{
		return false;
	}
	std::vector<float> rgba;
	if (data.hasPerVertexColors())
	{
		rgba.clear();
	}
	writeBack(data, std::move(xyz), std::move(rgba), std::move(normals));
	return true;
}

bool rigidRegisterPointCloudsIcp(
	const PointCloudBackendData& source,
	const PointCloudBackendData& target,
	PointCloudIcpResult& out,
	const int maxIterations,
	const double convergenceTransMm,
	const double maxPairDistanceMm,
	const std::size_t icpMaxPoints,
	std::string* errMsg)
{
	return pclalgo::rigidRegisterIcp(
		source.pointPositionsXyz(),
		target.pointPositionsXyz(),
		out.sourceToTarget,
		&out.rmseMm,
		maxIterations,
		convergenceTransMm,
		maxPairDistanceMm,
		icpMaxPoints,
		errMsg);
}

bool deformPointCloudTpsFromControls(
	PointCloudBackendData& data,
	const std::vector<std::size_t>& controlPointIndices,
	const std::vector<float>& controlDisplacementXyz,
	const double regularizationLambda,
	std::string* errMsg)
{
	std::vector<float> xyz = data.pointPositionsXyz();
	std::vector<double> displacementDouble(controlDisplacementXyz.begin(), controlDisplacementXyz.end());
	if (!pclalgo::tpsDeformFromControls(
			xyz,
			controlPointIndices,
			displacementDouble.data(),
			displacementDouble.size() / 3U,
			regularizationLambda,
			errMsg))
	{
		return false;
	}
	std::vector<float> rgba = data.pointVertexRgba();
	std::vector<float> normals;
	if (data.hasPointNormals())
	{
		normals.clear();
	}
	writeBack(data, std::move(xyz), std::move(rgba), std::move(normals));
	return true;
}

bool deformPointCloudTpsFitAndDeform(
	const PointCloudBackendData& source,
	const PointCloudBackendData& target,
	const std::vector<std::size_t>& correspondenceIndices,
	std::vector<float>& sourceXyzDeformedOut,
	const double regularizationLambda,
	std::string* errMsg)
{
	return pclalgo::tpsFitAndDeform(
		source.pointPositionsXyz(),
		target.pointPositionsXyz(),
		correspondenceIndices,
		sourceXyzDeformedOut,
		regularizationLambda,
		errMsg);
}

bool reconstructMeshPoisson(
	const PointCloudBackendData& pointCloud,
	MeshBackendData& meshOut,
	const double spacingMm,
	const double smAngleDeg,
	const double smRadiusRel,
	const double smDistanceRel,
	std::string* errMsg)
{
	const std::vector<float>& xyz = pointCloud.pointPositionsXyz();
	const std::vector<float>& normals = pointCloud.pointNormalsNxNyNz();
	if (normals.empty())
	{
		if (errMsg != nullptr)
		{
			*errMsg = "Poisson requires normals";
		}
		return false;
	}
	std::vector<float> soup;
	if (!pclalgo::reconstructPoisson(
			xyz, normals, soup, spacingMm, smAngleDeg, smRadiusRel, smDistanceRel, errMsg))
	{
		return false;
	}
	meshOut.setTriangleSoup(std::move(soup));
	return true;
}

bool reconstructMeshFromPointCloudPoisson(
	const PointCloudBackendData& pointCloud,
	MeshBackendData& meshOut,
	const double voxelPrefilterMm,
	std::string* errMsg)
{
	std::vector<float> xyz = pointCloud.pointPositionsXyz();
	std::vector<float> soup;
	if (!pclalgo::reconstructPoissonAuto(std::move(xyz), soup, voxelPrefilterMm, 5.0, errMsg))
	{
		return false;
	}
	meshOut.setTriangleSoup(std::move(soup));
	return true;
}

bool reconstructMeshScaleSpace(
	const PointCloudBackendData& pointCloud,
	MeshBackendData& meshOut,
	const std::size_t smoothIterations,
	const double meshingRadiusMm,
	std::string* errMsg)
{
	const std::vector<float>& xyz = pointCloud.pointPositionsXyz();
	std::vector<float> soup;
	if (!pclalgo::reconstructScaleSpace(xyz, soup, smoothIterations, meshingRadiusMm, errMsg))
	{
		return false;
	}
	meshOut.setTriangleSoup(std::move(soup));
	return true;
}

} // namespace point_cloud_backend_ops
