#include "PointCloudBackendOps.h"

#include "MeshBackendData.h"
#include "PointCloudBackendData.h"

#include "Downsample.h"
#include "Reconstruction.h"
#include "Transform.h"

namespace point_cloud_backend_ops
{

bool downsamplePointCloud(PointCloudBackendData& data, const double voxelSizeMm, std::string* errMsg)
{
	std::vector<float> xyz = data.pointPositionsXyz();
	std::vector<float> rgba = data.pointVertexRgba();
	if (!pclalgo::downsampleVoxelGrid(xyz, voxelSizeMm, 1U, data.hasPerVertexColors() ? &rgba : nullptr))
	{
		if (errMsg != nullptr)
		{
			*errMsg = "downsampleVoxelGrid failed";
		}
		return false;
	}
	data.setPointBuffers(std::move(xyz), std::move(rgba));
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
	data.setPointBuffers(std::move(xyz), std::move(rgba));
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

} // namespace point_cloud_backend_ops
