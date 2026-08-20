#ifndef POINTCLOUDALGORITHM_SDF_DISTANCEFIELD_H
#define POINTCLOUDALGORITHM_SDF_DISTANCEFIELD_H

/// @file DistanceField.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 目标表面有向距离 / 有符号距离场（体素缓存可选）

#include "KdTreePointSet.h"
#include "RegistrationSdf.h"

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

namespace pclalgo
{
namespace sdf
{

struct FieldSample
{
	Eigen::Vector3d closest;   ///< Π(x)
	Eigen::Vector3d directed;  ///< d = x − Π(x) 真有向距离向量
	Eigen::Vector3d normal;	   ///< 目标法线
	double signedDistance = 0; ///< φ = d·n
	bool valid = false;
};

class DistanceField
{
public:
	bool buildFromPointCloud(const std::vector<float>& xyz, const std::vector<float>& normals, double fieldVoxelMm,
							 std::string* errMsg);
	bool buildFromMeshSoup(const std::vector<float>& soup, double fieldVoxelMm, std::string* errMsg);

	FieldSample query(const Eigen::Vector3d& x) const;
	double fieldVoxelMmUsed() const { return fieldVoxelMmUsed_; }

private:
	void rebuildVoxelCache();
	FieldSample queryExact(const Eigen::Vector3d& x) const;
	FieldSample queryVoxel(const Eigen::Vector3d& x) const;

	std::vector<float> xyz_;
	std::vector<float> normals_;
	std::unique_ptr<KdTreePointSet> tree_;

	bool useVoxel_ = false;
	double fieldVoxelMmUsed_ = 0.0;
	Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();
	Eigen::Vector3i dims_ = Eigen::Vector3i::Zero();
	Eigen::Vector3d voxelStep_ = Eigen::Vector3d::Ones();
	std::vector<float> voxelPhi_;
	std::vector<float> voxelDx_;
	std::vector<float> voxelDy_;
	std::vector<float> voxelDz_;
	std::vector<float> voxelNx_;
	std::vector<float> voxelNy_;
	std::vector<float> voxelNz_;
};

} // namespace sdf
} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_SDF_DISTANCEFIELD_H
