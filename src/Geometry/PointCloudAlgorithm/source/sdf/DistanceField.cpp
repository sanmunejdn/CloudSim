#include "sdf/DistanceField.h"

#include "Measure.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>

namespace pclalgo
{
namespace sdf
{
namespace
{

double autoVoxelMm(const std::vector<float>& xyz, double requested)
{
	if (requested > 0.0)
	{
		return requested;
	}
	const double spacing = computeAverageSpacingMm(xyz, 6U);
	if (spacing > 1e-9)
	{
		return spacing;
	}
	return 1.0;
}

} // namespace

bool DistanceField::buildFromPointCloud(const std::vector<float>& xyz, const std::vector<float>& normals,
										double fieldVoxelMm, std::string* errMsg)
{
	if (xyz.size() < 9U || xyz.size() % 3U != 0U)
	{
		if (errMsg)
		{
			*errMsg = "DistanceField: invalid target xyz";
		}
		return false;
	}
	if (normals.size() != xyz.size())
	{
		if (errMsg)
		{
			*errMsg = "DistanceField: normals size mismatch";
		}
		return false;
	}
	xyz_ = xyz;
	normals_ = normals;
	tree_ = std::make_unique<KdTreePointSet>(xyz_);
	if (tree_->empty())
	{
		if (errMsg)
		{
			*errMsg = "DistanceField: empty KdTree";
		}
		return false;
	}
	fieldVoxelMmUsed_ = autoVoxelMm(xyz_, fieldVoxelMm);
	// NN 场在体素上不连续，插值会引入错误梯度；求解一律走精确最近邻
	useVoxel_ = false;
	voxelPhi_.clear();
	voxelDx_.clear();
	voxelDy_.clear();
	voxelDz_.clear();
	voxelNx_.clear();
	voxelNy_.clear();
	voxelNz_.clear();
	return true;
}

bool DistanceField::buildFromMeshSoup(const std::vector<float>& soup, double fieldVoxelMm, std::string* errMsg)
{
	if (soup.size() < 9U || soup.size() % 9U != 0U)
	{
		if (errMsg)
		{
			*errMsg = "DistanceField: invalid mesh soup";
		}
		return false;
	}
	std::vector<float> xyz;
	std::vector<float> normals;
	xyz.reserve(soup.size() / 3U);
	normals.reserve(soup.size() / 3U);
	const std::size_t triCount = soup.size() / 9U;
	for (std::size_t t = 0; t < triCount; ++t)
	{
		const float* p = soup.data() + t * 9U;
		Eigen::Vector3d a(p[0], p[1], p[2]);
		Eigen::Vector3d b(p[3], p[4], p[5]);
		Eigen::Vector3d c(p[6], p[7], p[8]);
		Eigen::Vector3d n = (b - a).cross(c - a);
		const double nlen = n.norm();
		if (nlen < 1e-12)
		{
			continue;
		}
		n /= nlen;
		const Eigen::Vector3d verts[3] = {a, b, c};
		for (const Eigen::Vector3d& v : verts)
		{
			xyz.push_back(static_cast<float>(v.x()));
			xyz.push_back(static_cast<float>(v.y()));
			xyz.push_back(static_cast<float>(v.z()));
			normals.push_back(static_cast<float>(n.x()));
			normals.push_back(static_cast<float>(n.y()));
			normals.push_back(static_cast<float>(n.z()));
		}
	}
	return buildFromPointCloud(xyz, normals, fieldVoxelMm, errMsg);
}

FieldSample DistanceField::queryExact(const Eigen::Vector3d& x) const
{
	FieldSample s;
	if (!tree_ || tree_->empty())
	{
		return s;
	}
	double dist2 = 0.0;
	const std::size_t idx = tree_->findNearest(x.x(), x.y(), x.z(), std::numeric_limits<double>::max(), dist2);
	if (idx == static_cast<std::size_t>(-1))
	{
		return s;
	}
	const std::size_t b = idx * 3U;
	s.closest = Eigen::Vector3d(xyz_[b], xyz_[b + 1U], xyz_[b + 2U]);
	s.normal = Eigen::Vector3d(normals_[b], normals_[b + 1U], normals_[b + 2U]);
	const double nlen = s.normal.norm();
	if (nlen > 1e-12)
	{
		s.normal /= nlen;
	}
	else
	{
		s.normal = Eigen::Vector3d(0, 0, 1);
	}
	// 真 DDF：最近点向量；标量 SDF 为其沿法线分量
	s.directed = x - s.closest;
	s.signedDistance = s.directed.dot(s.normal);
	s.valid = true;
	return s;
}

void DistanceField::rebuildVoxelCache()
{
	useVoxel_ = false;
}

FieldSample DistanceField::queryVoxel(const Eigen::Vector3d& x) const
{
	return queryExact(x);
}

FieldSample DistanceField::query(const Eigen::Vector3d& x) const
{
	return queryExact(x);
}

} // namespace sdf
} // namespace pclalgo
