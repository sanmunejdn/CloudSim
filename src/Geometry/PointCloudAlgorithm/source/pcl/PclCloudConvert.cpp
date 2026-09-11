/// @file PclCloudConvert.cpp
/// @brief xyz/法线 �?PCL PointNormal

#include "pcl/PclCloudConvert.h"

#ifdef CLOUDSIM_HAS_PCL

#include "PointCloudBuffer.h"

#include <algorithm>
#include <cmath>

namespace pclalgo
{
namespace pcl_bridge
{
CloudNTPtr makeCloudFromXyzNormals(const std::vector<float>& xyz, const std::vector<float>& normalsNxNyNz)
{
	CloudNTPtr cloud(new CloudNT);
	if (!validXyzLength(xyz) || xyz.empty())
	{
		return cloud;
	}
	const std::size_t n = pointCountFromXyz(xyz);
	const bool hasN = normalsNxNyNz.size() == xyz.size();
	cloud->points.resize(n);
	cloud->width = static_cast<std::uint32_t>(n);
	cloud->height = 1;
	cloud->is_dense = false;
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		PointNT& p = cloud->points[i];
		p.x = xyz[b];
		p.y = xyz[b + 1U];
		p.z = xyz[b + 2U];
		if (hasN)
		{
			p.normal_x = normalsNxNyNz[b];
			p.normal_y = normalsNxNyNz[b + 1U];
			p.normal_z = normalsNxNyNz[b + 2U];
		}
		else
		{
			p.normal_x = 0.f;
			p.normal_y = 0.f;
			p.normal_z = 0.f;
		}
	}
	return cloud;
}

bool cloudHasUsableNormals(const CloudNT& cloud)
{
	if (cloud.empty())
	{
		return false;
	}
	std::size_t ok = 0;
	const std::size_t step = (std::max)(std::size_t(1), cloud.size() / 64U);
	for (std::size_t i = 0; i < cloud.size(); i += step)
	{
		const PointNT& p = cloud.points[i];
		const float n2 = p.normal_x * p.normal_x + p.normal_y * p.normal_y + p.normal_z * p.normal_z;
		if (n2 > 1e-8f)
		{
			++ok;
		}
	}
	return ok >= 8U;
}

} // namespace pcl_bridge
} // namespace pclalgo

#endif // CLOUDSIM_HAS_PCL
