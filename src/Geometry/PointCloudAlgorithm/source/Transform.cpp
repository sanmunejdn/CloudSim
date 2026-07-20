/// @file Transform.cpp
/// @brief Transform 实现

#include "Transform.h"

#include "PointCloudBuffer.h"

namespace pclalgo
{
void transformXyzInPlace(std::vector<float>& xyz, const Eigen::Isometry3d& t)
{
	if (!validXyzLength(xyz))
	{
		return;
	}
	const std::size_t n = pointCountFromXyz(xyz);
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		Eigen::Vector3d p(xyz[b], xyz[b + 1U], xyz[b + 2U]);
		p = t * p;
		xyz[b] = static_cast<float>(p.x());
		xyz[b + 1U] = static_cast<float>(p.y());
		xyz[b + 2U] = static_cast<float>(p.z());
	}
}

void transformXyz(const float* srcXyz, const std::size_t pointCount, const Eigen::Isometry3d& t,
				  std::vector<float>& outXyz)
{
	outXyz.resize(pointCount * 3U);
	for (std::size_t i = 0; i < pointCount; ++i)
	{
		const std::size_t b = i * 3U;
		Eigen::Vector3d p(srcXyz[b], srcXyz[b + 1U], srcXyz[b + 2U]);
		p = t * p;
		outXyz[b] = static_cast<float>(p.x());
		outXyz[b + 1U] = static_cast<float>(p.y());
		outXyz[b + 2U] = static_cast<float>(p.z());
	}
}

} // namespace pclalgo
