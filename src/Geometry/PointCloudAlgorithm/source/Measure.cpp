/// @file Measure.cpp
/// @brief Measure 实现

#include "Measure.h"

#include "PointCloudBuffer.h"

#include <limits>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/tags.h>

namespace pclalgo
{
namespace
{
using CgalKernel = CGAL::Simple_cartesian<double>;
using CgalPoint = CgalKernel::Point_3;

} // namespace

Eigen::AlignedBox3d computeBoundingBox(const std::vector<float>& xyz)
{
	Eigen::AlignedBox3d box;
	if (!validXyzLength(xyz) || xyz.empty())
	{
		return box;
	}
	const std::size_t n = pointCountFromXyz(xyz);
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		box.extend(Eigen::Vector3d(xyz[b], xyz[b + 1U], xyz[b + 2U]));
	}
	return box;
}

Eigen::Vector3d computeCentroid(const std::vector<float>& xyz)
{
	if (!validXyzLength(xyz) || xyz.empty())
	{
		return Eigen::Vector3d::Zero();
	}
	Eigen::Vector3d sum = Eigen::Vector3d::Zero();
	const std::size_t n = pointCountFromXyz(xyz);
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		sum += Eigen::Vector3d(xyz[b], xyz[b + 1U], xyz[b + 2U]);
	}
	return sum / static_cast<double>(n);
}

double computeAverageSpacingMm(const std::vector<float>& xyz, const unsigned int kNeighbors)
{
	if (!validXyzLength(xyz) || pointCountFromXyz(xyz) < 2U)
	{
		return 0.0;
	}

	std::vector<CgalPoint> points;
	points.reserve(pointCountFromXyz(xyz));
	for (std::size_t i = 0; i < pointCountFromXyz(xyz); ++i)
	{
		const std::size_t b = i * 3U;
		points.emplace_back(xyz[b], xyz[b + 1U], xyz[b + 2U]);
	}

	const unsigned int k = (std::max)(2U, (std::min)(kNeighbors, static_cast<unsigned int>(points.size() - 1U)));
#ifdef CGAL_LINKED_WITH_TBB
	return CGAL::compute_average_spacing<CGAL::Parallel_tag>(points, k);
#else
	return CGAL::compute_average_spacing<CGAL::Sequential_tag>(points, k);
#endif
}

} // namespace pclalgo
