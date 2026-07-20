/// @file Downsample.cpp
/// @brief Downsample 实现

#include "Downsample.h"

#include "PointCloudBuffer.h"

#include <algorithm>
#include <map>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/random_simplify_point_set.h>

namespace pclalgo
{
namespace
{
using CgalKernel = CGAL::Simple_cartesian<double>;
using CgalPoint = CgalKernel::Point_3;

bool syncRgbaAfterPointErase(const std::vector<CgalPoint>& pointsBefore, const std::vector<CgalPoint>& pointsAfter,
							 std::vector<float>& rgba)
{
	if (rgba.empty())
	{
		return true;
	}

	const std::size_t nBefore = pointsBefore.size();
	if (rgba.size() != nBefore * 4U)
	{
		rgba.clear();
		return false;
	}

	// 构建索引映射 O(n log n)
	std::map<CgalPoint, std::size_t> indexMap;
	for (std::size_t i = 0; i < pointsBefore.size(); ++i)
	{
		indexMap[pointsBefore[i]] = i;
	}

	// 使用映射查找 O(n log n)
	std::vector<float> newRgba;
	newRgba.reserve(pointsAfter.size() * 4U);
	for (const CgalPoint& p : pointsAfter)
	{
		const auto it = indexMap.find(p);
		if (it == indexMap.end())
		{
			continue;
		}
		const std::size_t idx = it->second;
		const std::size_t b = idx * 4U;
		newRgba.push_back(rgba[b]);
		newRgba.push_back(rgba[b + 1U]);
		newRgba.push_back(rgba[b + 2U]);
		newRgba.push_back(rgba[b + 3U]);
	}
	rgba = std::move(newRgba);
	return true;
}

} // namespace

bool downsampleVoxelGrid(std::vector<float>& xyzInOut, const double voxelSizeMm, const unsigned int minPointsPerCell,
						 std::vector<float>* rgbaInOut)
{
	if (!validXyzLength(xyzInOut) || voxelSizeMm <= 0.0)
	{
		return false;
	}

	std::vector<CgalPoint> points;
	points.reserve(pointCountFromXyz(xyzInOut));
	for (std::size_t i = 0; i < pointCountFromXyz(xyzInOut); ++i)
	{
		const std::size_t b = i * 3U;
		points.emplace_back(xyzInOut[b], xyzInOut[b + 1U], xyzInOut[b + 2U]);
	}

	const std::vector<CgalPoint> before = points;
	const auto endIt =
		CGAL::grid_simplify_point_set(points, voxelSizeMm, CGAL::parameters::min_points_per_cell(minPointsPerCell));
	points.erase(endIt, points.end());

	if (rgbaInOut != nullptr && !rgbaInOut->empty())
	{
		syncRgbaAfterPointErase(before, points, *rgbaInOut);
	}

	xyzInOut.resize(points.size() * 3U);
	for (std::size_t i = 0; i < points.size(); ++i)
	{
		xyzInOut[i * 3U] = static_cast<float>(points[i].x());
		xyzInOut[i * 3U + 1U] = static_cast<float>(points[i].y());
		xyzInOut[i * 3U + 2U] = static_cast<float>(points[i].z());
	}
	return true;
}

bool downsampleRandom(std::vector<float>& xyzInOut, const double retainedFraction, std::vector<float>* rgbaInOut)
{
	if (!validXyzLength(xyzInOut) || retainedFraction <= 0.0 || retainedFraction > 1.0)
	{
		return false;
	}

	std::vector<CgalPoint> points;
	points.reserve(pointCountFromXyz(xyzInOut));
	for (std::size_t i = 0; i < pointCountFromXyz(xyzInOut); ++i)
	{
		const std::size_t b = i * 3U;
		points.emplace_back(xyzInOut[b], xyzInOut[b + 1U], xyzInOut[b + 2U]);
	}

	const std::vector<CgalPoint> before = points;
	const auto endIt = CGAL::random_simplify_point_set(points, retainedFraction);
	points.erase(endIt, points.end());

	if (rgbaInOut != nullptr && !rgbaInOut->empty())
	{
		syncRgbaAfterPointErase(before, points, *rgbaInOut);
	}

	xyzInOut.resize(points.size() * 3U);
	for (std::size_t i = 0; i < points.size(); ++i)
	{
		xyzInOut[i * 3U] = static_cast<float>(points[i].x());
		xyzInOut[i * 3U + 1U] = static_cast<float>(points[i].y());
		xyzInOut[i * 3U + 2U] = static_cast<float>(points[i].z());
	}
	return true;
}

} // namespace pclalgo
