/// @file Preprocess.cpp
/// @brief Preprocess 实现

#include "Preprocess.h"

#include "Downsample.h"
#include "PointCloudBuffer.h"

#include <algorithm>
#include <map>
#include <utility>

#include <CGAL/IO/io.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/remove_outliers.h>

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

namespace pclalgo
{
namespace
{
using Kernel = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Point_with_normal = std::pair<Point_3, Vector_3>;

struct PointCompare
{
	bool operator()(const Point_3& a, const Point_3& b) const
	{
		if (a.x() != b.x())
			return a.x() < b.x();
		if (a.y() != b.y())
			return a.y() < b.y();
		return a.z() < b.z();
	}
};

bool buildPointNormal(const std::vector<float>& xyz, std::vector<Point_with_normal>& points, std::string* errMsg)
{
	if (!validXyzLength(xyz) || xyz.empty())
	{
		if (errMsg != nullptr)
		{
			*errMsg = "empty or invalid xyz";
		}
		return false;
	}
	points.clear();
	points.reserve(pointCountFromXyz(xyz));
	for (std::size_t i = 0; i < pointCountFromXyz(xyz); ++i)
	{
		const std::size_t b = i * 3U;
		points.emplace_back(Point_3(xyz[b], xyz[b + 1U], xyz[b + 2U]), Vector_3(0.0, 0.0, 1.0));
	}
	return true;
}

void exportPointNormal(const std::vector<Point_with_normal>& points, std::vector<float>& xyz,
					   std::vector<float>& normals)
{
	xyz.resize(points.size() * 3U);
	normals.resize(points.size() * 3U);
	for (std::size_t i = 0; i < points.size(); ++i)
	{
		xyz[i * 3U] = static_cast<float>(points[i].first.x());
		xyz[i * 3U + 1U] = static_cast<float>(points[i].first.y());
		xyz[i * 3U + 2U] = static_cast<float>(points[i].first.z());
		normals[i * 3U] = static_cast<float>(points[i].second.x());
		normals[i * 3U + 1U] = static_cast<float>(points[i].second.y());
		normals[i * 3U + 2U] = static_cast<float>(points[i].second.z());
	}
}

void syncRgbaAfterErase(const std::vector<Point_with_normal>& before, const std::vector<Point_with_normal>& after,
						std::vector<float>& rgba)
{
	if (rgba.empty())
	{
		return;
	}

	// 构建索引映射 O(n log n)
	std::map<Point_3, std::size_t, PointCompare> indexMap;
	for (std::size_t i = 0; i < before.size(); ++i)
	{
		indexMap[before[i].first] = i;
	}

	// 使用映射查找 O(n log n)
	std::vector<float> newRgba;
	newRgba.reserve(after.size() * 4U);
	for (const Point_with_normal& pn : after)
	{
		const auto it = indexMap.find(pn.first);
		if (it == indexMap.end())
		{
			continue;
		}
		const std::size_t idx = it->second;
		const std::size_t b = idx * 4U;
		if (b + 3U < rgba.size())
		{
			newRgba.push_back(rgba[b]);
			newRgba.push_back(rgba[b + 1U]);
			newRgba.push_back(rgba[b + 2U]);
			newRgba.push_back(rgba[b + 3U]);
		}
	}
	rgba = std::move(newRgba);
}

} // namespace

bool estimateNormalsPca(const std::vector<float>& xyz, std::vector<float>& normalsOut, const unsigned int kNeighbors,
						std::string* errMsg)
{
	std::vector<Point_with_normal> points;
	if (!buildPointNormal(xyz, points, errMsg))
	{
		return false;
	}

	const unsigned int k = (std::max)(3U, kNeighbors);
#ifdef CGAL_LINKED_WITH_TBB
	CGAL::pca_estimate_normals<CGAL::Parallel_tag>(
#else
	CGAL::pca_estimate_normals<CGAL::Sequential_tag>(
#endif
		points, k,
		CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Point_with_normal>())
			.normal_map(CGAL::Second_of_pair_property_map<Point_with_normal>()));

	std::vector<float> xyzCopy;
	exportPointNormal(points, xyzCopy, normalsOut);
	return true;
}

bool estimateNormalsJet(const std::vector<float>& xyz, std::vector<float>& normalsOut, const unsigned int kNeighbors,
						const unsigned int degreeFitting, std::string* errMsg)
{
	std::vector<Point_with_normal> points;
	if (!buildPointNormal(xyz, points, errMsg))
	{
		return false;
	}

	const unsigned int k = (std::max)(3U, kNeighbors);
#ifdef CGAL_LINKED_WITH_TBB
	CGAL::jet_estimate_normals<CGAL::Parallel_tag>(
#else
	CGAL::jet_estimate_normals<CGAL::Sequential_tag>(
#endif
		points, k,
		CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Point_with_normal>())
			.normal_map(CGAL::Second_of_pair_property_map<Point_with_normal>())
			.degree_fitting(degreeFitting));

	std::vector<float> xyzCopy;
	exportPointNormal(points, xyzCopy, normalsOut);
	return true;
}

bool orientNormalsMst(std::vector<float>& xyz, std::vector<float>& normalsInOut, const unsigned int kNeighbors,
					  std::vector<float>* rgbaInOut, std::string* errMsg)
{
	if (!validXyzLength(xyz) || normalsInOut.size() != xyz.size())
	{
		if (errMsg != nullptr)
		{
			*errMsg = "xyz/normals size mismatch";
		}
		return false;
	}

	std::vector<Point_with_normal> points;
	points.reserve(pointCountFromXyz(xyz));
	for (std::size_t i = 0; i < pointCountFromXyz(xyz); ++i)
	{
		const std::size_t b = i * 3U;
		points.emplace_back(Point_3(xyz[b], xyz[b + 1U], xyz[b + 2U]),
							Vector_3(normalsInOut[b], normalsInOut[b + 1U], normalsInOut[b + 2U]));
	}

	const std::vector<Point_with_normal> before = points;
	const unsigned int k = (std::max)(3U, kNeighbors);
	const auto endIt =
		CGAL::mst_orient_normals(points, k,
								 CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Point_with_normal>())
									 .normal_map(CGAL::Second_of_pair_property_map<Point_with_normal>()));
	points.erase(endIt, points.end());

	if (rgbaInOut != nullptr && !rgbaInOut->empty())
	{
		syncRgbaAfterErase(before, points, *rgbaInOut);
	}

	exportPointNormal(points, xyz, normalsInOut);
	return true;
}

bool removeOutliers(std::vector<float>& xyzInOut, const double removalPercent, const unsigned int kNeighbors,
					std::vector<float>* normalsInOut, std::vector<float>* rgbaInOut, std::string* errMsg)
{
	std::vector<Point_with_normal> points;
	if (!buildPointNormal(xyzInOut, points, errMsg))
	{
		return false;
	}

	if (normalsInOut != nullptr && normalsInOut->size() == xyzInOut.size())
	{
		for (std::size_t i = 0; i < points.size(); ++i)
		{
			const std::size_t b = i * 3U;
			points[i].second = Vector_3((*normalsInOut)[b], (*normalsInOut)[b + 1U], (*normalsInOut)[b + 2U]);
		}
	}

	const std::vector<Point_with_normal> before = points;
#ifdef CGAL_LINKED_WITH_TBB
	const auto endIt = CGAL::remove_outliers<CGAL::Parallel_tag>(
#else
	const auto endIt = CGAL::remove_outliers<CGAL::Sequential_tag>(
#endif
		points, kNeighbors,
		CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Point_with_normal>())
			.threshold_percent(removalPercent));
	points.erase(endIt, points.end());

	if (rgbaInOut != nullptr && !rgbaInOut->empty())
	{
		syncRgbaAfterErase(before, points, *rgbaInOut);
	}

	std::vector<float> dummyNormals;
	if (normalsInOut != nullptr)
	{
		exportPointNormal(points, xyzInOut, *normalsInOut);
	}
	else
	{
		exportPointNormal(points, xyzInOut, dummyNormals);
	}
	return true;
}

bool smoothBilateral(std::vector<float>& xyzInOut, std::vector<float>* normalsInOut, std::string* errMsg)
{
	std::vector<Point_with_normal> points;
	if (!buildPointNormal(xyzInOut, points, errMsg))
	{
		return false;
	}

	if (normalsInOut != nullptr && normalsInOut->size() == xyzInOut.size())
	{
		for (std::size_t i = 0; i < points.size(); ++i)
		{
			const std::size_t b = i * 3U;
			points[i].second = Vector_3((*normalsInOut)[b], (*normalsInOut)[b + 1U], (*normalsInOut)[b + 2U]);
		}
	}

#ifdef CGAL_LINKED_WITH_TBB
	CGAL::bilateral_smooth_point_set<CGAL::Parallel_tag>(
#else
	CGAL::bilateral_smooth_point_set<CGAL::Sequential_tag>(
#endif
		points, 12U,
		CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Point_with_normal>())
			.normal_map(CGAL::Second_of_pair_property_map<Point_with_normal>()));

	if (normalsInOut != nullptr)
	{
		exportPointNormal(points, xyzInOut, *normalsInOut);
	}
	else
	{
		std::vector<float> dummy;
		exportPointNormal(points, xyzInOut, dummy);
	}
	return true;
}

bool preprocessForReconstruction(std::vector<float>& xyzInOut, std::vector<float>& normalsOut,
								 const double voxelPrefilterMm, const double outlierRemovalPercent, std::string* errMsg)
{
	if (voxelPrefilterMm > 0.0)
	{
		if (!downsampleVoxelGrid(xyzInOut, voxelPrefilterMm))
		{
			if (errMsg != nullptr)
			{
				*errMsg = "voxel prefilter failed";
			}
			return false;
		}
	}

	if (outlierRemovalPercent > 0.0)
	{
		if (!removeOutliers(xyzInOut, outlierRemovalPercent, 24, nullptr, nullptr, errMsg))
		{
			return false;
		}
	}

	if (!estimateNormalsPca(xyzInOut, normalsOut, 12, errMsg))
	{
		return false;
	}

	return orientNormalsMst(xyzInOut, normalsOut, 12, nullptr, errMsg);
}

} // namespace pclalgo
