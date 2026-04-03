#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "BackendGeometryMetrics.h"

#include <algorithm>
#include <cmath>

namespace backend_geometry_metrics {

osg::Vec3f pointCloudCenterFromXyz(const std::vector<float>& xyz)
{
	if (xyz.size() < 3U || (xyz.size() % 3U) != 0U)
	{
		return osg::Vec3f(0.0f, 0.0f, 0.0f);
	}
	float minx = xyz[0], maxx = xyz[0];
	float miny = xyz[1], maxy = xyz[1];
	float minz = xyz[2], maxz = xyz[2];
	for (std::size_t i = 0; i + 2 < xyz.size(); i += 3U)
	{
		const float x = xyz[i], y = xyz[i + 1], z = xyz[i + 2];
		minx = std::min(minx, x);
		maxx = std::max(maxx, x);
		miny = std::min(miny, y);
		maxy = std::max(maxy, y);
		minz = std::min(minz, z);
		maxz = std::max(maxz, z);
	}
	return osg::Vec3f(0.5f * (minx + maxx), 0.5f * (miny + maxy), 0.5f * (minz + maxz));
}

float pointCloudDiagonalFromXyz(const std::vector<float>& xyz)
{
	if (xyz.size() < 3U || (xyz.size() % 3U) != 0U)
	{
		return 1.0f;
	}
	float minx = xyz[0], maxx = xyz[0];
	float miny = xyz[1], maxy = xyz[1];
	float minz = xyz[2], maxz = xyz[2];
	for (std::size_t i = 0; i + 2 < xyz.size(); i += 3U)
	{
		const float x = xyz[i], y = xyz[i + 1], z = xyz[i + 2];
		minx = std::min(minx, x);
		maxx = std::max(maxx, x);
		miny = std::min(miny, y);
		maxy = std::max(maxy, y);
		minz = std::min(minz, z);
		maxz = std::max(maxz, z);
	}
	const float dx = maxx - minx;
	const float dy = maxy - miny;
	const float dz = maxz - minz;
	return std::max(1.0f, std::sqrt(dx * dx + dy * dy + dz * dz));
}

osg::Vec3f meshCenterFromSoup(const std::vector<float>& soup)
{
	if (soup.size() < 3U || (soup.size() % 3U) != 0U)
	{
		return osg::Vec3f(0.0f, 0.0f, 0.0f);
	}
	float minx = soup[0], maxx = soup[0], miny = soup[1], maxy = soup[1], minz = soup[2], maxz = soup[2];
	for (std::size_t i = 0; i + 2 < soup.size(); i += 3U)
	{
		const float x = soup[i], y = soup[i + 1], z = soup[i + 2];
		minx = std::min(minx, x);
		maxx = std::max(maxx, x);
		miny = std::min(miny, y);
		maxy = std::max(maxy, y);
		minz = std::min(minz, z);
		maxz = std::max(maxz, z);
	}
	return osg::Vec3f(0.5f * (minx + maxx), 0.5f * (miny + maxy), 0.5f * (minz + maxz));
}

float meshDiagonalFromSoup(const std::vector<float>& soup)
{
	if (soup.size() < 3U || (soup.size() % 3U) != 0U)
	{
		return 1.0f;
	}
	float minx = soup[0], maxx = soup[0], miny = soup[1], maxy = soup[1], minz = soup[2], maxz = soup[2];
	for (std::size_t i = 0; i + 2 < soup.size(); i += 3U)
	{
		const float x = soup[i], y = soup[i + 1], z = soup[i + 2];
		minx = std::min(minx, x);
		maxx = std::max(maxx, x);
		miny = std::min(miny, y);
		maxy = std::max(maxy, y);
		minz = std::min(minz, z);
		maxz = std::max(maxz, z);
	}
	const float dx = maxx - minx;
	const float dy = maxy - miny;
	const float dz = maxz - minz;
	return std::max(1.0f, std::sqrt(dx * dx + dy * dy + dz * dz));
}

} // namespace backend_geometry_metrics
