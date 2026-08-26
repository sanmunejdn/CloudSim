/// @file MeshBackendData_cgal_io.cpp
/// @brief Mesh 后端数据

#include "pch.h"

#include "BackendImporters.h"
#include "BackendSpatial.h"
#include "MeshBackendData.h"
#include "MeshBackendData_loaders.h"
#include "RunLogger.h"

#include <filesystem>

using namespace mesh_backend_load;

bool MeshBackendData::loadFromFile(const std::string& path, std::string* errMsg, const int meshImportQuality)
{
	if (meshImportQuality == 0)
	{
		// 抽稀已移出导入源路径（B3），该参数不再生效；告警防调用方误以为会抽稀
		RunLogger::warn("[MeshBackendData] meshImportQuality=0 is deprecated and has no effect (no decimation on import).");
	}
	return backend_io::loadMeshFromFile(*this, path, errMsg, meshImportQuality);
}

bool MeshBackendData::writeTriangleMeshPly(const std::string& utf8Path, std::string* errMsg) const
{
	if (m_triangleSoup.empty() || (m_triangleSoup.size() % 9U) != 0U)
	{
		meshLoadErr(errMsg, "No triangle mesh geometry to write.");
		return false;
	}

	using K = CGAL::Simple_cartesian<double>;
	using Point_3 = K::Point_3;
	const std::size_t triCount = m_triangleSoup.size() / 9U;
	std::vector<Point_3> points;
	points.reserve(triCount * 3U);
	std::vector<std::vector<std::size_t>> polygons;
	polygons.reserve(triCount);
	for (std::size_t t = 0; t < triCount; ++t)
	{
		const std::size_t base = t * 9U;
		const std::size_t vBase = t * 3U;
		points.emplace_back(static_cast<double>(m_triangleSoup[base]), static_cast<double>(m_triangleSoup[base + 1U]),
							static_cast<double>(m_triangleSoup[base + 2U]));
		points.emplace_back(static_cast<double>(m_triangleSoup[base + 3U]),
							static_cast<double>(m_triangleSoup[base + 4U]),
							static_cast<double>(m_triangleSoup[base + 5U]));
		points.emplace_back(static_cast<double>(m_triangleSoup[base + 6U]),
							static_cast<double>(m_triangleSoup[base + 7U]),
							static_cast<double>(m_triangleSoup[base + 8U]));
		polygons.push_back({vBase, vBase + 1U, vBase + 2U});
	}

	// §4.0.1 统一约定：本地路径按本地编码构造，禁止 u8path（中文 Windows 非法 UTF-8 序列会抛）
	const std::filesystem::path outPath(utf8Path);
	if (!CGAL::IO::write_polygon_soup(outPath.string(), points, polygons))
	{
		meshLoadErr(errMsg, "Failed to write mesh PLY.");
		return false;
	}
	RunLogger::info("[MeshBackendData] Triangle mesh PLY exported successfully.");
	return true;
}

bool MeshBackendData::writeTriangleMeshPly(const std::string& utf8Path, const std::vector<float>& soupOverride,
										   std::string* errMsg) const
{
	if (soupOverride.empty() || (soupOverride.size() % 9U) != 0U)
	{
		meshLoadErr(errMsg, "No triangle mesh geometry to write.");
		return false;
	}

	using K = CGAL::Simple_cartesian<double>;
	using Point_3 = K::Point_3;
	const std::size_t triCount = soupOverride.size() / 9U;
	std::vector<Point_3> points;
	points.reserve(triCount * 3U);
	std::vector<std::vector<std::size_t>> polygons;
	polygons.reserve(triCount);
	for (std::size_t t = 0; t < triCount; ++t)
	{
		const std::size_t base = t * 9U;
		const std::size_t vBase = t * 3U;
		points.emplace_back(static_cast<double>(soupOverride[base]), static_cast<double>(soupOverride[base + 1U]),
							static_cast<double>(soupOverride[base + 2U]));
		points.emplace_back(static_cast<double>(soupOverride[base + 3U]), static_cast<double>(soupOverride[base + 4U]),
							static_cast<double>(soupOverride[base + 5U]));
		points.emplace_back(static_cast<double>(soupOverride[base + 6U]), static_cast<double>(soupOverride[base + 7U]),
							static_cast<double>(soupOverride[base + 8U]));
		polygons.push_back({vBase, vBase + 1U, vBase + 2U});
	}

	// §4.0.1 统一约定：本地路径按本地编码构造，禁止 u8path（中文 Windows 非法 UTF-8 序列会抛）
	const std::filesystem::path outPath(utf8Path);
	if (!CGAL::IO::write_polygon_soup(outPath.string(), points, polygons))
	{
		meshLoadErr(errMsg, "Failed to write mesh PLY.");
		return false;
	}
	RunLogger::info("[MeshBackendData] Triangle mesh PLY exported successfully.");
	return true;
}

std::vector<float> MeshBackendData::worldTriangleSoup() const
{
	if (m_triangleSoup.empty())
	{
		return {};
	}
	std::vector<float> transformed = m_triangleSoup;
	transformTriangleSoupToWorld(transformed, worldMatrix());
	return transformed;
}
