/// @file MeshBackendData_cgal_io.cpp
/// @brief Mesh 后端数据

#include "pch.h"

#include "BackendImporters.h"
#include "BackendSpatial.h"
#include "MeshBackendData.h"
#include "MeshBackendData_loaders.h"
#include "RunLogger.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include <filesystem>
#include <fstream>

using namespace mesh_backend_load;

namespace
{
using PlyK = CGAL::Simple_cartesian<double>;
using PlyPoint_3 = PlyK::Point_3;

std::filesystem::path pathFromUtf8Bytes(const std::string& utf8Path)
{
#ifdef _WIN32
	if (utf8Path.empty())
	{
		return {};
	}
	const int n = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, utf8Path.data(),
									  static_cast<int>(utf8Path.size()), nullptr, 0);
	if (n <= 0)
	{
		return {};
	}
	std::wstring wide(static_cast<std::size_t>(n), L'\0');
	if (MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, utf8Path.data(), static_cast<int>(utf8Path.size()),
							wide.data(), n) <= 0)
	{
		return {};
	}
	return std::filesystem::path(wide);
#else
	try
	{
		return std::filesystem::u8path(utf8Path);
	}
	catch (...)
	{
		return {};
	}
#endif
}

void soupToPlyGeometry(const std::vector<float>& soup, std::vector<PlyPoint_3>& points,
					   std::vector<std::vector<std::size_t>>& polygons)
{
	const std::size_t triCount = soup.size() / 9U;
	points.clear();
	polygons.clear();
	points.reserve(triCount * 3U);
	polygons.reserve(triCount);
	for (std::size_t t = 0; t < triCount; ++t)
	{
		const std::size_t base = t * 9U;
		const std::size_t vBase = t * 3U;
		points.emplace_back(static_cast<double>(soup[base]), static_cast<double>(soup[base + 1U]),
							static_cast<double>(soup[base + 2U]));
		points.emplace_back(static_cast<double>(soup[base + 3U]), static_cast<double>(soup[base + 4U]),
							static_cast<double>(soup[base + 5U]));
		points.emplace_back(static_cast<double>(soup[base + 6U]), static_cast<double>(soup[base + 7U]),
							static_cast<double>(soup[base + 8U]));
		polygons.push_back({vBase, vBase + 1U, vBase + 2U});
	}
}

bool writeSoupPlyFile(const std::string& utf8Path, const std::vector<float>& soup, std::string* errMsg)
{
	if (soup.empty() || (soup.size() % 9U) != 0U)
	{
		meshLoadErr(errMsg, "No triangle mesh geometry to write.");
		return false;
	}

	std::vector<PlyPoint_3> points;
	std::vector<std::vector<std::size_t>> polygons;
	soupToPlyGeometry(soup, points, polygons);

	const std::filesystem::path outPath = pathFromUtf8Bytes(utf8Path);
	if (outPath.empty())
	{
		meshLoadErr(errMsg, "Invalid PLY path encoding.");
		return false;
	}

	// CGAL write_polygon_soup 走 ofstream(string)，Windows 上按 ANSI 建文件，UTF-8 中文名会乱码
	std::ofstream ofs(outPath, std::ios::out | std::ios::binary | std::ios::trunc);
	if (!ofs)
	{
		meshLoadErr(errMsg, "Cannot open file for writing.");
		return false;
	}
	CGAL::IO::set_mode(ofs, CGAL::IO::BINARY);
	if (!CGAL::IO::write_PLY(ofs, points, polygons))
	{
		meshLoadErr(errMsg, "Failed to write mesh PLY.");
		return false;
	}
	RunLogger::info("[MeshBackendData] Triangle mesh PLY exported successfully.");
	return true;
}
} // namespace

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
	return writeSoupPlyFile(utf8Path, m_triangleSoup, errMsg);
}

bool MeshBackendData::writeTriangleMeshPly(const std::string& utf8Path, const std::vector<float>& soupOverride,
										   std::string* errMsg) const
{
	return writeSoupPlyFile(utf8Path, soupOverride, errMsg);
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
