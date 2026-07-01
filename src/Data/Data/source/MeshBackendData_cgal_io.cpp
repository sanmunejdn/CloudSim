#include "pch.h"
#include "MeshBackendData_loaders.h"
#include "MeshBackendData.h"
#include "BackendSpatial.h"
#include "RunLogger.h"

#include <filesystem>

using namespace mesh_backend_load;

bool MeshBackendData::loadFromFile(const std::string& path, std::string* errMsg, const int meshImportQuality)
{
	clearGeometry();
	const std::string ext = meshLowerExtension(path);
	if (ext.empty())
	{
		meshLoadErr(errMsg, "Missing file extension.");
		return false;
	}

	// STEP：OCCT 读入并离散为三角 soup
	if (ext == "step" || ext == "stp")
	{
		std::vector<float> soup;
		if (!meshLoadStepSingleFile(path, soup, errMsg))
		{
			return false;
		}
		setTriangleSoup(std::move(soup));
		RunLogger::info("[MeshBackendData] STEP mesh loaded successfully.");
		return !m_triangleSoup.empty();
	}

	if (ext == "dxf")
	{
		std::vector<float> soup;
		if (!meshLoadDxfSingleFile(path, soup, errMsg))
		{
			return false;
		}
		setTriangleSoup(std::move(soup));
		RunLogger::info("[MeshBackendData] DXF mesh loaded successfully.");
		return true;
	}

	// OBJ 保留 vn 供光照（CGAL soup 会丢 vn）
	if (ext == "obj")
	{
		std::vector<float> objSoup;
		std::vector<float> objNormals;
		if (meshTryLoadObjWithVertexNormals(path, objSoup, objNormals))
		{
			if (meshImportQuality == 0 && objNormals.size() == objSoup.size())
			{
				meshApplyImportQualityToSoup(objNormals, meshImportQuality);
			}
			meshApplyImportQualityToSoup(objSoup, meshImportQuality);
			setTriangleSoupWithNormals(std::move(objSoup), std::move(objNormals));
			RunLogger::info("[MeshBackendData] OBJ loaded with file vertex normals for lighting.");
			return !m_triangleSoup.empty();
		}
	}

	return meshLoadCgalMeshFile(*this, path, ext, errMsg, meshImportQuality);
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
		points.emplace_back(
			static_cast<double>(m_triangleSoup[base]),
			static_cast<double>(m_triangleSoup[base + 1U]),
			static_cast<double>(m_triangleSoup[base + 2U]));
		points.emplace_back(
			static_cast<double>(m_triangleSoup[base + 3U]),
			static_cast<double>(m_triangleSoup[base + 4U]),
			static_cast<double>(m_triangleSoup[base + 5U]));
		points.emplace_back(
			static_cast<double>(m_triangleSoup[base + 6U]),
			static_cast<double>(m_triangleSoup[base + 7U]),
			static_cast<double>(m_triangleSoup[base + 8U]));
		polygons.push_back({vBase, vBase + 1U, vBase + 2U});
	}

	const std::filesystem::path outPath = std::filesystem::u8path(utf8Path);
	if (!CGAL::IO::write_polygon_soup(outPath.string(), points, polygons))
	{
		meshLoadErr(errMsg, "Failed to write mesh PLY.");
		return false;
	}
	RunLogger::info("[MeshBackendData] Triangle mesh PLY exported successfully.");
	return true;
}

bool MeshBackendData::writeTriangleMeshPly(const std::string& utf8Path, const std::vector<float>& soupOverride, std::string* errMsg) const
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
		points.emplace_back(
			static_cast<double>(soupOverride[base]),
			static_cast<double>(soupOverride[base + 1U]),
			static_cast<double>(soupOverride[base + 2U]));
		points.emplace_back(
			static_cast<double>(soupOverride[base + 3U]),
			static_cast<double>(soupOverride[base + 4U]),
			static_cast<double>(soupOverride[base + 5U]));
		points.emplace_back(
			static_cast<double>(soupOverride[base + 6U]),
			static_cast<double>(soupOverride[base + 7U]),
			static_cast<double>(soupOverride[base + 8U]));
		polygons.push_back({vBase, vBase + 1U, vBase + 2U});
	}

	const std::filesystem::path outPath = std::filesystem::u8path(utf8Path);
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
