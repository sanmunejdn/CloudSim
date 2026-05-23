#include "pch.h"
#include "MeshBackendData_loaders.h"
#include "MeshBackendData.h"
#include "RunLogger.h"

using namespace mesh_backend_load;

bool MeshBackendData::loadFromFile(const std::string& path, std::string* errMsg)
{
	clearGeometry();
	const std::string ext = meshLowerExtension(path);
	if (ext.empty())
	{
		meshLoadErr(errMsg, "Missing file extension.");
		return false;
	}

	// STEP 入口：使用 OCCT 读取并离散化成三角形 soup。
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

	// OBJ：若文件含 vn，保留文件法线供光照（CGAL read_polygon_soup 会丢弃 vn）
	if (ext == "obj")
	{
		std::vector<float> objSoup;
		std::vector<float> objNormals;
		if (meshTryLoadObjWithVertexNormals(path, objSoup, objNormals))
		{
			setTriangleSoupWithNormals(std::move(objSoup), std::move(objNormals));
			RunLogger::info("[MeshBackendData] OBJ loaded with file vertex normals for lighting.");
			return !m_triangleSoup.empty();
		}
	}

	return meshLoadCgalMeshFile(*this, path, ext, errMsg);
}
