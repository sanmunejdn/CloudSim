/// @file BackendImporters.cpp
/// @brief 后端几何文件导入自由函数（CGAL/OCCT IO 实现落点）

#include "BackendImporters.h"

#include "BrepBackendData.h"
#include "MeshBackendData.h"
#include "MeshBackendData_loaders.h"
#include "PlyIo.h"
#include "PointCloudBackendData.h"
#include "RunLogger.h"

#include <Discretize.h>
#include <ShapeHandle.h>
#include <ShapeIo.h>
#include <Types.h>

#include <cctype>
#include <fstream>
#include <sstream>

using namespace mesh_backend_load;

namespace
{
void setErr(std::string* errMsg, const std::string& text)
{
	if (errMsg)
	{
		*errMsg = text;
	}
}

bool readPointCloudFromXyzFilePath(PointCloudBackendData& pc, const std::string& nativePath, std::string* errMsg)
{
	std::ifstream in(nativePath);
	if (!in)
	{
		setErr(errMsg, "Cannot open XYZ file.");
		return false;
	}
	std::vector<float> xyz;
	std::string line;
	std::size_t badLines = 0;
	while (std::getline(in, line))
	{
		std::size_t i = 0;
		while (i < line.size() && (line[i] == ' ' || line[i] == '\t' || line[i] == '\r'))
		{
			++i;
		}
		if (i >= line.size() || line[i] == '#')
		{
			continue;
		}
		std::istringstream ls(line.substr(i));
		double vx = 0.0;
		double vy = 0.0;
		double vz = 0.0;
		if (ls >> vx >> vy >> vz)
		{
			xyz.push_back(static_cast<float>(vx));
			xyz.push_back(static_cast<float>(vy));
			xyz.push_back(static_cast<float>(vz));
		}
		else
		{
			// P3-3: 坏行计数，结尾统一告警
			++badLines;
		}
	}
	if (badLines > 0)
	{
		RunLogger::warn("[backend_io] XYZ: " + std::to_string(badLines) + " bad line(s) skipped.");
	}
	if (xyz.empty())
	{
		setErr(errMsg, "No valid XYZ points.");
		return false;
	}
	pc.setPointBuffers(std::move(xyz), {});
	return true;
}
} // namespace

namespace backend_io
{
bool loadPointCloudFromFile(PointCloudBackendData& data, const std::string& path, std::string* errMsg)
{
	// 两阶段：失败不动原有几何（勿先 clearGeometry）
	const auto dot = path.find_last_of('.');
	if (dot == std::string::npos)
	{
		setErr(errMsg, "Missing file extension.");
		return false;
	}
	std::string ext = path.substr(dot + 1);
	for (char& c : ext)
	{
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	}
	if (ext == "ply")
	{
		if (plyFileHasTriangleFaces(path))
		{
			setErr(errMsg, "PLY contains faces; import as mesh instead of point cloud.");
			return false;
		}
		return data.readPointCloudFromPlyFile(path, errMsg);
	}
	if (ext == "xyz")
	{
		return readPointCloudFromXyzFilePath(data, path, errMsg);
	}
	if (ext == "las" || ext == "laz")
	{
		setErr(errMsg, "LAS/LAZ: use OSG import path or convert to PLY/XYZ.");
		return false;
	}
	setErr(errMsg, "Unsupported point cloud extension for CGAL backend load.");
	return false;
}

bool loadMeshFromFile(MeshBackendData& mesh, const std::string& path, std::string* errMsg, const int meshImportQuality)
{
	// 两阶段：解析到临时缓冲，成功后才置换；失败保留原几何
	const std::string ext = meshLowerExtension(path);
	if (ext.empty())
	{
		meshLoadErr(errMsg, "Missing file extension.");
		return false;
	}

	if (ext == "step" || ext == "stp")
	{
		std::vector<float> soup;
		if (!meshLoadStepSingleFile(path, soup, errMsg))
		{
			return false;
		}
		// P2: 空判定移到置换前，与 CGAL 路径对齐（纯曲线/点 STEP 不清空原几何）
		if (soup.empty())
		{
			meshLoadErr(errMsg, "STEP tessellation produced empty soup (curves/points only?).");
			return false;
		}
		mesh.setTriangleSoup(std::move(soup));
		RunLogger::info("[backend_io] STEP mesh loaded successfully.");
		return true;
	}

	if (ext == "dxf")
	{
		std::vector<float> soup;
		if (!meshLoadDxfSingleFile(path, soup, errMsg))
		{
			return false;
		}
		// P2: 空判定移到置换前，与 CGAL 路径对齐（空 DXF 不清空原几何）
		if (soup.empty())
		{
			meshLoadErr(errMsg, "DXF produced empty soup (no visible entities?).");
			return false;
		}
		mesh.setTriangleSoup(std::move(soup));
		RunLogger::info("[backend_io] DXF mesh loaded successfully.");
		return true;
	}

	// OBJ 保留 vn 供光照（CGAL soup 会丢 vn）
	if (ext == "obj")
	{
		std::vector<float> objSoup;
		std::vector<float> objNormals;
		if (meshTryLoadObjWithVertexNormals(path, objSoup, objNormals))
		{
			(void)meshImportQuality;
			mesh.setTriangleSoupWithNormals(std::move(objSoup), std::move(objNormals));
			RunLogger::info("[backend_io] OBJ loaded with file vertex normals for lighting.");
			return !mesh.triangleSoup().empty();
		}
	}

	return meshLoadCgalMeshFile(mesh, path, ext, errMsg, meshImportQuality);
}

bool loadBrepFromStepFile(BrepBackendData& brep, const std::string& path, std::string* errMsg)
{
	geoalgo::ShapeHandle shape;
	if (!geoalgo::readStepIntoHandle(path, shape, errMsg))
	{
		return false;
	}
	brep.setShape(std::move(shape));
	return true;
}

bool loadMeshStepHierarchy(const std::string& path, std::vector<MeshHierarchyPart>& outParts, std::string* errMsg)
{
	outParts.clear();
	std::vector<geoalgo::MeshHierarchyPart> parts;
	geoalgo::TessellateParams params;
	params.flipReversedFaces = kMeshStepFlipReversedFaceWinding;
	if (!geoalgo::tessellateStepHierarchy(path, params, parts, errMsg))
	{
		return false;
	}
	outParts.reserve(parts.size());
	for (const geoalgo::MeshHierarchyPart& p : parts)
	{
		MeshHierarchyPart mp;
		mp.partPath = p.partPath;
		mp.parentPartPath = p.parentPartPath;
		mp.displayName = p.displayName;
		mp.triangleSoup = p.triangleSoup;
		outParts.push_back(std::move(mp));
	}
	RunLogger::info("[backend_io] STEP mesh hierarchy loaded successfully.");
	return true;
}

bool loadBrepStepHierarchy(const std::string& path, std::vector<BrepHierarchyPart>& outParts, std::string* errMsg,
						   geoalgo::ShapeHandle* outAssembly)
{
	outParts.clear();
	geoalgo::ShapeHandle assembly;
	if (!geoalgo::readStepIntoHandle(path, assembly, errMsg))
	{
		return false;
	}
	if (outAssembly)
	{
		*outAssembly = assembly;
	}
	std::vector<geoalgo::ShapeHierarchyPart> topParts;
	if (!geoalgo::collectBrepTopLevelShapeParts(assembly, topParts, errMsg))
	{
		return false;
	}
	outParts.reserve(topParts.size());
	for (const geoalgo::ShapeHierarchyPart& sp : topParts)
	{
		BrepHierarchyPart bp;
		bp.partPath = sp.partPath;
		bp.parentPartPath = sp.parentPartPath;
		bp.displayName = sp.displayName;
		bp.shapeRef = sp.shape;
		outParts.push_back(std::move(bp));
	}
	RunLogger::info("[backend_io] STEP brep hierarchy loaded (top-level shape parts).");
	return true;
}
} // namespace backend_io
