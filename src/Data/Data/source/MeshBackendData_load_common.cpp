#include "pch.h"
#include "MeshBackendData_loaders.h"
#include "MeshBackendData.h"
#include "RunLogger.h"

namespace mesh_backend_load {

void meshLoadErr(std::string* errMsg, const char* text)
{
	if (errMsg)
	{
		*errMsg = text;
	}
	RunLogger::error(std::string("[MeshBackendData] ") + text);
}

std::string meshLowerExtension(const std::string& path)
{
	const auto dot = path.find_last_of('.');
	if (dot == std::string::npos)
	{
		return {};
	}
	std::string ext = path.substr(dot + 1);
	for (char& c : ext)
	{
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	}
	return ext;
}

template <typename PointRange>
static bool meshComputePointsCentroid(const PointRange& points, double& centX, double& centY, double& centZ)
{
	if (points.empty())
	{
		return false;
	}
	centX = 0.0;
	centY = 0.0;
	centZ = 0.0;
	for (const auto& p : points)
	{
		centX += p.x();
		centY += p.y();
		centZ += p.z();
	}
	const double inv = 1.0 / static_cast<double>(points.size());
	centX *= inv;
	centY *= inv;
	centZ *= inv;
	return true;
}

static bool meshTriangleNormalPointsOutward(double nx, double ny, double nz, double triX, double triY, double triZ,
	double centX, double centY, double centZ)
{
	const double ox = triX - centX;
	const double oy = triY - centY;
	const double oz = triZ - centZ;
	return nx * ox + ny * oy + nz * oz >= 0.0;
}

template <typename PointRange, typename PolygonRange>
static void meshBuildSoupFromPolygons(const PointRange& points, const PolygonRange& polygons, std::vector<float>& soup)
{
	double centX = 0.0;
	double centY = 0.0;
	double centZ = 0.0;
	const bool hasCentroid = meshComputePointsCentroid(points, centX, centY, centZ);
	for (const auto& poly : polygons)
	{
		if (poly.size() < 3U)
		{
			continue;
		}
		for (std::size_t k = 1; k + 1 < poly.size(); ++k)
		{
			const std::size_t i0 = poly[0];
			const std::size_t i1 = poly[k];
			const std::size_t i2 = poly[k + 1];
			if (i0 >= points.size() || i1 >= points.size() || i2 >= points.size())
			{
				continue;
			}
			const auto& p0 = points[i0];
			const auto& p1 = points[i1];
			const auto& p2 = points[i2];
			const double abx = p1.x() - p0.x();
			const double aby = p1.y() - p0.y();
			const double abz = p1.z() - p0.z();
			const double acx = p2.x() - p0.x();
			const double acy = p2.y() - p0.y();
			const double acz = p2.z() - p0.z();
			const double nx = aby * acz - abz * acy;
			const double ny = abz * acx - abx * acz;
			const double nz = abx * acy - aby * acx;
			const double triX = (p0.x() + p1.x() + p2.x()) / 3.0;
			const double triY = (p0.y() + p1.y() + p2.y()) / 3.0;
			const double triZ = (p0.z() + p1.z() + p2.z()) / 3.0;
			const bool flipTri = hasCentroid
				&& !meshTriangleNormalPointsOutward(nx, ny, nz, triX, triY, triZ, centX, centY, centZ);
			if (flipTri)
			{
				meshPushTri(soup, p0.x(), p0.y(), p0.z(), p2.x(), p2.y(), p2.z(), p1.x(), p1.y(), p1.z());
			}
			else
			{
				meshPushTri(soup, p0.x(), p0.y(), p0.z(), p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z());
			}
		}
	}
}

struct MeshObjVec3
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

struct MeshObjCorner
{
	int v = 0;
	int vt = 0;
	int vn = 0;
};

static bool meshParseObjFaceToken(const std::string& token, MeshObjCorner& corner)
{
	corner = {};
	const std::size_t slash1 = token.find('/');
	if (slash1 == std::string::npos)
	{
		if (token.empty())
		{
			return false;
		}
		corner.v = std::stoi(token);
		return true;
	}
	corner.v = std::stoi(token.substr(0, slash1));
	const std::size_t slash2 = token.find('/', slash1 + 1);
	if (slash2 == std::string::npos)
	{
		const std::string vtPart = token.substr(slash1 + 1);
		if (!vtPart.empty())
		{
			corner.vt = std::stoi(vtPart);
		}
		return true;
	}
	const std::string vtPart = token.substr(slash1 + 1, slash2 - slash1 - 1);
	if (!vtPart.empty())
	{
		corner.vt = std::stoi(vtPart);
	}
	const std::string vnPart = token.substr(slash2 + 1);
	if (!vnPart.empty())
	{
		corner.vn = std::stoi(vnPart);
	}
	return true;
}

static std::size_t meshResolveObjIndex(int idx, std::size_t count)
{
	if (idx > 0)
	{
		return static_cast<std::size_t>(idx - 1);
	}
	if (idx < 0)
	{
		const std::size_t back = static_cast<std::size_t>(-idx);
		return back <= count ? count - back : count;
	}
	return count;
}

bool meshTryLoadObjWithVertexNormals(const std::string& path, std::vector<float>& soup, std::vector<float>& normalSoup)
{
	std::ifstream in(path);
	if (!in)
	{
		return false;
	}
	std::vector<MeshObjVec3> vertices;
	std::vector<MeshObjVec3> normals;
	bool hasNormals = false;
	soup.clear();
	normalSoup.clear();

	auto resolveVertex = [&](int idx) -> const MeshObjVec3* {
		const std::size_t i = meshResolveObjIndex(idx, vertices.size());
		return i < vertices.size() ? &vertices[i] : nullptr;
	};
	auto resolveNormal = [&](int idx) -> const MeshObjVec3* {
		if (!hasNormals)
		{
			return nullptr;
		}
		const std::size_t i = meshResolveObjIndex(idx, normals.size());
		return i < normals.size() ? &normals[i] : nullptr;
	};

	auto emitTriangle = [&](const MeshObjCorner& c0, const MeshObjCorner& c1, const MeshObjCorner& c2) {
		const MeshObjVec3* p0 = resolveVertex(c0.v);
		const MeshObjVec3* p1 = resolveVertex(c1.v);
		const MeshObjVec3* p2 = resolveVertex(c2.v);
		if (!p0 || !p1 || !p2)
		{
			return;
		}
		const double abx = p1->x - p0->x;
		const double aby = p1->y - p0->y;
		const double abz = p1->z - p0->z;
		const double acx = p2->x - p0->x;
		const double acy = p2->y - p0->y;
		const double acz = p2->z - p0->z;
		const double nx = aby * acz - abz * acy;
		const double ny = abz * acx - abx * acz;
		const double nz = abx * acy - aby * acx;
		meshPushTri(soup, p0->x, p0->y, p0->z, p1->x, p1->y, p1->z, p2->x, p2->y, p2->z);
		auto pushCornerNormal = [&](const MeshObjCorner& corner) {
			const MeshObjVec3* n = resolveNormal(corner.vn);
			if (n)
			{
				normalSoup.push_back(static_cast<float>(n->x));
				normalSoup.push_back(static_cast<float>(n->y));
				normalSoup.push_back(static_cast<float>(n->z));
				return;
			}
			const double len2 = nx * nx + ny * ny + nz * nz;
			if (len2 < 1e-30)
			{
				normalSoup.push_back(0.0f);
				normalSoup.push_back(0.0f);
				normalSoup.push_back(1.0f);
				return;
			}
			const double invLen = 1.0 / std::sqrt(len2);
			normalSoup.push_back(static_cast<float>(nx * invLen));
			normalSoup.push_back(static_cast<float>(ny * invLen));
			normalSoup.push_back(static_cast<float>(nz * invLen));
		};
		pushCornerNormal(c0);
		pushCornerNormal(c1);
		pushCornerNormal(c2);
	};

	std::string line;
	while (std::getline(in, line))
	{
		if (line.empty())
		{
			continue;
		}
		if (line[0] == '#')
		{
			continue;
		}
		if (line.size() >= 2 && line[0] == 'v' && line[1] == ' ')
		{
			std::istringstream iss(line.substr(2));
			MeshObjVec3 p;
			if (iss >> p.x >> p.y >> p.z)
			{
				vertices.push_back(p);
			}
			continue;
		}
		if (line.size() >= 3 && line[0] == 'v' && line[1] == 'n' && line[2] == ' ')
		{
			std::istringstream iss(line.substr(3));
			MeshObjVec3 n;
			if (iss >> n.x >> n.y >> n.z)
			{
				normals.push_back(n);
				hasNormals = true;
			}
			continue;
		}
		if (line.size() >= 2 && line[0] == 'f' && line[1] == ' ')
		{
			std::istringstream iss(line.substr(2));
			std::vector<MeshObjCorner> corners;
			std::string token;
			while (iss >> token)
			{
				MeshObjCorner corner;
				if (meshParseObjFaceToken(token, corner))
				{
					corners.push_back(corner);
				}
			}
			if (corners.size() == 3U)
			{
				emitTriangle(corners[0], corners[1], corners[2]);
			}
			else if (corners.size() > 3U)
			{
				for (std::size_t k = 1; k + 1 < corners.size(); ++k)
				{
					emitTriangle(corners[0], corners[k], corners[k + 1]);
				}
			}
		}
	}

	if (!hasNormals || soup.empty() || normalSoup.size() != soup.size())
	{
		soup.clear();
		normalSoup.clear();
		return false;
	}
	return true;
}
void meshPushTri(std::vector<float>& soup, double ax, double ay, double az, double bx, double by, double bz,
	double cx, double cy, double cz)
{
	const double abx = bx - ax;
	const double aby = by - ay;
	const double abz = bz - az;
	const double acx = cx - ax;
	const double acy = cy - ay;
	const double acz = cz - az;
	const double nx = aby * acz - abz * acy;
	const double ny = abz * acx - abx * acz;
	const double nz = abx * acy - aby * acx;
	if (nx * nx + ny * ny + nz * nz < 1e-30)
	{
		return;
	}
	soup.push_back(static_cast<float>(ax));
	soup.push_back(static_cast<float>(ay));
	soup.push_back(static_cast<float>(az));
	soup.push_back(static_cast<float>(bx));
	soup.push_back(static_cast<float>(by));
	soup.push_back(static_cast<float>(bz));
	soup.push_back(static_cast<float>(cx));
	soup.push_back(static_cast<float>(cy));
	soup.push_back(static_cast<float>(cz));
}

bool meshLoadCgalMeshFile(MeshBackendData& mesh, const std::string& path, const std::string& ext, std::string* errMsg)
{
	if (!(ext == "obj" || ext == "stl" || ext == "ply" || ext == "off"))
	{
		meshLoadErr(errMsg, "CGAL backend supports .obj .stl .ply .off; other formats use the OSG import path.");
		return false;
	}

	using K = CGAL::Simple_cartesian<double>;
	using Point3 = K::Point_3;
	std::vector<Point3> points;
	std::vector<std::vector<std::size_t>> polygons;
	if (!CGAL::IO::read_polygon_soup(path, points, polygons))
	{
		meshLoadErr(errMsg, "CGAL could not read mesh (polygon soup).");
		return false;
	}
	if (points.empty())
	{
		meshLoadErr(errMsg, "Mesh file contains no vertices.");
		return false;
	}

	if (!CGAL::Polygon_mesh_processing::orient_polygon_soup(points, polygons))
	{
		RunLogger::warn("[MeshBackendData] orient_polygon_soup duplicated vertices (non-manifold geometry).");
	}

	std::vector<float> soup;
	meshBuildSoupFromPolygons(points, polygons, soup);

	if (soup.empty())
	{
		meshLoadErr(errMsg, "No triangles extracted from mesh file.");
		return false;
	}
	mesh.setTriangleSoup(std::move(soup));
	RunLogger::info("[MeshBackendData] Polygon-soup mesh loaded successfully.");
	return true;
}

} // namespace mesh_backend_load
