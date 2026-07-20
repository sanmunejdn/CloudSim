/// @file MeshSurfaceReconstructionAmrtoLoader.cpp
/// @brief MeshSurfaceReconstructionAmrtoLoader 实现

#include "MeshSurfaceReconstructionAmrtoLoader.h"

#include "MeshSurfaceReconstructionPartitionCommon.h"
#include "RunLogger.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace fs = std::filesystem;

namespace geoalgo
{
namespace meshrecon
{
namespace
{
struct Vec3dLite
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

Vec3dLite readV(const std::vector<float>& v, const int i)
{
	const std::size_t b = static_cast<std::size_t>(i) * 3U;
	return {v[b], v[b + 1U], v[b + 2U]};
}

bool parseFaceToken(const std::string& tok, int& outVi, int& outVti)
{
	outVi = -1;
	outVti = -1;
	const std::size_t slash = tok.find('/');
	if (slash == std::string::npos)
	{
		outVi = std::stoi(tok) - 1;
		return true;
	}
	outVi = std::stoi(tok.substr(0, slash)) - 1;
	if (slash + 1U < tok.size())
	{
		const std::size_t slash2 = tok.find('/', slash + 1U);
		const std::string vtPart =
			(slash2 == std::string::npos) ? tok.substr(slash + 1U) : tok.substr(slash + 1U, slash2 - slash - 1U);
		if (!vtPart.empty())
		{
			outVti = std::stoi(vtPart) - 1;
		}
	}
	return true;
}

struct ChartAccel
{
	std::vector<Vec3dLite> triCentroids;
	std::vector<int> chartIds;

	void addTriangle(const Vec3dLite& a, const Vec3dLite& b, const Vec3dLite& c, const int chartId)
	{
		triCentroids.push_back({(a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0, (a.z + b.z + c.z) / 3.0});
		chartIds.push_back(chartId);
	}

	int findNearestChart(const Vec3dLite& p) const
	{
		if (triCentroids.empty())
		{
			return -1;
		}
		int best = chartIds.front();
		double bestD = 1e300;
		for (std::size_t i = 0; i < triCentroids.size(); ++i)
		{
			const Vec3dLite& c = triCentroids[i];
			const double dx = c.x - p.x;
			const double dy = c.y - p.y;
			const double dz = c.z - p.z;
			const double d = dx * dx + dy * dy + dz * dz;
			if (d < bestD)
			{
				bestD = d;
				best = chartIds[i];
			}
		}
		return best;
	}
};

void addChartTris(const QuadMeshLite& chartQuad, const int chartId, ChartAccel& accel)
{
	const int qCount = static_cast<int>(chartQuad.quadFaces.size() / 4U);
	for (int qi = 0; qi < qCount; ++qi)
	{
		const std::size_t b = static_cast<std::size_t>(qi) * 4U;
		const int i0 = chartQuad.quadFaces[b];
		const int i1 = chartQuad.quadFaces[b + 1U];
		const int i2 = chartQuad.quadFaces[b + 2U];
		const int i3 = chartQuad.quadFaces[b + 3U];
		if (i0 < 0 || i3 < 0)
		{
			continue;
		}
		const Vec3dLite v0 = readV(chartQuad.vertices, i0);
		const Vec3dLite v1 = readV(chartQuad.vertices, i1);
		const Vec3dLite v2 = readV(chartQuad.vertices, i2);
		const Vec3dLite v3 = readV(chartQuad.vertices, i3);
		accel.addTriangle(v0, v1, v2, chartId);
		accel.addTriangle(v0, v2, v3, chartId);
	}
}

int findGlobalVertexIndex(const IndexedMeshLite& mesh, const Vec3dLite& p, const double eps = 1e-4)
{
	const int n = static_cast<int>(mesh.vertices.size() / 3U);
	for (int vi = 0; vi < n; ++vi)
	{
		const Vec3dLite q = readV(mesh.vertices, vi);
		if (std::abs(q.x - p.x) <= eps && std::abs(q.y - p.y) <= eps && std::abs(q.z - p.z) <= eps)
		{
			return vi;
		}
	}
	return -1;
}

struct MeshBoundsLite
{
	Vec3dLite minPt{};
	Vec3dLite maxPt{};
	double diagonal = 0.0;
};

MeshBoundsLite computeMeshBounds(const IndexedMeshLite& mesh)
{
	MeshBoundsLite bounds;
	const int vCount = static_cast<int>(mesh.vertices.size() / 3U);
	if (vCount < 1)
	{
		return bounds;
	}
	bounds.minPt.x = mesh.vertices[0];
	bounds.minPt.y = mesh.vertices[1U];
	bounds.minPt.z = mesh.vertices[2U];
	bounds.maxPt = bounds.minPt;
	for (int vi = 1; vi < vCount; ++vi)
	{
		const std::size_t b = static_cast<std::size_t>(vi) * 3U;
		const Vec3dLite p = {mesh.vertices[b], mesh.vertices[b + 1U], mesh.vertices[b + 2U]};
		bounds.minPt.x = std::min(bounds.minPt.x, p.x);
		bounds.minPt.y = std::min(bounds.minPt.y, p.y);
		bounds.minPt.z = std::min(bounds.minPt.z, p.z);
		bounds.maxPt.x = std::max(bounds.maxPt.x, p.x);
		bounds.maxPt.y = std::max(bounds.maxPt.y, p.y);
		bounds.maxPt.z = std::max(bounds.maxPt.z, p.z);
	}
	const double dx = bounds.maxPt.x - bounds.minPt.x;
	const double dy = bounds.maxPt.y - bounds.minPt.y;
	const double dz = bounds.maxPt.z - bounds.minPt.z;
	bounds.diagonal = std::sqrt(dx * dx + dy * dy + dz * dz);
	return bounds;
}

Vec3dLite meshBoundsCenter(const MeshBoundsLite& bounds)
{
	return {(bounds.minPt.x + bounds.maxPt.x) * 0.5, (bounds.minPt.y + bounds.maxPt.y) * 0.5,
			(bounds.minPt.z + bounds.maxPt.z) * 0.5};
}

} // namespace

bool detectChartCornersFromUv(const QuadMeshLite& chartQuad, std::array<int, 4>& outCorners)
{
	outCorners = {-1, -1, -1, -1};
	if (chartQuad.vertexUv.size() < 2U)
	{
		return false;
	}
	const auto nearCorner = [&](const double u, const double v, const int cornerIdx)
	{
		for (int vi = 0; vi < static_cast<int>(chartQuad.vertexUv.size() / 2U); ++vi)
		{
			const double uu = chartQuad.vertexUv[static_cast<std::size_t>(vi) * 2U];
			const double vv = chartQuad.vertexUv[static_cast<std::size_t>(vi) * 2U + 1U];
			if (std::abs(uu - u) <= 0.02 && std::abs(vv - v) <= 0.02)
			{
				outCorners[static_cast<std::size_t>(cornerIdx)] = vi;
			}
		}
	};
	nearCorner(0.0, 0.0, 0);
	nearCorner(1.0, 0.0, 3);
	nearCorner(0.0, 1.0, 2);
	nearCorner(1.0, 1.0, 1);
	return outCorners[0] >= 0 && outCorners[1] >= 0 && outCorners[2] >= 0 && outCorners[3] >= 0;
}

std::string resolveCloudSimSdkRoot()
{
#if defined(_MSC_VER)
	char* env = nullptr;
	size_t envLen = 0;
	if (_dupenv_s(&env, &envLen, "CLOUDSIM_SDK") == 0 && env != nullptr)
	{
		std::string root(env);
		free(env);
		return root;
	}
#else
	if (const char* env = std::getenv("CLOUDSIM_SDK"))
	{
		return env;
	}
#endif
	return "D:\\Project\\VSprogram\\CGAL5.5.2\\bin\\SDK";
}

std::string defaultAmrtoToolsDirectory()
{
	return resolveCloudSimSdkRoot() + "\\AMRTO\\Instant-Meshes and GMCG_revision";
}

std::string defaultAmrtoGoldenDataDirectory()
{
	return resolveCloudSimSdkRoot() + "\\CODE_AMRTO\\data_smooth";
}

bool loadObjQuadMeshWithVt(const std::string& objPath, QuadMeshLite& outMesh, std::string* errMsg)
{
	outMesh = {};
	std::ifstream in(objPath);
	if (!in)
	{
		if (errMsg)
		{
			*errMsg = "cannot open obj: " + objPath;
		}
		return false;
	}

	std::vector<std::array<int, 4>> rawFaces;
	std::vector<std::array<int, 4>> rawFaceVt;
	std::string line;
	while (std::getline(in, line))
	{
		if (line.empty() || line[0] == '#')
		{
			continue;
		}
		std::istringstream iss(line);
		std::string tag;
		iss >> tag;
		if (tag == "v")
		{
			double x = 0.0;
			double y = 0.0;
			double z = 0.0;
			iss >> x >> y >> z;
			outMesh.vertices.push_back(static_cast<float>(x));
			outMesh.vertices.push_back(static_cast<float>(y));
			outMesh.vertices.push_back(static_cast<float>(z));
		}
		else if (tag == "vt")
		{
			double u = 0.0;
			double v = 0.0;
			iss >> u >> v;
			outMesh.vertexUv.push_back(static_cast<float>(u));
			outMesh.vertexUv.push_back(static_cast<float>(v));
		}
		else if (tag == "f")
		{
			std::array<int, 4> face{-1, -1, -1, -1};
			std::array<int, 4> faceVt{-1, -1, -1, -1};
			for (int i = 0; i < 4; ++i)
			{
				std::string tok;
				if (!(iss >> tok))
				{
					break;
				}
				parseFaceToken(tok, face[static_cast<std::size_t>(i)], faceVt[static_cast<std::size_t>(i)]);
			}
			rawFaces.push_back(face);
			rawFaceVt.push_back(faceVt);
		}
	}

	if (rawFaces.empty() || outMesh.vertices.empty())
	{
		if (errMsg)
		{
			*errMsg = "obj has no faces or vertices: " + objPath;
		}
		return false;
	}

	std::unordered_map<int, int> oldToNew;
	std::vector<float> compactVerts;
	std::vector<float> compactUv;
	auto mapVertex = [&](const int oldIdx, const int oldVtIdx) -> int
	{
		if (oldIdx < 0)
		{
			return -1;
		}
		auto it = oldToNew.find(oldIdx);
		if (it != oldToNew.end())
		{
			return it->second;
		}
		const int ni = static_cast<int>(compactVerts.size() / 3U);
		oldToNew[oldIdx] = ni;
		const std::size_t b = static_cast<std::size_t>(oldIdx) * 3U;
		compactVerts.push_back(outMesh.vertices[b]);
		compactVerts.push_back(outMesh.vertices[b + 1U]);
		compactVerts.push_back(outMesh.vertices[b + 2U]);
		if (oldVtIdx >= 0 && static_cast<std::size_t>(oldVtIdx) * 2U + 1U < outMesh.vertexUv.size())
		{
			compactUv.push_back(outMesh.vertexUv[static_cast<std::size_t>(oldVtIdx) * 2U]);
			compactUv.push_back(outMesh.vertexUv[static_cast<std::size_t>(oldVtIdx) * 2U + 1U]);
		}
		else
		{
			compactUv.push_back(0.f);
			compactUv.push_back(0.f);
		}
		return ni;
	};

	for (std::size_t fi = 0; fi < rawFaces.size(); ++fi)
	{
		const auto& face = rawFaces[fi];
		const auto& faceVt = rawFaceVt[fi];
		if (face[3] < 0)
		{
			continue;
		}
		for (int c = 0; c < 4; ++c)
		{
			const int ni = mapVertex(face[static_cast<std::size_t>(c)], faceVt[static_cast<std::size_t>(c)]);
			outMesh.quadFaces.push_back(ni);
		}
	}

	outMesh.vertices = std::move(compactVerts);
	outMesh.vertexUv = std::move(compactUv);
	return !outMesh.quadFaces.empty();
}

bool triangulateQuadMeshToIndexed(const QuadMeshLite& quadMesh, IndexedMeshLite& outTri, std::string* errMsg)
{
	outTri = {};
	if (quadMesh.vertices.empty() || quadMesh.quadFaces.size() % 4U != 0U)
	{
		if (errMsg)
		{
			*errMsg = "invalid quad mesh for triangulation";
		}
		return false;
	}
	outTri.vertices = quadMesh.vertices;
	const int qCount = static_cast<int>(quadMesh.quadFaces.size() / 4U);
	for (int qi = 0; qi < qCount; ++qi)
	{
		const std::size_t b = static_cast<std::size_t>(qi) * 4U;
		const int i0 = quadMesh.quadFaces[b];
		const int i1 = quadMesh.quadFaces[b + 1U];
		const int i2 = quadMesh.quadFaces[b + 2U];
		const int i3 = quadMesh.quadFaces[b + 3U];
		outTri.faces.push_back(i0);
		outTri.faces.push_back(i1);
		outTri.faces.push_back(i2);
		outTri.faces.push_back(i0);
		outTri.faces.push_back(i2);
		outTri.faces.push_back(i3);
	}
	return true;
}

bool loadAmrtoGoldenDataset(const std::string& datasetRoot, const std::string& globalResultObjName,
							GmcgResult& outResult, std::string* errMsg)
{
	outResult = {};
	const fs::path root(datasetRoot);
	const fs::path globalPath = root / globalResultObjName;
	if (!loadObjQuadMeshWithVt(globalPath.string(), outResult.globalQuad, errMsg))
	{
		return false;
	}
	outResult.globalResultObjPath = globalPath.string();

	const fs::path quadDir = root / "output_quad";
	if (!fs::is_directory(quadDir))
	{
		if (errMsg)
		{
			*errMsg = "missing output_quad directory: " + quadDir.string();
		}
		return false;
	}

	std::vector<fs::path> quadFiles;
	for (const auto& entry : fs::directory_iterator(quadDir))
	{
		if (entry.path().extension() == ".obj")
		{
			quadFiles.push_back(entry.path());
		}
	}
	std::sort(quadFiles.begin(), quadFiles.end());

	outResult.charts.reserve(quadFiles.size());
	for (const fs::path& qp : quadFiles)
	{
		GmcgChart chart;
		chart.chartId = static_cast<int>(outResult.charts.size());
		if (!loadObjQuadMeshWithVt(qp.string(), chart.quadMesh, errMsg))
		{
			RunLogger::warn("amrto golden skip chart: " + qp.string());
			continue;
		}
		chart.vertexUv = chart.quadMesh.vertexUv;
		detectChartCornersFromUv(chart.quadMesh, chart.cornerVertexIndices);
		outResult.charts.push_back(std::move(chart));
	}

	if (outResult.charts.empty())
	{
		if (errMsg)
		{
			*errMsg = "no charts loaded from " + quadDir.string();
		}
		return false;
	}
	return true;
}

bool gmcgResultToQuadPatches(const IndexedMeshLite& mesh, const GmcgResult& gmcg, std::vector<QuadPatch>& patches,
							 std::string* errMsg)
{
	const int faceCount = static_cast<int>(mesh.faces.size() / 3U);
	if (faceCount < 1 || gmcg.charts.empty())
	{
		if (errMsg)
		{
			*errMsg = "gmcgResultToQuadPatches: empty mesh or charts";
		}
		return false;
	}

	ChartAccel accel;
	for (const GmcgChart& chart : gmcg.charts)
	{
		addChartTris(chart.quadMesh, chart.chartId, accel);
	}

	std::vector<int> faceChart(static_cast<std::size_t>(faceCount), -1);
	for (int f = 0; f < faceCount; ++f)
	{
		const std::size_t b = static_cast<std::size_t>(f) * 3U;
		const Vec3dLite p0 = readV(mesh.vertices, mesh.faces[b]);
		const Vec3dLite p1 = readV(mesh.vertices, mesh.faces[b + 1U]);
		const Vec3dLite p2 = readV(mesh.vertices, mesh.faces[b + 2U]);
		const Vec3dLite c = {(p0.x + p1.x + p2.x) / 3.0, (p0.y + p1.y + p2.y) / 3.0, (p0.z + p1.z + p2.z) / 3.0};
		faceChart[static_cast<std::size_t>(f)] = accel.findNearestChart(c);
	}

	const MeshAdjacency adj = buildMeshAdjacency(mesh, faceCount);
	for (int pass = 0; pass < 8; ++pass)
	{
		bool changed = false;
		for (int f = 0; f < faceCount; ++f)
		{
			if (faceChart[static_cast<std::size_t>(f)] >= 0)
			{
				continue;
			}
			std::unordered_map<int, int> votes;
			for (const int nb : adj.fullAdj[static_cast<std::size_t>(f)])
			{
				const int c = faceChart[static_cast<std::size_t>(nb)];
				if (c >= 0)
				{
					++votes[c];
				}
			}
			int best = -1;
			int bestV = 0;
			for (const auto& kv : votes)
			{
				if (kv.second > bestV)
				{
					bestV = kv.second;
					best = kv.first;
				}
			}
			if (best >= 0)
			{
				faceChart[static_cast<std::size_t>(f)] = best;
				changed = true;
			}
		}
		if (!changed)
		{
			break;
		}
	}
	for (int& c : faceChart)
	{
		if (c < 0)
		{
			c = 0;
		}
	}

	std::vector<std::vector<int>> chartFaces(gmcg.charts.size());
	for (int f = 0; f < faceCount; ++f)
	{
		const int c = faceChart[static_cast<std::size_t>(f)];
		if (c >= 0 && static_cast<std::size_t>(c) < chartFaces.size())
		{
			chartFaces[static_cast<std::size_t>(c)].push_back(f);
		}
	}

	patches.clear();
	for (std::size_t ci = 0; ci < gmcg.charts.size(); ++ci)
	{
		if (chartFaces[ci].empty())
		{
			continue;
		}
		QuadPatch patch;
		patch.faceIndices = std::move(chartFaces[ci]);
		const GmcgChart& chart = gmcg.charts[ci];
		patch.hasSquareCorners = false;
		for (int k = 0; k < 4; ++k)
		{
			const int lvi = chart.cornerVertexIndices[static_cast<std::size_t>(k)];
			if (lvi < 0)
			{
				continue;
			}
			const Vec3dLite p = readV(chart.quadMesh.vertices, lvi);
			const int gvi = findGlobalVertexIndex(mesh, p);
			if (gvi >= 0)
			{
				patch.cornerMeshVertices[static_cast<std::size_t>(k)] = gvi;
				patch.hasSquareCorners = true;
			}
		}
		patches.push_back(std::move(patch));
	}

	if (patches.empty())
	{
		if (errMsg)
		{
			*errMsg = "gmcgResultToQuadPatches produced no patches";
		}
		return false;
	}
	return true;
}

bool loadAmrtoChartsFromDirectory(const std::string& datasetRoot, GmcgResult& outResult, std::string* errMsg)
{
	outResult.charts.clear();
	const fs::path quadDir = fs::path(datasetRoot) / "output_quad";
	if (!fs::is_directory(quadDir))
	{
		if (errMsg)
		{
			*errMsg = "missing output_quad directory: " + quadDir.string();
		}
		return false;
	}
	std::vector<fs::path> quadFiles;
	for (const auto& entry : fs::directory_iterator(quadDir))
	{
		if (entry.path().extension() == ".obj")
		{
			quadFiles.push_back(entry.path());
		}
	}
	std::sort(quadFiles.begin(), quadFiles.end());
	for (const fs::path& qp : quadFiles)
	{
		GmcgChart chart;
		chart.chartId = static_cast<int>(outResult.charts.size());
		if (!loadObjQuadMeshWithVt(qp.string(), chart.quadMesh, errMsg))
		{
			continue;
		}
		chart.vertexUv = chart.quadMesh.vertexUv;
		detectChartCornersFromUv(chart.quadMesh, chart.cornerVertexIndices);
		outResult.charts.push_back(std::move(chart));
	}
	if (outResult.charts.empty())
	{
		if (errMsg)
		{
			*errMsg = "no charts in " + quadDir.string();
		}
		return false;
	}
	return true;
}

bool isGoldenDatasetMeshCompatible(const IndexedMeshLite& mesh, const std::string& datasetRoot,
								   const std::string& globalResultObjName, std::string* errMsg)
{
	const fs::path globalPath = fs::path(datasetRoot) / globalResultObjName;
	QuadMeshLite goldenQuad;
	if (!loadObjQuadMeshWithVt(globalPath.string(), goldenQuad, errMsg))
	{
		return false;
	}
	IndexedMeshLite goldenTri;
	if (!triangulateQuadMeshToIndexed(goldenQuad, goldenTri, errMsg))
	{
		return false;
	}

	const int inputFaces = static_cast<int>(mesh.faces.size() / 3U);
	const int goldenFaces = static_cast<int>(goldenTri.faces.size() / 3U);
	if (inputFaces < 1 || goldenFaces < 1)
	{
		if (errMsg)
		{
			*errMsg = "golden compatibility: empty input or golden mesh";
		}
		return false;
	}
	const double faceRatio = static_cast<double>(inputFaces) / static_cast<double>(goldenFaces);
	if (faceRatio < 0.90 || faceRatio > 1.10)
	{
		if (errMsg)
		{
			*errMsg = "input mesh does not match golden dataset (face count " + std::to_string(inputFaces) + " vs " +
					  std::to_string(goldenFaces) +
					  "); golden loader only works for smooth_060 / matching CODE_AMRTO mesh";
		}
		return false;
	}

	const MeshBoundsLite inputBounds = computeMeshBounds(mesh);
	const MeshBoundsLite goldenBounds = computeMeshBounds(goldenTri);
	if (inputBounds.diagonal <= 1e-9 || goldenBounds.diagonal <= 1e-9)
	{
		if (errMsg)
		{
			*errMsg = "golden compatibility: degenerate mesh bounds";
		}
		return false;
	}
	const Vec3dLite ci = meshBoundsCenter(inputBounds);
	const Vec3dLite cg = meshBoundsCenter(goldenBounds);
	const double dx = ci.x - cg.x;
	const double dy = ci.y - cg.y;
	const double dz = ci.z - cg.z;
	const double centerDist = std::sqrt(dx * dx + dy * dy + dz * dz);
	const double refDiag = std::max(inputBounds.diagonal, goldenBounds.diagonal);
	if (centerDist > 0.05 * refDiag)
	{
		if (errMsg)
		{
			*errMsg = "input mesh bbox center differs from golden smooth_060; use instant-meshes for this model";
		}
		return false;
	}
	const auto extentRatioOk = [](const double a, const double b)
	{
		if (a <= 1e-9 || b <= 1e-9)
		{
			return false;
		}
		const double ratio = a / b;
		return ratio >= 0.90 && ratio <= 1.10;
	};
	const double inputExt[3] = {inputBounds.maxPt.x - inputBounds.minPt.x, inputBounds.maxPt.y - inputBounds.minPt.y,
								inputBounds.maxPt.z - inputBounds.minPt.z};
	const double goldenExt[3] = {goldenBounds.maxPt.x - goldenBounds.minPt.x,
								 goldenBounds.maxPt.y - goldenBounds.minPt.y,
								 goldenBounds.maxPt.z - goldenBounds.minPt.z};
	for (int axis = 0; axis < 3; ++axis)
	{
		if (!extentRatioOk(inputExt[axis], goldenExt[axis]))
		{
			if (errMsg)
			{
				*errMsg = "input mesh bbox size differs from golden smooth_060; use instant-meshes for this model";
			}
			return false;
		}
	}
	return true;
}

bool partitionFromGoldenLoader(const IndexedMeshLite& mesh, const std::string& datasetRoot,
							   const std::string& globalResultObjName, std::vector<QuadPatch>& patches,
							   int& outJunctionCount, std::string* errMsg)
{
	GmcgResult gmcg;
	if (!loadAmrtoGoldenDataset(datasetRoot, globalResultObjName, gmcg, errMsg))
	{
		return false;
	}
	if (!gmcgResultToQuadPatches(mesh, gmcg, patches, errMsg))
	{
		return false;
	}
	const int faceCount = static_cast<int>(mesh.faces.size() / 3U);
	const int chartCount = static_cast<int>(gmcg.charts.size());
	const int patchCount = static_cast<int>(patches.size());
	const int minExpectedPatches = std::max(10, chartCount / 4);
	if (patchCount < minExpectedPatches)
	{
		if (errMsg)
		{
			*errMsg = "golden partition quality too low: " + std::to_string(patchCount) + " patches from " +
					  std::to_string(chartCount) + " charts; input mesh likely does not match CODE_AMRTO golden data";
		}
		patches.clear();
		return false;
	}
	int maxPatchFaces = 0;
	for (const QuadPatch& patch : patches)
	{
		maxPatchFaces = std::max(maxPatchFaces, static_cast<int>(patch.faceIndices.size()));
	}
	if (faceCount > 0 && maxPatchFaces > static_cast<int>(static_cast<double>(faceCount) * 0.60))
	{
		if (errMsg)
		{
			*errMsg = "golden partition dominated by one patch; input mesh does not match golden charts";
		}
		patches.clear();
		return false;
	}
	rebuildPatchAdjacency(buildMeshAdjacency(mesh, faceCount).fullAdj, faceCount, patches);
	outJunctionCount = computeJunctionCount(patches);
	RunLogger::info(std::string("amrto golden loader: ") + std::to_string(patches.size()) + " patches from " +
					datasetRoot);
	return true;
}

} // namespace meshrecon
} // namespace geoalgo
