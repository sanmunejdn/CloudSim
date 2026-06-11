#include "MeshNormalSmooth.h"
#include "VcgMeshAdapter.h"

#include <cmath>
#include <vector>

namespace vcgalgo
{
namespace
{

struct Vec3
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;

	Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
	Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
	Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }

	double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
	double length() const { return std::sqrt(dot(*this)); }

	Vec3 normalized() const
	{
		const double len = length();
		if (len < 1e-12)
		{
			return {0.0, 0.0, 1.0};
		}
		return *this * (1.0 / len);
	}
};

struct FaceAdj
{
	int v0 = 0;
	int v1 = 0;
	int v2 = 0;
	Vec3 normal;
	Vec3 centroid;
	double area = 0.0;
	std::vector<int> adjacentFaces;
};

static Vec3 cross(const Vec3& a, const Vec3& b)
{
	return {
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x};
}

static double triangleArea(const Vec3& a, const Vec3& b, const Vec3& c)
{
	return cross(b - a, c - a).length() * 0.5;
}

static Vec3 readVert(const std::vector<float>& verts, const int idx)
{
	const std::size_t b = static_cast<std::size_t>(idx) * 3U;
	return {verts[b], verts[b + 1U], verts[b + 2U]};
}

static void writeVert(std::vector<float>& verts, const int idx, const Vec3& p)
{
	const std::size_t b = static_cast<std::size_t>(idx) * 3U;
	verts[b] = static_cast<float>(p.x);
	verts[b + 1U] = static_cast<float>(p.y);
	verts[b + 2U] = static_cast<float>(p.z);
}

static void buildFaceAdjacency(
	const IndexedMesh& mesh,
	std::vector<FaceAdj>& faces,
	std::vector<std::vector<int>>& vertexFaces)
{
	const int faceCount = static_cast<int>(mesh.faces.size() / 3U);
	const int vertCount = static_cast<int>(mesh.vertices.size() / 3U);
	faces.resize(static_cast<std::size_t>(faceCount));
	vertexFaces.assign(static_cast<std::size_t>(vertCount), {});

	for (int f = 0; f < faceCount; ++f)
	{
		const std::size_t b = static_cast<std::size_t>(f) * 3U;
		FaceAdj& fa = faces[static_cast<std::size_t>(f)];
		fa.v0 = mesh.faces[b];
		fa.v1 = mesh.faces[b + 1U];
		fa.v2 = mesh.faces[b + 2U];
		const Vec3 p0 = readVert(mesh.vertices, fa.v0);
		const Vec3 p1 = readVert(mesh.vertices, fa.v1);
		const Vec3 p2 = readVert(mesh.vertices, fa.v2);
		fa.centroid = (p0 + p1 + p2) * (1.0 / 3.0);
		fa.area = triangleArea(p0, p1, p2);
		Vec3 n = cross(p1 - p0, p2 - p0);
		fa.normal = n.normalized();
		vertexFaces[static_cast<std::size_t>(fa.v0)].push_back(f);
		vertexFaces[static_cast<std::size_t>(fa.v1)].push_back(f);
		vertexFaces[static_cast<std::size_t>(fa.v2)].push_back(f);
	}

	for (int f = 0; f < faceCount; ++f)
	{
		FaceAdj& fa = faces[static_cast<std::size_t>(f)];
		std::vector<int> candidates;
		for (const int vi : {fa.v0, fa.v1, fa.v2})
		{
			for (const int nf : vertexFaces[static_cast<std::size_t>(vi)])
			{
				if (nf != f)
				{
					candidates.push_back(nf);
				}
			}
		}
		for (const int nf : candidates)
		{
			const FaceAdj& fb = faces[static_cast<std::size_t>(nf)];
			int shared = 0;
			for (const int va : {fa.v0, fa.v1, fa.v2})
			{
				for (const int vb : {fb.v0, fb.v1, fb.v2})
				{
					if (va == vb)
					{
						++shared;
					}
				}
			}
			if (shared >= 2)
			{
				fa.adjacentFaces.push_back(nf);
			}
		}
	}
}

static double normalAngle(const Vec3& a, const Vec3& b)
{
	const double d = std::max(-1.0, std::min(1.0, a.dot(b)));
	return std::acos(d);
}

static double vertexVariation(const FaceAdj& face, const int cornerIdx, const std::vector<FaceAdj>& faces)
{
	const int vi = (cornerIdx == 0) ? face.v0 : (cornerIdx == 1 ? face.v1 : face.v2);
	std::vector<int> oneRing;
	for (const int af : face.adjacentFaces)
	{
		const FaceAdj& nb = faces[static_cast<std::size_t>(af)];
		if (nb.v0 == vi || nb.v1 == vi || nb.v2 == vi)
		{
			oneRing.push_back(af);
		}
	}
	if (oneRing.empty())
	{
		return 0.0;
	}
	double sumAlpha = 0.0;
	for (const int nf : oneRing)
	{
		sumAlpha += normalAngle(face.normal, faces[static_cast<std::size_t>(nf)].normal);
	}
	const double meanAlpha = sumAlpha / static_cast<double>(oneRing.size());
	double sumDev = 0.0;
	for (const int nf : oneRing)
	{
		const double a = normalAngle(face.normal, faces[static_cast<std::size_t>(nf)].normal);
		sumDev += std::abs(a - meanAlpha);
	}
	return sumDev;
}

static Vec3 kuwaharaNormal(
	const FaceAdj& face,
	const std::vector<FaceAdj>& faces,
	const double k)
{
	const int vi = face.v0;
	double minV = 1e30;
	int bestCorner = 0;
	for (int c = 0; c < 3; ++c)
	{
		const double v = vertexVariation(face, c, faces);
		if (v < minV)
		{
			minV = v;
			bestCorner = c;
		}
	}
	const int pivot = (bestCorner == 0) ? face.v0 : (bestCorner == 1 ? face.v1 : face.v2);
	std::vector<int> ring;
	for (const int af : face.adjacentFaces)
	{
		const FaceAdj& nb = faces[static_cast<std::size_t>(af)];
		if (nb.v0 == pivot || nb.v1 == pivot || nb.v2 == pivot)
		{
			ring.push_back(af);
		}
	}
	if (ring.empty())
	{
		return face.normal;
	}
	Vec3 weightedSum{0.0, 0.0, 0.0};
	double wSum = 0.0;
	for (const int nf : ring)
	{
		const FaceAdj& nb = faces[static_cast<std::size_t>(nf)];
		const double ang = normalAngle(face.normal, nb.normal);
		const double w = std::exp(-k * ang * ang);
		weightedSum = weightedSum + nb.normal * w;
		wSum += w;
	}
	if (wSum < 1e-12)
	{
		return face.normal;
	}
	return (weightedSum * (1.0 / wSum)).normalized();
}

static Vec3 laplacianNormal(const FaceAdj& face, const std::vector<FaceAdj>& faces, const double lambda)
{
	if (face.adjacentFaces.empty())
	{
		return face.normal;
	}
	Vec3 avg{0.0, 0.0, 0.0};
	double wSum = 0.0;
	for (const int nf : face.adjacentFaces)
	{
		const FaceAdj& nb = faces[static_cast<std::size_t>(nf)];
		int sharedEdge = 0;
		for (const int va : {face.v0, face.v1, face.v2})
		{
			for (const int vb : {nb.v0, nb.v1, nb.v2})
			{
				if (va == vb)
				{
					++sharedEdge;
				}
			}
		}
		const double w = (sharedEdge >= 2) ? 1.5 : 1.0;
		avg = avg + nb.normal * w;
		wSum += w;
	}
	if (wSum < 1e-12)
	{
		return face.normal;
	}
	avg = avg * (1.0 / wSum);
	return (face.normal + (avg - face.normal) * lambda).normalized();
}

static double approximateGapVolume(
	const std::vector<float>& beforeVerts,
	const std::vector<float>& afterVerts,
	const std::vector<FaceAdj>& faces)
{
	double volume = 0.0;
	for (const FaceAdj& fa : faces)
	{
		const Vec3 p0b = readVert(beforeVerts, fa.v0);
		const Vec3 p1b = readVert(beforeVerts, fa.v1);
		const Vec3 p2b = readVert(beforeVerts, fa.v2);
		const Vec3 p0a = readVert(afterVerts, fa.v0);
		const Vec3 p1a = readVert(afterVerts, fa.v1);
		const Vec3 p2a = readVert(afterVerts, fa.v2);
		const Vec3 cb = (p0b + p1b + p2b) * (1.0 / 3.0);
		const Vec3 ca = (p0a + p1a + p2a) * (1.0 / 3.0);
		volume += triangleArea(p0b, p1b, p2b) * (cb - ca).length();
	}
	return volume;
}

} // namespace

bool smoothMeshByNormalAdjustment(
	const std::vector<float>& soupIn,
	std::vector<float>& soupOut,
	const MeshNormalSmoothParams& params,
	double* outGapVolume,
	std::string* errMsg)
{
	IndexedMesh mesh;
	if (!triangleSoupToIndexedMesh(soupIn, mesh, errMsg))
	{
		return false;
	}
	if (mesh.faces.empty())
	{
		if (errMsg)
		{
			*errMsg = "empty mesh";
		}
		return false;
	}

	std::vector<FaceAdj> faces;
	std::vector<std::vector<int>> vertexFaces;
	buildFaceAdjacency(mesh, faces, vertexFaces);

	const std::vector<float> vertsBefore = mesh.vertices;

	for (int iter = 0; iter < params.iterations; ++iter)
	{
		std::vector<Vec3> newNormals(faces.size());
		for (std::size_t f = 0; f < faces.size(); ++f)
		{
			const FaceAdj& fa = faces[f];
			double c = 0.0;
			for (int corner = 0; corner < 3; ++corner)
			{
				c += vertexVariation(fa, corner, faces);
			}
			Vec3 n = (c <= params.featureThresholdC0)
				? laplacianNormal(fa, faces, params.laplacianLambda)
				: kuwaharaNormal(fa, faces, params.bilateralK);
			newNormals[f] = n.normalized();
		}

		for (std::size_t f = 0; f < faces.size(); ++f)
		{
			faces[f].normal = newNormals[f];
		}

		std::vector<double> vertAreaSum(mesh.vertices.size() / 3U, 0.0);
		std::vector<Vec3> vertDelta(mesh.vertices.size() / 3U, {0.0, 0.0, 0.0});

		for (const FaceAdj& fa : faces)
		{
			const Vec3 verts[3] = {
				readVert(mesh.vertices, fa.v0),
				readVert(mesh.vertices, fa.v1),
				readVert(mesh.vertices, fa.v2)};
			const int indices[3] = {fa.v0, fa.v1, fa.v2};
			for (int i = 0; i < 3; ++i)
			{
				const Vec3 toCenter = fa.centroid - verts[i];
				const double proj = toCenter.dot(fa.normal);
				const Vec3 disp = fa.normal * proj;
				const std::size_t vi = static_cast<std::size_t>(indices[i]);
				vertDelta[vi] = vertDelta[vi] + disp * fa.area;
				vertAreaSum[vi] += fa.area;
			}
		}

		for (std::size_t vi = 0; vi < vertDelta.size(); ++vi)
		{
			if (vertAreaSum[vi] > 1e-12)
			{
				const Vec3 p = readVert(mesh.vertices, static_cast<int>(vi));
				writeVert(mesh.vertices, static_cast<int>(vi), p + vertDelta[vi] * (1.0 / vertAreaSum[vi]));
			}
		}
	}

	if (outGapVolume != nullptr)
	{
		*outGapVolume = approximateGapVolume(vertsBefore, mesh.vertices, faces);
	}

	return indexedMeshToTriangleSoup(mesh, soupOut);
}

} // namespace vcgalgo
