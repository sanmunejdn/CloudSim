/// @file TubularGrindingCommon.cpp
/// @brief TubularGrindingCommon 实现

#include "TubularGrindingCommon.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <unordered_map>

namespace geoalgo
{
namespace tg
{
double axisAngleDeg(const Vec3& a, const Vec3& b);

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kSoupVertexQuantizeScale = 1000.0;

struct QuantizedVertexKey
{
	int64_t x = 0;
	int64_t y = 0;
	int64_t z = 0;

	bool operator==(const QuantizedVertexKey& other) const { return x == other.x && y == other.y && z == other.z; }
};

struct QuantizedVertexKeyHash
{
	std::size_t operator()(const QuantizedVertexKey& key) const
	{
		return static_cast<std::size_t>(key.x ^ (key.y << 16) ^ (key.z << 32));
	}
};

QuantizedVertexKey quantizeSoupVertexKey(const std::vector<float>& soup, const std::size_t base)
{
	return QuantizedVertexKey{
		static_cast<int64_t>(std::round(static_cast<double>(soup[base]) * kSoupVertexQuantizeScale)),
		static_cast<int64_t>(std::round(static_cast<double>(soup[base + 1U]) * kSoupVertexQuantizeScale)),
		static_cast<int64_t>(std::round(static_cast<double>(soup[base + 2U]) * kSoupVertexQuantizeScale))};
}

struct WeldedEdgeKey
{
	int a = 0;
	int b = 0;

	bool operator==(const WeldedEdgeKey& other) const { return a == other.a && b == other.b; }
};

struct WeldedEdgeKeyHash
{
	std::size_t operator()(const WeldedEdgeKey& key) const { return static_cast<std::size_t>(key.a ^ (key.b << 16)); }
};

WeldedEdgeKey makeWeldedEdgeKey(const int v0, const int v1)
{
	if (v0 < v1)
	{
		return {v0, v1};
	}
	return {v1, v0};
}

} // namespace

Vec3 normalizeVec3(const Vec3& v)
{
	const double len = length(v);
	if (len < 1e-12)
	{
		return {0.0, 0.0, 1.0};
	}
	return {v.x / len, v.y / len, v.z / len};
}

double dot(const Vec3& a, const Vec3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b)
{
	return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

Vec3 add(const Vec3& a, const Vec3& b)
{
	return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 sub(const Vec3& a, const Vec3& b)
{
	return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 scale(const Vec3& v, const double s)
{
	return {v.x * s, v.y * s, v.z * s};
}

double length(const Vec3& v)
{
	return std::sqrt(dot(v, v));
}

double clamp01(const double v)
{
	return std::max(0.0, std::min(1.0, v));
}

void segmentDisplayRgb(const int segmentIndex, const int segmentCount, float& outR, float& outG, float& outB)
{
	const float golden = 0.6180339887f;
	const float hue = std::fmod(golden * static_cast<float>(segmentIndex), 1.0f);
	const float sat = 0.72f;
	const float lit = 0.55f;
	const float c = (1.0f - std::fabs(2.0f * lit - 1.0f)) * sat;
	const float x = c * (1.0f - std::fabs(std::fmod(hue * 6.0f, 2.0f) - 1.0f));
	const float m = lit - 0.5f * c;
	float r = 0.0f;
	float g = 0.0f;
	float b = 0.0f;
	if (hue < 1.0f / 6.0f)
	{
		r = c;
		g = x;
	}
	else if (hue < 2.0f / 6.0f)
	{
		r = x;
		g = c;
	}
	else if (hue < 3.0f / 6.0f)
	{
		g = c;
		b = x;
	}
	else if (hue < 4.0f / 6.0f)
	{
		g = x;
		b = c;
	}
	else if (hue < 5.0f / 6.0f)
	{
		r = x;
		b = c;
	}
	else
	{
		r = c;
		b = x;
	}
	outR = r + m;
	outG = g + m;
	outB = b + m;
	(void)segmentCount;
}

bool buildIndexedMeshLite(const std::vector<float>& soup, IndexedMeshLite& out, std::string* errMsg)
{
	out = IndexedMeshLite{};
	out.soup = soup;
	if (soup.size() < 9U || (soup.size() % 9U) != 0U)
	{
		if (errMsg)
		{
			*errMsg = "invalid triangle soup";
		}
		return false;
	}
	out.faceCount = static_cast<int>(soup.size() / 9U);
	out.faceCentroids.resize(static_cast<std::size_t>(out.faceCount));
	out.faceNormals.resize(static_cast<std::size_t>(out.faceCount));
	out.faceVerts.resize(static_cast<std::size_t>(out.faceCount));
	out.faceNeighbors.assign(static_cast<std::size_t>(out.faceCount), {});

	out.bboxMin = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
				   std::numeric_limits<double>::max()};
	out.bboxMax = {-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(),
				   -std::numeric_limits<double>::max()};

	std::unordered_map<QuantizedVertexKey, int, QuantizedVertexKeyHash> weldedVertexMap;
	std::unordered_map<WeldedEdgeKey, std::vector<int>, WeldedEdgeKeyHash> edgeToFaces;
	for (int f = 0; f < out.faceCount; ++f)
	{
		const std::size_t base = static_cast<std::size_t>(f) * 9U;
		Vec3 v0{soup[base + 0], soup[base + 1], soup[base + 2]};
		Vec3 v1{soup[base + 3], soup[base + 4], soup[base + 5]};
		Vec3 v2{soup[base + 6], soup[base + 7], soup[base + 8]};
		out.faceCentroids[static_cast<std::size_t>(f)] = scale(add(add(v0, v1), v2), 1.0 / 3.0);
		const Vec3 e1 = sub(v1, v0);
		const Vec3 e2 = sub(v2, v0);
		Vec3 n = cross(e1, e2);
		n = normalizeVec3(n);
		out.faceNormals[static_cast<std::size_t>(f)] = n;

		for (const Vec3& p : {v0, v1, v2})
		{
			out.bboxMin[0] = std::min(out.bboxMin[0], p.x);
			out.bboxMin[1] = std::min(out.bboxMin[1], p.y);
			out.bboxMin[2] = std::min(out.bboxMin[2], p.z);
			out.bboxMax[0] = std::max(out.bboxMax[0], p.x);
			out.bboxMax[1] = std::max(out.bboxMax[1], p.y);
			out.bboxMax[2] = std::max(out.bboxMax[2], p.z);
		}

		int weldedVerts[3] = {0, 0, 0};
		for (int c = 0; c < 3; ++c)
		{
			const std::size_t cornerBase = base + static_cast<std::size_t>(c) * 3U;
			const QuantizedVertexKey key = quantizeSoupVertexKey(soup, cornerBase);
			const auto it = weldedVertexMap.find(key);
			if (it == weldedVertexMap.end())
			{
				const int newIndex = static_cast<int>(weldedVertexMap.size());
				weldedVertexMap[key] = newIndex;
				weldedVerts[c] = newIndex;
			}
			else
			{
				weldedVerts[c] = it->second;
			}
		}
		out.faceVerts[static_cast<std::size_t>(f)] = {weldedVerts[0], weldedVerts[1], weldedVerts[2]};
		const int edges[3][2] = {
			{weldedVerts[0], weldedVerts[1]}, {weldedVerts[1], weldedVerts[2]}, {weldedVerts[2], weldedVerts[0]}};
		for (const auto& e : edges)
		{
			edgeToFaces[makeWeldedEdgeKey(e[0], e[1])].push_back(f);
		}
	}

	for (const auto& kv : edgeToFaces)
	{
		const auto& faces = kv.second;
		if (faces.size() < 2U)
		{
			continue;
		}
		for (std::size_t i = 0; i < faces.size(); ++i)
		{
			for (std::size_t j = i + 1U; j < faces.size(); ++j)
			{
				const int fa = faces[i];
				const int fb = faces[j];
				out.faceNeighbors[static_cast<std::size_t>(fa)].push_back(fb);
				out.faceNeighbors[static_cast<std::size_t>(fb)].push_back(fa);
			}
		}
	}
	orientMeshFaceNormals(out);
	return true;
}

void orientMeshFaceNormals(IndexedMeshLite& mesh)
{
	if (mesh.faceCount <= 0)
	{
		return;
	}
	std::vector<uint8_t> visited(static_cast<std::size_t>(mesh.faceCount), 0U);
	for (int seed = 0; seed < mesh.faceCount; ++seed)
	{
		if (visited[static_cast<std::size_t>(seed)] != 0U)
		{
			continue;
		}
		std::queue<int> q;
		q.push(seed);
		visited[static_cast<std::size_t>(seed)] = 1U;
		while (!q.empty())
		{
			const int f = q.front();
			q.pop();
			const Vec3 nf = mesh.faceNormals[static_cast<std::size_t>(f)];
			for (const int nb : mesh.faceNeighbors[static_cast<std::size_t>(f)])
			{
				if (visited[static_cast<std::size_t>(nb)] != 0U)
				{
					continue;
				}
				visited[static_cast<std::size_t>(nb)] = 1U;
				if (dot(nf, mesh.faceNormals[static_cast<std::size_t>(nb)]) < 0.0)
				{
					Vec3& nn = mesh.faceNormals[static_cast<std::size_t>(nb)];
					nn = scale(nn, -1.0);
				}
				q.push(nb);
			}
		}
	}
}

bool isPlanarStencilFace(const IndexedMeshLite& mesh, const int faceIndex, const double minNormalSpreadDeg)
{
	double maxSpread = 0.0;
	const Vec3 n0 = mesh.faceNormals[static_cast<std::size_t>(faceIndex)];
	for (const int nb : mesh.faceNeighbors[static_cast<std::size_t>(faceIndex)])
	{
		maxSpread = std::max(maxSpread, axisAngleDeg(n0, mesh.faceNormals[static_cast<std::size_t>(nb)]));
	}
	return maxSpread < minNormalSpreadDeg;
}

bool solveSymmetric3x3(const double a[3][3], const double b[3], double x[3])
{
	double m[3][4];
	for (int r = 0; r < 3; ++r)
	{
		for (int c = 0; c < 3; ++c)
		{
			m[r][c] = a[r][c];
		}
		m[r][3] = b[r];
	}
	for (int col = 0; col < 3; ++col)
	{
		int pivot = col;
		for (int r = col + 1; r < 3; ++r)
		{
			if (std::fabs(m[r][col]) > std::fabs(m[pivot][col]))
			{
				pivot = r;
			}
		}
		if (std::fabs(m[pivot][col]) < 1e-12)
		{
			return false;
		}
		if (pivot != col)
		{
			for (int c = col; c < 4; ++c)
			{
				std::swap(m[pivot][c], m[col][c]);
			}
		}
		const double inv = 1.0 / m[col][col];
		for (int c = col; c < 4; ++c)
		{
			m[col][c] *= inv;
		}
		for (int r = 0; r < 3; ++r)
		{
			if (r == col)
			{
				continue;
			}
			const double factor = m[r][col];
			for (int c = col; c < 4; ++c)
			{
				m[r][c] -= factor * m[col][c];
			}
		}
	}
	for (int r = 0; r < 3; ++r)
	{
		x[r] = m[r][3];
	}
	return true;
}

bool rayBundleCenterPoint(const std::vector<Vec3>& origins, const std::vector<Vec3>& inwardDirs, Vec3& outCenter)
{
	if (origins.size() < 2U || origins.size() != inwardDirs.size())
	{
		return false;
	}
	double a[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
	double b[3] = {0.0, 0.0, 0.0};
	for (std::size_t i = 0; i < origins.size(); ++i)
	{
		const Vec3 o = origins[i];
		const Vec3 d = normalizeVec3(inwardDirs[i]);
		const double m00 = 1.0 - d.x * d.x;
		const double m01 = -d.x * d.y;
		const double m02 = -d.x * d.z;
		const double m11 = 1.0 - d.y * d.y;
		const double m12 = -d.y * d.z;
		const double m22 = 1.0 - d.z * d.z;
		a[0][0] += m00;
		a[0][1] += m01;
		a[0][2] += m02;
		a[1][0] += m01;
		a[1][1] += m11;
		a[1][2] += m12;
		a[2][0] += m02;
		a[2][1] += m12;
		a[2][2] += m22;
		b[0] += m00 * o.x + m01 * o.y + m02 * o.z;
		b[1] += m01 * o.x + m11 * o.y + m12 * o.z;
		b[2] += m02 * o.x + m12 * o.y + m22 * o.z;
	}
	double x[3] = {0.0, 0.0, 0.0};
	if (!solveSymmetric3x3(a, b, x))
	{
		return false;
	}
	outCenter = {x[0], x[1], x[2]};
	return true;
}

namespace
{
double distancePointToRay(const Vec3& p, const Vec3& o, const Vec3& d)
{
	const Vec3 w = sub(p, o);
	const double t = dot(w, d);
	const Vec3 closest = add(o, scale(d, t));
	return length(sub(p, closest));
}

double meanDistanceToRays(const Vec3& center, const std::vector<Vec3>& origins, const std::vector<Vec3>& inwardDirs)
{
	if (origins.empty())
	{
		return 0.0;
	}
	double sum = 0.0;
	for (std::size_t i = 0; i < origins.size(); ++i)
	{
		sum += distancePointToRay(center, origins[i], normalizeVec3(inwardDirs[i]));
	}
	return sum / static_cast<double>(origins.size());
}

bool closestApproachMidpoint(const Vec3& o1, const Vec3& d1, const Vec3& o2, const Vec3& d2, Vec3& outMid)
{
	const Vec3 w = sub(o1, o2);
	const double a = dot(d1, d1);
	const double b = dot(d1, d2);
	const double c = dot(d2, d2);
	const double d = dot(d1, w);
	const double e = dot(d2, w);
	const double denom = a * c - b * b;
	if (std::fabs(denom) < 1e-10)
	{
		return false;
	}
	const double t1 = (b * e - c * d) / denom;
	const double t2 = (a * e - b * d) / denom;
	const Vec3 p1 = add(o1, scale(d1, t1));
	const Vec3 p2 = add(o2, scale(d2, t2));
	outMid = scale(add(p1, p2), 0.5);
	return true;
}

Vec3 pairwiseRayMidpointCenter(const std::vector<Vec3>& origins, const std::vector<Vec3>& inwardDirs)
{
	Vec3 sum{0.0, 0.0, 0.0};
	int count = 0;
	for (std::size_t i = 0; i < origins.size(); ++i)
	{
		const Vec3 d1 = normalizeVec3(inwardDirs[i]);
		for (std::size_t j = i + 1U; j < origins.size(); ++j)
		{
			const Vec3 d2 = normalizeVec3(inwardDirs[j]);
			Vec3 mid;
			if (closestApproachMidpoint(origins[i], d1, origins[j], d2, mid))
			{
				sum = add(sum, mid);
				++count;
			}
		}
	}
	if (count <= 0)
	{
		return origins[0];
	}
	return scale(sum, 1.0 / static_cast<double>(count));
}

double estimateLocalRaySpan(const std::vector<Vec3>& origins)
{
	double maxSpan = 0.0;
	for (std::size_t i = 0; i < origins.size(); ++i)
	{
		for (std::size_t j = i + 1U; j < origins.size(); ++j)
		{
			maxSpan = std::max(maxSpan, length(sub(origins[j], origins[i])));
		}
	}
	return maxSpan;
}

} // namespace

bool approximateRayBundleCenter(const std::vector<Vec3>& origins, const std::vector<Vec3>& inwardDirs,
								const double maxMeanDistanceMm, Vec3& outCenter)
{
	if (origins.size() < 2U || origins.size() != inwardDirs.size())
	{
		return false;
	}
	const double localSpan = std::max(1.0, estimateLocalRaySpan(origins));
	const double tol = maxMeanDistanceMm > 0.0 ? maxMeanDistanceMm : std::max(3.0, localSpan * 0.55);

	Vec3 candidate;
	if (rayBundleCenterPoint(origins, inwardDirs, candidate))
	{
		if (meanDistanceToRays(candidate, origins, inwardDirs) <= tol)
		{
			outCenter = candidate;
			return true;
		}
	}

	candidate = pairwiseRayMidpointCenter(origins, inwardDirs);
	if (meanDistanceToRays(candidate, origins, inwardDirs) <= tol * 1.35)
	{
		outCenter = candidate;
		return true;
	}

	// 沿各射线固定步长投票，不要求严格交于一点
	double depthSum = 0.0;
	int depthCount = 0;
	for (std::size_t i = 0; i < origins.size(); ++i)
	{
		const Vec3 di = normalizeVec3(inwardDirs[i]);
		for (std::size_t j = 0; j < origins.size(); ++j)
		{
			if (i == j)
			{
				continue;
			}
			const Vec3 w = sub(origins[j], origins[i]);
			const double t = dot(w, di);
			if (t > 0.0)
			{
				depthSum += t;
				++depthCount;
			}
		}
	}
	if (depthCount > 0)
	{
		const double depth = depthSum / static_cast<double>(depthCount);
		Vec3 voteSum{0.0, 0.0, 0.0};
		for (std::size_t i = 0; i < origins.size(); ++i)
		{
			voteSum = add(voteSum, add(origins[i], scale(normalizeVec3(inwardDirs[i]), depth)));
		}
		candidate = scale(voteSum, 1.0 / static_cast<double>(origins.size()));
		if (meanDistanceToRays(candidate, origins, inwardDirs) <= tol * 1.75)
		{
			outCenter = candidate;
			return true;
		}
	}

	outCenter = candidate;
	return meanDistanceToRays(outCenter, origins, inwardDirs) <= tol * 2.5;
}

bool smallestEigenvector3(const double cov[3][3], Vec3& outEigenvector)
{
	Vec3 v{1.0, 0.0, 0.0};
	for (int iter = 0; iter < 32; ++iter)
	{
		Vec3 w{cov[0][0] * v.x + cov[0][1] * v.y + cov[0][2] * v.z, cov[1][0] * v.x + cov[1][1] * v.y + cov[1][2] * v.z,
			   cov[2][0] * v.x + cov[2][1] * v.y + cov[2][2] * v.z};
		const double lenW = length(w);
		if (lenW < 1e-12)
		{
			break;
		}
		v = scale(w, 1.0 / lenW);
	}
	// 逆迭代求最小特征向量：对 (trace*I - A) 做幂迭代
	const double trace = cov[0][0] + cov[1][1] + cov[2][2];
	Vec3 u{0.0, 1.0, 0.0};
	for (int iter = 0; iter < 48; ++iter)
	{
		Vec3 w{(trace - cov[0][0]) * u.x - cov[0][1] * u.y - cov[0][2] * u.z,
			   -cov[1][0] * u.x + (trace - cov[1][1]) * u.y - cov[1][2] * u.z,
			   -cov[2][0] * u.x - cov[2][1] * u.y + (trace - cov[2][2]) * u.z};
		const double lenW = length(w);
		if (lenW < 1e-12)
		{
			u = v;
			break;
		}
		u = scale(w, 1.0 / lenW);
	}
	outEigenvector = normalizeVec3(u);
	return true;
}

Vec3 computeLocalAxisFromFaceNormals(const IndexedMeshLite& mesh, const int faceIndex)
{
	double cov[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
	std::vector<int> stack;
	stack.push_back(faceIndex);
	stack.push_back(faceIndex);
	for (const int nb : mesh.faceNeighbors[static_cast<std::size_t>(faceIndex)])
	{
		stack.push_back(nb);
	}
	for (const int fi : stack)
	{
		if (fi < 0 || fi >= mesh.faceCount)
		{
			continue;
		}
		const Vec3 n = mesh.faceNormals[static_cast<std::size_t>(fi)];
		cov[0][0] += n.x * n.x;
		cov[0][1] += n.x * n.y;
		cov[0][2] += n.x * n.z;
		cov[1][1] += n.y * n.y;
		cov[1][2] += n.y * n.z;
		cov[2][2] += n.z * n.z;
	}
	cov[1][0] = cov[0][1];
	cov[2][0] = cov[0][2];
	cov[2][1] = cov[1][2];
	Vec3 axis;
	smallestEigenvector3(cov, axis);
	if (dot(axis, mesh.faceNormals[static_cast<std::size_t>(faceIndex)]) > 0.0)
	{
		axis = scale(axis, -1.0);
	}
	return normalizeVec3(axis);
}

Vec3 computeLocalAxisFromNormalCrossProducts(const IndexedMeshLite& mesh, const int faceIndex, const int neighborHop)
{
	// 收集 neighborHop 跳邻居面
	std::vector<int> stack;
	stack.push_back(faceIndex);
	for (int hop = 0; hop < neighborHop; ++hop)
	{
		const std::size_t stackSize = stack.size();
		for (std::size_t si = 0; si < stackSize; ++si)
		{
			const int f = stack[si];
			for (const int nb : mesh.faceNeighbors[static_cast<std::size_t>(f)])
			{
				stack.push_back(nb);
			}
		}
	}
	// 去重
	std::sort(stack.begin(), stack.end());
	stack.erase(std::unique(stack.begin(), stack.end()), stack.end());

	// 对每个邻居面法向量与中心面法向量做叉积
	const Vec3 n0 = mesh.faceNormals[static_cast<std::size_t>(faceIndex)];
	Vec3 axisSum{0.0, 0.0, 0.0};
	int validCount = 0;
	for (const int f : stack)
	{
		if (f == faceIndex)
		{
			continue;
		}
		const Vec3 nf = mesh.faceNormals[static_cast<std::size_t>(f)];
		const Vec3 cp = cross(n0, nf);
		const double len = length(cp);
		// 过滤叉积长度 < 1e-6 的退化情况（平行法向量）
		if (len > 1e-6)
		{
			axisSum = add(axisSum, scale(cp, 1.0 / len));
			++validCount;
		}
	}
	if (validCount == 0)
	{
		// 退化情况：所有邻居法向量与中心法向量平行
		return computeLocalAxisFromFaceNormals(mesh, faceIndex);
	}
	return normalizeVec3(axisSum);
}

bool computeFaceCenterFromNormals(const IndexedMeshLite& mesh, const int faceIndex, const double convergenceEpsMm,
								  Vec3& outCenter, double& outRadius)
{
	// 计算局部轴线
	const Vec3 localAxis = computeLocalAxisFromNormalCrossProducts(mesh, faceIndex, 2);

	// 收集 1-hop 邻居面质心
	std::vector<int> neighbors;
	neighbors.push_back(faceIndex);
	for (const int nb : mesh.faceNeighbors[static_cast<std::size_t>(faceIndex)])
	{
		neighbors.push_back(nb);
	}

	// 将质心投影到局部轴线上
	const Vec3 refPoint = mesh.faceCentroids[static_cast<std::size_t>(faceIndex)];
	double projectionSum = 0.0;
	for (const int f : neighbors)
	{
		const Vec3 c = mesh.faceCentroids[static_cast<std::size_t>(f)];
		const double t = dot(sub(c, refPoint), localAxis);
		projectionSum += t;
	}
	const double avgProjection = projectionSum / static_cast<double>(neighbors.size());

	// 中心点 = 参考点 + 轴线方向 * 平均投影
	outCenter = add(refPoint, scale(localAxis, avgProjection));

	// 计算半径（质心到轴线的平均距离）
	double radiusSum = 0.0;
	for (const int f : neighbors)
	{
		const Vec3 c = mesh.faceCentroids[static_cast<std::size_t>(f)];
		const Vec3 d = sub(c, outCenter);
		const double proj = dot(d, localAxis);
		const Vec3 projPoint = add(outCenter, scale(localAxis, proj));
		radiusSum += length(sub(c, projPoint));
	}
	outRadius = radiusSum / static_cast<double>(neighbors.size());

	// 半径过小或过大时返回 false
	const double diag =
		std::sqrt(std::pow(mesh.bboxMax[0] - mesh.bboxMin[0], 2) + std::pow(mesh.bboxMax[1] - mesh.bboxMin[1], 2) +
				  std::pow(mesh.bboxMax[2] - mesh.bboxMin[2], 2));
	if (outRadius < 1e-3 || outRadius > diag * 0.55)
	{
		return false;
	}
	(void)convergenceEpsMm;
	return true;
}

Vec3 computeMainAxisFromFaceAxes(const std::vector<Vec3>& faceAxes)
{
	if (faceAxes.empty())
	{
		return {0.0, 0.0, 1.0};
	}

	// 构建协方差矩阵
	double cov[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
	for (const Vec3& axis : faceAxes)
	{
		cov[0][0] += axis.x * axis.x;
		cov[0][1] += axis.x * axis.y;
		cov[0][2] += axis.x * axis.z;
		cov[1][1] += axis.y * axis.y;
		cov[1][2] += axis.y * axis.z;
		cov[2][2] += axis.z * axis.z;
	}
	cov[1][0] = cov[0][1];
	cov[2][0] = cov[0][2];
	cov[2][1] = cov[1][2];

	// 使用最大特征向量作为主轴（逆迭代求最大特征向量）
	Vec3 v{1.0, 0.0, 0.0};
	for (int iter = 0; iter < 64; ++iter)
	{
		Vec3 w{cov[0][0] * v.x + cov[0][1] * v.y + cov[0][2] * v.z, cov[1][0] * v.x + cov[1][1] * v.y + cov[1][2] * v.z,
			   cov[2][0] * v.x + cov[2][1] * v.y + cov[2][2] * v.z};
		const double lenW = length(w);
		if (lenW < 1e-12)
		{
			break;
		}
		v = scale(w, 1.0 / lenW);
	}
	return normalizeVec3(v);
}

int runDbscan(const std::vector<Vec3>& featurePoints, const double eps, const int minPts, std::vector<int>& outLabels)
{
	const int n = static_cast<int>(featurePoints.size());
	outLabels.assign(static_cast<std::size_t>(n), -1);
	int clusterId = 0;
	const double eps2 = eps * eps;

	auto regionQuery = [&](const int idx)
	{
		std::vector<int> neighbors;
		const Vec3& p = featurePoints[static_cast<std::size_t>(idx)];
		for (int j = 0; j < n; ++j)
		{
			const Vec3 d = sub(featurePoints[static_cast<std::size_t>(j)], p);
			if (dot(d, d) <= eps2)
			{
				neighbors.push_back(j);
			}
		}
		return neighbors;
	};

	for (int i = 0; i < n; ++i)
	{
		if (outLabels[static_cast<std::size_t>(i)] != -1)
		{
			continue;
		}
		const std::vector<int> neighbors = regionQuery(i);
		if (static_cast<int>(neighbors.size()) < minPts)
		{
			outLabels[static_cast<std::size_t>(i)] = -2;
			continue;
		}
		outLabels[static_cast<std::size_t>(i)] = clusterId;
		std::queue<int> q;
		for (const int nb : neighbors)
		{
			if (nb != i)
			{
				q.push(nb);
			}
		}
		while (!q.empty())
		{
			const int j = q.front();
			q.pop();
			if (outLabels[static_cast<std::size_t>(j)] == -2)
			{
				outLabels[static_cast<std::size_t>(j)] = clusterId;
			}
			if (outLabels[static_cast<std::size_t>(j)] != -1)
			{
				continue;
			}
			outLabels[static_cast<std::size_t>(j)] = clusterId;
			const std::vector<int> nbs = regionQuery(j);
			if (static_cast<int>(nbs.size()) >= minPts)
			{
				for (const int nb : nbs)
				{
					if (outLabels[static_cast<std::size_t>(nb)] == -1 || outLabels[static_cast<std::size_t>(nb)] == -2)
					{
						q.push(nb);
					}
				}
			}
		}
		++clusterId;
	}
	return clusterId;
}

double axisAngleDeg(const Vec3& a, const Vec3& b)
{
	// 管轴无方向，正反平行视为 0°
	const double c = std::min(1.0, std::fabs(dot(normalizeVec3(a), normalizeVec3(b))));
	return std::acos(c) * 180.0 / kPi;
}

bool fitCircle2d(const std::vector<std::array<double, 2>>& pts, double& outCx, double& outCy, double& outRadius)
{
	if (pts.size() < 3U)
	{
		return false;
	}
	double sx = 0.0;
	double sy = 0.0;
	double sxx = 0.0;
	double syy = 0.0;
	double sxy = 0.0;
	double sx3 = 0.0;
	double sy3 = 0.0;
	double sx2y = 0.0;
	double sxy2 = 0.0;
	for (const auto& p : pts)
	{
		const double x = p[0];
		const double y = p[1];
		const double x2 = x * x;
		const double y2 = y * y;
		sx += x;
		sy += y;
		sxx += x2;
		syy += y2;
		sxy += x * y;
		sx3 += x2 * x;
		sy3 += y2 * y;
		sx2y += x2 * y;
		sxy2 += x * y2;
	}
	const double n = static_cast<double>(pts.size());
	const double a11 = 2.0 * (sxx - sx * sx / n);
	const double a12 = 2.0 * (sxy - sx * sy / n);
	const double a22 = 2.0 * (syy - sy * sy / n);
	const double b1 = sx3 + sxy2 - (sxx + syy) * sx / n;
	const double b2 = sy3 + sx2y - (sxx + syy) * sy / n;
	const double det = a11 * a22 - a12 * a12;
	if (std::fabs(det) < 1e-12)
	{
		return false;
	}
	outCx = (b1 * a22 - b2 * a12) / det;
	outCy = (a11 * b2 - a12 * b1) / det;
	double r2Sum = 0.0;
	for (const auto& p : pts)
	{
		const double dx = p[0] - outCx;
		const double dy = p[1] - outCy;
		r2Sum += dx * dx + dy * dy;
	}
	outRadius = std::sqrt(r2Sum / n);
	return outRadius > 1e-6;
}

void buildFrenetFrames(const std::vector<TubularCenterlineSample>& samples,
					   std::vector<TubularCenterlineSample>& outSamples)
{
	outSamples = samples;
	if (outSamples.empty())
	{
		return;
	}
	Vec3 prevBin{0.0, 0.0, 1.0};
	for (std::size_t i = 0; i < outSamples.size(); ++i)
	{
		Vec3 t{outSamples[i].tangent[0], outSamples[i].tangent[1], outSamples[i].tangent[2]};
		t = normalizeVec3(t);
		Vec3 ref = std::fabs(dot(t, prevBin)) > 0.95 ? Vec3{1.0, 0.0, 0.0} : prevBin;
		Vec3 n = normalizeVec3(cross(ref, t));
		if (length(n) < 1e-6)
		{
			n = normalizeVec3(cross(Vec3{0.0, 1.0, 0.0}, t));
		}
		Vec3 b = normalizeVec3(cross(t, n));
		outSamples[i].tangent[0] = t.x;
		outSamples[i].tangent[1] = t.y;
		outSamples[i].tangent[2] = t.z;
		outSamples[i].normal[0] = n.x;
		outSamples[i].normal[1] = n.y;
		outSamples[i].normal[2] = n.z;
		outSamples[i].binormal[0] = b.x;
		outSamples[i].binormal[1] = b.y;
		outSamples[i].binormal[2] = b.z;
		prevBin = b;
	}
}

TubularGrindingTemplateKind selectTemplateKind(const TubularPipeSegment& segment,
											   const std::vector<TubularCenterlineSample>& samples)
{
	if (samples.size() < 4U)
	{
		return TubularGrindingTemplateKind::Circumferential;
	}
	double minR = samples.front().radiusMm;
	double maxR = samples.front().radiusMm;
	double arcLen = 0.0;
	for (std::size_t i = 1; i < samples.size(); ++i)
	{
		minR = std::min(minR, samples[i].radiusMm);
		maxR = std::max(maxR, samples[i].radiusMm);
		const Vec3 a{samples[i - 1].positionMm[0], samples[i - 1].positionMm[1], samples[i - 1].positionMm[2]};
		const Vec3 b{samples[i].positionMm[0], samples[i].positionMm[1], samples[i].positionMm[2]};
		arcLen += length(sub(b, a));
	}
	const double avgR = 0.5 * (minR + maxR);
	const double aspect = avgR > 1e-6 ? arcLen / (2.0 * avgR) : arcLen;
	const double radiusVar = avgR > 1e-6 ? (maxR - minR) / avgR : 0.0;
	(void)segment;
	if (radiusVar > 0.15)
	{
		return TubularGrindingTemplateKind::AxialParallel;
	}
	if (aspect > 6.0)
	{
		return TubularGrindingTemplateKind::Helical;
	}
	if (aspect > 3.0)
	{
		return TubularGrindingTemplateKind::Zigzag;
	}
	return TubularGrindingTemplateKind::Circumferential;
}

// === 广义管状分析新增实现 ===

std::vector<int> collectAdaptiveNeighborhood(const IndexedMeshLite& mesh, const int faceIndex,
											 const double targetGeodesicRadiusMm,
											 std::vector<double>& outGeodesicDistances)
{
	outGeodesicDistances.clear();
	const int n = mesh.faceCount;
	if (n <= 0 || faceIndex < 0 || faceIndex >= n)
	{
		return {};
	}

	// 自动估计搜索半径：2-hop 邻居平均边长 × 3
	double radius = targetGeodesicRadiusMm;
	if (radius <= 0.0)
	{
		double sumLen = 0.0;
		int count = 0;
		for (const int nb : mesh.faceNeighbors[static_cast<std::size_t>(faceIndex)])
		{
			const double d = length(sub(mesh.faceCentroids[static_cast<std::size_t>(nb)],
										mesh.faceCentroids[static_cast<std::size_t>(faceIndex)]));
			sumLen += d;
			++count;
			for (const int nb2 : mesh.faceNeighbors[static_cast<std::size_t>(nb)])
			{
				if (nb2 == faceIndex)
					continue;
				const double d2 = length(sub(mesh.faceCentroids[static_cast<std::size_t>(nb2)],
											 mesh.faceCentroids[static_cast<std::size_t>(nb)]));
				sumLen += d2;
				++count;
			}
		}
		radius = (count > 0) ? (sumLen / static_cast<double>(count)) * 3.0 : 10.0;
	}

	// Dijkstra 测地线距离
	std::vector<double> dist(static_cast<std::size_t>(n), std::numeric_limits<double>::max());
	dist[static_cast<std::size_t>(faceIndex)] = 0.0;

	// 优先队列：(距离, 面索引)
	using DistFace = std::pair<double, int>;
	std::priority_queue<DistFace, std::vector<DistFace>, std::greater<DistFace>> pq;
	pq.push({0.0, faceIndex});

	std::vector<int> result;
	std::vector<double> resultDist;

	while (!pq.empty())
	{
		const auto [d, f] = pq.top();
		pq.pop();

		if (d > dist[static_cast<std::size_t>(f)])
		{
			continue;
		}

		// 高曲率区域收缩：相邻面法向量夹角 > 15° 时缩小搜索
		if (f != faceIndex)
		{
			const double angleDeg = axisAngleDeg(mesh.faceNormals[static_cast<std::size_t>(f)],
												 mesh.faceNormals[static_cast<std::size_t>(faceIndex)]);
			if (angleDeg > 60.0)
			{
				continue; // 跳过高曲率区域
			}
		}

		result.push_back(f);
		resultDist.push_back(d);

		for (const int nb : mesh.faceNeighbors[static_cast<std::size_t>(f)])
		{
			const double edgeLen = length(
				sub(mesh.faceCentroids[static_cast<std::size_t>(nb)], mesh.faceCentroids[static_cast<std::size_t>(f)]));
			const double newDist = d + edgeLen;
			if (newDist < dist[static_cast<std::size_t>(nb)] && newDist <= radius)
			{
				dist[static_cast<std::size_t>(nb)] = newDist;
				pq.push({newDist, nb});
			}
		}
	}

	outGeodesicDistances = std::move(resultDist);
	return result;
}

Vec3 computeLocalAxisFromWeightedPCA(const IndexedMeshLite& mesh, const int faceIndex,
									 const std::vector<int>& neighborhood, const std::vector<double>& geodesicDistances)
{
	if (neighborhood.size() < 3)
	{
		return computeLocalAxisFromFaceNormals(mesh, faceIndex);
	}

	// 计算 sigma（测地线距离中位数）
	std::vector<double> sortedDist = geodesicDistances;
	std::sort(sortedDist.begin(), sortedDist.end());
	const double sigma = sortedDist[sortedDist.size() / 2];
	const double sigma2 = (sigma > 1e-12) ? sigma * sigma : 1.0;

	// 加权协方差矩阵
	double cov[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
	double wSum = 0.0;
	for (std::size_t i = 0; i < neighborhood.size(); ++i)
	{
		const Vec3 n = mesh.faceNormals[static_cast<std::size_t>(neighborhood[i])];
		const double d2 = geodesicDistances[i] * geodesicDistances[i];
		const double w = std::exp(-d2 / sigma2);
		cov[0][0] += w * n.x * n.x;
		cov[0][1] += w * n.x * n.y;
		cov[0][2] += w * n.x * n.z;
		cov[1][1] += w * n.y * n.y;
		cov[1][2] += w * n.y * n.z;
		cov[2][2] += w * n.z * n.z;
		wSum += w;
	}
	if (wSum < 1e-12)
	{
		return computeLocalAxisFromFaceNormals(mesh, faceIndex);
	}

	// 归一化
	const double invW = 1.0 / wSum;
	cov[0][0] *= invW;
	cov[0][1] *= invW;
	cov[0][2] *= invW;
	cov[1][1] *= invW;
	cov[1][2] *= invW;
	cov[2][2] *= invW;
	cov[1][0] = cov[0][1];
	cov[2][0] = cov[0][2];
	cov[2][1] = cov[1][2];

	// 最小特征向量即局部轴线
	Vec3 axis;
	if (!smallestEigenvector3(cov, axis))
	{
		return computeLocalAxisFromFaceNormals(mesh, faceIndex);
	}

	// 确保方向一致性
	if (dot(axis, mesh.faceNormals[static_cast<std::size_t>(faceIndex)]) > 0.0)
	{
		axis = scale(axis, -1.0);
	}
	return normalizeVec3(axis);
}

bool fitEllipse2D(const std::vector<std::array<double, 2>>& pts, double& outSemiMajor, double& outSemiMinor,
				  double& outCx, double& outCy, double& outRotationRad)
{
	const std::size_t n = pts.size();
	if (n < 5)
	{
		return false;
	}

	// 最小二乘椭圆拟合：代数距离法
	// 椭圆方程：ax^2 + bxy + cy^2 + dx + ey + f = 0
	// 约束：4ac - b^2 = 1
	double sx = 0.0, sy = 0.0;
	for (const auto& p : pts)
	{
		sx += p[0];
		sy += p[1];
	}
	const double mx = sx / static_cast<double>(n);
	const double my = sy / static_cast<double>(n);

	// 构建设计矩阵
	std::vector<std::array<double, 6>> D(n);
	for (std::size_t i = 0; i < n; ++i)
	{
		const double x = pts[i][0] - mx;
		const double y = pts[i][1] - my;
		D[i] = {x * x, x * y, y * y, x, y, 1.0};
	}

	// 构建散射矩阵 S1, S2, S3
	double S1[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
	double S2[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
	double S3[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

	for (std::size_t i = 0; i < n; ++i)
	{
		for (int r = 0; r < 3; ++r)
		{
			for (int c = 0; c < 3; ++c)
			{
				S1[r][c] += D[i][r] * D[i][c];
				S2[r][c] += D[i][r] * D[i][c + 3];
				S3[r][c] += D[i][r + 3] * D[i][c + 3];
			}
		}
	}

	// 简化：使用惯量矩法近似
	// 计算二阶矩
	double cxx = 0.0, cxy = 0.0, cyy = 0.0;
	for (std::size_t i = 0; i < n; ++i)
	{
		const double x = pts[i][0] - mx;
		const double y = pts[i][1] - my;
		cxx += x * x;
		cxy += x * y;
		cyy += y * y;
	}
	cxx /= static_cast<double>(n);
	cxy /= static_cast<double>(n);
	cyy /= static_cast<double>(n);

	// 特征值分解
	const double trace = cxx + cyy;
	const double det = cxx * cyy - cxy * cxy;
	const double disc = std::sqrt(std::max(0.0, trace * trace * 0.25 - det));
	const double lambda1 = trace * 0.5 + disc;
	const double lambda2 = trace * 0.5 - disc;

	outSemiMajor = std::sqrt(std::max(0.0, lambda1)) * 2.0;
	outSemiMinor = std::sqrt(std::max(0.0, lambda2)) * 2.0;
	outCx = mx;
	outCy = my;

	// 旋转角
	if (std::fabs(cxy) > 1e-12)
	{
		outRotationRad = std::atan2(2.0 * cxy, cxx - cyy) * 0.5;
	}
	else
	{
		outRotationRad = (cxx >= cyy) ? 0.0 : kPi * 0.5;
	}

	// 确保 semiMajor >= semiMinor
	if (outSemiMajor < outSemiMinor)
	{
		std::swap(outSemiMajor, outSemiMinor);
		outRotationRad += kPi * 0.5;
	}

	return outSemiMajor > 1e-6;
}

Vec3 computeConvexHullCentroid2D(const std::vector<std::array<double, 2>>& pts)
{
	if (pts.empty())
	{
		return {0.0, 0.0, 0.0};
	}
	double cx = 0.0, cy = 0.0;
	for (const auto& p : pts)
	{
		cx += p[0];
		cy += p[1];
	}
	const double inv = 1.0 / static_cast<double>(pts.size());
	return {cx * inv, cy * inv, 0.0};
}

bool analyzeCrossSection(const IndexedMeshLite& mesh, const std::vector<int>& neighborhood, const Vec3& localAxis,
						 const SectionFitMode fitMode, double& outSemiMajor, double& outSemiMinor,
						 double& outRotationDeg, Vec3& outCenter)
{
	if (neighborhood.size() < 3)
	{
		return false;
	}

	// 构建切平面基向量
	Vec3 n0 = normalizeVec3(cross(localAxis, Vec3{0.0, 0.0, 1.0}));
	if (length(n0) < 1e-6)
	{
		n0 = normalizeVec3(cross(localAxis, Vec3{0.0, 1.0, 0.0}));
	}
	const Vec3 b0 = normalizeVec3(cross(localAxis, n0));

	// 投影到切平面
	Vec3 centerSum{0.0, 0.0, 0.0};
	for (const int f : neighborhood)
	{
		centerSum = add(centerSum, mesh.faceCentroids[static_cast<std::size_t>(f)]);
	}
	const Vec3 sliceCenter = scale(centerSum, 1.0 / static_cast<double>(neighborhood.size()));

	std::vector<std::array<double, 2>> projPts;
	projPts.reserve(neighborhood.size());
	for (const int f : neighborhood)
	{
		const Vec3 d = sub(mesh.faceCentroids[static_cast<std::size_t>(f)], sliceCenter);
		const double u = dot(d, n0);
		const double v = dot(d, b0);
		projPts.push_back({u, v});
	}

	if (fitMode == SectionFitMode::Ellipse)
	{
		double cx, cy, rotRad;
		if (fitEllipse2D(projPts, outSemiMajor, outSemiMinor, cx, cy, rotRad))
		{
			outCenter = add(add(sliceCenter, scale(n0, cx)), scale(b0, cy));
			outRotationDeg = rotRad * 180.0 / kPi;
			return true;
		}
	}

	// 回退：凸包中心 + 圆近似
	const Vec3 centroid2d = computeConvexHullCentroid2D(projPts);
	outCenter = add(add(sliceCenter, scale(n0, centroid2d.x)), scale(b0, centroid2d.y));

	// 计算平均半径
	double radiusSum = 0.0;
	for (const auto& p : projPts)
	{
		const double dx = p[0] - centroid2d.x;
		const double dy = p[1] - centroid2d.y;
		radiusSum += std::sqrt(dx * dx + dy * dy);
	}
	const double avgRadius = radiusSum / static_cast<double>(projPts.size());
	outSemiMajor = avgRadius;
	outSemiMinor = avgRadius;
	outRotationDeg = 0.0;
	return true;
}

int runDbscanEnhanced(const std::vector<Vec3>& spatialPoints, const std::vector<double>& semiMajorValues,
					  const std::vector<double>& semiMinorValues, const double eps, const int minPts,
					  const double featureScale, std::vector<int>& outLabels)
{
	const int n = static_cast<int>(spatialPoints.size());
	outLabels.assign(static_cast<std::size_t>(n), -1);
	if (n != static_cast<int>(semiMajorValues.size()) || n != static_cast<int>(semiMinorValues.size()))
	{
		return 0;
	}

	int clusterId = 0;
	const double eps2 = eps * eps;

	auto regionQuery = [&](const int idx)
	{
		std::vector<int> neighbors;
		const Vec3& p = spatialPoints[static_cast<std::size_t>(idx)];
		const double a0 = semiMajorValues[static_cast<std::size_t>(idx)];
		const double b0 = semiMinorValues[static_cast<std::size_t>(idx)];
		for (int j = 0; j < n; ++j)
		{
			const Vec3 d = sub(spatialPoints[static_cast<std::size_t>(j)], p);
			const double spatialDist2 = dot(d, d);
			const double featureDist2 = featureScale * featureScale *
										((semiMajorValues[static_cast<std::size_t>(j)] - a0) *
											 (semiMajorValues[static_cast<std::size_t>(j)] - a0) +
										 (semiMinorValues[static_cast<std::size_t>(j)] - b0) *
											 (semiMinorValues[static_cast<std::size_t>(j)] - b0));
			if (spatialDist2 + featureDist2 <= eps2)
			{
				neighbors.push_back(j);
			}
		}
		return neighbors;
	};

	for (int i = 0; i < n; ++i)
	{
		if (outLabels[static_cast<std::size_t>(i)] != -1)
		{
			continue;
		}
		const std::vector<int> neighbors = regionQuery(i);
		if (static_cast<int>(neighbors.size()) < minPts)
		{
			outLabels[static_cast<std::size_t>(i)] = -2;
			continue;
		}
		outLabels[static_cast<std::size_t>(i)] = clusterId;
		std::queue<int> q;
		for (const int nb : neighbors)
		{
			if (nb != i)
			{
				q.push(nb);
			}
		}
		while (!q.empty())
		{
			const int j = q.front();
			q.pop();
			if (outLabels[static_cast<std::size_t>(j)] == -2)
			{
				outLabels[static_cast<std::size_t>(j)] = clusterId;
			}
			if (outLabels[static_cast<std::size_t>(j)] != -1)
			{
				continue;
			}
			outLabels[static_cast<std::size_t>(j)] = clusterId;
			const std::vector<int> nbs = regionQuery(j);
			if (static_cast<int>(nbs.size()) >= minPts)
			{
				for (const int nb : nbs)
				{
					if (outLabels[static_cast<std::size_t>(nb)] == -1 || outLabels[static_cast<std::size_t>(nb)] == -2)
					{
						q.push(nb);
					}
				}
			}
		}
		++clusterId;
	}
	return clusterId;
}

std::vector<int> detectTransitionZones(const IndexedMeshLite& mesh, const std::vector<TubularCrossSectionRing>& rings,
									   const double aspectRatioChangeThreshold,
									   const double curvatureChangeThresholdDeg)
{
	std::vector<int> transitionFaces;
	if (rings.size() < 2)
	{
		return transitionFaces;
	}

	for (std::size_t i = 1; i < rings.size(); ++i)
	{
		const auto& prev = rings[i - 1];
		const auto& curr = rings[i];

		// 长短轴比突变
		const double arChange = std::fabs(curr.aspectRatio - prev.aspectRatio);
		if (arChange > aspectRatioChangeThreshold)
		{
			for (const int f : curr.faceIndices)
			{
				transitionFaces.push_back(f);
			}
			continue;
		}

		// 中心线曲率突变
		const double angle = axisAngleDeg({curr.axisHint[0], curr.axisHint[1], curr.axisHint[2]},
										  {prev.axisHint[0], prev.axisHint[1], prev.axisHint[2]});
		if (angle > curvatureChangeThresholdDeg)
		{
			for (const int f : curr.faceIndices)
			{
				transitionFaces.push_back(f);
			}
		}
	}

	// 去重
	std::sort(transitionFaces.begin(), transitionFaces.end());
	transitionFaces.erase(std::unique(transitionFaces.begin(), transitionFaces.end()), transitionFaces.end());
	return transitionFaces;
}

bool smoothCenterlineIterative(std::vector<TubularCenterlineSample>& samples, const int maxIterations,
							   const double convergenceEpsilonMm)
{
	if (samples.size() < 4 || maxIterations <= 0)
	{
		return true;
	}

	for (int iter = 0; iter < maxIterations; ++iter)
	{
		double maxDisplacement = 0.0;

		// 对内部点进行平滑（保留端点不变）
		for (std::size_t i = 1; i + 1 < samples.size(); ++i)
		{
			const auto& prev = samples[i - 1];
			const auto& curr = samples[i];
			const auto& next = samples[i + 1];

			// 三点平均平滑
			const double newX = (prev.positionMm[0] + curr.positionMm[0] + next.positionMm[0]) / 3.0;
			const double newY = (prev.positionMm[1] + curr.positionMm[1] + next.positionMm[1]) / 3.0;
			const double newZ = (prev.positionMm[2] + curr.positionMm[2] + next.positionMm[2]) / 3.0;

			const double dx = newX - curr.positionMm[0];
			const double dy = newY - curr.positionMm[1];
			const double dz = newZ - curr.positionMm[2];
			const double disp = std::sqrt(dx * dx + dy * dy + dz * dz);
			maxDisplacement = std::max(maxDisplacement, disp);

			samples[i].positionMm[0] = newX;
			samples[i].positionMm[1] = newY;
			samples[i].positionMm[2] = newZ;
		}

		// 重新计算弧长和切线
		double arc = 0.0;
		for (std::size_t i = 0; i < samples.size(); ++i)
		{
			if (i > 0)
			{
				const double dx = samples[i].positionMm[0] - samples[i - 1].positionMm[0];
				const double dy = samples[i].positionMm[1] - samples[i - 1].positionMm[1];
				const double dz = samples[i].positionMm[2] - samples[i - 1].positionMm[2];
				arc += std::sqrt(dx * dx + dy * dy + dz * dz);
			}
			samples[i].arcLengthMm = arc;

			if (i > 0 && i + 1 < samples.size())
			{
				const Vec3 ta{samples[i].positionMm[0] - samples[i - 1].positionMm[0],
							  samples[i].positionMm[1] - samples[i - 1].positionMm[1],
							  samples[i].positionMm[2] - samples[i - 1].positionMm[2]};
				const Vec3 tb{samples[i + 1].positionMm[0] - samples[i].positionMm[0],
							  samples[i + 1].positionMm[1] - samples[i].positionMm[1],
							  samples[i + 1].positionMm[2] - samples[i].positionMm[2]};
				const Vec3 t = normalizeVec3(add(ta, tb));
				samples[i].tangent[0] = t.x;
				samples[i].tangent[1] = t.y;
				samples[i].tangent[2] = t.z;
			}
		}

		if (maxDisplacement < convergenceEpsilonMm)
		{
			return true;
		}
	}

	// 重建 Frenet 标架
	buildFrenetFrames(samples, samples);
	return true;
}

double ellipseCurvature(const double semiMajor, const double semiMinor, const double tRad)
{
	const double a2 = semiMajor * semiMajor;
	const double b2 = semiMinor * semiMinor;
	const double sinT = std::sin(tRad);
	const double cosT = std::cos(tRad);
	const double denom = std::pow(a2 * sinT * sinT + b2 * cosT * cosT, 1.5);
	if (denom < 1e-12)
	{
		return 0.0;
	}
	return (semiMajor * semiMinor) / denom;
}

std::vector<double> computeAnisotropicAngleSamples(const double semiMajor, const double semiMinor,
												   const int targetPointCount)
{
	if (targetPointCount < 4)
	{
		return {0.0, kPi * 0.5, kPi, kPi * 1.5};
	}

	// 根据曲率自适应采样
	std::vector<double> angles;
	angles.reserve(static_cast<std::size_t>(targetPointCount));

	const double dt = 2.0 * kPi / static_cast<double>(targetPointCount * 4); // 过采样
	double totalWeight = 0.0;
	std::vector<double> weights;

	for (double t = 0.0; t < 2.0 * kPi; t += dt)
	{
		const double k = ellipseCurvature(semiMajor, semiMinor, t);
		const double w = std::pow(k, 0.5); // 曲率平方根作为权重
		weights.push_back(w);
		totalWeight += w;
	}

	// 累积分布采样
	double cumWeight = 0.0;
	const double step = totalWeight / static_cast<double>(targetPointCount);
	std::size_t idx = 0;
	for (int i = 0; i < targetPointCount; ++i)
	{
		const double target = step * static_cast<double>(i);
		while (idx < weights.size() && cumWeight < target)
		{
			cumWeight += weights[idx];
			++idx;
		}
		const double t = dt * static_cast<double>(std::min(idx, weights.size() - 1));
		angles.push_back(std::fmod(t, 2.0 * kPi));
	}

	return angles;
}

Vec3 computeSectionNormal(const double semiMajor, const double semiMinor, const double sectionRotationDeg,
						  const double tRad, const Vec3& normalAxis, const Vec3& binormalAxis)
{
	// 椭圆参数方程导数：(-a*sin(t), b*cos(t))
	// 法线方向：(b*cos(t), a*sin(t)) 归一化
	const double rotRad = sectionRotationDeg * kPi / 180.0;
	const double cosR = std::cos(rotRad);
	const double sinR = std::sin(rotRad);

	const double nx = semiMinor * std::cos(tRad);
	const double ny = semiMajor * std::sin(tRad);

	// 旋转到世界坐标
	const double wx = nx * cosR - ny * sinR;
	const double wy = nx * sinR + ny * cosR;

	return normalizeVec3(add(scale(normalAxis, wx), scale(binormalAxis, wy)));
}

double computeEllipseFittingResiduals(const std::vector<std::array<double, 2>>& pts, const double semiMajor,
									  const double semiMinor, const double cx, const double cy,
									  const double rotationRad, std::vector<double>& outResiduals)
{
	outResiduals.clear();
	if (pts.empty() || semiMajor < 1e-6 || semiMinor < 1e-6)
	{
		return 0.0;
	}
	outResiduals.reserve(pts.size());

	const double cosR = std::cos(-rotationRad);
	const double sinR = std::sin(-rotationRad);
	const double invA = 1.0 / semiMajor;
	const double invB = 1.0 / semiMinor;

	double sumSq = 0.0;
	for (const auto& p : pts)
	{
		// 变换到椭圆局部坐标系（去除平移和旋转）
		const double dx = p[0] - cx;
		const double dy = p[1] - cy;
		const double lx = dx * cosR - dy * sinR;
		const double ly = dx * sinR + dy * cosR;

		// 归一化到单位圆坐标
		const double nx = lx * invA;
		const double ny = ly * invB;

		// 点到单位圆的距离（单位圆参数化：cos(t), sin(t)）
		// 找最近点：t = atan2(ny, nx)
		const double distToOrigin = std::sqrt(nx * nx + ny * ny);
		if (distToOrigin < 1e-12)
		{
			// 点在中心，残差 = min(a, b)
			const double residual = std::min(semiMajor, semiMinor);
			outResiduals.push_back(residual);
			sumSq += residual * residual;
			continue;
		}

		// 单位圆上最近点
		const double t = std::atan2(ny, nx);
		const double ec = std::cos(t);
		const double es = std::sin(t);

		// 最近点在原始坐标系中的位置
		const double nearX = semiMajor * ec;
		const double nearY = semiMinor * es;

		// 旋转回世界坐标
		const double cosRinv = std::cos(rotationRad);
		const double sinRinv = std::sin(rotationRad);
		const double wnx = nearX * cosRinv - nearY * sinRinv + cx;
		const double wny = nearX * sinRinv + nearY * cosRinv + cy;

		const double residual = std::sqrt((p[0] - wnx) * (p[0] - wnx) + (p[1] - wny) * (p[1] - wny));
		outResiduals.push_back(residual);
		sumSq += residual * residual;
	}

	return std::sqrt(sumSq / static_cast<double>(pts.size()));
}

// === 拉普拉斯收缩骨架提取实现 ===

int countWeldedVertices(const IndexedMeshLite& mesh)
{
	int maxIdx = -1;
	for (const auto& face : mesh.faceVerts)
	{
		for (int corner = 0; corner < 3; ++corner)
		{
			maxIdx = std::max(maxIdx, face[static_cast<std::size_t>(corner)]);
		}
	}
	return maxIdx + 1;
}

std::vector<std::vector<int>> buildVertexAdjacency(const IndexedMeshLite& mesh, const int /*kNeighbors*/)
{
	const int vertexCount = countWeldedVertices(mesh);
	std::vector<std::vector<int>> adj(static_cast<std::size_t>(vertexCount));

	const auto addEdge = [&adj, vertexCount](const int a, const int b)
	{
		if (a == b || a < 0 || b < 0 || a >= vertexCount || b >= vertexCount)
		{
			return;
		}
		adj[static_cast<std::size_t>(a)].push_back(b);
		adj[static_cast<std::size_t>(b)].push_back(a);
	};

	for (const auto& face : mesh.faceVerts)
	{
		addEdge(face[0], face[1]);
		addEdge(face[1], face[2]);
		addEdge(face[2], face[0]);
	}

	for (auto& neighbors : adj)
	{
		std::sort(neighbors.begin(), neighbors.end());
		neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
	}

	return adj;
}

std::vector<Vec3> computeLaplacianCoordinates(const std::vector<Vec3>& positions,
											  const std::vector<std::vector<int>>& adjacency)
{
	const int n = static_cast<int>(positions.size());
	std::vector<Vec3> lap(n, {0.0, 0.0, 0.0});
	for (int i = 0; i < n; ++i)
	{
		if (adjacency[i].empty())
		{
			continue;
		}
		Vec3 sum{0.0, 0.0, 0.0};
		for (int j : adjacency[i])
		{
			sum = add(sum, positions[j]);
		}
		const double inv = 1.0 / static_cast<double>(adjacency[i].size());
		const Vec3 mean = scale(sum, inv);
		lap[i] = sub(mean, positions[i]);
	}
	return lap;
}

void contractVerticesIterative(std::vector<Vec3>& positions, const std::vector<Vec3>& originalPositions,
							   const std::vector<std::vector<int>>& adjacency, const int iterations,
							   const double weightStart, const double weightEnd)
{
	const int n = static_cast<int>(positions.size());
	if (n <= 0 || static_cast<int>(originalPositions.size()) != n)
	{
		return;
	}

	for (int it = 0; it < iterations; ++it)
	{
		const double t = (iterations <= 1) ? 1.0 : static_cast<double>(it) / static_cast<double>(iterations - 1);
		const double w = weightStart + t * (weightEnd - weightStart);
		std::vector<Vec3> next(static_cast<std::size_t>(n));
		for (int i = 0; i < n; ++i)
		{
			const auto& neighbors = adjacency[static_cast<std::size_t>(i)];
			if (neighbors.empty())
			{
				next[static_cast<std::size_t>(i)] = positions[static_cast<std::size_t>(i)];
				continue;
			}
			Vec3 neighborSum{0.0, 0.0, 0.0};
			for (const int j : neighbors)
			{
				neighborSum = add(neighborSum, positions[static_cast<std::size_t>(j)]);
			}
			const double degree = static_cast<double>(neighbors.size());
			const Vec3 numerator = add(neighborSum, scale(originalPositions[static_cast<std::size_t>(i)], w));
			next[static_cast<std::size_t>(i)] = scale(numerator, 1.0 / (degree + w));
		}
		positions = std::move(next);
	}
}

namespace
{
struct SkeletonGraph
{
	std::vector<Vec3> positions;
	std::vector<Vec3> anchors;
	std::vector<std::array<int, 3>> faces;
	std::vector<std::pair<int, int>> edges;
	std::vector<std::vector<int>> adjacency;

	void rebuildEdgesFromFaces()
	{
		edges.clear();
		edges.reserve(faces.size() * 3U);
		const auto addEdge = [this](const int a, const int b)
		{
			if (a == b || a < 0 || b < 0 || a >= static_cast<int>(positions.size()) ||
				b >= static_cast<int>(positions.size()))
			{
				return;
			}
			const int lo = std::min(a, b);
			const int hi = std::max(a, b);
			edges.emplace_back(lo, hi);
		};
		for (const auto& face : faces)
		{
			addEdge(face[0], face[1]);
			addEdge(face[1], face[2]);
			addEdge(face[2], face[0]);
		}
		std::sort(edges.begin(), edges.end());
		edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
		rebuildAdjacency();
	}

	void rebuildAdjacency()
	{
		adjacency.assign(positions.size(), {});
		for (const auto& edge : edges)
		{
			const int a = edge.first;
			const int b = edge.second;
			if (a < 0 || b < 0 || a >= static_cast<int>(positions.size()) || b >= static_cast<int>(positions.size()))
			{
				continue;
			}
			adjacency[static_cast<std::size_t>(a)].push_back(b);
			adjacency[static_cast<std::size_t>(b)].push_back(a);
		}
		for (auto& neighbors : adjacency)
		{
			std::sort(neighbors.begin(), neighbors.end());
			neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
		}
	}

	double averageEdgeLength() const
	{
		if (edges.empty())
		{
			return 0.0;
		}
		double sum = 0.0;
		for (const auto& edge : edges)
		{
			sum += length(
				sub(positions[static_cast<std::size_t>(edge.second)], positions[static_cast<std::size_t>(edge.first)]));
		}
		return sum / static_cast<double>(edges.size());
	}

	void remapFacesAfterVertexRemoval(const int removedIdx, const int lastIdx)
	{
		std::vector<std::array<int, 3>> nextFaces;
		nextFaces.reserve(faces.size());
		for (const auto& face : faces)
		{
			std::array<int, 3> mapped = face;
			for (int& vid : mapped)
			{
				if (vid == removedIdx)
				{
					vid = -1;
				}
				else if (vid == lastIdx)
				{
					vid = removedIdx;
				}
			}
			if (mapped[0] < 0 || mapped[1] < 0 || mapped[2] < 0)
			{
				continue;
			}
			if (mapped[0] == mapped[1] || mapped[1] == mapped[2] || mapped[0] == mapped[2])
			{
				continue;
			}
			nextFaces.push_back(mapped);
		}
		faces = std::move(nextFaces);
	}

	void remapFacesAfterMerge(const int remove, const int last, const int survivorIdx)
	{
		const auto remapIndex = [remove, last, survivorIdx](const int idx)
		{
			if (idx == remove)
			{
				return survivorIdx;
			}
			if (idx == last)
			{
				return remove;
			}
			return idx;
		};

		std::vector<std::array<int, 3>> nextFaces;
		nextFaces.reserve(faces.size());
		for (const auto& face : faces)
		{
			const int a = remapIndex(face[0]);
			const int b = remapIndex(face[1]);
			const int c = remapIndex(face[2]);
			if (a == b || b == c || a == c)
			{
				continue;
			}
			nextFaces.push_back({a, b, c});
		}
		faces = std::move(nextFaces);
	}

	void mergeVertices(const int u, const int v)
	{
		if (u == v || u < 0 || v < 0 || u >= static_cast<int>(positions.size()) ||
			v >= static_cast<int>(positions.size()))
		{
			return;
		}
		const int survivorIdx = std::min(u, v);
		const int remove = std::max(u, v);
		positions[static_cast<std::size_t>(survivorIdx)] = scale(
			add(positions[static_cast<std::size_t>(survivorIdx)], positions[static_cast<std::size_t>(remove)]), 0.5);

		const int last = static_cast<int>(positions.size()) - 1;
		if (remove != last)
		{
			positions[static_cast<std::size_t>(remove)] = positions[static_cast<std::size_t>(last)];
			anchors[static_cast<std::size_t>(remove)] = anchors[static_cast<std::size_t>(last)];
		}
		positions.pop_back();
		anchors.pop_back();

		// anchor 跟随收缩后位置，避免被拉回原始表面
		anchors[static_cast<std::size_t>(survivorIdx)] = positions[static_cast<std::size_t>(survivorIdx)];

		remapFacesAfterMerge(remove, last, survivorIdx);

		const auto remapIndex = [remove, last, survivorIdx](const int idx)
		{
			if (idx == remove)
			{
				return survivorIdx;
			}
			if (idx == last)
			{
				return remove;
			}
			return idx;
		};

		std::vector<std::pair<int, int>> nextEdges;
		nextEdges.reserve(edges.size());
		for (const auto& meshEdge : edges)
		{
			int a = remapIndex(meshEdge.first);
			int b = remapIndex(meshEdge.second);
			if (a > b)
			{
				std::swap(a, b);
			}
			if (a == b)
			{
				continue;
			}
			nextEdges.emplace_back(a, b);
		}
		std::sort(nextEdges.begin(), nextEdges.end());
		nextEdges.erase(std::unique(nextEdges.begin(), nextEdges.end()), nextEdges.end());
		edges = std::move(nextEdges);
		rebuildAdjacency();
	}

	bool collapseShortestEdge(const double maxLength)
	{
		int bestU = -1;
		int bestV = -1;
		double bestLen = maxLength;
		for (const auto& edge : edges)
		{
			const double edgeLen = length(
				sub(positions[static_cast<std::size_t>(edge.second)], positions[static_cast<std::size_t>(edge.first)]));
			if (edgeLen < bestLen)
			{
				bestLen = edgeLen;
				bestU = edge.first;
				bestV = edge.second;
			}
		}
		if (bestU < 0)
		{
			return false;
		}
		mergeVertices(bestU, bestV);
		return true;
	}

	void collapseAllBelowLength(const double maxLength, const int maxPasses)
	{
		for (int pass = 0; pass < maxPasses; ++pass)
		{
			if (!collapseShortestEdge(maxLength))
			{
				break;
			}
		}
	}

	void removeDegenerateFaces(const double minArea, const double minEdgeLength)
	{
		if (faces.empty())
		{
			return;
		}
		std::vector<std::array<int, 3>> kept;
		kept.reserve(faces.size());
		for (const auto& face : faces)
		{
			const Vec3& p0 = positions[static_cast<std::size_t>(face[0])];
			const Vec3& p1 = positions[static_cast<std::size_t>(face[1])];
			const Vec3& p2 = positions[static_cast<std::size_t>(face[2])];
			const double e0 = length(sub(p1, p0));
			const double e1 = length(sub(p2, p1));
			const double e2 = length(sub(p0, p2));
			if (e0 < minEdgeLength || e1 < minEdgeLength || e2 < minEdgeLength)
			{
				continue;
			}
			const double area = 0.5 * length(cross(sub(p1, p0), sub(p2, p0)));
			if (area < minArea)
			{
				continue;
			}
			kept.push_back(face);
		}
		if (kept.size() == faces.size())
		{
			return;
		}
		faces = std::move(kept);
		rebuildEdgesFromFaces();
	}
};

bool buildSkeletonGraphFromMesh(const IndexedMeshLite& mesh, SkeletonGraph& outGraph)
{
	const int vertexCount = countWeldedVertices(mesh);
	if (vertexCount <= 0)
	{
		return false;
	}

	outGraph.positions.assign(static_cast<std::size_t>(vertexCount), Vec3{});
	for (int f = 0; f < mesh.faceCount; ++f)
	{
		const std::size_t base = static_cast<std::size_t>(f) * 9U;
		const auto& face = mesh.faceVerts[static_cast<std::size_t>(f)];
		outGraph.positions[static_cast<std::size_t>(face[0])] = {mesh.soup[base + 0], mesh.soup[base + 1],
																 mesh.soup[base + 2]};
		outGraph.positions[static_cast<std::size_t>(face[1])] = {mesh.soup[base + 3], mesh.soup[base + 4],
																 mesh.soup[base + 5]};
		outGraph.positions[static_cast<std::size_t>(face[2])] = {mesh.soup[base + 6], mesh.soup[base + 7],
																 mesh.soup[base + 8]};
	}
	outGraph.anchors = outGraph.positions;

	outGraph.faces.clear();
	outGraph.faces.reserve(static_cast<std::size_t>(mesh.faceCount));
	for (const auto& face : mesh.faceVerts)
	{
		outGraph.faces.push_back(face);
	}
	outGraph.rebuildEdgesFromFaces();
	return !outGraph.edges.empty();
}

double meshBBoxDiagonal(const IndexedMeshLite& mesh)
{
	const Vec3 mn{mesh.bboxMin[0], mesh.bboxMin[1], mesh.bboxMin[2]};
	const Vec3 mx{mesh.bboxMax[0], mesh.bboxMax[1], mesh.bboxMax[2]};
	return length(sub(mx, mn));
}

void contractSkeletonGraphStep(SkeletonGraph& graph, const double anchorWeight)
{
	const int n = static_cast<int>(graph.positions.size());
	for (int i = 0; i < n; ++i)
	{
		const auto& neighbors = graph.adjacency[static_cast<std::size_t>(i)];
		if (neighbors.empty())
		{
			continue;
		}
		Vec3 neighborSum{0.0, 0.0, 0.0};
		for (const int j : neighbors)
		{
			neighborSum = add(neighborSum, graph.positions[static_cast<std::size_t>(j)]);
		}
		const double degree = static_cast<double>(neighbors.size());
		if (anchorWeight <= 0.0)
		{
			graph.positions[static_cast<std::size_t>(i)] = scale(neighborSum, 1.0 / degree);
		}
		else
		{
			const Vec3 numerator = add(neighborSum, scale(graph.anchors[static_cast<std::size_t>(i)], anchorWeight));
			graph.positions[static_cast<std::size_t>(i)] = scale(numerator, 1.0 / (degree + anchorWeight));
		}
	}
}

double detailComputeContractionAnchorWeight(const int iteration, const int totalIterations, const double weightStart,
											const double weightPeak)
{
	if (totalIterations <= 1)
	{
		return weightPeak;
	}
	const int anchorPhaseEnd = std::max(1, totalIterations * 6 / 10);
	if (iteration < anchorPhaseEnd)
	{
		const double phaseT = static_cast<double>(iteration) / static_cast<double>(anchorPhaseEnd);
		const double logStart = std::log(std::max(weightStart, 1e-6));
		const double logPeak = std::log(std::max(weightPeak, 1e-6));
		return std::exp(logStart + phaseT * (logPeak - logStart));
	}
	const double phaseT = static_cast<double>(iteration - anchorPhaseEnd) /
						  static_cast<double>(std::max(1, totalIterations - anchorPhaseEnd));
	return weightPeak * (1.0 - phaseT);
}

void pruneShortLeafBranches(SkeletonGraph& graph, const double minBranchLength)
{
	if (graph.positions.size() < 3U || minBranchLength <= 0.0)
	{
		return;
	}

	bool changed = true;
	while (changed && graph.positions.size() > 2U)
	{
		changed = false;
		graph.rebuildAdjacency();
		for (int i = 0; i < static_cast<int>(graph.positions.size()); ++i)
		{
			const auto& neighbors = graph.adjacency[static_cast<std::size_t>(i)];
			if (neighbors.size() != 1U)
			{
				continue;
			}
			const int parent = neighbors.front();
			const double edgeLen = length(
				sub(graph.positions[static_cast<std::size_t>(i)], graph.positions[static_cast<std::size_t>(parent)]));
			if (edgeLen >= minBranchLength)
			{
				continue;
			}

			const int last = static_cast<int>(graph.positions.size()) - 1;
			if (i != last)
			{
				graph.positions[static_cast<std::size_t>(i)] = graph.positions[static_cast<std::size_t>(last)];
				graph.anchors[static_cast<std::size_t>(i)] = graph.anchors[static_cast<std::size_t>(last)];
			}
			graph.positions.pop_back();
			graph.anchors.pop_back();
			graph.remapFacesAfterVertexRemoval(i, last);

			const auto remapIndex = [i, last](int idx)
			{
				if (idx == i)
				{
					return -1;
				}
				if (idx == last)
				{
					return i;
				}
				return idx;
			};

			std::vector<std::pair<int, int>> nextEdges;
			nextEdges.reserve(graph.edges.size());
			for (const auto& meshEdge : graph.edges)
			{
				int a = remapIndex(meshEdge.first);
				int b = remapIndex(meshEdge.second);
				if (a < 0 || b < 0 || a == b)
				{
					continue;
				}
				if (a > b)
				{
					std::swap(a, b);
				}
				nextEdges.emplace_back(a, b);
			}
			std::sort(nextEdges.begin(), nextEdges.end());
			nextEdges.erase(std::unique(nextEdges.begin(), nextEdges.end()), nextEdges.end());
			graph.edges = std::move(nextEdges);
			graph.rebuildAdjacency();
			changed = true;
			break;
		}
	}
}

int bfsFarthestVertex(const std::vector<std::vector<int>>& adjacency, const int source, std::vector<int>& outHopDist)
{
	const int n = static_cast<int>(adjacency.size());
	outHopDist.assign(static_cast<std::size_t>(n), -1);
	std::vector<int> queue;
	queue.push_back(source);
	outHopDist[static_cast<std::size_t>(source)] = 0;
	int farthest = source;
	for (std::size_t head = 0; head < queue.size(); ++head)
	{
		const int u = queue[head];
		for (const int v : adjacency[static_cast<std::size_t>(u)])
		{
			if (outHopDist[static_cast<std::size_t>(v)] >= 0)
			{
				continue;
			}
			outHopDist[static_cast<std::size_t>(v)] = outHopDist[static_cast<std::size_t>(u)] + 1;
			queue.push_back(v);
			if (outHopDist[static_cast<std::size_t>(v)] > outHopDist[static_cast<std::size_t>(farthest)])
			{
				farthest = v;
			}
		}
	}
	return farthest;
}

std::vector<int> dijkstraVertexPath(const std::vector<Vec3>& positions, const std::vector<std::vector<int>>& adjacency,
									const int start, const int end)
{
	const int n = static_cast<int>(adjacency.size());
	constexpr double kInf = 1e100;
	std::vector<double> dist(static_cast<std::size_t>(n), kInf);
	std::vector<int> prev(static_cast<std::size_t>(n), -1);
	using Node = std::pair<double, int>;
	std::priority_queue<Node, std::vector<Node>, std::greater<Node>> heap;
	dist[static_cast<std::size_t>(start)] = 0.0;
	heap.push({0.0, start});
	while (!heap.empty())
	{
		const auto [d, u] = heap.top();
		heap.pop();
		if (d > dist[static_cast<std::size_t>(u)])
		{
			continue;
		}
		if (u == end)
		{
			break;
		}
		for (const int v : adjacency[static_cast<std::size_t>(u)])
		{
			const double edgeLen =
				length(sub(positions[static_cast<std::size_t>(v)], positions[static_cast<std::size_t>(u)]));
			const double nextDist = d + edgeLen;
			if (nextDist < dist[static_cast<std::size_t>(v)])
			{
				dist[static_cast<std::size_t>(v)] = nextDist;
				prev[static_cast<std::size_t>(v)] = u;
				heap.push({nextDist, v});
			}
		}
	}
	std::vector<int> path;
	if (dist[static_cast<std::size_t>(end)] >= kInf * 0.5)
	{
		return path;
	}
	for (int at = end; at >= 0; at = prev[static_cast<std::size_t>(at)])
	{
		path.push_back(at);
	}
	std::reverse(path.begin(), path.end());
	return path;
}

std::array<double, 3> vecToArray(const Vec3& v)
{
	return {v.x, v.y, v.z};
}

void assignSampleTangentFromPolyline(TubularCenterlineSample& sample, const std::vector<Vec3>& polyline,
									 const Vec3& position, const std::size_t vertexHint)
{
	Vec3 dir{1.0, 0.0, 0.0};
	if (vertexHint + 1U < polyline.size())
	{
		dir = sub(polyline[vertexHint + 1U], polyline[vertexHint]);
	}
	else if (vertexHint > 0U)
	{
		dir = sub(polyline[vertexHint], polyline[vertexHint - 1U]);
	}
	else if (polyline.size() >= 2U)
	{
		dir = sub(polyline.back(), polyline.front());
	}
	(void)position;
	const Vec3 n = normalizeVec3(dir);
	if (length(n) >= 1e-9)
	{
		sample.tangent = {n.x, n.y, n.z};
	}
}

void detailResamplePolylineToSamples(const std::vector<Vec3>& polyline, const double spacingMm,
									 std::vector<TubularCenterlineSample>& outSamples)
{
	outSamples.clear();
	if (polyline.empty() || spacingMm <= 0.0)
	{
		return;
	}

	TubularCenterlineSample first;
	first.pipeId = 0;
	first.arcLengthMm = 0.0;
	first.radiusMm = 1.0;
	first.positionMm = vecToArray(polyline.front());
	assignSampleTangentFromPolyline(first, polyline, polyline.front(), 0U);
	outSamples.push_back(first);

	double arcLength = 0.0;
	double distSinceLast = 0.0;
	for (std::size_t i = 1; i < polyline.size(); ++i)
	{
		const Vec3 a = polyline[i - 1];
		const Vec3 b = polyline[i];
		const double segLen = length(sub(b, a));
		if (segLen < 1e-9)
		{
			continue;
		}
		double walked = 0.0;
		while (distSinceLast + (segLen - walked) >= spacingMm)
		{
			const double need = spacingMm - distSinceLast;
			walked += need;
			const double t = walked / segLen;
			const Vec3 p = add(a, scale(sub(b, a), t));
			arcLength += spacingMm;
			TubularCenterlineSample sample;
			sample.pipeId = 0;
			sample.arcLengthMm = arcLength;
			sample.radiusMm = 1.0;
			sample.positionMm = vecToArray(p);
			sample.tangent = vecToArray(normalizeVec3(sub(b, a)));
			outSamples.push_back(sample);
			distSinceLast = 0.0;
		}
		distSinceLast += segLen - walked;
	}

	if (outSamples.size() == 1U && polyline.size() >= 2U)
	{
		TubularCenterlineSample last = first;
		last.positionMm = vecToArray(polyline.back());
		last.arcLengthMm = length(sub(polyline.back(), polyline.front()));
		assignSampleTangentFromPolyline(last, polyline, polyline.back(), polyline.size() - 1U);
		outSamples.push_back(last);
	}
}

bool extractLongestPathPolyline(const SkeletonGraph& graph, std::vector<Vec3>& outPolyline)
{
	outPolyline.clear();
	if (graph.positions.size() < 2U)
	{
		return false;
	}

	int seed = -1;
	for (int i = 0; i < static_cast<int>(graph.adjacency.size()); ++i)
	{
		if (!graph.adjacency[static_cast<std::size_t>(i)].empty())
		{
			seed = i;
			break;
		}
	}
	if (seed < 0)
	{
		return false;
	}

	std::vector<int> hopDist;
	const int endpointA = bfsFarthestVertex(graph.adjacency, seed, hopDist);
	const int endpointB = bfsFarthestVertex(graph.adjacency, endpointA, hopDist);
	const std::vector<int> pathIndices = dijkstraVertexPath(graph.positions, graph.adjacency, endpointA, endpointB);
	if (pathIndices.size() < 2U)
	{
		return false;
	}

	outPolyline.reserve(pathIndices.size());
	for (const int idx : pathIndices)
	{
		outPolyline.push_back(graph.positions[static_cast<std::size_t>(idx)]);
	}
	return true;
}

Vec3 computePrincipalAxisFromPoints(const std::vector<Vec3>& points)
{
	if (points.empty())
	{
		return {1.0, 0.0, 0.0};
	}
	Vec3 mean{0.0, 0.0, 0.0};
	for (const Vec3& p : points)
	{
		mean = add(mean, p);
	}
	mean = scale(mean, 1.0 / static_cast<double>(points.size()));

	double cov[3][3] = {};
	for (const Vec3& p : points)
	{
		const Vec3 d = sub(p, mean);
		cov[0][0] += d.x * d.x;
		cov[0][1] += d.x * d.y;
		cov[0][2] += d.x * d.z;
		cov[1][1] += d.y * d.y;
		cov[1][2] += d.y * d.z;
		cov[2][2] += d.z * d.z;
	}
	cov[1][0] = cov[0][1];
	cov[2][0] = cov[0][2];
	cov[2][1] = cov[1][2];

	Vec3 axis{1.0, 0.0, 0.0};
	for (int iter = 0; iter < 24; ++iter)
	{
		const Vec3 next{cov[0][0] * axis.x + cov[0][1] * axis.y + cov[0][2] * axis.z,
						cov[1][0] * axis.x + cov[1][1] * axis.y + cov[1][2] * axis.z,
						cov[2][0] * axis.x + cov[2][1] * axis.y + cov[2][2] * axis.z};
		axis = normalizeVec3(next);
	}
	if (length(axis) < 1e-9)
	{
		axis = {1.0, 0.0, 0.0};
	}
	return axis;
}

bool computeCenterlinePcaFromPoints(const std::vector<Vec3>& points, Vec3& outCentroid, Vec3& outAxis,
									double& outExtentMin, double& outExtentMax)
{
	outCentroid = {0.0, 0.0, 0.0};
	outAxis = {1.0, 0.0, 0.0};
	outExtentMin = 0.0;
	outExtentMax = 0.0;
	if (points.size() < 3U)
	{
		return false;
	}

	for (const Vec3& p : points)
	{
		outCentroid = add(outCentroid, p);
	}
	outCentroid = scale(outCentroid, 1.0 / static_cast<double>(points.size()));
	outAxis = computePrincipalAxisFromPoints(points);

	double tMin = std::numeric_limits<double>::max();
	double tMax = -std::numeric_limits<double>::max();
	for (const Vec3& p : points)
	{
		const double t = dot(sub(p, outCentroid), outAxis);
		tMin = std::min(tMin, t);
		tMax = std::max(tMax, t);
	}
	outExtentMin = tMin;
	outExtentMax = tMax;
	return tMax > tMin + 1e-9;
}

void fillCenterlinePcaAxis(const Vec3& centroid, const Vec3& axis, const double extentMin, const double extentMax,
						   TubularCenterlinePcaAxis& outPca)
{
	outPca.centroidMm = {centroid.x, centroid.y, centroid.z};
	outPca.axis = {axis.x, axis.y, axis.z};
	outPca.extentMinMm = extentMin;
	outPca.extentMaxMm = extentMax;
	outPca.valid = true;
}

bool extractCenterlineBySliceCentroids(const std::vector<Vec3>& points, const double binWidthMm,
									   std::vector<Vec3>& outPolyline)
{
	outPolyline.clear();
	if (points.size() < 3U || binWidthMm <= 0.0)
	{
		return false;
	}

	Vec3 mean{0.0, 0.0, 0.0};
	Vec3 axis{1.0, 0.0, 0.0};
	double tMin = 0.0;
	double tMax = 0.0;
	if (!computeCenterlinePcaFromPoints(points, mean, axis, tMin, tMax))
	{
		return false;
	}

	std::vector<std::pair<double, Vec3>> projected;
	projected.reserve(points.size());
	for (const Vec3& p : points)
	{
		projected.emplace_back(dot(sub(p, mean), axis), p);
	}
	std::sort(projected.begin(), projected.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

	if (tMax - tMin < binWidthMm * 0.5)
	{
		return false;
	}

	for (double binStart = tMin; binStart <= tMax + 1e-9; binStart += binWidthMm)
	{
		const double binEnd = binStart + binWidthMm;
		Vec3 sum{0.0, 0.0, 0.0};
		int count = 0;
		for (const auto& entry : projected)
		{
			if (entry.first < binStart || entry.first >= binEnd)
			{
				continue;
			}
			sum = add(sum, entry.second);
			++count;
		}
		if (count <= 0)
		{
			continue;
		}
		outPolyline.push_back(scale(sum, 1.0 / static_cast<double>(count)));
	}

	return outPolyline.size() >= 2U;
}

void buildKnnSkeletonGraph(SkeletonGraph& graph, const int knnK)
{
	const int n = static_cast<int>(graph.positions.size());
	if (n < 2 || knnK <= 0)
	{
		return;
	}
	const int kk = std::min(knnK, n - 1);
	graph.edges.clear();
	graph.edges.reserve(static_cast<std::size_t>(n) * static_cast<std::size_t>(kk));
	for (int i = 0; i < n; ++i)
	{
		std::vector<std::pair<double, int>> dists;
		dists.reserve(static_cast<std::size_t>(n - 1));
		const Vec3 pi = graph.positions[static_cast<std::size_t>(i)];
		for (int j = 0; j < n; ++j)
		{
			if (i == j)
			{
				continue;
			}
			dists.emplace_back(length(sub(graph.positions[static_cast<std::size_t>(j)], pi)), j);
		}
		const int pick = std::min(kk, static_cast<int>(dists.size()));
		std::partial_sort(dists.begin(), dists.begin() + pick, dists.end());
		for (int t = 0; t < pick; ++t)
		{
			const int j = dists[static_cast<std::size_t>(t)].second;
			const int lo = std::min(i, j);
			const int hi = std::max(i, j);
			graph.edges.emplace_back(lo, hi);
		}
	}
	std::sort(graph.edges.begin(), graph.edges.end());
	graph.edges.erase(std::unique(graph.edges.begin(), graph.edges.end()), graph.edges.end());
	graph.rebuildAdjacency();
}

std::vector<Vec3> subsamplePointsUniform(const std::vector<Vec3>& points, const std::size_t maxCount)
{
	if (points.size() <= maxCount || maxCount < 2U)
	{
		return points;
	}
	std::vector<Vec3> out;
	out.reserve(maxCount);
	const double step = static_cast<double>(points.size() - 1U) / static_cast<double>(maxCount - 1U);
	for (std::size_t i = 0; i < maxCount; ++i)
	{
		const std::size_t idx = static_cast<std::size_t>(step * static_cast<double>(i) + 0.5);
		out.push_back(points[std::min(idx, points.size() - 1U)]);
	}
	return out;
}

bool extractOrderedCenterlinePolylineImpl(const std::vector<Vec3>& points, const double binWidthMm,
										  std::vector<Vec3>& outPolyline)
{
	outPolyline.clear();
	if (points.size() < 2U)
	{
		return false;
	}
	if (extractCenterlineBySliceCentroids(points, binWidthMm, outPolyline))
	{
		return true;
	}

	const std::vector<Vec3> graphPts = subsamplePointsUniform(points, 8000U);
	SkeletonGraph graph;
	graph.positions = graphPts;
	const int knnK = std::min(12, static_cast<int>(graphPts.size()) - 1);
	buildKnnSkeletonGraph(graph, knnK);
	return extractLongestPathPolyline(graph, outPolyline);
}

} // namespace

bool extractOrderedCenterlinePolyline(const std::vector<Vec3>& points, const double binWidthMm,
									  std::vector<Vec3>& outPolyline)
{
	return extractOrderedCenterlinePolylineImpl(points, binWidthMm, outPolyline);
}

void resamplePolylineToSamples(const std::vector<Vec3>& polyline, const double spacingMm,
							   std::vector<TubularCenterlineSample>& outSamples)
{
	detailResamplePolylineToSamples(polyline, spacingMm, outSamples);
}

double computeContractionAnchorWeight(const int iteration, const int totalIterations, const double weightStart,
									  const double weightPeak)
{
	return detailComputeContractionAnchorWeight(iteration, totalIterations, weightStart, weightPeak);
}

bool extractLongestPathPolylineFromGraph(const std::vector<Vec3>& positions,
										 const std::vector<std::vector<int>>& adjacency, std::vector<Vec3>& outPolyline)
{
	SkeletonGraph graph;
	graph.positions = positions;
	graph.adjacency = adjacency;
	return extractLongestPathPolyline(graph, outPolyline);
}

bool computeCenterlinePcaAxisFromPoints(const std::vector<Vec3>& points, TubularCenterlinePcaAxis& outPca)
{
	Vec3 centroid{};
	Vec3 axis{};
	double extentMin = 0.0;
	double extentMax = 0.0;
	if (!computeCenterlinePcaFromPoints(points, centroid, axis, extentMin, extentMax))
	{
		return false;
	}
	fillCenterlinePcaAxis(centroid, axis, extentMin, extentMax, outPca);
	return true;
}

bool runLaplacianSkeletonCenterline(const IndexedMeshLite& mesh, const TubularGrindingParams& params,
									std::vector<TubularCenterlineSample>& outSamples,
									TubularCenterlinePcaAxis* outPcaAxis)
{
	outSamples.clear();
	if (outPcaAxis)
	{
		*outPcaAxis = TubularCenterlinePcaAxis{};
	}

	SkeletonGraph graph;
	if (!buildSkeletonGraphFromMesh(mesh, graph))
	{
		return false;
	}

	const int iterations = params.centerlineIterations > 0 ? params.centerlineIterations : 80;
	const double weightStart = std::max(1.0, params.laplacianAttraction * 5.0);
	const double weightPeak = std::clamp(params.laplacianLambda * 500.0, 10.0, 200.0);
	const double bboxDiag = meshBBoxDiagonal(mesh);
	const double pruneLength = bboxDiag * 0.02;
	const double sampleSpacing = params.sectionSpacingMm > 0.0 ? params.sectionSpacingMm : 2.0;

	for (int it = 0; it < iterations; ++it)
	{
		const double t = (iterations <= 1) ? 1.0 : static_cast<double>(it) / static_cast<double>(iterations - 1);
		const double anchorWeight = computeContractionAnchorWeight(it, iterations, weightStart, weightPeak);

		graph.rebuildAdjacency();
		contractSkeletonGraphStep(graph, anchorWeight);
		if (anchorWeight <= weightPeak * 0.25)
		{
			graph.anchors = graph.positions;
		}

		const double absCollapse = bboxDiag * (0.003 + 0.025 * t);
		const double avgEdge = graph.averageEdgeLength();
		const double collapseLength =
			(avgEdge > 1e-9) ? std::max(absCollapse, avgEdge * (0.2 + 0.55 * t)) : absCollapse;
		graph.collapseAllBelowLength(collapseLength, 64);

		const double minFaceArea = bboxDiag * bboxDiag * (1e-6 + 8e-5 * t);
		graph.removeDegenerateFaces(minFaceArea, absCollapse * 0.5);
	}

	const double finalCollapse = std::max(bboxDiag * 0.02, graph.averageEdgeLength() * 0.85);
	graph.collapseAllBelowLength(finalCollapse, 128);
	graph.removeDegenerateFaces(bboxDiag * bboxDiag * 1e-4, bboxDiag * 0.005);
	pruneShortLeafBranches(graph, pruneLength);

	if (outPcaAxis && graph.positions.size() >= 3U)
	{
		Vec3 centroid{};
		Vec3 axis{};
		double extentMin = 0.0;
		double extentMax = 0.0;
		if (computeCenterlinePcaFromPoints(graph.positions, centroid, axis, extentMin, extentMax))
		{
			fillCenterlinePcaAxis(centroid, axis, extentMin, extentMax, *outPcaAxis);
		}
	}

	std::vector<Vec3> polyline;
	if (!extractCenterlineBySliceCentroids(graph.positions, sampleSpacing, polyline) &&
		!extractLongestPathPolyline(graph, polyline))
	{
		return false;
	}

	resamplePolylineToSamples(polyline, sampleSpacing, outSamples);
	return outSamples.size() >= 2U;
}

} // namespace tg
} // namespace geoalgo
