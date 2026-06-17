#include "TubularGrindingCommon.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
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

	bool operator==(const QuantizedVertexKey& other) const
	{
		return x == other.x && y == other.y && z == other.z;
	}
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
	std::size_t operator()(const WeldedEdgeKey& key) const
	{
		return static_cast<std::size_t>(key.a ^ (key.b << 16));
	}
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
	return {
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x};
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

	out.bboxMin = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
	out.bboxMax = {-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()};

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
			{weldedVerts[0], weldedVerts[1]},
			{weldedVerts[1], weldedVerts[2]},
			{weldedVerts[2], weldedVerts[0]}};
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

bool rayBundleCenterPoint(
	const std::vector<Vec3>& origins,
	const std::vector<Vec3>& inwardDirs,
	Vec3& outCenter)
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

double meanDistanceToRays(
	const Vec3& center,
	const std::vector<Vec3>& origins,
	const std::vector<Vec3>& inwardDirs)
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

bool closestApproachMidpoint(
	const Vec3& o1,
	const Vec3& d1,
	const Vec3& o2,
	const Vec3& d2,
	Vec3& outMid)
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

Vec3 pairwiseRayMidpointCenter(
	const std::vector<Vec3>& origins,
	const std::vector<Vec3>& inwardDirs)
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

bool approximateRayBundleCenter(
	const std::vector<Vec3>& origins,
	const std::vector<Vec3>& inwardDirs,
	const double maxMeanDistanceMm,
	Vec3& outCenter)
{
	if (origins.size() < 2U || origins.size() != inwardDirs.size())
	{
		return false;
	}
	const double localSpan = std::max(1.0, estimateLocalRaySpan(origins));
	const double tol = maxMeanDistanceMm > 0.0
		? maxMeanDistanceMm
		: std::max(3.0, localSpan * 0.55);

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
		Vec3 w{
			cov[0][0] * v.x + cov[0][1] * v.y + cov[0][2] * v.z,
			cov[1][0] * v.x + cov[1][1] * v.y + cov[1][2] * v.z,
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
		Vec3 w{
			(trace - cov[0][0]) * u.x - cov[0][1] * u.y - cov[0][2] * u.z,
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

int runDbscan(
	const std::vector<Vec3>& featurePoints,
	const double eps,
	const int minPts,
	std::vector<int>& outLabels)
{
	const int n = static_cast<int>(featurePoints.size());
	outLabels.assign(static_cast<std::size_t>(n), -1);
	int clusterId = 0;
	const double eps2 = eps * eps;

	auto regionQuery = [&](const int idx) {
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

bool fitCircle2d(
	const std::vector<std::array<double, 2>>& pts,
	double& outCx,
	double& outCy,
	double& outRadius)
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

void buildFrenetFrames(
	const std::vector<TubularCenterlineSample>& samples,
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
		Vec3 t{
			outSamples[i].tangent[0],
			outSamples[i].tangent[1],
			outSamples[i].tangent[2]};
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

TubularGrindingTemplateKind selectTemplateKind(
	const TubularPipeSegment& segment,
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
		const Vec3 a{
			samples[i - 1].positionMm[0],
			samples[i - 1].positionMm[1],
			samples[i - 1].positionMm[2]};
		const Vec3 b{
			samples[i].positionMm[0],
			samples[i].positionMm[1],
			samples[i].positionMm[2]};
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

} // namespace tg
} // namespace geoalgo
