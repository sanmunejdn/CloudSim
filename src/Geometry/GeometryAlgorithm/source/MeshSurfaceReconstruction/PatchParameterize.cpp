/// @file PatchParameterize.cpp
/// @brief PatchParameterize 实现

#include "MeshSurfaceReconstructionInternal.h"
#include "NurbsSurfaceFitting.h"
#include "RunLogger.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace geoalgo
{
namespace meshrecon
{
namespace
{
constexpr int kHarmonicMaxIters = 80;
constexpr int kDefaultHarmonicMaxFaces = 8000;
constexpr int kMaxGridPointsPerPatch = 4096;
constexpr int kDefaultMaxEdgeWhenUnlimited = 48;

struct Vec2d
{
	double u = 0.0;
	double v = 0.0;
};

struct Vec3d
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	Vec3d operator+(const Vec3d& o) const { return {x + o.x, y + o.y, z + o.z}; }
	Vec3d operator-(const Vec3d& o) const { return {x - o.x, y - o.y, z - o.z}; }
	Vec3d operator*(double s) const { return {x * s, y * s, z * s}; }
	double dot(const Vec3d& o) const { return x * o.x + y * o.y + z * o.z; }
	double length() const { return std::sqrt(dot(*this)); }
	Vec3d normalized() const
	{
		const double l = length();
		return (l > 1e-12) ? (*this * (1.0 / l)) : Vec3d{0, 0, 1};
	}
};

Vec3d crossv(const Vec3d& a, const Vec3d& b)
{
	return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

Vec3d readV(const std::vector<float>& v, int i)
{
	const std::size_t b = static_cast<std::size_t>(i) * 3U;
	return {v[b], v[b + 1U], v[b + 2U]};
}

void collectPatchPoints(const IndexedMeshLite& mesh, const QuadPatch& patch, std::vector<Vec3d>& outPts)
{
	outPts.clear();
	outPts.reserve(patch.faceIndices.size() * 3U);
	for (const int fi : patch.faceIndices)
	{
		const std::size_t b = static_cast<std::size_t>(fi) * 3U;
		outPts.push_back(readV(mesh.vertices, mesh.faces[b]));
		outPts.push_back(readV(mesh.vertices, mesh.faces[b + 1U]));
		outPts.push_back(readV(mesh.vertices, mesh.faces[b + 2U]));
	}
}

double bboxDiagonal(const std::vector<Vec3d>& pts)
{
	if (pts.empty())
	{
		return 1.0;
	}
	double xmin = pts[0].x, xmax = pts[0].x;
	double ymin = pts[0].y, ymax = pts[0].y;
	double zmin = pts[0].z, zmax = pts[0].z;
	for (const Vec3d& p : pts)
	{
		xmin = std::min(xmin, p.x);
		xmax = std::max(xmax, p.x);
		ymin = std::min(ymin, p.y);
		ymax = std::max(ymax, p.y);
		zmin = std::min(zmin, p.z);
		zmax = std::max(zmax, p.z);
	}
	const double dx = xmax - xmin;
	const double dy = ymax - ymin;
	const double dz = zmax - zmin;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 共线/共面分块时 UV 跨度可能为 0，按包围盒对角线兜底
void expandDegenerateUvSpan(double& uMin, double& uMax, double& vMin, double& vMax, const double charLen)
{
	constexpr double kMinSpanRatio = 0.08;
	const double minSpan = std::max(1e-3, charLen * kMinSpanRatio);
	if (uMax - uMin < minSpan)
	{
		const double mid = 0.5 * (uMin + uMax);
		uMin = mid - 0.5 * minSpan;
		uMax = mid + 0.5 * minSpan;
	}
	if (vMax - vMin < minSpan)
	{
		const double mid = 0.5 * (vMin + vMax);
		vMin = mid - 0.5 * minSpan;
		vMax = mid + 0.5 * minSpan;
	}
}

Vec3d meanFaceNormal(const IndexedMeshLite& mesh, const QuadPatch& patch)
{
	Vec3d sum{0, 0, 0};
	int count = 0;
	for (const int fi : patch.faceIndices)
	{
		const std::size_t b = static_cast<std::size_t>(fi) * 3U;
		const Vec3d p0 = readV(mesh.vertices, mesh.faces[b]);
		const Vec3d p1 = readV(mesh.vertices, mesh.faces[b + 1U]);
		const Vec3d p2 = readV(mesh.vertices, mesh.faces[b + 2U]);
		Vec3d n = crossv(p1 - p0, p2 - p0);
		const double l = n.length();
		if (l > 1e-12)
		{
			sum = sum + n * (1.0 / l);
			++count;
		}
	}
	if (count < 1 || sum.length() < 1e-12)
	{
		return {};
	}
	return sum.normalized();
}

// 协方差矩阵最小特征方向 ≈ 拟合平面法向；共线行时换行做叉积
Vec3d planeNormalFromCovariance(const double cxx, const double cxy, const double cxz, const double cyy,
								const double cyz, const double czz)
{
	const Vec3d r0{cxx, cxy, cxz};
	const Vec3d r1{cxy, cyy, cyz};
	const Vec3d r2{cxz, cyz, czz};
	Vec3d n = crossv(r0, r1);
	if (n.length() < 1e-9)
	{
		n = crossv(r1, r2);
	}
	if (n.length() < 1e-9)
	{
		n = crossv(r0, r2);
	}
	if (n.length() < 1e-9)
	{
		return {};
	}
	return n.normalized();
}

Vec3d alignPlaneNormalWithMesh(const Vec3d& planeN, const Vec3d& meshN)
{
	if (meshN.length() < 1e-9)
	{
		return planeN.length() > 1e-9 ? planeN : Vec3d{0, 0, 1};
	}
	Vec3d n = planeN.length() > 1e-9 ? planeN : meshN;
	// PCA 退化时法向可能与三角面法向正交，竖直面会落成水平条带
	if (std::abs(n.dot(meshN)) < 0.5)
	{
		n = meshN;
	}
	if (n.dot(meshN) < 0.0)
	{
		n = n * -1.0;
	}
	return n.normalized();
}

int clampSamplesPerEdge(const double uvSpanMm, const double targetSpacingMm, const int minEdge, const int maxEdge)
{
	if (uvSpanMm <= 1e-6)
	{
		return minEdge;
	}
	const int n = static_cast<int>(std::ceil(uvSpanMm / targetSpacingMm));
	const int effectiveMax = maxEdge > 0 ? maxEdge : kDefaultMaxEdgeWhenUnlimited;
	return std::max(minEdge, std::min(effectiveMax, n));
}

void clampGridResolution(int& nu, int& nv)
{
	nu = std::max(1, nu);
	nv = std::max(1, nv);
	while ((nu + 1) * (nv + 1) > kMaxGridPointsPerPatch)
	{
		if (nu >= nv)
		{
			--nu;
		}
		else
		{
			--nv;
		}
	}
	nu = std::max(1, nu);
	nv = std::max(1, nv);
}

Vec3d closestPointOnTriangle(const Vec3d& p, const Vec3d& a, const Vec3d& b, const Vec3d& c)
{
	const Vec3d ab = b - a;
	const Vec3d ac = c - a;
	const Vec3d ap = p - a;
	const double d1 = ab.dot(ap);
	const double d2 = ac.dot(ap);
	if (d1 <= 0.0 && d2 <= 0.0)
	{
		return a;
	}

	const Vec3d bp = p - b;
	const double d3 = ab.dot(bp);
	const double d4 = ac.dot(bp);
	if (d3 >= 0.0 && d4 <= d3)
	{
		return b;
	}

	const Vec3d cp = p - c;
	const double d5 = ab.dot(cp);
	const double d6 = ac.dot(cp);
	if (d6 >= 0.0 && d5 <= d6)
	{
		return c;
	}

	const double vc = d1 * d4 - d3 * d2;
	if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
	{
		const double v = d1 / (d1 - d3);
		return a + ab * v;
	}

	const double vb = d5 * d2 - d1 * d6;
	if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
	{
		const double w = d2 / (d2 - d6);
		return a + ac * w;
	}

	const double va = d3 * d6 - d5 * d4;
	if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
	{
		const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		return b + (c - b) * w;
	}

	const double denom = 1.0 / (va + vb + vc);
	const double v = vb * denom;
	const double w = vc * denom;
	return a + ab * v + ac * w;
}

struct PatchTriRef
{
	Vec3d a;
	Vec3d b;
	Vec3d c;
	Vec3d centroid;
};

int64_t packCellKey(const int cx, const int cy, const int cz)
{
	return (static_cast<int64_t>(cx) * 73856093) ^ (static_cast<int64_t>(cy) * 19349663) ^
		   (static_cast<int64_t>(cz) * 83492791);
}

class PatchClosestAccel
{
public:
	void build(const IndexedMeshLite& mesh, const QuadPatch& patch)
	{
		tris_.clear();
		cells_.clear();
		hasData_ = false;
		xmin_ = ymin_ = zmin_ = 1e30;
		xmax_ = ymax_ = zmax_ = -1e30;
		for (const int fi : patch.faceIndices)
		{
			if (fi < 0)
			{
				continue;
			}
			const std::size_t b = static_cast<std::size_t>(fi) * 3U;
			if (b + 2U >= mesh.faces.size())
			{
				continue;
			}
			PatchTriRef tri;
			tri.a = readV(mesh.vertices, mesh.faces[b]);
			tri.b = readV(mesh.vertices, mesh.faces[b + 1U]);
			tri.c = readV(mesh.vertices, mesh.faces[b + 2U]);
			tri.centroid = (tri.a + tri.b + tri.c) * (1.0 / 3.0);
			xmin_ = std::min(xmin_, std::min({tri.a.x, tri.b.x, tri.c.x}));
			xmax_ = std::max(xmax_, std::max({tri.a.x, tri.b.x, tri.c.x}));
			ymin_ = std::min(ymin_, std::min({tri.a.y, tri.b.y, tri.c.y}));
			ymax_ = std::max(ymax_, std::max({tri.a.y, tri.b.y, tri.c.y}));
			zmin_ = std::min(zmin_, std::min({tri.a.z, tri.b.z, tri.c.z}));
			zmax_ = std::max(zmax_, std::max({tri.a.z, tri.b.z, tri.c.z}));
			tris_.push_back(tri);
		}
		if (tris_.empty())
		{
			return;
		}
		const double dx = xmax_ - xmin_;
		const double dy = ymax_ - ymin_;
		const double dz = zmax_ - zmin_;
		const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
		cellSize_ = std::max(diag / std::max(8.0, std::cbrt(static_cast<double>(tris_.size())) * 1.5), 1e-4);
		invCell_ = 1.0 / cellSize_;
		for (std::size_t i = 0U; i < tris_.size(); ++i)
		{
			const Vec3d& c = tris_[i].centroid;
			const int cx = cellIndex(c.x, xmin_);
			const int cy = cellIndex(c.y, ymin_);
			const int cz = cellIndex(c.z, zmin_);
			cells_[packCellKey(cx, cy, cz)].push_back(static_cast<int>(i));
		}
		hasData_ = true;
	}

	Vec3d closest(const Vec3d& query) const
	{
		if (!hasData_)
		{
			return query;
		}
		if (tris_.size() <= 600U)
		{
			return closestFull(query);
		}
		Vec3d best = query;
		double bestDist2 = 1e60;
		const int qx = cellIndex(query.x, xmin_);
		const int qy = cellIndex(query.y, ymin_);
		const int qz = cellIndex(query.z, zmin_);
		for (int radius = 0; radius <= maxSearchRadius(query); ++radius)
		{
			for (int dx = -radius; dx <= radius; ++dx)
			{
				for (int dy = -radius; dy <= radius; ++dy)
				{
					for (int dz = -radius; dz <= radius; ++dz)
					{
						if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != radius)
						{
							continue;
						}
						const auto it = cells_.find(packCellKey(qx + dx, qy + dy, qz + dz));
						if (it == cells_.end())
						{
							continue;
						}
						for (const int triIdx : it->second)
						{
							const PatchTriRef& tri = tris_[static_cast<std::size_t>(triIdx)];
							const Vec3d cp = closestPointOnTriangle(query, tri.a, tri.b, tri.c);
							const Vec3d d = cp - query;
							const double dist2 = d.dot(d);
							if (dist2 < bestDist2)
							{
								bestDist2 = dist2;
								best = cp;
							}
						}
					}
				}
			}
		}
		if (bestDist2 >= 1e59)
		{
			return closestFull(query);
		}
		return best;
	}

private:
	std::vector<PatchTriRef> tris_;
	std::unordered_map<int64_t, std::vector<int>> cells_;
	double cellSize_ = 1.0;
	double invCell_ = 1.0;
	double xmin_ = 0.0;
	double ymin_ = 0.0;
	double zmin_ = 0.0;
	double xmax_ = 0.0;
	double ymax_ = 0.0;
	double zmax_ = 0.0;
	bool hasData_ = false;

	int maxSearchRadius(const Vec3d& query) const
	{
		const double dx = std::max(query.x - xmin_, xmax_ - query.x);
		const double dy = std::max(query.y - ymin_, ymax_ - query.y);
		const double dz = std::max(query.z - zmin_, zmax_ - query.z);
		const double reach = std::sqrt(dx * dx + dy * dy + dz * dz) + cellSize_;
		return std::min(32, static_cast<int>(std::ceil(reach * invCell_)) + 2);
	}

	int cellIndex(const double v, const double origin) const
	{
		return static_cast<int>(std::floor((v - origin) * invCell_));
	}

	Vec3d closestFull(const Vec3d& query) const
	{
		Vec3d best = query;
		double bestDist2 = 1e60;
		for (const PatchTriRef& tri : tris_)
		{
			const Vec3d cp = closestPointOnTriangle(query, tri.a, tri.b, tri.c);
			const Vec3d d = cp - query;
			const double dist2 = d.dot(d);
			if (dist2 < bestDist2)
			{
				bestDist2 = dist2;
				best = cp;
			}
		}
		return best;
	}
};

struct PatchPcaFrame
{
	Vec3d origin;
	Vec3d uDir;
	Vec3d vDir;
	double uMin = 0.0;
	double uMax = 0.0;
	double vMin = 0.0;
	double vMax = 0.0;
};

bool buildPatchPcaFrame(const IndexedMeshLite& mesh, const QuadPatch& patch, const std::vector<Vec3d>& patchPts,
						const double charLen, PatchPcaFrame& out)
{
	if (patchPts.empty())
	{
		return false;
	}
	out.origin = Vec3d{};
	for (const Vec3d& c : patchPts)
	{
		out.origin = out.origin + c;
	}
	out.origin = out.origin * (1.0 / static_cast<double>(patchPts.size()));

	double cxx = 0.0;
	double cyy = 0.0;
	double czz = 0.0;
	double cxy = 0.0;
	double cxz = 0.0;
	double cyz = 0.0;
	for (const Vec3d& c : patchPts)
	{
		const Vec3d d = c - out.origin;
		cxx += d.x * d.x;
		cyy += d.y * d.y;
		czz += d.z * d.z;
		cxy += d.x * d.y;
		cxz += d.x * d.z;
		cyz += d.y * d.z;
	}
	const Vec3d meshN = meanFaceNormal(mesh, patch);
	Vec3d planeN = planeNormalFromCovariance(cxx, cxy, cxz, cyy, cyz, czz);
	planeN = alignPlaneNormalWithMesh(planeN, meshN);
	const Vec3d ref = (std::abs(planeN.z) < 0.9) ? Vec3d{0, 0, 1} : Vec3d{1, 0, 0};
	out.uDir = crossv(planeN, ref).normalized();
	out.vDir = crossv(planeN, out.uDir).normalized();

	out.uMin = 1e30;
	out.uMax = -1e30;
	out.vMin = 1e30;
	out.vMax = -1e30;
	for (const Vec3d& c : patchPts)
	{
		const Vec3d rel = c - out.origin;
		out.uMin = std::min(out.uMin, rel.dot(out.uDir));
		out.uMax = std::max(out.uMax, rel.dot(out.uDir));
		out.vMin = std::min(out.vMin, rel.dot(out.vDir));
		out.vMax = std::max(out.vMax, rel.dot(out.vDir));
	}
	for (const int fi : patch.faceIndices)
	{
		if (fi < 0)
		{
			continue;
		}
		const std::size_t b = static_cast<std::size_t>(fi) * 3U;
		if (b + 2U >= mesh.faces.size())
		{
			continue;
		}
		const Vec3d p0 = readV(mesh.vertices, mesh.faces[b]);
		const Vec3d p1 = readV(mesh.vertices, mesh.faces[b + 1U]);
		const Vec3d p2 = readV(mesh.vertices, mesh.faces[b + 2U]);
		const Vec3d centroid = (p0 + p1 + p2) * (1.0 / 3.0);
		const Vec3d rel = centroid - out.origin;
		out.uMin = std::min(out.uMin, rel.dot(out.uDir));
		out.uMax = std::max(out.uMax, rel.dot(out.uDir));
		out.vMin = std::min(out.vMin, rel.dot(out.vDir));
		out.vMax = std::max(out.vMax, rel.dot(out.vDir));
	}
	expandDegenerateUvSpan(out.uMin, out.uMax, out.vMin, out.vMax, charLen);
	return true;
}

bool sampleUniformPhysicalGrid(const PatchPcaFrame& frame, const PatchClosestAccel& accel, const int nu, const int nv,
							   std::vector<float>& outXyz)
{
	const double uSpan = frame.uMax - frame.uMin;
	const double vSpan = frame.vMax - frame.vMin;
	const double du = uSpan / static_cast<double>(nu);
	const double dv = vSpan / static_cast<double>(nv);
	outXyz.clear();
	outXyz.reserve(static_cast<std::size_t>((nu + 1) * (nv + 1) * 3U));
	for (int iu = 0; iu <= nu; ++iu)
	{
		for (int iv = 0; iv <= nv; ++iv)
		{
			const double u = frame.uMin + static_cast<double>(iu) * du;
			const double v = frame.vMin + static_cast<double>(iv) * dv;
			const Vec3d planeP = frame.origin + frame.uDir * u + frame.vDir * v;
			const Vec3d p = accel.closest(planeP);
			outXyz.push_back(static_cast<float>(p.x));
			outXyz.push_back(static_cast<float>(p.y));
			outXyz.push_back(static_cast<float>(p.z));
		}
	}
	return !outXyz.empty();
}

bool sampleCentroidAnchoredPcaGrid(const IndexedMeshLite& mesh, const QuadPatch& patch, const PatchPcaFrame& frame,
								   const int nu, const int nv, std::vector<float>& outXyz)
{
	struct FaceAnchor
	{
		Vec3d pos;
		double u = 0.0;
		double v = 0.0;
	};
	std::vector<FaceAnchor> anchors;
	anchors.reserve(patch.faceIndices.size());
	for (const int fi : patch.faceIndices)
	{
		if (fi < 0)
		{
			continue;
		}
		const std::size_t b = static_cast<std::size_t>(fi) * 3U;
		if (b + 2U >= mesh.faces.size())
		{
			continue;
		}
		const Vec3d p0 = readV(mesh.vertices, mesh.faces[b]);
		const Vec3d p1 = readV(mesh.vertices, mesh.faces[b + 1U]);
		const Vec3d p2 = readV(mesh.vertices, mesh.faces[b + 2U]);
		const Vec3d centroid = (p0 + p1 + p2) * (1.0 / 3.0);
		const Vec3d rel = centroid - frame.origin;
		anchors.push_back(FaceAnchor{centroid, rel.dot(frame.uDir), rel.dot(frame.vDir)});
	}
	if (anchors.empty())
	{
		return false;
	}

	const double uSpan = frame.uMax - frame.uMin;
	const double vSpan = frame.vMax - frame.vMin;
	const double du = uSpan / static_cast<double>(nu);
	const double dv = vSpan / static_cast<double>(nv);
	outXyz.clear();
	outXyz.reserve(static_cast<std::size_t>((nu + 1) * (nv + 1) * 3U));
	for (int iu = 0; iu <= nu; ++iu)
	{
		for (int iv = 0; iv <= nv; ++iv)
		{
			const double u = frame.uMin + static_cast<double>(iu) * du;
			const double v = frame.vMin + static_cast<double>(iv) * dv;
			double bestD2 = 1e60;
			Vec3d bestP = anchors[0].pos;
			for (const FaceAnchor& anchor : anchors)
			{
				const double duA = anchor.u - u;
				const double dvA = anchor.v - v;
				const double d2 = duA * duA + dvA * dvA;
				if (d2 < bestD2)
				{
					bestD2 = d2;
					bestP = anchor.pos;
				}
			}
			outXyz.push_back(static_cast<float>(bestP.x));
			outXyz.push_back(static_cast<float>(bestP.y));
			outXyz.push_back(static_cast<float>(bestP.z));
		}
	}
	return !outXyz.empty();
}

double bboxDiagonalFromXyz(const std::vector<float>& xyz)
{
	if (xyz.size() < 3U)
	{
		return 0.0;
	}
	double xmin = xyz[0];
	double xmax = xyz[0];
	double ymin = xyz[1];
	double ymax = xyz[1];
	double zmin = xyz[2];
	double zmax = xyz[2];
	for (std::size_t i = 0U; i + 2U < xyz.size(); i += 3U)
	{
		xmin = std::min(xmin, static_cast<double>(xyz[i]));
		xmax = std::max(xmax, static_cast<double>(xyz[i]));
		ymin = std::min(ymin, static_cast<double>(xyz[i + 1U]));
		ymax = std::max(ymax, static_cast<double>(xyz[i + 1U]));
		zmin = std::min(zmin, static_cast<double>(xyz[i + 2U]));
		zmax = std::max(zmax, static_cast<double>(xyz[i + 2U]));
	}
	const double dx = xmax - xmin;
	const double dy = ymax - ymin;
	const double dz = zmax - zmin;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double estimateUniqueSampleRatio(const std::vector<float>& sampleXyz)
{
	const std::size_t ptCount = sampleXyz.size() / 3U;
	if (ptCount == 0U)
	{
		return 0.0;
	}
	std::unordered_set<int64_t> uniqueKeys;
	uniqueKeys.reserve(ptCount);
	for (std::size_t i = 0U; i < ptCount; ++i)
	{
		const std::size_t off = i * 3U;
		const int64_t key =
			(static_cast<int64_t>(std::llround(static_cast<double>(sampleXyz[off]) * 10.0)) << 42) ^
			(static_cast<int64_t>(std::llround(static_cast<double>(sampleXyz[off + 1U]) * 10.0)) << 21) ^
			static_cast<int64_t>(std::llround(static_cast<double>(sampleXyz[off + 2U]) * 10.0));
		uniqueKeys.insert(key);
	}
	return static_cast<double>(uniqueKeys.size()) / static_cast<double>(ptCount);
}

bool passesSampleQuality(const std::vector<float>& sampleXyz, const std::vector<Vec3d>& patchPts)
{
	if (sampleXyz.size() < 9U || patchPts.empty())
	{
		return false;
	}
	const double patchDiag = bboxDiagonal(patchPts);
	const double sampleDiag = bboxDiagonalFromXyz(sampleXyz);
	const double diagRatio = patchDiag > 1e-9 ? sampleDiag / patchDiag : 0.0;
	const double uniqueRatio = estimateUniqueSampleRatio(sampleXyz);
	return diagRatio >= 0.75 && uniqueRatio >= 0.65;
}

struct PatchLocalMesh
{
	std::vector<Vec3d> pos;
	std::vector<int> globalVertexIndices;
	std::vector<std::array<int, 3>> tris;
	std::vector<std::vector<int>> adj;
	std::vector<Vec2d> uv;
};

using EdgeKey = std::pair<int, int>;

struct EdgeKeyHash
{
	std::size_t operator()(const EdgeKey& key) const noexcept
	{
		return static_cast<std::size_t>(key.first) * 1315423911U + static_cast<std::size_t>(key.second);
	}
};

EdgeKey normEdgeKey(const int a, const int b)
{
	return a < b ? EdgeKey{a, b} : EdgeKey{b, a};
}

bool buildPatchLocalMesh(const IndexedMeshLite& mesh, const QuadPatch& patch, PatchLocalMesh& out)
{
	out.pos.clear();
	out.globalVertexIndices.clear();
	out.tris.clear();
	out.adj.clear();
	out.uv.clear();
	std::unordered_map<int, int> globalToLocal;
	for (const int fi : patch.faceIndices)
	{
		if (fi < 0)
		{
			continue;
		}
		const std::size_t b = static_cast<std::size_t>(fi) * 3U;
		if (b + 2U >= mesh.faces.size())
		{
			continue;
		}
		std::array<int, 3> tri{};
		for (int c = 0; c < 3; ++c)
		{
			const int gv = mesh.faces[b + static_cast<std::size_t>(c)];
			auto it = globalToLocal.find(gv);
			if (it == globalToLocal.end())
			{
				const int li = static_cast<int>(out.pos.size());
				globalToLocal[gv] = li;
				out.pos.push_back(readV(mesh.vertices, gv));
				out.globalVertexIndices.push_back(gv);
				tri[static_cast<std::size_t>(c)] = li;
			}
			else
			{
				tri[static_cast<std::size_t>(c)] = it->second;
			}
		}
		out.tris.push_back(tri);
	}
	const int n = static_cast<int>(out.pos.size());
	if (n < 3 || out.tris.empty())
	{
		return false;
	}
	out.adj.assign(static_cast<std::size_t>(n), {});
	for (const auto& tri : out.tris)
	{
		for (int e = 0; e < 3; ++e)
		{
			const int a = tri[static_cast<std::size_t>(e)];
			const int b = tri[static_cast<std::size_t>((e + 1) % 3)];
			auto& na = out.adj[static_cast<std::size_t>(a)];
			if (std::find(na.begin(), na.end(), b) == na.end())
			{
				na.push_back(b);
			}
		}
	}
	return true;
}

std::vector<int> findLongestBoundaryLoop(const PatchLocalMesh& local)
{
	std::unordered_map<EdgeKey, int, EdgeKeyHash> edgeCount;
	for (const auto& tri : local.tris)
	{
		for (int e = 0; e < 3; ++e)
		{
			const EdgeKey key =
				normEdgeKey(tri[static_cast<std::size_t>(e)], tri[static_cast<std::size_t>((e + 1) % 3)]);
			++edgeCount[key];
		}
	}
	std::unordered_map<int, std::vector<int>> boundaryAdj;
	for (const auto& kv : edgeCount)
	{
		if (kv.second != 1)
		{
			continue;
		}
		boundaryAdj[kv.first.first].push_back(kv.first.second);
		boundaryAdj[kv.first.second].push_back(kv.first.first);
	}
	if (boundaryAdj.empty())
	{
		return {};
	}

	std::vector<int> bestLoop;
	std::vector<char> visited(static_cast<std::size_t>(local.pos.size()), 0);
	for (const auto& kv : boundaryAdj)
	{
		const int start = kv.first;
		if (start < 0 || start >= static_cast<int>(visited.size()) || visited[static_cast<std::size_t>(start)])
		{
			continue;
		}
		int prev = -1;
		int cur = start;
		std::vector<int> loop;
		for (int guard = 0; guard < static_cast<int>(local.pos.size()) + 4; ++guard)
		{
			loop.push_back(cur);
			visited[static_cast<std::size_t>(cur)] = 1;
			const auto& nbs = boundaryAdj[cur];
			int next = -1;
			for (const int nb : nbs)
			{
				if (nb != prev)
				{
					next = nb;
					break;
				}
			}
			if (next < 0 || next == start)
			{
				break;
			}
			prev = cur;
			cur = next;
		}
		if (loop.size() > bestLoop.size())
		{
			bestLoop = std::move(loop);
		}
	}
	return bestLoop.size() >= 3 ? bestLoop : std::vector<int>{};
}

double cotanAtCorner(const Vec3d& a, const Vec3d& b, const Vec3d& corner)
{
	const Vec3d u = a - corner;
	const Vec3d v = b - corner;
	const double sinL = crossv(u, v).length();
	if (sinL < 1e-12)
	{
		return 0.0;
	}
	return u.dot(v) / sinL;
}

void accumulateCotanWeights(const PatchLocalMesh& local, std::vector<std::vector<std::pair<int, double>>>& weights)
{
	const int n = static_cast<int>(local.pos.size());
	weights.assign(static_cast<std::size_t>(n), {});
	auto addWeight = [&](const int i, const int j, const double w)
	{
		if (i == j || w <= 0.0)
		{
			return;
		}
		auto& row = weights[static_cast<std::size_t>(i)];
		for (auto& kv : row)
		{
			if (kv.first == j)
			{
				kv.second += w;
				return;
			}
		}
		row.emplace_back(j, w);
	};
	for (const auto& tri : local.tris)
	{
		const int a = tri[0];
		const int b = tri[1];
		const int c = tri[2];
		const Vec3d& pa = local.pos[static_cast<std::size_t>(a)];
		const Vec3d& pb = local.pos[static_cast<std::size_t>(b)];
		const Vec3d& pc = local.pos[static_cast<std::size_t>(c)];
		const double wab = 0.5 * cotanAtCorner(pa, pb, pc);
		const double wbc = 0.5 * cotanAtCorner(pb, pc, pa);
		const double wca = 0.5 * cotanAtCorner(pc, pa, pb);
		addWeight(a, b, wab);
		addWeight(b, a, wab);
		addWeight(b, c, wbc);
		addWeight(c, b, wbc);
		addWeight(c, a, wca);
		addWeight(a, c, wca);
	}
}

bool solveHarmonicUv(const PatchLocalMesh& local, const std::vector<int>& boundaryLoop,
					 const MeshSurfaceHarmonicBoundaryMode boundaryMode, std::vector<Vec2d>& outUv)
{
	const int n = static_cast<int>(local.pos.size());
	if (n < 3 || boundaryLoop.size() < 3)
	{
		return false;
	}
	std::vector<char> isBoundary(static_cast<std::size_t>(n), 0);
	for (const int bi : boundaryLoop)
	{
		if (bi >= 0 && bi < n)
		{
			isBoundary[static_cast<std::size_t>(bi)] = 1;
		}
	}

	outUv.assign(static_cast<std::size_t>(n), Vec2d{});
	double totalLen = 0.0;
	std::vector<double> arcLen(boundaryLoop.size(), 0.0);
	for (std::size_t i = 1; i < boundaryLoop.size(); ++i)
	{
		const Vec3d& p0 = local.pos[static_cast<std::size_t>(boundaryLoop[i - 1])];
		const Vec3d& p1 = local.pos[static_cast<std::size_t>(boundaryLoop[i])];
		totalLen += (p1 - p0).length();
		arcLen[i] = totalLen;
	}
	if (totalLen < 1e-9)
	{
		return false;
	}

	if (boundaryMode == MeshSurfaceHarmonicBoundaryMode::GeodesicSquare && boundaryLoop.size() >= 4)
	{
		const auto cornerAtFraction = [&](const double frac) -> int
		{
			const double target = frac * totalLen;
			for (std::size_t i = 0; i < arcLen.size(); ++i)
			{
				if (arcLen[i] + 1e-9 >= target)
				{
					return boundaryLoop[i];
				}
			}
			return boundaryLoop.back();
		};
		const int c0 = cornerAtFraction(0.0);
		const int c1 = cornerAtFraction(0.25);
		const int c2 = cornerAtFraction(0.50);
		const int c3 = cornerAtFraction(0.75);
		const int corners[4] = {c0, c1, c2, c3};

		auto sideRange = [&](const int startCorner, const int endCorner)
		{
			std::vector<int> side;
			bool started = false;
			for (const int vi : boundaryLoop)
			{
				if (vi == startCorner)
				{
					started = true;
				}
				if (started)
				{
					side.push_back(vi);
				}
				if (started && vi == endCorner)
				{
					break;
				}
			}
			return side;
		};

		const std::array<std::vector<int>, 4> sides = {
			sideRange(corners[0], corners[1]), sideRange(corners[1], corners[2]), sideRange(corners[2], corners[3]),
			sideRange(corners[3], corners[0])};

		const auto mapSideToSquare = [&](const std::vector<int>& side, const int sideIdx)
		{
			if (side.size() < 2)
			{
				return;
			}
			double sideLen = 0.0;
			std::vector<double> sideArc(side.size(), 0.0);
			for (std::size_t i = 1; i < side.size(); ++i)
			{
				const Vec3d& p0 = local.pos[static_cast<std::size_t>(side[i - 1])];
				const Vec3d& p1 = local.pos[static_cast<std::size_t>(side[i])];
				sideLen += (p1 - p0).length();
				sideArc[i] = sideLen;
			}
			if (sideLen < 1e-9)
			{
				sideLen = 1.0;
			}
			for (std::size_t i = 0; i < side.size(); ++i)
			{
				const double t = sideArc[i] / sideLen;
				const int li = side[i];
				switch (sideIdx)
				{
				case 0:
					outUv[static_cast<std::size_t>(li)] = Vec2d{t, 0.0};
					break;
				case 1:
					outUv[static_cast<std::size_t>(li)] = Vec2d{1.0, t};
					break;
				case 2:
					outUv[static_cast<std::size_t>(li)] = Vec2d{1.0 - t, 1.0};
					break;
				default:
					outUv[static_cast<std::size_t>(li)] = Vec2d{0.0, 1.0 - t};
					break;
				}
			}
		};

		for (int si = 0; si < 4; ++si)
		{
			mapSideToSquare(sides[static_cast<std::size_t>(si)], si);
		}
	}
	else
	{
		for (std::size_t i = 0; i < boundaryLoop.size(); ++i)
		{
			const double t = arcLen[i] / totalLen;
			const double ang = t * 6.283185307179586;
			const int li = boundaryLoop[i];
			outUv[static_cast<std::size_t>(li)] = Vec2d{0.5 + 0.5 * std::cos(ang), 0.5 + 0.5 * std::sin(ang)};
		}
	}

	std::vector<std::vector<std::pair<int, double>>> weights;
	accumulateCotanWeights(local, weights);
	const int iterCount = std::min(kHarmonicMaxIters, std::max(24, n / 8));
	for (int it = 0; it < iterCount; ++it)
	{
		for (int i = 0; i < n; ++i)
		{
			if (isBoundary[static_cast<std::size_t>(i)])
			{
				continue;
			}
			double sumW = 0.0;
			double sumU = 0.0;
			double sumV = 0.0;
			for (const auto& kv : weights[static_cast<std::size_t>(i)])
			{
				sumW += kv.second;
				sumU += kv.second * outUv[static_cast<std::size_t>(kv.first)].u;
				sumV += kv.second * outUv[static_cast<std::size_t>(kv.first)].v;
			}
			if (sumW > 1e-12)
			{
				outUv[static_cast<std::size_t>(i)].u = sumU / sumW;
				outUv[static_cast<std::size_t>(i)].v = sumV / sumW;
			}
		}
	}
	return true;
}

bool barycentric2d(const Vec2d& p, const Vec2d& a, const Vec2d& b, const Vec2d& c, double& w0, double& w1, double& w2)
{
	const double denom = (b.v - c.v) * (a.u - c.u) + (c.u - b.u) * (a.v - c.v);
	if (std::abs(denom) < 1e-18)
	{
		return false;
	}
	w0 = ((b.v - c.v) * (p.u - c.u) + (c.u - b.u) * (p.v - c.v)) / denom;
	w1 = ((c.v - a.v) * (p.u - c.u) + (a.u - c.u) * (p.v - c.v)) / denom;
	w2 = 1.0 - w0 - w1;
	return w0 >= -1e-5 && w1 >= -1e-5 && w2 >= -1e-5;
}

void faceCentroidUvBounds(const PatchLocalMesh& local, double& uMin, double& uMax, double& vMin, double& vMax)
{
	uMin = 1e30;
	uMax = -1e30;
	vMin = 1e30;
	vMax = -1e30;
	for (const auto& tri : local.tris)
	{
		const Vec2d cu{(local.uv[static_cast<std::size_t>(tri[0])].u + local.uv[static_cast<std::size_t>(tri[1])].u +
						local.uv[static_cast<std::size_t>(tri[2])].u) /
						   3.0,
					   (local.uv[static_cast<std::size_t>(tri[0])].v + local.uv[static_cast<std::size_t>(tri[1])].v +
						local.uv[static_cast<std::size_t>(tri[2])].v) /
						   3.0};
		uMin = std::min(uMin, cu.u);
		uMax = std::max(uMax, cu.u);
		vMin = std::min(vMin, cu.v);
		vMax = std::max(vMax, cu.v);
	}
}

bool locateUvInPatchLight(const PatchLocalMesh& local, const Vec2d& query, int& outTriIdx, double& w0, double& w1,
						  double& w2, const int hintTriIdx = -1)
{
	const auto tryTriangle = [&](const std::size_t ti) -> bool
	{
		const auto& tri = local.tris[ti];
		double tw0 = 0.0;
		double tw1 = 0.0;
		double tw2 = 0.0;
		if (!barycentric2d(query, local.uv[static_cast<std::size_t>(tri[0])],
						   local.uv[static_cast<std::size_t>(tri[1])], local.uv[static_cast<std::size_t>(tri[2])], tw0,
						   tw1, tw2))
		{
			return false;
		}
		outTriIdx = static_cast<int>(ti);
		w0 = tw0;
		w1 = tw1;
		w2 = tw2;
		return true;
	};
	if (hintTriIdx >= 0 && static_cast<std::size_t>(hintTriIdx) < local.tris.size())
	{
		if (tryTriangle(static_cast<std::size_t>(hintTriIdx)))
		{
			return true;
		}
	}
	for (std::size_t ti = 0; ti < local.tris.size(); ++ti)
	{
		if (tryTriangle(ti))
		{
			return true;
		}
	}
	return false;
}

Vec3d interpolate3dFromUv(const PatchLocalMesh& local, const int triIdx, const double w0, const double w1,
						  const double w2)
{
	const auto& tri = local.tris[static_cast<std::size_t>(triIdx)];
	return local.pos[static_cast<std::size_t>(tri[0])] * w0 + local.pos[static_cast<std::size_t>(tri[1])] * w1 +
		   local.pos[static_cast<std::size_t>(tri[2])] * w2;
}

bool locateNearestUvTriangle(const PatchLocalMesh& local, const Vec2d& query, int& outTriIdx, double& w0, double& w1,
							 double& w2)
{
	int bestTri = 0;
	double bestD2 = 1e60;
	for (std::size_t ti = 0; ti < local.tris.size(); ++ti)
	{
		const auto& tri = local.tris[ti];
		const Vec2d cu{(local.uv[static_cast<std::size_t>(tri[0])].u + local.uv[static_cast<std::size_t>(tri[1])].u +
						local.uv[static_cast<std::size_t>(tri[2])].u) /
						   3.0,
					   (local.uv[static_cast<std::size_t>(tri[0])].v + local.uv[static_cast<std::size_t>(tri[1])].v +
						local.uv[static_cast<std::size_t>(tri[2])].v) /
						   3.0};
		const double du = cu.u - query.u;
		const double dv = cu.v - query.v;
		const double d2 = du * du + dv * dv;
		if (d2 < bestD2)
		{
			bestD2 = d2;
			bestTri = static_cast<int>(ti);
		}
	}
	outTriIdx = bestTri;
	const auto& tri = local.tris[static_cast<std::size_t>(bestTri)];
	if (!barycentric2d(query, local.uv[static_cast<std::size_t>(tri[0])], local.uv[static_cast<std::size_t>(tri[1])],
					   local.uv[static_cast<std::size_t>(tri[2])], w0, w1, w2))
	{
		w0 = 1.0 / 3.0;
		w1 = 1.0 / 3.0;
		w2 = 1.0 / 3.0;
	}
	return true;
}

bool sampleHarmonicFaceCentroidGrid(const PatchLocalMesh& local, const int nu, const int nv, const double uMin,
									const double uMax, const double vMin, const double vMax, std::vector<float>& outXyz)
{
	const double du = (uMax - uMin) / static_cast<double>(nu);
	const double dv = (vMax - vMin) / static_cast<double>(nv);
	outXyz.clear();
	outXyz.reserve(static_cast<std::size_t>((nu + 1) * (nv + 1) * 3U));
	int hintTriIdx = -1;
	for (int iu = 0; iu <= nu; ++iu)
	{
		for (int iv = 0; iv <= nv; ++iv)
		{
			const Vec2d query{uMin + static_cast<double>(iu) * du, vMin + static_cast<double>(iv) * dv};
			int triIdx = -1;
			double w0 = 0.0;
			double w1 = 0.0;
			double w2 = 0.0;
			if (!locateUvInPatchLight(local, query, triIdx, w0, w1, w2, hintTriIdx))
			{
				(void)locateNearestUvTriangle(local, query, triIdx, w0, w1, w2);
			}
			hintTriIdx = triIdx;
			const Vec3d p = interpolate3dFromUv(local, triIdx, w0, w1, w2);
			outXyz.push_back(static_cast<float>(p.x));
			outXyz.push_back(static_cast<float>(p.y));
			outXyz.push_back(static_cast<float>(p.z));
		}
	}
	return !outXyz.empty();
}

bool computeHarmonicPatchUv(const IndexedMeshLite& mesh, const QuadPatch& patch,
							const MeshSurfaceReconstructParams& params, PatchLocalMesh& local)
{
	if (!buildPatchLocalMesh(mesh, patch, local))
	{
		return false;
	}
	const std::vector<int> boundaryLoop = findLongestBoundaryLoop(local);
	if (!solveHarmonicUv(local, boundaryLoop, params.harmonicBoundaryMode, local.uv))
	{
		return false;
	}
	return true;
}

std::vector<Vec2d> constructParameterGrid(const int nu, const int nv, const int gridMode)
{
	std::vector<Vec2d> grid;
	grid.reserve(static_cast<std::size_t>((nu + 1) * (nv + 1)));
	if (gridMode == 1)
	{
		for (int iu = 0; iu <= nu; ++iu)
		{
			const double u = static_cast<double>(iu) / static_cast<double>(std::max(1, nu));
			for (int iv = 0; iv <= nv; ++iv)
			{
				const double v = static_cast<double>(iv) / static_cast<double>(std::max(1, nv));
				grid.push_back(Vec2d{u, v});
			}
		}
	}
	else if (gridMode == 2)
	{
		for (int iu = 0; iu <= nu; ++iu)
		{
			const double u = 0.02 + 0.96 * static_cast<double>(iu) / static_cast<double>(std::max(1, nu));
			for (int iv = 0; iv <= nv; ++iv)
			{
				const double v = 0.02 + 0.96 * static_cast<double>(iv) / static_cast<double>(std::max(1, nv));
				grid.push_back(Vec2d{u, v});
			}
		}
	}
	else
	{
		for (int iu = 0; iu <= nu; ++iu)
		{
			const double u = static_cast<double>(iu + 1) / static_cast<double>(nu + 2);
			for (int iv = 0; iv <= nv; ++iv)
			{
				const double v = static_cast<double>(iv + 1) / static_cast<double>(nv + 2);
				grid.push_back(Vec2d{u, v});
			}
		}
	}
	return grid;
}

bool sampleAmrtoHarmonicGrid(const PatchLocalMesh& local, const int nu, const int nv, const double uMin,
							 const double uMax, const double vMin, const double vMax, const int gridMode,
							 std::vector<float>& outXyz)
{
	const std::vector<Vec2d> unitGrid = constructParameterGrid(nu, nv, gridMode);
	outXyz.clear();
	outXyz.reserve(unitGrid.size() * 3U);
	int hintTriIdx = -1;
	for (const Vec2d& unit : unitGrid)
	{
		const Vec2d query{uMin + unit.u * (uMax - uMin), vMin + unit.v * (vMax - vMin)};
		int triIdx = -1;
		double w0 = 0.0;
		double w1 = 0.0;
		double w2 = 0.0;
		if (!locateUvInPatchLight(local, query, triIdx, w0, w1, w2, hintTriIdx))
		{
			(void)locateNearestUvTriangle(local, query, triIdx, w0, w1, w2);
		}
		hintTriIdx = triIdx;
		const Vec3d p = interpolate3dFromUv(local, triIdx, w0, w1, w2);
		outXyz.push_back(static_cast<float>(p.x));
		outXyz.push_back(static_cast<float>(p.y));
		outXyz.push_back(static_cast<float>(p.z));
	}
	return outXyz.size() == unitGrid.size() * 3U;
}

void uvBounds(const std::vector<Vec2d>& uv, double& uMin, double& uMax, double& vMin, double& vMax)
{
	uMin = 1e30;
	uMax = -1e30;
	vMin = 1e30;
	vMax = -1e30;
	for (const Vec2d& p : uv)
	{
		uMin = std::min(uMin, p.u);
		uMax = std::max(uMax, p.u);
		vMin = std::min(vMin, p.v);
		vMax = std::max(vMax, p.v);
	}
}

} // namespace

bool samplePatchGrids(const IndexedMeshLite& mesh, std::vector<QuadPatch>& patches,
					  const MeshSurfaceReconstructParams& params, std::string* errMsg)
{
	const int minEdge = std::max(4, params.minSamplesPerEdge);
	const int maxEdge = params.maxSamplesPerEdge;
	int fixedN = std::max(minEdge, params.samplesPerPatchEdge);
	if (maxEdge > 0)
	{
		fixedN = std::min(maxEdge, fixedN);
	}
	const bool useAdaptiveSpacing = params.targetUvSpacingMm > 1e-6;
	const int harmonicFaceLimit = params.harmonicMaxFaces > 0 ? params.harmonicMaxFaces : kDefaultHarmonicMaxFaces;

	int patchIdx = 0;
	const int patchTotal = static_cast<int>(patches.size());
	for (QuadPatch& patch : patches)
	{
		if (patch.faceIndices.empty())
		{
			++patchIdx;
			continue;
		}
		std::vector<Vec3d> patchPts;
		collectPatchPoints(mesh, patch, patchPts);
		if (patchPts.empty())
		{
			++patchIdx;
			continue;
		}
		const double charLen = bboxDiagonal(patchPts);

		int nu = fixedN;
		int nv = fixedN;

		PatchPcaFrame frame;
		if (!buildPatchPcaFrame(mesh, patch, patchPts, charLen, frame))
		{
			++patchIdx;
			continue;
		}
		const double uSpan = frame.uMax - frame.uMin;
		const double vSpan = frame.vMax - frame.vMin;

		// 弯曲 patch 优先在面心 UV 域做调和曲面插值；平面投影仅作回退
		PatchLocalMesh local;
		double huMin = 0.0;
		double huMax = 1.0;
		double hvMin = 0.0;
		double hvMax = 1.0;
		const bool harmonicOk = static_cast<int>(patch.faceIndices.size()) <= harmonicFaceLimit &&
								computeHarmonicPatchUv(mesh, patch, params, local);
		double uvAspect = 1.0;
		if (harmonicOk)
		{
			faceCentroidUvBounds(local, huMin, huMax, hvMin, hvMax);
			const double uvDiag = std::max(huMax - huMin, hvMax - hvMin);
			expandDegenerateUvSpan(huMin, huMax, hvMin, hvMax, std::max(0.05, uvDiag * 0.08));
			uvAspect = (hvMax - hvMin) / std::max(1e-9, huMax - huMin);

			const AmrtoGridResolution amrtoRes = computeAmrtoGridResolution(huMax - huMin, hvMax - hvMin, params);
			nu = std::max(minEdge, amrtoRes.sampleNu);
			nv = std::max(minEdge, amrtoRes.sampleNv);
			if (params.samplesPerPatchEdge > 0)
			{
				nu = std::min(nu, std::max(minEdge, params.samplesPerPatchEdge));
				nv = std::min(nv, std::max(minEdge, params.samplesPerPatchEdge));
			}
			if (maxEdge > 0)
			{
				nu = std::min(maxEdge, nu);
				nv = std::min(maxEdge, nv);
			}
			else
			{
				nu = std::min(kDefaultMaxEdgeWhenUnlimited, nu);
				nv = std::min(kDefaultMaxEdgeWhenUnlimited, nv);
			}
			if (!useAdaptiveSpacing && uvAspect > 0.15)
			{
				const int effectiveMax = maxEdge > 0 ? maxEdge : kDefaultMaxEdgeWhenUnlimited;
				nv = std::max(minEdge, std::min(effectiveMax, static_cast<int>(std::lround(nu * uvAspect))));
			}
			patch.computedCtrlPtsU = amrtoRes.ctrlPtsU;
			patch.computedCtrlPtsV = amrtoRes.ctrlPtsV;
			patch.uvNormMinU = 0.0;
			patch.uvNormMaxU = 1.0;
			patch.uvNormMinV = 0.0;
			patch.uvNormMaxV = 1.0;
		}

		if (useAdaptiveSpacing)
		{
			nu = clampSamplesPerEdge(uSpan, params.targetUvSpacingMm, minEdge, maxEdge);
			nv = clampSamplesPerEdge(vSpan, params.targetUvSpacingMm, minEdge, maxEdge);
		}
		clampGridResolution(nu, nv);

		patch.gridNu = nu;
		patch.gridNv = nv;
		patch.gridN = std::max(nu, nv);
		patch.uvUSpanMm = uSpan;
		patch.uvVSpanMm = vSpan;
		patch.sampleXyz.clear();

		PatchClosestAccel accel;
		accel.build(mesh, patch);
		const char* samplingPath = "pca";
		bool usedAmrtoHarmonic = false;
		(void)sampleUniformPhysicalGrid(frame, accel, nu, nv, patch.sampleXyz);

		const bool pcaQualityOk = passesSampleQuality(patch.sampleXyz, patchPts);
		if (harmonicOk)
		{
			std::vector<float> amrtoXyz;
			if (sampleAmrtoHarmonicGrid(local, nu, nv, huMin, huMax, hvMin, hvMax, params.parameterGridMode,
										amrtoXyz) &&
				passesSampleQuality(amrtoXyz, patchPts))
			{
				patch.sampleXyz = std::move(amrtoXyz);
				samplingPath = params.harmonicBoundaryMode == MeshSurfaceHarmonicBoundaryMode::GeodesicSquare
								   ? "amrto-harmonic-geo"
								   : "amrto-harmonic";
				usedAmrtoHarmonic = true;
			}
		}
		if (!usedAmrtoHarmonic && harmonicOk)
		{
			std::vector<float> harmonicXyz;
			if (sampleHarmonicFaceCentroidGrid(local, nu, nv, huMin, huMax, hvMin, hvMax, harmonicXyz) &&
				passesSampleQuality(harmonicXyz, patchPts))
			{
				const double pcaDiag = bboxDiagonalFromXyz(patch.sampleXyz);
				const double harmonicDiag = bboxDiagonalFromXyz(harmonicXyz);
				if (!pcaQualityOk || harmonicDiag > pcaDiag * 1.05)
				{
					patch.sampleXyz = std::move(harmonicXyz);
					samplingPath = "harmonic";
				}
			}
		}

		if (!passesSampleQuality(patch.sampleXyz, patchPts))
		{
			std::vector<float> anchoredXyz;
			if (sampleCentroidAnchoredPcaGrid(mesh, patch, frame, nu, nv, anchoredXyz))
			{
				const double pcaDiag = bboxDiagonalFromXyz(patch.sampleXyz);
				const double anchoredDiag = bboxDiagonalFromXyz(anchoredXyz);
				const double pcaUnique = estimateUniqueSampleRatio(patch.sampleXyz);
				const double anchoredUnique = estimateUniqueSampleRatio(anchoredXyz);
				if (passesSampleQuality(anchoredXyz, patchPts) || anchoredDiag > pcaDiag * 1.1 ||
					anchoredUnique > pcaUnique * 1.2)
				{
					patch.sampleXyz = std::move(anchoredXyz);
					samplingPath = "pca-centroid";
				}
			}
		}

		patch.samplingPath = samplingPath;

		if (!patch.sampleXyz.empty())
		{
			const double patchDiag = bboxDiagonal(patchPts);
			const double sampleDiag = bboxDiagonalFromXyz(patch.sampleXyz);
			const double diagRatio = patchDiag > 1e-9 ? sampleDiag / patchDiag : 0.0;
			const double uniqueRatio = estimateUniqueSampleRatio(patch.sampleXyz);
			RunLogger::info(std::string("patch ") + std::to_string(patchIdx) +
							" sample: grid=" + std::to_string(nu + 1) + "x" + std::to_string(nv + 1) +
							" path=" + samplingPath + " diagRatio=" + std::to_string(diagRatio) +
							" unique=" + std::to_string(static_cast<int>(uniqueRatio * 100.0)) + "%");
		}

		++patchIdx;
		if (patchIdx % 5 == 0 || patchIdx == patchTotal)
		{
			RunLogger::info(std::string("sample grids progress ") + std::to_string(patchIdx) + "/" +
							std::to_string(patchTotal));
		}
	}

	if (patches.empty())
	{
		if (errMsg)
		{
			*errMsg = "no patches to sample";
		}
		return false;
	}

	int sampledPatchCount = 0;
	for (const QuadPatch& patch : patches)
	{
		if (!patch.sampleXyz.empty())
		{
			++sampledPatchCount;
		}
	}
	if (sampledPatchCount < 1)
	{
		if (errMsg)
		{
			*errMsg = "patch grid sampling produced no points";
		}
		return false;
	}
	return true;
}

void assignPatchCornerMetadata(const IndexedMeshLite& mesh, QuadPatch& patch)
{
	patch.cornerMeshVertices = {-1, -1, -1, -1};
	patch.hasSquareCorners = false;
	PatchLocalMesh local;
	if (!buildPatchLocalMesh(mesh, patch, local))
	{
		return;
	}
	const std::vector<int> boundaryLoop = findLongestBoundaryLoop(local);
	if (boundaryLoop.size() < 4 || local.globalVertexIndices.size() != local.pos.size())
	{
		return;
	}
	double totalLen = 0.0;
	std::vector<double> arcLen(boundaryLoop.size(), 0.0);
	for (std::size_t i = 1; i < boundaryLoop.size(); ++i)
	{
		const Vec3d& p0 = local.pos[static_cast<std::size_t>(boundaryLoop[i - 1])];
		const Vec3d& p1 = local.pos[static_cast<std::size_t>(boundaryLoop[i])];
		totalLen += (p1 - p0).length();
		arcLen[i] = totalLen;
	}
	if (totalLen < 1e-9)
	{
		return;
	}
	const auto cornerLocalAt = [&](const double frac) -> int
	{
		const double target = frac * totalLen;
		for (std::size_t i = 0; i < arcLen.size(); ++i)
		{
			if (arcLen[i] + 1e-9 >= target)
			{
				return boundaryLoop[i];
			}
		}
		return boundaryLoop.back();
	};
	const int locals[4] = {cornerLocalAt(0.0), cornerLocalAt(0.25), cornerLocalAt(0.50), cornerLocalAt(0.75)};
	for (int i = 0; i < 4; ++i)
	{
		const int li = locals[i];
		if (li >= 0 && static_cast<std::size_t>(li) < local.globalVertexIndices.size())
		{
			patch.cornerMeshVertices[static_cast<std::size_t>(i)] =
				local.globalVertexIndices[static_cast<std::size_t>(li)];
		}
	}
	patch.hasSquareCorners = true;
	(void)mesh;
}

void assignAllPatchCornerMetadata(const IndexedMeshLite& mesh, std::vector<QuadPatch>& patches)
{
	for (QuadPatch& patch : patches)
	{
		assignPatchCornerMetadata(mesh, patch);
	}
}

} // namespace meshrecon
} // namespace geoalgo
