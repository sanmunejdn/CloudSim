#include "MeshSurfaceReconstructionInternal.h"

#include <cmath>

namespace geoalgo
{
namespace meshrecon
{
namespace
{

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
	return {
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x};
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
void expandDegenerateUvSpan(
	double& uMin,
	double& uMax,
	double& vMin,
	double& vMax,
	const double charLen)
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
Vec3d planeNormalFromCovariance(
	const double cxx,
	const double cxy,
	const double cxz,
	const double cyy,
	const double cyz,
	const double czz)
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

} // namespace

bool samplePatchGrids(
	const IndexedMeshLite& mesh,
	std::vector<QuadPatch>& patches,
	const MeshSurfaceReconstructParams& params,
	std::string* errMsg)
{
	const int n = std::max(4, std::min(32, params.samplesPerPatchEdge));

	for (QuadPatch& patch : patches)
	{
		if (patch.faceIndices.empty())
		{
			continue;
		}
		std::vector<Vec3d> patchPts;
		collectPatchPoints(mesh, patch, patchPts);
		const double charLen = bboxDiagonal(patchPts);

		Vec3d origin{0, 0, 0};
		for (const Vec3d& c : patchPts)
		{
			origin = origin + c;
		}
		origin = origin * (1.0 / static_cast<double>(patchPts.size()));

		double cxx = 0, cyy = 0, czz = 0, cxy = 0, cxz = 0, cyz = 0;
		for (const Vec3d& c : patchPts)
		{
			const Vec3d d = c - origin;
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
		Vec3d ref = (std::abs(planeN.z) < 0.9) ? Vec3d{0, 0, 1} : Vec3d{1, 0, 0};
		const Vec3d uDir = crossv(planeN, ref).normalized();
		const Vec3d vDir = crossv(planeN, uDir).normalized();

		double uMin = 1e30, uMax = -1e30, vMin = 1e30, vMax = -1e30;
		for (const Vec3d& c : patchPts)
		{
			const Vec3d rel = c - origin;
			uMin = std::min(uMin, rel.dot(uDir));
			uMax = std::max(uMax, rel.dot(uDir));
			vMin = std::min(vMin, rel.dot(vDir));
			vMax = std::max(vMax, rel.dot(vDir));
		}
		expandDegenerateUvSpan(uMin, uMax, vMin, vMax, charLen);

		patch.gridN = n;
		patch.sampleXyz.clear();
		patch.sampleXyz.reserve(static_cast<std::size_t>((n + 1) * (n + 1) * 3U));
		const double du = (uMax - uMin) / static_cast<double>(n);
		const double dv = (vMax - vMin) / static_cast<double>(n);
		const int gridSize = n + 1;
		std::vector<Vec3d> gridPts(static_cast<std::size_t>(gridSize * gridSize));
		std::vector<int> gridCounts(static_cast<std::size_t>(gridSize * gridSize), 0);

		for (const Vec3d& c : patchPts)
		{
			const Vec3d rel = c - origin;
			const double u = rel.dot(uDir);
			const double v = rel.dot(vDir);
			int iu = static_cast<int>(std::floor((u - uMin) / du));
			int iv = static_cast<int>(std::floor((v - vMin) / dv));
			iu = std::max(0, std::min(n, iu));
			iv = std::max(0, std::min(n, iv));
			const std::size_t cell = static_cast<std::size_t>(iu * gridSize + iv);
			if (gridCounts[cell] == 0)
			{
				gridPts[cell] = c;
			}
			else
			{
				gridPts[cell] = gridPts[cell] + c;
			}
			++gridCounts[cell];
		}

		for (int iu = 0; iu <= n; ++iu)
		{
			for (int iv = 0; iv <= n; ++iv)
			{
				const std::size_t cell = static_cast<std::size_t>(iu * gridSize + iv);
				Vec3d p;
				if (gridCounts[cell] > 0)
				{
					p = gridPts[cell] * (1.0 / static_cast<double>(gridCounts[cell]));
				}
				else
				{
					const double u = uMin + static_cast<double>(iu) * du;
					const double v = vMin + static_cast<double>(iv) * dv;
					p = origin + uDir * u + vDir * v;
				}
				patch.sampleXyz.push_back(static_cast<float>(p.x));
				patch.sampleXyz.push_back(static_cast<float>(p.y));
				patch.sampleXyz.push_back(static_cast<float>(p.z));
			}
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
	return true;
}

} // namespace meshrecon
} // namespace geoalgo
