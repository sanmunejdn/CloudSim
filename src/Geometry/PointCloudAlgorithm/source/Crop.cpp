#include "Crop.h"

#include "PointCloudBuffer.h"

namespace pclalgo
{

namespace
{

template<typename Predicate>
void cropXyzImpl(
	const std::vector<float>& srcXyz,
	const std::vector<float>* srcRgba,
	Predicate keepPredicate,
	std::vector<float>& outXyz,
	std::vector<float>* outRgba,
	std::vector<std::size_t>* keptIndices)
{
	if (!validXyzLength(srcXyz))
	{
		outXyz.clear();
		if (outRgba != nullptr)
		{
			outRgba->clear();
		}
		return;
	}

	const std::size_t n = pointCountFromXyz(srcXyz);
	std::vector<std::size_t> kept;
	kept.reserve(n);
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		const Eigen::Vector3d p(srcXyz[b], srcXyz[b + 1U], srcXyz[b + 2U]);
		if (keepPredicate(p))
		{
			kept.push_back(i);
		}
	}

	compactXyzByIndices(srcXyz, kept, outXyz);
	if (srcRgba != nullptr && outRgba != nullptr && validRgbaLength(*srcRgba, n))
	{
		compactRgbaByIndices(*srcRgba, kept, *outRgba);
	}
	else if (outRgba != nullptr)
	{
		outRgba->clear();
	}

	if (keptIndices != nullptr)
	{
		*keptIndices = std::move(kept);
	}
}

} // namespace

void cropXyzByBox(
	const std::vector<float>& srcXyz,
	const Eigen::AlignedBox3d& box,
	std::vector<float>& outXyz,
	std::vector<std::size_t>* keptIndices)
{
	cropXyzImpl(
		srcXyz,
		nullptr,
		[&box](const Eigen::Vector3d& p) { return box.contains(p); },
		outXyz,
		nullptr,
		keptIndices);
}

void cropXyzByBox(
	const std::vector<float>& srcXyz,
	const std::vector<float>& srcRgba,
	const Eigen::AlignedBox3d& box,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::vector<std::size_t>* keptIndices)
{
	cropXyzImpl(
		srcXyz,
		&srcRgba,
		[&box](const Eigen::Vector3d& p) { return box.contains(p); },
		outXyz,
		&outRgba,
		keptIndices);
}

void cropXyzBySphere(
	const std::vector<float>& srcXyz,
	const Eigen::Vector3d& centerMm,
	const double radiusMm,
	std::vector<float>& outXyz,
	std::vector<std::size_t>* keptIndices)
{
	const double r2 = radiusMm * radiusMm;
	cropXyzImpl(
		srcXyz,
		nullptr,
		[&centerMm, r2](const Eigen::Vector3d& p) { return (p - centerMm).squaredNorm() <= r2; },
		outXyz,
		nullptr,
		keptIndices);
}

namespace
{

void transformPointColumnMajor(const double m[16], const double x, const double y, const double z, double& ox, double& oy, double& oz)
{
	ox = x * m[0] + y * m[4] + z * m[8] + m[12];
	oy = x * m[1] + y * m[5] + z * m[9] + m[13];
	oz = x * m[2] + y * m[6] + z * m[10] + m[14];
}

/// OSG 行向量：clip = world * mvp，矩阵按 col*4+row 存 m(row,col)
void transformPointRowVectorOsg4(
	const double m[16],
	const double x,
	const double y,
	const double z,
	double& ox,
	double& oy,
	double& oz,
	double& ow)
{
	ox = x * m[0] + y * m[1] + z * m[2] + m[3];
	oy = x * m[4] + y * m[5] + z * m[6] + m[7];
	oz = x * m[8] + y * m[9] + z * m[10] + m[11];
	ow = x * m[12] + y * m[13] + z * m[14] + m[15];
}

bool pointInPolygon2D(const double x, const double y, const std::vector<float>& poly)
{
	if (poly.size() < 6U)
	{
		return false;
	}
	const std::size_t vertexCount = poly.size() / 2U;
	bool inside = false;
	for (std::size_t i = 0, j = vertexCount - 1U; i < vertexCount; j = i++)
	{
		const double xi = static_cast<double>(poly[i * 2U]);
		const double yi = static_cast<double>(poly[i * 2U + 1U]);
		const double xj = static_cast<double>(poly[j * 2U]);
		const double yj = static_cast<double>(poly[j * 2U + 1U]);
		const bool intersect = ((yi > y) != (yj > y))
			&& (x < (xj - xi) * (y - yi) / (yj - yi + 1e-30) + xi);
		if (intersect)
		{
			inside = !inside;
		}
	}
	return inside;
}

bool projectToScreen(
	const double mvpMatrix[16],
	const double modelToWorld[16],
	int viewportWidth,
	int viewportHeight,
	const double x,
	const double y,
	const double z,
	double& outScreenX,
	double& outScreenY)
{
	double wx = 0.0;
	double wy = 0.0;
	double wz = 0.0;
	transformPointColumnMajor(modelToWorld, x, y, z, wx, wy, wz);

	double cx = 0.0;
	double cy = 0.0;
	double cz = 0.0;
	double cw = 0.0;
	transformPointRowVectorOsg4(mvpMatrix, wx, wy, wz, cx, cy, cz, cw);
	if (cw <= 1e-12)
	{
		return false;
	}
	const double invW = 1.0 / cw;
	const double ndcX = cx * invW;
	const double ndcY = cy * invW;
	outScreenX = (ndcX * 0.5 + 0.5) * static_cast<double>(viewportWidth);
	outScreenY = (1.0 - (ndcY * 0.5 + 0.5)) * static_cast<double>(viewportHeight);
	return true;
}

} // namespace

void cropXyzByPolyline2D(
	const std::vector<float>& srcXyz,
	const std::vector<float>& srcRgba,
	const std::vector<float>& polylineScreenXy,
	const double mvpMatrix[16],
	const double modelToWorld[16],
	const int viewportWidth,
	const int viewportHeight,
	const bool keepInside,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::vector<std::size_t>* keptIndices)
{
	if (!validXyzLength(srcXyz) || polylineScreenXy.size() < 6U || viewportWidth <= 0 || viewportHeight <= 0)
	{
		outXyz.clear();
		outRgba.clear();
		if (keptIndices != nullptr)
		{
			keptIndices->clear();
		}
		return;
	}

	const std::size_t n = pointCountFromXyz(srcXyz);
	const bool hasRgba = validRgbaLength(srcRgba, n);
	std::vector<std::size_t> kept;
	kept.reserve(n);
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		double sx = 0.0;
		double sy = 0.0;
		const bool visible = projectToScreen(
			mvpMatrix,
			modelToWorld,
			viewportWidth,
			viewportHeight,
			static_cast<double>(srcXyz[b]),
			static_cast<double>(srcXyz[b + 1U]),
			static_cast<double>(srcXyz[b + 2U]),
			sx,
			sy);
		const bool inside = visible && pointInPolygon2D(sx, sy, polylineScreenXy);
		const bool keep = keepInside ? inside : !inside;
		if (keep)
		{
			kept.push_back(i);
		}
	}

	compactXyzByIndices(srcXyz, kept, outXyz);
	if (hasRgba)
	{
		compactRgbaByIndices(srcRgba, kept, outRgba);
	}
	else
	{
		outRgba.clear();
	}
	if (keptIndices != nullptr)
	{
		*keptIndices = std::move(kept);
	}
}

} // namespace pclalgo
