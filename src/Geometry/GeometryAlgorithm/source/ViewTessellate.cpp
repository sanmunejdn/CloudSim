#include "detail/OccIncludes.h"

#include "Discretize.h"
#include "ShapeHandle.h"
#include "ShapeQuery.h"
#include "ViewTessellate.h"

#include <algorithm>
#include <cmath>

namespace geoalgo
{
namespace
{

double bboxDiagonalMm(const ShapeHandle::BoundsMm& b)
{
	if (!b.valid)
	{
		return 1.0;
	}
	const double dx = b.maxX - b.minX;
	const double dy = b.maxY - b.minY;
	const double dz = b.maxZ - b.minZ;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double transformPointZ(const double mColMajor16[16], double x, double y, double z)
{
	return mColMajor16[2] * x + mColMajor16[6] * y + mColMajor16[10] * z + mColMajor16[14];
}

double estimateViewLinearDeflectionMm(
	const ShapeHandle& shape,
	const double viewMatrixColMajor16[16],
	const double projMatrixColMajor16[16],
	int viewportWidthPx,
	int viewportHeightPx,
	const ViewTessellateParams& params)
{
	const ShapeHandle::BoundsMm b = shape.boundingBoxMm();
	const double cx = b.valid ? (b.minX + b.maxX) * 0.5 : 0.0;
	const double cy = b.valid ? (b.minY + b.maxY) * 0.5 : 0.0;
	const double cz = b.valid ? (b.minZ + b.maxZ) * 0.5 : 0.0;
	const double eyeZ = transformPointZ(viewMatrixColMajor16, cx, cy, cz);
	const double depth = std::max(1.0, std::abs(eyeZ));
	const int vpMin = std::max(1, std::min(viewportWidthPx, viewportHeightPx));
	const double worldPerPixel = bboxDiagonalMm(b) / static_cast<double>(vpMin);
	const double px = std::max(0.5, params.pixelsPerEdge);
	double lin = worldPerPixel * px * (depth / 1000.0);
	lin = std::clamp(lin, params.minLinearDeflectionMm, params.maxLinearDeflectionMm);
	(void)projMatrixColMajor16;
	return lin;
}

} // namespace

void computeTriangleSoupNormals(const std::vector<float>& soup, std::vector<float>& outNormals)
{
	outNormals.resize(soup.size());
	for (std::size_t i = 0; i + 8 < soup.size(); i += 9U)
	{
		const float x0 = soup[i];
		const float y0 = soup[i + 1];
		const float z0 = soup[i + 2];
		const float x1 = soup[i + 3];
		const float y1 = soup[i + 4];
		const float z1 = soup[i + 5];
		const float x2 = soup[i + 6];
		const float y2 = soup[i + 7];
		const float z2 = soup[i + 8];
		const float ax = x1 - x0;
		const float ay = y1 - y0;
		const float az = z1 - z0;
		const float bx = x2 - x0;
		const float by = y2 - y0;
		const float bz = z2 - z0;
		float nx = ay * bz - az * by;
		float ny = az * bx - ax * bz;
		float nz = ax * by - ay * bx;
		const float len2 = nx * nx + ny * ny + nz * nz;
		if (len2 > 1e-20f)
		{
			const float inv = 1.0f / std::sqrt(len2);
			nx *= inv;
			ny *= inv;
			nz *= inv;
		}
		for (int v = 0; v < 3; ++v)
		{
			outNormals[i + static_cast<std::size_t>(v) * 3U] = nx;
			outNormals[i + static_cast<std::size_t>(v) * 3U + 1U] = ny;
			outNormals[i + static_cast<std::size_t>(v) * 3U + 2U] = nz;
		}
	}
}

bool tessellateShapeMedium(
	const ShapeHandle& shape,
	std::vector<float>& outSoup,
	std::vector<float>* outNormals,
	std::string* errMsg)
{
	outSoup.clear();
	if (outNormals)
	{
		outNormals->clear();
	}
	if (shape.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native))
	{
		if (errMsg)
		{
			*errMsg = "shape access failed";
		}
		return false;
	}
	TessellateParams disc;
	disc.linearDeflectionMm = 0.01;
	disc.angularDeflectionDeg = 0.5;
	disc.linearDeflectionRelative = false;
	if (!discretizeShapeToSoup(native, disc, outSoup, errMsg))
	{
		return false;
	}
	if (outNormals && !outSoup.empty())
	{
		computeTriangleSoupNormals(outSoup, *outNormals);
	}
	return true;
}

bool tessellateShapePerFaceMedium(
	const ShapeHandle& shape,
	std::vector<float>& outSoup,
	std::vector<int>& outTriangleFaceIndex,
	std::vector<std::vector<float>>* outFaceSoups,
	std::string* errMsg)
{
	outSoup.clear();
	outTriangleFaceIndex.clear();
	if (outFaceSoups)
	{
		outFaceSoups->clear();
	}
	if (shape.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	TessellateParams disc;
	disc.linearDeflectionMm = 0.01;
	disc.angularDeflectionDeg = 0.5;
	disc.linearDeflectionRelative = false;
	return discretizeShapeToSoupPerFace(shape, disc, outSoup, outTriangleFaceIndex, outFaceSoups, errMsg);
}

bool tessellateShapeForView(
	const ShapeHandle& shape,
	const ViewTessellateParams& params,
	const double viewMatrixColMajor16[16],
	const double projMatrixColMajor16[16],
	int viewportWidthPx,
	int viewportHeightPx,
	std::vector<float>& outSoup,
	std::vector<float>* outNormals,
	std::string* errMsg)
{
	outSoup.clear();
	if (outNormals)
	{
		outNormals->clear();
	}
	if (shape.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native))
	{
		if (errMsg)
		{
			*errMsg = "shape access failed";
		}
		return false;
	}
	ShapeHandle work = shape.clone();
	TopoDS_Shape workNative;
	if (!ShapeHandleAccess::nativeShape(work, &workNative))
	{
		if (errMsg)
		{
			*errMsg = "shape clone failed";
		}
		return false;
	}
	TessellateParams disc;
	disc.linearDeflectionMm =
		estimateViewLinearDeflectionMm(shape, viewMatrixColMajor16, projMatrixColMajor16, viewportWidthPx,
			viewportHeightPx, params);
	disc.linearDeflectionRelative = false;
	disc.angularDeflectionDeg = params.angularDeflectionDeg;
	disc.flipReversedFaces = params.flipReversedFaces;
	if (!discretizeShapeToSoup(workNative, disc, outSoup, errMsg))
	{
		return false;
	}
	if (outNormals && !outSoup.empty())
	{
		computeTriangleSoupNormals(outSoup, *outNormals);
	}
	return true;
}

} // namespace geoalgo
