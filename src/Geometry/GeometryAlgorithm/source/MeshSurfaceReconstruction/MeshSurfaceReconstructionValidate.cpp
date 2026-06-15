#include "MeshSurfaceReconstructionInternal.h"
#include "Discretize.h"
#include "ShapeHandle.h"
#include "ShapeQuery.h"

#include "detail/OccIncludes.h"

#include <BRep_Tool.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <Poly_Triangulation.hxx>

#include <cmath>
#include <vector>

namespace geoalgo
{
namespace meshrecon
{
namespace
{

struct Bbox3
{
	double xmin = 0;
	double xmax = 0;
	double ymin = 0;
	double ymax = 0;
	double zmin = 0;
	double zmax = 0;
	bool valid = false;
};

Bbox3 bboxFromSoup(const std::vector<float>& soup)
{
	Bbox3 bb;
	if (soup.size() < 9U)
	{
		return bb;
	}
	bb.xmin = bb.xmax = soup[0];
	bb.ymin = bb.ymax = soup[1];
	bb.zmin = bb.zmax = soup[2];
	bb.valid = true;
	for (std::size_t i = 0; i + 2U < soup.size(); i += 3U)
	{
		bb.xmin = std::min(bb.xmin, static_cast<double>(soup[i]));
		bb.xmax = std::max(bb.xmax, static_cast<double>(soup[i]));
		bb.ymin = std::min(bb.ymin, static_cast<double>(soup[i + 1U]));
		bb.ymax = std::max(bb.ymax, static_cast<double>(soup[i + 1U]));
		bb.zmin = std::min(bb.zmin, static_cast<double>(soup[i + 2U]));
		bb.zmax = std::max(bb.zmax, static_cast<double>(soup[i + 2U]));
	}
	return bb;
}

Bbox3 bboxFromShapeHandle(const ShapeHandle& shape)
{
	Bbox3 bb;
	if (shape.isNull())
	{
		return bb;
	}
	const ShapeHandle::BoundsMm b = shape.boundingBoxMm();
	if (!b.valid)
	{
		return bb;
	}
	bb.xmin = b.minX;
	bb.xmax = b.maxX;
	bb.ymin = b.minY;
	bb.ymax = b.maxY;
	bb.zmin = b.minZ;
	bb.zmax = b.maxZ;
	bb.valid = true;
	return bb;
}

double bboxDiagonal(const Bbox3& bb)
{
	if (!bb.valid)
	{
		return 0.0;
	}
	const double dx = bb.xmax - bb.xmin;
	const double dy = bb.ymax - bb.ymin;
	const double dz = bb.zmax - bb.zmin;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace

bool validateTessellationSanity(const ShapeHandle& shape, const MeshSurfaceReconstructParams& params, std::string* errMsg)
{
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native))
	{
		if (errMsg)
		{
			*errMsg = "tessellation validation: null shape";
		}
		return false;
	}
	const int faceCount = shapeFaceCount(native);
	if (faceCount <= 0)
	{
		if (errMsg)
		{
			*errMsg = "tessellation validation: no faces";
		}
		return false;
	}

	TessellateParams disc;
	// 自适应离散精度：大尺寸形状用更粗的偏差，避免产生过多三角形
	const ShapeHandle::BoundsMm bb = shape.boundingBoxMm();
	const double diag = bb.valid
		? std::sqrt((bb.maxX - bb.minX) * (bb.maxX - bb.minX)
			+ (bb.maxY - bb.minY) * (bb.maxY - bb.minY)
			+ (bb.maxZ - bb.minZ) * (bb.maxZ - bb.minZ))
		: 100.0;
	const double adaptiveDeflection = std::max(0.1, diag * 0.003);
	disc.linearDeflectionMm = std::max(adaptiveDeflection, params.tessellateLinearDeflectionMm);
	disc.angularDeflectionDeg = 5.0;
	disc.linearDeflectionRelative = false;
	TopoDS_Shape meshed = native;
	(void)meshShapeIncremental(meshed, disc, nullptr);

	constexpr int kMaxTrisPerFace = 8000;
	int nonemptyFaces = 0;
	int maxTriCount = 0;
	int maxTriFaceIdx = -1;
	for (int faceIdx = 0; faceIdx < faceCount; ++faceIdx)
	{
		TopoDS_Face face;
		if (!shapeFaceAtIndex(meshed, faceIdx, face, nullptr))
		{
			continue;
		}
		TopLoc_Location loc;
		const Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
		const int triCount =
			(!tri.IsNull() && tri->HasGeometry()) ? static_cast<int>(tri->NbTriangles()) : 0;
		if (triCount > 0)
		{
			++nonemptyFaces;
		}
		if (triCount > maxTriCount)
		{
			maxTriCount = triCount;
			maxTriFaceIdx = faceIdx;
		}
		if (triCount > kMaxTrisPerFace)
		{
			if (errMsg)
			{
				*errMsg = "reconstructed face tessellation too dense";
			}
			return false;
		}
	}
	if (nonemptyFaces < 1)
	{
		if (errMsg)
		{
			*errMsg = "tessellation validation: all faces empty";
		}
		return false;
	}
	return true;
}

bool validateOutputBbox(
	const std::vector<float>& soup,
	const ShapeHandle& shape,
	const double maxDiagRatio,
	std::string* errMsg)
{
	const double inDiag = bboxDiagonal(bboxFromSoup(soup));
	const double outDiag = bboxDiagonal(bboxFromShapeHandle(shape));
	const double ratio = inDiag > 1e-6 ? outDiag / inDiag : 0.0;
	if (ratio > maxDiagRatio)
	{
		if (errMsg)
		{
			*errMsg = "reconstructed shape bounding box invalid";
		}
		return false;
	}
	return true;
}

} // namespace meshrecon
} // namespace geoalgo
