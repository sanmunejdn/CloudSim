#include "MeshSurfaceReconstructionInternal.h"

#include "ShapeHandle.h"

#include "detail/OccIncludes.h"

#include <BRep_Builder.hxx>

#include <cmath>
#include <limits>

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
	double dot(const Vec3d& o) const { return x * o.x + y * o.y + z * o.z; }
	Vec3d operator-(const Vec3d& o) const { return {x - o.x, y - o.y, z - o.z}; }
	double length() const { return std::sqrt(dot(*this)); }
};

} // namespace

bool assembleBrepShape(
	const IndexedMeshLite& mesh,
	const std::vector<QuadPatch>& patches,
	ShapeHandle& outShape,
	std::string* errMsg)
{
	std::vector<TopoDS_Face> faces;
	std::size_t faceReserve = patches.size();
	for (const QuadPatch& patch : patches)
	{
		if (patch.meshFallback)
		{
			faceReserve += patch.meshFallbackFaces.size();
		}
	}
	faces.reserve(faceReserve);
	for (const QuadPatch& patchIn : patches)
	{
		QuadPatch patch = patchIn;
		if (patch.meshFallback)
		{
			if (!rebuildPatchFace(mesh, patch))
			{
				continue;
			}
			for (const TopoDS_Face& triFace : patch.meshFallbackFaces)
			{
				if (!triFace.IsNull())
				{
					faces.push_back(triFace);
				}
			}
			continue;
		}
		if (!rebuildPatchFace(mesh, patch) || patch.face.IsNull())
		{
			continue;
		}
		faces.push_back(patch.face);
	}
	if (faces.empty())
	{
		if (errMsg)
		{
			*errMsg = "no faces to assemble";
		}
		return false;
	}

	// 开放分片曲面不做 Sewing，避免自由边被合并后三角化失败
	TopoDS_Compound compound;
	BRep_Builder builder;
	builder.MakeCompound(compound);
	for (const TopoDS_Face& face : faces)
	{
		builder.Add(compound, face);
	}

	outShape = ShapeHandleAccess::fromNativeShape(&compound);
	return !outShape.isNull();
}

double computeMaxDeviationMm(
	const std::vector<float>& soup,
	const std::vector<QuadPatch>& patches)
{
	if (soup.size() < 9U)
	{
		return 0.0;
	}
	double maxDev = 0.0;
	const std::size_t stride = std::max<std::size_t>(1U, (soup.size() / 9U) / 512U);
	for (std::size_t t = 0; t < soup.size() / 9U; t += stride)
	{
		const std::size_t b = t * 9U;
		const Vec3d p{soup[b], soup[b + 1U], soup[b + 2U]};
		double best = std::numeric_limits<double>::max();
		for (const QuadPatch& patch : patches)
		{
			if (patch.sampleXyz.size() < 3U)
			{
				continue;
			}
			for (std::size_t i = 0; i + 2U < patch.sampleXyz.size(); i += 3U)
			{
				const Vec3d q{
					patch.sampleXyz[i],
					patch.sampleXyz[i + 1U],
					patch.sampleXyz[i + 2U]};
				best = std::min(best, (p - q).length());
			}
		}
		if (best < std::numeric_limits<double>::max())
		{
			maxDev = std::max(maxDev, best);
		}
	}
	return maxDev;
}

} // namespace meshrecon
} // namespace geoalgo
