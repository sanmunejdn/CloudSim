#include "BrepImportArtifacts.h"

#include "Discretize.h"
#include "ShapeQuery.h"
#include "ViewTessellate.h"

#include <chrono>
#include <mutex>
#include <vector>

namespace geoalgo
{
namespace
{

struct CacheEntry
{
	ShapeHandle shapeKey;
	std::shared_ptr<const BrepImportArtifacts> artifacts;
};

std::mutex g_cacheMutex;
std::vector<CacheEntry> g_cacheEntries;

} // namespace

bool buildBrepImportArtifacts(const ShapeHandle& shape, BrepImportArtifacts& out, std::string* errMsg)
{
	out = {};
	if (shape.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	const auto t0 = std::chrono::steady_clock::now();
	if (!tessellateShapePerFaceMedium(shape, out.displaySoup, out.triangleFaceIndex, &out.faceSoups, errMsg))
	{
		return false;
	}
	const std::size_t triCount = out.displaySoup.size() / 9U;
	if (triCount != out.triangleFaceIndex.size())
	{
		if (errMsg)
		{
			*errMsg = "triangle face index size mismatch";
		}
		return false;
	}
	if (!collectShapeFaceEdgeIndices(shape, out.faceEdgeIndices, errMsg))
	{
		return false;
	}
	TessellateParams edgeParams;
	edgeParams.linearDeflectionMm = 0.05;
	edgeParams.angularDeflectionDeg = 1.0;
	edgeParams.linearDeflectionRelative = false;
	std::vector<Polyline3d> polylines;
	if (!discretizeShapeEdges(shape, edgeParams, polylines, errMsg))
	{
		return false;
	}
	out.edgePolylines.clear();
	out.edgePolylines.reserve(polylines.size());
	for (const Polyline3d& pl : polylines)
	{
		out.edgePolylines.push_back(pl.xyz);
	}
	(void)t0;
	return true;
}

std::shared_ptr<const BrepImportArtifacts> getOrBuildBrepImportArtifacts(
	const ShapeHandle& shape,
	std::string* errMsg)
{
	if (shape.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return {};
	}
	{
		std::lock_guard<std::mutex> lock(g_cacheMutex);
		for (const CacheEntry& entry : g_cacheEntries)
		{
			if (entry.artifacts && shape.isSame(entry.shapeKey))
			{
				return entry.artifacts;
			}
		}
	}
	auto artifacts = std::make_shared<BrepImportArtifacts>();
	if (!buildBrepImportArtifacts(shape, *artifacts, errMsg))
	{
		return {};
	}
	{
		std::lock_guard<std::mutex> lock(g_cacheMutex);
		for (const CacheEntry& entry : g_cacheEntries)
		{
			if (entry.artifacts && shape.isSame(entry.shapeKey))
			{
				return entry.artifacts;
			}
		}
		CacheEntry entry;
		entry.shapeKey = shape;
		entry.artifacts = artifacts;
		g_cacheEntries.push_back(std::move(entry));
	}
	return artifacts;
}

void clearBrepImportArtifactsCache()
{
	std::lock_guard<std::mutex> lock(g_cacheMutex);
	g_cacheEntries.clear();
}

} // namespace geoalgo
