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
	std::shared_ptr<BrepImportArtifacts> artifacts;
};

std::mutex g_cacheMutex;
std::vector<CacheEntry> g_cacheEntries;

constexpr std::size_t kMaxBrepArtifactsCacheEntries = 16U;

std::int64_t elapsedMs(const std::chrono::steady_clock::time_point t0)
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
}

} // namespace

bool buildBrepImportArtifactsDisplay(
	const ShapeHandle& shape,
	BrepImportArtifacts& out,
	BrepImportBuildTimings* timings,
	std::string* errMsg)
{
	out.displaySoup.clear();
	out.displayNormals.clear();
	out.triangleFaceIndex.clear();
	out.faceSoups.clear();
	out.pickReady.store(false, std::memory_order_release);
	out.pickShapeKey = shape;
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
	if (!out.displaySoup.empty())
	{
		computeTriangleSoupNormals(out.displaySoup, out.displayNormals);
	}
	if (timings)
	{
		timings->meshMs = elapsedMs(t0);
		timings->triangleCount = triCount;
	}
	return true;
}

bool buildBrepImportArtifactsPick(
	const ShapeHandle& shape,
	BrepImportArtifacts& out,
	BrepImportBuildTimings* timings,
	std::string* errMsg)
{
	const auto t0 = std::chrono::steady_clock::now();
	out.edgePolylines.clear();
	out.faceEdgeIndices.clear();
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
	out.edgePolylines.reserve(polylines.size());
	for (const Polyline3d& pl : polylines)
	{
		out.edgePolylines.push_back(pl.xyz);
	}
	out.pickReady.store(true, std::memory_order_release);
	if (timings)
	{
		timings->pickMs = elapsedMs(t0);
	}
	return true;
}

bool ensureBrepImportPickArtifacts(const ShapeHandle& shape, BrepImportArtifacts& artifacts, std::string* errMsg)
{
	if (artifacts.pickReady.load(std::memory_order_acquire))
	{
		return true;
	}
	std::lock_guard<std::mutex> lock(artifacts.pickBuildMutex);
	if (artifacts.pickReady.load(std::memory_order_acquire))
	{
		return true;
	}
	const ShapeHandle source = artifacts.pickShapeKey.isNull() ? shape : artifacts.pickShapeKey;
	return buildBrepImportArtifactsPick(source, artifacts, nullptr, errMsg);
}

bool buildBrepImportArtifacts(const ShapeHandle& shape, BrepImportArtifacts& out, std::string* errMsg)
{
	BrepImportBuildTimings timings;
	if (!buildBrepImportArtifactsDisplay(shape, out, &timings, errMsg))
	{
		return false;
	}
	return buildBrepImportArtifactsPick(shape, out, &timings, errMsg);
}

std::shared_ptr<BrepImportArtifacts> getOrBuildBrepImportArtifacts(
	const ShapeHandle& shape,
	std::string* errMsg,
	BrepImportBuildTimings* timings)
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
	BrepImportBuildTimings localTimings;
	if (!buildBrepImportArtifactsDisplay(shape, *artifacts, timings ? timings : &localTimings, errMsg))
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
		if (g_cacheEntries.size() >= kMaxBrepArtifactsCacheEntries)
		{
			g_cacheEntries.erase(g_cacheEntries.begin());
		}
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
