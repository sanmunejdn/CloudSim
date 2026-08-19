/// @file BrepImportArtifacts.cpp
/// @brief BrepImportArtifacts 实现

#include "BrepImportArtifacts.h"

#include "Discretize.h"
#include "ShapeQuery.h"
#include "ViewTessellate.h"
#include "detail/OccIncludes.h"

#include <chrono>
#include <mutex>
#include <vector>

#include <TopTools_DataMapOfShapeInteger.hxx>

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

bool buildBrepImportArtifactsDisplay(const ShapeHandle& shape, BrepImportArtifacts& out,
									 BrepImportBuildTimings* timings, std::string* errMsg)
{
	out.displaySoup.clear();
	out.displayNormals.clear();
	out.triangleFaceIndex.clear();
	out.faceSoups.clear();
	out.pickReady.store(false, std::memory_order_release);
	out.pickShapeKey = shape;
	const auto t0 = std::chrono::steady_clock::now();
	TessellateParams disc;
	disc.linearDeflectionRelative = true;
	disc.linearDeflectionMm = 0.002; // 相对包围盒；0.01mm 绝对会把装配体打爆
	disc.angularDeflectionDeg = 0.5;
	if (!discretizeShapeToSoupPerFace(shape, disc, out.displaySoup, out.triangleFaceIndex, &out.faceSoups, errMsg))
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

bool buildBrepImportArtifactsPick(const ShapeHandle& shape, BrepImportArtifacts& out, BrepImportBuildTimings* timings,
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
	const bool ok = buildBrepImportArtifactsPick(source, artifacts, nullptr, errMsg);
	return ok;
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

std::shared_ptr<BrepImportArtifacts> getOrBuildBrepImportArtifacts(const ShapeHandle& shape, std::string* errMsg,
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

void putBrepImportArtifacts(const ShapeHandle& shape, std::shared_ptr<BrepImportArtifacts> artifacts)
{
	if (shape.isNull() || !artifacts || !artifacts->hasDisplayData())
	{
		return;
	}
	std::lock_guard<std::mutex> lock(g_cacheMutex);
	for (CacheEntry& entry : g_cacheEntries)
	{
		if (entry.artifacts && shape.isSame(entry.shapeKey))
		{
			entry.artifacts = std::move(artifacts);
			return;
		}
	}
	CacheEntry entry;
	entry.shapeKey = shape;
	entry.artifacts = std::move(artifacts);
	if (g_cacheEntries.size() >= kMaxBrepArtifactsCacheEntries)
	{
		g_cacheEntries.erase(g_cacheEntries.begin());
	}
	g_cacheEntries.push_back(std::move(entry));
}

std::shared_ptr<BrepImportArtifacts> sliceBrepImportArtifactsForShape(const ShapeHandle& sourceShape,
																	  const BrepImportArtifacts& source,
																	  const ShapeHandle& destShape, std::string* errMsg)
{
	TopoDS_Shape srcNative;
	TopoDS_Shape dstNative;
	if (!ShapeHandleAccess::nativeShape(sourceShape, &srcNative) ||
		!ShapeHandleAccess::nativeShape(destShape, &dstNative))
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return {};
	}
	if (!source.hasDisplayData())
	{
		if (errMsg)
		{
			*errMsg = "source artifacts have no display soup";
		}
		return {};
	}

	TopTools_DataMapOfShapeInteger srcFaceIndex;
	int srcIdx = 0;
	for (TopExp_Explorer exp(srcNative, TopAbs_FACE); exp.More(); exp.Next(), ++srcIdx)
	{
		const TopoDS_Shape face = exp.Current();
		if (!srcFaceIndex.IsBound(face))
		{
			srcFaceIndex.Bind(face, srcIdx);
		}
	}
	if (srcIdx <= 0 || source.faceSoups.size() != static_cast<std::size_t>(srcIdx))
	{
		if (errMsg)
		{
			*errMsg = "source faceSoups size mismatch";
		}
		return {};
	}

	auto out = std::make_shared<BrepImportArtifacts>();
	out->pickShapeKey = destShape;
	int dstIdx = 0;
	for (TopExp_Explorer exp(dstNative, TopAbs_FACE); exp.More(); exp.Next(), ++dstIdx)
	{
		const TopoDS_Shape destFace = exp.Current();
		int mapped = -1;
		if (srcFaceIndex.IsBound(destFace))
		{
			mapped = srcFaceIndex.Find(destFace);
		}
		else
		{
			int probe = 0;
			for (TopExp_Explorer srcExp(srcNative, TopAbs_FACE); srcExp.More(); srcExp.Next(), ++probe)
			{
				if (srcExp.Current().IsPartner(destFace))
				{
					mapped = probe;
					break;
				}
			}
		}
		std::vector<float> faceSoup;
		if (mapped >= 0 && static_cast<std::size_t>(mapped) < source.faceSoups.size())
		{
			faceSoup = source.faceSoups[static_cast<std::size_t>(mapped)];
		}
		out->faceSoups.push_back(faceSoup);
		if (faceSoup.size() < 9U || (faceSoup.size() % 9U) != 0U)
		{
			continue;
		}
		const std::size_t triCount = faceSoup.size() / 9U;
		out->triangleFaceIndex.insert(out->triangleFaceIndex.end(), triCount, dstIdx);
		out->displaySoup.insert(out->displaySoup.end(), faceSoup.begin(), faceSoup.end());
	}
	if (dstIdx <= 0 || !out->hasDisplayData())
	{
		if (errMsg)
		{
			*errMsg = "sliced artifacts empty";
		}
		return {};
	}
	computeTriangleSoupNormals(out->displaySoup, out->displayNormals);
	return out;
}

void clearBrepImportArtifactsCache()
{
	std::lock_guard<std::mutex> lock(g_cacheMutex);
	g_cacheEntries.clear();
}

bool extractDisplaySoupPointCloud(const ShapeHandle& shape, std::vector<float>& outXyz, std::vector<float>& outNormals,
								  const std::size_t maxPoints, std::size_t* outTriangleCount, std::string* errMsg)
{
	outXyz.clear();
	outNormals.clear();
	if (outTriangleCount)
	{
		*outTriangleCount = 0U;
	}
	if (shape.isNull())
	{
		if (errMsg)
		{
			*errMsg = "null shape";
		}
		return false;
	}
	if (maxPoints < 3U)
	{
		if (errMsg)
		{
			*errMsg = "maxPoints too small";
		}
		return false;
	}

	const std::shared_ptr<BrepImportArtifacts> artifacts = getOrBuildBrepImportArtifacts(shape, errMsg);
	if (!artifacts || !artifacts->hasDisplayData())
	{
		if (errMsg && errMsg->empty())
		{
			*errMsg = "B-rep display soup unavailable";
		}
		return false;
	}

	const std::vector<float>& soup = artifacts->displaySoup;
	const std::vector<float>& norms = artifacts->displayNormals;
	const bool hasNormals = norms.size() == soup.size();
	const std::size_t triCount = soup.size() / 9U;
	if (outTriangleCount)
	{
		*outTriangleCount = triCount;
	}

	outXyz.reserve(std::min(maxPoints, triCount) * 3U);
	if (hasNormals)
	{
		outNormals.reserve(std::min(maxPoints, triCount) * 3U);
	}

	const std::size_t triStep = std::max<std::size_t>(1U, triCount / maxPoints);
	for (std::size_t tri = 0U; tri < triCount; tri += triStep)
	{
		if ((outXyz.size() / 3U) >= maxPoints)
		{
			break;
		}
		const std::size_t base = tri * 9U;
		for (std::size_t corner = 0U; corner < 3U; ++corner)
		{
			if ((outXyz.size() / 3U) >= maxPoints)
			{
				break;
			}
			const std::size_t vb = base + corner * 3U;
			outXyz.push_back(soup[vb]);
			outXyz.push_back(soup[vb + 1U]);
			outXyz.push_back(soup[vb + 2U]);
			if (hasNormals)
			{
				outNormals.push_back(norms[vb]);
				outNormals.push_back(norms[vb + 1U]);
				outNormals.push_back(norms[vb + 2U]);
			}
		}
	}

	if (outXyz.size() < 9U)
	{
		if (errMsg)
		{
			*errMsg = "display soup produced too few points";
		}
		outXyz.clear();
		outNormals.clear();
		return false;
	}
	return true;
}

} // namespace geoalgo
