/// @file BrepImportArtifacts.cpp
/// @brief BrepImportArtifacts 实现

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "BrepImportArtifacts.h"

#include "Discretize.h"
#include "ShapeQuery.h"
#include "ViewTessellate.h"
#include "detail/OccIncludes.h"

#include <chrono>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
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
constexpr std::uint32_t kDisplayCacheMagic = 0x48534D43u; // 'CMSH' LE
constexpr std::uint32_t kDisplayCacheVersion = 2u; // v2：不存 displayNormals
// 屏幕级显示上限；超过则放宽弦高/角偏差重剖，避免 0.5° 级密度打爆内存
constexpr std::size_t kMaxDisplayTriangles = 2000000U;

std::mutex g_diskCacheMutex;
std::string g_diskCacheDir; // UTF-8；空=尚未初始化默认
bool g_diskCacheDisabled = false;

std::int64_t elapsedMs(const std::chrono::steady_clock::time_point t0)
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
}

void mixU64(std::uint64_t& h, std::uint64_t v)
{
	h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6U) + (h >> 2U);
}

std::string defaultDiskCacheDirUtf8()
{
#if defined(_WIN32)
	wchar_t tmp[MAX_PATH] = {};
	const DWORD n = GetTempPathW(MAX_PATH, tmp);
	if (n == 0 || n >= MAX_PATH)
	{
		return {};
	}
	std::wstring w = tmp;
	if (!w.empty() && (w.back() == L'\\' || w.back() == L'/'))
	{
		w.pop_back();
	}
	w += L"\\CloudSim\\brep_display_v1";
	const int bytes = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0, nullptr, nullptr);
	if (bytes <= 1)
	{
		return {};
	}
	std::string utf8(static_cast<std::size_t>(bytes - 1), '\0');
	WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, utf8.data(), bytes, nullptr, nullptr);
	return utf8;
#else
	return "/tmp/CloudSim/brep_display_v1";
#endif
}

std::string resolvedDiskCacheDir()
{
	std::lock_guard<std::mutex> lock(g_diskCacheMutex);
	if (g_diskCacheDisabled)
	{
		return {};
	}
	if (g_diskCacheDir.empty())
	{
		g_diskCacheDir = defaultDiskCacheDirUtf8();
	}
	return g_diskCacheDir;
}

bool ensureDirUtf8(const std::string& pathUtf8)
{
	if (pathUtf8.empty())
	{
		return false;
	}
#if defined(_WIN32)
	const int wlen = MultiByteToWideChar(CP_UTF8, 0, pathUtf8.c_str(), -1, nullptr, 0);
	if (wlen <= 1)
	{
		return false;
	}
	std::wstring w(static_cast<std::size_t>(wlen - 1), L'\0');
	MultiByteToWideChar(CP_UTF8, 0, pathUtf8.c_str(), -1, w.data(), wlen);
	for (std::size_t i = 0; i < w.size(); ++i)
	{
		if (w[i] != L'\\' && w[i] != L'/')
		{
			continue;
		}
		if (i == 0)
		{
			continue;
		}
		const std::wstring prefix = w.substr(0, i);
		if (prefix.empty() || (prefix.size() == 2 && prefix[1] == L':'))
		{
			continue;
		}
		CreateDirectoryW(prefix.c_str(), nullptr);
	}
	if (CreateDirectoryW(w.c_str(), nullptr) != 0)
	{
		return true;
	}
	const DWORD err = GetLastError();
	return err == ERROR_ALREADY_EXISTS;
#else
	return true;
#endif
}

TessellateParams phase1DisplayParams()
{
	// 屏幕光滑即可：0.5° 绝对角偏差会在自由曲面上产生千万级三角、数 GB 内存
	TessellateParams disc;
	disc.linearDeflectionRelative = true;
	disc.linearDeflectionMm = 0.001;
	disc.angularDeflectionDeg = 3.0;
	return disc;
}

TessellateParams phase1DisplayParamsCoarse()
{
	TessellateParams disc;
	disc.linearDeflectionRelative = true;
	disc.linearDeflectionMm = 0.002;
	disc.angularDeflectionDeg = 8.0;
	return disc;
}

std::string makeDisplayCacheKey(const ShapeHandle& shape, const TessellateParams& params)
{
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native) || native.IsNull())
	{
		return {};
	}
	std::uint64_t h = 14695981039346656037ULL;
	mixU64(h, params.linearDeflectionRelative ? 1ULL : 0ULL);
	mixU64(h, static_cast<std::uint64_t>(params.linearDeflectionMm * 1.0e9));
	mixU64(h, static_cast<std::uint64_t>(params.angularDeflectionDeg * 1.0e6));

	int faces = 0;
	int edges = 0;
	int verts = 0;
	for (TopExp_Explorer exp(native, TopAbs_FACE); exp.More(); exp.Next())
	{
		++faces;
	}
	for (TopExp_Explorer exp(native, TopAbs_EDGE); exp.More(); exp.Next())
	{
		++edges;
	}
	for (TopExp_Explorer exp(native, TopAbs_VERTEX); exp.More(); exp.Next())
	{
		++verts;
	}
	mixU64(h, static_cast<std::uint64_t>(faces));
	mixU64(h, static_cast<std::uint64_t>(edges));
	mixU64(h, static_cast<std::uint64_t>(verts));

	const ShapeHandle::BoundsMm b = shape.boundingBoxMm();
	if (b.valid)
	{
		auto q = [](double v) -> std::uint64_t {
			return static_cast<std::uint64_t>(static_cast<std::int64_t>(v * 1000.0));
		};
		mixU64(h, q(b.minX));
		mixU64(h, q(b.minY));
		mixU64(h, q(b.minZ));
		mixU64(h, q(b.maxX));
		mixU64(h, q(b.maxY));
		mixU64(h, q(b.maxZ));
	}

	// 抽样面心，降低「同包围盒不同拓扑」碰撞
	int sampled = 0;
	for (TopExp_Explorer exp(native, TopAbs_FACE); exp.More() && sampled < 32; exp.Next(), ++sampled)
	{
		Bnd_Box box;
		BRepBndLib::Add(exp.Current(), box, Standard_False);
		if (box.IsVoid())
		{
			continue;
		}
		Standard_Real xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
		box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
		mixU64(h, static_cast<std::uint64_t>(static_cast<std::int64_t>((xmin + xmax) * 500.0)));
		mixU64(h, static_cast<std::uint64_t>(static_cast<std::int64_t>((ymin + ymax) * 500.0)));
		mixU64(h, static_cast<std::uint64_t>(static_cast<std::int64_t>((zmin + zmax) * 500.0)));
	}

	std::ostringstream oss;
	oss << std::hex << h;
	return oss.str();
}

std::string cacheFilePath(const std::string& dir, const std::string& key)
{
	if (dir.empty() || key.empty())
	{
		return {};
	}
	std::string path = dir;
	if (path.back() != '\\' && path.back() != '/')
	{
		path.push_back('\\');
	}
	path += key;
	path += ".csmsh";
	return path;
}

template <typename T>
bool writePod(std::ofstream& out, const T& v)
{
	out.write(reinterpret_cast<const char*>(&v), static_cast<std::streamsize>(sizeof(T)));
	return static_cast<bool>(out);
}

template <typename T>
bool readPod(std::ifstream& in, T& v)
{
	in.read(reinterpret_cast<char*>(&v), static_cast<std::streamsize>(sizeof(T)));
	return static_cast<bool>(in);
}

bool writeDisplayCacheFile(const std::string& path, const BrepImportArtifacts& art)
{
	if (path.empty() || !art.hasDisplayData())
	{
		return false;
	}
	const std::size_t triCount = art.displaySoup.size() / 9U;
	if (triCount != art.triangleFaceIndex.size())
	{
		return false;
	}
#if defined(_WIN32)
	const int wlen = MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, nullptr, 0);
	if (wlen <= 1)
	{
		return false;
	}
	std::wstring wpath(static_cast<std::size_t>(wlen - 1), L'\0');
	MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, wpath.data(), wlen);
	std::ofstream out(wpath, std::ios::binary | std::ios::trunc);
#else
	std::ofstream out(path, std::ios::binary | std::ios::trunc);
#endif
	if (!out)
	{
		return false;
	}
	if (!writePod(out, kDisplayCacheMagic) || !writePod(out, kDisplayCacheVersion))
	{
		return false;
	}
	const std::uint64_t triU64 = static_cast<std::uint64_t>(triCount);
	const std::int32_t faceCount = art.faceCount;
	if (!writePod(out, triU64) || !writePod(out, faceCount))
	{
		return false;
	}
	out.write(reinterpret_cast<const char*>(art.displaySoup.data()),
			  static_cast<std::streamsize>(art.displaySoup.size() * sizeof(float)));
	out.write(reinterpret_cast<const char*>(art.triangleFaceIndex.data()),
			  static_cast<std::streamsize>(art.triangleFaceIndex.size() * sizeof(int)));
	return static_cast<bool>(out);
}

bool readDisplayCacheFile(const std::string& path, BrepImportArtifacts& outArt)
{
	if (path.empty())
	{
		return false;
	}
#if defined(_WIN32)
	const int wlen = MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, nullptr, 0);
	if (wlen <= 1)
	{
		return false;
	}
	std::wstring wpath(static_cast<std::size_t>(wlen - 1), L'\0');
	MultiByteToWideChar(CP_UTF8, 0, path.c_str(), -1, wpath.data(), wlen);
	std::ifstream in(wpath, std::ios::binary);
#else
	std::ifstream in(path, std::ios::binary);
#endif
	if (!in)
	{
		return false;
	}
	std::uint32_t magic = 0;
	std::uint32_t version = 0;
	if (!readPod(in, magic) || !readPod(in, version) || magic != kDisplayCacheMagic || version != kDisplayCacheVersion)
	{
		return false;
	}
	std::uint64_t triU64 = 0;
	std::int32_t faceCount = 0;
	if (!readPod(in, triU64) || !readPod(in, faceCount) || triU64 == 0U || triU64 > 200000000ULL)
	{
		return false;
	}
	const std::size_t triCount = static_cast<std::size_t>(triU64);
	const std::size_t floatCount = triCount * 9U;
	outArt.displaySoup.resize(floatCount);
	outArt.displayNormals.clear();
	outArt.triangleFaceIndex.resize(triCount);
	outArt.faceCount = faceCount;
	outArt.faceSoups.clear();
	outArt.pickReady.store(false, std::memory_order_release);
	in.read(reinterpret_cast<char*>(outArt.displaySoup.data()),
			static_cast<std::streamsize>(floatCount * sizeof(float)));
	in.read(reinterpret_cast<char*>(outArt.triangleFaceIndex.data()),
			static_cast<std::streamsize>(triCount * sizeof(int)));
	return static_cast<bool>(in) && outArt.hasDisplayData();
}

int countFaces(const ShapeHandle& shape)
{
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native) || native.IsNull())
	{
		return 0;
	}
	int n = 0;
	for (TopExp_Explorer exp(native, TopAbs_FACE); exp.More(); exp.Next())
	{
		++n;
	}
	return n;
}

} // namespace

bool extractFaceSoup(const BrepImportArtifacts& artifacts, int faceIndex, std::vector<float>& outSoup)
{
	outSoup.clear();
	if (faceIndex < 0 || !artifacts.hasDisplayData())
	{
		return false;
	}
	const std::size_t triCount = artifacts.displaySoup.size() / 9U;
	if (triCount != artifacts.triangleFaceIndex.size())
	{
		return false;
	}
	std::size_t reserveTris = 0U;
	for (std::size_t i = 0; i < triCount; ++i)
	{
		if (artifacts.triangleFaceIndex[i] == faceIndex)
		{
			++reserveTris;
		}
	}
	if (reserveTris == 0U)
	{
		return false;
	}
	outSoup.reserve(reserveTris * 9U);
	for (std::size_t i = 0; i < triCount; ++i)
	{
		if (artifacts.triangleFaceIndex[i] != faceIndex)
		{
			continue;
		}
		const std::size_t base = i * 9U;
		outSoup.insert(outSoup.end(), artifacts.displaySoup.begin() + static_cast<std::ptrdiff_t>(base),
					   artifacts.displaySoup.begin() + static_cast<std::ptrdiff_t>(base + 9U));
	}
	return outSoup.size() >= 9U;
}

bool buildBrepImportArtifactsDisplay(const ShapeHandle& shape, BrepImportArtifacts& out,
									 BrepImportBuildTimings* timings, std::string* errMsg)
{
	out.displaySoup.clear();
	out.displayNormals.clear();
	out.triangleFaceIndex.clear();
	out.faceSoups.clear();
	out.faceCount = 0;
	out.pickReady.store(false, std::memory_order_release);
	out.pickShapeKey = shape;
	const auto t0 = std::chrono::steady_clock::now();
	const TessellateParams disc = phase1DisplayParams();

	const std::string cacheDir = resolvedDiskCacheDir();
	const std::string cacheKey = cacheDir.empty() ? std::string() : makeDisplayCacheKey(shape, disc);
	const std::string cachePath = cacheFilePath(cacheDir, cacheKey);
	if (!cachePath.empty() && readDisplayCacheFile(cachePath, out))
	{
		out.pickShapeKey = shape;
		out.pickReady.store(false, std::memory_order_release);
		if (out.faceCount <= 0)
		{
			out.faceCount = countFaces(shape);
		}
		if (timings)
		{
			timings->meshMs = elapsedMs(t0);
			timings->triangleCount = out.displaySoup.size() / 9U;
			timings->displayCacheHit = true;
		}
		return true;
	}

	// 不写 faceSoups / displayNormals：OSG 上屏时现算法线，避免常驻双倍浮点数组
	auto meshOnce = [&](const TessellateParams& p) -> bool {
		out.displaySoup.clear();
		out.triangleFaceIndex.clear();
		return discretizeShapeToSoupPerFace(shape, p, out.displaySoup, out.triangleFaceIndex, nullptr, errMsg);
	};
	if (!meshOnce(disc))
	{
		return false;
	}
	std::size_t triCount = out.displaySoup.size() / 9U;
	if (triCount != out.triangleFaceIndex.size())
	{
		if (errMsg)
		{
			*errMsg = "triangle face index size mismatch";
		}
		return false;
	}
	if (triCount > kMaxDisplayTriangles)
	{
		const TessellateParams coarse = phase1DisplayParamsCoarse();
		if (!meshOnce(coarse))
		{
			return false;
		}
		triCount = out.displaySoup.size() / 9U;
		if (triCount != out.triangleFaceIndex.size())
		{
			if (errMsg)
			{
				*errMsg = "triangle face index size mismatch";
			}
			return false;
		}
	}
	out.faceCount = countFaces(shape);
	out.displayNormals.clear();
	out.displayNormals.shrink_to_fit();
	if (!cachePath.empty() && ensureDirUtf8(cacheDir))
	{
		(void)writeDisplayCacheFile(cachePath, out);
	}
	if (timings)
	{
		timings->meshMs = elapsedMs(t0);
		timings->triangleCount = triCount;
		timings->displayCacheHit = false;
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
				if (timings)
				{
					timings->meshMs = 0;
					timings->triangleCount = entry.artifacts->displaySoup.size() / 9U;
					timings->displayCacheHit = true;
				}
				return entry.artifacts;
			}
		}
	}
	// 有意在锁外做昂贵的 Phase1 构建：网格化耗时可达数秒，持锁会阻塞全部缓存查询。
	// 代价是两线程并发首导同一 shape 会重复构建、败者在下方锁内复查时丢弃——结果正确，仅费工
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
	const std::size_t srcTriCount = source.displaySoup.size() / 9U;
	if (srcTriCount != source.triangleFaceIndex.size())
	{
		if (errMsg)
		{
			*errMsg = "source triangle face index size mismatch";
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
	if (srcIdx <= 0)
	{
		if (errMsg)
		{
			*errMsg = "source has no faces";
		}
		return {};
	}
	if (source.faceCount > 0 && source.faceCount != srcIdx)
	{
		if (errMsg)
		{
			*errMsg = "source faceCount mismatch";
		}
		return {};
	}

	// srcFace -> 该面在 displaySoup 中的三角区间（按面聚合一次，避免反复扫全表）
	std::vector<std::vector<std::size_t>> trisByFace(static_cast<std::size_t>(srcIdx));
	for (std::size_t ti = 0; ti < srcTriCount; ++ti)
	{
		const int fi = source.triangleFaceIndex[ti];
		if (fi >= 0 && fi < srcIdx)
		{
			trisByFace[static_cast<std::size_t>(fi)].push_back(ti);
		}
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
			// IsBound 未命中时逐面 IsPartner 探测：O(dst面数 × src面数)，仅拆 Solid 等小场景可接受
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
		if (mapped < 0 || mapped >= srcIdx)
		{
			continue;
		}
		const std::vector<std::size_t>& tris = trisByFace[static_cast<std::size_t>(mapped)];
		for (const std::size_t ti : tris)
		{
			const std::size_t base = ti * 9U;
			out->displaySoup.insert(out->displaySoup.end(),
									source.displaySoup.begin() + static_cast<std::ptrdiff_t>(base),
									source.displaySoup.begin() + static_cast<std::ptrdiff_t>(base + 9U));
			out->triangleFaceIndex.push_back(dstIdx);
		}
	}
	out->faceCount = dstIdx;
	out->displayNormals.clear();
	if (dstIdx <= 0 || !out->hasDisplayData())
	{
		if (errMsg)
		{
			*errMsg = "sliced artifacts empty";
		}
		return {};
	}
	return out;
}

void clearBrepImportArtifactsCache()
{
	std::lock_guard<std::mutex> lock(g_cacheMutex);
	g_cacheEntries.clear();
}

void setBrepImportDisplayCacheDirectory(const std::string& pathUtf8)
{
	std::lock_guard<std::mutex> lock(g_diskCacheMutex);
	g_diskCacheDir = pathUtf8;
	g_diskCacheDisabled = pathUtf8.empty();
}

std::string brepImportDisplayCacheDirectory()
{
	return resolvedDiskCacheDir();
}

void clearBrepImportDisplayDiskCache()
{
	const std::string dir = resolvedDiskCacheDir();
	if (dir.empty())
	{
		return;
	}
#if defined(_WIN32)
	const int wlen = MultiByteToWideChar(CP_UTF8, 0, dir.c_str(), -1, nullptr, 0);
	if (wlen <= 1)
	{
		return;
	}
	std::wstring wdir(static_cast<std::size_t>(wlen - 1), L'\0');
	MultiByteToWideChar(CP_UTF8, 0, dir.c_str(), -1, wdir.data(), wlen);
	const std::wstring pattern = wdir + L"\\*.csmsh";
	WIN32_FIND_DATAW fd = {};
	const HANDLE h = FindFirstFileW(pattern.c_str(), &fd);
	if (h == INVALID_HANDLE_VALUE)
	{
		return;
	}
	do
	{
		if ((fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0)
		{
			const std::wstring file = wdir + L"\\" + fd.cFileName;
			DeleteFileW(file.c_str());
		}
	} while (FindNextFileW(h, &fd));
	FindClose(h);
#endif
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
