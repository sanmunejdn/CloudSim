#pragma once

#include "geometry_algorithm_global.h"
#include "ShapeHandle.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace geoalgo
{

/// BREP 导入预处理：显示/拾取/线框共用，避免重复 OCCT 离散
struct GEOMETRY_ALGORITHM_API BrepImportArtifacts
{
	std::vector<float> displaySoup;
	std::vector<float> displayNormals;
	std::vector<int> triangleFaceIndex;
	std::vector<std::vector<float>> faceSoups;
	std::vector<std::vector<float>> edgePolylines;
	std::vector<std::vector<int>> faceEdgeIndices;

	std::atomic<bool> pickReady{false};
	mutable std::mutex pickBuildMutex;
	ShapeHandle pickShapeKey;

	bool hasDisplayData() const
	{
		return displaySoup.size() >= 9U && (displaySoup.size() % 9U) == 0U;
	}
};

struct GEOMETRY_ALGORITHM_API BrepImportBuildTimings
{
	std::int64_t meshMs = 0;
	std::int64_t pickMs = 0;
	std::size_t triangleCount = 0;
};

/// Phase1：显示 soup + 面映射 + 预计算法线
GEOMETRY_ALGORITHM_API bool buildBrepImportArtifactsDisplay(
	const ShapeHandle& shape,
	BrepImportArtifacts& out,
	BrepImportBuildTimings* timings = nullptr,
	std::string* errMsg = nullptr);

/// Phase2：边拓扑 + 线框折线（可延后）
GEOMETRY_ALGORITHM_API bool buildBrepImportArtifactsPick(
	const ShapeHandle& shape,
	BrepImportArtifacts& out,
	BrepImportBuildTimings* timings = nullptr,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool ensureBrepImportPickArtifacts(
	const ShapeHandle& shape,
	BrepImportArtifacts& artifacts,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildBrepImportArtifacts(
	const ShapeHandle& shape,
	BrepImportArtifacts& out,
	std::string* errMsg = nullptr);

/// 按 ShapeHandle 共享 identity 缓存；命中则零成本返回
GEOMETRY_ALGORITHM_API std::shared_ptr<BrepImportArtifacts> getOrBuildBrepImportArtifacts(
	const ShapeHandle& shape,
	std::string* errMsg = nullptr,
	BrepImportBuildTimings* timings = nullptr);

GEOMETRY_ALGORITHM_API void clearBrepImportArtifactsCache();

/// 从 displaySoup 抽取模板点云（STEP 坐标，与 OSG 显示一致）
GEOMETRY_ALGORITHM_API bool extractDisplaySoupPointCloud(
	const ShapeHandle& shape,
	std::vector<float>& outXyz,
	std::vector<float>& outNormals,
	std::size_t maxPoints,
	std::size_t* outTriangleCount = nullptr,
	std::string* errMsg = nullptr);

} // namespace geoalgo
