#ifndef GEOMETRYALGORITHM_BREPIMPORTARTIFACTS_H
#define GEOMETRYALGORITHM_BREPIMPORTARTIFACTS_H

/// @file BrepImportArtifacts.h
/// @brief BREP 导入预处理：显示 soup、面/边离散、按 ShapeHandle 共享缓存

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
/** BREP 导入预处理产物：显示/拾取/线框共用，避免重复 OCCT 离散 */
struct GEOMETRY_ALGORITHM_API BrepImportArtifacts
{
	std::vector<float> displaySoup;          ///< 9T float 显示三角（mm）
	std::vector<float> displayNormals;       ///< 与 displaySoup 同布局，预计算光照法线
	std::vector<int> triangleFaceIndex;      ///< 每三角 → shapeFaceAtIndex 面索引
	std::vector<std::vector<float>> faceSoups; ///< 每面局部 soup（面拾取/高亮）
	std::vector<std::vector<float>> edgePolylines; ///< Phase2 边线框折线
	std::vector<std::vector<int>> faceEdgeIndices; ///< 每面边界边 global index

	std::atomic<bool> pickReady{false};      ///< Phase2 是否已构建
	mutable std::mutex pickBuildMutex;
	ShapeHandle pickShapeKey;

	bool hasDisplayData() const { return displaySoup.size() >= 9U && (displaySoup.size() % 9U) == 0U; }
};

struct GEOMETRY_ALGORITHM_API BrepImportBuildTimings
{
	std::int64_t meshMs = 0;       ///< Phase1 耗时
	std::int64_t pickMs = 0;       ///< Phase2 耗时
	std::size_t triangleCount = 0;
};

/**
 * Phase1：显示离散 + tri→face 映射 + displayNormals
 * 相对包围盒偏差，避免装配体 0.01mm 绝对网格卡死
 * @return false：null shape 或 mesh 为空
 */
GEOMETRY_ALGORITHM_API bool buildBrepImportArtifactsDisplay(const ShapeHandle& shape, BrepImportArtifacts& out,
															BrepImportBuildTimings* timings = nullptr,
															std::string* errMsg = nullptr);

/**
 * Phase2：collectShapeFaceEdgeIndices + 边线框（0.05mm / 1°）
 * @return false：null shape 或边拓扑失败
 */
GEOMETRY_ALGORITHM_API bool buildBrepImportArtifactsPick(const ShapeHandle& shape, BrepImportArtifacts& out,
														 BrepImportBuildTimings* timings = nullptr,
														 std::string* errMsg = nullptr);

/**
 * 懒构建 Phase2（pickReady + mutex；bind/线框/拾取时调用）
 * @return false：Phase2 构建失败
 */
GEOMETRY_ALGORITHM_API bool ensureBrepImportPickArtifacts(const ShapeHandle& shape, BrepImportArtifacts& artifacts,
														  std::string* errMsg = nullptr);

/** Phase1 + Phase2 全量（兼容旧调用） */
GEOMETRY_ALGORITHM_API bool buildBrepImportArtifacts(const ShapeHandle& shape, BrepImportArtifacts& out,
													 std::string* errMsg = nullptr);

/**
 * 按 ShapeHandle::isSame() 缓存 Phase1；最多 16 条目 LRU
 * @return nullptr：构建失败（errMsg 有详情）
 */
GEOMETRY_ALGORITHM_API std::shared_ptr<BrepImportArtifacts>
getOrBuildBrepImportArtifacts(const ShapeHandle& shape, std::string* errMsg = nullptr,
							  BrepImportBuildTimings* timings = nullptr);

/** 测试或工程切换时清空缓存 */
GEOMETRY_ALGORITHM_API void clearBrepImportArtifactsCache();

/**
 * 从 displaySoup 均匀抽点作为模板点云（STEP 坐标，与 OSG 显示一致）
 * @param maxPoints 抽样上限
 * @return false：无 display soup 或抽样过少
 */
GEOMETRY_ALGORITHM_API bool extractDisplaySoupPointCloud(const ShapeHandle& shape, std::vector<float>& outXyz,
														 std::vector<float>& outNormals, std::size_t maxPoints,
														 std::size_t* outTriangleCount = nullptr,
														 std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_BREPIMPORTARTIFACTS_H
