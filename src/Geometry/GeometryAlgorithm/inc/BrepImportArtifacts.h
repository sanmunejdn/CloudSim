#ifndef GEOMETRYALGORITHM_BREPIMPORTARTIFACTS_H
#define GEOMETRYALGORITHM_BREPIMPORTARTIFACTS_H

/// @file BrepImportArtifacts.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief BREP 导入预处理：显示 soup、面/边离散、按 ShapeHandle 共享缓存

#include "geometry_algorithm_global.h"

#include "ShapeHandle.h"
#include "Types.h"

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
	std::vector<float> displaySoup;				   ///< 9T float 显示三角（mm）
	std::vector<float> displayNormals;			   ///< 可选；Phase1 不填充，OSG 上屏时现算后清空
	std::vector<int> triangleFaceIndex;			   ///< 每三角 → shapeFaceAtIndex 面索引
	std::vector<std::vector<float>> faceSoups;	   ///< 可选；Phase1 不再填充，按需 extractFaceSoup
	std::vector<std::vector<float>> edgePolylines; ///< Phase2 边线框折线
	std::vector<std::vector<int>> faceEdgeIndices; ///< 每面边界边 global index
	int faceCount = 0;							   ///< TopExp FACE 数；0 表示未知
	BrepDisplayLod displayLod = BrepDisplayLod::Medium;

	std::atomic<bool> pickReady{false}; ///< Phase2 是否已构建
	mutable std::mutex pickBuildMutex;
	ShapeHandle pickShapeKey;

	bool hasDisplayData() const { return displaySoup.size() >= 9U && (displaySoup.size() % 9U) == 0U; }
};

struct GEOMETRY_ALGORITHM_API BrepImportBuildTimings
{
	std::int64_t meshMs = 0; ///< Phase1 耗时（含磁盘命中时的读盘）
	std::int64_t pickMs = 0; ///< Phase2 耗时
	std::size_t triangleCount = 0;
	bool displayCacheHit = false; ///< Phase1 是否命中磁盘缓存
};

/** 将 int 钳制为合法 BrepDisplayLod（默认 Medium） */
GEOMETRY_ALGORITHM_API BrepDisplayLod clampBrepDisplayLod(int lod);

/** 各 LOD 对应的相对弦高 / 角偏差 */
GEOMETRY_ALGORITHM_API TessellateParams tessellateParamsForDisplayLod(BrepDisplayLod lod);

/**
 * 从 displaySoup + triangleFaceIndex 抽出单面三角（不依赖 faceSoups）
 * @return false：faceIndex 无效或该面无三角
 */
GEOMETRY_ALGORITHM_API bool extractFaceSoup(const BrepImportArtifacts& artifacts, int faceIndex,
											std::vector<float>& outSoup);

/**
 * Phase1：按 lod 显示离散 + tri→face 映射（不写 faceSoups）
 * @return false：null shape 或 mesh 为空
 */
GEOMETRY_ALGORITHM_API bool buildBrepImportArtifactsDisplay(const ShapeHandle& shape, BrepImportArtifacts& out,
															BrepImportBuildTimings* timings = nullptr,
															std::string* errMsg = nullptr,
															BrepDisplayLod lod = BrepDisplayLod::Medium);

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
													 std::string* errMsg = nullptr,
													 BrepDisplayLod lod = BrepDisplayLod::Medium);

/**
 * 按 ShapeHandle::isSame() + lod 缓存 Phase1；最多 16 条目 LRU
 * @return nullptr：构建失败（errMsg 有详情）
 */
GEOMETRY_ALGORITHM_API std::shared_ptr<BrepImportArtifacts>
getOrBuildBrepImportArtifacts(const ShapeHandle& shape, std::string* errMsg = nullptr,
							  BrepImportBuildTimings* timings = nullptr,
							  BrepDisplayLod lod = BrepDisplayLod::Medium);

/**
 * 把已有 Phase1 写入缓存（指定 lod）
 * shape 空或 artifacts 无显示数据时忽略
 */
GEOMETRY_ALGORITHM_API void putBrepImportArtifacts(const ShapeHandle& shape,
												   std::shared_ptr<BrepImportArtifacts> artifacts,
												   BrepDisplayLod lod = BrepDisplayLod::Medium);

/** 移除某 shape 在全部 lod 下的内存缓存项（换档重剖前调用） */
GEOMETRY_ALGORITHM_API void removeBrepImportArtifactsCacheForShape(const ShapeHandle& shape);

/**
 * 按 dest 面序从 source 的 displaySoup+triangleFaceIndex 切片
 * @return nullptr：对不上面或切片后无三角
 */
GEOMETRY_ALGORITHM_API std::shared_ptr<BrepImportArtifacts>
sliceBrepImportArtifactsForShape(const ShapeHandle& sourceShape, const BrepImportArtifacts& source,
								 const ShapeHandle& destShape, std::string* errMsg = nullptr);

/** 测试或工程切换时清空内存缓存 */
GEOMETRY_ALGORITHM_API void clearBrepImportArtifactsCache();

/**
 * Phase1 磁盘缓存目录（UTF-8）；空字符串禁用
 * 默认：系统 TEMP/CloudSim/brep_display_v1
 */
GEOMETRY_ALGORITHM_API void setBrepImportDisplayCacheDirectory(const std::string& pathUtf8);
GEOMETRY_ALGORITHM_API std::string brepImportDisplayCacheDirectory();

/** 清空磁盘缓存目录内 .csmsh（目录本身保留） */
GEOMETRY_ALGORITHM_API void clearBrepImportDisplayDiskCache();

/**
 * 从 displaySoup 均匀抽点作为模板点云（STEP 坐标，与 OSG 显示一致）
 * @param maxPoints 抽样上限
 * @return false：无 display soup 或抽样过少
 */
GEOMETRY_ALGORITHM_API bool extractDisplaySoupPointCloud(const ShapeHandle& shape, std::vector<float>& outXyz,
														 std::vector<float>& outNormals, std::size_t maxPoints,
														 std::size_t* outTriangleCount = nullptr,
														 std::string* errMsg = nullptr,
														 BrepDisplayLod lod = BrepDisplayLod::Medium);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_BREPIMPORTARTIFACTS_H
