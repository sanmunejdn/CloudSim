#ifndef GEOMETRYALGORITHM_FEATUREDISCRETIZERBRIDGE_H
#define GEOMETRYALGORITHM_FEATUREDISCRETIZERBRIDGE_H

/// @file FeatureDiscretizerBridge.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief CAD 轨迹特征 v2：Registry + JSON 策略，discretizeFeatureList 为唯一离散入口

#include "geometry_algorithm_global.h"

#include "FeatureDiscretizerConfigRegistry.h"
#include "FeatureDiscretizerRegistry.h"
#include "FeatureListDocument.h"
#include "ShapeHandle.h"

#include <string>
#include <vector>

namespace geoalgo
{
/** 注册全部内置离散策略（EdgeChain / FaceBoundary 等） */
GEOMETRY_ALGORITHM_API void ensureFeatureDiscretizersRegistered();

/**
 * 从 resource/feature/discretizers/ 加载外置 JSON 参数模板
 * @param resourceBaseDir 通常为 bin/x64(d)/resource/feature/
 * @return false：目录或 JSON 读取失败
 */
GEOMETRY_ALGORITHM_API bool ensureFeatureDiscretizerConfigsLoaded(const std::string& resourceBaseDir,
																  std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API FeatureDiscretizerRegistry& featureDiscretizerRegistry();
GEOMETRY_ALGORITHM_API FeatureDiscretizerConfigRegistry& featureDiscretizerConfigRegistry();

GEOMETRY_ALGORITHM_API const IFeatureDiscretizer* featureDiscretizerGet(const std::string& strategyId);
GEOMETRY_ALGORITHM_API std::vector<std::string> featureDiscretizerListStrategyIds();

GEOMETRY_ALGORITHM_API std::vector<FeatureDiscretizerParamField>
featureDiscretizerAllParamFields(const std::string& strategyId);

/**
 * v2 特征列表 → RawPath；Coordinator 按各策略 mergePolicy 合并
 * @return false：文档校验失败、shape 无效或任一行离散失败
 */
GEOMETRY_ALGORITHM_API bool discretizeFeatureList(const FeatureListDocument& doc, RawPath& out,
												  std::string* errMsg = nullptr);

/** 带已加载 ShapeHandle，避免重复读 STEP */
GEOMETRY_ALGORITHM_API bool discretizeFeatureList(const FeatureListDocument& doc, const ShapeHandle& shape,
												  RawPath& out, std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool featureListFromJson(const std::string& jsonUtf8, FeatureListDocument& out,
												std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API std::string featureListToJson(const FeatureListDocument& doc);

/**
 * 结构 + 策略参数校验（不读 STEP）
 * @return false：schemaVersion≠2、featureId 重复、strategyId 未知等
 */
GEOMETRY_ALGORITHM_API bool validateFeatureListDocument(const FeatureListDocument& doc, std::string* errMsg = nullptr);

/** 附加校验 geometry 索引在 shape 范围内 */
GEOMETRY_ALGORITHM_API bool validateFeatureListDocumentWithShape(const FeatureListDocument& doc,
																 const ShapeHandle& shape,
																 std::string* errMsg = nullptr);

/**
 * 从 STEP 枚举候选特征（边链/面界等）
 * @return false：stepPathUtf8 空或读 STEP 失败
 */
GEOMETRY_ALGORITHM_API bool enumerateFeatureCatalog(const WorkpieceRef& workpiece, FeatureCatalog& out,
													std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool enumerateFeatureCatalog(const WorkpieceRef& workpiece, const ShapeHandle& shape,
													FeatureCatalog& out, std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API std::string featureCatalogToJson(const FeatureCatalog& catalog);

/**
 * 按 intentUtf8 启发式从 catalog 生成 FeatureListDocument
 * @return false：catalog 空或无法匹配
 */
GEOMETRY_ALGORITHM_API bool suggestFeaturesFromCatalog(const FeatureCatalog& catalog, const std::string& intentUtf8,
													   FeatureListDocument& out, std::string* errMsg = nullptr);

/** 计算特征锚点（UI 标注） */
GEOMETRY_ALGORITHM_API bool computeFeatureAnchor(const WorkpieceRef& workpiece, const FeatureGeometry& geometry,
												 FeatureAnchor& out, std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool computeFeatureAnchor(const WorkpieceRef& workpiece, const ShapeHandle& shape,
												 const FeatureGeometry& geometry, FeatureAnchor& out,
												 std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_FEATUREDISCRETIZERBRIDGE_H
