#pragma once

#include "FeatureDiscretizerConfigRegistry.h"
#include "FeatureDiscretizerRegistry.h"
#include "FeatureListDocument.h"
#include "ShapeHandle.h"
#include "geometry_algorithm_global.h"

#include <string>
#include <vector>

namespace geoalgo
{

GEOMETRY_ALGORITHM_API void ensureFeatureDiscretizersRegistered();

GEOMETRY_ALGORITHM_API bool ensureFeatureDiscretizerConfigsLoaded(
	const std::string& resourceBaseDir,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API FeatureDiscretizerRegistry& featureDiscretizerRegistry();
GEOMETRY_ALGORITHM_API FeatureDiscretizerConfigRegistry& featureDiscretizerConfigRegistry();

GEOMETRY_ALGORITHM_API const IFeatureDiscretizer* featureDiscretizerGet(const std::string& strategyId);
GEOMETRY_ALGORITHM_API std::vector<std::string> featureDiscretizerListStrategyIds();

GEOMETRY_ALGORITHM_API std::vector<FeatureDiscretizerParamField> featureDiscretizerAllParamFields(
	const std::string& strategyId);

GEOMETRY_ALGORITHM_API bool discretizeFeatureList(
	const FeatureListDocument& doc,
	RawPath& out,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeFeatureList(
	const FeatureListDocument& doc,
	const ShapeHandle& shape,
	RawPath& out,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool featureListFromJson(
	const std::string& jsonUtf8,
	FeatureListDocument& out,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API std::string featureListToJson(const FeatureListDocument& doc);

GEOMETRY_ALGORITHM_API bool validateFeatureListDocument(
	const FeatureListDocument& doc,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool validateFeatureListDocumentWithShape(
	const FeatureListDocument& doc,
	const ShapeHandle& shape,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool enumerateFeatureCatalog(
	const WorkpieceRef& workpiece,
	FeatureCatalog& out,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool enumerateFeatureCatalog(
	const WorkpieceRef& workpiece,
	const ShapeHandle& shape,
	FeatureCatalog& out,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API std::string featureCatalogToJson(const FeatureCatalog& catalog);

GEOMETRY_ALGORITHM_API bool suggestFeaturesFromCatalog(
	const FeatureCatalog& catalog,
	const std::string& intentUtf8,
	FeatureListDocument& out,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool computeFeatureAnchor(
	const WorkpieceRef& workpiece,
	const FeatureGeometry& geometry,
	FeatureAnchor& out,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool computeFeatureAnchor(
	const WorkpieceRef& workpiece,
	const ShapeHandle& shape,
	const FeatureGeometry& geometry,
	FeatureAnchor& out,
	std::string* errMsg = nullptr);

} // namespace geoalgo
