#ifndef DATA_GEOMETRYREF_H
#define DATA_GEOMETRYREF_H

/// @file GeometryRef.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief GeometryRef 接口

#include "geometry_services_global.h"

#include <string>
#include <vector>

#include <FeatureListDocument.h>
#include <ShapeHandle.h>
#include <json.hpp>

class BackendDataManager;

namespace geometry_backend_ops
{
struct GeometryRef
{
	std::string backendIdUtf8;
	std::string stepPathUtf8;
	std::string frameId = "workpiece";
};

enum class WorkpieceShapeSource
{
	InMemoryBrep,
	StepFileFallback,
	Unavailable
};

GEOMETRY_SERVICES_EXPORT bool resolveGeometryRef(const GeometryRef& ref, geoalgo::WorkpieceRef& out, std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT WorkpieceShapeSource resolveWorkpieceShape(const std::string& backendIdUtf8, BackendDataManager& mgr,
													   const std::string& stepPathUtf8Optional,
													   geoalgo::ShapeHandle& outShape, geoalgo::WorkpieceRef& outRef,
													   std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool discretizeFeatureList(const geoalgo::FeatureListDocument& doc, geoalgo::RawPath& out,
									   std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool discretizeFeatureList(const geoalgo::FeatureListDocument& doc, const geoalgo::ShapeHandle& shape,
									   geoalgo::RawPath& out, std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool featureListFromJson(const std::string& jsonUtf8, geoalgo::FeatureListDocument& out,
									 std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT std::string featureListToJson(const geoalgo::FeatureListDocument& doc);

GEOMETRY_SERVICES_EXPORT bool validateFeatureListDocument(const geoalgo::FeatureListDocument& doc, std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool enumerateFeatureCatalog(const geoalgo::WorkpieceRef& workpiece, geoalgo::FeatureCatalog& out,
										 std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool enumerateFeatureCatalog(const geoalgo::WorkpieceRef& workpiece, const geoalgo::ShapeHandle& shape,
										 geoalgo::FeatureCatalog& out, std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT std::string featureCatalogToJson(const geoalgo::FeatureCatalog& catalog);

GEOMETRY_SERVICES_EXPORT bool suggestFeaturesFromCatalog(const geoalgo::FeatureCatalog& catalog, const std::string& intentUtf8,
											geoalgo::FeatureListDocument& out, std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool computeFeatureAnchor(const geoalgo::WorkpieceRef& workpiece, const geoalgo::FeatureGeometry& geometry,
									  geoalgo::FeatureAnchor& out, std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool computeFeatureAnchor(const geoalgo::WorkpieceRef& workpiece, const geoalgo::ShapeHandle& shape,
									  const geoalgo::FeatureGeometry& geometry, geoalgo::FeatureAnchor& out,
									  std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT void ensureFeatureDiscretizersRegistered();

GEOMETRY_SERVICES_EXPORT bool ensureFeatureDiscretizerConfigsLoaded(const std::string& resourceBaseDir,
													   std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT std::vector<std::string> featureDiscretizerListStrategyIds();

GEOMETRY_SERVICES_EXPORT std::vector<geoalgo::FeatureDiscretizerParamField>
featureDiscretizerAllParamFields(const std::string& strategyId);

GEOMETRY_SERVICES_EXPORT std::string featureDiscretizerDisplayNameZh(const std::string& strategyId);

GEOMETRY_SERVICES_EXPORT geoalgo::GeometryAffinity featureDiscretizerAffinity(const std::string& strategyId);

GEOMETRY_SERVICES_EXPORT nlohmann::json featureDiscretizerDefaultParams(const std::string& strategyId);

GEOMETRY_SERVICES_EXPORT bool buildFeatureEntryFromModelPick(const geoalgo::WorkpieceRef& workpiece,
												const geoalgo::ShapeHandle& shape, const std::string& strategyId,
												bool pickFace, const geoalgo::Point3d& modelPointA,
												const geoalgo::Point3d& modelPointB, geoalgo::FeatureEntry& out,
												std::string* errMsg = nullptr, int knownFaceIndex = -1,
												int knownEdgeIndex = -1);

} // namespace geometry_backend_ops

#endif // DATA_GEOMETRYREF_H
