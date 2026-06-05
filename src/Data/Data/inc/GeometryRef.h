#pragma once

#include "data_global.h"

#include <FeatureSpec.h>
#include <ShapeHandle.h>

#include <string>
#include <vector>

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

DATA_EXPORT bool resolveGeometryRef(
	const GeometryRef& ref,
	geoalgo::WorkpieceRef& out,
	std::string* errMsg = nullptr);

DATA_EXPORT WorkpieceShapeSource resolveWorkpieceShape(
	const std::string& backendIdUtf8,
	BackendDataManager& mgr,
	const std::string& stepPathUtf8Optional,
	geoalgo::ShapeHandle& outShape,
	geoalgo::WorkpieceRef& outRef,
	std::string* errMsg = nullptr);

DATA_EXPORT bool discretizeFeature(
	const geoalgo::FeatureSpec& spec,
	geoalgo::RawPath& out,
	std::string* errMsg = nullptr);

DATA_EXPORT bool discretizeFeature(
	const geoalgo::FeatureSpec& spec,
	const geoalgo::ShapeHandle& shape,
	geoalgo::RawPath& out,
	std::string* errMsg = nullptr);

DATA_EXPORT bool discretizeFeatures(
	const std::vector<geoalgo::FeatureSpec>& specs,
	std::vector<geoalgo::RawPath>& out,
	std::string* errMsg = nullptr);

DATA_EXPORT bool validateFeatureSpec(const geoalgo::FeatureSpec& spec, std::string* errMsg = nullptr);

DATA_EXPORT bool enumerateFeatureCatalog(
	const geoalgo::WorkpieceRef& workpiece,
	geoalgo::FeatureCatalog& out,
	std::string* errMsg = nullptr);

DATA_EXPORT bool enumerateFeatureCatalog(
	const geoalgo::WorkpieceRef& workpiece,
	const geoalgo::ShapeHandle& shape,
	geoalgo::FeatureCatalog& out,
	std::string* errMsg = nullptr);

DATA_EXPORT bool featureSpecFromJson(const std::string& jsonUtf8, geoalgo::FeatureSpec& out, std::string* errMsg = nullptr);
DATA_EXPORT std::string featureSpecToJson(const geoalgo::FeatureSpec& spec);
DATA_EXPORT std::string featureCatalogToJson(const geoalgo::FeatureCatalog& catalog);

DATA_EXPORT bool suggestFeaturesFromCatalog(
	const geoalgo::FeatureCatalog& catalog,
	const std::string& intentUtf8,
	std::vector<geoalgo::FeatureSpec>& out,
	std::string* errMsg = nullptr);

DATA_EXPORT bool computeFeatureAnchor(
	const geoalgo::WorkpieceRef& workpiece,
	const geoalgo::FeatureRefs& refs,
	geoalgo::FeatureAnchor& out,
	std::string* errMsg = nullptr);

DATA_EXPORT bool buildFeatureSpecFromModelPick(
	const geoalgo::WorkpieceRef& workpiece,
	bool pickFace,
	geoalgo::FeatureKind faceKindForPick,
	const geoalgo::Point3d& modelPointA,
	const geoalgo::Point3d& modelPointB,
	geoalgo::FeatureSpec& out,
	std::string* errMsg = nullptr,
	int knownFaceIndex = -1,
	int knownEdgeIndex = -1);

DATA_EXPORT bool buildFeatureSpecFromModelPick(
	const geoalgo::WorkpieceRef& workpiece,
	const geoalgo::ShapeHandle& shape,
	bool pickFace,
	geoalgo::FeatureKind faceKindForPick,
	const geoalgo::Point3d& modelPointA,
	const geoalgo::Point3d& modelPointB,
	geoalgo::FeatureSpec& out,
	std::string* errMsg = nullptr,
	int knownFaceIndex = -1,
	int knownEdgeIndex = -1);

} // namespace geometry_backend_ops
