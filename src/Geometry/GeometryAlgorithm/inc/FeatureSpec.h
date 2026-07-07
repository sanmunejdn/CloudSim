#pragma once

#include "geometry_algorithm_global.h"
#include "ShapeHandle.h"
#include "Types.h"

#include <string>
#include <vector>

namespace geoalgo
{

struct Vec3d
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

enum class FeatureKind
{
	EdgeChain,
	FaceBoundary,
	FaceIntersection,
	FaceOffsetCurve,
	FaceUVGrid,
	Composite,
	SyntheticPolyline
};

struct WorkpieceRef
{
	std::string backendIdUtf8;
	std::string stepPathUtf8;
	std::string frameId = "workpiece";
};

struct FeatureRefs
{
	std::vector<int> edgeIndices;
	std::vector<int> faceIndices;
	double offsetMm = 0.0;
	int uvCountU = 32;
	int uvCountV = 32;
	double gridAngleDeg = 0.0;
	std::vector<struct FeatureSpec> children;
	std::vector<float> polylineXyz;
};

struct DiscretizeParams
{
	double stepMm = 2.0;
	double linearDeflectionMm = 0.01;
	bool closedPreserveEndpoint = false;
	bool outputTangent = true;
	bool outputNormal = true;
};

struct FeatureSpec
{
	int schemaVersion = 1;
	std::string featureId;
	WorkpieceRef workpiece;
	FeatureKind kind = FeatureKind::EdgeChain;
	FeatureRefs refs;
	DiscretizeParams discretize;
};

struct RawPathPoint
{
	Point3d positionMm;
	Vec3d tangent;
	Vec3d normal;
	bool hasTangent = false;
	bool hasNormal = false;
};

struct RawPath
{
	FeatureSpec sourceSpec;
	std::vector<RawPathPoint> points;
	bool closed = false;
	/// 各子折线在 points 中的结束下标（不含）；空表示整条为一段
	std::vector<std::size_t> segmentEndExclusive;
};

struct FeatureCandidate
{
	std::string candidateId;
	FeatureKind suggestedKind = FeatureKind::EdgeChain;
	std::string summary;
	FeatureRefs refs;
	double lengthMm = 0.0;
	double areaMm2 = 0.0;
	double dihedralDeg = 0.0;
};

struct FeatureCatalog
{
	std::string stepPathUtf8;
	std::string backendIdUtf8;
	std::vector<FeatureCandidate> candidates;
};

/// 特征标注锚点（STEP 文件坐标 mm）
struct FeatureAnchor
{
	std::string candidateId;
	double anchorXyzMm[3]{};
	double labelOffsetXyzMm[3]{};
	bool hasEdgeSegment = false;
	double edgeEndAXyzMm[3]{};
	double edgeEndBXyzMm[3]{};
};

GEOMETRY_ALGORITHM_API bool computeFeatureAnchor(
	const WorkpieceRef& workpiece,
	const FeatureRefs& refs,
	FeatureAnchor& out,
	std::string* errMsg = nullptr);
GEOMETRY_ALGORITHM_API bool computeFeatureAnchor(
	const WorkpieceRef& workpiece,
	const ShapeHandle& shape,
	const FeatureRefs& refs,
	FeatureAnchor& out,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API const char* featureKindToString(FeatureKind kind);
GEOMETRY_ALGORITHM_API bool featureKindFromString(const std::string& s, FeatureKind& out);

GEOMETRY_ALGORITHM_API bool validateFeatureSpec(const FeatureSpec& spec, std::string* errMsg = nullptr);
GEOMETRY_ALGORITHM_API bool validateFeatureSpecWithShape(const FeatureSpec& spec, std::string* errMsg = nullptr);
GEOMETRY_ALGORITHM_API bool validateFeatureSpecWithShape(
	const FeatureSpec& spec,
	const ShapeHandle& shape,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeFeature(const FeatureSpec& spec, RawPath& out, std::string* errMsg = nullptr);
GEOMETRY_ALGORITHM_API bool discretizeFeature(
	const FeatureSpec& spec,
	const ShapeHandle& shape,
	RawPath& out,
	std::string* errMsg = nullptr);
GEOMETRY_ALGORITHM_API bool discretizeFeatures(
	const std::vector<FeatureSpec>& specs,
	std::vector<RawPath>& out,
	std::string* errMsg = nullptr);
GEOMETRY_ALGORITHM_API bool discretizeFeatures(
	const std::vector<FeatureSpec>& specs,
	const ShapeHandle& shape,
	std::vector<RawPath>& out,
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

GEOMETRY_ALGORITHM_API bool featureSpecFromJson(const std::string& jsonUtf8, FeatureSpec& out, std::string* errMsg = nullptr);
GEOMETRY_ALGORITHM_API std::string featureSpecToJson(const FeatureSpec& spec);
GEOMETRY_ALGORITHM_API std::string featureCatalogToJson(const FeatureCatalog& catalog);

/// 规则启发式：按意图从目录生成 FeatureSpec 列表
GEOMETRY_ALGORITHM_API bool suggestFeaturesFromCatalog(
	const FeatureCatalog& catalog,
	const std::string& intentUtf8,
	std::vector<FeatureSpec>& out,
	std::string* errMsg = nullptr);

} // namespace geoalgo
