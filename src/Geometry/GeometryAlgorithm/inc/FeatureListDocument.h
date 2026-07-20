#ifndef GEOMETRYALGORITHM_FEATURELISTDOCUMENT_H
#define GEOMETRYALGORITHM_FEATURELISTDOCUMENT_H

/// @file FeatureListDocument.h
/// @brief 策略无关的几何索引；策略专有参数在 FeatureEntry::params

#include "geometry_algorithm_global.h"

#include "Types.h"

#include <cstdint>
#include <string>
#include <vector>

#include <json.hpp>

namespace geoalgo
{
struct Vec3d
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

struct WorkpieceRef
{
	std::string backendIdUtf8;
	std::string stepPathUtf8;
	std::string frameId = "workpiece";
};

/// 策略无关的几何索引；策略专有参数在 FeatureEntry::params
struct FeatureGeometry
{
	std::vector<int> edgeIndices;
	std::vector<int> faceIndices;
	std::vector<float> polylineXyz;
};

struct FeatureEntry
{
	std::string featureId;
	std::string strategyId = "EdgeChain";
	FeatureGeometry geometry;
	nlohmann::json params = nlohmann::json::object();
};

struct FeatureListDocument
{
	int schemaVersion = 2;
	WorkpieceRef workpiece;
	std::string defaultStrategyId = "EdgeChain";
	std::vector<FeatureEntry> features;
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
	std::string sourceFeatureId;
	std::vector<RawPathPoint> points;
	bool closed = false;
	/// 各子折线在 points 中的结束下标（不含）；空表示整条为一段
	std::vector<std::size_t> segmentEndExclusive;
};

enum class GeometryAffinity
{
	Line = 0,
	Face,
	Any
};

enum class MergePolicy
{
	None = 0,
	LineConnectivity,
	FaceUnion
};

enum class FeatureParamType
{
	Double = 0,
	Int,
	Bool,
	Enum,
	Vec3,
	Message
};

struct GEOMETRY_ALGORITHM_API FeatureDiscretizerParamField
{
	std::string key;
	FeatureParamType type = FeatureParamType::Double;
	std::string labelEn;
	std::string labelZh;
	std::string unit;
	std::string group;
	int order = 0;

	double minValue = -1e6;
	double maxValue = 1e6;
	double step = 1.0;
	int minInt = 0;
	int maxInt = 9999;
	double defaultDouble = 0.0;
	int defaultInt = 1;
	bool defaultBool = false;

	std::vector<std::string> enumValues;
	std::vector<std::string> enumLabelsZh;
	std::vector<std::string> enumLabelsEn;

	std::string messageEn;
	std::string messageZh;
};

struct FeatureCandidate
{
	std::string candidateId;
	std::string suggestedStrategyId = "EdgeChain";
	std::string summary;
	FeatureGeometry geometry;
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

struct FeatureAnchor
{
	std::string candidateId;
	double anchorXyzMm[3]{};
	double labelOffsetXyzMm[3]{};
	bool hasEdgeSegment = false;
	double edgeEndAXyzMm[3]{};
	double edgeEndBXyzMm[3]{};
};

struct FeatureDiscretizeInput
{
	WorkpieceRef workpiece;
	FeatureGeometry geometry;
	nlohmann::json params = nlohmann::json::object();
	std::string strategyId;
	std::string featureId;
};

struct DiscretizeParams
{
	double stepMm = 2.0;
	double linearDeflectionMm = 0.01;
	bool closedPreserveEndpoint = false;
	bool outputTangent = true;
	bool outputNormal = true;
};

GEOMETRY_ALGORITHM_API FeatureDiscretizerParamField
doubleFeatureParamField(const std::string& key, const std::string& labelEn, const std::string& labelZh,
						const std::string& unit, double minValue, double maxValue, double step, double defaultValue,
						int order = 0, const std::string& group = "discretize");

GEOMETRY_ALGORITHM_API FeatureDiscretizerParamField intFeatureParamField(const std::string& key,
																		 const std::string& labelEn,
																		 const std::string& labelZh, int minValue,
																		 int maxValue, int defaultValue, int order = 0,
																		 const std::string& group = "discretize");

GEOMETRY_ALGORITHM_API FeatureDiscretizerParamField boolFeatureParamField(const std::string& key,
																		  const std::string& labelEn,
																		  const std::string& labelZh, bool defaultValue,
																		  int order = 0,
																		  const std::string& group = "discretize");

GEOMETRY_ALGORITHM_API FeatureDiscretizerParamField enumFeatureParamField(
	const std::string& key, const std::string& labelEn, const std::string& labelZh,
	const std::vector<std::string>& values, const std::vector<std::string>& labelsZh,
	const std::vector<std::string>& labelsEn, int defaultIndex, int order = 0, const std::string& group = "discretize");

GEOMETRY_ALGORITHM_API std::vector<FeatureDiscretizerParamField> featureDiscretizerCommonParamFields();

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_FEATURELISTDOCUMENT_H
