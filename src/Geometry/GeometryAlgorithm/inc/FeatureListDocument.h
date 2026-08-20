#ifndef GEOMETRYALGORITHM_FEATURELISTDOCUMENT_H
#define GEOMETRYALGORITHM_FEATURELISTDOCUMENT_H

/// @file FeatureListDocument.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief CAD 轨迹特征 v2 文档类型：FeatureListDocument / RawPath / 策略参数契约

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
	std::string stepPathUtf8;   ///< 本地窄字节 STEP 路径
	std::string frameId = "workpiece";
};

/** 策略无关几何索引；策略专有参数在 FeatureEntry::params */
struct FeatureGeometry
{
	std::vector<int> edgeIndices;   ///< shapeEdgeAtIndex 顺序
	std::vector<int> faceIndices;   ///< shapeFaceAtIndex 顺序
	std::vector<float> polylineXyz; ///< SyntheticPolyline 等：3N mm
};

struct FeatureEntry
{
	std::string featureId;
	std::string strategyId = "EdgeChain"; ///< EdgeChain / FaceBoundary / FaceIntersection 等
	FeatureGeometry geometry;
	nlohmann::json params = nlohmann::json::object(); ///< stepMm、linearDeflectionMm 等
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

/** 离散输出轨迹；segmentEndExclusive 标记多段分界（Mesh 截面法等多交线） */
struct RawPath
{
	std::string sourceFeatureId;
	std::vector<RawPathPoint> points;
	bool closed = false;
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
	None = 0,           ///< 逐行离散后拼接
	LineConnectivity,   ///< 相连边合并 wire
	FaceUnion           ///< 多面 fuse 后离散
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
	double stepMm = 2.0;                  ///< 弧长重采样间距（mm）
	double linearDeflectionMm = 0.01;   ///< BREP 边/面离散线性偏差（mm）
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

/** 各策略共用的 stepMm / linearDeflectionMm 等字段定义 */
GEOMETRY_ALGORITHM_API std::vector<FeatureDiscretizerParamField> featureDiscretizerCommonParamFields();

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_FEATURELISTDOCUMENT_H
