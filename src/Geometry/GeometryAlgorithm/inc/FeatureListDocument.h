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
	std::string stepPathUtf8; ///< 本地窄字节 STEP 路径
	std::string frameId = "workpiece";
};

enum class GeometryRefKind
{
	Edge = 0,
	Face,
	Polyline
};

/** 带可选独立工件的几何引用；索引相对所属 shape */
struct GeometryRef
{
	GeometryRefKind kind = GeometryRefKind::Edge;
	std::string role; ///< "" | sourceEdge | targetFace
	WorkpieceRef workpiece; ///< 空 → 回落 FeatureEntry.workpiece
	int index = -1;
	std::vector<float> polylineXyz; ///< kind==Polyline：3N mm
};

struct FeatureGeometry
{
	std::vector<GeometryRef> refs;
};

struct FeatureEntry
{
	std::string featureId;
	std::string strategyId = "EdgeChain"; ///< EdgeChain / FaceBoundary / FaceIntersection 等
	FeatureGeometry geometry;
	nlohmann::json params = nlohmann::json::object(); ///< stepMm、linearDeflectionMm 等
	/// 输出坐标系工件（primary）；空则取 FeatureListDocument::workpiece
	WorkpieceRef workpiece;
};

/// 跨工件叠加时 workpiece 为首个特征的工件，离散点统一在其模型系
struct FeatureListDocument
{
	int schemaVersion = 2;
	WorkpieceRef workpiece;
	std::string defaultStrategyId = "EdgeChain";
	std::vector<FeatureEntry> features;
};

inline const WorkpieceRef& featureEntryWorkpiece(const FeatureListDocument& doc, const FeatureEntry& entry)
{
	return entry.workpiece.backendIdUtf8.empty() ? doc.workpiece : entry.workpiece;
}

inline const WorkpieceRef& geometryRefWorkpiece(const FeatureEntry& entry, const GeometryRef& ref)
{
	return ref.workpiece.backendIdUtf8.empty() ? entry.workpiece : ref.workpiece;
}

inline const WorkpieceRef& geometryRefWorkpiece(const FeatureListDocument& doc, const FeatureEntry& entry,
											   const GeometryRef& ref)
{
	return ref.workpiece.backendIdUtf8.empty() ? featureEntryWorkpiece(doc, entry) : ref.workpiece;
}

inline std::vector<int> collectGeometryIndices(const FeatureGeometry& geometry, GeometryRefKind kind,
											   const std::string& role = {})
{
	std::vector<int> out;
	for (const GeometryRef& ref : geometry.refs)
	{
		if (ref.kind != kind)
		{
			continue;
		}
		if (!role.empty() && ref.role != role)
		{
			continue;
		}
		if (ref.index >= 0)
		{
			out.push_back(ref.index);
		}
	}
	return out;
}

inline const GeometryRef* findGeometryRef(const FeatureGeometry& geometry, const std::string& role)
{
	for (const GeometryRef& ref : geometry.refs)
	{
		if (ref.role == role)
		{
			return &ref;
		}
	}
	return nullptr;
}

inline GeometryRef* findGeometryRef(FeatureGeometry& geometry, const std::string& role)
{
	for (GeometryRef& ref : geometry.refs)
	{
		if (ref.role == role)
		{
			return &ref;
		}
	}
	return nullptr;
}

inline void upsertGeometryRef(FeatureGeometry& geometry, GeometryRef ref)
{
	if (!ref.role.empty())
	{
		if (GeometryRef* existing = findGeometryRef(geometry, ref.role))
		{
			*existing = std::move(ref);
			return;
		}
	}
	geometry.refs.push_back(std::move(ref));
}

inline bool geometryHasKind(const FeatureGeometry& geometry, GeometryRefKind kind)
{
	for (const GeometryRef& ref : geometry.refs)
	{
		if (ref.kind == kind)
		{
			return true;
		}
	}
	return false;
}

inline std::vector<float> collectPolylineXyz(const FeatureGeometry& geometry)
{
	for (const GeometryRef& ref : geometry.refs)
	{
		if (ref.kind == GeometryRefKind::Polyline && !ref.polylineXyz.empty())
		{
			return ref.polylineXyz;
		}
	}
	return {};
}

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
	None = 0,		  ///< 逐行离散后拼接
	LineConnectivity, ///< 相连边合并 wire
	FaceUnion		  ///< 多面 fuse 后离散
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

struct FeatureGeometrySlotSchema
{
	std::string role;
	GeometryRefKind kind = GeometryRefKind::Edge;
	int minCount = 1;
	int maxCount = 1; ///< -1 = 不限
};

struct FeatureGeometrySchema
{
	bool allowCrossWorkpiece = false;
	std::string primaryRole;
	std::vector<FeatureGeometrySlotSchema> slotDefs; ///< 不用名 slots：与 Qt slots 宏冲突
};

struct DiscretizeParams
{
	double stepMm = 2.0;			  ///< 弧长重采样间距（mm）
	double linearDeflectionMm = 0.01; ///< BREP 边/面离散线性偏差（mm）
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
