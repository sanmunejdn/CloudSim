#pragma once

#include "geometry_algorithm_global.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace geoalgo
{

enum class TubularGrindingStage : int
{
	None = 0,
	Segment = 1,
	Centerline = 2,
	TemplatePoints = 3,
	Project = 4,
};

enum class TubularGrindingTemplateKind : int
{
	Helical = 0,
	Circumferential = 1,
	AxialParallel = 2,
	Zigzag = 3,
	Auto = 4,
};

struct TubularGrindingParams
{
	/// 环心 DBSCAN 半径（mm）；0 表示按网格尺度自动估计
	double ringCenterClusterEpsMm = 0.0;
	/// 有效环最少三角面数
	double minRingFaces = 4.0;
	/// 环链合并：相邻环轴线夹角上限（°）
	double axisMergeAngleDeg = 28.0;
	/// 交汇环判定：邻接环数 ≥ 3 且轴线散布超过该值（°）
	double junctionAxisSpreadDeg = 38.0;
	/// 有效管段最少三角面数
	double minSegmentFaces = 40.0;
	/// 法向射线汇聚容差（mm）；0 表示按局部管径自动放宽
	double ringRayConvergenceEpsMm = 0.0;
	/// 法向轴可视化长度（mm）；0 表示按网格尺度自动
	double faceNormalAxisLengthMm = 0.0;
	/// 保留兼容
	double regionGrowAxisAngleDeg = 28.0;

	double sectionSpacingMm = 2.0;
	int minSectionPoints = 8;

	TubularGrindingTemplateKind templateKind = TubularGrindingTemplateKind::Auto;
	int helicalCoils = 8;
	int circumferentialRings = 30;
	int axialMeridians = 24;
	int zigzagPasses = 40;

	double projectionMaxDistMm = 10.0;
};

struct TubularPipeSegment
{
	int id = 0;
	std::vector<int> faceIndices;
	std::array<double, 3> axisHint{{0.0, 0.0, 1.0}};
};

struct TubularCrossSectionRing
{
	int id = 0;
	std::vector<int> faceIndices;
	std::array<double, 3> centerMm{{0.0, 0.0, 0.0}};
	std::array<double, 3> axisHint{{0.0, 0.0, 1.0}};
	double radiusMm = 0.0;
};

struct TubularCenterlineSample
{
	int pipeId = 0;
	double arcLengthMm = 0.0;
	double radiusMm = 0.0;
	std::array<double, 3> positionMm{{0.0, 0.0, 0.0}};
	std::array<double, 3> tangent{{0.0, 0.0, 1.0}};
	std::array<double, 3> normal{{1.0, 0.0, 0.0}};
	std::array<double, 3> binormal{{0.0, 1.0, 0.0}};
};

struct TubularTemplatePoint
{
	int pipeId = 0;
	double paramT = 0.0;
	std::array<double, 3> positionMm{{0.0, 0.0, 0.0}};
	std::array<double, 3> normalMm{{0.0, 0.0, 1.0}};
};

struct TubularProjectedPoint
{
	int pipeId = 0;
	std::array<double, 3> positionMm{{0.0, 0.0, 0.0}};
	std::array<double, 3> normalMm{{0.0, 0.0, 1.0}};
};

struct TubularGrindingProjectedPoints
{
	std::vector<TubularProjectedPoint> points;
};

struct TubularGrindingReport
{
	int pipeCount = 0;
	int ringCount = 0;
	int junctionFaceCount = 0;
	int regionCountBeforeFilter = 0;
	int centerlinePointCount = 0;
	int templatePointCount = 0;
	int projectedPointCount = 0;
	int sectionFitFailCount = 0;
	double projectionHitRate = 0.0;
};

class GEOMETRY_ALGORITHM_API TubularGrindingSession
{
public:
	explicit TubularGrindingSession(std::vector<float> sourceSoup);
	~TubularGrindingSession();

	TubularGrindingSession(const TubularGrindingSession&) = delete;
	TubularGrindingSession& operator=(const TubularGrindingSession&) = delete;

	TubularGrindingStage lastCompleted() const;
	const TubularGrindingReport& report() const;
	TubularGrindingReport& report();
	const std::vector<TubularPipeSegment>& pipeSegments() const;
	const std::vector<TubularCrossSectionRing>& crossSectionRings() const;
	const std::vector<TubularCenterlineSample>& centerlineSamples() const;
	const std::vector<TubularTemplatePoint>& templatePoints() const;
	const std::vector<TubularProjectedPoint>& projectedPoints() const;

private:
	friend GEOMETRY_ALGORITHM_API bool runTubularGrindingStage(
		TubularGrindingSession& session,
		TubularGrindingStage stage,
		const TubularGrindingParams& params,
		std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildSegmentColoredMeshSoup(
		const TubularGrindingSession& session,
		std::vector<float>& outSoup,
		std::vector<float>& outRgbPerVertex,
		std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildRingColoredMeshSoup(
		const TubularGrindingSession& session,
		std::vector<float>& outSoup,
		std::vector<float>& outRgbPerVertex,
		std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildRingCenterPointsCloud(
		const TubularGrindingSession& session,
		std::vector<float>& outXyz,
		std::vector<float>& outRgba,
		std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildFaceNormalAxisLineSegments(
		const TubularGrindingSession& session,
		const TubularGrindingParams& params,
		std::vector<float>& outLineXyz,
		std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildCenterlinePointsCloud(
		const TubularGrindingSession& session,
		std::vector<float>& outXyz,
		std::vector<float>& outRgba,
		std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildCenterlinePolylineXyz(
		const TubularGrindingSession& session,
		std::vector<float>& outXyz,
		std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildTemplatePointsCloud(
		const TubularGrindingSession& session,
		std::vector<float>& outXyz,
		std::vector<float>& outRgba,
		std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildProjectedPointsCloud(
		const TubularGrindingSession& session,
		std::vector<float>& outXyz,
		std::vector<float>& outRgba,
		std::string* errMsg);

	struct Impl;
	std::unique_ptr<Impl> m_impl;
};

using TubularGrindingSessionPtr = std::shared_ptr<TubularGrindingSession>;

GEOMETRY_ALGORITHM_API TubularGrindingSessionPtr createTubularGrindingSession(
	std::vector<float> sourceSoup);

GEOMETRY_ALGORITHM_API bool runTubularGrindingStage(
	TubularGrindingSession& session,
	TubularGrindingStage stage,
	const TubularGrindingParams& params,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildSegmentColoredMeshSoup(
	const TubularGrindingSession& session,
	std::vector<float>& outSoup,
	std::vector<float>& outRgbPerVertex,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildRingColoredMeshSoup(
	const TubularGrindingSession& session,
	std::vector<float>& outSoup,
	std::vector<float>& outRgbPerVertex,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildRingCenterPointsCloud(
	const TubularGrindingSession& session,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildFaceNormalAxisLineSegments(
	const TubularGrindingSession& session,
	const TubularGrindingParams& params,
	std::vector<float>& outLineXyz,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildCenterlinePointsCloud(
	const TubularGrindingSession& session,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildCenterlinePolylineXyz(
	const TubularGrindingSession& session,
	std::vector<float>& outXyz,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildTemplatePointsCloud(
	const TubularGrindingSession& session,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildProjectedPointsCloud(
	const TubularGrindingSession& session,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg = nullptr);

} // namespace geoalgo
