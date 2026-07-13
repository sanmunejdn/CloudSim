#pragma once



#include "data_global.h"



#include <TemplateBrepUpdate.h>
#include <MeshSurfaceReconstruction.h>
#include <TubularGrinding.h>

#include <Types.h>



#include <string>

#include <vector>



class BrepBackendData;

class PointCloudBackendData;

class MeshBackendData;



namespace geometry_backend_ops

{



DATA_EXPORT bool discretizeStepToMesh(

	const std::string& stepPathUtf8,

	const geoalgo::MeshDiscretizeParams& params,

	std::vector<float>& soup,

	geoalgo::MeshDiscretizeReport& report,

	std::string* errMsg = nullptr);



DATA_EXPORT bool discretizeStepFaceToMesh(

	const std::string& stepPathUtf8,

	int faceIndex,

	const geoalgo::MeshDiscretizeParams& params,

	std::vector<float>& soup,

	geoalgo::MeshDiscretizeReport& report,

	std::string* errMsg = nullptr);



DATA_EXPORT bool discretizePolylineToMesh(

	const std::vector<float>& polylineXyz,

	const geoalgo::MeshDiscretizeParams& params,

	std::vector<float>& soup,

	std::string* errMsg = nullptr);



DATA_EXPORT bool discretizeStepEdgesToPolylines(

	const std::string& stepPathUtf8,

	const geoalgo::TessellateParams& params,

	std::vector<geoalgo::Polyline3d>& outPolylines,

	std::string* errMsg = nullptr);



DATA_EXPORT bool intersectStepEdges(

	const std::string& stepPathUtf8,

	int edgeIndex1,

	int edgeIndex2,

	const geoalgo::IntersectionParams& params,

	geoalgo::IntersectionResult& result,

	std::string* errMsg = nullptr);



DATA_EXPORT bool intersectStepEdgeFace(

	const std::string& stepPathUtf8,

	int edgeIndex,

	int faceIndex,

	const geoalgo::IntersectionParams& params,

	geoalgo::IntersectionResult& result,

	std::string* errMsg = nullptr);



DATA_EXPORT bool intersectStepFaces(

	const std::string& stepPathUtf8,

	int faceIndex1,

	int faceIndex2,

	const geoalgo::IntersectionParams& params,

	geoalgo::IntersectionResult& result,

	std::string* errMsg = nullptr);



DATA_EXPORT bool intersectStepFiles(

	const std::string& targetStepPathUtf8,

	const std::string& toolStepPathUtf8,

	const geoalgo::IntersectionParams& params,

	geoalgo::IntersectionResult& result,

	std::string* errMsg = nullptr);



DATA_EXPORT bool brepBooleanStepFilesToMesh(

	const std::string& targetStepPathUtf8,

	const std::string& toolStepPathUtf8,

	geoalgo::BrepBooleanOp op,

	const geoalgo::MeshDiscretizeParams& meshParams,

	std::vector<float>& outSoup,

	std::string* errMsg = nullptr);



DATA_EXPORT bool fuseStepEdgesToPolyline(

	const std::string& stepPathUtf8,

	const std::vector<int>& edgeIndices,

	const geoalgo::TessellateParams& disc,

	geoalgo::Polyline3d& out,

	std::string* errMsg = nullptr);



DATA_EXPORT bool sewStepFacesToMesh(

	const std::string& stepPathUtf8,

	const std::vector<int>& faceIndices,

	double toleranceMm,

	const geoalgo::MeshDiscretizeParams& meshParams,

	std::vector<float>& outSoup,

	std::string* errMsg = nullptr);



DATA_EXPORT void applyQualityPreset(geoalgo::MeshDiscretizeParams& params);

DATA_EXPORT void fillMeshReport(const std::vector<float>& soup, geoalgo::MeshDiscretizeReport& report);

/// 从三角 soup 均匀采样顶点（与 B-rep display soup 提取策略一致），供模板面重构 mesh 输入
DATA_EXPORT bool sampleTriangleSoupToPointBuffers(
	const std::vector<float>& triangleSoup,
	const std::vector<float>& triangleVertexNormals,
	std::vector<float>& outXyz,
	std::vector<float>& outNormals,
	std::size_t maxPoints,
	std::string* errMsg = nullptr);

/// mesh → 临时点云视图（几何系顶点 + 继承 worldMatrix），不注册 backend
DATA_EXPORT bool buildPointCloudFromMeshForTemplateBrep(
	const MeshBackendData& mesh,
	PointCloudBackendData& outScan,
	std::size_t maxPoints = 120000U,
	std::string* errMsg = nullptr);

/// CAD 模板 B-rep + 扫描点云：ICP 对齐后逐面更新几何，输出新 BrepBackendData

DATA_EXPORT bool registerScanToCadTemplate(
	const BrepBackendData& templateBrep,
	const PointCloudBackendData& scanCloud,
	geoalgo::TemplateBrepUpdateParams params,
	geoalgo::TemplateBrepUpdateResult& outReport,
	std::string* errMsg = nullptr,
	const std::string& templateStepPathUtf8 = std::string(),
	geoalgo::TemplateBrepRegistrationCheckpoint* registrationCheckpoint = nullptr);

DATA_EXPORT bool updateBrepFromAlignedScan(
	const BrepBackendData& templateBrep,
	const PointCloudBackendData& scanCloud,
	geoalgo::TemplateBrepUpdateParams params,
	BrepBackendData& brepOut,
	geoalgo::TemplateBrepUpdateResult& outReport,
	std::string* errMsg = nullptr,
	const std::string& templateStepPathUtf8 = std::string());

DATA_EXPORT bool updateBrepFromCadTemplate(

	const BrepBackendData& templateBrep,

	const PointCloudBackendData& scanCloud,

	geoalgo::TemplateBrepUpdateParams params,

	BrepBackendData& brepOut,

	geoalgo::TemplateBrepUpdateResult& outReport,

	std::string* errMsg = nullptr,

	const std::string& templateStepPathUtf8 = std::string());



DATA_EXPORT bool registrationCoarsePipelineSelfTest(std::string* errMsg = nullptr);

/// Vcg 修复 + 法矢光顺（曲面重构预处理）
DATA_EXPORT bool preprocessMeshSoupForSurfaceReconstruct(
	const std::vector<float>& soup,
	const geoalgo::MeshSurfaceReconstructParams& params,
	std::vector<float>& outSoup,
	geoalgo::MeshSurfaceReconstructReport& report,
	std::string* errMsg = nullptr);

DATA_EXPORT geoalgo::MeshSurfaceReconstructSessionPtr createMeshSurfaceReconstructSession(
	std::vector<float> preprocessedSoup);

DATA_EXPORT bool runMeshSurfaceReconstructStage(
	geoalgo::MeshSurfaceReconstructSession& session,
	geoalgo::MeshSurfaceReconstructStage stage,
	const geoalgo::MeshSurfaceReconstructParams& params,
	geoalgo::ShapeHandle* outShape,
	geoalgo::MeshSurfaceReconstructReport& report,
	std::string* errMsg = nullptr);

DATA_EXPORT bool buildPartitionColoredMeshSoup(
	const geoalgo::MeshSurfaceReconstructSession& session,
	std::vector<float>& outSoup,
	std::vector<float>& outRgbPerVertex,
	std::string* errMsg = nullptr);

DATA_EXPORT bool buildSamplePointsCloud(
	const geoalgo::MeshSurfaceReconstructSession& session,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg = nullptr);

DATA_EXPORT bool buildFitPreviewShape(
	const geoalgo::MeshSurfaceReconstructSession& session,
	geoalgo::ShapeHandle& outShape,
	std::string* errMsg = nullptr);

DATA_EXPORT bool meshSurfaceReconstructShapeToBrep(
	const geoalgo::ShapeHandle& shape,
	std::shared_ptr<BrepBackendData>& outBrep,
	std::string* errMsg = nullptr);

/// 网格 soup → B 样条 B-rep 曲面重构（含可选 vcg 预处理）
DATA_EXPORT bool reconstructBrepFromMeshSoup(
	const std::vector<float>& soup,
	const geoalgo::MeshSurfaceReconstructParams& params,
	std::shared_ptr<BrepBackendData>& outBrep,
	geoalgo::MeshSurfaceReconstructReport& report,
	std::string* errMsg = nullptr);

DATA_EXPORT geoalgo::TubularGrindingSessionPtr createTubularGrindingSession(
	std::vector<float> sourceSoup);

DATA_EXPORT geoalgo::TubularGrindingSessionPtr createTubularGrindingSessionFromPointCloud(
	std::vector<float> pointXyz);

DATA_EXPORT bool runTubularGrindingStage(
	geoalgo::TubularGrindingSession& session,
	geoalgo::TubularGrindingStage stage,
	const geoalgo::TubularGrindingParams& params,
	geoalgo::TubularGrindingReport& report,
	std::string* errMsg = nullptr);

DATA_EXPORT bool buildTubularGrindingSegmentColoredMeshSoup(
	const geoalgo::TubularGrindingSession& session,
	std::vector<float>& outSoup,
	std::vector<float>& outRgbPerVertex,
	std::string* errMsg = nullptr);

DATA_EXPORT bool buildTubularGrindingFpfhRegionColoredMeshSoup(
	const geoalgo::TubularGrindingSession& session,
	std::vector<float>& outSoup,
	std::vector<float>& outRgbPerVertex,
	std::string* errMsg = nullptr);

DATA_EXPORT bool buildTubularGrindingRingColoredMeshSoup(
	const geoalgo::TubularGrindingSession& session,
	std::vector<float>& outSoup,
	std::vector<float>& outRgbPerVertex,
	std::string* errMsg = nullptr);

DATA_EXPORT bool buildTubularGrindingRingCenterPointsCloud(
	const geoalgo::TubularGrindingSession& session,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg = nullptr);

DATA_EXPORT bool buildTubularGrindingFaceNormalAxisLineSegments(
	const geoalgo::TubularGrindingSession& session,
	const geoalgo::TubularGrindingParams& params,
	std::vector<float>& outLineXyz,
	std::string* errMsg = nullptr);

/// Phase 1 局部轴线线段（双向可视化）
DATA_EXPORT bool buildTubularGrindingLocalAxisLineSegments(
	const geoalgo::TubularGrindingSession& session,
	const geoalgo::TubularGrindingParams& params,
	std::vector<float>& outLineXyz,
	std::string* errMsg = nullptr);

/// 椭圆拟合残差报告（每环 RMS + 全局摘要）
DATA_EXPORT bool computeTubularGrindingEllipseResidualReport(
	const geoalgo::TubularGrindingSession& session,
	const geoalgo::TubularGrindingParams& params,
	std::vector<double>& outPerRingRmsResiduals,
	std::string& outSummaryText,
	std::string* errMsg = nullptr);

DATA_EXPORT bool buildTubularGrindingCenterlinePointsCloud(
	const geoalgo::TubularGrindingSession& session,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg = nullptr);

DATA_EXPORT bool buildTubularGrindingCenterlinePolylineXyz(
	const geoalgo::TubularGrindingSession& session,
	std::vector<float>& outXyz,
	std::string* errMsg = nullptr);

DATA_EXPORT bool buildTubularGrindingCenterlinePcaAxisArrowLineSegments(
	const geoalgo::TubularGrindingSession& session,
	std::vector<float>& outLineXyz,
	std::string* errMsg = nullptr);

DATA_EXPORT bool buildTubularGrindingTemplatePointsCloud(
	const geoalgo::TubularGrindingSession& session,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg = nullptr);

DATA_EXPORT bool buildTubularGrindingProjectedPointsCloud(
	const geoalgo::TubularGrindingSession& session,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg = nullptr);

DATA_EXPORT int tubularGrindingIterationSnapshotCount(
	const geoalgo::TubularGrindingSession& session);

DATA_EXPORT int tubularGrindingIterationSnapshotIteration(
	const geoalgo::TubularGrindingSession& session,
	int snapshotIndex);

DATA_EXPORT bool buildTubularGrindingIterationSnapshotPointsCloud(
	const geoalgo::TubularGrindingSession& session,
	int snapshotIndex,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg = nullptr);

DATA_EXPORT bool buildTubularGrindingIterationSnapshotContractedPointsCloud(
	const geoalgo::TubularGrindingSession& session,
	int snapshotIndex,
	std::vector<float>& outXyz,
	std::vector<float>& outRgba,
	std::string* errMsg = nullptr);

} // namespace geometry_backend_ops

