#ifndef DATA_GEOMETRYBACKENDOPS_H
#define DATA_GEOMETRYBACKENDOPS_H

/// @file GeometryBackendOps.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 从三角 soup 均匀采样顶点（与 B-rep display soup 提取策略一致），供模板面重构 mesh 输入

#include "geometry_services_global.h"

#include <string>
#include <vector>

#include <MeshSurfaceReconstruction.h>
#include <TemplateBrepUpdate.h>
#include <TubularGrinding.h>
#include <Types.h>

class BrepBackendData;

class PointCloudBackendData;

class MeshBackendData;

namespace geometry_backend_ops

{
GEOMETRY_SERVICES_EXPORT bool discretizeStepToMesh(

	const std::string& stepPathUtf8,

	const geoalgo::MeshDiscretizeParams& params,

	std::vector<float>& soup,

	geoalgo::MeshDiscretizeReport& report,

	std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool discretizeStepFaceToMesh(

	const std::string& stepPathUtf8,

	int faceIndex,

	const geoalgo::MeshDiscretizeParams& params,

	std::vector<float>& soup,

	geoalgo::MeshDiscretizeReport& report,

	std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool discretizePolylineToMesh(

	const std::vector<float>& polylineXyz,

	const geoalgo::MeshDiscretizeParams& params,

	std::vector<float>& soup,

	std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool discretizeStepEdgesToPolylines(

	const std::string& stepPathUtf8,

	const geoalgo::TessellateParams& params,

	std::vector<geoalgo::Polyline3d>& outPolylines,

	std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool intersectStepEdges(

	const std::string& stepPathUtf8,

	int edgeIndex1,

	int edgeIndex2,

	const geoalgo::IntersectionParams& params,

	geoalgo::IntersectionResult& result,

	std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool intersectStepEdgeFace(

	const std::string& stepPathUtf8,

	int edgeIndex,

	int faceIndex,

	const geoalgo::IntersectionParams& params,

	geoalgo::IntersectionResult& result,

	std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool intersectStepFaces(

	const std::string& stepPathUtf8,

	int faceIndex1,

	int faceIndex2,

	const geoalgo::IntersectionParams& params,

	geoalgo::IntersectionResult& result,

	std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool intersectStepFiles(

	const std::string& targetStepPathUtf8,

	const std::string& toolStepPathUtf8,

	const geoalgo::IntersectionParams& params,

	geoalgo::IntersectionResult& result,

	std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool brepBooleanStepFilesToMesh(

	const std::string& targetStepPathUtf8,

	const std::string& toolStepPathUtf8,

	geoalgo::BrepBooleanOp op,

	const geoalgo::MeshDiscretizeParams& meshParams,

	std::vector<float>& outSoup,

	std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool fuseStepEdgesToPolyline(

	const std::string& stepPathUtf8,

	const std::vector<int>& edgeIndices,

	const geoalgo::TessellateParams& disc,

	geoalgo::Polyline3d& out,

	std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool sewStepFacesToMesh(

	const std::string& stepPathUtf8,

	const std::vector<int>& faceIndices,

	double toleranceMm,

	const geoalgo::MeshDiscretizeParams& meshParams,

	std::vector<float>& outSoup,

	std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT void applyQualityPreset(geoalgo::MeshDiscretizeParams& params);

GEOMETRY_SERVICES_EXPORT void fillMeshReport(const std::vector<float>& soup, geoalgo::MeshDiscretizeReport& report);

/// 从三角 soup 均匀采样顶点（与 B-rep display soup 提取策略一致），供模板面重构 mesh 输入
GEOMETRY_SERVICES_EXPORT bool sampleTriangleSoupToPointBuffers(const std::vector<float>& triangleSoup,
												  const std::vector<float>& triangleVertexNormals,
												  std::vector<float>& outXyz, std::vector<float>& outNormals,
												  std::size_t maxPoints, std::string* errMsg = nullptr);

/// mesh → 临时点云视图（几何系顶点 + 继承 worldMatrix），不注册 backend
GEOMETRY_SERVICES_EXPORT bool buildPointCloudFromMeshForTemplateBrep(const MeshBackendData& mesh, PointCloudBackendData& outScan,
														std::size_t maxPoints = 120000U, std::string* errMsg = nullptr);

/// CAD 模板 B-rep + 扫描点云：ICP 对齐后逐面更新几何，输出新 BrepBackendData

GEOMETRY_SERVICES_EXPORT bool
registerScanToCadTemplate(const BrepBackendData& templateBrep, const PointCloudBackendData& scanCloud,
						  geoalgo::TemplateBrepUpdateParams params, geoalgo::TemplateBrepUpdateResult& outReport,
						  std::string* errMsg = nullptr, const std::string& templateStepPathUtf8 = std::string(),
						  geoalgo::TemplateBrepRegistrationCheckpoint* registrationCheckpoint = nullptr);

GEOMETRY_SERVICES_EXPORT bool updateBrepFromAlignedScan(const BrepBackendData& templateBrep, const PointCloudBackendData& scanCloud,
										   geoalgo::TemplateBrepUpdateParams params, BrepBackendData& brepOut,
										   geoalgo::TemplateBrepUpdateResult& outReport, std::string* errMsg = nullptr,
										   const std::string& templateStepPathUtf8 = std::string());

GEOMETRY_SERVICES_EXPORT bool updateBrepFromCadTemplate(

	const BrepBackendData& templateBrep,

	const PointCloudBackendData& scanCloud,

	geoalgo::TemplateBrepUpdateParams params,

	BrepBackendData& brepOut,

	geoalgo::TemplateBrepUpdateResult& outReport,

	std::string* errMsg = nullptr,

	const std::string& templateStepPathUtf8 = std::string());

GEOMETRY_SERVICES_EXPORT bool registrationCoarsePipelineSelfTest(std::string* errMsg = nullptr);

/// Vcg 修复 + 法矢光顺（曲面重构预处理）
GEOMETRY_SERVICES_EXPORT bool preprocessMeshSoupForSurfaceReconstruct(const std::vector<float>& soup,
														 const geoalgo::MeshSurfaceReconstructParams& params,
														 std::vector<float>& outSoup,
														 geoalgo::MeshSurfaceReconstructReport& report,
														 std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT geoalgo::MeshSurfaceReconstructSessionPtr
createMeshSurfaceReconstructSession(std::vector<float> preprocessedSoup);

GEOMETRY_SERVICES_EXPORT bool runMeshSurfaceReconstructStage(geoalgo::MeshSurfaceReconstructSession& session,
												geoalgo::MeshSurfaceReconstructStage stage,
												const geoalgo::MeshSurfaceReconstructParams& params,
												geoalgo::ShapeHandle* outShape,
												geoalgo::MeshSurfaceReconstructReport& report,
												std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool buildPartitionColoredMeshSoup(const geoalgo::MeshSurfaceReconstructSession& session,
											   std::vector<float>& outSoup, std::vector<float>& outRgbPerVertex,
											   std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool buildSamplePointsCloud(const geoalgo::MeshSurfaceReconstructSession& session,
										std::vector<float>& outXyz, std::vector<float>& outRgba,
										std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool buildFitPreviewShape(const geoalgo::MeshSurfaceReconstructSession& session,
									  geoalgo::ShapeHandle& outShape, std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool meshSurfaceReconstructShapeToBrep(const geoalgo::ShapeHandle& shape,
												   std::shared_ptr<BrepBackendData>& outBrep,
												   std::string* errMsg = nullptr);

/// 网格 soup → B 样条 B-rep 曲面重构（含可选 vcg 预处理）
GEOMETRY_SERVICES_EXPORT bool reconstructBrepFromMeshSoup(const std::vector<float>& soup,
											 const geoalgo::MeshSurfaceReconstructParams& params,
											 std::shared_ptr<BrepBackendData>& outBrep,
											 geoalgo::MeshSurfaceReconstructReport& report,
											 std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT geoalgo::TubularGrindingSessionPtr createTubularGrindingSession(std::vector<float> sourceSoup);

GEOMETRY_SERVICES_EXPORT geoalgo::TubularGrindingSessionPtr createTubularGrindingSessionFromPointCloud(std::vector<float> pointXyz);

GEOMETRY_SERVICES_EXPORT bool runTubularGrindingStage(geoalgo::TubularGrindingSession& session, geoalgo::TubularGrindingStage stage,
										 const geoalgo::TubularGrindingParams& params,
										 geoalgo::TubularGrindingReport& report, std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool buildTubularGrindingSegmentColoredMeshSoup(const geoalgo::TubularGrindingSession& session,
															std::vector<float>& outSoup,
															std::vector<float>& outRgbPerVertex,
															std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool buildTubularGrindingFpfhRegionColoredMeshSoup(const geoalgo::TubularGrindingSession& session,
															   std::vector<float>& outSoup,
															   std::vector<float>& outRgbPerVertex,
															   std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool buildTubularGrindingRingColoredMeshSoup(const geoalgo::TubularGrindingSession& session,
														 std::vector<float>& outSoup,
														 std::vector<float>& outRgbPerVertex,
														 std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool buildTubularGrindingRingCenterPointsCloud(const geoalgo::TubularGrindingSession& session,
														   std::vector<float>& outXyz, std::vector<float>& outRgba,
														   std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool buildTubularGrindingFaceNormalAxisLineSegments(const geoalgo::TubularGrindingSession& session,
																const geoalgo::TubularGrindingParams& params,
																std::vector<float>& outLineXyz,
																std::string* errMsg = nullptr);

/// Phase 1 局部轴线线段（双向可视化）
GEOMETRY_SERVICES_EXPORT bool buildTubularGrindingLocalAxisLineSegments(const geoalgo::TubularGrindingSession& session,
														   const geoalgo::TubularGrindingParams& params,
														   std::vector<float>& outLineXyz,
														   std::string* errMsg = nullptr);

/// 椭圆拟合残差报告（每环 RMS + 全局摘要）
GEOMETRY_SERVICES_EXPORT bool computeTubularGrindingEllipseResidualReport(const geoalgo::TubularGrindingSession& session,
															 const geoalgo::TubularGrindingParams& params,
															 std::vector<double>& outPerRingRmsResiduals,
															 std::string& outSummaryText,
															 std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool buildTubularGrindingCenterlinePointsCloud(const geoalgo::TubularGrindingSession& session,
														   std::vector<float>& outXyz, std::vector<float>& outRgba,
														   std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool buildTubularGrindingCenterlinePolylineXyz(const geoalgo::TubularGrindingSession& session,
														   std::vector<float>& outXyz, std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool buildTubularGrindingCenterlinePcaAxisArrowLineSegments(const geoalgo::TubularGrindingSession& session,
																		std::vector<float>& outLineXyz,
																		std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool buildTubularGrindingTemplatePointsCloud(const geoalgo::TubularGrindingSession& session,
														 std::vector<float>& outXyz, std::vector<float>& outRgba,
														 std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool buildTubularGrindingProjectedPointsCloud(const geoalgo::TubularGrindingSession& session,
														  std::vector<float>& outXyz, std::vector<float>& outRgba,
														  std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT int tubularGrindingIterationSnapshotCount(const geoalgo::TubularGrindingSession& session);

GEOMETRY_SERVICES_EXPORT int tubularGrindingIterationSnapshotIteration(const geoalgo::TubularGrindingSession& session,
														  int snapshotIndex);

GEOMETRY_SERVICES_EXPORT bool buildTubularGrindingIterationSnapshotPointsCloud(const geoalgo::TubularGrindingSession& session,
																  int snapshotIndex, std::vector<float>& outXyz,
																  std::vector<float>& outRgba,
																  std::string* errMsg = nullptr);

GEOMETRY_SERVICES_EXPORT bool
buildTubularGrindingIterationSnapshotContractedPointsCloud(const geoalgo::TubularGrindingSession& session,
														   int snapshotIndex, std::vector<float>& outXyz,
														   std::vector<float>& outRgba, std::string* errMsg = nullptr);

} // namespace geometry_backend_ops

#endif // DATA_GEOMETRYBACKENDOPS_H
