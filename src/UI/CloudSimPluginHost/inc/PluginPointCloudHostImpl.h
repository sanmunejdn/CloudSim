#ifndef CLOUDSIMPLUGINHOST_PLUGINPOINTCLOUDHOSTIMPL_H
#define CLOUDSIMPLUGINHOST_PLUGINPOINTCLOUDHOSTIMPL_H

/// @file PluginPointCloudHostImpl.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief PluginPointCloudHostImpl 接口

#include "IPluginPointCloudHost.h"

#include <string>
#include <unordered_map>
#include <vector>

#include <BackendFollowMath.h>
#include <MeshSurfaceReconstruction.h>
#include <TemplateBrepUpdate.h>
#include <TubularGrinding.h>

class PluginHostContext;

class PluginPointCloudHostImpl : public IPluginPointCloudHost
{
public:
	explicit PluginPointCloudHostImpl(PluginHostContext* hostContext);

	void downsamplePointCloudVoxel(IPluginDocument* doc, const std::string& backendIdUtf8,
								   const PluginPointCloudDownsampleVoxelParams& params,
								   PluginPointCloudFinishedFn onFinished) override;

	void downsamplePointCloudRandom(IPluginDocument* doc, const std::string& backendIdUtf8,
									const PluginPointCloudDownsampleRandomParams& params,
									PluginPointCloudFinishedFn onFinished) override;

	void cropPointCloudByBox(IPluginDocument* doc, const std::string& backendIdUtf8,
							 const PluginPointCloudCropBoxParams& params,
							 PluginPointCloudFinishedFn onFinished) override;

	void cropPointCloudBySphere(IPluginDocument* doc, const std::string& backendIdUtf8,
								const PluginPointCloudCropSphereParams& params,
								PluginPointCloudFinishedFn onFinished) override;

	void applyRigidTransformToPointCloud(IPluginDocument* doc, const std::string& backendIdUtf8,
										 const PluginPointCloudRigidTransformParams& params,
										 PluginPointCloudFinishedFn onFinished) override;

	void removePointCloudOutliers(IPluginDocument* doc, const std::string& backendIdUtf8,
								  const PluginPointCloudOutlierParams& params,
								  PluginPointCloudFinishedFn onFinished) override;

	void smoothPointCloudBilateral(IPluginDocument* doc, const std::string& backendIdUtf8,
								   PluginPointCloudFinishedFn onFinished) override;

	void estimatePointCloudNormalsPca(IPluginDocument* doc, const std::string& backendIdUtf8,
									  const PluginPointCloudNormalsParams& params,
									  PluginPointCloudFinishedFn onFinished) override;

	void estimatePointCloudNormalsJet(IPluginDocument* doc, const std::string& backendIdUtf8,
									  const PluginPointCloudNormalsParams& params,
									  PluginPointCloudFinishedFn onFinished) override;

	void orientPointCloudNormalsMst(IPluginDocument* doc, const std::string& backendIdUtf8,
									const PluginPointCloudNormalsParams& params,
									PluginPointCloudFinishedFn onFinished) override;

	void preprocessPointCloudForReconstruction(IPluginDocument* doc, const std::string& backendIdUtf8,
											   const PluginPointCloudPreprocessParams& params,
											   PluginPointCloudFinishedFn onFinished) override;

	void rigidRegisterPointCloudsIcp(IPluginDocument* doc, const std::string& sourceBackendIdUtf8,
									 const PluginPointCloudIcpParams& params,
									 PluginPointCloudFinishedFn onFinished) override;

	void deformPointCloudTpsFromControls(IPluginDocument* doc, const std::string& backendIdUtf8,
										 const PluginPointCloudTpsControlParams& params,
										 PluginPointCloudFinishedFn onFinished) override;

	void deformPointCloudTpsFitAndDeform(IPluginDocument* doc, const std::string& sourceBackendIdUtf8,
										 const PluginPointCloudTpsFitParams& params,
										 PluginPointCloudFinishedFn onFinished) override;

	void reconstructMeshPoisson(IPluginDocument* doc, const std::string& backendIdUtf8,
								const PluginPointCloudReconstructPoissonParams& params,
								PluginPointCloudFinishedFn onFinished) override;

	void reconstructMeshPoissonAuto(IPluginDocument* doc, const std::string& backendIdUtf8,
									const PluginPointCloudReconstructPoissonAutoParams& params,
									PluginPointCloudFinishedFn onFinished) override;

	void reconstructMeshScaleSpace(IPluginDocument* doc, const std::string& backendIdUtf8,
								   const PluginPointCloudReconstructScaleSpaceParams& params,
								   PluginPointCloudFinishedFn onFinished) override;

	void registerScanToCadTemplate(IPluginDocument* doc, const std::string& scanBackendIdUtf8,
								   const PluginPointCloudTemplateBrepUpdateParams& params,
								   PluginPointCloudTemplateBrepRegisterFinishedFn onFinished) override;

	void updateTemplateBrepFromAlignedScan(IPluginDocument* doc, const std::string& scanBackendIdUtf8,
										   const PluginPointCloudTemplateBrepUpdateParams& params,
										   PluginPointCloudTemplateBrepUpdateFinishedFn onFinished) override;

	// 网格后处理（1.9.0+）
	bool queryMeshInfo(IPluginDocument* doc, const std::string& backendIdUtf8, PluginMeshInfo& out) const override;

	void simplifyMesh(IPluginDocument* doc, const std::string& backendIdUtf8, const PluginMeshSimplifyParams& params,
					  PluginMeshFinishedFn onFinished) override;

	void smoothMesh(IPluginDocument* doc, const std::string& backendIdUtf8, const PluginMeshSmoothParams& params,
					PluginMeshFinishedFn onFinished) override;

	void repairMesh(IPluginDocument* doc, const std::string& backendIdUtf8, const PluginMeshRepairParams& params,
					PluginMeshFinishedFn onFinished) override;

	void remeshMeshIsotropic(IPluginDocument* doc, const std::string& backendIdUtf8,
							 const PluginMeshRemeshParams& params, PluginMeshFinishedFn onFinished) override;

	void analyzeMeshDefects(IPluginDocument* doc, const std::string& backendIdUtf8,
							const PluginMeshDefectParams& params, PluginMeshDefectFinishedFn onFinished) override;

	void clearMeshDefectHighlight(IPluginDocument* doc) override;

	void pickPolylineFromViewport(IPluginDocument* doc, PluginPointCloudPolylinePickFinishedFn onFinished) override;

	void cropPointCloudByPolyline(IPluginDocument* doc, const std::string& backendIdUtf8,
								  const PluginPointCloudCropPolylineParams& params,
								  PluginPointCloudFinishedFn onFinished) override;

	void reconstructSurfaceFromMesh(IPluginDocument* doc, const std::string& meshBackendIdUtf8,
									const PluginMeshSurfaceReconstructParams& params,
									PluginMeshSurfaceReconstructFinishedFn onFinished) override;

	PluginMeshSurfaceReconstructSessionId
	beginMeshSurfaceReconstructSession(IPluginDocument* doc, const std::string& meshBackendIdUtf8) override;

	void runMeshSurfaceReconstructStage(IPluginDocument* doc, const PluginMeshSurfaceReconstructSessionId& sessionId,
										PluginMeshSurfaceReconstructStage stage,
										const PluginMeshSurfaceReconstructParams& params,
										PluginMeshSurfaceReconstructFinishedFn onFinished) override;

	void clearMeshSurfaceReconstructSession(IPluginDocument* doc,
											const PluginMeshSurfaceReconstructSessionId& sessionId) override;

	PluginTubularGrindingSessionId beginTubularGrindingSession(IPluginDocument* doc,
															   const std::string& meshBackendIdUtf8) override;

	void runTubularGrindingStage(IPluginDocument* doc, const PluginTubularGrindingSessionId& sessionId,
								 PluginTubularGrindingStage stage, const PluginTubularGrindingParams& params,
								 PluginTubularGrindingFinishedFn onFinished) override;

	void clearTubularGrindingSession(IPluginDocument* doc, const PluginTubularGrindingSessionId& sessionId) override;

	void nonRigidRegisterSpare(IPluginDocument* doc, const std::string& sourceBackendIdUtf8,
							   const PluginPointCloudSpareParams& params,
							   PluginPointCloudFinishedFn onFinished) override;

	void nonRigidRegisterSdf(IPluginDocument* doc, const std::string& sourceBackendIdUtf8,
							 const PluginPointCloudSdfParams& params, PluginPointCloudFinishedFn onFinished) override;

private:
	struct SurfaceReconHostSession
	{
		std::string sessionId;
		std::string docId;
		std::string meshBackendId;
		std::vector<float> rawSoup;
		std::vector<float> workingSoup;
		geoalgo::MeshSurfaceReconstructSessionPtr geoSession;
		PluginMeshSurfaceReconstructStage lastCompleted = PluginMeshSurfaceReconstructStage::None;
		std::string preprocessedMeshBackendId;
		std::string partitionColoredMeshBackendId;
		std::string samplePointsBackendId;
		std::string fitPreviewBrepBackendId;
		std::string boundaryBlendPreviewBrepBackendId;
		std::string junctionBlendPreviewBrepBackendId;
	};

	struct TubularGrindingHostSession
	{
		std::string sessionId;
		std::string docId;
		std::string meshBackendId;
		std::vector<float> rawSoup;
		std::vector<float> rawPointXyz;
		int inputKind = 0; // 0=mesh, 1=pointcloud
		geoalgo::TubularGrindingSessionPtr geoSession;
		PluginTubularGrindingStage lastCompleted = PluginTubularGrindingStage::None;
		std::string normalAxisLinesBackendId;
		std::string localAxisLinesBackendId;
		std::string centerlinePointsBackendId;
		std::string centerlinePcaAxisBackendId;
		std::string templatePointsBackendId;
		std::string projectedPointsBackendId;
		std::string fpfhRegionColoredMeshBackendId;
		std::vector<std::string> iterationSnapshotBackendIds;
	};

	struct TemplateBrepAlignCache
	{
		IPluginDocument* doc = nullptr;
		std::string scanId;
		std::string templateId;
		BackendMat4 templateWorldMatrixAtRegister = BackendMat4::identity();
		double icpRmseMm = 0.0;
		geoalgo::TemplateBrepUpdateResult report;
		geoalgo::TemplateBrepRegistrationCheckpoint registrationCheckpoint;
	};

	bool cacheMatches(IPluginDocument* doc, const std::string& scanId, const std::string& templateId) const;

	void eraseSurfaceReconSession(const std::string& sessionId, IPluginDocument* doc);
	SurfaceReconHostSession* findSurfaceReconSession(const std::string& sessionId, IPluginDocument* doc);

	void eraseTubularGrindingSession(const std::string& sessionId, IPluginDocument* doc);
	TubularGrindingHostSession* findTubularGrindingSession(const std::string& sessionId, IPluginDocument* doc);

	PluginHostContext* m_host = nullptr;
	TemplateBrepAlignCache m_templateBrepAlignCache;
	std::unordered_map<std::string, SurfaceReconHostSession> m_surfaceReconSessions;
	std::unordered_map<std::string, TubularGrindingHostSession> m_tubularGrindingSessions;
};

#endif // CLOUDSIMPLUGINHOST_PLUGINPOINTCLOUDHOSTIMPL_H
