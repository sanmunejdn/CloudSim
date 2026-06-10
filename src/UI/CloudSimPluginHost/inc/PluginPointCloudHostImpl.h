#pragma once

#include "IPluginPointCloudHost.h"

#include <TemplateBrepUpdate.h>

#include <string>
#include <vector>

class PluginHostContext;

class PluginPointCloudHostImpl : public IPluginPointCloudHost
{
public:
	explicit PluginPointCloudHostImpl(PluginHostContext* hostContext);

	void downsamplePointCloudVoxel(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudDownsampleVoxelParams& params,
		PluginPointCloudFinishedFn onFinished) override;

	void downsamplePointCloudRandom(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudDownsampleRandomParams& params,
		PluginPointCloudFinishedFn onFinished) override;

	void cropPointCloudByBox(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudCropBoxParams& params,
		PluginPointCloudFinishedFn onFinished) override;

	void cropPointCloudBySphere(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudCropSphereParams& params,
		PluginPointCloudFinishedFn onFinished) override;

	void applyRigidTransformToPointCloud(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudRigidTransformParams& params,
		PluginPointCloudFinishedFn onFinished) override;

	void removePointCloudOutliers(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudOutlierParams& params,
		PluginPointCloudFinishedFn onFinished) override;

	void smoothPointCloudBilateral(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		PluginPointCloudFinishedFn onFinished) override;

	void estimatePointCloudNormalsPca(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudNormalsParams& params,
		PluginPointCloudFinishedFn onFinished) override;

	void estimatePointCloudNormalsJet(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudNormalsParams& params,
		PluginPointCloudFinishedFn onFinished) override;

	void orientPointCloudNormalsMst(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudNormalsParams& params,
		PluginPointCloudFinishedFn onFinished) override;

	void preprocessPointCloudForReconstruction(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudPreprocessParams& params,
		PluginPointCloudFinishedFn onFinished) override;

	void rigidRegisterPointCloudsIcp(
		IPluginDocument* doc,
		const std::string& sourceBackendIdUtf8,
		const PluginPointCloudIcpParams& params,
		PluginPointCloudFinishedFn onFinished) override;

	void deformPointCloudTpsFromControls(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudTpsControlParams& params,
		PluginPointCloudFinishedFn onFinished) override;

	void deformPointCloudTpsFitAndDeform(
		IPluginDocument* doc,
		const std::string& sourceBackendIdUtf8,
		const PluginPointCloudTpsFitParams& params,
		PluginPointCloudFinishedFn onFinished) override;

	void reconstructMeshPoisson(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudReconstructPoissonParams& params,
		PluginPointCloudFinishedFn onFinished) override;

	void reconstructMeshPoissonAuto(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudReconstructPoissonAutoParams& params,
		PluginPointCloudFinishedFn onFinished) override;

	void reconstructMeshScaleSpace(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudReconstructScaleSpaceParams& params,
		PluginPointCloudFinishedFn onFinished) override;

	void registerScanToCadTemplate(
		IPluginDocument* doc,
		const std::string& scanBackendIdUtf8,
		const PluginPointCloudTemplateBrepUpdateParams& params,
		PluginPointCloudTemplateBrepRegisterFinishedFn onFinished) override;

	void updateTemplateBrepFromAlignedScan(
		IPluginDocument* doc,
		const std::string& scanBackendIdUtf8,
		const PluginPointCloudTemplateBrepUpdateParams& params,
		PluginPointCloudTemplateBrepUpdateFinishedFn onFinished) override;

	// 网格后处理（1.9.0+）
	bool queryMeshInfo(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		PluginMeshInfo& out) const override;

	void simplifyMesh(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginMeshSimplifyParams& params,
		PluginMeshFinishedFn onFinished) override;

	void smoothMesh(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginMeshSmoothParams& params,
		PluginMeshFinishedFn onFinished) override;

	void repairMesh(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginMeshRepairParams& params,
		PluginMeshFinishedFn onFinished) override;

	void remeshMeshIsotropic(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginMeshRemeshParams& params,
		PluginMeshFinishedFn onFinished) override;

	void analyzeMeshDefects(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginMeshDefectParams& params,
		PluginMeshDefectFinishedFn onFinished) override;

	void clearMeshDefectHighlight(IPluginDocument* doc) override;

	void pickPolylineFromViewport(
		IPluginDocument* doc,
		PluginPointCloudPolylinePickFinishedFn onFinished) override;

	void cropPointCloudByPolyline(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudCropPolylineParams& params,
		PluginPointCloudFinishedFn onFinished) override;

private:
	struct TemplateBrepAlignCache
	{
		IPluginDocument* doc = nullptr;
		std::string scanId;
		std::string templateId;
		std::vector<float> alignedWorkXyz;
		std::vector<float> alignedWorkNormals;
		geoalgo::TemplateBrepUpdateResult report;
	};

	bool cacheMatches(
		IPluginDocument* doc,
		const std::string& scanId,
		const std::string& templateId) const;

	PluginHostContext* m_host = nullptr;
	TemplateBrepAlignCache m_templateBrepAlignCache;
};
