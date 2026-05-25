#pragma once

#include "IPluginPointCloudHost.h"

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

private:
	PluginHostContext* m_host = nullptr;
};
