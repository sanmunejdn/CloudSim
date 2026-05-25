#pragma once

#include "cloudsim_plugin_sdk_global.h"

#include "PluginPointCloudTypes.h"

class IPluginDocument;

/// 点云算法宿主 API（1.2.0+）；插件经 IPluginHostContext::pointCloudHost() 获取
class IPluginPointCloudHost
{
public:
	virtual ~IPluginPointCloudHost() = default;

	virtual void downsamplePointCloudVoxel(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudDownsampleVoxelParams& params,
		PluginPointCloudFinishedFn onFinished) = 0;

	virtual void downsamplePointCloudRandom(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudDownsampleRandomParams& params,
		PluginPointCloudFinishedFn onFinished) = 0;

	virtual void cropPointCloudByBox(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudCropBoxParams& params,
		PluginPointCloudFinishedFn onFinished) = 0;

	virtual void cropPointCloudBySphere(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudCropSphereParams& params,
		PluginPointCloudFinishedFn onFinished) = 0;

	virtual void applyRigidTransformToPointCloud(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudRigidTransformParams& params,
		PluginPointCloudFinishedFn onFinished) = 0;

	virtual void removePointCloudOutliers(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudOutlierParams& params,
		PluginPointCloudFinishedFn onFinished) = 0;

	virtual void smoothPointCloudBilateral(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		PluginPointCloudFinishedFn onFinished) = 0;

	virtual void estimatePointCloudNormalsPca(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudNormalsParams& params,
		PluginPointCloudFinishedFn onFinished) = 0;

	virtual void estimatePointCloudNormalsJet(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudNormalsParams& params,
		PluginPointCloudFinishedFn onFinished) = 0;

	virtual void orientPointCloudNormalsMst(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudNormalsParams& params,
		PluginPointCloudFinishedFn onFinished) = 0;

	virtual void preprocessPointCloudForReconstruction(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudPreprocessParams& params,
		PluginPointCloudFinishedFn onFinished) = 0;

	virtual void rigidRegisterPointCloudsIcp(
		IPluginDocument* doc,
		const std::string& sourceBackendIdUtf8,
		const PluginPointCloudIcpParams& params,
		PluginPointCloudFinishedFn onFinished) = 0;

	virtual void deformPointCloudTpsFromControls(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudTpsControlParams& params,
		PluginPointCloudFinishedFn onFinished) = 0;

	virtual void deformPointCloudTpsFitAndDeform(
		IPluginDocument* doc,
		const std::string& sourceBackendIdUtf8,
		const PluginPointCloudTpsFitParams& params,
		PluginPointCloudFinishedFn onFinished) = 0;

	virtual void reconstructMeshPoisson(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudReconstructPoissonParams& params,
		PluginPointCloudFinishedFn onFinished) = 0;

	virtual void reconstructMeshPoissonAuto(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudReconstructPoissonAutoParams& params,
		PluginPointCloudFinishedFn onFinished) = 0;

	virtual void reconstructMeshScaleSpace(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudReconstructScaleSpaceParams& params,
		PluginPointCloudFinishedFn onFinished) = 0;
};
