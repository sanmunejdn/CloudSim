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

	/// 1.8.0+：扫描点云与 CAD 模板 ICP 配准（不写回 B-rep）
	virtual void registerScanToCadTemplate(
		IPluginDocument* doc,
		const std::string& scanBackendIdUtf8,
		const PluginPointCloudTemplateBrepUpdateParams& params,
		PluginPointCloudTemplateBrepRegisterFinishedFn onFinished) = 0;

	/// 1.8.0+：基于已配准缓存更新模板 B-rep 面（须先 registerScanToCadTemplate）
	virtual void updateTemplateBrepFromAlignedScan(
		IPluginDocument* doc,
		const std::string& scanBackendIdUtf8,
		const PluginPointCloudTemplateBrepUpdateParams& params,
		PluginPointCloudTemplateBrepUpdateFinishedFn onFinished) = 0;

	// === 网格后处理（1.9.0+，需宿主链接 VcgAlgorithms.dll） ===

	/// 查询网格信息（UI 线程）
	virtual bool queryMeshInfo(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		PluginMeshInfo& out) const = 0;

	/// quadric-edge-collapse 网格简化
	virtual void simplifyMesh(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginMeshSimplifyParams& params,
		PluginMeshFinishedFn onFinished) = 0;

	/// 网格平滑（Laplacian 或 Implicit Fairing）
	virtual void smoothMesh(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginMeshSmoothParams& params,
		PluginMeshFinishedFn onFinished) = 0;

	/// 网格修复（去退化/重复/非流形/填孔）
	virtual void repairMesh(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginMeshRepairParams& params,
		PluginMeshFinishedFn onFinished) = 0;

	/// 各向同性重网格
	virtual void remeshMeshIsotropic(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginMeshRemeshParams& params,
		PluginMeshFinishedFn onFinished) = 0;

	// === 网格缺陷分析（1.10.0+，只读，overlay 高亮） ===

	virtual void analyzeMeshDefects(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginMeshDefectParams& params,
		PluginMeshDefectFinishedFn onFinished) = 0;

	virtual void clearMeshDefectHighlight(IPluginDocument* doc) = 0;

	// === 多边形裁剪（1.11.0+） ===

	/// 进入 3D 视图多边形绘制；左键加点、右键/双击闭合、Esc 取消
	virtual void pickPolylineFromViewport(
		IPluginDocument* doc,
		PluginPointCloudPolylinePickFinishedFn onFinished) = 0;

	/// 屏幕多边形裁剪（须先 pickPolylineFromViewport 或自行填充 params）
	virtual void cropPointCloudByPolyline(
		IPluginDocument* doc,
		const std::string& backendIdUtf8,
		const PluginPointCloudCropPolylineParams& params,
		PluginPointCloudFinishedFn onFinished) = 0;
};
