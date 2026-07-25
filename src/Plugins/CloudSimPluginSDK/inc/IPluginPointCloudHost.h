#ifndef CLOUDSIMPLUGINSDK_IPLUGINPOINTCLOUDHOST_H
#define CLOUDSIMPLUGINSDK_IPLUGINPOINTCLOUDHOST_H

/// @file IPluginPointCloudHost.h
/// @brief 点云算法宿主 API（1.2.0+）；插件经 IPluginHostContext::pointCloudHost() 获取

#include "cloudsim_plugin_sdk_global.h"

#include "PluginPointCloudTypes.h"

class IPluginDocument;

/// 点云算法宿主 API（1.2.0+）；插件经 IPluginHostContext::pointCloudHost() 获取
class IPluginPointCloudHost
{
public:
	virtual ~IPluginPointCloudHost() = default;

	virtual void downsamplePointCloudVoxel(IPluginDocument* doc, const std::string& backendIdUtf8,
										   const PluginPointCloudDownsampleVoxelParams& params,
										   PluginPointCloudFinishedFn onFinished) = 0;

	virtual void downsamplePointCloudRandom(IPluginDocument* doc, const std::string& backendIdUtf8,
											const PluginPointCloudDownsampleRandomParams& params,
											PluginPointCloudFinishedFn onFinished) = 0;

	virtual void cropPointCloudByBox(IPluginDocument* doc, const std::string& backendIdUtf8,
									 const PluginPointCloudCropBoxParams& params,
									 PluginPointCloudFinishedFn onFinished) = 0;

	virtual void cropPointCloudBySphere(IPluginDocument* doc, const std::string& backendIdUtf8,
										const PluginPointCloudCropSphereParams& params,
										PluginPointCloudFinishedFn onFinished) = 0;

	virtual void applyRigidTransformToPointCloud(IPluginDocument* doc, const std::string& backendIdUtf8,
												 const PluginPointCloudRigidTransformParams& params,
												 PluginPointCloudFinishedFn onFinished) = 0;

	virtual void removePointCloudOutliers(IPluginDocument* doc, const std::string& backendIdUtf8,
										  const PluginPointCloudOutlierParams& params,
										  PluginPointCloudFinishedFn onFinished) = 0;

	virtual void smoothPointCloudBilateral(IPluginDocument* doc, const std::string& backendIdUtf8,
										   PluginPointCloudFinishedFn onFinished) = 0;

	virtual void estimatePointCloudNormalsPca(IPluginDocument* doc, const std::string& backendIdUtf8,
											  const PluginPointCloudNormalsParams& params,
											  PluginPointCloudFinishedFn onFinished) = 0;

	virtual void estimatePointCloudNormalsJet(IPluginDocument* doc, const std::string& backendIdUtf8,
											  const PluginPointCloudNormalsParams& params,
											  PluginPointCloudFinishedFn onFinished) = 0;

	virtual void orientPointCloudNormalsMst(IPluginDocument* doc, const std::string& backendIdUtf8,
											const PluginPointCloudNormalsParams& params,
											PluginPointCloudFinishedFn onFinished) = 0;

	virtual void preprocessPointCloudForReconstruction(IPluginDocument* doc, const std::string& backendIdUtf8,
													   const PluginPointCloudPreprocessParams& params,
													   PluginPointCloudFinishedFn onFinished) = 0;

	virtual void rigidRegisterPointCloudsIcp(IPluginDocument* doc, const std::string& sourceBackendIdUtf8,
											 const PluginPointCloudIcpParams& params,
											 PluginPointCloudFinishedFn onFinished) = 0;

	virtual void deformPointCloudTpsFromControls(IPluginDocument* doc, const std::string& backendIdUtf8,
												 const PluginPointCloudTpsControlParams& params,
												 PluginPointCloudFinishedFn onFinished) = 0;

	virtual void deformPointCloudTpsFitAndDeform(IPluginDocument* doc, const std::string& sourceBackendIdUtf8,
												 const PluginPointCloudTpsFitParams& params,
												 PluginPointCloudFinishedFn onFinished) = 0;

	virtual void reconstructMeshPoisson(IPluginDocument* doc, const std::string& backendIdUtf8,
										const PluginPointCloudReconstructPoissonParams& params,
										PluginPointCloudFinishedFn onFinished) = 0;

	virtual void reconstructMeshPoissonAuto(IPluginDocument* doc, const std::string& backendIdUtf8,
											const PluginPointCloudReconstructPoissonAutoParams& params,
											PluginPointCloudFinishedFn onFinished) = 0;

	virtual void reconstructMeshScaleSpace(IPluginDocument* doc, const std::string& backendIdUtf8,
										   const PluginPointCloudReconstructScaleSpaceParams& params,
										   PluginPointCloudFinishedFn onFinished) = 0;

	/// 1.8.0+：扫描数据（点云或 Model 网格）与 CAD 模板 ICP 配准（不写回 B-rep）
	virtual void registerScanToCadTemplate(IPluginDocument* doc, const std::string& scanBackendIdUtf8,
										   const PluginPointCloudTemplateBrepUpdateParams& params,
										   PluginPointCloudTemplateBrepRegisterFinishedFn onFinished) = 0;

	/// 1.8.0+：基于已配准缓存更新模板 B-rep 面（须先 registerScanToCadTemplate）
	virtual void updateTemplateBrepFromAlignedScan(IPluginDocument* doc, const std::string& scanBackendIdUtf8,
												   const PluginPointCloudTemplateBrepUpdateParams& params,
												   PluginPointCloudTemplateBrepUpdateFinishedFn onFinished) = 0;

	// === 网格后处理（1.9.0+，需宿主链接 VcgAlgorithms.dll） ===

	/// 查询网格信息（UI 线程）
	virtual bool queryMeshInfo(IPluginDocument* doc, const std::string& backendIdUtf8, PluginMeshInfo& out) const = 0;

	/// quadric-edge-collapse 网格简化
	virtual void simplifyMesh(IPluginDocument* doc, const std::string& backendIdUtf8,
							  const PluginMeshSimplifyParams& params, PluginMeshFinishedFn onFinished) = 0;

	/// 网格平滑（Laplacian 或 Taubin）
	virtual void smoothMesh(IPluginDocument* doc, const std::string& backendIdUtf8,
							const PluginMeshSmoothParams& params, PluginMeshFinishedFn onFinished) = 0;

	/// 网格修复（去退化/重复/非流形/填孔）
	virtual void repairMesh(IPluginDocument* doc, const std::string& backendIdUtf8,
							const PluginMeshRepairParams& params, PluginMeshFinishedFn onFinished) = 0;

	/// 各向同性重网格
	virtual void remeshMeshIsotropic(IPluginDocument* doc, const std::string& backendIdUtf8,
									 const PluginMeshRemeshParams& params, PluginMeshFinishedFn onFinished) = 0;

	// === 网格缺陷分析（1.10.0+，只读，overlay 高亮） ===

	virtual void analyzeMeshDefects(IPluginDocument* doc, const std::string& backendIdUtf8,
									const PluginMeshDefectParams& params, PluginMeshDefectFinishedFn onFinished) = 0;

	virtual void clearMeshDefectHighlight(IPluginDocument* doc) = 0;

	// === 多边形裁剪（1.11.0+） ===

	/// 进入 3D 视图多边形绘制；左键加点、右键/双击闭合、Esc 取消
	virtual void pickPolylineFromViewport(IPluginDocument* doc, PluginPointCloudPolylinePickFinishedFn onFinished) = 0;

	/// 屏幕多边形裁剪（须先 pickPolylineFromViewport 或自行填充 params）
	virtual void cropPointCloudByPolyline(IPluginDocument* doc, const std::string& backendIdUtf8,
										  const PluginPointCloudCropPolylineParams& params,
										  PluginPointCloudFinishedFn onFinished) = 0;

	// === 网格曲面重构（1.12.0+） ===

	/// 三角网格 → B-rep 曲面重构（输出新 BrepModel，源网格保留）
	virtual void reconstructSurfaceFromMesh(IPluginDocument* doc, const std::string& meshBackendIdUtf8,
											const PluginMeshSurfaceReconstructParams& params,
											PluginMeshSurfaceReconstructFinishedFn onFinished) = 0;

	// === 网格曲面重构分阶段（1.13.0+） ===

	virtual PluginMeshSurfaceReconstructSessionId
	beginMeshSurfaceReconstructSession(IPluginDocument* doc, const std::string& meshBackendIdUtf8) = 0;

	virtual void runMeshSurfaceReconstructStage(IPluginDocument* doc,
												const PluginMeshSurfaceReconstructSessionId& sessionId,
												PluginMeshSurfaceReconstructStage stage,
												const PluginMeshSurfaceReconstructParams& params,
												PluginMeshSurfaceReconstructFinishedFn onFinished) = 0;

	virtual void clearMeshSurfaceReconstructSession(IPluginDocument* doc,
													const PluginMeshSurfaceReconstructSessionId& sessionId) = 0;

	// === 管状铸件特征构建分阶段（1.15.0+） ===

	virtual PluginTubularGrindingSessionId beginTubularGrindingSession(IPluginDocument* doc,
																	   const std::string& meshBackendIdUtf8) = 0;

	virtual void runTubularGrindingStage(IPluginDocument* doc, const PluginTubularGrindingSessionId& sessionId,
										 PluginTubularGrindingStage stage, const PluginTubularGrindingParams& params,
										 PluginTubularGrindingFinishedFn onFinished) = 0;

	virtual void clearTubularGrindingSession(IPluginDocument* doc, const PluginTubularGrindingSessionId& sessionId) = 0;

	/// 1.16.0+：SPARE 非刚性配准（点云/网格源 → 点云/网格目标）
	/// 必须追加在接口末尾，禁止在中间插入虚函数（会破坏插件/宿主 vtable 兼容）
	virtual void nonRigidRegisterSpare(IPluginDocument* doc, const std::string& sourceBackendIdUtf8,
									   const PluginPointCloudSpareParams& params,
									   PluginPointCloudFinishedFn onFinished) = 0;

	/// 1.17.0+：SDF/DDF 混合非刚性配准（粗场驱动 + 细默认点-面）
	virtual void nonRigidRegisterSdf(IPluginDocument* doc, const std::string& sourceBackendIdUtf8,
									 const PluginPointCloudSdfParams& params,
									 PluginPointCloudFinishedFn onFinished) = 0;
};

#endif // CLOUDSIMPLUGINSDK_IPLUGINPOINTCLOUDHOST_H
