#ifndef CLOUDSIMPLUGINSDK_IPLUGINGEOMETRYHOST_H
#define CLOUDSIMPLUGINSDK_IPLUGINGEOMETRYHOST_H

/// @file IPluginGeometryHost.h
/// @brief 几何算法宿主 API（1.5.0+）；插件经 IPluginHostContext::geometryHost() 获取

#include "cloudsim_plugin_sdk_global.h"

#include "PluginGeometryTypes.h"

class IPluginDocument;
class QByteArray;
class QString;

/// 几何算法宿主 API（1.5.0+）；插件经 IPluginHostContext::geometryHost() 获取
class IPluginGeometryHost
{
public:
	virtual ~IPluginGeometryHost() = default;

	virtual void discretizeStepToMesh(IPluginDocument* doc, const std::string& stepPathUtf8,
									  const PluginMeshDiscretizeParams& params, const PluginMeshCreateOptions& options,
									  PluginGeometryFinishedFn onFinished) = 0;

	virtual void discretizeBackendToMesh(IPluginDocument* doc, const std::string& stepPathUtf8,
										 const PluginMeshDiscretizeParams& params,
										 const PluginMeshCreateOptions& options,
										 PluginGeometryFinishedFn onFinished) = 0;

	virtual void discretizeBackendFaceToMesh(IPluginDocument* doc, const PluginGeometryStepRef& faceRef,
											 const PluginMeshDiscretizeParams& params,
											 const PluginMeshCreateOptions& options,
											 PluginGeometryFinishedFn onFinished) = 0;

	virtual void discretizeWireToTubeMesh(IPluginDocument* doc, const std::vector<float>& polylineXyz,
										  const PluginMeshDiscretizeParams& params,
										  const PluginMeshCreateOptions& options,
										  PluginGeometryFinishedFn onFinished) = 0;

	virtual void discretizeWireToRibbonMesh(IPluginDocument* doc, const std::vector<float>& polylineXyz,
											const PluginMeshDiscretizeParams& params,
											const PluginMeshCreateOptions& options,
											PluginGeometryFinishedFn onFinished) = 0;

	virtual void discretizePolylineToMesh(IPluginDocument* doc, const std::vector<float>& polylineXyz,
										  const PluginMeshDiscretizeParams& params,
										  const PluginMeshCreateOptions& options,
										  PluginGeometryFinishedFn onFinished) = 0;

	virtual void discretizeBackendEdgesToPolylines(IPluginDocument* doc, const std::string& stepPathUtf8,
												   const PluginMeshDiscretizeParams& params,
												   PluginGeometryFinishedFn onFinished) = 0;

	virtual void intersectEdges(IPluginDocument* doc, const PluginGeometryStepRef& edge1,
								const PluginGeometryStepRef& edge2, const PluginGeometryIntersectionParams& params,
								PluginGeometryFinishedFn onFinished) = 0;

	virtual void intersectEdgeFace(IPluginDocument* doc, const PluginGeometryStepRef& edgeRef,
								   const PluginGeometryStepRef& faceRef, const PluginGeometryIntersectionParams& params,
								   PluginGeometryFinishedFn onFinished) = 0;

	virtual void intersectFaces(IPluginDocument* doc, const PluginGeometryStepRef& face1,
								const PluginGeometryStepRef& face2, const PluginGeometryIntersectionParams& params,
								PluginGeometryFinishedFn onFinished) = 0;

	virtual void intersectBackends(IPluginDocument* doc, const std::string& targetStepPathUtf8,
								   const std::string& toolStepPathUtf8, const PluginGeometryIntersectionParams& params,
								   PluginGeometryFinishedFn onFinished) = 0;

	virtual void brepBooleanToMesh(IPluginDocument* doc, const std::string& targetStepPathUtf8,
								   const std::string& toolStepPathUtf8, const PluginGeometryBrepBooleanParams& params,
								   PluginGeometryFinishedFn onFinished) = 0;

	virtual void fuseWiresToPolyline(IPluginDocument* doc, const std::string& stepPathUtf8,
									 const std::vector<int>& edgeIndices,
									 const PluginGeometryIntersectionParams& params,
									 PluginGeometryFinishedFn onFinished) = 0;

	virtual void sewFacesToMesh(IPluginDocument* doc, const std::string& stepPathUtf8,
								const std::vector<int>& faceIndices, const PluginMeshDiscretizeParams& meshParams,
								const PluginMeshCreateOptions& options, PluginGeometryFinishedFn onFinished) = 0;

	/// 枚举活动文档中可参与 STEP/BRep 运算的后端对象
	virtual bool listComputableBackends(IPluginDocument* doc, std::vector<PluginGeometryBackendEntry>& outBackends,
										QString* outError = nullptr) = 0;

	/// 进入一次视图拾取并返回 STEP edge/face 引用
	virtual void pickStepElementFromViewport(IPluginDocument* doc, const PluginGeometryElementPickRequest& request,
											 PluginGeometryElementPickedFn onFinished) = 0;

	/// 1.21.0+：面 → 草图平面（一期仅平面 Face）
	virtual bool queryFaceSketchPlane(IPluginDocument* doc, const PluginGeometryStepRef& faceRef,
									  PluginSketchPlane& outPlane, QString* outError = nullptr) = 0;

	/// 1.21.0+：草图 overlay（世界折线）
	virtual void setSketchOverlay(IPluginDocument* doc, const std::vector<PluginSketchOverlaySegment>& segments) = 0;
	virtual void clearSketchOverlay(IPluginDocument* doc) = 0;

	/// 1.21.0+：屏幕点 → 草图平面交点（世界 mm）
	virtual bool mapScreenToSketchPlane(IPluginDocument* doc, int screenX, int screenY, const PluginSketchPlane& plane,
										PluginPoint3d& outWorldMm, QString* outError = nullptr) = 0;

	/// 1.21.0+：闭合轮廓 Pad/Pocket → 新或更新 BrepModel
	/// 1.23.0+：优先写入/追加 ParametricBrepModel（见 targetParametricBackendIdUtf8）
	virtual void extrudeSketchProfileToBrep(IPluginDocument* doc, const std::vector<float>& closedPolylineXyzMm,
											const PluginSketchPlane& plane, const PluginSketchExtrudeParams& params,
											PluginGeometryFinishedFn onFinished) = 0;

	/// 1.23.0+：读取参数化 Body 特征链 JSON（parametricHistory）
	virtual bool queryParametricBodyHistoryJson(IPluginDocument* doc, const std::string& backendIdUtf8,
												QByteArray& outJsonUtf8, QString* outError = nullptr) = 0;

	/// 1.23.0+：整表替换特征链并 rebuild（Undo 快照恢复）
	virtual void setParametricBodyHistoryJson(IPluginDocument* doc, const std::string& backendIdUtf8,
											  const QByteArray& historyJsonUtf8,
											  PluginGeometryFinishedFn onFinished) = 0;

	/// 1.24.0+：草图视口输入会话（射线落面后回调）
	virtual bool beginSketchInput(IPluginDocument* doc, const PluginSketchPlane& plane, PluginSketchInputFn onInput,
								  QString* outError = nullptr) = 0;
	virtual void endSketchInput(IPluginDocument* doc) = 0;

	/// 1.26.0+：点选 XY/XZ/YZ 半透明基准面（新建草图默认）
	virtual void pickOriginSketchPlane(IPluginDocument* doc, PluginOriginPlanePickedFn onFinished) = 0;
	virtual void cancelOriginSketchPlanePick(IPluginDocument* doc) = 0;

	/// 1.27.0+：拉伸预览（不写入 Parametric Body）
	virtual void previewSketchExtrude(IPluginDocument* doc, const std::vector<float>& closedPolylineXyzMm,
									  const PluginSketchPlane& plane, const PluginSketchExtrudeParams& params) = 0;
	virtual void clearSketchExtrudePreview(IPluginDocument* doc) = 0;

	/// 1.28.0+：列举文档内 Parametric Body id
	virtual bool listParametricBodyIds(IPluginDocument* doc, std::vector<std::string>& outIds,
									   QString* outError = nullptr) = 0;

	/// 1.28.0+：点选面进入特征编辑；多特征时由插件弹菜单（本 API 返回 Body id + 建议 featureId）
	using PluginParametricFeaturePickedFn =
		std::function<void(bool ok, const QString& error, const QString& backendId, const QString& suggestedFeatureId)>;
	virtual void pickParametricFeatureForEdit(IPluginDocument* doc, PluginParametricFeaturePickedFn onFinished) = 0;

	/// 1.31.1+：扫描预览；失败返回 false 并填 errOut（同时清 staging）
	virtual bool previewSketchSweep(IPluginDocument* doc, const std::vector<float>& profilePolylineXyzMm,
									const std::vector<float>& pathPolylineXyzMm, const PluginSketchSweepParams& params,
									QString* errOut = nullptr) = 0;
	/// 1.31.0+：扫描提交 → Parametric Body
	virtual void sweepSketchProfileToBrep(IPluginDocument* doc, const std::vector<float>& profilePolylineXyzMm,
										  const std::vector<float>& pathPolylineXyzMm,
										  const PluginSketchSweepParams& params,
										  PluginGeometryFinishedFn onFinished) = 0;
};

#endif // CLOUDSIMPLUGINSDK_IPLUGINGEOMETRYHOST_H
