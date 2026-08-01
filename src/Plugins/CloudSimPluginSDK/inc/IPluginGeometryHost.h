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

	/// 1.40.0+：按面索引离散该面边界边（含内环）为多条折线
	virtual void discretizeBackendFaceEdgesToPolylines(IPluginDocument* doc, const PluginGeometryStepRef& faceRef,
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

	/// 1.49.0+：原点基面 + 模型平面面 + 用户候选面（如 DatumPlane）并行拾取
	virtual void pickSketchSupportPlane(IPluginDocument* doc, const std::vector<PluginSupportPlaneCandidate>& extras,
										PluginSupportPlanePickedFn onFinished) = 0;

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

	/// 1.38.0+：边圆角预览 / 提交
	virtual bool previewFilletEdges(IPluginDocument* doc, const PluginSketchFilletParams& params,
									QString* errOut = nullptr) = 0;
	virtual void filletEdgesToBrep(IPluginDocument* doc, const PluginSketchFilletParams& params,
								   PluginGeometryFinishedFn onFinished) = 0;
	/// 1.38.0+：边倒角预览 / 提交
	virtual bool previewChamferEdges(IPluginDocument* doc, const PluginSketchChamferParams& params,
									 QString* errOut = nullptr) = 0;
	virtual void chamferEdgesToBrep(IPluginDocument* doc, const PluginSketchChamferParams& params,
									PluginGeometryFinishedFn onFinished) = 0;
	/// 1.38.0+：旋转凸台/切除预览 / 提交
	virtual bool previewSketchRevolve(IPluginDocument* doc, const std::vector<float>& profilePolylineXyzMm,
									  const PluginSketchRevolveParams& params, QString* errOut = nullptr) = 0;
	virtual void revolveSketchProfileToBrep(IPluginDocument* doc, const std::vector<float>& profilePolylineXyzMm,
											const PluginSketchRevolveParams& params,
											PluginGeometryFinishedFn onFinished) = 0;
	/// 1.38.0+：线性阵列预览 / 提交
	virtual bool previewLinearPattern(IPluginDocument* doc, const PluginSketchLinearPatternParams& params,
									  QString* errOut = nullptr) = 0;
	virtual void linearPatternBodyToBrep(IPluginDocument* doc, const PluginSketchLinearPatternParams& params,
										PluginGeometryFinishedFn onFinished) = 0;
	/// 1.38.0+：镜像预览 / 提交
	virtual bool previewMirror3d(IPluginDocument* doc, const PluginSketchMirror3dParams& params,
								 QString* errOut = nullptr) = 0;
	virtual void mirror3dBodyToBrep(IPluginDocument* doc, const PluginSketchMirror3dParams& params,
								  PluginGeometryFinishedFn onFinished) = 0;
	/// 1.38.0+：放样预览 / 提交
	virtual bool previewSketchLoft(IPluginDocument* doc, const std::vector<float>& profilePolylineAXyzMm,
								   const std::vector<float>& profilePolylineBXyzMm, const PluginSketchLoftParams& params,
								   QString* errOut = nullptr) = 0;
	virtual void loftSketchProfilesToBrep(IPluginDocument* doc, const std::vector<float>& profilePolylineAXyzMm,
										  const std::vector<float>& profilePolylineBXyzMm,
										  const PluginSketchLoftParams& params,
										  PluginGeometryFinishedFn onFinished) = 0;
	/// 1.38.0+：抽壳预览 / 提交
	virtual bool previewShellFaces(IPluginDocument* doc, const PluginSketchShellParams& params,
								   QString* errOut = nullptr) = 0;
	virtual void shellFacesToBrep(IPluginDocument* doc, const PluginSketchShellParams& params,
								  PluginGeometryFinishedFn onFinished) = 0;

	/// 1.39.0+：拔模预览 / 提交
	virtual bool previewDraftFaces(IPluginDocument* doc, const PluginSketchDraftParams& params,
								   QString* errOut = nullptr) = 0;
	virtual void draftFacesToBrep(IPluginDocument* doc, const PluginSketchDraftParams& params,
								PluginGeometryFinishedFn onFinished) = 0;

	/// 1.33.0+：B-rep → 第一角法三视图 HLR 折线（异步）
	virtual void projectBrepHlrToDrawing(IPluginDocument* doc, const std::string& backendIdUtf8,
										 PluginDrawingHlrFinishedFn onFinished) = 0;

	/// 1.34.0+：三视图 + 可选轴测/剖视（第三角法可选）
	virtual void projectBrepToEngineeringDrawing(IPluginDocument* doc, const std::string& backendIdUtf8,
												 const PluginDrawingProjectParams& params,
												 PluginDrawingHlrFinishedFn onFinished) = 0;

	/// 1.37.0+：世界原点/三基准面 overlay 显隐（默认全显；退出建模可全关）
	virtual void setOriginReferenceVisibility(IPluginDocument* doc,
											  const PluginOriginReferenceVisibility& visibility) = 0;

	/// 1.47.0+：圆周阵列预览 / 提交（虚表尾部追加，勿插入中间）
	virtual bool previewCircularPattern(IPluginDocument* doc, const PluginSketchCircularPatternParams& params,
										QString* errOut = nullptr) = 0;
	virtual void circularPatternBodyToBrep(IPluginDocument* doc, const PluginSketchCircularPatternParams& params,
										   PluginGeometryFinishedFn onFinished) = 0;
};

#endif // CLOUDSIMPLUGINSDK_IPLUGINGEOMETRYHOST_H
