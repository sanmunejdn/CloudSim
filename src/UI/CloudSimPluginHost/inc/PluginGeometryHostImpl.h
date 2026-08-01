#ifndef CLOUDSIMPLUGINHOST_PLUGINGEOMETRYHOSTIMPL_H
#define CLOUDSIMPLUGINHOST_PLUGINGEOMETRYHOSTIMPL_H

/// @file PluginGeometryHostImpl.h
/// @brief IPluginGeometryHost 实现（编入 CloudSimHost）

#include "IPluginGeometryHost.h"

#include <QMetaObject>
#include <memory>

class PluginHostContext;

class PluginGeometryHostImpl : public IPluginGeometryHost
{
public:
	explicit PluginGeometryHostImpl(PluginHostContext* hostContext);

	void discretizeStepToMesh(IPluginDocument* doc, const std::string& stepPathUtf8,
							  const PluginMeshDiscretizeParams& params, const PluginMeshCreateOptions& options,
							  PluginGeometryFinishedFn onFinished) override;

	void discretizeBackendToMesh(IPluginDocument* doc, const std::string& stepPathUtf8,
								 const PluginMeshDiscretizeParams& params, const PluginMeshCreateOptions& options,
								 PluginGeometryFinishedFn onFinished) override;

	void discretizeBackendFaceToMesh(IPluginDocument* doc, const PluginGeometryStepRef& faceRef,
									 const PluginMeshDiscretizeParams& params, const PluginMeshCreateOptions& options,
									 PluginGeometryFinishedFn onFinished) override;

	void discretizeWireToTubeMesh(IPluginDocument* doc, const std::vector<float>& polylineXyz,
								  const PluginMeshDiscretizeParams& params, const PluginMeshCreateOptions& options,
								  PluginGeometryFinishedFn onFinished) override;

	void discretizeWireToRibbonMesh(IPluginDocument* doc, const std::vector<float>& polylineXyz,
									const PluginMeshDiscretizeParams& params, const PluginMeshCreateOptions& options,
									PluginGeometryFinishedFn onFinished) override;

	void discretizePolylineToMesh(IPluginDocument* doc, const std::vector<float>& polylineXyz,
								  const PluginMeshDiscretizeParams& params, const PluginMeshCreateOptions& options,
								  PluginGeometryFinishedFn onFinished) override;

	void discretizeBackendEdgesToPolylines(IPluginDocument* doc, const std::string& stepPathUtf8,
										   const PluginMeshDiscretizeParams& params,
										   PluginGeometryFinishedFn onFinished) override;

	void discretizeBackendFaceEdgesToPolylines(IPluginDocument* doc, const PluginGeometryStepRef& faceRef,
											   const PluginMeshDiscretizeParams& params,
											   PluginGeometryFinishedFn onFinished) override;

	void intersectEdges(IPluginDocument* doc, const PluginGeometryStepRef& edge1, const PluginGeometryStepRef& edge2,
						const PluginGeometryIntersectionParams& params, PluginGeometryFinishedFn onFinished) override;

	void intersectEdgeFace(IPluginDocument* doc, const PluginGeometryStepRef& edgeRef,
						   const PluginGeometryStepRef& faceRef, const PluginGeometryIntersectionParams& params,
						   PluginGeometryFinishedFn onFinished) override;

	void intersectFaces(IPluginDocument* doc, const PluginGeometryStepRef& face1, const PluginGeometryStepRef& face2,
						const PluginGeometryIntersectionParams& params, PluginGeometryFinishedFn onFinished) override;

	void intersectBackends(IPluginDocument* doc, const std::string& targetStepPathUtf8,
						   const std::string& toolStepPathUtf8, const PluginGeometryIntersectionParams& params,
						   PluginGeometryFinishedFn onFinished) override;

	void brepBooleanToMesh(IPluginDocument* doc, const std::string& targetStepPathUtf8,
						   const std::string& toolStepPathUtf8, const PluginGeometryBrepBooleanParams& params,
						   PluginGeometryFinishedFn onFinished) override;

	void fuseWiresToPolyline(IPluginDocument* doc, const std::string& stepPathUtf8, const std::vector<int>& edgeIndices,
							 const PluginGeometryIntersectionParams& params,
							 PluginGeometryFinishedFn onFinished) override;

	void sewFacesToMesh(IPluginDocument* doc, const std::string& stepPathUtf8, const std::vector<int>& faceIndices,
						const PluginMeshDiscretizeParams& meshParams, const PluginMeshCreateOptions& options,
						PluginGeometryFinishedFn onFinished) override;

	bool listComputableBackends(IPluginDocument* doc, std::vector<PluginGeometryBackendEntry>& outBackends,
								QString* outError = nullptr) override;

	void pickStepElementFromViewport(IPluginDocument* doc, const PluginGeometryElementPickRequest& request,
									 PluginGeometryElementPickedFn onFinished) override;

	bool queryFaceSketchPlane(IPluginDocument* doc, const PluginGeometryStepRef& faceRef, PluginSketchPlane& outPlane,
							  QString* outError = nullptr) override;
	void setSketchOverlay(IPluginDocument* doc, const std::vector<PluginSketchOverlaySegment>& segments) override;
	void clearSketchOverlay(IPluginDocument* doc) override;
	bool mapScreenToSketchPlane(IPluginDocument* doc, int screenX, int screenY, const PluginSketchPlane& plane,
								PluginPoint3d& outWorldMm, QString* outError = nullptr) override;
	void extrudeSketchProfileToBrep(IPluginDocument* doc, const std::vector<float>& closedPolylineXyzMm,
									const PluginSketchPlane& plane, const PluginSketchExtrudeParams& params,
									PluginGeometryFinishedFn onFinished) override;
	bool queryParametricBodyHistoryJson(IPluginDocument* doc, const std::string& backendIdUtf8,
										QByteArray& outJsonUtf8, QString* outError = nullptr) override;
	void setParametricBodyHistoryJson(IPluginDocument* doc, const std::string& backendIdUtf8,
									  const QByteArray& historyJsonUtf8, PluginGeometryFinishedFn onFinished) override;
	bool beginSketchInput(IPluginDocument* doc, const PluginSketchPlane& plane, PluginSketchInputFn onInput,
						  QString* outError = nullptr) override;
	void endSketchInput(IPluginDocument* doc) override;
	void pickOriginSketchPlane(IPluginDocument* doc, PluginOriginPlanePickedFn onFinished) override;
	void cancelOriginSketchPlanePick(IPluginDocument* doc) override;
	void pickSketchSupportPlane(IPluginDocument* doc, const std::vector<PluginSupportPlaneCandidate>& extras,
								PluginSupportPlanePickedFn onFinished) override;
	void previewSketchExtrude(IPluginDocument* doc, const std::vector<float>& closedPolylineXyzMm,
							  const PluginSketchPlane& plane, const PluginSketchExtrudeParams& params) override;
	void clearSketchExtrudePreview(IPluginDocument* doc) override;
	bool listParametricBodyIds(IPluginDocument* doc, std::vector<std::string>& outIds, QString* outError = nullptr) override;
	void pickParametricFeatureForEdit(IPluginDocument* doc, PluginParametricFeaturePickedFn onFinished) override;
	bool previewSketchSweep(IPluginDocument* doc, const std::vector<float>& profilePolylineXyzMm,
							const std::vector<float>& pathPolylineXyzMm, const PluginSketchSweepParams& params,
							QString* errOut = nullptr) override;
	void sweepSketchProfileToBrep(IPluginDocument* doc, const std::vector<float>& profilePolylineXyzMm,
								  const std::vector<float>& pathPolylineXyzMm, const PluginSketchSweepParams& params,
								  PluginGeometryFinishedFn onFinished) override;

	bool previewFilletEdges(IPluginDocument* doc, const PluginSketchFilletParams& params,
							QString* errOut = nullptr) override;
	void filletEdgesToBrep(IPluginDocument* doc, const PluginSketchFilletParams& params,
						   PluginGeometryFinishedFn onFinished) override;
	bool previewChamferEdges(IPluginDocument* doc, const PluginSketchChamferParams& params,
							 QString* errOut = nullptr) override;
	void chamferEdgesToBrep(IPluginDocument* doc, const PluginSketchChamferParams& params,
							PluginGeometryFinishedFn onFinished) override;
	bool previewSketchRevolve(IPluginDocument* doc, const std::vector<float>& profilePolylineXyzMm,
							  const PluginSketchRevolveParams& params, QString* errOut = nullptr) override;
	void revolveSketchProfileToBrep(IPluginDocument* doc, const std::vector<float>& profilePolylineXyzMm,
								  const PluginSketchRevolveParams& params,
								  PluginGeometryFinishedFn onFinished) override;
	bool previewLinearPattern(IPluginDocument* doc, const PluginSketchLinearPatternParams& params,
							  QString* errOut = nullptr) override;
	void linearPatternBodyToBrep(IPluginDocument* doc, const PluginSketchLinearPatternParams& params,
								 PluginGeometryFinishedFn onFinished) override;
	bool previewMirror3d(IPluginDocument* doc, const PluginSketchMirror3dParams& params,
						 QString* errOut = nullptr) override;
	void mirror3dBodyToBrep(IPluginDocument* doc, const PluginSketchMirror3dParams& params,
							PluginGeometryFinishedFn onFinished) override;
	bool previewSketchLoft(IPluginDocument* doc, const std::vector<float>& profilePolylineAXyzMm,
						   const std::vector<float>& profilePolylineBXyzMm, const PluginSketchLoftParams& params,
						   QString* errOut = nullptr) override;
	void loftSketchProfilesToBrep(IPluginDocument* doc, const std::vector<float>& profilePolylineAXyzMm,
								  const std::vector<float>& profilePolylineBXyzMm, const PluginSketchLoftParams& params,
								  PluginGeometryFinishedFn onFinished) override;
	bool previewShellFaces(IPluginDocument* doc, const PluginSketchShellParams& params,
						   QString* errOut = nullptr) override;
	void shellFacesToBrep(IPluginDocument* doc, const PluginSketchShellParams& params,
						  PluginGeometryFinishedFn onFinished) override;
	bool previewDraftFaces(IPluginDocument* doc, const PluginSketchDraftParams& params,
						   QString* errOut = nullptr) override;
	void draftFacesToBrep(IPluginDocument* doc, const PluginSketchDraftParams& params,
						  PluginGeometryFinishedFn onFinished) override;

	void projectBrepHlrToDrawing(IPluginDocument* doc, const std::string& backendIdUtf8,
								 PluginDrawingHlrFinishedFn onFinished) override;

	void projectBrepToEngineeringDrawing(IPluginDocument* doc, const std::string& backendIdUtf8,
										 const PluginDrawingProjectParams& params,
										 PluginDrawingHlrFinishedFn onFinished) override;

	void setOriginReferenceVisibility(IPluginDocument* doc,
									  const PluginOriginReferenceVisibility& visibility) override;

	bool previewCircularPattern(IPluginDocument* doc, const PluginSketchCircularPatternParams& params,
								QString* errOut = nullptr) override;
	void circularPatternBodyToBrep(IPluginDocument* doc, const PluginSketchCircularPatternParams& params,
								   PluginGeometryFinishedFn onFinished) override;

private:
	void clearSketchSupportPlanePick();

	PluginHostContext* m_host = nullptr;
	PluginSketchPlane m_sketchInputPlane{};
	IPluginDocument* m_sketchInputDoc = nullptr;
	/// 基面+模型面联合拾取会话
	std::shared_ptr<bool> m_supportPlanePickDone;
	QMetaObject::Connection m_supportPlaneFaceConn;
};

#endif // CLOUDSIMPLUGINHOST_PLUGINGEOMETRYHOSTIMPL_H
