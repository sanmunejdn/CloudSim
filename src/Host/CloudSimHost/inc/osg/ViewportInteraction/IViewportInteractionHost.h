#ifndef CLOUDSIMHOST_IVIEWPORTINTERACTIONHOST_H
#define CLOUDSIMHOST_IVIEWPORTINTERACTIONHOST_H

/// @file IViewportInteractionHost.h
/// @brief SelectionOperation 访问视口交互态的窄接口（实现于 OsgWidget）

#include "OsgScene.h"
#include "PickTypes.h"

#include <QElapsedTimer>
#include <QPoint>
#include <QString>
#include <QVector>

#include <osg/MatrixTransform>
#include <osg/Vec3d>
#include <osg/Vec3f>
#include <vector>

class QObject;
class IViewportPickEngine;
class ObjectGizmoFrame;

struct ViewportInteractionPointerState
{
	QPoint& lastMousePos;
	QElapsedTimer& feedbackTimer;
};

struct ViewportObjectGizmoDragState
{
	bool& dragging;
	bool& rotating;
	OsgScene::DragAxis& dragAxis;
	OsgScene::DragAxis& hoverAxis;
	osg::Vec3d& gizmoScreenDragAxisWorld;
	bool& gizmoTransDragPlaneActive;
	bool& gizmoRotatePivotActive;
	bool& gizmoRotateScreenActive;
	osg::Vec3d& gizmoRotatePivotWorld;
	osg::Vec3d& gizmoTransDragPlaneO;
	osg::Vec3d& gizmoTransDragPlaneN;
	osg::Vec3d& gizmoDragLastHitWorld;
	float& activeModelDiagonal;
};

struct ViewportTcpTeachDragState
{
	bool& dragging;
	bool& rotating;
	OsgScene::DragAxis& dragAxis;
	OsgScene::DragAxis& hoverAxis;
	bool& transDragPlaneActive;
	bool& rotatePivotActive;
	osg::Vec3d& rotatePivotWorld;
	osg::Vec3d& dragLastHitWorld;
	float& modelDiagonal;
};

struct ViewportMeshSectionPlaneDragState
{
	bool& dragging;
	bool& rotating;
	bool& planeDragging;
	OsgScene::DragAxis& dragAxis;
	OsgScene::DragAxis& hoverAxis;
	osg::Vec3d& rotatePivotWorld;
	osg::Vec3d& dragLastHitWorld;
	float& modelDiagonal;
};

/// OsgWidget 对 SelectionOperation 族暴露的交互宿主
class IViewportInteractionHost
{
public:
	virtual ~IViewportInteractionHost() = default;

	virtual QObject* viewportGlWidget() const = 0;
	virtual ViewportInteractionPointerState interactionPointerState() = 0;

	virtual bool pointPickMode() const = 0;
	virtual bool polylinePickMode() const = 0;
	virtual bool meshLinePickMode() const = 0;
	virtual bool meshFacePickMode() const = 0;
	virtual bool objectSelectionMode() const = 0;
	virtual bool tcpTeachActive() const = 0;
	virtual bool meshSectionPlaneEditActive() const = 0;
	virtual bool labelingClickPickMode() const = 0;
	virtual bool labelingBrushPickMode() const = 0;
	virtual bool labelingMeshFaceMode() const = 0;
	virtual float labelingBrushRadiusPx() const = 0;
	virtual bool crossObjectMeshPick() const = 0;
	virtual std::string activeBackendId() const = 0;
	virtual bool originPlanePickActive() const = 0;
	virtual int originPlaneHoverIndex() const = 0;

	virtual void requestRedraw() const = 0;
	virtual IViewportPickEngine* pickEngine() = 0;
	virtual PickResult queryPick(const PickQuery& query) = 0;
	virtual std::size_t pickablePointCount() const = 0;

	virtual void updatePointPickMarker(const osg::Vec3f& pointWorld, bool hit) = 0;
	virtual void clearPointPickMarker() = 0;
	virtual void addPointAnnotation(const osg::Vec3f& pointWorld) = 0;

	virtual void updatePolylinePickOverlay(const std::vector<QPoint>& vertices, const QPoint* cursorPos) = 0;
	virtual void commitPolylinePick(const std::vector<QPoint>& vertices) = 0;

	virtual void showMeshFaceHighlight(const std::vector<osg::Vec3f>& vertsWorld) = 0;
	virtual void showMeshEdgeHighlight(const osg::Vec3f& aWorld, const osg::Vec3f& bWorld) = 0;
	virtual void showMeshEdgeHighlight(const std::vector<osg::Vec3f>& polylineWorld) = 0;
	virtual void hideMeshElementHighlight() = 0;
	virtual int resolveSketchSupportOriginIndex(int screenX, int screenY) const = 0;
	virtual void collectPointIndicesInScreenRadius(int screenX, int screenY, float radiusPx,
												   std::vector<int>& outIndices) const = 0;

	virtual ViewportObjectGizmoDragState objectGizmoDragState() = 0;
	virtual bool hasActiveObjectOuterPat() const = 0;
	virtual osg::MatrixTransform* activeObjectOuterPat() = 0;
	virtual OsgScene::TransformGizmoFrame transformGizmoFrame() const = 0;
	virtual OsgScene::DragAxis pickObjectGizmoAxisAtScreenPos(const QPoint& mousePos, bool preferRing,
															  bool* outPickedRing = nullptr) = 0;
	virtual bool pickAndActivateBackendAtScreenPos(const QPoint& mousePos) = 0;
	virtual bool beginGizmoScreenDrag(OsgScene::DragAxis axis) = 0;
	virtual bool beginGizmoScreenRotate(OsgScene::DragAxis axis, double mouseX, double mouseY) = 0;
	virtual double gizmoScreenDragDs(double mouseXCur, double mouseYCur, double mouseXLast, double mouseYLast) const = 0;
	virtual double gizmoScreenRotateDeltaRad(double mouseX, double mouseY) = 0;
	virtual bool readActiveObjectGizmoFrame(ObjectGizmoFrame& out) const = 0;
	virtual void syncActiveBackendRootFromObjectFrame(const ObjectGizmoFrame& cur, bool dragging) = 0;
	virtual void syncActiveBackendRootFromSelectedTransform() = 0;
	virtual void syncCompassGizmoOrientation() = 0;
	virtual void updateObjectGizmoCompassHighlight(OsgScene::DragAxis axis, bool highlightRing = false) = 0;
	virtual void cacheSelectionGizmoPose() = 0;
	virtual void refreshAnnotationTexts() = 0;
	virtual void logGizmoPivotDiagnostics(const char* reasonTag) const = 0;
	virtual osg::Vec3f selectedPosition() const = 0;
	virtual osg::Vec3f selectedRotationEulerDeg() const = 0;
	virtual QString gizmoAxisToString(OsgScene::DragAxis axis) const = 0;
	virtual void resetObjectGizmoDragSession() = 0;
	virtual bool cacheObjectGizmoRotatePivot() = 0;
	virtual double objectGizmoMaxTranslateStepWorld() const = 0;
	virtual osg::Vec3d gizmoWorldUnitAxis(OsgScene::DragAxis axis) const = 0;
	virtual bool computeCameraScreenRayWorld(double mouseX, double mouseY, osg::Vec3d& outRayOriginWorld,
											  osg::Vec3d& outRayDirUnitWorld) const = 0;
	virtual void computeGizmoPivotWorld(osg::Vec3f& outPivotWorld) const = 0;

	virtual ViewportTcpTeachDragState tcpTeachDragState() = 0;
	virtual int pickTcpTeachAxisAtScreenPos(const QPoint& mousePos, bool preferRing, bool* outPickedRing = nullptr) const = 0;
	virtual bool beginTcpTeachScreenDrag() = 0;
	virtual double tcpTeachScreenDragDsMm(const QPoint& curPos, const QPoint& lastPos) const = 0;
	virtual void updateTcpTeachCompassHighlight(OsgScene::DragAxis axis, bool highlightRing = false) = 0;
	virtual osg::Vec3d tcpTeachWorldUnitAxis(OsgScene::DragAxis axis) const = 0;
	virtual void computeTcpTeachPivotWorld(osg::Vec3f& outPivotWorld) const = 0;
	virtual void applyTcpTeachTranslationBody(int axisIndex, double dsWorld) = 0;
	virtual void applyTcpTeachRotationBody(int axisIndex, double deltaRad) = 0;
	virtual void resetTcpTeachDragSession() = 0;
	virtual double tcpTeachMaxTranslateStep() const = 0;
	virtual void emitTcpDragTeachPoseChanged() = 0;

	virtual ViewportMeshSectionPlaneDragState meshSectionPlaneDragState() = 0;
	virtual int pickMeshSectionPlaneAxisAtScreenPos(const QPoint& mousePos, bool preferRing,
													bool* outPickedRing = nullptr) const = 0;
	virtual bool beginMeshSectionPlaneScreenDrag() = 0;
	virtual bool pickMeshSectionPlaneDragPoint(const QPoint& mousePos, osg::Vec3d& outHitWorld) const = 0;
	virtual double meshSectionPlaneScreenDragDsMm(const QPoint& curPos, const QPoint& lastPos) const = 0;
	virtual void updateMeshSectionPlaneCompassHighlight(OsgScene::DragAxis axis, bool highlightRing = false) = 0;
	virtual void computeMeshSectionPlanePivotWorld(osg::Vec3d& outPivotWorld) const = 0;
	virtual bool meshSectionPlaneCompassUnitAxisWorld(OsgScene::DragAxis axis, osg::Vec3d& outAxisWorld) const = 0;
	virtual void applyMeshSectionPlaneTranslationWorld(const osg::Vec3d& hitWorld, const osg::Vec3d& lastHitWorld) = 0;
	virtual void applyMeshSectionPlaneTranslationAxis(int axisIndex, double dsWorld) = 0;
	virtual void applyMeshSectionPlaneRotationAxis(int axisIndex, double deltaRad) = 0;
	virtual void notifyMeshSectionPlaneChanged() = 0;

	virtual void emitPointPickFeedback(const QString& text) = 0;
	virtual void emitPolylinePickFeedback(const QString& text) = 0;
	virtual void emitActiveAxisChanged(const QString& axisName) = 0;
	virtual void emitSelectedObjectPoseChanged(float x, float y, float z) = 0;
	virtual void emitSelectedObjectRotationChanged(float rx, float ry, float rz) = 0;
	virtual void emitTransformGizmoCommitted() = 0;
	virtual void emitMeshPickFeedback(const QString& text) = 0;
	virtual void emitMeshPickCommitted(const PickResult& pick, int pickKindInt) = 0;
	virtual void emitLabelingClickCommitted(const PickResult& pick) = 0;
	virtual void emitLabelingBrushStroke(const QVector<int>& indices) = 0;
	virtual void emitLabelingBrushFinished() = 0;
};

#endif // CLOUDSIMHOST_IVIEWPORTINTERACTIONHOST_H
