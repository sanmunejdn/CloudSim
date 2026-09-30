/// @file OsgWidgetViewportInteractionHost.cpp
/// @brief IViewportInteractionHost 在 OsgWidget 上的实现

#include "OsgWidget.h"

#include "ObjectGizmoFrame.h"

#include <algorithm>
#include <cmath>

QObject* OsgWidget::viewportGlWidget() const
{
	return m_glWidget;
}

ViewportInteractionPointerState OsgWidget::interactionPointerState()
{
	return ViewportInteractionPointerState{m_lastMousePos, m_feedbackTimer};
}

bool OsgWidget::meshSectionPlaneEditActive() const
{
	return isMeshSectionPlaneEditActive();
}

std::size_t OsgWidget::pickablePointCount() const
{
	return m_pickablePointsLocal.size();
}

void OsgWidget::showMeshEdgeHighlight(const osg::Vec3f& aWorld, const osg::Vec3f& bWorld)
{
	OsgScene::showMeshEdgeHighlight(aWorld, bWorld);
}

void OsgWidget::showMeshEdgeHighlight(const std::vector<osg::Vec3f>& polylineWorld)
{
	OsgScene::showMeshEdgeHighlight(polylineWorld);
}

void OsgWidget::collectPointIndicesInScreenRadius(int screenX, int screenY, float radiusPx,
												  std::vector<int>& outIndices) const
{
	OsgScene::collectPointIndicesInScreenRadius(static_cast<double>(screenX), static_cast<double>(screenY),
												static_cast<double>(radiusPx), outIndices);
}

ViewportObjectGizmoDragState OsgWidget::objectGizmoDragState()
{
	return ViewportObjectGizmoDragState{m_dragging,
										m_rotating,
										m_dragAxis,
										m_hoverAxis,
										m_gizmoScreenDragAxisWorld,
										m_gizmoTransDragPlaneActive,
										m_gizmoRotatePivotActive,
										m_gizmoRotateScreenActive,
										m_gizmoRotatePivotWorld,
										m_gizmoTransDragPlaneO,
										m_gizmoTransDragPlaneN,
										m_gizmoDragLastHitWorld,
										m_activeModelDiagonal};
}

bool OsgWidget::hasActiveObjectOuterPat() const
{
	return m_activeBackendOuterPat.valid();
}

osg::MatrixTransform* OsgWidget::activeObjectOuterPat()
{
	return m_activeBackendOuterPat.get();
}

OsgWidget::DragAxis OsgWidget::pickObjectGizmoAxisAtScreenPos(const QPoint& mousePos, bool preferRing,
															  bool* outPickedRing)
{
	return pickAxisAtScreenPos(mousePos, preferRing, outPickedRing);
}

bool OsgWidget::beginGizmoScreenDrag(DragAxis axis)
{
	return OsgScene::beginGizmoScreenDrag(axis);
}

bool OsgWidget::beginGizmoScreenRotate(DragAxis axis, double mouseX, double mouseY)
{
	return OsgScene::beginGizmoScreenRotate(axis, mouseX, mouseY);
}

double OsgWidget::gizmoScreenDragDs(double mouseXCur, double mouseYCur, double mouseXLast, double mouseYLast) const
{
	return OsgScene::gizmoScreenDragDs(mouseXCur, mouseYCur, mouseXLast, mouseYLast);
}

double OsgWidget::gizmoScreenRotateDeltaRad(double mouseX, double mouseY)
{
	return OsgScene::gizmoScreenRotateDeltaRad(mouseX, mouseY);
}

bool OsgWidget::readActiveObjectGizmoFrame(ObjectGizmoFrame& out) const
{
	return OsgScene::readActiveObjectGizmoFrame(out);
}

void OsgWidget::updateObjectGizmoCompassHighlight(DragAxis axis, bool highlightRing)
{
	updateCompassHighlight(axis, highlightRing);
}

void OsgWidget::cacheSelectionGizmoPose()
{
	OsgScene::cacheSelectionGizmoPose();
}

QString OsgWidget::gizmoAxisToString(DragAxis axis) const
{
	return axisToString(axis);
}

void OsgWidget::resetObjectGizmoDragSession()
{
	m_gizmoTransDragPlaneActive = false;
	m_gizmoRotatePivotActive = false;
	m_gizmoRotateScreenActive = false;
}

bool OsgWidget::cacheObjectGizmoRotatePivot()
{
	if (!m_activeBackendOuterPat.valid())
	{
		return false;
	}
	ObjectGizmoFrame gf;
	if (!readActiveObjectGizmoFrame(gf))
	{
		return false;
	}
	osg::Vec3f pivotF;
	computeGizmoPivotWorld(pivotF);
	m_gizmoRotatePivotWorld.set(static_cast<double>(pivotF.x()), static_cast<double>(pivotF.y()),
								static_cast<double>(pivotF.z()));
	m_gizmoRotatePivotActive = true;
	return true;
}

double OsgWidget::objectGizmoMaxTranslateStepWorld() const
{
	const double diag = std::max(1.0, static_cast<double>(m_activeModelDiagonal));
	return std::max(5.0, diag * 0.35);
}

osg::Vec3d OsgWidget::gizmoWorldUnitAxis(DragAxis axis) const
{
	using DA = DragAxis;
	if (axis == DA::None || !m_activeBackendOuterPat.valid())
	{
		return osg::Vec3d(0.0, 0.0, 1.0);
	}
	if (transformGizmoFrame() == TransformGizmoFrame::World)
	{
		if (axis == DA::X)
			return osg::Vec3d(1.0, 0.0, 0.0);
		if (axis == DA::Y)
			return osg::Vec3d(0.0, 1.0, 0.0);
		return osg::Vec3d(0.0, 0.0, 1.0);
	}
	ObjectGizmoFrame gf;
	if (!readActiveObjectGizmoFrame(gf))
	{
		return osg::Vec3d(0.0, 0.0, 1.0);
	}
	osg::Vec3f loc(0.0f, 0.0f, 0.0f);
	switch (axis)
	{
	case DA::X:
		loc.set(1.0f, 0.0f, 0.0f);
		break;
	case DA::Y:
		loc.set(0.0f, 1.0f, 0.0f);
		break;
	case DA::Z:
		loc.set(0.0f, 0.0f, 1.0f);
		break;
	default:
		break;
	}
	const osg::Quat q = gf.attitude();
	const osg::Vec3f w = q * loc;
	osg::Vec3d wd(static_cast<double>(w.x()), static_cast<double>(w.y()), static_cast<double>(w.z()));
	const double len = wd.length();
	if (len < 1e-12)
	{
		return osg::Vec3d(0.0, 0.0, 1.0);
	}
	return wd / len;
}

bool OsgWidget::computeCameraScreenRayWorld(double mouseX, double mouseY, osg::Vec3d& outRayOriginWorld,
											osg::Vec3d& outRayDirUnitWorld) const
{
	return OsgScene::computeCameraScreenRayWorld(mouseX, mouseY, outRayOriginWorld, outRayDirUnitWorld);
}

void OsgWidget::computeGizmoPivotWorld(osg::Vec3f& outPivotWorld) const
{
	OsgScene::computeGizmoPivotWorld(outPivotWorld);
}

ViewportTcpTeachDragState OsgWidget::tcpTeachDragState()
{
	return ViewportTcpTeachDragState{m_tcpTeachDragging,
									 m_tcpTeachRotating,
									 m_tcpTeachDragAxis,
									 m_tcpTeachHoverAxis,
									 m_tcpTeachTransDragPlaneActive,
									 m_tcpTeachRotatePivotActive,
									 m_tcpTeachRotatePivotWorld,
									 m_tcpTeachDragLastHitWorld,
									 m_tcpTeachModelDiagonal};
}

osg::Vec3d OsgWidget::tcpTeachWorldUnitAxis(DragAxis axis) const
{
	osg::Vec3d out(0.0, 0.0, 1.0);
	if (tcpTeachCompassUnitAxisWorld(axis, out))
	{
		return out;
	}
	return osg::Vec3d(0.0, 0.0, 1.0);
}

void OsgWidget::resetTcpTeachDragSession()
{
	m_tcpTeachTransDragPlaneActive = false;
	m_tcpTeachRotatePivotActive = false;
}

double OsgWidget::tcpTeachMaxTranslateStep() const
{
	const double diag = std::max(100.0, static_cast<double>(m_tcpTeachModelDiagonal));
	return std::max(2.0, diag * 0.04);
}

ViewportMeshSectionPlaneDragState OsgWidget::meshSectionPlaneDragState()
{
	return ViewportMeshSectionPlaneDragState{m_sectionPlaneDragging,
											 m_sectionPlaneRotating,
											 m_sectionPlanePlaneDragging,
											 m_sectionPlaneDragAxis,
											 m_sectionPlaneHoverAxis,
											 m_sectionPlaneRotatePivotWorld,
											 m_sectionPlaneDragLastHitWorld,
											 m_sectionPlaneModelDiagonal};
}

void OsgWidget::emitPointPickFeedback(const QString& text)
{
	emit pointPickFeedback(text);
}

void OsgWidget::emitPolylinePickFeedback(const QString& text)
{
	emit polylinePickFeedback(text);
}

void OsgWidget::emitActiveAxisChanged(const QString& axisName)
{
	emit activeAxisChanged(axisName);
}

void OsgWidget::emitSelectedObjectPoseChanged(float x, float y, float z)
{
	emit selectedObjectPoseChanged(x, y, z);
}

void OsgWidget::emitSelectedObjectRotationChanged(float rx, float ry, float rz)
{
	emit selectedObjectRotationChanged(rx, ry, rz);
}

void OsgWidget::emitTransformGizmoCommitted()
{
	emit transformGizmoCommitted();
}

void OsgWidget::emitMeshPickFeedback(const QString& text)
{
	emit meshPickFeedback(text);
}

void OsgWidget::emitMeshPickCommitted(const PickResult& pick, int pickKindInt)
{
	emit meshPickCommitted(pick, pickKindInt);
}

void OsgWidget::emitLabelingClickCommitted(const PickResult& pick)
{
	emit labelingClickCommitted(pick);
}

void OsgWidget::emitLabelingBrushStroke(const QVector<int>& indices)
{
	emit labelingBrushStroke(indices);
}

void OsgWidget::emitLabelingBrushFinished()
{
	emit labelingBrushFinished();
}
