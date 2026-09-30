/// @file ObjectTransformOperation.cpp
/// @brief Plane-drag world delta multiplier (tune 0.85–1.25 for scene unit / feel).

#include "ObjectTransformOperation.h"

#include "ObjectGizmoFrame.h"
#include "OsgScene.h"
#include "ViewportInteraction/IViewportInteractionHost.h"

#include <QEvent>
#include <QMouseEvent>
#include <algorithm>
#include <cmath>

#include <osg/MatrixTransform>
#include <osg/Quat>
#include <osg/Vec3>

namespace
{
constexpr double kGizmoTranslatePlaneGain = 1.08;
constexpr double kGizmoRotateArcGain = 1.18;

int dragAxisToIndex(OsgScene::DragAxis axis)
{
	switch (axis)
	{
	case OsgScene::DragAxis::X:
		return 0;
	case OsgScene::DragAxis::Y:
		return 1;
	case OsgScene::DragAxis::Z:
		return 2;
	default:
		return 2;
	}
}

double clampGizmoTranslateDsWorld(double dsWorld, const IViewportInteractionHost* host)
{
	const double cap = host->objectGizmoMaxTranslateStepWorld();
	if (dsWorld > cap)
	{
		return cap;
	}
	if (dsWorld < -cap)
	{
		return -cap;
	}
	return dsWorld;
}

} // namespace

ObjectTransformOperation::ObjectTransformOperation(IViewportInteractionHost* host) : SelectionOperation(host) {}

void ObjectTransformOperation::beginGizmoDragSession()
{
	m_gizmoSessionModified = false;
}

void ObjectTransformOperation::markGizmoSessionModified()
{
	m_gizmoSessionModified = true;
}

bool ObjectTransformOperation::handleEvent(QObject* watched, QEvent* event)
{
	IViewportInteractionHost* h = host();
	if (!h || watched != h->viewportGlWidget() || !h->objectSelectionMode())
	{
		return false;
	}

	const bool hasActiveObject = h->hasActiveObjectOuterPat();
	ViewportObjectGizmoDragState gizmo = h->objectGizmoDragState();
	ViewportInteractionPointerState pointer = h->interactionPointerState();

	if (event->type() == QEvent::MouseButtonPress)
	{
		QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
		if (mouseEvent->button() == Qt::LeftButton)
		{
			if (hasActiveObject)
			{
				gizmo.dragAxis = h->pickObjectGizmoAxisAtScreenPos(mouseEvent->pos(), false);
				if (gizmo.dragAxis != OsgScene::DragAxis::None)
				{
					beginGizmoDragSession();
					gizmo.dragging = true;
					gizmo.rotating = false;
					pointer.lastMousePos = mouseEvent->pos();
					h->resetObjectGizmoDragSession();
					(void)h->beginGizmoScreenDrag(gizmo.dragAxis);
					h->updateObjectGizmoCompassHighlight(gizmo.dragAxis, false);
					h->emitActiveAxisChanged(h->gizmoAxisToString(gizmo.dragAxis));
					h->requestRedraw();
					return true;
				}
			}
			if (h->pickAndActivateBackendAtScreenPos(mouseEvent->pos()))
			{
				return true;
			}
			return false;
		}
		if (mouseEvent->button() == Qt::RightButton)
		{
			if (!hasActiveObject)
			{
				return false;
			}
			gizmo.dragAxis = h->pickObjectGizmoAxisAtScreenPos(mouseEvent->pos(), true);
			if (gizmo.dragAxis == OsgScene::DragAxis::None)
			{
				gizmo.dragAxis = gizmo.hoverAxis;
			}
			if (gizmo.dragAxis == OsgScene::DragAxis::None)
			{
				gizmo.dragAxis = OsgScene::DragAxis::Z;
			}
			beginGizmoDragSession();
			gizmo.rotating = true;
			gizmo.dragging = false;
			pointer.lastMousePos = mouseEvent->pos();
			h->resetObjectGizmoDragSession();
			(void)h->cacheObjectGizmoRotatePivot();
			(void)h->beginGizmoScreenRotate(gizmo.dragAxis, static_cast<double>(mouseEvent->pos().x()),
											static_cast<double>(mouseEvent->pos().y()));
			h->updateObjectGizmoCompassHighlight(gizmo.dragAxis, true);
			h->emitActiveAxisChanged(h->gizmoAxisToString(gizmo.dragAxis));
			h->requestRedraw();
			return true;
		}
		return false;
	}

	if (event->type() == QEvent::MouseMove && (gizmo.dragging || gizmo.rotating))
	{
		if (!hasActiveObject)
		{
			return false;
		}
		QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
		const QPoint pos = mouseEvent->pos();
		if (gizmo.dragging)
		{
			double dsWorld = h->gizmoScreenDragDs(static_cast<double>(pos.x()), static_cast<double>(pos.y()),
												  static_cast<double>(pointer.lastMousePos.x()),
												  static_cast<double>(pointer.lastMousePos.y()));
			dsWorld *= kGizmoTranslatePlaneGain;
			dsWorld = clampGizmoTranslateDsWorld(dsWorld, h);
			pointer.lastMousePos = pos;

			ObjectGizmoFrame f;
			osg::MatrixTransform* const outer = h->activeObjectOuterPat();
			if (outer && h->readActiveObjectGizmoFrame(f) && std::abs(dsWorld) > 1e-10)
			{
				f.translateAlongWorldDirection(outer, gizmo.gizmoScreenDragAxisWorld, dsWorld);
				f.applyToOuter(outer);
				(void)ObjectGizmoFrame::fromOuter(outer, f.modelCenter(), f);
				markGizmoSessionModified();
				h->syncActiveBackendRootFromObjectFrame(f, true);
				h->syncCompassGizmoOrientation();
				const osg::Vec3f pose = f.backendPoseRelativeToCenter();
				h->emitSelectedObjectPoseChanged(pose.x(), pose.y(), pose.z());
				h->requestRedraw();
			}
		}
		else if (gizmo.rotating)
		{
			double deltaRad = h->gizmoScreenRotateDeltaRad(static_cast<double>(pos.x()), static_cast<double>(pos.y()));
			deltaRad *= kGizmoRotateArcGain;
			pointer.lastMousePos = pos;

			if (std::abs(deltaRad) <= 1e-8)
			{
				return true;
			}

			ObjectGizmoFrame f;
			osg::MatrixTransform* const outerRot = h->activeObjectOuterPat();
			if (h->readActiveObjectGizmoFrame(f) && outerRot)
			{
				const osg::Quat R_old = f.attitude();
				const int axisIndex = dragAxisToIndex(gizmo.dragAxis);
				const bool worldFrame = h->transformGizmoFrame() == OsgScene::TransformGizmoFrame::World;
				osg::Vec3d axisForQuat;
				osg::Quat R_new = R_old;
				if (ObjectGizmoFrame::dragAxisDirectionOuterParent(outerRot, worldFrame, R_old, axisIndex, axisForQuat))
				{
					const osg::Quat deltaQuat(static_cast<float>(deltaRad),
											  osg::Vec3(static_cast<float>(axisForQuat.x()),
														static_cast<float>(axisForQuat.y()),
														static_cast<float>(axisForQuat.z())));
					R_new = worldFrame ? (deltaQuat * R_old) : (R_old * deltaQuat);
				}
				f.adjustCenterPlusPoseForRotationDelta(R_old, R_new);
				f.applyToOuter(outerRot);
				markGizmoSessionModified();
				h->syncActiveBackendRootFromObjectFrame(f, true);
				h->syncCompassGizmoOrientation();
				const osg::Vec3f euler = h->selectedRotationEulerDeg();
				h->emitSelectedObjectRotationChanged(euler.x(), euler.y(), euler.z());
				h->requestRedraw();
			}
		}
		return true;
	}

	if (event->type() == QEvent::MouseMove && !gizmo.dragging && !gizmo.rotating)
	{
		if (!hasActiveObject)
		{
			return false;
		}
		QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
		if (mouseEvent->buttons().testFlag(Qt::LeftButton) || mouseEvent->buttons().testFlag(Qt::MiddleButton) ||
			mouseEvent->buttons().testFlag(Qt::RightButton))
		{
			return false;
		}
		bool hoverRing = false;
		gizmo.hoverAxis = h->pickObjectGizmoAxisAtScreenPos(mouseEvent->pos(), true, &hoverRing);
		if (gizmo.hoverAxis == OsgScene::DragAxis::None)
		{
			gizmo.hoverAxis = h->pickObjectGizmoAxisAtScreenPos(mouseEvent->pos(), false, &hoverRing);
			hoverRing = false;
		}
		h->updateObjectGizmoCompassHighlight(gizmo.hoverAxis, hoverRing);
		const int ax = static_cast<int>(gizmo.hoverAxis);
		if (m_lastEmittedHoverAxis != ax || m_lastEmittedHoverRing != hoverRing)
		{
			m_lastEmittedHoverAxis = ax;
			m_lastEmittedHoverRing = hoverRing;
			h->emitActiveAxisChanged(h->gizmoAxisToString(gizmo.hoverAxis));
		}
		h->requestRedraw();
		return true;
	}

	if (event->type() == QEvent::MouseButtonRelease)
	{
		QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
		const bool hadGizmoDrag = gizmo.dragging || gizmo.rotating;
		if (mouseEvent->button() == Qt::LeftButton || mouseEvent->button() == Qt::RightButton)
		{
			gizmo.dragging = false;
			gizmo.rotating = false;
			gizmo.dragAxis = OsgScene::DragAxis::None;
			h->resetObjectGizmoDragSession();
			h->updateObjectGizmoCompassHighlight(OsgScene::DragAxis::None);
			m_lastEmittedHoverAxis = -1;
			m_lastEmittedHoverRing = false;
			h->emitActiveAxisChanged(QStringLiteral("None"));
		}
		if (hadGizmoDrag && m_gizmoSessionModified)
		{
			h->syncActiveBackendRootFromSelectedTransform();
			h->cacheSelectionGizmoPose();
			h->refreshAnnotationTexts();
			h->logGizmoPivotDiagnostics("gizmo_mouse_release_before_commit");
			h->emitTransformGizmoCommitted();
			h->logGizmoPivotDiagnostics("gizmo_mouse_release_after_commit");
			h->requestRedraw();
		}
		else if (hadGizmoDrag)
		{
			h->requestRedraw();
		}
		return hadGizmoDrag;
	}

	if (event->type() == QEvent::Wheel || event->type() == QEvent::MouseButtonDblClick)
	{
		return false;
	}

	return false;
}
