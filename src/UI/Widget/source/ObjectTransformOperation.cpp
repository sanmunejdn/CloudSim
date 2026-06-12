#include "ObjectTransformOperation.h"

#include <QEvent>
#include <QMouseEvent>

#include <algorithm>
#include <cmath>

#include <osg/Matrixd>
#include <osg/Quat>
#include <osg/Vec3>
#include <osg/Vec3d>

#include "OsgWidget.h"
#include "ObjectGizmoFrame.h"

namespace
{
/// Plane-drag world delta multiplier (tune 0.85–1.25 for scene unit / feel).
constexpr double kGizmoTranslatePlaneGain = 1.08;
/// Fallback screen heuristic when ray misses the drag plane.
constexpr double kGizmoTranslateFallbackGain = 0.55;
/// Right-drag rotation: radians per atan2 step feel.
constexpr double kGizmoRotateArcGain = 1.18;

osg::Vec3f gizmoLocalAxis(OsgWidget::DragAxis axis)
{
	switch (axis)
	{
	case OsgWidget::DragAxis::X:
		return osg::Vec3f(1.0f, 0.0f, 0.0f);
	case OsgWidget::DragAxis::Y:
		return osg::Vec3f(0.0f, 1.0f, 0.0f);
	case OsgWidget::DragAxis::Z:
		return osg::Vec3f(0.0f, 0.0f, 1.0f);
	default:
		return osg::Vec3f(0.0f, 0.0f, 0.0f);
	}
}

int dragAxisToIndex(OsgWidget::DragAxis axis)
{
	switch (axis)
	{
	case OsgWidget::DragAxis::X:
		return 0;
	case OsgWidget::DragAxis::Y:
		return 1;
	case OsgWidget::DragAxis::Z:
		return 2;
	default:
		return 2;
	}
}

osg::Vec3d worldUnitAxisForGizmo(OsgWidget* owner, OsgWidget::DragAxis axis)
{
	using DA = OsgWidget::DragAxis;
	if (axis == DA::None || !owner->m_activeBackendOuterPat.valid())
	{
		return osg::Vec3d(0.0, 0.0, 1.0);
	}
	if (owner->transformGizmoFrame() == OsgWidget::TransformGizmoFrame::World)
	{
		if (axis == DA::X) return osg::Vec3d(1.0, 0.0, 0.0);
		if (axis == DA::Y) return osg::Vec3d(0.0, 1.0, 0.0);
		return osg::Vec3d(0.0, 0.0, 1.0);
	}
	ObjectGizmoFrame gf;
	if (!owner->readActiveObjectGizmoFrame(gf))
	{
		return osg::Vec3d(0.0, 0.0, 1.0);
	}
	const osg::Vec3f loc = gizmoLocalAxis(axis);
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

bool rayClosestPointOnLine(
	const osg::Vec3d& rayOrigin,
	const osg::Vec3d& rayDirUnit,
	const osg::Vec3d& lineOrigin,
	const osg::Vec3d& lineDirUnit,
	osg::Vec3d& outPointOnLine,
	double maxAbsLineParam)
{
	const osg::Vec3d w0 = rayOrigin - lineOrigin;
	const double a = rayDirUnit * rayDirUnit;
	const double b = rayDirUnit * lineDirUnit;
	const double c = lineDirUnit * lineDirUnit;
	const double d = rayDirUnit * w0;
	const double e = lineDirUnit * w0;
	const double denom = a * c - b * b;
	if (std::abs(denom) < 1e-8)
	{
		return false;
	}
	const double tLine = (b * e - c * d) / denom;
	if (maxAbsLineParam > 0.0 && std::abs(tLine) > maxAbsLineParam)
	{
		return false;
	}
	outPointOnLine = lineOrigin + lineDirUnit * tLine;
	return true;
}

double maxGizmoTranslateStepWorld(const OsgWidget* owner)
{
	const double diag = std::max(1.0, static_cast<double>(owner->m_activeModelDiagonal));
	return std::max(5.0, diag * 0.35);
}

double clampGizmoTranslateDsWorld(double dsWorld, const OsgWidget* owner)
{
	const double cap = maxGizmoTranslateStepWorld(owner);
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

bool currentGizmoPivotWorldD(const OsgWidget* owner, osg::Vec3d& outPivot)
{
	osg::Vec3f pivotF;
	owner->computeGizmoPivotWorld(pivotF);
	outPivot.set(static_cast<double>(pivotF.x()), static_cast<double>(pivotF.y()), static_cast<double>(pivotF.z()));
	return true;
}

bool rayPlaneIntersect(
	const osg::Vec3d& rayOrigin,
	const osg::Vec3d& rayDirUnit,
	const osg::Vec3d& planePoint,
	const osg::Vec3d& planeNormalUnit,
	osg::Vec3d& outHit)
{
	const double denom = rayDirUnit * planeNormalUnit;
	if (std::abs(denom) < 1e-10)
	{
		return false;
	}
	const double t = ((planePoint - rayOrigin) * planeNormalUnit) / denom;
	if (t < -1e-3)
	{
		return false;
	}
	outHit = rayOrigin + rayDirUnit * t;
	return true;
}

void resetGizmoDragSession(OsgWidget* o)
{
	o->m_gizmoTransDragPlaneActive = false;
	o->m_gizmoRotatePivotActive = false;
	o->m_gizmoRotateScreenActive = false;
}

bool cacheRotatePivotInParentSpace(OsgWidget* o)
{
	if (!o->m_activeBackendOuterPat.valid())
	{
		return false;
	}
	ObjectGizmoFrame gf;
	if (!o->readActiveObjectGizmoFrame(gf))
	{
		return false;
	}
	osg::Vec3f pivotF;
	o->computeGizmoPivotWorld(pivotF);
	o->m_gizmoRotatePivotWorld.set(static_cast<double>(pivotF.x()), static_cast<double>(pivotF.y()),
		static_cast<double>(pivotF.z()));
	o->m_gizmoRotatePivotActive = true;
	return true;
}

bool tryBeginTranslatePlane(OsgWidget* o, const QPoint& pos)
{
	resetGizmoDragSession(o);
	osg::Vec3f pivotF;
	o->computeGizmoPivotWorld(pivotF);
	const osg::Vec3d pivot(static_cast<double>(pivotF.x()), static_cast<double>(pivotF.y()), static_cast<double>(pivotF.z()));
	const osg::Vec3d axisW = worldUnitAxisForGizmo(o, o->m_dragAxis);
	osg::Vec3d eye, dir;
	if (!o->computeCameraScreenRayWorld(static_cast<double>(pos.x()), static_cast<double>(pos.y()), eye, dir))
	{
		return false;
	}
	osg::Vec3d viewDir = eye - pivot;
	if (viewDir.length2() < 1e-18)
	{
		viewDir.set(0.0, 0.0, 1.0);
	}
	else
	{
		viewDir.normalize();
	}
	osg::Vec3d n = axisW ^ viewDir;
	if (n.length2() < 1e-16)
	{
		n = axisW ^ osg::Vec3d(0.0, 1.0, 0.0);
		if (n.length2() < 1e-16)
		{
			n = axisW ^ osg::Vec3d(1.0, 0.0, 0.0);
		}
	}
	n.normalize();
	osg::Vec3d hit;
	if (!rayPlaneIntersect(eye, dir, pivot, n, hit))
	{
		return false;
	}
	o->m_gizmoTransDragPlaneO = pivot;
	o->m_gizmoTransDragPlaneN = n;
	o->m_gizmoDragLastHitWorld = hit;
	o->m_gizmoTransDragPlaneActive = true;
	return true;
}

bool seedTranslateDragOnAxisLine(OsgWidget* o, const QPoint& pos)
{
	osg::Vec3f pivotF;
	o->computeGizmoPivotWorld(pivotF);
	const osg::Vec3d pivot(static_cast<double>(pivotF.x()), static_cast<double>(pivotF.y()), static_cast<double>(pivotF.z()));
	osg::Vec3d axisW = worldUnitAxisForGizmo(o, o->m_dragAxis);
	const double axisLen = axisW.length();
	if (axisLen < 1e-12)
	{
		return false;
	}
	axisW /= axisLen;
	osg::Vec3d eye;
	osg::Vec3d dir;
	if (!o->computeCameraScreenRayWorld(static_cast<double>(pos.x()), static_cast<double>(pos.y()), eye, dir))
	{
		o->m_gizmoDragLastHitWorld = pivot;
		o->m_gizmoTransDragPlaneActive = true;
		return true;
	}
	osg::Vec3d hit;
	const double maxLineT = maxGizmoTranslateStepWorld(o) * 4.0;
	if (!rayClosestPointOnLine(eye, dir, pivot, axisW, hit, maxLineT))
	{
		hit = pivot;
	}
	o->m_gizmoDragLastHitWorld = hit;
	o->m_gizmoTransDragPlaneActive = true;
	return true;
}

void tryBeginRotateHit(OsgWidget* o, const QPoint& pos)
{
	o->m_gizmoTransDragPlaneActive = false;
	(void)cacheRotatePivotInParentSpace(o);
	osg::Vec3f pivotF;
	o->computeGizmoPivotWorld(pivotF);
	const osg::Vec3d pivot(static_cast<double>(pivotF.x()), static_cast<double>(pivotF.y()), static_cast<double>(pivotF.z()));
	const osg::Vec3d axisW = worldUnitAxisForGizmo(o, o->m_dragAxis);
	osg::Vec3d eye, dir;
	if (!o->computeCameraScreenRayWorld(static_cast<double>(pos.x()), static_cast<double>(pos.y()), eye, dir))
	{
		o->m_gizmoDragLastHitWorld = pivot;
		return;
	}
	osg::Vec3d hit;
	if (rayPlaneIntersect(eye, dir, pivot, axisW, hit))
	{
		o->m_gizmoDragLastHitWorld = hit;
	}
	else
	{
		o->m_gizmoDragLastHitWorld = pivot;
	}
}

} // namespace

ObjectTransformOperation::ObjectTransformOperation(OsgWidget* owner)
	: SelectionOperation(owner)
{
}

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
	if (!m_owner || watched != m_owner->m_glWidget || !m_owner->m_objectSelectionMode)
	{
		return false;
	}

	const bool hasActiveObject = m_owner->m_activeBackendOuterPat.valid();

	if (event->type() == QEvent::MouseButtonPress)
	{
		QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
		if (mouseEvent->button() == Qt::LeftButton)
		{
			if (hasActiveObject)
			{
				m_owner->m_dragAxis = m_owner->pickAxisAtScreenPos(mouseEvent->pos(), false);
				if (m_owner->m_dragAxis != OsgWidget::DragAxis::None)
				{
					beginGizmoDragSession();
					m_owner->m_dragging = true;
					m_owner->m_rotating = false;
					m_owner->m_lastMousePos = mouseEvent->pos();
					resetGizmoDragSession(m_owner);
					(void)m_owner->beginGizmoScreenDrag(m_owner->m_dragAxis);
					m_owner->updateCompassHighlight(m_owner->m_dragAxis, false);
					emit m_owner->activeAxisChanged(m_owner->axisToString(m_owner->m_dragAxis));
					m_owner->requestRedraw();
					return true;
				}
			}
			if (m_owner->pickAndActivateBackendAtScreenPos(mouseEvent->pos()))
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
			m_owner->m_dragAxis = m_owner->pickAxisAtScreenPos(mouseEvent->pos(), true);
			if (m_owner->m_dragAxis == OsgWidget::DragAxis::None) m_owner->m_dragAxis = m_owner->m_hoverAxis;
			if (m_owner->m_dragAxis == OsgWidget::DragAxis::None) m_owner->m_dragAxis = OsgWidget::DragAxis::Z;
			beginGizmoDragSession();
			m_owner->m_rotating = true;
			m_owner->m_dragging = false;
			m_owner->m_lastMousePos = mouseEvent->pos();
			resetGizmoDragSession(m_owner);
			(void)cacheRotatePivotInParentSpace(m_owner);
			(void)m_owner->beginGizmoScreenRotate(
				m_owner->m_dragAxis,
				static_cast<double>(mouseEvent->pos().x()),
				static_cast<double>(mouseEvent->pos().y()));
			m_owner->updateCompassHighlight(m_owner->m_dragAxis, true);
			emit m_owner->activeAxisChanged(m_owner->axisToString(m_owner->m_dragAxis));
			m_owner->requestRedraw();
			return true;
		}
		return false;
	}

	if (event->type() == QEvent::MouseMove && (m_owner->m_dragging || m_owner->m_rotating))
	{
		if (!hasActiveObject)
		{
			return false;
		}
		QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
		const QPoint pos = mouseEvent->pos();
		if (m_owner->m_dragging)
		{
			double dsWorld = m_owner->gizmoScreenDragDs(
				static_cast<double>(pos.x()),
				static_cast<double>(pos.y()),
				static_cast<double>(m_owner->m_lastMousePos.x()),
				static_cast<double>(m_owner->m_lastMousePos.y()));
			dsWorld *= kGizmoTranslatePlaneGain;
			dsWorld = clampGizmoTranslateDsWorld(dsWorld, m_owner);
			m_owner->m_lastMousePos = pos;

			ObjectGizmoFrame f;
			osg::MatrixTransform* const outer = m_owner->m_activeBackendOuterPat.get();
			if (outer && m_owner->readActiveObjectGizmoFrame(f) && std::abs(dsWorld) > 1e-10)
			{
				f.translateAlongWorldDirection(outer, m_owner->m_gizmoScreenDragAxisWorld, dsWorld);
				f.applyToOuter(outer);
				(void)ObjectGizmoFrame::fromOuter(outer, f.modelCenter(), f);
				markGizmoSessionModified();
				m_owner->syncActiveBackendRootFromObjectFrame(f, true);
				m_owner->syncCompassGizmoOrientation();
				const osg::Vec3f pose = f.backendPoseRelativeToCenter();
				emit m_owner->selectedObjectPoseChanged(pose.x(), pose.y(), pose.z());
				m_owner->requestRedraw();
			}
		}
		else if (m_owner->m_rotating)
		{
			double deltaRad = m_owner->gizmoScreenRotateDeltaRad(
				static_cast<double>(pos.x()), static_cast<double>(pos.y()));
			deltaRad *= kGizmoRotateArcGain;
			m_owner->m_lastMousePos = pos;

			if (std::abs(deltaRad) <= 1e-8)
			{
				return true;
			}

			ObjectGizmoFrame f;
			osg::MatrixTransform* const outerRot = m_owner->m_activeBackendOuterPat.get();
			if (m_owner->readActiveObjectGizmoFrame(f) && outerRot)
			{
				const osg::Quat R_old = f.attitude();
				const int axisIndex = dragAxisToIndex(m_owner->m_dragAxis);
				const bool worldFrame =
					m_owner->transformGizmoFrame() == OsgWidget::TransformGizmoFrame::World;
				osg::Vec3d axisForQuat;
				osg::Quat R_new = R_old;
				if (ObjectGizmoFrame::dragAxisDirectionOuterParent(
						outerRot, worldFrame, R_old, axisIndex, axisForQuat))
				{
					const osg::Quat deltaQuat(
						static_cast<float>(deltaRad),
						osg::Vec3(static_cast<float>(axisForQuat.x()), static_cast<float>(axisForQuat.y()),
							static_cast<float>(axisForQuat.z())));
					R_new = worldFrame ? (deltaQuat * R_old) : (R_old * deltaQuat);
				}
				f.adjustCenterPlusPoseForRotationDelta(R_old, R_new);
				f.applyToOuter(outerRot);
				markGizmoSessionModified();
				m_owner->syncActiveBackendRootFromObjectFrame(f, true);
				m_owner->syncCompassGizmoOrientation();
				const osg::Vec3f euler = m_owner->selectedRotationEulerDeg();
				emit m_owner->selectedObjectRotationChanged(euler.x(), euler.y(), euler.z());
				m_owner->requestRedraw();
			}
		}
		return true;
	}

	if (event->type() == QEvent::MouseMove && !m_owner->m_dragging && !m_owner->m_rotating)
	{
		if (!hasActiveObject)
		{
			return false;
		}
		QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
		if (mouseEvent->buttons().testFlag(Qt::LeftButton)
			|| mouseEvent->buttons().testFlag(Qt::MiddleButton)
			|| mouseEvent->buttons().testFlag(Qt::RightButton))
		{
			return false;
		}
		bool hoverRing = false;
		m_owner->m_hoverAxis = m_owner->pickAxisAtScreenPos(mouseEvent->pos(), true, &hoverRing);
		if (m_owner->m_hoverAxis == OsgWidget::DragAxis::None)
		{
			m_owner->m_hoverAxis = m_owner->pickAxisAtScreenPos(mouseEvent->pos(), false, &hoverRing);
			hoverRing = false;
		}
		m_owner->updateCompassHighlight(m_owner->m_hoverAxis, hoverRing);
		const int ax = static_cast<int>(m_owner->m_hoverAxis);
		if (m_lastEmittedHoverAxis != ax || m_lastEmittedHoverRing != hoverRing)
		{
			m_lastEmittedHoverAxis = ax;
			m_lastEmittedHoverRing = hoverRing;
			emit m_owner->activeAxisChanged(m_owner->axisToString(m_owner->m_hoverAxis));
		}
		m_owner->requestRedraw();
		return true;
	}

	if (event->type() == QEvent::MouseButtonRelease)
	{
		QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
		const bool hadGizmoDrag = m_owner->m_dragging || m_owner->m_rotating;
		if (mouseEvent->button() == Qt::LeftButton || mouseEvent->button() == Qt::RightButton)
		{
			m_owner->m_dragging = false;
			m_owner->m_rotating = false;
			m_owner->m_dragAxis = OsgWidget::DragAxis::None;
			resetGizmoDragSession(m_owner);
			m_owner->updateCompassHighlight(OsgWidget::DragAxis::None);
			m_lastEmittedHoverAxis = -1;
			m_lastEmittedHoverRing = false;
			emit m_owner->activeAxisChanged(QStringLiteral("None"));
		}
		if (hadGizmoDrag && m_gizmoSessionModified)
		{
			m_owner->syncActiveBackendRootFromSelectedTransform();
			m_owner->cacheSelectionGizmoPose();
			m_owner->refreshAnnotationTexts();
			m_owner->logGizmoPivotDiagnostics("gizmo_mouse_release_before_commit");
			emit m_owner->transformGizmoCommitted();
			m_owner->logGizmoPivotDiagnostics("gizmo_mouse_release_after_commit");
			m_owner->requestRedraw();
		}
		else if (hadGizmoDrag)
		{
			m_owner->requestRedraw();
		}
		return hadGizmoDrag;
	}

	if (event->type() == QEvent::Wheel || event->type() == QEvent::MouseButtonDblClick)
	{
		return false;
	}

	return false;
}
