#include "ObjectTransformOperation.h"

#include <QEvent>
#include <QMouseEvent>

#include <algorithm>
#include <cmath>

#include <osg/Quat>
#include <osg/Vec3>
#include <osg/Vec3d>

#include "OsgWidget.h"

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

osg::Vec3d worldUnitAxisForGizmo(OsgWidget* owner, OsgWidget::DragAxis axis)
{
	using DA = OsgWidget::DragAxis;
	if (axis == DA::None || !owner->m_selectedTransform.valid())
	{
		return osg::Vec3d(0.0, 0.0, 1.0);
	}
	if (owner->transformGizmoFrame() == OsgWidget::TransformGizmoFrame::World)
	{
		if (axis == DA::X) return osg::Vec3d(1.0, 0.0, 0.0);
		if (axis == DA::Y) return osg::Vec3d(0.0, 1.0, 0.0);
		return osg::Vec3d(0.0, 0.0, 1.0);
	}
	const osg::Vec3f loc = gizmoLocalAxis(axis);
	const osg::Quat q = owner->m_selectedTransform->getAttitude();
	const osg::Vec3f w = q * loc;
	osg::Vec3d wd(static_cast<double>(w.x()), static_cast<double>(w.y()), static_cast<double>(w.z()));
	const double len = wd.length();
	if (len < 1e-12)
	{
		return osg::Vec3d(0.0, 0.0, 1.0);
	}
	return wd / len;
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

void tryBeginRotateHit(OsgWidget* o, const QPoint& pos)
{
	o->m_gizmoTransDragPlaneActive = false;
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

bool ObjectTransformOperation::handleEvent(QObject* watched, QEvent* event)
{
	if (!m_owner || watched != m_owner->m_glWidget || !m_owner->m_objectSelectionMode || !m_owner->m_selectedTransform.valid())
	{
		return false;
	}

	if (event->type() == QEvent::MouseButtonPress)
	{
		QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
		if (mouseEvent->button() == Qt::LeftButton)
		{
			m_owner->m_dragAxis = m_owner->pickAxisAtScreenPos(mouseEvent->pos(), false);
			if (m_owner->m_dragAxis != OsgWidget::DragAxis::None)
			{
				m_owner->m_dragging = true;
				m_owner->m_rotating = false;
				m_owner->m_lastMousePos = mouseEvent->pos();
				tryBeginTranslatePlane(m_owner, mouseEvent->pos());
				m_owner->updateCompassHighlight(m_owner->m_dragAxis);
				emit m_owner->activeAxisChanged(m_owner->axisToString(m_owner->m_dragAxis));
				return true;
			}
			if (m_owner->pickAndActivateBackendAtScreenPos(mouseEvent->pos()))
			{
				return false;
			}
			return false;
		}
		if (mouseEvent->button() == Qt::RightButton)
		{
			m_owner->m_dragAxis = m_owner->pickAxisAtScreenPos(mouseEvent->pos(), true);
			if (m_owner->m_dragAxis == OsgWidget::DragAxis::None) m_owner->m_dragAxis = m_owner->m_hoverAxis;
			if (m_owner->m_dragAxis == OsgWidget::DragAxis::None) m_owner->m_dragAxis = OsgWidget::DragAxis::Z;
			m_owner->m_rotating = true;
			m_owner->m_dragging = false;
			m_owner->m_lastMousePos = mouseEvent->pos();
			tryBeginRotateHit(m_owner, mouseEvent->pos());
			m_owner->updateCompassHighlight(m_owner->m_dragAxis);
			emit m_owner->activeAxisChanged(m_owner->axisToString(m_owner->m_dragAxis));
			return true;
		}
		return false;
	}

	if (event->type() == QEvent::MouseMove && (m_owner->m_dragging || m_owner->m_rotating))
	{
		QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
		const QPoint pos = mouseEvent->pos();
		if (m_owner->m_dragging)
		{
			const osg::Vec3d axisW = worldUnitAxisForGizmo(m_owner, m_owner->m_dragAxis);
			double dsWorld = 0.0;
			osg::Vec3d eye, dir;
			const bool rayOk = m_owner->computeCameraScreenRayWorld(static_cast<double>(pos.x()), static_cast<double>(pos.y()), eye, dir);
			osg::Vec3d qNew;
			const bool hitOk = rayOk && m_owner->m_gizmoTransDragPlaneActive
				&& rayPlaneIntersect(eye, dir, m_owner->m_gizmoTransDragPlaneO, m_owner->m_gizmoTransDragPlaneN, qNew);
			if (hitOk)
			{
				dsWorld = (qNew - m_owner->m_gizmoDragLastHitWorld) * axisW;
				dsWorld *= kGizmoTranslatePlaneGain;
				m_owner->m_gizmoDragLastHitWorld = qNew;
			}
			else
			{
				const QPoint delta = pos - m_owner->m_lastMousePos;
				const float modelFactor = std::max(1.0f, m_owner->m_activeModelDiagonal * 0.0025f);
				const double dpr = (m_owner->OsgScene::devicePixelRatio() > 0.0) ? m_owner->OsgScene::devicePixelRatio() : 1.0;
				dsWorld = static_cast<double>(delta.x() - delta.y()) * static_cast<double>(1.15f * modelFactor * kGizmoTranslateFallbackGain * dpr);
			}
			m_owner->m_lastMousePos = pos;

			const osg::Vec3f oldPos = m_owner->m_selectedTransform->getPosition();
			osg::Vec3f newPos = oldPos;
			if (m_owner->transformGizmoFrame() == OsgWidget::TransformGizmoFrame::World)
			{
				osg::Vec3f position = oldPos - m_owner->m_modelCenter;
				if (m_owner->m_dragAxis == OsgWidget::DragAxis::X) position.x() += static_cast<float>(dsWorld);
				if (m_owner->m_dragAxis == OsgWidget::DragAxis::Y) position.y() += static_cast<float>(dsWorld);
				if (m_owner->m_dragAxis == OsgWidget::DragAxis::Z) position.z() += static_cast<float>(dsWorld);
				newPos = m_owner->m_modelCenter + position;
			}
			else
			{
				const osg::Vec3f step(
					static_cast<float>(axisW.x() * dsWorld),
					static_cast<float>(axisW.y() * dsWorld),
					static_cast<float>(axisW.z() * dsWorld));
				newPos = oldPos + step;
			}
			m_owner->m_selectedTransform->setPosition(newPos);
			m_owner->syncActiveBackendRootFromSelectedTransform();
			m_owner->refreshAnnotationTexts();
			m_owner->syncCompassGizmoOrientation();
			const osg::Vec3f pose = newPos - m_owner->m_modelCenter;
			emit m_owner->selectedObjectPoseChanged(pose.x(), pose.y(), pose.z());
		}
		else if (m_owner->m_rotating)
		{
			osg::Vec3f pivotF;
			m_owner->computeGizmoPivotWorld(pivotF);
			const osg::Vec3d pivot(static_cast<double>(pivotF.x()), static_cast<double>(pivotF.y()), static_cast<double>(pivotF.z()));
			const osg::Vec3d axisW = worldUnitAxisForGizmo(m_owner, m_owner->m_dragAxis);
			osg::Vec3d eye, dir;
			double deltaRad = 0.0;
			if (m_owner->computeCameraScreenRayWorld(static_cast<double>(pos.x()), static_cast<double>(pos.y()), eye, dir))
			{
				osg::Vec3d qHit;
				if (rayPlaneIntersect(eye, dir, pivot, axisW, qHit))
				{
					osg::Vec3d v0 = m_owner->m_gizmoDragLastHitWorld - pivot;
					osg::Vec3d v1 = qHit - pivot;
					const double along0 = v0 * axisW;
					const double along1 = v1 * axisW;
					v0 -= axisW * along0;
					v1 -= axisW * along1;
					const double l0 = v0.length();
					const double l1 = v1.length();
					if (l0 > 1e-8 && l1 > 1e-8)
					{
						v0 /= l0;
						v1 /= l1;
						const osg::Vec3d crossv = v0 ^ v1;
						const double sinTh = crossv * axisW;
						const double cosTh = v0 * v1;
						deltaRad = std::atan2(sinTh, cosTh);
						deltaRad *= kGizmoRotateArcGain;
					}
					m_owner->m_gizmoDragLastHitWorld = qHit;
				}
			}
			m_owner->m_lastMousePos = pos;

			const osg::Vec3f rBody(
				-m_owner->m_modelCenter.x(),
				-m_owner->m_modelCenter.y(),
				-m_owner->m_modelCenter.z());
			const osg::Vec3f T_old = m_owner->m_selectedTransform->getPosition();
			const osg::Quat R_old = m_owner->m_selectedTransform->getAttitude();
			osg::Quat R_new;
			if (m_owner->transformGizmoFrame() == OsgWidget::TransformGizmoFrame::World)
			{
				osg::Vec3 axis(0.0f, 0.0f, 1.0f);
				if (m_owner->m_dragAxis == OsgWidget::DragAxis::X) axis.set(1.0f, 0.0f, 0.0f);
				if (m_owner->m_dragAxis == OsgWidget::DragAxis::Y) axis.set(0.0f, 1.0f, 0.0f);
				if (m_owner->m_dragAxis == OsgWidget::DragAxis::Z) axis.set(0.0f, 0.0f, 1.0f);
				const osg::Quat deltaQuat(deltaRad, axis);
				R_new = deltaQuat * R_old;
			}
			else
			{
				const osg::Vec3f axf = gizmoLocalAxis(m_owner->m_dragAxis);
				const osg::Quat deltaQuat(deltaRad, osg::Vec3(axf.x(), axf.y(), axf.z()));
				R_new = R_old * deltaQuat;
			}
			const osg::Vec3f wOld = R_old * rBody;
			const osg::Vec3f wNew = R_new * rBody;
			const osg::Vec3f T_new = T_old + wOld - wNew;
			m_owner->m_selectedTransform->setPosition(T_new);
			m_owner->m_selectedTransform->setAttitude(R_new);
			m_owner->syncActiveBackendRootFromSelectedTransform();
			m_owner->refreshAnnotationTexts();
			m_owner->syncCompassGizmoOrientation();

			const osg::Vec3f euler = m_owner->quatToEulerDeg(R_new);
			emit m_owner->selectedObjectRotationChanged(euler.x(), euler.y(), euler.z());
		}
		return true;
	}

	if (event->type() == QEvent::MouseMove && !m_owner->m_dragging && !m_owner->m_rotating)
	{
		QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
		if (mouseEvent->buttons().testFlag(Qt::LeftButton)
			|| mouseEvent->buttons().testFlag(Qt::MiddleButton)
			|| mouseEvent->buttons().testFlag(Qt::RightButton))
		{
			return false;
		}
		m_owner->m_hoverAxis = m_owner->pickAxisAtScreenPos(mouseEvent->pos(), true);
		if (m_owner->m_hoverAxis == OsgWidget::DragAxis::None)
		{
			m_owner->m_hoverAxis = m_owner->pickAxisAtScreenPos(mouseEvent->pos(), false);
		}
		m_owner->updateCompassHighlight(m_owner->m_hoverAxis);
		emit m_owner->activeAxisChanged(m_owner->axisToString(m_owner->m_hoverAxis));
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
			emit m_owner->activeAxisChanged(QStringLiteral("None"));
		}
		if (hadGizmoDrag)
		{
			emit m_owner->transformGizmoCommitted();
		}
		return hadGizmoDrag;
	}

	if (event->type() == QEvent::Wheel || event->type() == QEvent::MouseButtonDblClick)
	{
		return false;
	}

	return false;
}
