#include "ObjectTransformOperation.h"

#include <QEvent>
#include <QMouseEvent>

#include <osg/Quat>
#include <osg/Vec3>

#include "OsgWidget.h"

namespace
{
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
				m_owner->updateCompassHighlight(m_owner->m_dragAxis);
				emit m_owner->activeAxisChanged(m_owner->axisToString(m_owner->m_dragAxis));
				return true;
			}
			// Click-select object under cursor when not hitting the gizmo axis.
			if (m_owner->pickAndActivateBackendAtScreenPos(mouseEvent->pos()))
			{
				// Do not swallow: allow left-drag camera rotate from same mode.
				return false;
			}
			// Not axis/object hit: forward to camera manipulator.
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
			m_owner->updateCompassHighlight(m_owner->m_dragAxis);
			emit m_owner->activeAxisChanged(m_owner->axisToString(m_owner->m_dragAxis));
			return true;
		}
		// Middle / extra buttons: forward for view pan/zoom behaviors.
		return false;
	}

	if (event->type() == QEvent::MouseMove && (m_owner->m_dragging || m_owner->m_rotating))
	{
		QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
		const QPoint delta = mouseEvent->pos() - m_owner->m_lastMousePos;
		m_owner->m_lastMousePos = mouseEvent->pos();
		if (m_owner->m_dragging)
		{
			// Adaptive move sensitivity:
			// - keep a usable minimum for small models
			// - scale up with model size for large assemblies.
			const float modelFactor = std::max(1.0f, m_owner->m_activeModelDiagonal * 0.0025f);
			const float move = static_cast<float>(delta.x() - delta.y()) * (1.2f * modelFactor);
			const osg::Vec3f oldPos = m_owner->m_selectedTransform->getPosition();
			osg::Vec3f newPos = oldPos;
			if (m_owner->transformGizmoFrame() == OsgWidget::TransformGizmoFrame::World)
			{
				osg::Vec3f position = oldPos - m_owner->m_modelCenter;
				if (m_owner->m_dragAxis == OsgWidget::DragAxis::X) position.x() += move;
				if (m_owner->m_dragAxis == OsgWidget::DragAxis::Y) position.y() += move;
				if (m_owner->m_dragAxis == OsgWidget::DragAxis::Z) position.z() += move;
				newPos = m_owner->m_modelCenter + position;
			}
			else
			{
				const osg::Quat q = m_owner->m_selectedTransform->getAttitude();
				const osg::Vec3f localAxis = gizmoLocalAxis(m_owner->m_dragAxis);
				const osg::Vec3f worldDir = q * localAxis;
				newPos = oldPos + worldDir * move;
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
			const float deltaDeg = static_cast<float>(delta.x() - delta.y()) * 0.12f;
			const float deltaRad = osg::DegreesToRadians(deltaDeg);
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
		// While navigation buttons are down, let camera manipulator handle move.
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
			m_owner->updateCompassHighlight(OsgWidget::DragAxis::None);
			emit m_owner->activeAxisChanged(QStringLiteral("None"));
		}
		// Release without active gizmo drag should go to camera manipulator.
		return hadGizmoDrag;
	}

	if (event->type() == QEvent::Wheel || event->type() == QEvent::MouseButtonDblClick)
	{
		return false;
	}

	return false;
}

