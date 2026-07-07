#include "MeshSectionPlaneEditOperation.h"

#include "OsgWidget.h"

#include <QEvent>
#include <QMouseEvent>

#include <algorithm>
#include <cmath>

#include <osg/Vec3d>

namespace
{
constexpr double kTranslateGain = 1.08;
constexpr double kRotateGain = 1.18;

int dragAxisToIndex(const OsgWidget::DragAxis axis)
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

} // namespace

MeshSectionPlaneEditOperation::MeshSectionPlaneEditOperation(OsgWidget* owner)
	: SelectionOperation(owner)
{
}

bool MeshSectionPlaneEditOperation::handleEvent(QObject* watched, QEvent* event)
{
	if (!m_owner || watched != m_owner->m_glWidget || !m_owner->isMeshSectionPlaneEditActive())
	{
		return false;
	}

	if (event->type() == QEvent::MouseButtonPress)
	{
		auto* mouseEvent = static_cast<QMouseEvent*>(event);
		if (mouseEvent->button() == Qt::LeftButton)
		{
			const int picked = m_owner->pickMeshSectionPlaneAxisAtScreenPos(mouseEvent->pos(), false);
			m_owner->m_sectionPlaneDragAxis = static_cast<OsgWidget::DragAxis>(picked);
			if (m_owner->m_sectionPlaneDragAxis != OsgWidget::DragAxis::None)
			{
				m_owner->m_sectionPlaneDragging = true;
				m_owner->m_sectionPlaneRotating = false;
				m_owner->m_lastMousePos = mouseEvent->pos();
				(void)m_owner->beginMeshSectionPlaneScreenDrag();
				m_owner->updateMeshSectionPlaneCompassHighlight(m_owner->m_sectionPlaneDragAxis, false);
				m_owner->requestRedraw();
				return true;
			}
			osg::Vec3d hitWorld;
			if (m_owner->pickMeshSectionPlaneDragPoint(mouseEvent->pos(), hitWorld))
			{
				m_owner->m_sectionPlanePlaneDragging = true;
				m_owner->m_sectionPlaneDragLastHitWorld = hitWorld;
				m_owner->m_lastMousePos = mouseEvent->pos();
				m_owner->requestRedraw();
				return true;
			}
		}
		if (mouseEvent->button() == Qt::RightButton)
		{
			bool hoverRing = false;
			int picked = m_owner->pickMeshSectionPlaneAxisAtScreenPos(mouseEvent->pos(), true, &hoverRing);
			if (picked == OsgWidget::kGizmoAxisNone)
			{
				picked = static_cast<int>(m_owner->m_sectionPlaneHoverAxis);
			}
			if (picked == OsgWidget::kGizmoAxisNone)
			{
				picked = OsgWidget::kGizmoAxisX;
			}
			m_owner->m_sectionPlaneDragAxis = static_cast<OsgWidget::DragAxis>(picked);
			m_owner->m_sectionPlaneRotating = true;
			m_owner->m_sectionPlaneDragging = false;
			m_owner->m_sectionPlanePlaneDragging = false;
			m_owner->m_lastMousePos = mouseEvent->pos();
			osg::Vec3d pivot;
			m_owner->computeMeshSectionPlanePivotWorld(pivot);
			m_owner->m_sectionPlaneRotatePivotWorld = pivot;
			osg::Vec3d eye;
			osg::Vec3d dir;
			if (m_owner->computeCameraScreenRayWorld(
					static_cast<double>(mouseEvent->pos().x()),
					static_cast<double>(mouseEvent->pos().y()),
					eye,
					dir))
			{
				osg::Vec3d axisW;
				(void)m_owner->meshSectionPlaneCompassUnitAxisWorld(m_owner->m_sectionPlaneDragAxis, axisW);
				osg::Vec3d hit;
				if (rayPlaneIntersect(eye, dir, pivot, axisW, hit))
				{
					m_owner->m_sectionPlaneDragLastHitWorld = hit;
				}
				else
				{
					m_owner->m_sectionPlaneDragLastHitWorld = pivot;
				}
			}
			m_owner->updateMeshSectionPlaneCompassHighlight(m_owner->m_sectionPlaneDragAxis, true);
			m_owner->requestRedraw();
			return true;
		}
		return false;
	}

	if (event->type() == QEvent::MouseMove
		&& (m_owner->m_sectionPlaneDragging || m_owner->m_sectionPlaneRotating || m_owner->m_sectionPlanePlaneDragging))
	{
		auto* mouseEvent = static_cast<QMouseEvent*>(event);
		const QPoint pos = mouseEvent->pos();
		if (m_owner->m_sectionPlanePlaneDragging)
		{
			osg::Vec3d hitWorld;
			if (m_owner->pickMeshSectionPlaneDragPoint(pos, hitWorld))
			{
				m_owner->applyMeshSectionPlaneTranslationWorld(
					hitWorld,
					m_owner->m_sectionPlaneDragLastHitWorld);
				m_owner->m_sectionPlaneDragLastHitWorld = hitWorld;
				m_owner->notifyMeshSectionPlaneChanged();
			}
		}
		else if (m_owner->m_sectionPlaneDragging)
		{
			double dsWorld = m_owner->meshSectionPlaneScreenDragDsMm(pos, m_owner->m_lastMousePos);
			dsWorld *= kTranslateGain;
			const double cap = std::max(2.0, static_cast<double>(m_owner->m_sectionPlaneModelDiagonal) * 0.04);
			dsWorld = std::max(-cap, std::min(cap, dsWorld));
			m_owner->m_lastMousePos = pos;
			if (std::abs(dsWorld) > 1e-10)
			{
				m_owner->applyMeshSectionPlaneTranslationAxis(0, dsWorld);
				m_owner->notifyMeshSectionPlaneChanged();
			}
		}
		else if (m_owner->m_sectionPlaneRotating)
		{
			const osg::Vec3d pivot = m_owner->m_sectionPlaneRotatePivotWorld;
			osg::Vec3d axisW;
			(void)m_owner->meshSectionPlaneCompassUnitAxisWorld(m_owner->m_sectionPlaneDragAxis, axisW);
			osg::Vec3d eye;
			osg::Vec3d dir;
			double deltaRad = 0.0;
			if (m_owner->computeCameraScreenRayWorld(static_cast<double>(pos.x()), static_cast<double>(pos.y()), eye, dir))
			{
				osg::Vec3d qHit;
				if (rayPlaneIntersect(eye, dir, pivot, axisW, qHit))
				{
					osg::Vec3d v0 = m_owner->m_sectionPlaneDragLastHitWorld - pivot;
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
						deltaRad = std::atan2(sinTh, cosTh) * kRotateGain;
					}
					m_owner->m_sectionPlaneDragLastHitWorld = qHit;
				}
			}
			m_owner->m_lastMousePos = pos;
			if (std::abs(deltaRad) > 1e-8)
			{
				const int ax = dragAxisToIndex(m_owner->m_sectionPlaneDragAxis);
				m_owner->applyMeshSectionPlaneRotationAxis(ax, deltaRad);
				m_owner->notifyMeshSectionPlaneChanged();
			}
		}
		m_owner->requestRedraw();
		return true;
	}

	if (event->type() == QEvent::MouseMove && !m_owner->m_sectionPlaneDragging && !m_owner->m_sectionPlaneRotating
		&& !m_owner->m_sectionPlanePlaneDragging)
	{
		auto* mouseEvent = static_cast<QMouseEvent*>(event);
		if (mouseEvent->buttons().testFlag(Qt::LeftButton) || mouseEvent->buttons().testFlag(Qt::MiddleButton)
			|| mouseEvent->buttons().testFlag(Qt::RightButton))
		{
			return false;
		}
		bool hoverRing = false;
		const int picked = m_owner->pickMeshSectionPlaneAxisAtScreenPos(mouseEvent->pos(), true, &hoverRing);
		if (picked == OsgWidget::kGizmoAxisNone)
		{
			bool ring2 = false;
			const int picked2 = m_owner->pickMeshSectionPlaneAxisAtScreenPos(mouseEvent->pos(), false, &ring2);
			m_owner->m_sectionPlaneHoverAxis = static_cast<OsgWidget::DragAxis>(picked2);
			hoverRing = false;
		}
		else
		{
			m_owner->m_sectionPlaneHoverAxis = static_cast<OsgWidget::DragAxis>(picked);
		}
		m_owner->updateMeshSectionPlaneCompassHighlight(m_owner->m_sectionPlaneHoverAxis, hoverRing);
		m_lastEmittedHoverAxis = picked;
		m_lastEmittedHoverRing = hoverRing;
		m_owner->requestRedraw();
		return true;
	}

	if (event->type() == QEvent::MouseButtonRelease)
	{
		auto* mouseEvent = static_cast<QMouseEvent*>(event);
		const bool hadDrag = m_owner->m_sectionPlaneDragging || m_owner->m_sectionPlaneRotating
			|| m_owner->m_sectionPlanePlaneDragging;
		if (mouseEvent->button() == Qt::LeftButton || mouseEvent->button() == Qt::RightButton)
		{
			m_owner->m_sectionPlaneDragging = false;
			m_owner->m_sectionPlaneRotating = false;
			m_owner->m_sectionPlanePlaneDragging = false;
			m_owner->m_sectionPlaneDragAxis = OsgWidget::DragAxis::None;
			m_owner->updateMeshSectionPlaneCompassHighlight(OsgWidget::DragAxis::None);
			m_lastEmittedHoverAxis = -1;
			m_lastEmittedHoverRing = false;
		}
		m_owner->requestRedraw();
		return hadDrag;
	}

	return false;
}
