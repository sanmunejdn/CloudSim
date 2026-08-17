/// @file RobotTcpDragTeachOperation.cpp
/// @brief RobotTcpDragTeach 操作

#include "RobotTcpDragTeachOperation.h"

#include "OsgWidget.h"

#include <QEvent>
#include <QMouseEvent>
#include <algorithm>
#include <cmath>

#include <osg/Vec3d>

namespace
{
constexpr double kGizmoTranslatePlaneGain = 1.08;
constexpr double kGizmoRotateArcGain = 1.18;

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

osg::Vec3d tcpTeachWorldUnitAxis(OsgWidget* owner, OsgWidget::DragAxis axis)
{
	// 与拾取/屏幕标定同一套轴向（含基座），避免 Local 旋转拖拽缺 R_base
	osg::Vec3d out(0.0, 0.0, 1.0);
	if (owner->tcpTeachCompassUnitAxisWorld(axis, out))
	{
		return out;
	}
	return osg::Vec3d(0.0, 0.0, 1.0);
}

bool rayPlaneIntersect(const osg::Vec3d& rayOrigin, const osg::Vec3d& rayDirUnit, const osg::Vec3d& planePoint,
					   const osg::Vec3d& planeNormalUnit, osg::Vec3d& outHit)
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

double maxTcpTeachTranslateStep(const OsgWidget* owner)
{
	const double diag = std::max(100.0, static_cast<double>(owner->m_tcpTeachModelDiagonal));
	return std::max(2.0, diag * 0.04);
}

double clampDs(const double ds, const OsgWidget* owner)
{
	const double cap = maxTcpTeachTranslateStep(owner);
	return std::max(-cap, std::min(cap, ds));
}

void resetTcpDragSession(OsgWidget* o)
{
	o->m_tcpTeachTransDragPlaneActive = false;
	o->m_tcpTeachRotatePivotActive = false;
}

bool currentTcpPivotWorldD(const OsgWidget* owner, osg::Vec3d& outPivot)
{
	osg::Vec3f pivotF;
	owner->computeTcpTeachPivotWorld(pivotF);
	outPivot.set(static_cast<double>(pivotF.x()), static_cast<double>(pivotF.y()), static_cast<double>(pivotF.z()));
	return true;
}

} // namespace

RobotTcpDragTeachOperation::RobotTcpDragTeachOperation(OsgWidget* owner) : SelectionOperation(owner) {}

bool RobotTcpDragTeachOperation::handleEvent(QObject* watched, QEvent* event)
{
	if (!m_owner || watched != m_owner->m_glWidget || !m_owner->m_tcpTeachActive)
	{
		return false;
	}

	if (event->type() == QEvent::MouseButtonPress)
	{
		auto* mouseEvent = static_cast<QMouseEvent*>(event);
		if (mouseEvent->button() == Qt::LeftButton)
		{
			const int picked = m_owner->pickTcpTeachAxisAtScreenPos(mouseEvent->pos(), false);
			m_owner->m_tcpTeachDragAxis = static_cast<OsgWidget::DragAxis>(picked);
			if (m_owner->m_tcpTeachDragAxis != OsgWidget::DragAxis::None)
			{
				m_sessionModified = false;
				m_owner->m_tcpTeachDragging = true;
				m_owner->m_tcpTeachRotating = false;
				m_owner->m_lastMousePos = mouseEvent->pos();
				resetTcpDragSession(m_owner);
				(void)m_owner->beginTcpTeachScreenDrag();
				m_owner->updateTcpTeachCompassHighlight(m_owner->m_tcpTeachDragAxis, false);
				m_owner->requestRedraw();
				return true;
			}
		}
		if (mouseEvent->button() == Qt::RightButton)
		{
			bool hoverRing = false;
			int picked = m_owner->pickTcpTeachAxisAtScreenPos(mouseEvent->pos(), true, &hoverRing);
			if (picked == OsgWidget::kGizmoAxisNone)
			{
				picked = static_cast<int>(m_owner->m_tcpTeachHoverAxis);
			}
			if (picked == OsgWidget::kGizmoAxisNone)
			{
				picked = OsgWidget::kGizmoAxisZ;
			}
			m_owner->m_tcpTeachDragAxis = static_cast<OsgWidget::DragAxis>(picked);
			m_sessionModified = false;
			m_owner->m_tcpTeachRotating = true;
			m_owner->m_tcpTeachDragging = false;
			m_owner->m_lastMousePos = mouseEvent->pos();
			osg::Vec3d pivot;
			currentTcpPivotWorldD(m_owner, pivot);
			m_owner->m_tcpTeachRotatePivotWorld = pivot;
			m_owner->m_tcpTeachRotatePivotActive = true;
			osg::Vec3d eye, dir;
			if (m_owner->computeCameraScreenRayWorld(static_cast<double>(mouseEvent->pos().x()),
													 static_cast<double>(mouseEvent->pos().y()), eye, dir))
			{
				const osg::Vec3d axisW = tcpTeachWorldUnitAxis(m_owner, m_owner->m_tcpTeachDragAxis);
				osg::Vec3d hit;
				if (rayPlaneIntersect(eye, dir, pivot, axisW, hit))
				{
					m_owner->m_tcpTeachDragLastHitWorld = hit;
				}
				else
				{
					m_owner->m_tcpTeachDragLastHitWorld = pivot;
				}
			}
			m_owner->updateTcpTeachCompassHighlight(m_owner->m_tcpTeachDragAxis, true);
			m_owner->requestRedraw();
			return true;
		}
		return false;
	}

	if (event->type() == QEvent::MouseMove && (m_owner->m_tcpTeachDragging || m_owner->m_tcpTeachRotating))
	{
		auto* mouseEvent = static_cast<QMouseEvent*>(event);
		const QPoint pos = mouseEvent->pos();
		if (m_owner->m_tcpTeachDragging)
		{
			double dsWorld = m_owner->tcpTeachScreenDragDsMm(pos, m_owner->m_lastMousePos);
			dsWorld *= kGizmoTranslatePlaneGain;
			dsWorld = clampDs(dsWorld, m_owner);
			const int ax = dragAxisToIndex(m_owner->m_tcpTeachDragAxis);
			m_owner->m_lastMousePos = pos;
			if (std::abs(dsWorld) > 1e-10)
			{
				// 示教平移固定沿 TCP 轴；勿跟 View 菜单切到世界系
				m_owner->applyTcpTeachTranslationBody(ax, dsWorld);
				m_sessionModified = true;
				m_owner->emitTcpDragTeachPoseChanged();
			}
		}
		else if (m_owner->m_tcpTeachRotating)
		{
			const osg::Vec3d pivot = m_owner->m_tcpTeachRotatePivotWorld;
			const osg::Vec3d axisW = tcpTeachWorldUnitAxis(m_owner, m_owner->m_tcpTeachDragAxis);
			osg::Vec3d eye;
			osg::Vec3d dir;
			double deltaRad = 0.0;
			if (m_owner->computeCameraScreenRayWorld(static_cast<double>(pos.x()), static_cast<double>(pos.y()), eye,
													 dir))
			{
				osg::Vec3d qHit;
				if (rayPlaneIntersect(eye, dir, pivot, axisW, qHit))
				{
					osg::Vec3d v0 = m_owner->m_tcpTeachDragLastHitWorld - pivot;
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
						deltaRad = std::atan2(sinTh, cosTh) * kGizmoRotateArcGain;
					}
					m_owner->m_tcpTeachDragLastHitWorld = qHit;
				}
			}
			m_owner->m_lastMousePos = pos;
			if (std::abs(deltaRad) > 1e-8)
			{
				const int ax = dragAxisToIndex(m_owner->m_tcpTeachDragAxis);
				// 示教旋转固定绕 TCP 轴
				m_owner->applyTcpTeachRotationBody(ax, deltaRad);
				m_sessionModified = true;
				m_owner->emitTcpDragTeachPoseChanged();
			}
		}
		m_owner->requestRedraw();
		return true;
	}

	if (event->type() == QEvent::MouseMove && !m_owner->m_tcpTeachDragging && !m_owner->m_tcpTeachRotating)
	{
		auto* mouseEvent = static_cast<QMouseEvent*>(event);
		if (mouseEvent->buttons().testFlag(Qt::LeftButton) || mouseEvent->buttons().testFlag(Qt::MiddleButton) ||
			mouseEvent->buttons().testFlag(Qt::RightButton))
		{
			return false;
		}
		bool hoverRing = false;
		const int picked = m_owner->pickTcpTeachAxisAtScreenPos(mouseEvent->pos(), true, &hoverRing);
		if (picked == OsgWidget::kGizmoAxisNone)
		{
			bool ring2 = false;
			const int picked2 = m_owner->pickTcpTeachAxisAtScreenPos(mouseEvent->pos(), false, &ring2);
			m_owner->m_tcpTeachHoverAxis = static_cast<OsgWidget::DragAxis>(picked2);
			hoverRing = false;
		}
		else
		{
			m_owner->m_tcpTeachHoverAxis = static_cast<OsgWidget::DragAxis>(picked);
		}
		m_owner->updateTcpTeachCompassHighlight(m_owner->m_tcpTeachHoverAxis, hoverRing);
		if (m_lastEmittedHoverAxis != picked || m_lastEmittedHoverRing != hoverRing)
		{
			m_lastEmittedHoverAxis = picked;
			m_lastEmittedHoverRing = hoverRing;
		}
		m_owner->requestRedraw();
		return true;
	}

	if (event->type() == QEvent::MouseButtonRelease)
	{
		auto* mouseEvent = static_cast<QMouseEvent*>(event);
		const bool hadDrag = m_owner->m_tcpTeachDragging || m_owner->m_tcpTeachRotating;
		if (mouseEvent->button() == Qt::LeftButton || mouseEvent->button() == Qt::RightButton)
		{
			m_owner->m_tcpTeachDragging = false;
			m_owner->m_tcpTeachRotating = false;
			m_owner->m_tcpTeachDragAxis = OsgWidget::DragAxis::None;
			resetTcpDragSession(m_owner);
			m_owner->updateTcpTeachCompassHighlight(OsgWidget::DragAxis::None);
			m_lastEmittedHoverAxis = -1;
			m_lastEmittedHoverRing = false;
		}
		if (hadDrag && m_sessionModified)
		{
			m_owner->emitTcpDragTeachPoseChanged();
		}
		m_owner->requestRedraw();
		return hadDrag;
	}

	return false;
}
