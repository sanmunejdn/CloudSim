/// @file RobotTcpDragTeachOperation.cpp
/// @brief RobotTcpDragTeach 操作

#include "RobotTcpDragTeachOperation.h"

#include "OsgScene.h"
#include "ViewportInteraction/IViewportInteractionHost.h"

#include <QEvent>
#include <QMouseEvent>
#include <algorithm>
#include <cmath>

#include <osg/Vec3d>

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

double clampDs(const double ds, const IViewportInteractionHost* host)
{
	const double cap = host->tcpTeachMaxTranslateStep();
	return std::max(-cap, std::min(cap, ds));
}

} // namespace

RobotTcpDragTeachOperation::RobotTcpDragTeachOperation(IViewportInteractionHost* host) : SelectionOperation(host) {}

bool RobotTcpDragTeachOperation::handleEvent(QObject* watched, QEvent* event)
{
	IViewportInteractionHost* h = host();
	if (!h || watched != h->viewportGlWidget() || !h->tcpTeachActive())
	{
		return false;
	}

	ViewportTcpTeachDragState tcp = h->tcpTeachDragState();
	ViewportInteractionPointerState pointer = h->interactionPointerState();

	if (event->type() == QEvent::MouseButtonPress)
	{
		auto* mouseEvent = static_cast<QMouseEvent*>(event);
		if (mouseEvent->button() == Qt::LeftButton)
		{
			const int picked = h->pickTcpTeachAxisAtScreenPos(mouseEvent->pos(), false);
			tcp.dragAxis = static_cast<OsgScene::DragAxis>(picked);
			if (tcp.dragAxis != OsgScene::DragAxis::None)
			{
				m_sessionModified = false;
				tcp.dragging = true;
				tcp.rotating = false;
				pointer.lastMousePos = mouseEvent->pos();
				h->resetTcpTeachDragSession();
				(void)h->beginTcpTeachScreenDrag();
				h->updateTcpTeachCompassHighlight(tcp.dragAxis, false);
				h->requestRedraw();
				return true;
			}
		}
		if (mouseEvent->button() == Qt::RightButton)
		{
			bool hoverRing = false;
			int picked = h->pickTcpTeachAxisAtScreenPos(mouseEvent->pos(), true, &hoverRing);
			if (picked == OsgScene::kGizmoAxisNone)
			{
				picked = static_cast<int>(tcp.hoverAxis);
			}
			if (picked == OsgScene::kGizmoAxisNone)
			{
				picked = OsgScene::kGizmoAxisZ;
			}
			tcp.dragAxis = static_cast<OsgScene::DragAxis>(picked);
			m_sessionModified = false;
			tcp.rotating = true;
			tcp.dragging = false;
			pointer.lastMousePos = mouseEvent->pos();
			osg::Vec3f pivotF;
			h->computeTcpTeachPivotWorld(pivotF);
			const osg::Vec3d pivot(static_cast<double>(pivotF.x()), static_cast<double>(pivotF.y()),
								   static_cast<double>(pivotF.z()));
			tcp.rotatePivotWorld = pivot;
			tcp.rotatePivotActive = true;
			osg::Vec3d eye, dir;
			if (h->computeCameraScreenRayWorld(static_cast<double>(mouseEvent->pos().x()),
											   static_cast<double>(mouseEvent->pos().y()), eye, dir))
			{
				const osg::Vec3d axisW = h->tcpTeachWorldUnitAxis(tcp.dragAxis);
				osg::Vec3d hit;
				if (rayPlaneIntersect(eye, dir, pivot, axisW, hit))
				{
					tcp.dragLastHitWorld = hit;
				}
				else
				{
					tcp.dragLastHitWorld = pivot;
				}
			}
			h->updateTcpTeachCompassHighlight(tcp.dragAxis, true);
			h->requestRedraw();
			return true;
		}
		return false;
	}

	if (event->type() == QEvent::MouseMove && (tcp.dragging || tcp.rotating))
	{
		auto* mouseEvent = static_cast<QMouseEvent*>(event);
		const QPoint pos = mouseEvent->pos();
		if (tcp.dragging)
		{
			double dsWorld = h->tcpTeachScreenDragDsMm(pos, pointer.lastMousePos);
			dsWorld *= kGizmoTranslatePlaneGain;
			dsWorld = clampDs(dsWorld, h);
			const int ax = dragAxisToIndex(tcp.dragAxis);
			pointer.lastMousePos = pos;
			if (std::abs(dsWorld) > 1e-10)
			{
				h->applyTcpTeachTranslationBody(ax, dsWorld);
				m_sessionModified = true;
				h->emitTcpDragTeachPoseChanged();
			}
		}
		else if (tcp.rotating)
		{
			const osg::Vec3d pivot = tcp.rotatePivotWorld;
			const osg::Vec3d axisW = h->tcpTeachWorldUnitAxis(tcp.dragAxis);
			osg::Vec3d eye;
			osg::Vec3d dir;
			double deltaRad = 0.0;
			if (h->computeCameraScreenRayWorld(static_cast<double>(pos.x()), static_cast<double>(pos.y()), eye, dir))
			{
				osg::Vec3d qHit;
				if (rayPlaneIntersect(eye, dir, pivot, axisW, qHit))
				{
					osg::Vec3d v0 = tcp.dragLastHitWorld - pivot;
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
					tcp.dragLastHitWorld = qHit;
				}
			}
			pointer.lastMousePos = pos;
			if (std::abs(deltaRad) > 1e-8)
			{
				const int ax = dragAxisToIndex(tcp.dragAxis);
				h->applyTcpTeachRotationBody(ax, deltaRad);
				m_sessionModified = true;
				h->emitTcpDragTeachPoseChanged();
			}
		}
		h->requestRedraw();
		return true;
	}

	if (event->type() == QEvent::MouseMove && !tcp.dragging && !tcp.rotating)
	{
		auto* mouseEvent = static_cast<QMouseEvent*>(event);
		if (mouseEvent->buttons().testFlag(Qt::LeftButton) || mouseEvent->buttons().testFlag(Qt::MiddleButton) ||
			mouseEvent->buttons().testFlag(Qt::RightButton))
		{
			return false;
		}
		bool hoverRing = false;
		const int picked = h->pickTcpTeachAxisAtScreenPos(mouseEvent->pos(), true, &hoverRing);
		if (picked == OsgScene::kGizmoAxisNone)
		{
			bool ring2 = false;
			const int picked2 = h->pickTcpTeachAxisAtScreenPos(mouseEvent->pos(), false, &ring2);
			tcp.hoverAxis = static_cast<OsgScene::DragAxis>(picked2);
			hoverRing = false;
		}
		else
		{
			tcp.hoverAxis = static_cast<OsgScene::DragAxis>(picked);
		}
		h->updateTcpTeachCompassHighlight(tcp.hoverAxis, hoverRing);
		if (m_lastEmittedHoverAxis != picked || m_lastEmittedHoverRing != hoverRing)
		{
			m_lastEmittedHoverAxis = picked;
			m_lastEmittedHoverRing = hoverRing;
		}
		h->requestRedraw();
		return true;
	}

	if (event->type() == QEvent::MouseButtonRelease)
	{
		auto* mouseEvent = static_cast<QMouseEvent*>(event);
		const bool hadDrag = tcp.dragging || tcp.rotating;
		if (mouseEvent->button() == Qt::LeftButton || mouseEvent->button() == Qt::RightButton)
		{
			tcp.dragging = false;
			tcp.rotating = false;
			tcp.dragAxis = OsgScene::DragAxis::None;
			h->resetTcpTeachDragSession();
			h->updateTcpTeachCompassHighlight(OsgScene::DragAxis::None);
			m_lastEmittedHoverAxis = -1;
			m_lastEmittedHoverRing = false;
		}
		if (hadDrag && m_sessionModified)
		{
			h->emitTcpDragTeachPoseChanged();
		}
		h->requestRedraw();
		return hadDrag;
	}

	return false;
}
