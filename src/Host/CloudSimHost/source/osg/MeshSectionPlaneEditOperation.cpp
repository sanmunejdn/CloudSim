/// @file MeshSectionPlaneEditOperation.cpp
/// @brief MeshSectionPlaneEdit 操作

#include "MeshSectionPlaneEditOperation.h"

#include "OsgScene.h"
#include "ViewportInteraction/IViewportInteractionHost.h"

#include <QEvent>
#include <QMouseEvent>
#include <algorithm>
#include <cmath>

#include <osg/Vec3d>

namespace
{
constexpr double kTranslateGain = 1.08;
constexpr double kRotateGain = 1.18;

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

} // namespace

MeshSectionPlaneEditOperation::MeshSectionPlaneEditOperation(IViewportInteractionHost* host) : SelectionOperation(host)
{
}

bool MeshSectionPlaneEditOperation::handleEvent(QObject* watched, QEvent* event)
{
	IViewportInteractionHost* h = host();
	if (!h || watched != h->viewportGlWidget() || !h->meshSectionPlaneEditActive())
	{
		return false;
	}

	ViewportMeshSectionPlaneDragState section = h->meshSectionPlaneDragState();
	ViewportInteractionPointerState pointer = h->interactionPointerState();

	if (event->type() == QEvent::MouseButtonPress)
	{
		auto* mouseEvent = static_cast<QMouseEvent*>(event);
		if (mouseEvent->button() == Qt::LeftButton)
		{
			const int picked = h->pickMeshSectionPlaneAxisAtScreenPos(mouseEvent->pos(), false);
			section.dragAxis = static_cast<OsgScene::DragAxis>(picked);
			if (section.dragAxis != OsgScene::DragAxis::None)
			{
				section.dragging = true;
				section.rotating = false;
				pointer.lastMousePos = mouseEvent->pos();
				(void)h->beginMeshSectionPlaneScreenDrag();
				h->updateMeshSectionPlaneCompassHighlight(section.dragAxis, false);
				h->requestRedraw();
				return true;
			}
			osg::Vec3d hitWorld;
			if (h->pickMeshSectionPlaneDragPoint(mouseEvent->pos(), hitWorld))
			{
				section.planeDragging = true;
				section.dragLastHitWorld = hitWorld;
				pointer.lastMousePos = mouseEvent->pos();
				h->requestRedraw();
				return true;
			}
		}
		if (mouseEvent->button() == Qt::RightButton)
		{
			bool hoverRing = false;
			int picked = h->pickMeshSectionPlaneAxisAtScreenPos(mouseEvent->pos(), true, &hoverRing);
			if (picked == OsgScene::kGizmoAxisNone)
			{
				picked = static_cast<int>(section.hoverAxis);
			}
			if (picked == OsgScene::kGizmoAxisNone)
			{
				picked = OsgScene::kGizmoAxisX;
			}
			section.dragAxis = static_cast<OsgScene::DragAxis>(picked);
			section.rotating = true;
			section.dragging = false;
			section.planeDragging = false;
			pointer.lastMousePos = mouseEvent->pos();
			osg::Vec3d pivot;
			h->computeMeshSectionPlanePivotWorld(pivot);
			section.rotatePivotWorld = pivot;
			osg::Vec3d eye;
			osg::Vec3d dir;
			if (h->computeCameraScreenRayWorld(static_cast<double>(mouseEvent->pos().x()),
											   static_cast<double>(mouseEvent->pos().y()), eye, dir))
			{
				osg::Vec3d axisW;
				(void)h->meshSectionPlaneCompassUnitAxisWorld(section.dragAxis, axisW);
				osg::Vec3d hit;
				if (rayPlaneIntersect(eye, dir, pivot, axisW, hit))
				{
					section.dragLastHitWorld = hit;
				}
				else
				{
					section.dragLastHitWorld = pivot;
				}
			}
			h->updateMeshSectionPlaneCompassHighlight(section.dragAxis, true);
			h->requestRedraw();
			return true;
		}
		return false;
	}

	if (event->type() == QEvent::MouseMove && (section.dragging || section.rotating || section.planeDragging))
	{
		auto* mouseEvent = static_cast<QMouseEvent*>(event);
		const QPoint pos = mouseEvent->pos();
		if (section.planeDragging)
		{
			osg::Vec3d hitWorld;
			if (h->pickMeshSectionPlaneDragPoint(pos, hitWorld))
			{
				h->applyMeshSectionPlaneTranslationWorld(hitWorld, section.dragLastHitWorld);
				section.dragLastHitWorld = hitWorld;
				h->notifyMeshSectionPlaneChanged();
			}
		}
		else if (section.dragging)
		{
			double dsWorld = h->meshSectionPlaneScreenDragDsMm(pos, pointer.lastMousePos);
			dsWorld *= kTranslateGain;
			const double cap = std::max(2.0, static_cast<double>(section.modelDiagonal) * 0.04);
			dsWorld = std::max(-cap, std::min(cap, dsWorld));
			pointer.lastMousePos = pos;
			if (std::abs(dsWorld) > 1e-10)
			{
				h->applyMeshSectionPlaneTranslationAxis(0, dsWorld);
				h->notifyMeshSectionPlaneChanged();
			}
		}
		else if (section.rotating)
		{
			const osg::Vec3d pivot = section.rotatePivotWorld;
			osg::Vec3d axisW;
			(void)h->meshSectionPlaneCompassUnitAxisWorld(section.dragAxis, axisW);
			osg::Vec3d eye;
			osg::Vec3d dir;
			double deltaRad = 0.0;
			if (h->computeCameraScreenRayWorld(static_cast<double>(pos.x()), static_cast<double>(pos.y()), eye, dir))
			{
				osg::Vec3d qHit;
				if (rayPlaneIntersect(eye, dir, pivot, axisW, qHit))
				{
					osg::Vec3d v0 = section.dragLastHitWorld - pivot;
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
					section.dragLastHitWorld = qHit;
				}
			}
			pointer.lastMousePos = pos;
			if (std::abs(deltaRad) > 1e-8)
			{
				const int ax = dragAxisToIndex(section.dragAxis);
				h->applyMeshSectionPlaneRotationAxis(ax, deltaRad);
				h->notifyMeshSectionPlaneChanged();
			}
		}
		h->requestRedraw();
		return true;
	}

	if (event->type() == QEvent::MouseMove && !section.dragging && !section.rotating && !section.planeDragging)
	{
		auto* mouseEvent = static_cast<QMouseEvent*>(event);
		if (mouseEvent->buttons().testFlag(Qt::LeftButton) || mouseEvent->buttons().testFlag(Qt::MiddleButton) ||
			mouseEvent->buttons().testFlag(Qt::RightButton))
		{
			return false;
		}
		bool hoverRing = false;
		const int picked = h->pickMeshSectionPlaneAxisAtScreenPos(mouseEvent->pos(), true, &hoverRing);
		if (picked == OsgScene::kGizmoAxisNone)
		{
			bool ring2 = false;
			const int picked2 = h->pickMeshSectionPlaneAxisAtScreenPos(mouseEvent->pos(), false, &ring2);
			section.hoverAxis = static_cast<OsgScene::DragAxis>(picked2);
			hoverRing = false;
		}
		else
		{
			section.hoverAxis = static_cast<OsgScene::DragAxis>(picked);
		}
		h->updateMeshSectionPlaneCompassHighlight(section.hoverAxis, hoverRing);
		m_lastEmittedHoverAxis = picked;
		m_lastEmittedHoverRing = hoverRing;
		h->requestRedraw();
		return true;
	}

	if (event->type() == QEvent::MouseButtonRelease)
	{
		auto* mouseEvent = static_cast<QMouseEvent*>(event);
		const bool hadDrag = section.dragging || section.rotating || section.planeDragging;
		if (mouseEvent->button() == Qt::LeftButton || mouseEvent->button() == Qt::RightButton)
		{
			section.dragging = false;
			section.rotating = false;
			section.planeDragging = false;
			section.dragAxis = OsgScene::DragAxis::None;
			h->updateMeshSectionPlaneCompassHighlight(OsgScene::DragAxis::None);
			m_lastEmittedHoverAxis = -1;
			m_lastEmittedHoverRing = false;
		}
		h->requestRedraw();
		return hadDrag;
	}

	return false;
}
