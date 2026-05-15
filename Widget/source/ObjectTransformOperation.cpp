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
}

bool cacheRotatePivotInParentSpace(OsgWidget* o)
{
	if (!o->m_activeBackendOuterPat.valid())
	{
		return false;
	}
	osg::Vec3f pivotF;
	o->computeGizmoPivotWorld(pivotF);
	const osg::Vec3d pivotW(static_cast<double>(pivotF.x()), static_cast<double>(pivotF.y()),
		static_cast<double>(pivotF.z()));
	o->m_gizmoRotatePivotWorld = pivotW;
	osg::NodePath pathOuter;
	for (osg::Node* n = o->m_activeBackendOuterPat.get(); n != nullptr;
		n = n->getNumParents() > 0 ? n->getParent(0) : nullptr)
	{
		pathOuter.insert(pathOuter.begin(), n);
	}
	osg::Matrixd parentWorld;
	if (pathOuter.size() >= 2U)
	{
		osg::NodePath parentPath;
		parentPath.reserve(pathOuter.size() - 1U);
		for (unsigned i = 0; i + 1U < pathOuter.size(); ++i)
		{
			parentPath.push_back(pathOuter[i]);
		}
		parentWorld = osg::computeLocalToWorld(parentPath);
	}
	else
	{
		parentWorld.makeIdentity();
	}
	o->m_gizmoRotatePivotInParent = pivotW * osg::Matrixd::inverse(parentWorld);
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
	if (!m_owner || watched != m_owner->m_glWidget || !m_owner->m_objectSelectionMode || !m_owner->m_activeBackendOuterPat.valid())
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
				beginGizmoDragSession();
				m_owner->m_dragging = true;
				m_owner->m_rotating = false;
				m_owner->m_lastMousePos = mouseEvent->pos();
				if (!tryBeginTranslatePlane(m_owner, mouseEvent->pos()))
				{
					(void)seedTranslateDragOnAxisLine(m_owner, mouseEvent->pos());
				}
				m_owner->updateCompassHighlight(m_owner->m_dragAxis, false);
				emit m_owner->activeAxisChanged(m_owner->axisToString(m_owner->m_dragAxis));
				m_owner->requestRedraw();
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
			beginGizmoDragSession();
			m_owner->m_rotating = true;
			m_owner->m_dragging = false;
			m_owner->m_lastMousePos = mouseEvent->pos();
			tryBeginRotateHit(m_owner, mouseEvent->pos());
			m_owner->updateCompassHighlight(m_owner->m_dragAxis, true);
			emit m_owner->activeAxisChanged(m_owner->axisToString(m_owner->m_dragAxis));
			m_owner->requestRedraw();
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
			osg::Vec3d pivotCur;
			currentGizmoPivotWorldD(m_owner, pivotCur);
			const bool hitOk = rayOk && m_owner->m_gizmoTransDragPlaneActive
				&& rayPlaneIntersect(eye, dir, pivotCur, m_owner->m_gizmoTransDragPlaneN, qNew);
			if (hitOk)
			{
				dsWorld = (qNew - m_owner->m_gizmoDragLastHitWorld) * axisW;
				dsWorld *= kGizmoTranslatePlaneGain;
				m_owner->m_gizmoDragLastHitWorld = qNew;
			}
			else if (m_owner->m_gizmoTransDragPlaneActive && rayOk)
			{
				osg::Vec3d lineHit;
				const double maxLineT = maxGizmoTranslateStepWorld(m_owner) * 4.0;
				if (rayClosestPointOnLine(eye, dir, pivotCur, axisW, lineHit, maxLineT))
				{
					dsWorld = (lineHit - m_owner->m_gizmoDragLastHitWorld) * axisW;
					dsWorld *= kGizmoTranslatePlaneGain;
					m_owner->m_gizmoDragLastHitWorld = lineHit;
				}
			}
			dsWorld = clampGizmoTranslateDsWorld(dsWorld, m_owner);
			m_owner->m_lastMousePos = pos;

			ObjectGizmoFrame f;
			osg::MatrixTransform* const outer = m_owner->m_activeBackendOuterPat.get();
			if (outer && m_owner->readActiveObjectGizmoFrame(f) && std::abs(dsWorld) > 1e-10)
			{
				if (m_owner->transformGizmoFrame() == OsgWidget::TransformGizmoFrame::World)
				{
					f.translateAlongWorldAxis(outer, dragAxisToIndex(m_owner->m_dragAxis), dsWorld);
				}
				else
				{
					f.translateAlongBodyAxis(outer, dragAxisToIndex(m_owner->m_dragAxis), dsWorld);
				}
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
			const osg::Vec3d pivot = m_owner->m_gizmoRotatePivotActive
				? m_owner->m_gizmoRotatePivotWorld
				: [&]() {
					osg::Vec3f pivotF;
					m_owner->computeGizmoPivotWorld(pivotF);
					return osg::Vec3d(static_cast<double>(pivotF.x()), static_cast<double>(pivotF.y()),
						static_cast<double>(pivotF.z()));
				}();
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

			if (std::abs(deltaRad) <= 1e-8)
			{
				return true;
			}

			ObjectGizmoFrame f;
			if (m_owner->readActiveObjectGizmoFrame(f))
			{
				const osg::Quat R_old = f.attitude();
				osg::Quat R_new;
				if (m_owner->transformGizmoFrame() == OsgWidget::TransformGizmoFrame::World)
				{
					osg::Vec3 axis(0.0f, 0.0f, 1.0f);
					if (m_owner->m_dragAxis == OsgWidget::DragAxis::X) axis.set(1.0f, 0.0f, 0.0f);
					if (m_owner->m_dragAxis == OsgWidget::DragAxis::Y) axis.set(0.0f, 1.0f, 0.0f);
					if (m_owner->m_dragAxis == OsgWidget::DragAxis::Z) axis.set(0.0f, 0.0f, 1.0f);
					const osg::Quat deltaQuat(static_cast<float>(deltaRad), axis);
					R_new = deltaQuat * R_old;
				}
				else
				{
					const osg::Vec3f axf = gizmoLocalAxis(m_owner->m_dragAxis);
					const osg::Quat deltaQuat(static_cast<float>(deltaRad), osg::Vec3(axf.x(), axf.y(), axf.z()));
					R_new = R_old * deltaQuat;
				}
				if (m_owner->m_gizmoRotatePivotActive)
				{
					f.setRotationKeepingPivotInOuterParent(m_owner->m_gizmoRotatePivotInParent, R_new);
				}
				else
				{
					f.adjustCenterPlusPoseForRotationDelta(R_old, R_new);
				}
				osg::MatrixTransform* const outerRot = m_owner->m_activeBackendOuterPat.get();
				if (outerRot)
				{
					f.applyToOuter(outerRot);
					(void)ObjectGizmoFrame::fromOuter(outerRot, f.modelCenter(), f);
				}
				markGizmoSessionModified();
				m_owner->syncActiveBackendRootFromObjectFrame(f, true);
				m_owner->syncCompassGizmoOrientation();

				const osg::Vec3f pose = f.backendPoseRelativeToCenter();
				emit m_owner->selectedObjectPoseChanged(pose.x(), pose.y(), pose.z());
				const osg::Vec3f euler = m_owner->quatToEulerDeg(f.attitude());
				emit m_owner->selectedObjectRotationChanged(euler.x(), euler.y(), euler.z());
				m_owner->requestRedraw();
			}
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
