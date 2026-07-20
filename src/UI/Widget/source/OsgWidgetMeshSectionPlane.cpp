/// @file OsgWidgetMeshSectionPlane.cpp
/// @brief OsgWidgetMeshSectionPlane 实现

#include "../../OsgWidgetCore/inc/OsgCompassGeometry.h"
#include "../../OsgWidgetCore/inc/OsgCompassRender.h"
#include "../../OsgWidgetCore/inc/OsgSectionPlaneGeometry.h"
#include "OsgWidget.h"

#include <algorithm>
#include <cmath>

#include <osg/Camera>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/Quat>

namespace
{
osg::Quat attitudeFromNormalAndAxisU(const osg::Vec3d& normalUnit, const osg::Vec3d& axisUUnit)
{
	const osg::Vec3d z = normalUnit;
	osg::Vec3d x = axisUUnit;
	x -= z * (x * z);
	const double xl = x.length();
	if (xl < 1e-12)
	{
		x = osg::Vec3d(1.0, 0.0, 0.0);
	}
	else
	{
		x /= xl;
	}
	const osg::Vec3d y = z ^ x;
	osg::Matrixd m;
	m(0, 0) = x.x();
	m(1, 0) = x.y();
	m(2, 0) = x.z();
	m(0, 1) = y.x();
	m(1, 1) = y.y();
	m(2, 1) = y.z();
	m(0, 2) = z.x();
	m(1, 2) = z.y();
	m(2, 2) = z.z();
	return m.getRotate();
}

void normalizeSectionAxes(osg::Vec3d& normal, osg::Vec3d& axisU)
{
	double nl = normal.length();
	if (nl < 1e-12)
	{
		normal.set(0.0, 0.0, 1.0);
	}
	else
	{
		normal /= nl;
	}
	axisU -= normal * (axisU * normal);
	const double ul = axisU.length();
	if (ul < 1e-12)
	{
		const osg::Vec3d ref = (std::abs(normal.z()) < 0.9) ? osg::Vec3d(0.0, 0.0, 1.0) : osg::Vec3d(0.0, 1.0, 0.0);
		axisU = ref ^ normal;
		axisU.normalize();
	}
	else
	{
		axisU /= ul;
	}
}

bool backendModelToWorld(const OsgWidget* self, const std::string& backendId, osg::Matrixd& outWorld)
{
	return self->getBackendRootWorldMatrix(backendId, outWorld);
}

bool worldPointToModel(const osg::Matrixd& modelToWorld, const osg::Vec3d& world, osg::Vec3d& outModel)
{
	osg::Matrixd inv = osg::Matrixd::inverse(modelToWorld);
	outModel = world * inv;
	return true;
}

osg::Vec3d modelPointToWorld(const osg::Matrixd& modelToWorld, const osg::Vec3d& model)
{
	return model * modelToWorld;
}

osg::Vec3d modelVectorToWorld(const osg::Matrixd& modelToWorld, const osg::Vec3d& modelVec)
{
	const osg::Vec3d w0 = osg::Vec3d(0.0, 0.0, 0.0) * modelToWorld;
	const osg::Vec3d w1 = modelVec * modelToWorld;
	osg::Vec3d w = w1 - w0;
	const double l = w.length();
	if (l < 1e-12)
	{
		return osg::Vec3d(0.0, 0.0, 1.0);
	}
	return w / l;
}

} // namespace

bool OsgWidget::isMeshSectionPlaneEditActive() const
{
	return m_sectionPlaneEditActive;
}

void OsgWidget::setMeshSectionPlaneCompassVisible(const bool visible)
{
	if (m_sectionPlaneCompassTransform.valid())
	{
		m_sectionPlaneCompassTransform->setNodeMask(visible ? ~0u : 0u);
	}
}

void OsgWidget::ensureMeshSectionPlaneOverlay(const std::string& backendIdUtf8)
{
	if (!m_sectionPlaneOverlayGroup.valid())
	{
		m_sectionPlaneOverlayGroup = new osg::Group;
		m_sectionPlaneOverlayGroup->setName("MeshSectionPlaneOverlay");
		if (m_backendObjectsGroup.valid())
		{
			m_backendObjectsGroup->addChild(m_sectionPlaneOverlayGroup.get());
		}
		else if (m_trajectoryOverlayGroup.valid())
		{
			m_trajectoryOverlayGroup->addChild(m_sectionPlaneOverlayGroup.get());
		}
	}

	if (!m_sectionPlaneWorldPat.valid())
	{
		m_sectionPlaneWorldPat = new osg::MatrixTransform;
		m_sectionPlaneWorldPat->setName("MeshSectionPlaneWorldPat");
		m_sectionPlaneOverlayGroup->addChild(m_sectionPlaneWorldPat.get());

		m_sectionPlaneQuadNode = osg_section_plane::buildSectionPlaneQuadNode(500.f);
		m_sectionPlaneWorldPat->addChild(m_sectionPlaneQuadNode.get());

		m_sectionPlaneCompassTransform = new osg::PositionAttitudeTransform;
		m_sectionPlaneCompassScaleTransform = new osg::MatrixTransform;
		osg_compass::applyUnlitHighlitStateSet(m_sectionPlaneCompassTransform->getOrCreateStateSet());
		osg_compass::applyUnlitHighlitStateSet(m_sectionPlaneCompassScaleTransform->getOrCreateStateSet());
		osg_compass::TransformCompassBranches branches;
		m_sectionPlaneCompassNode = osg_compass::buildTransformCompassNode(&branches);
		for (int i = 0; i < 3; ++i)
		{
			m_sectionPlaneAxisBranch[i] = branches.axis[i];
			m_sectionPlaneRingBranch[i] = branches.ring[i];
		}
		m_sectionPlaneCompassScaleTransform->addChild(m_sectionPlaneCompassNode.get());
		m_sectionPlaneCompassTransform->addChild(m_sectionPlaneCompassScaleTransform.get());
		m_sectionPlaneWorldPat->addChild(m_sectionPlaneCompassTransform.get());
	}

	if (m_sectionPlaneBackendId != backendIdUtf8)
	{
		m_sectionPlaneBackendId = backendIdUtf8;
		float diag = 1000.f;
		if (const auto it = m_backendObjectRoots.find(backendIdUtf8);
			it != m_backendObjectRoots.end() && it->second.valid())
		{
			(void)it;
		}
		m_sectionPlaneModelDiagonal = std::max(100.f, diag);
		m_sectionPlaneGizmoRefDistance = -1.0;
		m_sectionPlaneGizmoRefScale = 1.0;
	}
}

void OsgWidget::showMeshSectionPlane(const std::string& backendIdUtf8, const double originModelMm[3],
									 const double normalModel[3])
{
	ensureMeshSectionPlaneOverlay(backendIdUtf8);
	m_sectionPlaneVisible = true;
	m_sectionPlaneOriginModel.set(originModelMm[0], originModelMm[1], originModelMm[2]);
	m_sectionPlaneNormalModel.set(normalModel[0], normalModel[1], normalModel[2]);
	m_sectionPlaneAxisUModel = (std::abs(m_sectionPlaneNormalModel.z()) < 0.9)
								   ? osg::Vec3d(0.0, 0.0, 1.0) ^ m_sectionPlaneNormalModel
								   : osg::Vec3d(0.0, 1.0, 0.0) ^ m_sectionPlaneNormalModel;
	normalizeSectionAxes(m_sectionPlaneNormalModel, m_sectionPlaneAxisUModel);
	setMeshSectionPlaneCompassVisible(m_sectionPlaneEditActive);
	syncMeshSectionPlaneOverlayFromModel();
	if (m_sectionPlaneOverlayGroup.valid())
	{
		m_sectionPlaneOverlayGroup->setNodeMask(~0u);
	}
	requestRedraw();
}

void OsgWidget::beginMeshSectionPlaneEdit(const std::string& backendIdUtf8, const double originModelMm[3],
										  const double normalModel[3],
										  std::function<void(const double origin[3], const double normal[3])> onChanged)
{
	showMeshSectionPlane(backendIdUtf8, originModelMm, normalModel);
	m_sectionPlaneEditActive = true;
	m_sectionPlaneOnChanged = std::move(onChanged);
	setMeshSectionPlaneCompassVisible(true);
	requestRedraw();
}

void OsgWidget::updateMeshSectionPlanePose(const double originModelMm[3], const double normalModel[3])
{
	if (!m_sectionPlaneVisible)
	{
		return;
	}
	m_sectionPlaneOriginModel.set(originModelMm[0], originModelMm[1], originModelMm[2]);
	m_sectionPlaneNormalModel.set(normalModel[0], normalModel[1], normalModel[2]);
	normalizeSectionAxes(m_sectionPlaneNormalModel, m_sectionPlaneAxisUModel);
	syncMeshSectionPlaneOverlayFromModel();
	requestRedraw();
}

void OsgWidget::endMeshSectionPlaneEdit()
{
	m_sectionPlaneOnChanged = nullptr;
	m_sectionPlaneEditActive = false;
	m_sectionPlaneDragging = false;
	m_sectionPlaneRotating = false;
	m_sectionPlanePlaneDragging = false;
	setMeshSectionPlaneCompassVisible(false);
	if (m_viewer.valid())
	{
		requestRedraw();
	}
}

void OsgWidget::hideMeshSectionPlane()
{
	endMeshSectionPlaneEdit();
	m_sectionPlaneVisible = false;
	if (m_sectionPlaneOverlayGroup.valid())
	{
		m_sectionPlaneOverlayGroup->removeChildren(0, m_sectionPlaneOverlayGroup->getNumChildren());
	}
	m_sectionPlaneWorldPat = nullptr;
	m_sectionPlaneQuadNode = nullptr;
	m_sectionPlaneCompassTransform = nullptr;
	m_sectionPlaneCompassScaleTransform = nullptr;
	m_sectionPlaneCompassNode = nullptr;
	for (int i = 0; i < 3; ++i)
	{
		m_sectionPlaneAxisBranch[i] = nullptr;
		m_sectionPlaneRingBranch[i] = nullptr;
	}
	m_sectionPlaneBackendId.clear();
	if (m_viewer.valid())
	{
		requestRedraw();
	}
}

void OsgWidget::setMeshSectionPlanePreviewVisible(const bool visible)
{
	if (!m_sectionPlaneOverlayGroup.valid())
	{
		return;
	}
	m_sectionPlaneOverlayGroup->setNodeMask(visible ? ~0u : 0u);
	if (m_viewer.valid())
	{
		requestRedraw();
	}
}

void OsgWidget::notifyMeshSectionPlaneChanged()
{
	if (!m_sectionPlaneOnChanged)
	{
		return;
	}
	const double origin[3] = {m_sectionPlaneOriginModel.x(), m_sectionPlaneOriginModel.y(),
							  m_sectionPlaneOriginModel.z()};
	const double normal[3] = {m_sectionPlaneNormalModel.x(), m_sectionPlaneNormalModel.y(),
							  m_sectionPlaneNormalModel.z()};
	m_sectionPlaneOnChanged(origin, normal);
}

void OsgWidget::syncMeshSectionPlaneOverlayFromModel()
{
	if (!m_sectionPlaneVisible || !m_sectionPlaneWorldPat.valid())
	{
		return;
	}
	osg::Matrixd modelToWorld;
	if (!backendModelToWorld(this, m_sectionPlaneBackendId, modelToWorld))
	{
		modelToWorld.makeIdentity();
	}
	const osg::Vec3d originW = modelPointToWorld(modelToWorld, m_sectionPlaneOriginModel);
	const osg::Vec3d normalW = modelVectorToWorld(modelToWorld, m_sectionPlaneNormalModel);
	const osg::Vec3d axisUW = modelVectorToWorld(modelToWorld, m_sectionPlaneAxisUModel);
	const osg::Quat att = attitudeFromNormalAndAxisU(normalW, axisUW);
	m_sectionPlaneWorldPat->setMatrix(osg::Matrixd::translate(originW) * osg::Matrixd::rotate(att));
	updateMeshSectionPlaneCompassScale();
}

void OsgWidget::updateMeshSectionPlaneCompassScale()
{
	if (!m_sectionPlaneEditActive || !m_sectionPlaneCompassScaleTransform.valid() || !m_viewer.valid() ||
		!m_viewer->getCamera())
	{
		return;
	}
	osg::Vec3d pivot;
	computeMeshSectionPlanePivotWorld(pivot);
	osg::Vec3d eye;
	osg::Vec3d center;
	osg::Vec3d up;
	m_viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
	const double distance = (eye - pivot).length();
	if (m_sectionPlaneGizmoRefDistance < 0.0)
	{
		m_sectionPlaneGizmoRefDistance = std::max(1.0, distance);
		const double desiredAxisWorld =
			std::max(osg_compass::kCompassMinAxisWorld,
					 static_cast<double>(m_sectionPlaneModelDiagonal) * osg_compass::kCompassModelDiagonalFactor);
		m_sectionPlaneGizmoRefScale =
			std::max(0.4, std::min(800.0, desiredAxisWorld / static_cast<double>(osg_compass::kCompassAxisLength)));
	}
	double scale = m_sectionPlaneGizmoRefScale * (distance / m_sectionPlaneGizmoRefDistance);
	scale = std::max(0.3, std::min(1200.0, scale));
	m_sectionPlaneCompassScaleTransform->setMatrix(osg::Matrixd::scale(scale, scale, scale));
}

void OsgWidget::updateMeshSectionPlaneCompassHighlight(const DragAxis axis, const bool highlightRing)
{
	auto scaleBranch = [](const osg::ref_ptr<osg::MatrixTransform>& branch, const float s)
	{
		if (branch.valid())
		{
			branch->setMatrix(osg::Matrixd::scale(s, s, s));
		}
	};
	const float normal = 1.0f;
	const float hot = 1.35f;
	for (int i = 0; i < 3; ++i)
	{
		const DragAxis da = static_cast<DragAxis>(kGizmoAxisX + i);
		const bool axisHot = !highlightRing && axis == da;
		const bool ringHot = highlightRing && axis == da;
		scaleBranch(m_sectionPlaneAxisBranch[i], axisHot ? hot : normal);
		scaleBranch(m_sectionPlaneRingBranch[i], ringHot ? hot : normal);
	}
}

void OsgWidget::computeMeshSectionPlanePivotWorld(osg::Vec3d& outPivotWorld) const
{
	osg::Matrixd modelToWorld;
	if (!backendModelToWorld(this, m_sectionPlaneBackendId, modelToWorld))
	{
		modelToWorld.makeIdentity();
	}
	outPivotWorld = modelPointToWorld(modelToWorld, m_sectionPlaneOriginModel);
}

bool OsgWidget::meshSectionPlaneCompassUnitAxisWorld(const DragAxis axis, osg::Vec3d& outAxisWorld) const
{
	osg::Matrixd modelToWorld;
	if (!backendModelToWorld(this, m_sectionPlaneBackendId, modelToWorld))
	{
		modelToWorld.makeIdentity();
	}
	osg::Vec3d local(0.0, 0.0, 1.0);
	if (axis == DragAxis::X)
	{
		local = m_sectionPlaneAxisUModel;
	}
	else if (axis == DragAxis::Y)
	{
		local = m_sectionPlaneNormalModel ^ m_sectionPlaneAxisUModel;
	}
	else if (axis == DragAxis::Z)
	{
		local = m_sectionPlaneNormalModel;
	}
	outAxisWorld = modelVectorToWorld(modelToWorld, local);
	return true;
}

bool OsgWidget::beginMeshSectionPlaneScreenDrag()
{
	m_sectionPlaneDragScreenAxisUx = 1.0;
	m_sectionPlaneDragScreenAxisUy = 0.0;
	m_sectionPlaneDragMmPerPixel = 1.0;
	if (!m_sectionPlaneEditActive || !m_viewer.valid() || !m_viewer->getCamera() || viewportWidth() <= 0 ||
		viewportHeight() <= 0)
	{
		return false;
	}
	osg::Vec3d axisW;
	if (!meshSectionPlaneCompassUnitAxisWorld(m_sectionPlaneDragAxis, axisW))
	{
		return false;
	}
	m_sectionPlaneScreenDragAxisWorld = axisW;

	float gizmoScale = 1.0f;
	if (m_sectionPlaneCompassScaleTransform.valid())
	{
		const osg::Matrixd& sm = m_sectionPlaneCompassScaleTransform->getMatrix();
		gizmoScale = static_cast<float>(std::max({std::abs(sm(0, 0)), std::abs(sm(1, 1)), std::abs(sm(2, 2))}));
		if (gizmoScale < 1e-6f)
		{
			gizmoScale = 1.0f;
		}
	}
	const float axisLenMm = osg_compass::kCompassAxisLength * gizmoScale;

	osg::Vec3d pivot;
	computeMeshSectionPlanePivotWorld(pivot);
	const osg::Vec3d tipWorld(pivot.x() + axisW.x() * static_cast<double>(axisLenMm),
							  pivot.y() + axisW.y() * static_cast<double>(axisLenMm),
							  pivot.z() + axisW.z() * static_cast<double>(axisLenMm));

	osg::Camera* const camera = m_viewer->getCamera();
	const osg::Matrixd mvp = camera->getViewMatrix() * camera->getProjectionMatrix();
	auto projectToScreen = [&](const osg::Vec3d& world, double& sx, double& sy)
	{
		const osg::Vec3d clip = world * mvp;
		sx = (clip.x() * 0.5 + 0.5) * static_cast<double>(viewportWidth());
		sy = (1.0 - (clip.y() * 0.5 + 0.5)) * static_cast<double>(viewportHeight());
	};

	double ox = 0.0;
	double oy = 0.0;
	double tx = 0.0;
	double ty = 0.0;
	projectToScreen(pivot, ox, oy);
	projectToScreen(tipWorld, tx, ty);
	const double vx = tx - ox;
	const double vy = ty - oy;
	const double lenPx = std::hypot(vx, vy);
	if (lenPx < 1e-3)
	{
		return false;
	}
	m_sectionPlaneDragScreenAxisUx = vx / lenPx;
	m_sectionPlaneDragScreenAxisUy = vy / lenPx;
	m_sectionPlaneDragMmPerPixel = static_cast<double>(axisLenMm) / lenPx;
	return true;
}

double OsgWidget::meshSectionPlaneScreenDragDsMm(const QPoint& curPos, const QPoint& lastPos) const
{
	const double dx = static_cast<double>(curPos.x()) - static_cast<double>(lastPos.x());
	const double dy = static_cast<double>(curPos.y()) - static_cast<double>(lastPos.y());
	const double dsPx = dx * m_sectionPlaneDragScreenAxisUx + dy * m_sectionPlaneDragScreenAxisUy;
	return dsPx * m_sectionPlaneDragMmPerPixel;
}

void OsgWidget::applyMeshSectionPlaneTranslationAxis(const int axisIndex, const double dsWorld)
{
	(void)axisIndex;
	const osg::Vec3d axisW = m_sectionPlaneScreenDragAxisWorld;
	osg::Matrixd modelToWorld;
	if (!backendModelToWorld(this, m_sectionPlaneBackendId, modelToWorld))
	{
		modelToWorld.makeIdentity();
	}
	osg::Matrixd worldToModel = osg::Matrixd::inverse(modelToWorld);
	const osg::Vec3d deltaW = axisW * dsWorld;
	osg::Vec3d deltaM = deltaW * worldToModel;
	const osg::Vec3d o0(0.0, 0.0, 0.0);
	const osg::Vec3d o1 = deltaM;
	const osg::Vec3d dm = o1 - o0;
	m_sectionPlaneOriginModel += dm;
	syncMeshSectionPlaneOverlayFromModel();
}

void OsgWidget::applyMeshSectionPlaneTranslationWorld(const osg::Vec3d& hitWorld, const osg::Vec3d& lastHitWorld)
{
	osg::Matrixd modelToWorld;
	if (!backendModelToWorld(this, m_sectionPlaneBackendId, modelToWorld))
	{
		modelToWorld.makeIdentity();
	}
	const osg::Matrixd worldToModel = osg::Matrixd::inverse(modelToWorld);
	const osg::Vec3d m0 = lastHitWorld * worldToModel;
	const osg::Vec3d m1 = hitWorld * worldToModel;
	m_sectionPlaneOriginModel += (m1 - m0);
	syncMeshSectionPlaneOverlayFromModel();
}

void OsgWidget::applyMeshSectionPlaneRotationAxis(const int axisIndex, const double deltaRad)
{
	osg::Vec3d axisW;
	const DragAxis da = static_cast<DragAxis>(kGizmoAxisX + axisIndex);
	(void)meshSectionPlaneCompassUnitAxisWorld(da, axisW);
	osg::Matrixd modelToWorld;
	if (!backendModelToWorld(this, m_sectionPlaneBackendId, modelToWorld))
	{
		modelToWorld.makeIdentity();
	}
	osg::Matrixd worldToModel = osg::Matrixd::inverse(modelToWorld);
	const osg::Vec3d axisM = (axisW * worldToModel) - (osg::Vec3d(0.0, 0.0, 0.0) * worldToModel);
	osg::Vec3d axisModel = axisM;
	const double al = axisModel.length();
	if (al < 1e-12)
	{
		return;
	}
	axisModel /= al;
	const osg::Quat q(deltaRad, axisModel);
	m_sectionPlaneNormalModel = q * m_sectionPlaneNormalModel;
	m_sectionPlaneAxisUModel = q * m_sectionPlaneAxisUModel;
	normalizeSectionAxes(m_sectionPlaneNormalModel, m_sectionPlaneAxisUModel);
	syncMeshSectionPlaneOverlayFromModel();
}

bool OsgWidget::pickMeshSectionPlaneDragPoint(const QPoint& mousePos, osg::Vec3d& outHitWorld) const
{
	if (!m_sectionPlaneEditActive)
	{
		return false;
	}
	osg::Vec3d eye;
	osg::Vec3d dir;
	if (!computeCameraScreenRayWorld(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()), eye, dir))
	{
		return false;
	}
	osg::Vec3d pivot;
	computeMeshSectionPlanePivotWorld(pivot);
	osg::Vec3d normalW;
	(void)meshSectionPlaneCompassUnitAxisWorld(DragAxis::Z, normalW);
	const double denom = dir * normalW;
	if (std::abs(denom) < 1e-10)
	{
		return false;
	}
	const double t = ((pivot - eye) * normalW) / denom;
	if (t < 0.0)
	{
		return false;
	}
	outHitWorld = eye + dir * t;
	return true;
}

int OsgWidget::pickMeshSectionPlaneAxisAtScreenPos(const QPoint& mousePos, const bool preferRing,
												   bool* outPickedRing) const
{
	if (outPickedRing)
	{
		*outPickedRing = false;
	}
	if (!m_sectionPlaneEditActive || !m_viewer.valid() || !m_viewer->getCamera() || viewportWidth() <= 0 ||
		viewportHeight() <= 0)
	{
		return kGizmoAxisNone;
	}
	const double mx = static_cast<double>(mousePos.x());
	const double my = static_cast<double>(mousePos.y());

	float gizmoScale = 1.0f;
	if (m_sectionPlaneCompassScaleTransform.valid())
	{
		const osg::Matrixd& sm = m_sectionPlaneCompassScaleTransform->getMatrix();
		gizmoScale = static_cast<float>(std::max({std::abs(sm(0, 0)), std::abs(sm(1, 1)), std::abs(sm(2, 2))}));
		if (gizmoScale < 1e-6f)
		{
			gizmoScale = 1.0f;
		}
	}
	const float axisLen = osg_compass::kCompassAxisLength * gizmoScale;
	const float ringRadius = 65.0f * osg_compass::kCompassGeomScale * gizmoScale;

	osg::Camera* camera = m_viewer->getCamera();
	const osg::Matrixd mvp = camera->getViewMatrix() * camera->getProjectionMatrix();
	osg::Vec3d origin;
	computeMeshSectionPlanePivotWorld(origin);

	auto axisTipWorld = [&](const DragAxis axis) -> osg::Vec3f
	{
		osg::Vec3d axisW;
		if (!meshSectionPlaneCompassUnitAxisWorld(axis, axisW))
		{
			return osg::Vec3f(static_cast<float>(origin.x()), static_cast<float>(origin.y()),
							  static_cast<float>(origin.z()));
		}
		return osg::Vec3f(static_cast<float>(origin.x() + axisW.x() * static_cast<double>(axisLen)),
						  static_cast<float>(origin.y() + axisW.y() * static_cast<double>(axisLen)),
						  static_cast<float>(origin.z() + axisW.z() * static_cast<double>(axisLen)));
	};

	auto projectToScreen = [&](const osg::Vec3f& world, double& sx, double& sy)
	{
		const osg::Vec3d clip = osg::Vec3d(world) * mvp;
		sx = (clip.x() * 0.5 + 0.5) * static_cast<double>(viewportWidth());
		sy = (1.0 - (clip.y() * 0.5 + 0.5)) * static_cast<double>(viewportHeight());
	};

	double ox = 0;
	double oy = 0;
	double pxx = 0;
	double pxy = 0;
	double pyx = 0;
	double pyy = 0;
	double pzx = 0;
	double pzy = 0;
	projectToScreen(
		osg::Vec3f(static_cast<float>(origin.x()), static_cast<float>(origin.y()), static_cast<float>(origin.z())), ox,
		oy);
	projectToScreen(axisTipWorld(DragAxis::X), pxx, pxy);
	projectToScreen(axisTipWorld(DragAxis::Y), pyx, pyy);
	projectToScreen(axisTipWorld(DragAxis::Z), pzx, pzy);

	auto distanceToSegment = [](double p0x, double p0y, double p1x, double p1y, double qx, double qy) -> double
	{
		const double vx = p1x - p0x;
		const double vy = p1y - p0y;
		const double wx = qx - p0x;
		const double wy = qy - p0y;
		const double len2 = vx * vx + vy * vy;
		if (len2 <= 1e-6)
		{
			return std::hypot(qx - p0x, qy - p0y);
		}
		const double t = std::max(0.0, std::min(1.0, (wx * vx + wy * vy) / len2));
		return std::hypot(qx - (p0x + vx * t), qy - (p0y + vy * t));
	};

	const double dx = distanceToSegment(ox, oy, pxx, pxy, mx, my);
	const double dy = distanceToSegment(ox, oy, pyx, pyy, mx, my);
	const double dz = distanceToSegment(ox, oy, pzx, pzy, mx, my);
	const double axisLenPx =
		std::max({std::hypot(pxx - ox, pxy - oy), std::hypot(pyx - ox, pyy - oy), std::hypot(pzx - ox, pzy - oy), 1.0});
	const double threshold = std::clamp(0.22 * axisLenPx, 14.0, 44.0);
	const double ringThreshold = std::clamp(0.14 * axisLenPx, 10.0, 36.0);

	osg::Vec3d axisDirW[3];
	const DragAxis dragAxes[3] = {DragAxis::X, DragAxis::Y, DragAxis::Z};
	for (int ai = 0; ai < 3; ++ai)
	{
		axisDirW[ai].set(0.0, 0.0, 1.0);
		(void)meshSectionPlaneCompassUnitAxisWorld(dragAxes[ai], axisDirW[ai]);
	}
	auto ringPointWorld = [&](const int ringAxis, const float ca, const float sa) -> osg::Vec3f
	{
		const double rr = static_cast<double>(ringRadius);
		osg::Vec3d w = origin;
		if (ringAxis == kGizmoAxisX)
		{
			w += axisDirW[1] * (static_cast<double>(ca) * rr) + axisDirW[2] * (static_cast<double>(sa) * rr);
		}
		else if (ringAxis == kGizmoAxisY)
		{
			w += axisDirW[0] * (static_cast<double>(ca) * rr) + axisDirW[2] * (static_cast<double>(sa) * rr);
		}
		else
		{
			w += axisDirW[0] * (static_cast<double>(ca) * rr) + axisDirW[1] * (static_cast<double>(sa) * rr);
		}
		return osg::Vec3f(static_cast<float>(w.x()), static_cast<float>(w.y()), static_cast<float>(w.z()));
	};

	auto minDistanceToProjectedRing = [&](int axis) -> double
	{
		const int segments = 72;
		const float r = ringRadius;
		double minDist = 1e9;
		for (int i = 0; i < segments; ++i)
		{
			const float a0 = osg::PI * 2.0f * static_cast<float>(i) / static_cast<float>(segments);
			const float a1 = osg::PI * 2.0f * static_cast<float>(i + 1) / static_cast<float>(segments);
			osg::Vec3f w0;
			osg::Vec3f w1;
			if (axis == kGizmoAxisX)
			{
				w0 = ringPointWorld(kGizmoAxisX, std::cos(a0) * r, std::sin(a0) * r);
				w1 = ringPointWorld(kGizmoAxisX, std::cos(a1) * r, std::sin(a1) * r);
			}
			else if (axis == kGizmoAxisY)
			{
				w0 = ringPointWorld(kGizmoAxisY, std::cos(a0) * r, std::sin(a0) * r);
				w1 = ringPointWorld(kGizmoAxisY, std::cos(a1) * r, std::sin(a1) * r);
			}
			else
			{
				w0 = ringPointWorld(kGizmoAxisZ, std::cos(a0) * r, std::sin(a0) * r);
				w1 = ringPointWorld(kGizmoAxisZ, std::cos(a1) * r, std::sin(a1) * r);
			}
			double s0x = 0;
			double s0y = 0;
			double s1x = 0;
			double s1y = 0;
			projectToScreen(w0, s0x, s0y);
			projectToScreen(w1, s1x, s1y);
			minDist = std::min(minDist, distanceToSegment(s0x, s0y, s1x, s1y, mx, my));
		}
		return minDist;
	};

	if (preferRing)
	{
		const double drx = minDistanceToProjectedRing(kGizmoAxisX);
		const double dry = minDistanceToProjectedRing(kGizmoAxisY);
		const double drz = minDistanceToProjectedRing(kGizmoAxisZ);
		double best = ringThreshold;
		int ringAxis = kGizmoAxisNone;
		if (drx < best)
		{
			best = drx;
			ringAxis = kGizmoAxisX;
		}
		if (dry < best)
		{
			best = dry;
			ringAxis = kGizmoAxisY;
		}
		if (drz < best)
		{
			best = drz;
			ringAxis = kGizmoAxisZ;
		}
		if (ringAxis != kGizmoAxisNone)
		{
			if (outPickedRing)
			{
				*outPickedRing = true;
			}
			return ringAxis;
		}
	}

	double minDist = threshold;
	int axis = kGizmoAxisNone;
	if (dx < minDist)
	{
		minDist = dx;
		axis = kGizmoAxisX;
	}
	if (dy < minDist)
	{
		minDist = dy;
		axis = kGizmoAxisY;
	}
	if (dz < minDist)
	{
		axis = kGizmoAxisZ;
	}
	return axis;
}

bool OsgWidget::getCameraViewDirectionWorld(double outDirUnit[3]) const
{
	if (!m_viewer.valid() || !m_viewer->getCamera())
	{
		return false;
	}
	osg::Vec3d eye;
	osg::Vec3d center;
	osg::Vec3d up;
	m_viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
	osg::Vec3d dir = center - eye;
	const double l = dir.length();
	if (l < 1e-12)
	{
		return false;
	}
	dir /= l;
	outDirUnit[0] = dir.x();
	outDirUnit[1] = dir.y();
	outDirUnit[2] = dir.z();
	return true;
}

bool OsgWidget::getCameraViewDirectionInBackendModel(const std::string& backendIdUtf8, double outDirModel[3]) const
{
	double dirW[3] = {};
	if (!getCameraViewDirectionWorld(dirW))
	{
		return false;
	}
	osg::Matrixd modelToWorld;
	if (!getBackendRootWorldMatrix(backendIdUtf8, modelToWorld))
	{
		outDirModel[0] = dirW[0];
		outDirModel[1] = dirW[1];
		outDirModel[2] = dirW[2];
		return true;
	}
	const osg::Matrixd worldToModel = osg::Matrixd::inverse(modelToWorld);
	const osg::Vec3d w0(0.0, 0.0, 0.0);
	const osg::Vec3d w1(dirW[0], dirW[1], dirW[2]);
	const osg::Vec3d m0 = w0 * worldToModel;
	const osg::Vec3d m1 = w1 * worldToModel;
	osg::Vec3d dm = m1 - m0;
	const double l = dm.length();
	if (l < 1e-12)
	{
		return false;
	}
	dm /= l;
	outDirModel[0] = dm.x();
	outDirModel[1] = dm.y();
	outDirModel[2] = dm.z();
	return true;
}
