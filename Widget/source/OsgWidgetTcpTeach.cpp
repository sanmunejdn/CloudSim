#include "OsgWidget.h"
#include "RobotTcpDragTeachOperation.h"

#include <RigidTransform.h>
#include <Adapters.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>

#include <osg/BlendFunc>
#include <osg/Camera>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/LineWidth>
#include <osg/MatrixTransform>
#include <osg/PolygonOffset>
#include <osg/PositionAttitudeTransform>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/StateSet>

namespace
{
// #region agent log
void agentDbgLog4ce7a0(const char* hypothesisId, const char* message, const std::string& dataJson)
{
	using clock = std::chrono::system_clock;
	const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(clock::now().time_since_epoch()).count();
	std::ofstream out(R"(d:\Project\VSprogram\CGAL5.5.2\debug-4ce7a0.log)", std::ios::app);
	if (!out)
	{
		return;
	}
	out << "{\"sessionId\":\"4ce7a0\",\"runId\":\"screen-drag\",\"hypothesisId\":\"" << hypothesisId
	    << "\",\"location\":\"OsgWidgetTcpTeach.cpp\",\"message\":\"" << message << "\",\"data\":"
	    << (dataJson.empty() ? "{}" : dataJson) << ",\"timestamp\":" << ms << "}\n";
}
// #endregion

osg::Quat rigidRotationToOsgQuat(const engine::RigidTransform& rt)
{
	const Eigen::Quaterniond q = rt.rotation().normalized();
	return osg::Quat(q.x(), q.y(), q.z(), q.w());
}

osg::Node* buildTcpTeachCompassGeometry(
	osg::ref_ptr<osg::MatrixTransform> axisBranch[3],
	osg::ref_ptr<osg::MatrixTransform> ringBranch[3])
{
	for (int i = 0; i < 3; ++i)
	{
		axisBranch[i] = nullptr;
		ringBranch[i] = nullptr;
	}
	const float axisLen = 120.0f;
	const float coneH = 20.0f;
	const float coneR = 7.0f;
	const float tipExtension = 6.0f;
	const float ringRadius = 65.0f;
	const float tubeR = 2.85f;
	const int ringSegments = 36;

	auto applyCompassStateSet = [](osg::StateSet* ss) {
		ss->setAttributeAndModes(new osg::PolygonOffset(-1.0f, -1.0f), osg::StateAttribute::ON);
		ss->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON);
		ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
		ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
		ss->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
		osg::ref_ptr<osg::Depth> depth = new osg::Depth;
		depth->setFunction(osg::Depth::ALWAYS);
		depth->setWriteMask(false);
		ss->setAttributeAndModes(depth.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
		ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	};

	auto addPositiveAxis = [&](const osg::Vec3& p1, const osg::Vec3& coneDir, const osg::Vec4& col) -> osg::ref_ptr<osg::Geode> {
		osg::ref_ptr<osg::Geode> g = new osg::Geode;
		osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
		v->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
		v->push_back(p1);
		osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
		ca->push_back(col);
		osg::ref_ptr<osg::Geometry> lineGeom = new osg::Geometry;
		lineGeom->setVertexArray(v.get());
		lineGeom->setColorArray(ca.get(), osg::Array::BIND_OVERALL);
		lineGeom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));
		g->addDrawable(lineGeom.get());
		g->getOrCreateStateSet()->setAttribute(new osg::LineWidth(6.0f));

		const osg::Vec3 tip = coneDir * (axisLen + tipExtension);
		const osg::Vec3 coneCenter = tip - coneDir * (coneH * 0.5f);
		osg::ref_ptr<osg::Cone> cone = new osg::Cone(osg::Vec3(0.0f, 0.0f, 0.0f), coneR, coneH);
		osg::Quat rot;
		rot.makeRotate(osg::Vec3(0.0f, 0.0f, 1.0f), coneDir);
		cone->setRotation(rot);
		cone->setCenter(coneCenter);
		osg::ref_ptr<osg::ShapeDrawable> coneDraw = new osg::ShapeDrawable(cone.get());
		coneDraw->setColor(col);
		g->addDrawable(coneDraw.get());
		applyCompassStateSet(g->getOrCreateStateSet());
		return g;
	};

	osg::ref_ptr<osg::Group> root = new osg::Group;
	root->setName("TcpTeachCompass");

	const osg::Vec4 colX(0.92f, 0.22f, 0.22f, 0.95f);
	const osg::Vec4 colY(0.22f, 0.85f, 0.28f, 0.95f);
	const osg::Vec4 colZ(0.25f, 0.45f, 0.95f, 0.95f);

	auto wrapBranch = [&](osg::Node* child) -> osg::ref_ptr<osg::MatrixTransform> {
		osg::ref_ptr<osg::MatrixTransform> br = new osg::MatrixTransform;
		br->addChild(child);
		return br;
	};

	axisBranch[0] = wrapBranch(addPositiveAxis(osg::Vec3(axisLen, 0.0f, 0.0f), osg::Vec3(1.0f, 0.0f, 0.0f), colX).get());
	axisBranch[1] = wrapBranch(addPositiveAxis(osg::Vec3(0.0f, axisLen, 0.0f), osg::Vec3(0.0f, 1.0f, 0.0f), colY).get());
	axisBranch[2] = wrapBranch(addPositiveAxis(osg::Vec3(0.0f, 0.0f, axisLen), osg::Vec3(0.0f, 0.0f, 1.0f), colZ).get());
	root->addChild(axisBranch[0].get());
	root->addChild(axisBranch[1].get());
	root->addChild(axisBranch[2].get());

	auto addRing = [&](int axisIdx, const osg::Vec4& col) {
		osg::ref_ptr<osg::Geode> g = new osg::Geode;
		osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
		for (int i = 0; i <= ringSegments; ++i)
		{
			const float a = osg::PI * 2.0f * static_cast<float>(i) / static_cast<float>(ringSegments);
			osg::Vec3 p;
			if (axisIdx == 0)
			{
				p.set(0.0f, std::cos(a) * ringRadius, std::sin(a) * ringRadius);
			}
			else if (axisIdx == 1)
			{
				p.set(std::cos(a) * ringRadius, 0.0f, std::sin(a) * ringRadius);
			}
			else
			{
				p.set(std::cos(a) * ringRadius, std::sin(a) * ringRadius, 0.0f);
			}
			v->push_back(p);
		}
		osg::ref_ptr<osg::Vec4Array> ca = new osg::Vec4Array;
		ca->push_back(col);
		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
		geom->setVertexArray(v.get());
		geom->setColorArray(ca.get(), osg::Array::BIND_OVERALL);
		geom->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, v->size()));
		geom->getOrCreateStateSet()->setAttribute(new osg::LineWidth(4.0f));
		applyCompassStateSet(geom->getOrCreateStateSet());
		g->addDrawable(geom.get());
		ringBranch[axisIdx] = wrapBranch(g.get());
		root->addChild(ringBranch[axisIdx].get());
	};
	addRing(0, colX);
	addRing(1, colY);
	addRing(2, colZ);

	return root.release();
}
} // namespace

bool OsgWidget::tcpTeachResolveBaseWorld(osg::Matrixd& outBaseWorld) const
{
	if (m_tcpTeachMountBackendId.empty())
	{
		return false;
	}
	if (m_tcpTeachResolveRobotBaseWorld)
	{
		return m_tcpTeachResolveRobotBaseWorld(outBaseWorld);
	}
	return getBackendRootWorldMatrix(m_tcpTeachMountBackendId, outBaseWorld);
}

bool OsgWidget::tcpTeachToolWorldMatrix(osg::Matrixd& outToolWorld) const
{
	osg::Matrixd baseWorld;
	if (!tcpTeachResolveBaseWorld(baseWorld))
	{
		return false;
	}
	outToolWorld = engine::osgMatrixFromRigidTransform(m_tcpTeachTargetInBase) * baseWorld;
	return true;
}

void OsgWidget::tcpTeachSetTargetFromToolWorld(const osg::Matrixd& toolWorld)
{
	osg::Matrixd baseWorld;
	if (!tcpTeachResolveBaseWorld(baseWorld))
	{
		return;
	}
	const osg::Matrixd toolInBase = toolWorld * osg::Matrixd::inverse(baseWorld);
	m_tcpTeachTargetInBase = engine::rigidTransformFromOsg(toolInBase);
}

void OsgWidget::beginTcpDragTeach(
	const std::string& mountBackendId,
	const engine::RigidTransform& T_base_target,
	const float modelDiagonalMm,
	std::function<bool(osg::Matrixd&)> resolveRobotBaseWorld,
	const osg::Matrixd* toolLocalOnFlange)
{
	endTcpDragTeach();
	m_tcpTeachActive = true;
	m_tcpTeachMountBackendId = mountBackendId;
	m_tcpTeachResolveRobotBaseWorld = std::move(resolveRobotBaseWorld);
	m_tcpTeachUseFlangeLocalPlacement = (toolLocalOnFlange != nullptr);
	m_tcpTeachToolLocalOnFlange.makeIdentity();
	if (toolLocalOnFlange)
	{
		m_tcpTeachToolLocalOnFlange = *toolLocalOnFlange;
	}
	m_tcpTeachTargetInBase = T_base_target;
	m_tcpTeachModelDiagonal = std::max(100.0f, modelDiagonalMm);
	m_tcpTeachGizmoRefDistance = -1.0;
	m_tcpTeachGizmoRefScale = 1.0;

	auto it = m_backendObjectRoots.find(mountBackendId);
	if (it == m_backendObjectRoots.end() || !it->second.valid())
	{
		m_tcpTeachActive = false;
		m_tcpTeachMountBackendId.clear();
		m_tcpTeachResolveRobotBaseWorld = nullptr;
		m_tcpTeachUseFlangeLocalPlacement = false;
		return;
	}

	m_tcpTeachMountPat = new osg::MatrixTransform;
	m_tcpTeachMountPat->setName("TcpTeachMount");

	m_tcpTeachOverlayGroup = new osg::Group;
	m_tcpTeachOverlayGroup->setName("TcpTeachGizmoOverlay");
	m_tcpTeachCompassTransform = new osg::PositionAttitudeTransform;
	m_tcpTeachCompassScaleTransform = new osg::MatrixTransform;
	m_tcpTeachCompassNode = buildTcpTeachCompassGeometry(m_tcpTeachAxisBranch, m_tcpTeachRingBranch);
	m_tcpTeachCompassScaleTransform->addChild(m_tcpTeachCompassNode.get());
	m_tcpTeachCompassTransform->addChild(m_tcpTeachCompassScaleTransform.get());
	m_tcpTeachOverlayGroup->addChild(m_tcpTeachCompassTransform.get());
	m_tcpTeachMountPat->addChild(m_tcpTeachOverlayGroup.get());

	it->second->addChild(m_tcpTeachMountPat.get());
	updateTcpDragTeachFromTarget(T_base_target);
	syncTcpTeachCompassAttitude();
	updateTcpTeachCompassScale();
	m_tcpTeachCompassTransform->setNodeMask(0xffffffffu);
	requestRedraw();
}

void OsgWidget::endTcpDragTeach()
{
	if (m_tcpTeachMountPat.valid() && !m_tcpTeachMountBackendId.empty())
	{
		auto it = m_backendObjectRoots.find(m_tcpTeachMountBackendId);
		if (it != m_backendObjectRoots.end() && it->second.valid())
		{
			it->second->removeChild(m_tcpTeachMountPat.get());
		}
	}
	m_tcpTeachActive = false;
	m_tcpTeachResolveRobotBaseWorld = nullptr;
	m_tcpTeachUseFlangeLocalPlacement = false;
	m_tcpTeachDragging = false;
	m_tcpTeachRotating = false;
	m_tcpTeachDragAxis = DragAxis::None;
	m_tcpTeachDragAxisWorld.set(0.0, 0.0, 0.0);
	m_tcpTeachDragScreenAxisUx = 1.0;
	m_tcpTeachDragScreenAxisUy = 0.0;
	m_tcpTeachDragMmPerPixel = 1.0;
	m_tcpTeachHoverAxis = DragAxis::None;
	m_tcpTeachMountPat = nullptr;
	m_tcpTeachOverlayGroup = nullptr;
	m_tcpTeachCompassTransform = nullptr;
	m_tcpTeachCompassScaleTransform = nullptr;
	m_tcpTeachCompassNode = nullptr;
	for (int i = 0; i < 3; ++i)
	{
		m_tcpTeachAxisBranch[i] = nullptr;
		m_tcpTeachRingBranch[i] = nullptr;
	}
	m_tcpTeachMountBackendId.clear();
	requestRedraw();
}

void OsgWidget::updateTcpDragTeachFromTarget(const engine::RigidTransform& T_base_target)
{
	m_tcpTeachTargetInBase = T_base_target;
	if (!m_tcpTeachActive || !m_tcpTeachMountPat.valid() || m_tcpTeachMountBackendId.empty())
	{
		return;
	}
	if (m_tcpTeachUseFlangeLocalPlacement)
	{
		m_tcpTeachMountPat->setMatrix(m_tcpTeachToolLocalOnFlange);
		syncTcpTeachCompassAttitude();
		updateTcpTeachCompassScale();
		requestRedraw();
		return;
	}
	osg::Matrixd sceneWorld;
	sceneWorld.makeIdentity();
	if (!getBackendRootWorldMatrix(m_tcpTeachMountBackendId, sceneWorld))
	{
		return;
	}
	const osg::Matrixd toolInBase = engine::osgMatrixFromRigidTransform(T_base_target);
	const osg::Matrixd localOnRoot = toolInBase * osg::Matrixd::inverse(sceneWorld);
	m_tcpTeachMountPat->setMatrix(localOnRoot);
	syncTcpTeachCompassAttitude();
	updateTcpTeachCompassScale();
	requestRedraw();
}

void OsgWidget::updateTcpTeachCompassHighlight(const DragAxis axis, const bool highlightRing)
{
	for (int i = 0; i < 3; ++i)
	{
		if (m_tcpTeachAxisBranch[i].valid())
		{
			m_tcpTeachAxisBranch[i]->setMatrix(osg::Matrix::identity());
		}
		if (m_tcpTeachRingBranch[i].valid())
		{
			m_tcpTeachRingBranch[i]->setMatrix(osg::Matrix::identity());
		}
	}
	if (axis == DragAxis::None)
	{
		return;
	}
	int idx = 0;
	if (axis == DragAxis::Y)
	{
		idx = 1;
	}
	else if (axis == DragAxis::Z)
	{
		idx = 2;
	}
	const osg::Matrix hi = osg::Matrix::scale(1.28f, 1.28f, 1.28f);
	if (highlightRing)
	{
		if (m_tcpTeachRingBranch[idx].valid())
		{
			m_tcpTeachRingBranch[idx]->setMatrix(hi);
		}
	}
	else if (m_tcpTeachAxisBranch[idx].valid())
	{
		m_tcpTeachAxisBranch[idx]->setMatrix(hi);
	}
}

void OsgWidget::updateTcpTeachCompassScale()
{
	if (!m_tcpTeachCompassTransform.valid() || !m_tcpTeachCompassScaleTransform.valid() || !m_viewer.valid()
		|| !m_viewer->getCamera() || !m_tcpTeachActive)
	{
		return;
	}
	osg::Vec3d eye, center, up;
	m_viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
	osg::Vec3f pivotF;
	computeTcpTeachPivotWorld(pivotF);
	const osg::Vec3d anchor(static_cast<double>(pivotF.x()), static_cast<double>(pivotF.y()),
		static_cast<double>(pivotF.z()));
	const double distance = (eye - anchor).length();
	if (m_tcpTeachGizmoRefDistance < 0.0 || m_tcpTeachGizmoRefDistance <= 1e-6)
	{
		m_tcpTeachGizmoRefDistance = std::max(1.0, distance);
		const double desiredAxisWorld = std::max(20.0, static_cast<double>(m_tcpTeachModelDiagonal) * 0.08);
		m_tcpTeachGizmoRefScale = std::max(0.4, std::min(800.0, desiredAxisWorld / 120.0));
	}
	double scale = m_tcpTeachGizmoRefScale * (distance / m_tcpTeachGizmoRefDistance);
	scale = std::max(0.3, std::min(1200.0, scale));
	m_tcpTeachCompassScaleTransform->setMatrix(osg::Matrixd::scale(scale, scale, scale));
}

bool OsgWidget::beginTcpTeachScreenDrag()
{
	m_tcpTeachDragScreenAxisUx = 1.0;
	m_tcpTeachDragScreenAxisUy = 0.0;
	m_tcpTeachDragMmPerPixel = 1.0;
	if (!m_tcpTeachActive || !m_viewer.valid() || !m_viewer->getCamera() || viewportWidth() <= 0
		|| viewportHeight() <= 0)
	{
		return false;
	}
	osg::Vec3d axisW;
	if (!tcpTeachCompassUnitAxisWorld(m_tcpTeachDragAxis, axisW))
	{
		return false;
	}
	m_tcpTeachDragAxisWorld = axisW;

	float gizmoScale = 1.0f;
	if (m_tcpTeachCompassScaleTransform.valid())
	{
		const osg::Matrixd& sm = m_tcpTeachCompassScaleTransform->getMatrix();
		gizmoScale = static_cast<float>(std::max({ std::abs(sm(0, 0)), std::abs(sm(1, 1)), std::abs(sm(2, 2)) }));
		if (gizmoScale < 1e-6f)
		{
			gizmoScale = 1.0f;
		}
	}
	const float axisLenMm = 120.0f * gizmoScale;

	osg::Vec3f origin;
	computeTcpTeachPivotWorld(origin);
	const osg::Vec3f tipWorld(
		origin.x() + static_cast<float>(axisW.x() * static_cast<double>(axisLenMm)),
		origin.y() + static_cast<float>(axisW.y() * static_cast<double>(axisLenMm)),
		origin.z() + static_cast<float>(axisW.z() * static_cast<double>(axisLenMm)));

	osg::Camera* const camera = m_viewer->getCamera();
	const osg::Matrixd mvp = camera->getViewMatrix() * camera->getProjectionMatrix();
	auto projectToScreen = [&](const osg::Vec3f& world, double& sx, double& sy) {
		const osg::Vec3d clip = osg::Vec3d(world) * mvp;
		sx = (clip.x() * 0.5 + 0.5) * static_cast<double>(viewportWidth());
		sy = (1.0 - (clip.y() * 0.5 + 0.5)) * static_cast<double>(viewportHeight());
	};

	double ox = 0.0;
	double oy = 0.0;
	double tx = 0.0;
	double ty = 0.0;
	projectToScreen(origin, ox, oy);
	projectToScreen(tipWorld, tx, ty);
	const double vx = tx - ox;
	const double vy = ty - oy;
	const double lenPx = std::hypot(vx, vy);
	if (lenPx < 1e-3)
	{
		return false;
	}
	m_tcpTeachDragScreenAxisUx = vx / lenPx;
	m_tcpTeachDragScreenAxisUy = vy / lenPx;
	m_tcpTeachDragMmPerPixel = static_cast<double>(axisLenMm) / lenPx;
	return true;
}

double OsgWidget::tcpTeachScreenDragDsMm(const QPoint& curPos, const QPoint& lastPos) const
{
	const double dpr = (OsgScene::devicePixelRatio() > 0.0) ? OsgScene::devicePixelRatio() : 1.0;
	const double dx = (static_cast<double>(curPos.x()) - static_cast<double>(lastPos.x())) * dpr;
	const double dy = (static_cast<double>(curPos.y()) - static_cast<double>(lastPos.y())) * dpr;
	const double dPx = dx * m_tcpTeachDragScreenAxisUx + dy * m_tcpTeachDragScreenAxisUy;
	return dPx * m_tcpTeachDragMmPerPixel;
}

bool OsgWidget::tcpTeachCompassUnitAxisWorld(const DragAxis axis, osg::Vec3d& outAxisWorld) const
{
	outAxisWorld.set(0.0, 0.0, 1.0);
	if (!m_tcpTeachActive || axis == DragAxis::None)
	{
		return false;
	}
	osg::Vec3f origin;
	computeTcpTeachPivotWorld(origin);
	osg::Vec3f tipLocal(120.0f, 0.0f, 0.0f);
	if (axis == DragAxis::Y)
	{
		tipLocal.set(0.0f, 120.0f, 0.0f);
	}
	else if (axis == DragAxis::Z)
	{
		tipLocal.set(0.0f, 0.0f, 120.0f);
	}
	const osg::Quat attitude = rigidRotationToOsgQuat(m_tcpTeachTargetInBase);
	osg::Quat compassAtt;
	if (transformGizmoFrame() == TransformGizmoFrame::World)
	{
		compassAtt = attitude.inverse();
	}
	else
	{
		compassAtt = osg::Quat();
	}
	const osg::Vec3f tipWorld = origin + attitude * (compassAtt * tipLocal);
	outAxisWorld.set(
		static_cast<double>(tipWorld.x() - origin.x()),
		static_cast<double>(tipWorld.y() - origin.y()),
		static_cast<double>(tipWorld.z() - origin.z()));
	const double len = outAxisWorld.length();
	if (len < 1e-12)
	{
		return false;
	}
	outAxisWorld /= len;
	return true;
}

void OsgWidget::computeTcpTeachPivotWorld(osg::Vec3f& outPivotWorld) const
{
	outPivotWorld.set(0.0f, 0.0f, 0.0f);
	if (!m_tcpTeachMountPat.valid())
	{
		return;
	}
	osg::NodePath path;
	for (osg::Node* n = m_tcpTeachMountPat.get(); n != nullptr; n = n->getNumParents() > 0 ? n->getParent(0) : nullptr)
	{
		path.insert(path.begin(), n);
	}
	const osg::Vec3d w = osg::Vec3d(0.0, 0.0, 0.0) * osg::computeLocalToWorld(path);
	outPivotWorld.set(static_cast<float>(w.x()), static_cast<float>(w.y()), static_cast<float>(w.z()));
}

int OsgWidget::pickTcpTeachAxisAtScreenPos(const QPoint& mousePos, const bool preferRing, bool* outPickedRing) const
{
	if (outPickedRing)
	{
		*outPickedRing = false;
	}
	if (!m_tcpTeachActive || !m_viewer.valid() || !m_viewer->getCamera() || viewportWidth() <= 0
		|| viewportHeight() <= 0 || !m_tcpTeachCompassNode.valid())
	{
		return kGizmoAxisNone;
	}
	const double dpr =
		(OsgScene::devicePixelRatio() > 0.0) ? OsgScene::devicePixelRatio() : 1.0;
	const double mx = static_cast<double>(mousePos.x()) * dpr;
	const double my = static_cast<double>(mousePos.y()) * dpr;

	float gizmoScale = 1.0f;
	if (m_tcpTeachCompassScaleTransform.valid())
	{
		const osg::Matrixd& sm = m_tcpTeachCompassScaleTransform->getMatrix();
		gizmoScale = static_cast<float>(std::max({ std::abs(sm(0, 0)), std::abs(sm(1, 1)), std::abs(sm(2, 2)) }));
		if (gizmoScale < 1e-6f)
		{
			gizmoScale = 1.0f;
		}
	}
	const float axisLen = 120.0f * gizmoScale;
	const float ringRadius = 65.0f * gizmoScale;

	osg::Camera* camera = m_viewer->getCamera();
	const osg::Matrixd mvp = camera->getViewMatrix() * camera->getProjectionMatrix();
	const osg::Quat attitude = rigidRotationToOsgQuat(m_tcpTeachTargetInBase);
	osg::Vec3f origin;
	computeTcpTeachPivotWorld(origin);
	osg::Quat compassAtt;
	if (transformGizmoFrame() == TransformGizmoFrame::World)
	{
		compassAtt = attitude.inverse();
	}
	else
	{
		compassAtt = osg::Quat();
	}
	auto toWorld = [&](const osg::Vec3f& local) -> osg::Vec3f {
		return origin + attitude * (compassAtt * local);
	};

	auto projectToScreen = [&](const osg::Vec3f& world, double& sx, double& sy) {
		osg::Vec3d clip = osg::Vec3d(world) * mvp;
		sx = (clip.x() * 0.5 + 0.5) * static_cast<double>(viewportWidth());
		sy = (1.0 - (clip.y() * 0.5 + 0.5)) * static_cast<double>(viewportHeight());
	};

	double ox = 0, oy = 0, pxx = 0, pxy = 0, pyx = 0, pyy = 0, pzx = 0, pzy = 0;
	projectToScreen(origin, ox, oy);
	projectToScreen(toWorld(osg::Vec3f(axisLen, 0.0f, 0.0f)), pxx, pxy);
	projectToScreen(toWorld(osg::Vec3f(0.0f, axisLen, 0.0f)), pyx, pyy);
	projectToScreen(toWorld(osg::Vec3f(0.0f, 0.0f, axisLen)), pzx, pzy);

	auto distanceToSegment = [](double p0x, double p0y, double p1x, double p1y, double qx, double qy) -> double {
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
	const double axisLenPx = std::max({ std::hypot(pxx - ox, pxy - oy), std::hypot(pyx - ox, pyy - oy),
		std::hypot(pzx - ox, pzy - oy), 1.0 });
	const double threshold = std::clamp(0.22 * axisLenPx, 14.0, 44.0);
	const double ringThreshold = std::clamp(0.14 * axisLenPx, 10.0, 36.0);

	auto minDistanceToProjectedRing = [&](int axis) -> double {
		const int segments = 72;
		const float r = ringRadius;
		double minDist = 1e9;
		for (int i = 0; i < segments; ++i)
		{
			const float a0 = osg::PI * 2.0f * static_cast<float>(i) / static_cast<float>(segments);
			const float a1 = osg::PI * 2.0f * static_cast<float>(i + 1) / static_cast<float>(segments);
			osg::Vec3f w0, w1;
			if (axis == kGizmoAxisX)
			{
				w0 = toWorld(osg::Vec3f(0.0f, std::cos(a0) * r, std::sin(a0) * r));
				w1 = toWorld(osg::Vec3f(0.0f, std::cos(a1) * r, std::sin(a1) * r));
			}
			else if (axis == kGizmoAxisY)
			{
				w0 = toWorld(osg::Vec3f(std::cos(a0) * r, 0.0f, std::sin(a0) * r));
				w1 = toWorld(osg::Vec3f(std::cos(a1) * r, 0.0f, std::sin(a1) * r));
			}
			else
			{
				w0 = toWorld(osg::Vec3f(std::cos(a0) * r, std::sin(a0) * r, 0.0f));
				w1 = toWorld(osg::Vec3f(std::cos(a1) * r, std::sin(a1) * r, 0.0f));
			}
			double s0x = 0, s0y = 0, s1x = 0, s1y = 0;
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

void OsgWidget::emitTcpDragTeachPoseChanged()
{
	double px = 0.0;
	double py = 0.0;
	double pz = 0.0;
	double ex = 0.0;
	double ey = 0.0;
	double ez = 0.0;
	m_tcpTeachTargetInBase.translationMm(px, py, pz);
	m_tcpTeachTargetInBase.eulerDegForDisplay(ex, ey, ez);
	emit tcpDragTeachPoseChanged(px, py, pz, ex, ey, ez);
}

void OsgWidget::applyTcpTeachTranslationWorld(const int axisIndex, const double dsWorld)
{
	if (std::abs(dsWorld) < 1e-10)
	{
		return;
	}
	osg::Vec3d axisW = m_tcpTeachDragAxisWorld;
	if (axisW.length2() < 1e-18)
	{
		axisW = osg::Vec3d(0.0, 0.0, 1.0);
		if (axisIndex == 0)
		{
			axisW.set(1.0, 0.0, 0.0);
		}
		else if (axisIndex == 1)
		{
			axisW.set(0.0, 1.0, 0.0);
		}
	}
	osg::Vec3f pivotBefore;
	computeTcpTeachPivotWorld(pivotBefore);
	osg::Matrixd toolWorld;
	if (!tcpTeachToolWorldMatrix(toolWorld))
	{
		return;
	}
	engine::RigidTransform toolInWorld = engine::rigidTransformFromOsg(toolWorld);
	Eigen::Vector3d t = toolInWorld.translationMm();
	t += Eigen::Vector3d(axisW.x(), axisW.y(), axisW.z()) * dsWorld;
	toolInWorld.setTranslationMm(t);
	tcpTeachSetTargetFromToolWorld(engine::osgMatrixFromRigidTransform(toolInWorld));
	// #region agent log
	if (axisIndex == 0 || axisIndex == 1)
	{
		osg::Vec3f pivotAfter;
		computeTcpTeachPivotWorld(pivotAfter);
		const int frameWorld = (transformGizmoFrame() == TransformGizmoFrame::World) ? 1 : 0;
		const double pivotBeforeAxis = (axisIndex == 0) ? pivotBefore.x() : pivotBefore.y();
		const double pivotAfterAxis = (axisIndex == 0) ? pivotAfter.x() : pivotAfter.y();
		agentDbgLog4ce7a0(
			"H15",
			"apply_world_tool_delta",
			std::string("{\"axis\":") + std::to_string(axisIndex) + ",\"frameWorld\":" + std::to_string(frameWorld)
			+ ",\"dsWorldMm\":" + std::to_string(dsWorld) + ",\"pivotBefore\":" + std::to_string(pivotBeforeAxis)
			+ ",\"pivotAfter\":" + std::to_string(pivotAfterAxis)
			+ ",\"mmPerPx\":" + std::to_string(m_tcpTeachDragMmPerPixel) + "}");
	}
	// #endregion
}

void OsgWidget::applyTcpTeachTranslationBody(const int axisIndex, const double dsWorld)
{
	if (std::abs(dsWorld) < 1e-10)
	{
		return;
	}
	Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
	if (axisIndex == 0)
	{
		axis = Eigen::Vector3d::UnitX();
	}
	else if (axisIndex == 1)
	{
		axis = Eigen::Vector3d::UnitY();
	}
	const Eigen::Vector3d delta = m_tcpTeachTargetInBase.rotation() * (axis * dsWorld);
	Eigen::Vector3d t = m_tcpTeachTargetInBase.translationMm();
	t += delta;
	m_tcpTeachTargetInBase.setTranslationMm(t);
}

void OsgWidget::applyTcpTeachRotationWorld(const int axisIndex, const double deltaRad)
{
	if (std::abs(deltaRad) < 1e-10)
	{
		return;
	}
	osg::Matrixd toolWorld;
	if (!tcpTeachToolWorldMatrix(toolWorld))
	{
		return;
	}
	Eigen::Vector3d axisW = Eigen::Vector3d::UnitZ();
	if (axisIndex == 0)
	{
		axisW = Eigen::Vector3d::UnitX();
	}
	else if (axisIndex == 1)
	{
		axisW = Eigen::Vector3d::UnitY();
	}
	const Eigen::AngleAxisd aa(deltaRad, axisW.normalized());
	const Eigen::Matrix3d Rdelta = aa.toRotationMatrix();
	Eigen::Matrix3d Rtool = Eigen::Matrix3d::Identity();
	for (int r = 0; r < 3; ++r)
	{
		for (int c = 0; c < 3; ++c)
		{
			Rtool(r, c) = toolWorld(r, c);
		}
	}
	const Eigen::Matrix3d Rnew = Rdelta * Rtool;
	for (int r = 0; r < 3; ++r)
	{
		for (int c = 0; c < 3; ++c)
		{
			toolWorld(r, c) = Rnew(r, c);
		}
	}
	tcpTeachSetTargetFromToolWorld(toolWorld);
}

void OsgWidget::syncTcpTeachCompassAttitude()
{
	if (!m_tcpTeachCompassTransform.valid())
	{
		return;
	}
	const osg::Quat bodyQ = rigidRotationToOsgQuat(m_tcpTeachTargetInBase);
	if (transformGizmoFrame() == TransformGizmoFrame::World)
	{
		m_tcpTeachCompassTransform->setAttitude(bodyQ.inverse());
	}
	else
	{
		m_tcpTeachCompassTransform->setAttitude(osg::Quat());
	}
}

void OsgWidget::applyTcpTeachRotationBody(const int axisIndex, const double deltaRad)
{
	if (std::abs(deltaRad) < 1e-10)
	{
		return;
	}
	Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
	if (axisIndex == 0)
	{
		axis = Eigen::Vector3d::UnitX();
	}
	else if (axisIndex == 1)
	{
		axis = Eigen::Vector3d::UnitY();
	}
	const Eigen::AngleAxisd aa(deltaRad, axis.normalized());
	Eigen::Quaterniond qNew = m_tcpTeachTargetInBase.rotation() * Eigen::Quaterniond(aa);
	m_tcpTeachTargetInBase.setRotation(qNew);
}
