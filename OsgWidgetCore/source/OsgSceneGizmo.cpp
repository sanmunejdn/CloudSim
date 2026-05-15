#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "OsgScene.h"

#include "ObjectGizmoFrame.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>

#include "RunLogger.h"

#include <osg/GL>
#include <osg/BlendFunc>
#include <osg/Camera>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/LineWidth>
#include <osg/MatrixTransform>
#include <osg/Node>
#include <osg/PolygonOffset>
#include <osg/PositionAttitudeTransform>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osg/StateAttribute>
#include <osg/Matrixd>
#include <osg/Vec3>
#include <osg/Vec3d>
#include <osg/Vec4>
#include <osg/Vec4d>
#include <osgViewer/Viewer>

namespace
{
bool gizmoPivotDiagEnabled()
{
	const char* const e = std::getenv("POINTCLOUD_GIZMO_PIVOT_DIAG");
	return e != nullptr && e[0] != '\0' && std::strcmp(e, "0") != 0;
}

osg::NodePath nodePathToSceneRootFromLeaf(const osg::Node* leaf)
{
	osg::NodePath path;
	for (const osg::Node* n = leaf; n != nullptr; n = n->getNumParents() > 0 ? n->getParent(0) : nullptr)
	{
		path.insert(path.begin(), const_cast<osg::Node*>(n));
	}
	return path;
}
} // namespace

osg::Node* OsgScene::createCompassNode()
{
	for (int i = 0; i < 3; ++i)
	{
		m_compassAxisBranch[i] = nullptr;
		m_compassRingBranch[i] = nullptr;
	}

	const float axisLen = 120.0f;
	const float coneH = 20.0f;
	const float coneR = 7.0f;
	const float shaftEnd = axisLen - 12.0f;
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
		return g;
	};

	auto addTubeRing = [&](int plane, const osg::Vec4& color) -> osg::ref_ptr<osg::Group> {
		osg::ref_ptr<osg::Group> ringGroup = new osg::Group;
		const float h = (osg::PI * 2.0f * ringRadius / static_cast<float>(ringSegments)) * 1.18f;
		for (int i = 0; i < ringSegments; ++i)
		{
			const float am = osg::PI * 2.0f * (static_cast<float>(i) + 0.5f) / static_cast<float>(ringSegments);
			osg::Vec3 p;
			osg::Vec3 tangent;
			if (plane == 0)
			{
				p.set(0.0f, std::cos(am) * ringRadius, std::sin(am) * ringRadius);
				tangent.set(0.0f, -std::sin(am), std::cos(am));
			}
			else if (plane == 1)
			{
				p.set(std::cos(am) * ringRadius, 0.0f, std::sin(am) * ringRadius);
				tangent.set(-std::sin(am), 0.0f, std::cos(am));
			}
			else
			{
				p.set(std::cos(am) * ringRadius, std::sin(am) * ringRadius, 0.0f);
				tangent.set(-std::sin(am), std::cos(am), 0.0f);
			}
			osg::Quat rot;
			rot.makeRotate(osg::Vec3(0.0f, 0.0f, 1.0f), tangent);
			osg::ref_ptr<osg::Cylinder> cyl = new osg::Cylinder(osg::Vec3(0.0f, 0.0f, 0.0f), tubeR, h);
			cyl->setRotation(rot);
			osg::ref_ptr<osg::MatrixTransform> seg = new osg::MatrixTransform;
			seg->setMatrix(osg::Matrix::translate(p));
			osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(cyl.get());
			sd->setColor(color);
			osg::ref_ptr<osg::Geode> segGeode = new osg::Geode;
			segGeode->addDrawable(sd.get());
			seg->addChild(segGeode.get());
			ringGroup->addChild(seg.get());
		}
		return ringGroup;
	};

	osg::ref_ptr<osg::Group> root = new osg::Group;
	applyCompassStateSet(root->getOrCreateStateSet());

	const osg::Vec4 red(1.0f, 0.15f, 0.15f, 1.0f);
	const osg::Vec4 green(0.15f, 1.0f, 0.15f, 1.0f);
	const osg::Vec4 blue(0.15f, 0.45f, 1.0f, 1.0f);

	osg::ref_ptr<osg::MatrixTransform> ax = new osg::MatrixTransform;
	ax->addChild(addPositiveAxis(osg::Vec3(shaftEnd, 0.0f, 0.0f), osg::Vec3(1.0f, 0.0f, 0.0f), red).get());
	m_compassAxisBranch[0] = ax;
	root->addChild(ax.get());

	osg::ref_ptr<osg::MatrixTransform> ay = new osg::MatrixTransform;
	ay->addChild(addPositiveAxis(osg::Vec3(0.0f, shaftEnd, 0.0f), osg::Vec3(0.0f, 1.0f, 0.0f), green).get());
	m_compassAxisBranch[1] = ay;
	root->addChild(ay.get());

	osg::ref_ptr<osg::MatrixTransform> az = new osg::MatrixTransform;
	az->addChild(addPositiveAxis(osg::Vec3(0.0f, 0.0f, shaftEnd), osg::Vec3(0.0f, 0.0f, 1.0f), blue).get());
	m_compassAxisBranch[2] = az;
	root->addChild(az.get());

	osg::ref_ptr<osg::MatrixTransform> rx = new osg::MatrixTransform;
	rx->addChild(addTubeRing(0, osg::Vec4(1.0f, 0.35f, 0.35f, 0.92f)).get());
	m_compassRingBranch[0] = rx;
	root->addChild(rx.get());

	osg::ref_ptr<osg::MatrixTransform> ry = new osg::MatrixTransform;
	ry->addChild(addTubeRing(1, osg::Vec4(0.35f, 1.0f, 0.35f, 0.92f)).get());
	m_compassRingBranch[1] = ry;
	root->addChild(ry.get());

	osg::ref_ptr<osg::MatrixTransform> rz = new osg::MatrixTransform;
	rz->addChild(addTubeRing(2, osg::Vec4(0.35f, 0.55f, 1.0f, 0.92f)).get());
	m_compassRingBranch[2] = rz;
	root->addChild(rz.get());

	return root.release();
}

void OsgScene::syncCompassGizmoOrientation()
{
	if (!m_compassTransform.valid())
	{
		return;
	}
	ObjectGizmoFrame gf;
	if (!readActiveObjectGizmoFrame(gf))
	{
		m_compassTransform->setAttitude(osg::Quat());
		return;
	}
	if (m_transformGizmoFrame == TransformGizmoFrame::World)
	{
		m_compassTransform->setAttitude(gf.attitude().inverse());
	}
	else
	{
		m_compassTransform->setAttitude(osg::Quat());
	}
}

void OsgScene::attachCompassGraphics()
{
	static const unsigned int kMaskHelper = 0x2u;
	if (!m_compassTransform.valid() || !m_objectSelectionMode || !m_selectionActive)
	{
		return;
	}
	// Old layout: compass geode was a direct child of PAT; screen scale on PAT skewed the pivot away from model origin.
	if (m_compassNode.valid() && !m_compassScaleTransform.valid())
	{
		m_compassTransform->removeChild(m_compassNode.get());
		m_compassNode = nullptr;
		for (int i = 0; i < 3; ++i)
		{
			m_compassAxisBranch[i] = nullptr;
			m_compassRingBranch[i] = nullptr;
		}
	}
	if (m_compassScaleTransform.valid() && !m_compassNode.valid())
	{
		m_compassTransform->removeChild(m_compassScaleTransform.get());
		m_compassScaleTransform = nullptr;
	}
	if (m_compassNode.valid() && m_compassScaleTransform.valid())
	{
		m_compassTransform->setNodeMask(kMaskHelper);
		return;
	}
	m_compassScaleTransform = new osg::MatrixTransform;
	m_compassScaleTransform->setMatrix(osg::Matrix::identity());
	m_compassNode = createCompassNode();
	m_compassScaleTransform->addChild(m_compassNode.get());
	m_compassTransform->addChild(m_compassScaleTransform.get());
	m_compassTransform->setNodeMask(kMaskHelper);
	m_compassTransform->setPosition(osg::Vec3d(0.0, 0.0, 0.0));
}

void OsgScene::detachCompassGraphics()
{
	if (m_compassTransform.valid())
	{
		if (m_compassScaleTransform.valid())
		{
			m_compassScaleTransform->setMatrix(osg::Matrix::identity());
			m_compassTransform->removeChild(m_compassScaleTransform.get());
		}
		else if (m_compassNode.valid())
		{
			m_compassTransform->removeChild(m_compassNode.get());
		}
	}
	m_compassNode = nullptr;
	m_compassScaleTransform = nullptr;
	for (int i = 0; i < 3; ++i)
	{
		m_compassAxisBranch[i] = nullptr;
		m_compassRingBranch[i] = nullptr;
	}
	if (m_compassTransform.valid())
	{
		m_compassTransform->setNodeMask(0u);
		m_compassTransform->setScale(osg::Vec3d(1.0, 1.0, 1.0));
		m_compassTransform->setPosition(osg::Vec3d(0.0, 0.0, 0.0));
		m_compassTransform->setAttitude(osg::Quat());
	}
	m_gizmoReferenceDistance = -1.0;
	m_gizmoReferenceScale = 1.0;
}

void OsgScene::refreshCompassDrawVisibility()
{
	if (m_objectSelectionMode && m_selectionActive)
	{
		attachCompassGraphics();
	}
	else
	{
		detachCompassGraphics();
	}
}

void OsgScene::updateCompassHighlight(int axis, bool highlightRing)
{
	for (int i = 0; i < 3; ++i)
	{
		if (m_compassAxisBranch[i].valid())
		{
			m_compassAxisBranch[i]->setMatrix(osg::Matrix::identity());
		}
		if (m_compassRingBranch[i].valid())
		{
			m_compassRingBranch[i]->setMatrix(osg::Matrix::identity());
		}
	}
	if (axis == kGizmoAxisNone)
	{
		return;
	}
	const int idx = axis - 1;
	if (idx < 0 || idx > 2)
	{
		return;
	}
	const float s = 1.28f;
	const osg::Matrix hi = osg::Matrix::scale(s, s, s);
	if (highlightRing)
	{
		if (m_compassRingBranch[idx].valid())
		{
			m_compassRingBranch[idx]->setMatrix(hi);
		}
	}
	else
	{
		if (m_compassAxisBranch[idx].valid())
		{
			m_compassAxisBranch[idx]->setMatrix(hi);
		}
	}
}

void OsgScene::updateCompassScale()
{
	if (!m_compassTransform.valid() || !m_compassScaleTransform.valid() || !m_viewer.valid() || !m_viewer->getCamera()
		|| !m_selectionActive || !m_objectSelectionMode || !m_compassNode.valid())
	{
		return;
	}

	osg::Vec3d eye, center, up;
	m_viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
	const osg::Vec3d anchor = [&]() {
		osg::Vec3f pivotF;
		computeGizmoPivotWorld(pivotF);
		if (m_activeBackendOuterPat.valid())
		{
			return osg::Vec3d(static_cast<double>(pivotF.x()), static_cast<double>(pivotF.y()),
				static_cast<double>(pivotF.z()));
		}
		return center;
	}();
	const double distance = (eye - anchor).length();

	if (m_gizmoReferenceDistance < 0.0 || m_gizmoReferenceDistance <= 1e-6)
	{
		m_gizmoReferenceDistance = std::max(1.0, distance);
		const double desiredAxisWorld = std::max(20.0, static_cast<double>(m_activeModelDiagonal) * 0.08);
		m_gizmoReferenceScale = std::max(0.4, std::min(800.0, desiredAxisWorld / 120.0));
	}
	double scale = m_gizmoReferenceScale * (distance / m_gizmoReferenceDistance);
	scale = std::max(0.3, std::min(1200.0, scale));
	m_compassScaleTransform->setMatrix(osg::Matrixd::scale(scale, scale, scale));
}

int OsgScene::pickAxisAtScreenPos(double mouseX, double mouseY, bool preferRing, bool* outPickedRing) const
{
	if (outPickedRing)
	{
		*outPickedRing = false;
	}
	ObjectGizmoFrame gizmoFrame;
	const bool haveFrame = readActiveObjectGizmoFrame(gizmoFrame);
	if (!haveFrame || !m_viewer.valid() || !m_viewer->getCamera()
		|| viewportWidth() <= 0 || viewportHeight() <= 0)
	{
		return kGizmoAxisNone;
	}
	if (!m_objectSelectionMode || !m_compassNode.valid())
	{
		return kGizmoAxisNone;
	}
	// Viewport is device pixels (see QWidgetViewer::windowResized); Qt mouse is logical — match OSG.
	const double dpr = (m_devicePixelRatio > 0.0) ? m_devicePixelRatio : 1.0;
	const double mx = mouseX * dpr;
	const double my = mouseY * dpr;

	float gizmoScale = 1.0f;
	if (m_compassScaleTransform.valid())
	{
		const osg::Matrixd& sm = m_compassScaleTransform->getMatrix();
		const double sx = std::abs(sm(0, 0));
		const double sy = std::abs(sm(1, 1));
		const double sz = std::abs(sm(2, 2));
		gizmoScale = static_cast<float>(std::max(sx, std::max(sy, sz)));
		if (gizmoScale < 1e-6f)
		{
			gizmoScale = 1.0f;
		}
	}
	const float axisLen = 120.0f * gizmoScale;
	const float ringRadius = 65.0f * gizmoScale;

	osg::Camera* camera = m_viewer->getCamera();
	const osg::Matrixd mvp = camera->getViewMatrix() * camera->getProjectionMatrix();
	const osg::Quat attitude = gizmoFrame.attitude();
	osg::Vec3f origin;
	computeGizmoPivotWorld(origin);
	osg::Quat compassAtt;
	if (m_transformGizmoFrame == TransformGizmoFrame::World)
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
		if (len2 <= 1e-6) return std::hypot(qx - p0x, qy - p0y);
		const double t = std::max(0.0, std::min(1.0, (wx * vx + wy * vy) / len2));
		const double projx = p0x + vx * t;
		const double projy = p0y + vy * t;
		return std::hypot(qx - projx, qy - projy);
	};

	const double dx = distanceToSegment(ox, oy, pxx, pxy, mx, my);
	const double dy = distanceToSegment(ox, oy, pyx, pyy, mx, my);
	const double dz = distanceToSegment(ox, oy, pzx, pzy, mx, my);

	const double axisPx0 = std::hypot(pxx - ox, pxy - oy);
	const double axisPx1 = std::hypot(pyx - ox, pyy - oy);
	const double axisPx2 = std::hypot(pzx - ox, pzy - oy);
	const double axisLenPx = std::max({axisPx0, axisPx1, axisPx2, 1.0});
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
			if (axis == kGizmoAxisX) { w0 = toWorld(osg::Vec3f(0.0f, std::cos(a0) * r, std::sin(a0) * r)); w1 = toWorld(osg::Vec3f(0.0f, std::cos(a1) * r, std::sin(a1) * r)); }
			if (axis == kGizmoAxisY) { w0 = toWorld(osg::Vec3f(std::cos(a0) * r, 0.0f, std::sin(a0) * r)); w1 = toWorld(osg::Vec3f(std::cos(a1) * r, 0.0f, std::sin(a1) * r)); }
			if (axis == kGizmoAxisZ) { w0 = toWorld(osg::Vec3f(std::cos(a0) * r, std::sin(a0) * r, 0.0f)); w1 = toWorld(osg::Vec3f(std::cos(a1) * r, std::sin(a1) * r, 0.0f)); }
			double s0x = 0, s0y = 0, s1x = 0, s1y = 0;
			projectToScreen(w0, s0x, s0y);
			projectToScreen(w1, s1x, s1y);
			const double d = distanceToSegment(s0x, s0y, s1x, s1y, mx, my);
			if (d < minDist) minDist = d;
		}
		return minDist;
	};

	const double drx = minDistanceToProjectedRing(kGizmoAxisX);
	const double dry = minDistanceToProjectedRing(kGizmoAxisY);
	const double drz = minDistanceToProjectedRing(kGizmoAxisZ);

	if (preferRing)
	{
		double best = ringThreshold;
		int ringAxis = kGizmoAxisNone;
		if (drx < best) { best = drx; ringAxis = kGizmoAxisX; }
		if (dry < best) { best = dry; ringAxis = kGizmoAxisY; }
		if (drz < best) { best = drz; ringAxis = kGizmoAxisZ; }
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
	if (dx < minDist) { minDist = dx; axis = kGizmoAxisX; }
	if (dy < minDist) { minDist = dy; axis = kGizmoAxisY; }
	if (dz < minDist) { minDist = dz; axis = kGizmoAxisZ; }
	return axis;
}

void OsgScene::computeGizmoPivotWorld(osg::Vec3f& outPivotWorld) const
{
	outPivotWorld.set(0.0f, 0.0f, 0.0f);
	if (m_activeBackendOuterPat.valid() && m_activeBackendOuterPat->getNumChildren() >= 1)
	{
		osg::NodePath path = nodePathToSceneRootFromLeaf(m_activeBackendOuterPat.get());
		path.push_back(m_activeBackendOuterPat->getChild(0));
		const osg::Vec3d w = osg::Vec3d(0.0, 0.0, 0.0) * osg::computeLocalToWorld(path);
		outPivotWorld.set(static_cast<float>(w.x()), static_cast<float>(w.y()), static_cast<float>(w.z()));
		return;
	}
}

void OsgScene::logGizmoPivotDiagnostics(const char* reasonTag) const
{
	if (!gizmoPivotDiagEnabled())
	{
		return;
	}
	const char* const tag = (reasonTag && reasonTag[0] != '\0') ? reasonTag : "?";

	auto logLine = [](const std::string& line) {
		RunLogger::debug(line);
	};

	osg::Vec3f pivot{};
	computeGizmoPivotWorld(pivot);

	{
		std::ostringstream oss;
		oss << "[GizmoPivotDiag][" << tag << "] activeBackendId=" << m_activeBackendId << " transformGizmoFrame="
			<< (m_transformGizmoFrame == TransformGizmoFrame::World ? "World" : "Local");
		logLine(oss.str());
	}
	{
		std::ostringstream oss;
		oss << std::setprecision(8) << "  m_modelCenter=(" << m_modelCenter.x() << ',' << m_modelCenter.y() << ','
			<< m_modelCenter.z() << ')';
		logLine(oss.str());
	}
	{
		std::ostringstream oss;
		oss << std::setprecision(8) << "  pivotWorld(scene/file origin)=(" << pivot.x() << ',' << pivot.y()
			<< ',' << pivot.z() << ')';
		logLine(oss.str());
	}

	ObjectGizmoFrame gf;
	const bool haveGizmoFrame = readActiveObjectGizmoFrame(gf);
	if (haveGizmoFrame)
	{
		const osg::Vec3f cpp = gf.centerPlusPose();
		const osg::Quat sa = gf.attitude();
		std::ostringstream oss;
		oss << std::setprecision(8) << "  ObjectGizmoFrame center+pose=(" << cpp.x() << ',' << cpp.y() << ',' << cpp.z()
			<< ") quat=(" << sa.x() << ',' << sa.y() << ',' << sa.z() << ',' << sa.w() << ')';
		logLine(oss.str());
	}
	else
	{
		logLine("  ObjectGizmoFrame unreadable (no active outer or invalid inner branch)");
	}

	if (!m_activeBackendOuterPat.valid())
	{
		logLine("  m_activeBackendOuterPat=null (skip scene-graph file origin)");
		RunLogger::flush();
		return;
	}

	osg::MatrixTransform* const outer = m_activeBackendOuterPat.get();
	osg::Vec3d ot;
	osg::Quat oq;
	osg::Vec3d os;
	osg::Quat oso;
	outer->getMatrix().decompose(ot, oq, os, oso);
	{
		std::ostringstream oss;
		oss << std::setprecision(8) << "  outerLocal decompose t=(" << ot.x() << ',' << ot.y() << ',' << ot.z()
			<< ") quat=(" << oq.x() << ',' << oq.y() << ',' << oq.z() << ',' << oq.w() << ") scale=(" << os.x() << ','
			<< os.y() << ',' << os.z() << ")  (decompose t != center+pose when R≠I)";
		logLine(oss.str());
	}

	{
		osg::Quat oqDiag;
		osg::Vec3d otDiag;
		osg::Vec3d osDiag;
		osg::Quat osoDiag;
		outer->getMatrix().decompose(otDiag, oqDiag, osDiag, osoDiag);
		osg::Node* const innerForDiag = outer->getChild(0);
		osg::NodePath pathToInnerDiag = nodePathToSceneRootFromLeaf(outer);
		pathToInnerDiag.push_back(innerForDiag);
		const osg::Vec3d fileOriginDiag = osg::Vec3d(0.0, 0.0, 0.0) * osg::computeLocalToWorld(pathToInnerDiag);
		osg::NodePath pathOuterDiag = nodePathToSceneRootFromLeaf(outer);
		osg::Matrixd parentWorldDiag;
		if (pathOuterDiag.size() >= 2U)
		{
			osg::NodePath parentPath;
			parentPath.reserve(pathOuterDiag.size() - 1U);
			for (unsigned i = 0; i + 1U < pathOuterDiag.size(); ++i)
			{
				parentPath.push_back(pathOuterDiag[i]);
			}
			parentWorldDiag = osg::computeLocalToWorld(parentPath);
		}
		else
		{
			parentWorldDiag.makeIdentity();
		}
		const osg::Vec3d fileInOuterParentDiag = fileOriginDiag * osg::Matrixd::inverse(parentWorldDiag);
		const osg::Vec3d innerOffsetDiag(
			static_cast<double>(-m_modelCenter.x()),
			static_cast<double>(-m_modelCenter.y()),
			static_cast<double>(-m_modelCenter.z()));
		const osg::Vec3d rotatedOffDiag = innerOffsetDiag * osg::Matrixd::rotate(oqDiag);
		const osg::Vec3d centerPlusPoseDiag = fileInOuterParentDiag - rotatedOffDiag;
		std::ostringstream oss;
		oss << std::setprecision(8) << "  outer center+pose(recovered)=(" << centerPlusPoseDiag.x() << ','
			<< centerPlusPoseDiag.y() << ',' << centerPlusPoseDiag.z() << ')';
		logLine(oss.str());
		if (haveGizmoFrame)
		{
			const osg::Vec3f sp = gf.centerPlusPose();
			const double dcp = std::hypot(static_cast<double>(sp.x()) - centerPlusPoseDiag.x(),
				std::hypot(static_cast<double>(sp.y()) - centerPlusPoseDiag.y(),
					static_cast<double>(sp.z()) - centerPlusPoseDiag.z()));
			std::ostringstream oss2;
			oss2 << std::setprecision(8) << "  |frame.center+pose - outer center+pose(recovered)|=" << dcp;
			logLine(oss2.str());
		}
	}

	if (haveGizmoFrame)
	{
		const osg::Vec3f sp = gf.centerPlusPose();
		const osg::Quat sa = gf.attitude();
		const double dtp = std::hypot(static_cast<double>(sp.x()) - ot.x(),
			std::hypot(static_cast<double>(sp.y()) - ot.y(), static_cast<double>(sp.z()) - ot.z()));
		const double dq = std::abs(static_cast<double>(sa.x()) - oq.x()) + std::abs(static_cast<double>(sa.y()) - oq.y())
			+ std::abs(static_cast<double>(sa.z()) - oq.z()) + std::abs(static_cast<double>(sa.w()) - oq.w());
		std::ostringstream oss;
		oss << std::setprecision(8) << "  |frame.center+pose - outer.decompose.t|=" << dtp << "  quatL1diff=" << dq;
		logLine(oss.str());
	}

	if (outer->getNumChildren() < 1)
	{
		logLine("  outer has no children (skip file origin)");
		RunLogger::flush();
		return;
	}

	osg::Node* const inner0 = outer->getChild(0);
	osg::NodePath pathToInner = nodePathToSceneRootFromLeaf(outer);
	pathToInner.push_back(inner0);
	const osg::Matrixd innerToWorld = osg::computeLocalToWorld(pathToInner);
	const osg::Vec3d fileOriginWorld = osg::Vec3d(0.0, 0.0, 0.0) * innerToWorld;
	{
		std::ostringstream oss;
		oss << std::setprecision(8) << "  fileOriginWorld(inner local 0,0,0 -> world)=(" << fileOriginWorld.x() << ','
			<< fileOriginWorld.y() << ',' << fileOriginWorld.z() << ')';
		logLine(oss.str());
	}

	const osg::Vec3d pv(static_cast<double>(pivot.x()), static_cast<double>(pivot.y()), static_cast<double>(pivot.z()));
	const osg::Vec3d delta = pv - fileOriginWorld;
	{
		std::ostringstream oss;
		oss << std::setprecision(8) << "  delta pivot_minus_fileOrigin=(" << delta.x() << ',' << delta.y() << ','
			<< delta.z() << ") len=" << delta.length();
		logLine(oss.str());
	}

	if (osg::PositionAttitudeTransform* const innerPat = dynamic_cast<osg::PositionAttitudeTransform*>(inner0))
	{
		const osg::Vec3d ip(innerPat->getPosition().x(), innerPat->getPosition().y(), innerPat->getPosition().z());
		const osg::Vec3d negCenter(-static_cast<double>(m_modelCenter.x()), -static_cast<double>(m_modelCenter.y()),
			-static_cast<double>(m_modelCenter.z()));
		const osg::Vec3d innerMinusNegC = ip - negCenter;
		std::ostringstream oss;
		oss << std::setprecision(8) << "  innerPAT.position=(" << ip.x() << ',' << ip.y() << ',' << ip.z()
			<< ")  -m_modelCenter=(" << negCenter.x() << ',' << negCenter.y() << ',' << negCenter.z()
			<< ")  diff len=" << innerMinusNegC.length();
		logLine(oss.str());
	}
	else
	{
		std::ostringstream oss;
		oss << "  inner child[0] class=" << inner0->className()
			<< " (expected PositionAttitudeTransform for mesh center rebase)";
		logLine(oss.str());
	}

	if (m_compassTransform.valid())
	{
		const osg::NodePath pathComp = nodePathToSceneRootFromLeaf(m_compassTransform.get());
		const osg::Matrixd compassToWorld = osg::computeLocalToWorld(pathComp);
		const osg::Vec3d compassPatOriginWorld = osg::Vec3d(0.0, 0.0, 0.0) * compassToWorld;
		const osg::Vec3d dcp = compassPatOriginWorld - pv;
		std::ostringstream oss;
		oss << std::setprecision(8) << "  compassPAT_world_origin=(" << compassPatOriginWorld.x() << ','
			<< compassPatOriginWorld.y() << ',' << compassPatOriginWorld.z()
			<< ")  |compassOrigin - pivot|=" << dcp.length();
		logLine(oss.str());
	}
	else
	{
		logLine("  m_compassTransform=null");
	}

	RunLogger::flush();
}

bool OsgScene::computeCameraScreenRayWorld(double mouseX, double mouseY, osg::Vec3d& outRayOriginWorld, osg::Vec3d& outRayDirUnitWorld) const
{
	if (!m_viewer.valid() || !m_viewer->getCamera())
	{
		return false;
	}
	const osg::Camera* camera = m_viewer->getCamera();
	const int W = viewportWidth();
	const int H = viewportHeight();
	if (W <= 0 || H <= 0)
	{
		return false;
	}
	const double dpr = (m_devicePixelRatio > 0.0) ? m_devicePixelRatio : 1.0;
	const double mx = mouseX * dpr;
	const double my = mouseY * dpr;
	const osg::Matrixd invVP = osg::Matrixd::inverse(camera->getViewMatrix() * camera->getProjectionMatrix());
	const double clipX = 2.0 * mx / static_cast<double>(W) - 1.0;
	const double clipY = 1.0 - 2.0 * my / static_cast<double>(H);
	const osg::Vec4d n4(clipX, clipY, -1.0, 1.0);
	const osg::Vec4d f4(clipX, clipY, 1.0, 1.0);
	osg::Vec4d nw = n4 * invVP;
	osg::Vec4d fw = f4 * invVP;
	if (std::abs(nw.w()) < 1e-12 || std::abs(fw.w()) < 1e-12)
	{
		return false;
	}
	nw /= nw.w();
	fw /= fw.w();
	const osg::Vec3d rayNearW(nw.x(), nw.y(), nw.z());
	const osg::Vec3d rayFarW(fw.x(), fw.y(), fw.z());
	osg::Vec3d dir = rayFarW - rayNearW;
	const double len = dir.length();
	if (len < 1e-12)
	{
		return false;
	}
	outRayOriginWorld = rayNearW;
	outRayDirUnitWorld = dir / len;
	return true;
}
