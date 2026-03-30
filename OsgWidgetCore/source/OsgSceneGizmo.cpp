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

#include <algorithm>
#include <cmath>

#include <osg/GL>
#include <osg/BlendFunc>
#include <osg/Camera>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Node>
#include <osg/PolygonOffset>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osg/Shape>
#include <osg/StateSet>
#include <osg/StateAttribute>
#include <osg/Vec3>
#include <osg/Vec3d>
#include <osg/Vec4>
#include <osgViewer/Viewer>

osg::Node* OsgScene::createCompassNode()
{
	const float axisLen = 120.0f;
	const float headLen = 16.0f;
	const float headRadius = 6.0f;
	const float ringRadius = 65.0f;
	const int ringSegments = 72;

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	vertices->push_back(osg::Vec3(axisLen, 0.0f, 0.0f));
	vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	vertices->push_back(osg::Vec3(0.0f, axisLen, 0.0f));
	vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	vertices->push_back(osg::Vec3(0.0f, 0.0f, axisLen));

	m_compassColors = new osg::Vec4Array;
	m_compassColors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
	m_compassColors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
	m_compassColors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));

	osg::ref_ptr<osg::Geometry> axisGeom = new osg::Geometry;
	axisGeom->setVertexArray(vertices.get());
	axisGeom->setColorArray(m_compassColors.get(), osg::Array::BIND_PER_PRIMITIVE_SET);
	axisGeom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));
	axisGeom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 2, 2));
	axisGeom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 4, 2));

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(axisGeom.get());
	geode->getOrCreateStateSet()->setAttribute(new osg::LineWidth(5.0f));

	auto addArrow = [&](const osg::Vec3& center, const osg::Vec3& dir, const osg::Vec4& color) {
		osg::ref_ptr<osg::Cone> cone = new osg::Cone(center, headRadius, headLen);
		osg::Quat rot;
		rot.makeRotate(osg::Vec3(0.0f, 0.0f, 1.0f), dir);
		cone->setRotation(rot);
		osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(cone.get());
		shape->setColor(color);
		geode->addDrawable(shape.get());
	};

	addArrow(osg::Vec3(axisLen + headLen * 0.5f, 0.0f, 0.0f), osg::Vec3(1.0f, 0.0f, 0.0f), osg::Vec4(1.0f, 0.2f, 0.2f, 1.0f));
	addArrow(osg::Vec3(0.0f, axisLen + headLen * 0.5f, 0.0f), osg::Vec3(0.0f, 1.0f, 0.0f), osg::Vec4(0.2f, 1.0f, 0.2f, 1.0f));
	addArrow(osg::Vec3(0.0f, 0.0f, axisLen + headLen * 0.5f), osg::Vec3(0.0f, 0.0f, 1.0f), osg::Vec4(0.2f, 0.4f, 1.0f, 1.0f));

	auto addRing = [&](int plane, const osg::Vec4& color) {
		osg::ref_ptr<osg::Vec3Array> ringVertices = new osg::Vec3Array;
		for (int i = 0; i <= ringSegments; ++i)
		{
			const float a = osg::PI * 2.0f * static_cast<float>(i) / static_cast<float>(ringSegments);
			const float c = std::cos(a) * ringRadius;
			const float s = std::sin(a) * ringRadius;
			if (plane == 0) ringVertices->push_back(osg::Vec3(0.0f, c, s));
			if (plane == 1) ringVertices->push_back(osg::Vec3(c, 0.0f, s));
			if (plane == 2) ringVertices->push_back(osg::Vec3(c, s, 0.0f));
		}

		osg::ref_ptr<osg::Geometry> ringGeom = new osg::Geometry;
		ringGeom->setVertexArray(ringVertices.get());
		osg::ref_ptr<osg::Vec4Array> ringColor = new osg::Vec4Array;
		ringColor->push_back(color);
		ringGeom->setColorArray(ringColor.get(), osg::Array::BIND_OVERALL);
		if (plane == 0) m_ringColorX = ringColor;
		if (plane == 1) m_ringColorY = ringColor;
		if (plane == 2) m_ringColorZ = ringColor;
		ringGeom->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(ringVertices->size())));
		geode->addDrawable(ringGeom.get());
	};

	addRing(0, osg::Vec4(1.0f, 0.35f, 0.35f, 0.9f));
	addRing(1, osg::Vec4(0.35f, 1.0f, 0.35f, 0.9f));
	addRing(2, osg::Vec4(0.35f, 0.55f, 1.0f, 0.9f));

	osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), 7.0f);
	osg::ref_ptr<osg::ShapeDrawable> center = new osg::ShapeDrawable(sphere.get());
	center->setColor(osg::Vec4(1.0f, 1.0f, 1.0f, 0.45f));
	geode->addDrawable(center.get());

	osg::StateSet* ss = geode->getOrCreateStateSet();
	ss->setAttributeAndModes(new osg::PolygonOffset(-1.0f, -1.0f), osg::StateAttribute::ON);
	ss->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON);
	ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	osg::ref_ptr<osg::Depth> depth = new osg::Depth;
	depth->setFunction(osg::Depth::ALWAYS);
	depth->setWriteMask(false);
	ss->setAttributeAndModes(depth.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	return geode.release();
}

void OsgScene::attachCompassGraphics()
{
	static const unsigned int kMaskHelper = 0x2u;
	if (!m_compassTransform.valid() || !m_objectSelectionMode || !m_selectionActive)
	{
		return;
	}
	if (m_compassNode.valid())
	{
		m_compassTransform->setNodeMask(kMaskHelper);
		return;
	}
	m_compassNode = createCompassNode();
	m_compassTransform->addChild(m_compassNode.get());
	m_compassTransform->setNodeMask(kMaskHelper);
}

void OsgScene::detachCompassGraphics()
{
	if (m_compassTransform.valid() && m_compassNode.valid())
	{
		m_compassTransform->removeChild(m_compassNode.get());
	}
	m_compassNode = nullptr;
	m_compassColors = nullptr;
	m_ringColorX = nullptr;
	m_ringColorY = nullptr;
	m_ringColorZ = nullptr;
	if (m_compassTransform.valid())
	{
		m_compassTransform->setNodeMask(0u);
		m_compassTransform->setScale(osg::Vec3d(1.0, 1.0, 1.0));
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

void OsgScene::updateCompassHighlight(int axis)
{
	if (!m_compassColors.valid() || m_compassColors->size() < 3)
	{
		return;
	}

	(*m_compassColors)[0] = osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f);
	(*m_compassColors)[1] = osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f);
	(*m_compassColors)[2] = osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f);
	if (axis == kGizmoAxisX) (*m_compassColors)[0] = osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f);
	if (axis == kGizmoAxisY) (*m_compassColors)[1] = osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f);
	if (axis == kGizmoAxisZ) (*m_compassColors)[2] = osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f);
	m_compassColors->dirty();

	if (m_ringColorX.valid()) { (*m_ringColorX)[0] = osg::Vec4(1.0f, 0.35f, 0.35f, 0.9f); m_ringColorX->dirty(); }
	if (m_ringColorY.valid()) { (*m_ringColorY)[0] = osg::Vec4(0.35f, 1.0f, 0.35f, 0.9f); m_ringColorY->dirty(); }
	if (m_ringColorZ.valid()) { (*m_ringColorZ)[0] = osg::Vec4(0.35f, 0.55f, 1.0f, 0.9f); m_ringColorZ->dirty(); }

	if (axis == kGizmoAxisX && m_ringColorX.valid()) { (*m_ringColorX)[0] = osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f); m_ringColorX->dirty(); }
	if (axis == kGizmoAxisY && m_ringColorY.valid()) { (*m_ringColorY)[0] = osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f); m_ringColorY->dirty(); }
	if (axis == kGizmoAxisZ && m_ringColorZ.valid()) { (*m_ringColorZ)[0] = osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f); m_ringColorZ->dirty(); }

	if (m_compassNode.valid())
	{
		osg::Geode* geode = dynamic_cast<osg::Geode*>(m_compassNode.get());
		if (geode && geode->getNumDrawables() > 0)
		{
			osg::Drawable* drawable = geode->getDrawable(0);
			if (drawable)
			{
				drawable->dirtyDisplayList();
				drawable->dirtyBound();
			}
		}
	}
}

void OsgScene::updateCompassScale()
{
	if (!m_compassTransform.valid() || !m_viewer.valid() || !m_viewer->getCamera() || !m_selectionActive
		|| !m_objectSelectionMode || !m_compassNode.valid())
	{
		return;
	}

	osg::Vec3d eye, center, up;
	m_viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
	const osg::Vec3d anchor = m_selectedTransform.valid()
		? osg::Vec3d(m_selectedTransform->getPosition())
		: center;
	const double distance = (eye - anchor).length();

	if (m_gizmoReferenceDistance < 0.0 || m_gizmoReferenceDistance <= 1e-6)
	{
		m_gizmoReferenceDistance = std::max(1.0, distance);
		const double desiredAxisWorld = std::max(20.0, static_cast<double>(m_activeModelDiagonal) * 0.08);
		m_gizmoReferenceScale = std::max(0.4, std::min(800.0, desiredAxisWorld / 120.0));
	}
	double scale = m_gizmoReferenceScale * (distance / m_gizmoReferenceDistance);
	scale = std::max(0.3, std::min(1200.0, scale));
	m_compassTransform->setScale(osg::Vec3d(scale, scale, scale));
}

int OsgScene::pickAxisAtScreenPos(double mouseX, double mouseY, bool preferRing) const
{
	if (!m_selectedTransform.valid() || !m_viewer.valid() || !m_viewer->getCamera()
		|| viewportWidth() <= 0 || viewportHeight() <= 0)
	{
		return kGizmoAxisNone;
	}
	if (!m_objectSelectionMode || !m_compassNode.valid())
	{
		return kGizmoAxisNone;
	}

	float gizmoScale = 1.0f;
	if (m_compassTransform.valid())
	{
		const osg::Vec3d& sc = m_compassTransform->getScale();
		gizmoScale = static_cast<float>(std::max(sc.x(), std::max(sc.y(), sc.z())));
		if (gizmoScale < 1e-6f)
		{
			gizmoScale = 1.0f;
		}
	}
	const float axisLen = 120.0f * gizmoScale;
	const float ringRadius = 65.0f * gizmoScale;

	osg::Camera* camera = m_viewer->getCamera();
	const osg::Matrixd mvp = camera->getViewMatrix() * camera->getProjectionMatrix();
	const osg::Vec3f origin = m_selectedTransform->getPosition();
	const osg::Quat attitude = m_selectedTransform->getAttitude();

	auto toWorld = [&](const osg::Vec3f& local) -> osg::Vec3f {
		return origin + (attitude * local);
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

	const double dx = distanceToSegment(ox, oy, pxx, pxy, mouseX, mouseY);
	const double dy = distanceToSegment(ox, oy, pyx, pyy, mouseX, mouseY);
	const double dz = distanceToSegment(ox, oy, pzx, pzy, mouseX, mouseY);

	const double threshold = 24.0;
	const double ringThreshold = 18.0;

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
			const double d = distanceToSegment(s0x, s0y, s1x, s1y, mouseX, mouseY);
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
		if (ringAxis != kGizmoAxisNone) return ringAxis;
	}

	double minDist = threshold;
	int axis = kGizmoAxisNone;
	if (dx < minDist) { minDist = dx; axis = kGizmoAxisX; }
	if (dy < minDist) { minDist = dy; axis = kGizmoAxisY; }
	if (dz < minDist) { minDist = dz; axis = kGizmoAxisZ; }
	return axis;
}
