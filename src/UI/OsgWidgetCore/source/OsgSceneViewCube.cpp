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
#include <vector>

#include <osg/BlendFunc>
#include <osg/Camera>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/Image>
#include <osg/MatrixTransform>
#include <osg/PolygonOffset>
#include <osg/StateSet>
#include <osg/Texture2D>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osgViewer/Viewer>

namespace {

class ViewCubePickData : public osg::Referenced
{
public:
	ViewCubePickData(const osg::Vec3d& eyeDir, const osg::Vec3d& upHint)
		: m_eyeDir(eyeDir)
		, m_upHint(upHint)
	{
	}

	osg::Vec3d m_eyeDir;
	osg::Vec3d m_upHint;
};

class ViewCubeHudUpdateCallback : public osg::NodeCallback
{
public:
	explicit ViewCubeHudUpdateCallback(osg::Camera* mainCamera)
		: m_mainCamera(mainCamera)
	{
	}

	void operator()(osg::Node* node, osg::NodeVisitor* nv) override
	{
		auto* hud = static_cast<osg::Camera*>(node);
		if (m_mainCamera.valid())
		{
			const osg::Matrix& V = m_mainCamera->getViewMatrix();
			const osg::Quat q = V.getRotate();
			hud->setViewMatrix(osg::Matrix::rotate(q));
		}
		traverse(node, nv);
	}

private:
	osg::observer_ptr<osg::Camera> m_mainCamera;
};

osg::Vec3d pickUpHintForViewDir(const osg::Vec3d& dir)
{
	osg::Vec3d d = dir;
	d.normalize();
	if (std::abs(d.z()) > 0.92)
	{
		return osg::Vec3d(0.0, 1.0, 0.0);
	}
	return osg::Vec3d(0.0, 0.0, 1.0);
}

void applyLabelStateSet(osg::StateSet* ss)
{
	ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	ss->setMode(GL_BLEND, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	ss->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	ss->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
		osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	osg::ref_ptr<osg::Depth> depth = new osg::Depth;
	depth->setFunction(osg::Depth::ALWAYS);
	depth->setWriteMask(false);
	ss->setAttributeAndModes(depth.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	ss->setRenderBinDetails(25, "RenderBin");
}

void applyFaceStateSet(osg::StateSet* ss)
{
	ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	ss->setMode(GL_BLEND, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	ss->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
		osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	ss->setAttribute(new osg::PolygonOffset(1.0f, 1.0f));
}

osg::Geode* makeFaceGeode(const std::vector<osg::Vec3>& quad, const osg::Vec4& color,
	const osg::Vec3d& eyeDir, const osg::Vec3d& upHint)
{
	if (quad.size() != 4)
	{
		return nullptr;
	}

	osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
	for (const osg::Vec3& v : quad)
	{
		verts->push_back(v);
	}
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back(color);

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray(verts.get());
	geom->setColorArray(colors.get(), osg::Array::BIND_OVERALL);
	geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));
	applyFaceStateSet(geom->getOrCreateStateSet());

	osg::ref_ptr<ViewCubePickData> pickData = new ViewCubePickData(eyeDir, upHint);
	geom->setUserData(pickData.get());

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geom.get());
	geode->setUserData(pickData.get());
	return geode.release();
}

osg::Node* makeFaceLabelQuad(const osg::Vec3& anchor, const osg::Vec3& outwardNormal, osg::Image* image)
{
	if (!image || image->s() <= 0 || image->t() <= 0)
	{
		return nullptr;
	}

	osg::Vec3 n = outwardNormal;
	n.normalize();
	osg::Vec3 refUp(0.0f, 0.0f, 1.0f);
	if (std::abs(n * refUp) > 0.85f)
	{
		refUp.set(0.0f, 1.0f, 0.0f);
	}
	osg::Vec3 tangent = n ^ refUp;
	if (tangent.length2() < 1e-6f)
	{
		refUp.set(1.0f, 0.0f, 0.0f);
		tangent = n ^ refUp;
	}
	tangent.normalize();
	osg::Vec3 upOnFace = tangent ^ n;
	upOnFace.normalize();

	const float aspect = static_cast<float>(image->s()) / static_cast<float>(image->t());
	const float halfH = 0.20f;
	const float halfW = halfH * aspect;

	osg::ref_ptr<osg::MatrixTransform> labelXform = new osg::MatrixTransform;
	labelXform->setMatrix(osg::Matrix(
		tangent.x(), tangent.y(), tangent.z(), 0.0,
		upOnFace.x(), upOnFace.y(), upOnFace.z(), 0.0,
		n.x(), n.y(), n.z(), 0.0,
		0.0, 0.0, 0.0, 1.0) * osg::Matrix::translate(anchor));

	osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
	verts->push_back(osg::Vec3(-halfW, -halfH, 0.0f));
	verts->push_back(osg::Vec3(halfW, -halfH, 0.0f));
	verts->push_back(osg::Vec3(halfW, halfH, 0.0f));
	verts->push_back(osg::Vec3(-halfW, halfH, 0.0f));

	osg::ref_ptr<osg::Vec2Array> uvs = new osg::Vec2Array;
	uvs->push_back(osg::Vec2(0.0f, 1.0f));
	uvs->push_back(osg::Vec2(1.0f, 1.0f));
	uvs->push_back(osg::Vec2(1.0f, 0.0f));
	uvs->push_back(osg::Vec2(0.0f, 0.0f));

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray(verts.get());
	geom->setTexCoordArray(0, uvs.get());
	geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

	osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D(image);
	texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
	texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
	texture->setResizeNonPowerOfTwoHint(false);
	texture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
	texture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);

	osg::StateSet* ss = geom->getOrCreateStateSet();
	ss->setTextureAttributeAndModes(0, texture.get(), osg::StateAttribute::ON);
	applyLabelStateSet(ss);

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geom.get());
	labelXform->addChild(geode.get());
	return labelXform.release();
}

osg::Vec3 quadCenter(const std::vector<osg::Vec3>& quad)
{
	osg::Vec3 center(0.0f, 0.0f, 0.0f);
	for (const osg::Vec3& v : quad)
	{
		center += v;
	}
	return center * 0.25f;
}

void addFace(osg::Group* facesRoot, const std::vector<osg::Vec3>& quad, const osg::Vec3d& eyeDir)
{
	const osg::Vec4 faceColor(0.90f, 0.92f, 0.95f, 0.50f);
	if (osg::Geode* face = makeFaceGeode(quad, faceColor, eyeDir, pickUpHintForViewDir(eyeDir)))
	{
		facesRoot->addChild(face);
	}
}

osg::Node* createViewCubeFacesNode()
{
	const float h = 0.58f;
	osg::ref_ptr<osg::Group> root = new osg::Group;

	addFace(root.get(), {osg::Vec3(-h, -h, h), osg::Vec3(h, -h, h), osg::Vec3(h, h, h), osg::Vec3(-h, h, h)},
		osg::Vec3d(0.0, 0.0, 1.0));
	addFace(root.get(), {osg::Vec3(-h, h, -h), osg::Vec3(h, h, -h), osg::Vec3(h, -h, -h), osg::Vec3(-h, -h, -h)},
		osg::Vec3d(0.0, 0.0, -1.0));
	addFace(root.get(), {osg::Vec3(-h, h, -h), osg::Vec3(-h, h, h), osg::Vec3(h, h, h), osg::Vec3(h, h, -h)},
		osg::Vec3d(0.0, 1.0, 0.0));
	addFace(root.get(), {osg::Vec3(h, -h, -h), osg::Vec3(h, -h, h), osg::Vec3(-h, -h, h), osg::Vec3(-h, -h, -h)},
		osg::Vec3d(0.0, -1.0, 0.0));
	addFace(root.get(), {osg::Vec3(h, -h, -h), osg::Vec3(h, h, -h), osg::Vec3(h, h, h), osg::Vec3(h, -h, h)},
		osg::Vec3d(1.0, 0.0, 0.0));
	addFace(root.get(), {osg::Vec3(-h, h, -h), osg::Vec3(-h, -h, -h), osg::Vec3(-h, -h, h), osg::Vec3(-h, h, h)},
		osg::Vec3d(-1.0, 0.0, 0.0));

	return root.release();
}

struct ViewCubeLabelFaceDef
{
	osg::Vec3 center;
	osg::Vec3 normal;
	int imageIndex;
};

const ViewCubeLabelFaceDef kViewCubeLabelFaces[] = {
	{osg::Vec3(0.0f, 0.0f, 0.58f), osg::Vec3(0.0f, 0.0f, 1.0f), 0},
	{osg::Vec3(0.0f, 0.0f, -0.58f), osg::Vec3(0.0f, 0.0f, -1.0f), 1},
	{osg::Vec3(0.0f, 0.58f, 0.0f), osg::Vec3(0.0f, 1.0f, 0.0f), 2},
	{osg::Vec3(0.0f, -0.58f, 0.0f), osg::Vec3(0.0f, -1.0f, 0.0f), 3},
	{osg::Vec3(0.58f, 0.0f, 0.0f), osg::Vec3(1.0f, 0.0f, 0.0f), 4},
	{osg::Vec3(-0.58f, 0.0f, 0.0f), osg::Vec3(-1.0f, 0.0f, 0.0f), 5},
};

bool containsViewCubeLogicalRect(int logicalX, int logicalY, int widgetWidth, int margin, int size)
{
	const int x0 = widgetWidth - margin - size;
	const int y0 = margin;
	return logicalX >= x0 && logicalX <= x0 + size && logicalY >= y0 && logicalY <= y0 + size;
}

} // namespace

void OsgScene::initViewCubeHud()
{
	if (!m_viewer.valid() || !m_viewer->getCamera() || !m_graphicsWindow.valid() || !m_root.valid())
	{
		return;
	}

	m_viewCubeHudCamera = new osg::Camera;
	m_viewCubeHudCamera->setGraphicsContext(m_graphicsWindow.get());
	m_viewCubeHudCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	m_viewCubeHudCamera->setRenderOrder(osg::Camera::POST_RENDER);
	m_viewCubeHudCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
	m_viewCubeHudCamera->setAllowEventFocus(false);
	m_viewCubeHudCamera->setProjectionMatrixAsOrtho(-1.45f, 1.45f, -1.45f, 1.45f, -10.0, 10.0);
	m_viewCubeHudCamera->setCullMask(0xffffffffu);
	m_viewCubeHudCamera->setUpdateCallback(new ViewCubeHudUpdateCallback(m_viewer->getCamera()));

	osg::ref_ptr<osg::Group> hudRoot = new osg::Group;
	hudRoot->addChild(createViewCubeFacesNode());
	m_viewCubeLabelsGroup = new osg::Group;
	hudRoot->addChild(m_viewCubeLabelsGroup.get());
	m_viewCubeHudCamera->addChild(hudRoot.get());
	m_root->addChild(m_viewCubeHudCamera.get());
	const double initDpr = (m_devicePixelRatio > 0.0) ? m_devicePixelRatio : 1.0;
	updateViewCubeHudViewport(
		static_cast<int>(std::lround(static_cast<double>(m_viewportWidth) * initDpr)),
		static_cast<int>(std::lround(static_cast<double>(m_viewportHeight) * initDpr)));
}

void OsgScene::applyViewCubeFaceLabelImages(const osg::ref_ptr<osg::Image> images[6])
{
	if (!m_viewCubeLabelsGroup.valid())
	{
		return;
	}

	m_viewCubeLabelsGroup->removeChildren(0, m_viewCubeLabelsGroup->getNumChildren());

	for (const ViewCubeLabelFaceDef& face : kViewCubeLabelFaces)
	{
		if (face.imageIndex < 0 || face.imageIndex >= 6 || !images[face.imageIndex].valid())
		{
			continue;
		}

		osg::Vec3 n = face.normal;
		n.normalize();
		const osg::Vec3 anchor = face.center + n * 0.04f;
		if (osg::Node* quad = makeFaceLabelQuad(anchor, face.normal, images[face.imageIndex].get()))
		{
			m_viewCubeLabelsGroup->addChild(quad);
		}
	}

	requestRedraw();
}

void OsgScene::updateViewCubeHudViewport(int framebufferWidth, int framebufferHeight)
{
	if (!m_viewCubeHudCamera.valid())
	{
		return;
	}
	const HudCornerViewport vp = computeHudCornerViewport(
		framebufferWidth, framebufferHeight, m_viewCubeHudMargin, m_viewCubeHudSize, true);
	m_viewCubeHudEffectiveSize = vp.effectiveLogicalSize;
	m_viewCubeHudCamera->setViewport(vp.x, vp.y, vp.width, vp.height);
	applyHudSquareOrthoProjection(m_viewCubeHudCamera.get(), 1.45f, vp.width, vp.height);
}

bool OsgScene::tryPickViewCubeAtLogicalMouse(double logicalX, double logicalY)
{
	if (!m_viewCubeHudCamera.valid() || !m_viewer.valid())
	{
		return false;
	}
	if (!containsViewCubeLogicalRect(static_cast<int>(logicalX), static_cast<int>(logicalY), viewportWidth(),
			m_viewCubeHudMargin, m_viewCubeHudEffectiveSize))
	{
		return false;
	}

	double windowX = 0.0;
	double windowY = 0.0;
	logicalMouseToPickWindowCoords(logicalX, logicalY, windowX, windowY);

	osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
		new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, windowX, windowY);
	intersector->setIntersectionLimit(osgUtil::Intersector::LIMIT_NEAREST);
	osgUtil::IntersectionVisitor iv(intersector.get());
	m_viewCubeHudCamera->accept(iv);
	if (!intersector->containsIntersections())
	{
		return false;
	}

	const osgUtil::LineSegmentIntersector::Intersection& hit = intersector->getFirstIntersection();
	for (auto it = hit.nodePath.rbegin(); it != hit.nodePath.rend(); ++it)
	{
		osg::Node* node = *it;
		if (!node)
		{
			continue;
		}
		auto* pick = dynamic_cast<ViewCubePickData*>(node->getUserData());
		if (pick)
		{
			setCameraViewDirection(pick->m_eyeDir, pick->m_upHint);
			return true;
		}
	}
	return false;
}
