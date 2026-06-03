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
#include <array>
#include <cstdint>
#include <sstream>
#include <limits>
#include <queue>
#include <cmath>
#include <cfloat>
#include <vector>

#include <osg/GL>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/Light>
#include <osg/View>
#include <osg/LineWidth>
#include <osg/Matrix>
#include <osg/MatrixTransform>
#include <osg/NodeCallback>
#include <osg/PrimitiveSet>
#include <osg/AutoTransform>
#include <osg/ShapeDrawable>
#include <osg/Shape>
#include <osgText/Text>
#include <osg/StateSet>
#include <osg/Transform>
#include <osg/Drawable>
#include <osg/NodeVisitor>
#include <osg/Vec3d>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>
#include <osgViewer/GraphicsWindow>
#include <osgViewer/Viewer>

namespace {

class WorldAxesHudUpdateCallback : public osg::NodeCallback
{
public:
	explicit WorldAxesHudUpdateCallback(osg::Camera* mainCamera)
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

osg::Node* createWorldAxesHudGeode()
{
	const float L = 1.0f;
	const float arrowH = 0.17f;
	const float arrowR = 0.07f;
	const float shaftEnd = L - arrowH;

	osg::ref_ptr<osg::Group> root = new osg::Group;

	const osg::Vec4 cx0(0.98f, 0.28f, 0.28f, 1.0f);
	const osg::Vec4 cx1(0.98f, 0.52f, 0.48f, 1.0f);
	const osg::Vec4 cy0(0.28f, 0.96f, 0.38f, 1.0f);
	const osg::Vec4 cy1(0.48f, 1.0f, 0.58f, 1.0f);
	const osg::Vec4 cz0(0.32f, 0.52f, 1.0f, 1.0f);
	const osg::Vec4 cz1(0.55f, 0.72f, 1.0f, 1.0f);

	{
		osg::ref_ptr<osg::Vec3Array> vLine = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec4Array> cLine = new osg::Vec4Array;
		auto shaft = [&](const osg::Vec3& dir, const osg::Vec4& c0, const osg::Vec4& c1) {
			vLine->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
			vLine->push_back(osg::Vec3(dir.x() * shaftEnd, dir.y() * shaftEnd, dir.z() * shaftEnd));
			cLine->push_back(c0);
			cLine->push_back(c1);
		};
		shaft(osg::Vec3(1.0f, 0.0f, 0.0f), cx0, cx1);
		shaft(osg::Vec3(0.0f, 1.0f, 0.0f), cy0, cy1);
		shaft(osg::Vec3(0.0f, 0.0f, 1.0f), cz0, cz1);

		osg::ref_ptr<osg::Geometry> gLine = new osg::Geometry;
		gLine->setVertexArray(vLine.get());
		gLine->setColorArray(cLine.get(), osg::Array::BIND_PER_VERTEX);
		gLine->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, static_cast<GLsizei>(vLine->size())));

		osg::ref_ptr<osg::Geode> geodeLines = new osg::Geode;
		geodeLines->addDrawable(gLine.get());
		osg::StateSet* ssLine = geodeLines->getOrCreateStateSet();
		ssLine->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
		ssLine->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
		ssLine->setAttribute(new osg::LineWidth(3.0f));
		root->addChild(geodeLines.get());
	}

	{
		osg::ref_ptr<osg::Vec3Array> vTri = new osg::Vec3Array;
		osg::ref_ptr<osg::Vec4Array> cTri = new osg::Vec4Array;
		const float se = shaftEnd;
		const float r = arrowR;

		auto addPyramidX = [&](const osg::Vec4& col) {
			const osg::Vec3 apex(L, 0.0f, 0.0f);
			const osg::Vec3 b0(se, r, r);
			const osg::Vec3 b1(se, -r, r);
			const osg::Vec3 b2(se, -r, -r);
			const osg::Vec3 b3(se, r, -r);
			auto tri = [&](const osg::Vec3& a, const osg::Vec3& b, const osg::Vec3& c) {
				vTri->push_back(a);
				vTri->push_back(b);
				vTri->push_back(c);
				cTri->push_back(col);
				cTri->push_back(col);
				cTri->push_back(col);
			};
			tri(apex, b0, b1);
			tri(apex, b1, b2);
			tri(apex, b2, b3);
			tri(apex, b3, b0);
		};
		auto addPyramidY = [&](const osg::Vec4& col) {
			const osg::Vec3 apex(0.0f, L, 0.0f);
			const osg::Vec3 b0(r, se, r);
			const osg::Vec3 b1(-r, se, r);
			const osg::Vec3 b2(-r, se, -r);
			const osg::Vec3 b3(r, se, -r);
			auto tri = [&](const osg::Vec3& a, const osg::Vec3& b, const osg::Vec3& c) {
				vTri->push_back(a);
				vTri->push_back(b);
				vTri->push_back(c);
				cTri->push_back(col);
				cTri->push_back(col);
				cTri->push_back(col);
			};
			tri(apex, b0, b1);
			tri(apex, b1, b2);
			tri(apex, b2, b3);
			tri(apex, b3, b0);
		};
		auto addPyramidZ = [&](const osg::Vec4& col) {
			const osg::Vec3 apex(0.0f, 0.0f, L);
			const osg::Vec3 b0(r, r, se);
			const osg::Vec3 b1(-r, r, se);
			const osg::Vec3 b2(-r, -r, se);
			const osg::Vec3 b3(r, -r, se);
			auto tri = [&](const osg::Vec3& a, const osg::Vec3& b, const osg::Vec3& c) {
				vTri->push_back(a);
				vTri->push_back(b);
				vTri->push_back(c);
				cTri->push_back(col);
				cTri->push_back(col);
				cTri->push_back(col);
			};
			tri(apex, b0, b1);
			tri(apex, b1, b2);
			tri(apex, b2, b3);
			tri(apex, b3, b0);
		};

		addPyramidX(osg::Vec4(0.95f, 0.22f, 0.22f, 1.0f));
		addPyramidY(osg::Vec4(0.22f, 0.92f, 0.32f, 1.0f));
		addPyramidZ(osg::Vec4(0.28f, 0.48f, 0.98f, 1.0f));

		osg::ref_ptr<osg::Geometry> gTri = new osg::Geometry;
		gTri->setVertexArray(vTri.get());
		gTri->setColorArray(cTri.get(), osg::Array::BIND_PER_VERTEX);
		gTri->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vTri->size())));

		osg::ref_ptr<osg::Geode> geodeTri = new osg::Geode;
		geodeTri->addDrawable(gTri.get());
		osg::StateSet* ssTri = geodeTri->getOrCreateStateSet();
		ssTri->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
		ssTri->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
		ssTri->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
		root->addChild(geodeTri.get());
	}

	{
		osg::ref_ptr<osg::Sphere> sph = new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), 0.038f);
		osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(sph.get());
		sd->setColor(osg::Vec4(0.92f, 0.92f, 0.95f, 1.0f));
		osg::ref_ptr<osg::Geode> geodeS = new osg::Geode;
		geodeS->addDrawable(sd.get());
		osg::StateSet* ssS = geodeS->getOrCreateStateSet();
		ssS->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
		ssS->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
		root->addChild(geodeS.get());
	}

	return root.release();
}

} // namespace

OsgScene::OsgScene() = default;

void OsgScene::requestRedraw() const
{
	if (m_requestRedraw)
	{
		m_requestRedraw();
	}
}

void OsgScene::setViewportPixels(int w, int h)
{
	m_viewportWidth = (std::max)(1, w);
	m_viewportHeight = (std::max)(1, h);
}

void OsgScene::applyHeadlightToViewer(osgViewer::Viewer* viewer)
{
	if (!viewer || !m_headlight.valid())
	{
		return;
	}
	viewer->setLight(m_headlight.get());
	viewer->setLightingMode(osg::View::HEADLIGHT);
}

void OsgScene::initScene()
{
	static const unsigned int kMaskHelper = 0x2u;

	m_root = new osg::Group;
	m_root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	// 光源 0：在 initViewer 里通过 applyHeadlightToViewer 设为 View::HEADLIGHT，随主相机视点移动（固定管线头灯）
	// 主场景由 Viewer::setSceneData 挂接，不在 Camera 子图下；用 View 的头灯而非把 LightSource 挂到 Camera 上
	{
		m_headlight = new osg::Light;
		m_headlight->setLightNum(0);
		m_headlight->setAmbient(osg::Vec4(0.14f, 0.14f, 0.15f, 1.0f));
		m_headlight->setDiffuse(osg::Vec4(0.98f, 0.97f, 0.94f, 1.0f));
		m_headlight->setSpecular(osg::Vec4(0.45f, 0.45f, 0.42f, 1.0f));
	}

	m_sceneContentGroup = new osg::Group;
	m_sceneContentGroup->setName("SceneContent");
	m_sceneContentGroup->setNodeMask(0xffffffffu);

	m_backendObjectsGroup = new osg::Group;
	m_backendObjectsGroup->setName("BackendObjects");
	m_backendObjectsGroup->setNodeMask(0xffffffffu);
	// 不在此组上强制 GL_LIGHTING OFF：父级 OVERRIDE 会压过子节点「开启光照」，导致受光网格始终走无光照着色
	// 需要无光照的分支（点云、标注等）在各自节点上设置 OFF | OVERRIDE

	m_robotAssemblyGroup = new osg::Group;
	m_robotAssemblyGroup->setName("RobotAssembly");
	m_robotAssemblyGroup->setNodeMask(0xffffffffu);

	m_trajectoryOverlayGroup = new osg::Group;
	m_trajectoryOverlayGroup->setName("TrajectoryOverlay");
	m_trajectoryOverlayGroup->setNodeMask(0xffffffffu);

	m_tcpTeachSceneOverlayGroup = new osg::Group;
	m_tcpTeachSceneOverlayGroup->setName("TcpTeachSceneOverlay");
	m_tcpTeachSceneOverlayGroup->setNodeMask(0xffffffffu);
	m_tcpTeachSceneOverlayGroup->getOrCreateStateSet()->setMode(
		GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	m_tcpTeachSceneOverlayGroup->getOrCreateStateSet()->setMode(
		GL_COLOR_MATERIAL, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	m_tcpTeachSceneOverlayGroup->getOrCreateStateSet()->setMode(
		GL_FOG, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	m_trajectoryOverlayGroup->addChild(m_tcpTeachSceneOverlayGroup.get());

	m_stagingGroup = new osg::Group;
	m_stagingGroup->setNodeMask(0xffffffffu);
	m_stagingGroup->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	m_root->addChild(m_sceneContentGroup.get());
	m_root->addChild(m_stagingGroup.get());

	m_gizmoOverlayGroup = new osg::Group;
	m_gizmoOverlayGroup->setName("GizmoOverlay");
	m_gizmoOverlayGroup->setNodeMask(0xffffffffu);
	m_gizmoOverlayGroup->getOrCreateStateSet()->setMode(
		GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	m_gizmoOverlayGroup->getOrCreateStateSet()->setMode(
		GL_COLOR_MATERIAL, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	m_gizmoOverlayGroup->getOrCreateStateSet()->setMode(
		GL_FOG, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	m_compassTransform = new osg::PositionAttitudeTransform;
	m_compassTransform->setPosition(osg::Vec3d(0.0, 0.0, 0.0));
	m_compassTransform->setNodeMask(0u);
	m_gizmoOverlayGroup->addChild(m_compassTransform.get());

	m_pickFeedbackTransform = new osg::AutoTransform;
	m_pickFeedbackTransform->setNodeMask(kMaskHelper);
	m_pickFeedbackTransform->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
	m_pickFeedbackTransform->setAutoScaleToScreen(true);
	m_gizmoOverlayGroup->addChild(m_pickFeedbackTransform.get());

	m_annotationGroup = new osg::Group;
	m_annotationGroup->setName("Annotations");
	m_annotationGroup->setNodeMask(0xffffffffu);
	m_annotationGroup->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	// 顺序：标注 → 导入物 → 机器人 → 轨迹 overlay（后者后绘制，便于覆盖在场景几何之上）
	m_sceneContentGroup->addChild(m_annotationGroup.get());
	m_sceneContentGroup->addChild(m_backendObjectsGroup.get());
	m_sceneContentGroup->addChild(m_robotAssemblyGroup.get());
	m_sceneContentGroup->addChild(m_trajectoryOverlayGroup.get());

	m_meshPickOverlayGroup = new osg::Group;
	m_meshPickOverlayGroup->setNodeMask(0u);
	m_root->addChild(m_meshPickOverlayGroup.get());

	m_featureCatalogOverlayGroup = new osg::Group;
	m_featureCatalogOverlayGroup->setName("FeatureCatalogOverlay");
	m_trajectoryOverlayGroup->addChild(m_featureCatalogOverlayGroup.get());

	m_meshPickedFaceGeom = new osg::Geometry;
	m_meshPickedFaceVertices = new osg::Vec3Array;
	m_meshPickedFaceVertices->reserve(3);
	m_meshPickedFaceVertices->push_back(osg::Vec3f(0.0f, 0.0f, 0.0f));
	m_meshPickedFaceVertices->push_back(osg::Vec3f(0.0f, 0.0f, 0.0f));
	m_meshPickedFaceVertices->push_back(osg::Vec3f(0.0f, 0.0f, 0.0f));
	m_meshPickedFaceGeom->setVertexArray(m_meshPickedFaceVertices.get());
	m_meshPickedFaceGeom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, 3));
	m_meshPickedFaceColors = new osg::Vec4Array;
	m_meshPickedFaceColors->push_back(osg::Vec4(1.0f, 1.0f, 0.1f, 0.9f));
	m_meshPickedFaceGeom->setColorArray(m_meshPickedFaceColors.get(), osg::Array::BIND_OVERALL);
	m_meshPickedFaceGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	m_meshPickedFaceGeom->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	m_meshPickedFaceGeom->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	m_meshPickedFaceGeom->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	m_meshPickedFaceGeom->getOrCreateStateSet()->setAttributeAndModes(
		new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA), osg::StateAttribute::ON);
	m_meshPickedFaceGeom->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

	m_meshPickedEdgeGeom = new osg::Geometry;
	m_meshPickedEdgeVertices = new osg::Vec3Array;
	m_meshPickedEdgeVertices->reserve(2);
	m_meshPickedEdgeVertices->push_back(osg::Vec3f(0.0f, 0.0f, 0.0f));
	m_meshPickedEdgeVertices->push_back(osg::Vec3f(0.0f, 0.0f, 0.0f));
	m_meshPickedEdgeGeom->setVertexArray(m_meshPickedEdgeVertices.get());
	m_meshPickedEdgeGeom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));
	m_meshPickedEdgeColors = new osg::Vec4Array;
	m_meshPickedEdgeColors->push_back(osg::Vec4(0.1f, 1.0f, 0.2f, 0.9f));
	m_meshPickedEdgeGeom->setColorArray(m_meshPickedEdgeColors.get(), osg::Array::BIND_OVERALL);
	m_meshPickedEdgeGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	m_meshPickedEdgeGeom->getOrCreateStateSet()->setAttribute(new osg::LineWidth(3.0f));

	osg::ref_ptr<osg::Geode> overlayGeode = new osg::Geode;
	overlayGeode->addDrawable(m_meshPickedFaceGeom.get());
	overlayGeode->addDrawable(m_meshPickedEdgeGeom.get());
	overlayGeode->setNodeMask(kMaskHelper);
	m_meshPickOverlayGroup->addChild(overlayGeode.get());

	m_selectionActive = false;
	m_gizmoReferenceDistance = -1.0;
	m_gizmoReferenceScale = 1.0;
	m_hasLastSelectionPose = false;
}

void OsgScene::initWorldAxesHud()
{
	if (!m_viewer.valid() || !m_viewer->getCamera() || !m_graphicsWindow.valid() || !m_root.valid())
	{
		return;
	}
	m_worldAxesHudCamera = new osg::Camera;
	m_worldAxesHudCamera->setGraphicsContext(m_graphicsWindow.get());
	m_worldAxesHudCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	m_worldAxesHudCamera->setRenderOrder(osg::Camera::POST_RENDER);
	m_worldAxesHudCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
	m_worldAxesHudCamera->setAllowEventFocus(false);
	m_worldAxesHudCamera->setProjectionMatrixAsOrtho(-1.2f, 1.2f, -1.2f, 1.2f, -10.0, 10.0);
	m_worldAxesHudCamera->setCullMask(0xffffffffu);
	m_worldAxesHudCamera->setUpdateCallback(new WorldAxesHudUpdateCallback(m_viewer->getCamera()));
	m_worldAxesHudCamera->addChild(createWorldAxesHudGeode());
	m_root->addChild(m_worldAxesHudCamera.get());
	updateWorldAxesHudViewport(m_viewportWidth, m_viewportHeight);
}

void OsgScene::updateWorldAxesHudViewport(int widgetWidth, int widgetHeight)
{
	(void)widgetWidth;
	(void)widgetHeight;
	if (!m_worldAxesHudCamera.valid())
	{
		return;
	}
	const int margin = 10;
	const int sz = 120;
	m_worldAxesHudCamera->setViewport(margin, margin, sz, sz);
}

void OsgScene::bindBackendVisualRoot(const std::string& backendId, osg::Node* rootNode)
{
	m_backendVisualBindings.bindBackendRoot(backendId, rootNode);
	m_backendPickIndexes.bindBackendRoot(backendId, rootNode);
}

void OsgScene::unbindBackendVisualRoot(const std::string& backendId)
{
	m_backendVisualBindings.unbindBackend(backendId);
	m_backendPickIndexes.unbindBackend(backendId);
}

void OsgScene::clearBackendVisualBindings()
{
	m_backendVisualBindings.clear();
	m_backendPickIndexes.clear();
}

bool OsgScene::resolveBackendIdFromPickedPath(const osg::NodePath& path, std::string& outBackendId) const
{
	return m_backendVisualBindings.resolveBackendIdFromNodePath(path, outBackendId);
}

bool OsgScene::pickAndActivateBackendAtScreenPos(double mouseX, double mouseY)
{
	static const unsigned int kMaskContent = 0x1u;
	if (!m_viewer.valid() || !m_viewer->getCamera() || !m_root.valid())
	{
		return false;
	}
	const double x = mouseX;
	const double y = static_cast<double>(viewportHeight()) - mouseY;
	osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
		new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, x, y);
	osgUtil::IntersectionVisitor iv(intersector.get());
	iv.setTraversalMask(kMaskContent);
	m_viewer->getCamera()->accept(iv);
	if (!intersector->containsIntersections())
	{
		return false;
	}
	for (const auto& hit : intersector->getIntersections())
	{
		const osg::NodePath& path = hit.nodePath;
		std::string id;
		if (!resolveBackendIdFromPickedPath(path, id))
		{
			continue;
		}
		auto rootIt = m_backendObjectRoots.find(id);
		if (rootIt == m_backendObjectRoots.end() || !rootIt->second.valid())
		{
			continue;
		}
		m_activeBackendId = id;
		m_activeBackendOuterPat = rootIt->second;
		attachGizmoOverlayToActiveBackend();
		cacheSelectionGizmoPose();
		auto cIt = m_backendModelCenters.find(id);
		m_modelCenter = (cIt != m_backendModelCenters.end()) ? cIt->second : osg::Vec3f(0.0f, 0.0f, 0.0f);
		return true;
	}
	return false;
}
void OsgScene::cachePickablePointsFromNode(osg::Node* node)
{
	m_pickablePointsLocal.clear();
	m_pickablePointsPreviewLocal.clear();
	m_pickablePointsCenteredLocal.clear();
	m_kdNodes.clear();
	m_kdRoot = -1;
	if (!m_activeBackendId.empty())
	{
		if (const BackendPickBundle* bundle = m_backendPickIndexes.find(m_activeBackendId))
		{
			if (!bundle->pointIndex.empty())
			{
				importPickSpatialIndexForActiveBackend(bundle->pointIndex);
				return;
			}
		}
	}
	if (!node)
	{
		return;
	}

	struct PointCollectVisitor : public osg::NodeVisitor
	{
		PointCollectVisitor(std::vector<osg::Vec3f>& out)
			: osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), points(out) {}

		void apply(osg::Geode& geode) override
		{
			const osg::Matrixd localToRoot = osg::computeLocalToWorld(this->getNodePath());
			for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
			{
				osg::Geometry* geom = geode.getDrawable(i) ? geode.getDrawable(i)->asGeometry() : nullptr;
				if (!geom || !geom->getVertexArray())
				{
					continue;
				}
				const osg::Vec3Array* vertices = dynamic_cast<const osg::Vec3Array*>(geom->getVertexArray());
				if (vertices)
				{
					points.reserve(points.size() + vertices->size());
					for (const osg::Vec3& v : *vertices)
					{
						const osg::Vec3d p = osg::Vec3d(v.x(), v.y(), v.z()) * localToRoot;
						points.push_back(osg::Vec3f(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())));
					}
					continue;
				}

				const osg::Vec3dArray* verticesD = dynamic_cast<const osg::Vec3dArray*>(geom->getVertexArray());
				if (verticesD)
				{
					points.reserve(points.size() + verticesD->size());
					for (const osg::Vec3d& v : *verticesD)
					{
						const osg::Vec3d p = v * localToRoot;
						points.push_back(osg::Vec3f(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())));
					}
				}
			}
			traverse(geode);
		}

		std::vector<osg::Vec3f>& points;
	};

	PointCollectVisitor collector(m_pickablePointsLocal);
	node->accept(collector);

	if (m_pickablePointsLocal.empty())
	{
		return;
	}
	try
	{
		m_pickablePointsCenteredLocal.reserve(m_pickablePointsLocal.size());
		for (const osg::Vec3f& p : m_pickablePointsLocal)
		{
			m_pickablePointsCenteredLocal.push_back(p - m_modelCenter);
		}
		rebuildPointKdTree();
	}
	catch (...)
	{
		m_pickablePointsCenteredLocal.clear();
		m_kdNodes.clear();
		m_kdRoot = -1;
	}
}

bool OsgScene::pickPointAtScreenPos(double mouseX, double mouseY, osg::Vec3f& outPointWorld) const
{
	double distancePx = 0.0;
	if (!pickNearestPointAtScreenPos(mouseX, mouseY, outPointWorld, distancePx, false))
	{
		return false;
	}
	return distancePx <= kPointPickHitRadiusPx;
}

bool OsgScene::pickNearestPointAtScreenPos(double mouseX, double mouseY, osg::Vec3f& outPointWorld, double& outDistancePx, bool previewOnly) const
{
	ObjectGizmoFrame gizmoFrame;
	if (!readActiveObjectGizmoFrame(gizmoFrame))
	{
		return false;
	}
	if (!m_viewer.valid() || !m_viewer->getCamera())
	{
		return false;
	}
	const bool hasKdCache = (m_kdRoot >= 0 && !m_pickablePointsCenteredLocal.empty());
	// hover 有点缓存时跳过射线，避免每帧全场景相交遍历
	if (!previewOnly || !hasKdCache)
	{
		double rayDistPx = 0.0;
		osg::Vec3f rayWorld;
		if (pickPointByRayIntersection(mouseX, mouseY, rayWorld, rayDistPx)
			&& rayDistPx <= kPointPickHitRadiusPx)
		{
			outPointWorld = rayWorld;
			outDistancePx = rayDistPx;
			return true;
		}
	}

	const std::vector<osg::Vec3f>& points = m_pickablePointsLocal;
	if (points.empty())
	{
		return false;
	}

	osg::Camera* camera = m_viewer->getCamera();
	const osg::Matrixd mvp = camera->getViewMatrix() * camera->getProjectionMatrix();
	const osg::Quat attitude = gizmoFrame.attitude();
	const osg::Vec3f selectedPos = gizmoFrame.centerPlusPose();

	// click 与 hover 共用全量点集 + KD
	if (m_kdRoot >= 0 && !m_pickablePointsCenteredLocal.empty())
	{
		osg::Matrixd proj = camera->getProjectionMatrix();
		osg::Matrixd view = camera->getViewMatrix();
		osg::Matrixd inv = osg::Matrixd::inverse(view * proj);
		const double xNdc = (2.0 * mouseX / static_cast<double>(viewportWidth())) - 1.0;
		const double yNdc = 1.0 - (2.0 * mouseY / static_cast<double>(viewportHeight()));
		osg::Vec3d clipNearW = osg::Vec3d(xNdc, yNdc, -1.0) * inv;
		osg::Vec3d clipFarW = osg::Vec3d(xNdc, yNdc, 1.0) * inv;
		osg::Vec3f rayOriginWorld(static_cast<float>(clipNearW.x()), static_cast<float>(clipNearW.y()), static_cast<float>(clipNearW.z()));
		osg::Vec3f rayDirWorld(static_cast<float>(clipFarW.x() - clipNearW.x()), static_cast<float>(clipFarW.y() - clipNearW.y()), static_cast<float>(clipFarW.z() - clipNearW.z()));
		if (rayDirWorld.length2() > 1e-8f)
		{
			rayDirWorld.normalize();
		}

		const osg::Quat invAtt = attitude.inverse();
		const osg::Vec3f rayOriginLocal = invAtt * (rayOriginWorld - selectedPos);
		const osg::Vec3f rayDirLocal = invAtt * rayDirWorld;
		const float t = -(rayOriginLocal * rayDirLocal);
		const osg::Vec3f queryLocal = rayOriginLocal + rayDirLocal * t;

		std::vector<int> candidateIndices;
		nearestCandidatesByKdTree(queryLocal, 96, candidateIndices);
		const double hitRadiusPx2 = kPointPickHitRadiusPx * kPointPickHitRadiusPx;
		const double depthLayerEps = 0.02;
		const double screenTiePx2 = 9.0;
		struct InRadCand
		{
			double d2;
			double depth;
			osg::Vec3f world;
		};
		InRadCand inRad[96];
		int inRadCount = 0;
		double bestFallbackD2 = (std::numeric_limits<double>::max)();
		double bestFallbackDepth = (std::numeric_limits<double>::max)();
		osg::Vec3f bestFallbackWorld;
		bool fallbackFound = false;
		for (int idx : candidateIndices)
		{
			if (idx < 0 || idx >= static_cast<int>(m_pickablePointsCenteredLocal.size()))
			{
				continue;
			}
			const osg::Vec3f localCentered = m_pickablePointsCenteredLocal[static_cast<std::size_t>(idx)];
			const osg::Vec3f world = selectedPos + (attitude * localCentered);
			const osg::Vec3d clip = osg::Vec3d(world) * mvp;
			if (clip.z() < -1.0 || clip.z() > 1.0)
			{
				continue;
			}
			const double sx = (clip.x() * 0.5 + 0.5) * static_cast<double>(viewportWidth());
			const double sy = (1.0 - (clip.y() * 0.5 + 0.5)) * static_cast<double>(viewportHeight());
			const double dx = sx - mouseX;
			const double dy = sy - mouseY;
			const double d2 = dx * dx + dy * dy;
			const double depth = clip.z();
			if (d2 <= hitRadiusPx2 && inRadCount < 96)
			{
				inRad[inRadCount++] = {d2, depth, world};
			}
			else if (!previewOnly
				&& (!fallbackFound
					|| d2 + screenTiePx2 < bestFallbackD2
					|| (std::abs(d2 - bestFallbackD2) <= screenTiePx2 && depth < bestFallbackDepth)))
			{
				fallbackFound = true;
				bestFallbackD2 = d2;
				bestFallbackDepth = depth;
				bestFallbackWorld = world;
			}
		}
		double frontInRadiusDepth = (std::numeric_limits<double>::max)();
		for (int i = 0; i < inRadCount; ++i)
		{
			frontInRadiusDepth = std::min(frontInRadiusDepth, inRad[i].depth);
		}
		double bestInRadiusD2 = (std::numeric_limits<double>::max)();
		osg::Vec3f bestInRadiusWorld;
		bool inRadiusFound = false;
		for (int i = 0; i < inRadCount; ++i)
		{
			if (frontInRadiusDepth < (std::numeric_limits<double>::max)()
				&& inRad[i].depth <= frontInRadiusDepth + depthLayerEps)
			{
				if (!inRadiusFound || inRad[i].d2 < bestInRadiusD2)
				{
					inRadiusFound = true;
					bestInRadiusD2 = inRad[i].d2;
					bestInRadiusWorld = inRad[i].world;
				}
			}
		}
		if (inRadiusFound)
		{
			outPointWorld = bestInRadiusWorld;
			outDistancePx = std::sqrt(bestInRadiusD2);
			return true;
		}
		if (fallbackFound && !previewOnly)
		{
			outPointWorld = bestFallbackWorld;
			outDistancePx = std::sqrt(bestFallbackD2);
			return true;
		}
		if (previewOnly)
		{
			return false;
		}
	}

	
	const double hitRadiusPx2 = kPointPickHitRadiusPx * kPointPickHitRadiusPx;
	const double depthLayerEps = 0.02;
	const double screenTiePx2 = 9.0;
	double frontInRadiusDepth = (std::numeric_limits<double>::max)();
	for (const osg::Vec3f& pLocal : points)
	{
		const osg::Vec3f world = selectedPos + (attitude * (pLocal - m_modelCenter));
		const osg::Vec3d clip = osg::Vec3d(world) * mvp;
		if (clip.z() < -1.0 || clip.z() > 1.0)
		{
			continue;
		}
		const double sx = (clip.x() * 0.5 + 0.5) * static_cast<double>(viewportWidth());
		const double sy = (1.0 - (clip.y() * 0.5 + 0.5)) * static_cast<double>(viewportHeight());
		const double dx = sx - mouseX;
		const double dy = sy - mouseY;
		const double d2 = dx * dx + dy * dy;
		if (d2 <= hitRadiusPx2)
		{
			frontInRadiusDepth = std::min(frontInRadiusDepth, clip.z());
		}
	}
	double bestInRadiusD2 = 1e30;
	osg::Vec3f bestInRadiusWorld;
	bool inRadiusFound = false;
	double bestFallbackD2 = 1e30;
	double bestFallbackDepth = 1e30;
	osg::Vec3f bestFallbackWorld;
	bool fallbackFound = false;
	for (const osg::Vec3f& pLocal : points)
	{
		const osg::Vec3f world = selectedPos + (attitude * (pLocal - m_modelCenter));
		const osg::Vec3d clip = osg::Vec3d(world) * mvp;
		if (clip.z() < -1.0 || clip.z() > 1.0)
		{
			continue;
		}
		const double sx = (clip.x() * 0.5 + 0.5) * static_cast<double>(viewportWidth());
		const double sy = (1.0 - (clip.y() * 0.5 + 0.5)) * static_cast<double>(viewportHeight());
		const double dx = sx - mouseX;
		const double dy = sy - mouseY;
		const double d2 = dx * dx + dy * dy;
		const double depth = clip.z();
		if (d2 <= hitRadiusPx2
			&& frontInRadiusDepth < (std::numeric_limits<double>::max)()
			&& depth <= frontInRadiusDepth + depthLayerEps)
		{
			if (!inRadiusFound || d2 < bestInRadiusD2)
			{
				inRadiusFound = true;
				bestInRadiusD2 = d2;
				bestInRadiusWorld = world;
			}
		}
		else if (!previewOnly
			&& (!fallbackFound
				|| d2 + screenTiePx2 < bestFallbackD2
				|| (std::abs(d2 - bestFallbackD2) <= screenTiePx2 && depth < bestFallbackDepth)))
		{
			fallbackFound = true;
			bestFallbackD2 = d2;
			bestFallbackDepth = depth;
			bestFallbackWorld = world;
		}
	}
	if (inRadiusFound)
	{
		outPointWorld = bestInRadiusWorld;
		outDistancePx = std::sqrt(bestInRadiusD2);
		return true;
	}
	if (fallbackFound && !previewOnly)
	{
		outPointWorld = bestFallbackWorld;
		outDistancePx = std::sqrt(bestFallbackD2);
		return true;
	}
	return false;
}

bool OsgScene::pickPointByRayIntersection(double mouseX, double mouseY, osg::Vec3f& outPointWorld, double& outDistancePx) const
{
	static const unsigned int kMaskContent = 0x1u;
	ObjectGizmoFrame gizmoFrame;
	const bool haveGizmoFrame = readActiveObjectGizmoFrame(gizmoFrame);
	if (!m_viewer.valid() || !m_viewer->getCamera() || !m_root.valid())
	{
		return false;
	}

	const double x = mouseX;
	const double y = static_cast<double>(viewportHeight()) - mouseY;
	osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
		new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, x, y);
	intersector->setIntersectionLimit(osgUtil::Intersector::LIMIT_NEAREST);
	osgUtil::IntersectionVisitor iv(intersector.get());
	iv.setTraversalMask(kMaskContent);
	m_viewer->getCamera()->accept(iv);

	if (!intersector->containsIntersections())
	{
		return false;
	}

	const osg::Matrixd mvp = m_viewer->getCamera()->getViewMatrix() * m_viewer->getCamera()->getProjectionMatrix();
	const osg::Quat att = haveGizmoFrame ? gizmoFrame.attitude() : osg::Quat();
	const osg::Quat invAtt = att.inverse();
	const osg::Vec3f selectedPos = haveGizmoFrame ? gizmoFrame.centerPlusPose() : osg::Vec3f(0.0f, 0.0f, 0.0f);

	auto projectToScreen = [&](const osg::Vec3f& world, double& d2, double& depth) -> bool
	{
		const osg::Vec3d clip = osg::Vec3d(world) * mvp;
		if (clip.z() < -1.0 || clip.z() > 1.0)
		{
			return false;
		}
		const double sx = (clip.x() * 0.5 + 0.5) * static_cast<double>(viewportWidth());
		const double sy = (1.0 - (clip.y() * 0.5 + 0.5)) * static_cast<double>(viewportHeight());
		const double dx = sx - mouseX;
		const double dy = sy - mouseY;
		d2 = dx * dx + dy * dy;
		depth = clip.z(); // smaller => visually closer to camera
		return true;
	};

	bool found = false;
	osg::Vec3f bestWorld(0.0f, 0.0f, 0.0f);
	double bestD2 = (std::numeric_limits<double>::max)();
	double bestDepth = (std::numeric_limits<double>::max)();
	const double kScreenWindowPx = kPointPickHitRadiusPx;
	const double kScreenWindowPx2 = kScreenWindowPx * kScreenWindowPx;
	const double kDepthTie = 1e-4;
	const bool hasKd = (m_kdRoot >= 0 && !m_pickablePointsCenteredLocal.empty());
	const bool useKdBody = haveGizmoFrame && hasKd;

	for (const auto& hit : intersector->getIntersections())
	{
		const osg::Vec3d wp = hit.getWorldIntersectPoint();
		const osg::Vec3f hitWorld(static_cast<float>(wp.x()), static_cast<float>(wp.y()), static_cast<float>(wp.z()));

		std::vector<int> candidateIndices;
		if (useKdBody)
		{
			const osg::Vec3f queryLocalCentered = invAtt * (hitWorld - selectedPos);
			nearestCandidatesByKdTree(queryLocalCentered, 64, candidateIndices);
		}

		// Fallback to the exact hit point if no candidate points are available.
		if (candidateIndices.empty())
		{
			double d2 = 0.0;
			double depth = 0.0;
			if (projectToScreen(hitWorld, d2, depth))
			{
				const bool inWindow = d2 <= kScreenWindowPx2;
				const bool bestInWindow = bestD2 <= kScreenWindowPx2;
				const bool better = !found
					|| (inWindow && !bestInWindow)
					|| (inWindow == bestInWindow && (depth + kDepthTie < bestDepth
						|| (std::abs(depth - bestDepth) <= kDepthTie && d2 < bestD2)));
				if (better)
				{
					found = true;
					bestWorld = hitWorld;
					bestD2 = d2;
					bestDepth = depth;
				}
			}
			continue;
		}

		for (int idx : candidateIndices)
		{
			if (idx < 0 || idx >= static_cast<int>(m_pickablePointsCenteredLocal.size()))
			{
				continue;
			}
			const osg::Vec3f localCentered = m_pickablePointsCenteredLocal[static_cast<std::size_t>(idx)];
			const osg::Vec3f candidateWorld = selectedPos + (att * localCentered);
			double d2 = 0.0;
			double depth = 0.0;
			if (!projectToScreen(candidateWorld, d2, depth))
			{
				continue;
			}
			const bool inWindow = d2 <= kScreenWindowPx2;
			const bool bestInWindow = bestD2 <= kScreenWindowPx2;
			const bool better = !found
				|| (inWindow && !bestInWindow)
				|| (inWindow == bestInWindow && (depth + kDepthTie < bestDepth
					|| (std::abs(depth - bestDepth) <= kDepthTie && d2 < bestD2)));
			if (better)
			{
				found = true;
				bestWorld = candidateWorld;
				bestD2 = d2;
				bestDepth = depth;
			}
		}
	}

	if (!found)
	{
		return false;
	}
	outPointWorld = bestWorld;
	outDistancePx = std::sqrt(bestD2);
	return true;
}

namespace {

static inline int64_t quantPickWeld(float v)
{
	return static_cast<int64_t>(std::llround(static_cast<double>(v) * 1000000.0));
}

struct PickWeldKey
{
	int64_t x, y, z;
	bool operator==(const PickWeldKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct PickWeldKeyHash
{
	size_t operator()(const PickWeldKey& k) const noexcept
	{
		size_t h = 1469598103934665603ull;
		h ^= static_cast<size_t>(k.x);
		h *= 1099511628211ull;
		h ^= static_cast<size_t>(k.y);
		h *= 1099511628211ull;
		h ^= static_cast<size_t>(k.z);
		h *= 1099511628211ull;
		return h;
	}
};

struct PickEdgeKey
{
	std::uint32_t lo, hi;
	bool operator==(const PickEdgeKey& o) const { return lo == o.lo && hi == o.hi; }
};

struct PickEdgeKeyHash
{
	size_t operator()(const PickEdgeKey& k) const noexcept
	{
		return (static_cast<size_t>(k.lo) << 32) | static_cast<size_t>(k.hi);
	}
};

static osg::Vec3f pickVertexLocalToWorld(const osg::Vec3& vLocal, const osg::Matrixd* m)
{
	if (m)
	{
		const osg::Vec3d vw = osg::Vec3d(vLocal.x(), vLocal.y(), vLocal.z()) * (*m);
		return osg::Vec3f(static_cast<float>(vw.x()), static_cast<float>(vw.y()), static_cast<float>(vw.z()));
	}
	return osg::Vec3f(vLocal.x(), vLocal.y(), vLocal.z());
}

// 从命中三角沿共享边合并共面邻接三角（面拾取/高亮）
static void expandCoplanarTrianglesFromSoupLocal(
	const std::vector<osg::Vec3>& verts,
	const osg::Matrixd* m,
	std::uint32_t seedTri,
	std::vector<osg::Vec3f>& outVertsWorld)
{
	outVertsWorld.clear();
	const std::uint32_t nTri = static_cast<std::uint32_t>(verts.size() / 3U);
	if (nTri == 0U || seedTri >= nTri)
	{
		return;
	}

	std::unordered_map<PickWeldKey, std::uint32_t, PickWeldKeyHash> weld;
	weld.reserve(static_cast<std::size_t>(nTri) * 2U);
	std::vector<osg::Vec3> welded;
	welded.reserve(verts.size() / 2U);
	std::vector<std::array<std::uint32_t, 3>> triIdx(static_cast<std::size_t>(nTri));

	auto addVertex = [&](const osg::Vec3& p) -> std::uint32_t
	{
		const PickWeldKey k{ quantPickWeld(p.x()), quantPickWeld(p.y()), quantPickWeld(p.z()) };
		const auto it = weld.find(k);
		if (it != weld.end())
		{
			return it->second;
		}
		const std::uint32_t id = static_cast<std::uint32_t>(welded.size());
		welded.push_back(p);
		weld.emplace(k, id);
		return id;
	};

	for (std::uint32_t t = 0; t < nTri; ++t)
	{
		const std::size_t b = static_cast<std::size_t>(t) * 3U;
		triIdx[t][0] = addVertex(verts[b + 0]);
		triIdx[t][1] = addVertex(verts[b + 1]);
		triIdx[t][2] = addVertex(verts[b + 2]);
	}

	std::unordered_map<PickEdgeKey, std::vector<std::uint32_t>, PickEdgeKeyHash> edgeTris;
	edgeTris.reserve(static_cast<std::size_t>(nTri) * 2U);

	auto addEdge = [&](std::uint32_t a, std::uint32_t b, std::uint32_t triId)
	{
		if (a == b)
		{
			return;
		}
		const PickEdgeKey ek{ (std::min)(a, b), (std::max)(a, b) };
		edgeTris[ek].push_back(triId);
	};

	for (std::uint32_t t = 0; t < nTri; ++t)
	{
		const auto& tr = triIdx[t];
		const auto tid = t;
		addEdge(tr[0], tr[1], tid);
		addEdge(tr[1], tr[2], tid);
		addEdge(tr[2], tr[0], tid);
	}

	std::vector<osg::Vec3f> triNor(static_cast<std::size_t>(nTri));
	float minx = welded[0].x(), maxx = welded[0].x();
	float miny = welded[0].y(), maxy = welded[0].y();
	float minz = welded[0].z(), maxz = welded[0].z();
	for (const osg::Vec3& p : welded)
	{
		minx = (std::min)(minx, p.x());
		maxx = (std::max)(maxx, p.x());
		miny = (std::min)(miny, p.y());
		maxy = (std::max)(maxy, p.y());
		minz = (std::min)(minz, p.z());
		maxz = (std::max)(maxz, p.z());
	}
	const float dx = maxx - minx;
	const float dy = maxy - miny;
	const float dz = maxz - minz;
	const float diag = std::sqrt(dx * dx + dy * dy + dz * dz);
	const float planeEps = (std::max)(1e-7f, diag * 1e-6f);

	for (std::uint32_t t = 0; t < nTri; ++t)
	{
		const osg::Vec3& p0 = welded[triIdx[t][0]];
		const osg::Vec3& p1 = welded[triIdx[t][1]];
		const osg::Vec3& p2 = welded[triIdx[t][2]];
		const osg::Vec3 e1 = p1 - p0;
		const osg::Vec3 e2 = p2 - p0;
		osg::Vec3 n = e1 ^ e2;
		const float len2 = n.length2();
		if (len2 > 1e-20f)
		{
			n.normalize();
			triNor[t] = osg::Vec3f(n.x(), n.y(), n.z());
		}
		else
		{
			triNor[t] = osg::Vec3f(0.0f, 0.0f, 1.0f);
		}
	}

	const osg::Vec3f& nSeed = triNor[seedTri];
	const osg::Vec3 p0Seed = welded[triIdx[seedTri][0]];

	auto coplanarWithSeed = [&](std::uint32_t u) -> bool
	{
		const osg::Vec3f& nu = triNor[u];
		if (std::fabs(static_cast<double>(nSeed.x() * nu.x() + nSeed.y() * nu.y() + nSeed.z() * nu.z())) < 0.998)
		{
			return false;
		}
		for (int k = 0; k < 3; ++k)
		{
			const osg::Vec3& v = welded[triIdx[u][static_cast<std::size_t>(k)]];
			const osg::Vec3 w = v - p0Seed;
			const float d = std::fabs(
				nSeed.x() * static_cast<float>(w.x()) + nSeed.y() * static_cast<float>(w.y()) + nSeed.z() * static_cast<float>(w.z()));
			if (d > planeEps)
			{
				return false;
			}
		}
		return true;
	};

	std::vector<char> vis(static_cast<std::size_t>(nTri), 0);
	std::queue<std::uint32_t> q;
	vis[seedTri] = 1;
	q.push(seedTri);

	while (!q.empty())
	{
		const std::uint32_t t = q.front();
		q.pop();
		for (int e = 0; e < 3; ++e)
		{
			const std::uint32_t a = triIdx[t][static_cast<std::size_t>(e)];
			const std::uint32_t b = triIdx[t][static_cast<std::size_t>((e + 1) % 3)];
			const PickEdgeKey ek{ (std::min)(a, b), (std::max)(a, b) };
			const auto it = edgeTris.find(ek);
			if (it == edgeTris.end())
			{
				continue;
			}
			for (const std::uint32_t u : it->second)
			{
				if (u >= nTri || vis[u])
				{
					continue;
				}
				if (coplanarWithSeed(u))
				{
					vis[u] = 1;
					q.push(u);
				}
			}
		}
	}

	for (std::uint32_t t = 0; t < nTri; ++t)
	{
		if (!vis[t])
		{
			continue;
		}
		for (int k = 0; k < 3; ++k)
		{
			const osg::Vec3& v = welded[triIdx[t][static_cast<std::size_t>(k)]];
			outVertsWorld.push_back(pickVertexLocalToWorld(v, m));
		}
	}
}

} // namespace

bool OsgScene::pickMeshFaceByRayIntersection(double mouseX, double mouseY,
	osg::Vec3f& outPointWorld,
	osg::Vec3f& outAWorld,
	osg::Vec3f& outBWorld,
	osg::Vec3f& outCWorld,
	osg::Vec3f& outNormalWorld,
	std::vector<osg::Vec3f>* outMergedCoplanarVertsWorld,
	const std::string* scopeBackendId) const
{
	static const unsigned int kMaskContent = 0x1u;
	if (!m_viewer.valid() || !m_viewer->getCamera() || !m_root.valid())
	{
		return false;
	}

	const double x = mouseX;
	const double y = static_cast<double>(viewportHeight()) - mouseY;
	osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
		new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, x, y);
	intersector->setIntersectionLimit(osgUtil::Intersector::LIMIT_NEAREST);
	osgUtil::IntersectionVisitor iv(intersector.get());
	iv.setTraversalMask(kMaskContent);
	m_viewer->getCamera()->accept(iv);

	if (!intersector->containsIntersections())
	{
		return false;
	}

	osgUtil::LineSegmentIntersector::Intersection firstHit;
	const osgUtil::LineSegmentIntersector::Intersection* chosenHit = nullptr;
	if (!scopeBackendId || scopeBackendId->empty())
	{
		firstHit = intersector->getFirstIntersection();
		chosenHit = &firstHit;
	}
	else
	{
		for (const auto& candidate : intersector->getIntersections())
		{
			std::string id;
			if (resolveBackendIdFromPickedPath(candidate.nodePath, id) && id == *scopeBackendId)
			{
				chosenHit = &candidate;
				break;
			}
		}
		if (!chosenHit)
		{
			return false;
		}
	}

	const auto& hit = *chosenHit;
	const osg::Vec3d wp = hit.getWorldIntersectPoint();
	outPointWorld = osg::Vec3f(static_cast<float>(wp.x()), static_cast<float>(wp.y()), static_cast<float>(wp.z()));

	const osg::Drawable* d = hit.drawable.get();
	const osg::Geometry* geom = d ? d->asGeometry() : nullptr;
	if (!geom)
	{
		return false;
	}

	const osg::RefMatrix* m = hit.matrix.valid() ? hit.matrix.get() : nullptr;

	osg::Vec3f outPointLocal = outPointWorld;
	if (m)
	{
		const osg::Matrixd invM = osg::Matrixd::inverse(*m);
		const osg::Vec3d lp = osg::Vec3d(outPointWorld.x(), outPointWorld.y(), outPointWorld.z()) * invM;
		outPointLocal = osg::Vec3f(static_cast<float>(lp.x()), static_cast<float>(lp.y()), static_cast<float>(lp.z()));
	}
	auto pointInTriangle = [&](const osg::Vec3f& A, const osg::Vec3f& B, const osg::Vec3f& C, const osg::Vec3f& P) -> bool {
		const osg::Vec3f v0 = B - A;
		const osg::Vec3f v1 = C - A;
		const osg::Vec3f v2 = P - A;

		const double d00 = static_cast<double>(v0 * v0);
		const double d01 = static_cast<double>(v0 * v1);
		const double d11 = static_cast<double>(v1 * v1);
		const double d20 = static_cast<double>(v2 * v0);
		const double d21 = static_cast<double>(v2 * v1);
		const double denom = d00 * d11 - d01 * d01;
		if (std::abs(denom) < 1e-12)
		{
			return false;
		}

		const double v = (d11 * d20 - d01 * d21) / denom;
		const double w = (d00 * d21 - d01 * d20) / denom;
		const double u = 1.0 - v - w;

		const double eps = 0.05;
		return u >= -eps && v >= -eps && w >= -eps;
	};

	struct TriIdx { unsigned int i0; unsigned int i1; unsigned int i2; };
	std::vector<TriIdx> candidates;

	// 1) If intersection provides per-vertex indices, try them directly.
	if (hit.indexList.size() >= 3U)
	{
		candidates.push_back({ hit.indexList[0], hit.indexList[1], hit.indexList[2] });
	}

	{
		const unsigned int pi = static_cast<unsigned int>(hit.primitiveIndex);
		candidates.push_back({ pi * 3u, pi * 3u + 1u, pi * 3u + 2u });
	}

	{
		const unsigned int pi = static_cast<unsigned int>(hit.primitiveIndex);
		candidates.push_back({ pi, pi + 1u, pi + 2u });
	}

	unsigned int hitTriIndex = 0;
	bool haveHitTriIndex = false;

	auto extractAndValidate = [&](auto* vertices) -> bool {
		bool haveFallback = false;
		unsigned int triIdxFallback = 0;
		osg::Vec3f fallbackAWorld, fallbackBWorld, fallbackCWorld;

		for (const TriIdx& t : candidates)
		{
			const unsigned int sz = static_cast<unsigned int>(vertices->size());
			if (t.i0 >= sz || t.i1 >= sz || t.i2 >= sz)
			{
				continue;
			}

			const osg::Vec3& aLocal = (*vertices)[t.i0];
			const osg::Vec3& bLocal = (*vertices)[t.i1];
			const osg::Vec3& cLocal = (*vertices)[t.i2];

			const auto toWorld = [&](const osg::Vec3& vLocal) -> osg::Vec3f {
				if (m)
				{
					const osg::Vec3d vw = osg::Vec3d(vLocal.x(), vLocal.y(), vLocal.z()) * (*m);
					return osg::Vec3f(static_cast<float>(vw.x()), static_cast<float>(vw.y()), static_cast<float>(vw.z()));
				}
				return osg::Vec3f(vLocal.x(), vLocal.y(), vLocal.z());
			};

			if (!haveFallback)
			{
				triIdxFallback = t.i0 / 3u;
				fallbackAWorld = toWorld(aLocal);
				fallbackBWorld = toWorld(bLocal);
				fallbackCWorld = toWorld(cLocal);
				haveFallback = true;
			}

			if (!m)
			{
				outAWorld = fallbackAWorld;
				outBWorld = fallbackBWorld;
				outCWorld = fallbackCWorld;
				hitTriIndex = triIdxFallback;
				haveHitTriIndex = true;
				return true;
			}

			const osg::Vec3f A(aLocal.x(), aLocal.y(), aLocal.z());
			const osg::Vec3f B(bLocal.x(), bLocal.y(), bLocal.z());
			const osg::Vec3f C(cLocal.x(), cLocal.y(), cLocal.z());
			if (pointInTriangle(A, B, C, outPointLocal))
			{
				outAWorld = toWorld(aLocal);
				outBWorld = toWorld(bLocal);
				outCWorld = toWorld(cLocal);
				hitTriIndex = t.i0 / 3u;
				haveHitTriIndex = true;
				return true;
			}
		}

		if (haveFallback)
		{
			outAWorld = fallbackAWorld;
			outBWorld = fallbackBWorld;
			outCWorld = fallbackCWorld;
			hitTriIndex = triIdxFallback;
			haveHitTriIndex = true;
			return true;
		}

		return false;
	};

	if (const auto* va = dynamic_cast<const osg::Vec3Array*>(geom->getVertexArray()))
	{
		if (!extractAndValidate(const_cast<osg::Vec3Array*>(va)))
		{
			return false;
		}
	}
	else if (const auto* vda = dynamic_cast<const osg::Vec3dArray*>(geom->getVertexArray()))
	{
		const auto toLocal = [&](const osg::Vec3d& vLocal) -> osg::Vec3f {
			return osg::Vec3f(static_cast<float>(vLocal.x()), static_cast<float>(vLocal.y()), static_cast<float>(vLocal.z()));
		};
		const auto toWorld = [&](const osg::Vec3d& vLocal) -> osg::Vec3f {
			if (m)
			{
				const osg::Vec3d vw = vLocal * (*m);
				return osg::Vec3f(static_cast<float>(vw.x()), static_cast<float>(vw.y()), static_cast<float>(vw.z()));
			}
			return osg::Vec3f(static_cast<float>(vLocal.x()), static_cast<float>(vLocal.y()), static_cast<float>(vLocal.z()));
		};

		bool extracted = false;
		bool haveFallback = false;
		unsigned int triIdxFallbackVd = 0;
		osg::Vec3f fallbackAWorld, fallbackBWorld, fallbackCWorld;

		for (const TriIdx& t : candidates)
		{
			const unsigned int sz = static_cast<unsigned int>(vda->size());
			if (t.i0 >= sz || t.i1 >= sz || t.i2 >= sz)
			{
				continue;
			}

			const osg::Vec3f A = toLocal((*vda)[t.i0]);
			const osg::Vec3f B = toLocal((*vda)[t.i1]);
			const osg::Vec3f C = toLocal((*vda)[t.i2]);

			if (!haveFallback)
			{
				triIdxFallbackVd = t.i0 / 3u;
				fallbackAWorld = toWorld((*vda)[t.i0]);
				fallbackBWorld = toWorld((*vda)[t.i1]);
				fallbackCWorld = toWorld((*vda)[t.i2]);
				haveFallback = true;
			}

			if (!m)
			{
				outAWorld = fallbackAWorld;
				outBWorld = fallbackBWorld;
				outCWorld = fallbackCWorld;
				hitTriIndex = triIdxFallbackVd;
				haveHitTriIndex = true;
				extracted = true;
				break;
			}

			if (pointInTriangle(A, B, C, outPointLocal))
			{
				outAWorld = toWorld((*vda)[t.i0]);
				outBWorld = toWorld((*vda)[t.i1]);
				outCWorld = toWorld((*vda)[t.i2]);
				hitTriIndex = t.i0 / 3u;
				haveHitTriIndex = true;
				extracted = true;
				break;
			}
		}
		if (!extracted)
		{
			if (haveFallback)
			{
				outAWorld = fallbackAWorld;
				outBWorld = fallbackBWorld;
				outCWorld = fallbackCWorld;
				hitTriIndex = triIdxFallbackVd;
				haveHitTriIndex = true;
				extracted = true;
			}
			if (!extracted)
			{
				return false;
			}
		}
	}
	else
	{
		return false;
	}

	if (outMergedCoplanarVertsWorld)
	{
		outMergedCoplanarVertsWorld->clear();
		std::vector<osg::Vec3> soup;
		if (const auto* va = dynamic_cast<const osg::Vec3Array*>(geom->getVertexArray()))
		{
			soup.reserve(va->size());
			for (unsigned int i = 0; i < static_cast<unsigned int>(va->size()); ++i)
			{
				soup.push_back((*va)[i]);
			}
		}
		else if (const auto* vda = dynamic_cast<const osg::Vec3dArray*>(geom->getVertexArray()))
		{
			soup.reserve(vda->size());
			for (unsigned int i = 0; i < static_cast<unsigned int>(vda->size()); ++i)
			{
				const osg::Vec3d& v = (*vda)[i];
				soup.push_back(osg::Vec3(
					static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())));
			}
		}
		const std::uint32_t nTriSoup = static_cast<std::uint32_t>(soup.size() / 3U);
		if (soup.size() >= 9U && (soup.size() % 3U) == 0U && haveHitTriIndex && hitTriIndex < nTriSoup)
		{
			expandCoplanarTrianglesFromSoupLocal(soup, m, static_cast<std::uint32_t>(hitTriIndex), *outMergedCoplanarVertsWorld);
		}
		if (outMergedCoplanarVertsWorld->empty())
		{
			outMergedCoplanarVertsWorld->push_back(outAWorld);
			outMergedCoplanarVertsWorld->push_back(outBWorld);
			outMergedCoplanarVertsWorld->push_back(outCWorld);
		}
	}

	const osg::Vec3f ab = outBWorld - outAWorld;
	const osg::Vec3f ac = outCWorld - outAWorld;
	osg::Vec3f n = ab ^ ac;
	if (n.length2() > 1e-12f)
	{
		n.normalize();
	}
	else
	{
		n.set(0.0f, 0.0f, 1.0f);
	}
	outNormalWorld = n;
	return true;
}

bool OsgScene::pickMeshEdgeByRayIntersection(double mouseX, double mouseY, osg::Vec3f& outPointWorld, osg::Vec3f& outEdgeAWorld, osg::Vec3f& outEdgeBWorld, double* outEdgeDistancePx, const std::string* scopeBackendId) const
{
	osg::Vec3f p, a, b, c, n;
	if (!pickMeshFaceByRayIntersection(mouseX, mouseY, p, a, b, c, n, nullptr, scopeBackendId))
	{
		return false;
	}

	if (!m_viewer.valid() || !m_viewer->getCamera())
	{
		return false;
	}
	const osg::Matrixd mvp = m_viewer->getCamera()->getViewMatrix() * m_viewer->getCamera()->getProjectionMatrix();
	const auto toScreen = [&](const osg::Vec3f& world, double& sx, double& sy) {
		const osg::Vec3d clip = osg::Vec3d(world) * mvp;
		sx = (clip.x() * 0.5 + 0.5) * static_cast<double>(viewportWidth());
		sy = (1.0 - (clip.y() * 0.5 + 0.5)) * static_cast<double>(viewportHeight());
	};
	const double qx = mouseX;
	const double qy = mouseY;

	struct EdgeCand
	{
		osg::Vec3f a;
		osg::Vec3f b;
		double distPx = (std::numeric_limits<double>::max)();
	};

	auto edgeDistancePx = [&](const osg::Vec3f& ea, const osg::Vec3f& eb) -> double {
		double s0x = 0, s0y = 0, s1x = 0, s1y = 0;
		toScreen(ea, s0x, s0y);
		toScreen(eb, s1x, s1y);
		const double segVx = s1x - s0x;
		const double segVy = s1y - s0y;
		const double offVx = qx - s0x;
		const double offVy = qy - s0y;
		const double len2Seg = segVx * segVx + segVy * segVy;
		if (len2Seg <= 1e-9)
		{
			return std::hypot(qx - s0x, qy - s0y);
		}
		const double dot = offVx * segVx + offVy * segVy;
		const double t = (std::max)(0.0, (std::min)(1.0, dot / len2Seg));
		const double projx = s0x + segVx * t;
		const double projy = s0y + segVy * t;
		return std::hypot(qx - projx, qy - projy);
	};

	EdgeCand best;
	const EdgeCand cands[3] = {
		{ a, b, edgeDistancePx(a, b) },
		{ b, c, edgeDistancePx(b, c) },
		{ c, a, edgeDistancePx(c, a) }
	};
	for (const EdgeCand& e : cands)
	{
		if (e.distPx < best.distPx)
		{
			best = e;
		}
	}

	if (best.distPx > kMeshEdgeHitRadiusPx)
	{
		return false;
	}
	outPointWorld = p;
	outEdgeAWorld = best.a;
	outEdgeBWorld = best.b;
	if (outEdgeDistancePx)
	{
		*outEdgeDistancePx = best.distPx;
	}
	return true;
}
void OsgScene::showMeshFaceHighlight(const std::vector<osg::Vec3f>& vertsWorld)
{
	if (vertsWorld.size() < 3U || (vertsWorld.size() % 3U) != 0U)
	{
		return;
	}
	if (!m_meshPickOverlayGroup.valid() || !m_meshPickedFaceVertices.valid() || !m_meshPickedEdgeVertices.valid())
	{
		return;
	}

	constexpr unsigned int kMaskHelper = 0x2u;
	m_meshPickOverlayGroup->setNodeMask(kMaskHelper);

	m_meshPickedFaceVertices->clear();
	m_meshPickedFaceVertices->reserve(vertsWorld.size());
	for (const osg::Vec3f& v : vertsWorld)
	{
		m_meshPickedFaceVertices->push_back(osg::Vec3(v.x(), v.y(), v.z()));
	}

	if (m_meshPickedFaceGeom->getNumPrimitiveSets() > 0)
	{
		osg::DrawArrays* da = dynamic_cast<osg::DrawArrays*>(m_meshPickedFaceGeom->getPrimitiveSet(0));
		if (da)
		{
			da->setFirst(0);
			da->setCount(static_cast<GLsizei>(vertsWorld.size()));
		}
		else
		{
			m_meshPickedFaceGeom->removePrimitiveSet(0u, 1u);
			m_meshPickedFaceGeom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertsWorld.size())));
		}
	}
	else
	{
		m_meshPickedFaceGeom->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertsWorld.size())));
	}

	m_meshPickedEdgeVertices->at(0) = osg::Vec3f(0.0f, 0.0f, 0.0f);
	m_meshPickedEdgeVertices->at(1) = osg::Vec3f(0.0f, 0.0f, 0.0f);

	if (m_meshPickedFaceGeom.valid())
	{
		m_meshPickedFaceGeom->dirtyDisplayList();
		m_meshPickedFaceGeom->dirtyBound();
	}
	if (m_meshPickedEdgeGeom.valid())
	{
		m_meshPickedEdgeGeom->dirtyDisplayList();
		m_meshPickedEdgeGeom->dirtyBound();
	}
}

void OsgScene::showMeshFaceHighlight(const osg::Vec3f& aWorld, const osg::Vec3f& bWorld, const osg::Vec3f& cWorld)
{
	std::vector<osg::Vec3f> tri;
	tri.reserve(3);
	tri.push_back(aWorld);
	tri.push_back(bWorld);
	tri.push_back(cWorld);
	showMeshFaceHighlight(tri);
}

void OsgScene::showMeshEdgeHighlight(const osg::Vec3f& aWorld, const osg::Vec3f& bWorld)
{
	if (!m_meshPickOverlayGroup.valid() || !m_meshPickedFaceVertices.valid() || !m_meshPickedEdgeVertices.valid())
	{
		return;
	}

	constexpr unsigned int kMaskHelper = 0x2u;
	m_meshPickOverlayGroup->setNodeMask(kMaskHelper);

	// Overlay vertices are set in world space, because m_meshPickOverlayGroup is attached to m_root.
	m_meshPickedEdgeVertices->at(0) = aWorld;
	m_meshPickedEdgeVertices->at(1) = bWorld;

	m_meshPickedFaceVertices->at(0) = osg::Vec3f(0.0f, 0.0f, 0.0f);
	m_meshPickedFaceVertices->at(1) = osg::Vec3f(0.0f, 0.0f, 0.0f);
	m_meshPickedFaceVertices->at(2) = osg::Vec3f(0.0f, 0.0f, 0.0f);

	if (m_meshPickedFaceGeom.valid())
	{
		m_meshPickedFaceGeom->dirtyDisplayList();
		m_meshPickedFaceGeom->dirtyBound();
	}
	if (m_meshPickedEdgeGeom.valid())
	{
		m_meshPickedEdgeGeom->dirtyDisplayList();
		m_meshPickedEdgeGeom->dirtyBound();
	}
}

void OsgScene::hideMeshElementHighlight()
{
	if (!m_meshPickOverlayGroup.valid())
	{
		return;
	}
	m_meshPickOverlayGroup->setNodeMask(0u);
}

namespace
{

osg::ref_ptr<osg::Geode> makeFeatureOverlayLine(const osg::Vec3f& a, const osg::Vec3f& b, const osg::Vec4& color,
	float width)
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
	verts->push_back(a);
	verts->push_back(b);
	geom->setVertexArray(verts.get());
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back(color);
	geom->setColorArray(colors.get(), osg::Array::BIND_OVERALL);
	geom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));
	geode->addDrawable(geom.get());
	geode->getOrCreateStateSet()->setAttribute(new osg::LineWidth(width));
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	return geode;
}

void spreadFeatureCatalogLabels(std::vector<OsgScene::FeatureCatalogOverlayItem>& items)
{
	if (items.empty())
	{
		return;
	}

	osg::Vec3f minB(FLT_MAX, FLT_MAX, FLT_MAX);
	osg::Vec3f maxB(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	osg::Vec3f centroid(0.0f, 0.0f, 0.0f);
	for (const OsgScene::FeatureCatalogOverlayItem& item : items)
	{
		centroid += item.anchorWorldMm;
		minB.x() = std::min(minB.x(), item.anchorWorldMm.x());
		minB.y() = std::min(minB.y(), item.anchorWorldMm.y());
		minB.z() = std::min(minB.z(), item.anchorWorldMm.z());
		maxB.x() = std::max(maxB.x(), item.anchorWorldMm.x());
		maxB.y() = std::max(maxB.y(), item.anchorWorldMm.y());
		maxB.z() = std::max(maxB.z(), item.anchorWorldMm.z());
	}
	centroid /= static_cast<float>(items.size());
	const osg::Vec3f extent = maxB - minB;
	const float diag = extent.length();
	const float leaderLen = std::max(48.0f, diag * 0.2f);
	const float minSep = std::max(40.0f, diag * 0.16f);
	const float maxLeaderLen = leaderLen * 1.75f;

	auto clampLeader = [&](OsgScene::FeatureCatalogOverlayItem& it) {
		osg::Vec3f lead = it.labelWorldMm - it.anchorWorldMm;
		const float len = lead.length();
		if (len > maxLeaderLen && len > 1e-4f)
		{
			it.labelWorldMm = it.anchorWorldMm + lead * (maxLeaderLen / len);
		}
	};

	if (items.size() == 1)
	{
		osg::Vec3f dir = items[0].labelWorldMm - items[0].anchorWorldMm;
		if (dir.length2() < 1e-6f)
		{
			dir = items[0].anchorWorldMm - centroid;
		}
		if (dir.length2() < 1e-6f)
		{
			dir.set(1.0f, 0.0f, 0.0f);
		}
		dir.normalize();
		items[0].labelWorldMm = items[0].anchorWorldMm + dir * leaderLen;
		return;
	}

	int axisA = 0;
	int axisB = 1;
	if (extent.y() >= extent.x() && extent.y() >= extent.z())
	{
		axisA = 0;
		axisB = 2;
	}
	else if (extent.z() >= extent.x() && extent.z() >= extent.y())
	{
		axisA = 0;
		axisB = 1;
	}

	struct LayoutSlot
	{
		std::size_t itemIndex = 0;
		osg::Vec3f anchor;
		float angle = 0.0f;
	};

	std::vector<LayoutSlot> slots;
	slots.reserve(items.size());
	for (std::size_t i = 0; i < items.size(); ++i)
	{
		osg::Vec3f outward = items[i].anchorWorldMm - centroid;
		if (outward.length2() < 1e-8f)
		{
			outward.set(1.0f, 0.0f, 0.0f);
		}
		outward.normalize();
		LayoutSlot slot;
		slot.itemIndex = i;
		slot.anchor = items[i].anchorWorldMm;
		const float compA = axisA == 0 ? outward.x() : (axisA == 1 ? outward.y() : outward.z());
		const float compB = axisB == 0 ? outward.x() : (axisB == 1 ? outward.y() : outward.z());
		slot.angle = std::atan2(compB, compA);
		slots.push_back(slot);
	}

	std::sort(slots.begin(), slots.end(),
		[](const LayoutSlot& lhs, const LayoutSlot& rhs) { return lhs.angle < rhs.angle; });

	const float twoPi = 6.28318530718f;
	const float slotCount = static_cast<float>(slots.size());
	for (std::size_t k = 0; k < slots.size(); ++k)
	{
		const float targetAngle = twoPi * static_cast<float>(k) / slotCount;
		osg::Vec3f dir(0.0f, 0.0f, 0.0f);
		if (axisA == 0)
		{
			dir.x() = std::cos(targetAngle);
		}
		else if (axisA == 1)
		{
			dir.y() = std::cos(targetAngle);
		}
		else
		{
			dir.z() = std::cos(targetAngle);
		}
		if (axisB == 0)
		{
			dir.x() = std::sin(targetAngle);
		}
		else if (axisB == 1)
		{
			dir.y() = std::sin(targetAngle);
		}
		else
		{
			dir.z() = std::sin(targetAngle);
		}

		osg::Vec3f localOut = slots[k].anchor - centroid;
		if (localOut.length2() > 1e-8f)
		{
			localOut.normalize();
			dir = dir * 0.5f + localOut * 0.5f;
			dir.normalize();
		}
		else
		{
			dir.normalize();
		}

		const float ring = (k % 3 == 2) ? 1.4f : ((k % 3 == 1) ? 1.2f : 1.0f);
		items[slots[k].itemIndex].labelWorldMm = slots[k].anchor + dir * leaderLen * ring;
	}

	for (int pass = 0; pass < 24; ++pass)
	{
		for (std::size_t i = 0; i < items.size(); ++i)
		{
			for (std::size_t j = i + 1; j < items.size(); ++j)
			{
				osg::Vec3f delta = items[j].labelWorldMm - items[i].labelWorldMm;
				const float dist = delta.length();
				if (dist >= minSep || dist < 1e-4f)
				{
					continue;
				}
				delta /= dist;
				const float push = (minSep - dist) * 0.7f;
				items[i].labelWorldMm -= delta * push;
				items[j].labelWorldMm += delta * push;
				clampLeader(items[i]);
				clampLeader(items[j]);
			}
		}
	}
}

} // namespace

void OsgScene::clearFeatureCatalogOverlay()
{
	if (m_featureCatalogOverlayGroup.valid())
	{
		m_featureCatalogOverlayGroup->removeChildren(0, m_featureCatalogOverlayGroup->getNumChildren());
	}
}

void OsgScene::setFeatureCatalogOverlay(const std::vector<FeatureCatalogOverlayItem>& items)
{
	clearFeatureCatalogOverlay();
	if (!m_featureCatalogOverlayGroup.valid() || items.empty())
	{
		return;
	}

	std::vector<FeatureCatalogOverlayItem> layoutItems = items;
	spreadFeatureCatalogLabels(layoutItems);

	for (const FeatureCatalogOverlayItem& item : layoutItems)
	{
		osg::ref_ptr<osg::Group> itemGroup = new osg::Group;

		if (item.hasEdgeSegment)
		{
			itemGroup->addChild(makeFeatureOverlayLine(item.edgeAWorldMm, item.edgeBWorldMm,
				osg::Vec4(0.25f, 0.02f, 0.02f, 0.65f), 8.0f).get());
			itemGroup->addChild(makeFeatureOverlayLine(item.edgeAWorldMm, item.edgeBWorldMm,
				osg::Vec4(1.0f, 0.18f, 0.12f, 1.0f), 4.5f).get());
		}

		osg::ref_ptr<osg::Geode> anchorGeode = new osg::Geode;
		osg::ref_ptr<osg::ShapeDrawable> sphere = new osg::ShapeDrawable(new osg::Sphere(item.anchorWorldMm, 2.5f));
		sphere->setColor(osg::Vec4(1.0f, 0.15f, 0.1f, 1.0f));
		anchorGeode->addDrawable(sphere.get());
		osg::StateSet* anchorSs = anchorGeode->getOrCreateStateSet();
		anchorSs->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
		anchorSs->setAttribute(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
		anchorSs->setMode(GL_BLEND, osg::StateAttribute::ON);
		itemGroup->addChild(anchorGeode.get());

		itemGroup->addChild(
			makeFeatureOverlayLine(item.anchorWorldMm, item.labelWorldMm, osg::Vec4(1.0f, 0.55f, 0.5f, 0.95f), 2.0f)
				.get());

		osg::ref_ptr<osg::AutoTransform> labelXform = new osg::AutoTransform;
		labelXform->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
		labelXform->setAutoScaleToScreen(true);
		labelXform->setAutoScaleTransitionWidthRatio(0.25f);
		labelXform->setMinimumScale(0.35f);
		labelXform->setMaximumScale(2.0f);
		labelXform->setPosition(item.labelWorldMm);
		osg::ref_ptr<osgText::Text> text = new osgText::Text;
		text->setText(std::to_string(item.displayIndex));
		text->setCharacterSize(18.0f);
		text->setFontResolution(96, 96);
		text->setAxisAlignment(osgText::TextBase::SCREEN);
		text->setAlignment(osgText::Text::CENTER_CENTER);
		text->setColor(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
		text->setBackdropType(osgText::Text::NONE);
		osg::ref_ptr<osg::Geode> textGeode = new osg::Geode;
		textGeode->addDrawable(text.get());
		osg::StateSet* textSs = textGeode->getOrCreateStateSet();
		textSs->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
		textSs->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
		textSs->setRenderBinDetails(12, "RenderBin");
		labelXform->addChild(textGeode.get());
		itemGroup->addChild(labelXform.get());

		m_featureCatalogOverlayGroup->addChild(itemGroup.get());
	}
	requestRedraw();
}

void OsgScene::rebuildPointKdTree()
{
	m_kdNodes.clear();
	m_kdRoot = -1;
	if (m_pickablePointsCenteredLocal.empty())
	{
		return;
	}
	// Avoid excessive memory usage on huge clouds; fallback path remains available.
	static const std::size_t kMaxKdPoints = 1500000;
	if (m_pickablePointsCenteredLocal.size() > kMaxKdPoints)
	{
		return;
	}

	std::vector<int> indices(m_pickablePointsCenteredLocal.size());
	for (std::size_t i = 0; i < indices.size(); ++i)
	{
		indices[i] = static_cast<int>(i);
	}
	m_kdNodes.reserve(m_pickablePointsCenteredLocal.size());
	m_kdRoot = buildKdNode(indices, 0, static_cast<int>(indices.size()), 0);
}

int OsgScene::buildKdNode(std::vector<int>& indices, int begin, int end, int depth)
{
	if (begin >= end)
	{
		return -1;
	}
	const int axis = depth % 3;
	const int mid = begin + (end - begin) / 2;
	std::nth_element(indices.begin() + begin, indices.begin() + mid, indices.begin() + end,
		[this, axis](int a, int b) {
			const osg::Vec3f& pa = m_pickablePointsCenteredLocal[static_cast<std::size_t>(a)];
			const osg::Vec3f& pb = m_pickablePointsCenteredLocal[static_cast<std::size_t>(b)];
			if (axis == 0) return pa.x() < pb.x();
			if (axis == 1) return pa.y() < pb.y();
			return pa.z() < pb.z();
		});

	const int nodeIndex = static_cast<int>(m_kdNodes.size());
	m_kdNodes.emplace_back();
	m_kdNodes[static_cast<std::size_t>(nodeIndex)].pointIndex = indices[static_cast<std::size_t>(mid)];
	m_kdNodes[static_cast<std::size_t>(nodeIndex)].axis = axis;
	m_kdNodes[static_cast<std::size_t>(nodeIndex)].left = buildKdNode(indices, begin, mid, depth + 1);
	m_kdNodes[static_cast<std::size_t>(nodeIndex)].right = buildKdNode(indices, mid + 1, end, depth + 1);
	return nodeIndex;
}

int OsgScene::nearestPointByKdTree(const osg::Vec3f& queryLocalCentered) const
{
	if (m_kdRoot < 0 || m_kdNodes.empty() || m_pickablePointsCenteredLocal.empty())
	{
		return -1;
	}

	int bestIdx = -1;
	float bestDist2 = (std::numeric_limits<float>::max)();

	std::function<void(int)> dfs = [&](int nodeIndex) {
		if (nodeIndex < 0) return;
		const KdNode& node = m_kdNodes[static_cast<std::size_t>(nodeIndex)];
		const osg::Vec3f& p = m_pickablePointsCenteredLocal[static_cast<std::size_t>(node.pointIndex)];
		const osg::Vec3f d = p - queryLocalCentered;
		const float dist2 = d.length2();
		if (dist2 < bestDist2)
		{
			bestDist2 = dist2;
			bestIdx = node.pointIndex;
		}

		float diff = 0.0f;
		if (node.axis == 0) diff = queryLocalCentered.x() - p.x();
		else if (node.axis == 1) diff = queryLocalCentered.y() - p.y();
		else diff = queryLocalCentered.z() - p.z();

		const int nearChild = diff <= 0.0f ? node.left : node.right;
		const int farChild = diff <= 0.0f ? node.right : node.left;
		dfs(nearChild);
		if (diff * diff < bestDist2)
		{
			dfs(farChild);
		}
	};
	dfs(m_kdRoot);
	return bestIdx;
}

void OsgScene::nearestCandidatesByKdTree(const osg::Vec3f& queryLocalCentered, int k, std::vector<int>& outIndices) const
{
	outIndices.clear();
	if (m_kdRoot < 0 || m_kdNodes.empty() || m_pickablePointsCenteredLocal.empty() || k <= 0)
	{
		return;
	}

	using DistIndex = std::pair<float, int>;
	std::priority_queue<DistIndex> best; // max-heap by distance

	std::function<void(int)> dfs = [&](int nodeIndex) {
		if (nodeIndex < 0) return;
		const KdNode& node = m_kdNodes[static_cast<std::size_t>(nodeIndex)];
		const osg::Vec3f& p = m_pickablePointsCenteredLocal[static_cast<std::size_t>(node.pointIndex)];
		const osg::Vec3f d = p - queryLocalCentered;
		const float dist2 = d.length2();

		if (static_cast<int>(best.size()) < k)
		{
			best.push({ dist2, node.pointIndex });
		}
		else if (dist2 < best.top().first)
		{
			best.pop();
			best.push({ dist2, node.pointIndex });
		}

		float diff = 0.0f;
		if (node.axis == 0) diff = queryLocalCentered.x() - p.x();
		else if (node.axis == 1) diff = queryLocalCentered.y() - p.y();
		else diff = queryLocalCentered.z() - p.z();

		const int nearChild = diff <= 0.0f ? node.left : node.right;
		const int farChild = diff <= 0.0f ? node.right : node.left;
		dfs(nearChild);
		const float worst = (static_cast<int>(best.size()) < k) ? (std::numeric_limits<float>::max)() : best.top().first;
		if (diff * diff < worst)
		{
			dfs(farChild);
		}
	};
	dfs(m_kdRoot);

	outIndices.reserve(best.size());
	while (!best.empty())
	{
		outIndices.push_back(best.top().second);
		best.pop();
	}
}
