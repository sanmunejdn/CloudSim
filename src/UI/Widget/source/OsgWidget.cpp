#include "OsgWidget.h"

#include "BackendDataBase.h"
#include "BackendVisualRegistry.h"

#include <BrepImportArtifacts.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>

#include <QBuffer>
#include <QImage>
#include <QFileInfo>
#include <QMouseEvent>
#include <QPalette>
#include <QPainter>
#include <QShowEvent>
#include <QResizeEvent>
#include <QRegExp>
#include <QStringList>
#include <QTextStream>
#include <QVBoxLayout>

#include <osg/Array>
#include <osg/Camera>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/PrimitiveSet>
#include <osg/Group>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/Matrix>
#include <osg/MatrixTransform>
#include <osg/Node>
#include <osg/Point>
#include <osg/PolygonOffset>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osg/Shape>
#include <osg/Transform>
#include <osg/StateSet>
#include <osg/StateAttribute>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Vec4>
#include <osg/NodeVisitor>
#include <osg/NodeCallback>
#include <osg/AutoTransform>
#include <osg/BoundingSphere>
#include <osgDB/Options>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osgGA/EventQueue>
#include <osgGA/GUIEventAdapter>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgText/Text>
#include <osgViewer/GraphicsWindow>

#include "MeshBackendData.h"
#include "PointCloudBackendData.h"

#include <osg/Image>
#include <osg/Image>
#include <osgViewer/Viewer>

#include "OsgWidgetBackendLoadController.h"
#include "OsgWidgetImportController.h"
#include "OsgWidgetCaptureController.h"
#include "OsgWidgetPickAnnotationController.h"
#include "OsgWidgetColorController.h"
#include "OsgWidgetTransformHierarchyController.h"

#include "GraphicsWindowQt1.h"
#include "ObjectTransformOperation.h"
#include "RobotTcpDragTeachOperation.h"
#include "PointPickOperation.h"
#include "PolylinePickOperation.h"
#include "MeshEdgeFacePickOperation.h"
#include "PickTypes.h"
#include "QWidgetViewer.h"

#include "BackendFollowMath.h"
#include "BackendVisualMath.h"
#include "ObjectGizmoFrame.h"

namespace {

QColor viewerChromeColor(bool dark)
{
	return dark ? QColor(102, 102, 107) : QColor(250, 250, 250);
}

osg::ref_ptr<osg::Image> makeViewCubeLabelImage(const QString& text)
{
	const QFont font(QStringLiteral("Microsoft YaHei"), 14, QFont::Bold);
	const QFontMetrics metrics(font);
	const QRect bounds = metrics.boundingRect(text);
	const int w = bounds.width() + 10;
	const int h = bounds.height() + 8;
	QImage qimg(w, h, QImage::Format_RGBA8888);
	qimg.fill(Qt::transparent);
	QPainter painter(&qimg);
	painter.setRenderHint(QPainter::Antialiasing, true);
	painter.setRenderHint(QPainter::TextAntialiasing, true);
	painter.setFont(font);
	painter.setPen(QColor(13, 15, 18));
	painter.drawText(QRect(0, 0, w, h), Qt::AlignCenter, text);
	painter.end();

	osg::ref_ptr<osg::Image> image = new osg::Image;
	image->allocateImage(w, h, 1, GL_RGBA, GL_UNSIGNED_BYTE);
	unsigned char* dst = image->data();
	for (int y = 0; y < h; ++y)
	{
		const unsigned char* srcRow = qimg.constScanLine(y);
		for (int x = 0; x < w; ++x)
		{
			const unsigned char* srcPx = srcRow + x * 4;
			dst[0] = srcPx[0];
			dst[1] = srcPx[1];
			dst[2] = srcPx[2];
			dst[3] = srcPx[3];
			dst += 4;
		}
	}
	image->setInternalTextureFormat(GL_RGBA8);
	return image;
}

} // namespace

OsgWidget::OsgWidget(QWidget* parent)
	: QWidget(parent)
{
	qRegisterMetaType<PickResult>("PickResult");
	m_feedbackTimer.start();
	m_pointPickOperation = std::make_unique<PointPickOperation>(this);
	m_polylinePickOperation = std::make_unique<PolylinePickOperation>(this);
	m_objectTransformOperation = std::make_unique<ObjectTransformOperation>(this);
	m_tcpDragTeachOperation = std::make_unique<RobotTcpDragTeachOperation>(this);
	m_meshElementPickOperation = std::make_unique<MeshEdgeFacePickOperation>(this);
	m_importController = std::make_unique<OsgWidgetImportController>();
	m_backendLoadController = std::make_unique<OsgWidgetBackendLoadController>();
	m_captureController = std::make_unique<OsgWidgetCaptureController>();
	m_pickAnnotationController = std::make_unique<OsgWidgetPickAnnotationController>();
	initScene();
	initUi();
	setRequestRedraw([this]() {
		if (m_tcpTeachActive)
		{
			syncTcpTeachWorldPatFromMount();
		}
		emit sceneRedrawRequested();
		if (m_glWidget)
		{
			m_glWidget->update();
		}
	});
	if (m_glWidget)
	{
		setViewportPixels(m_glWidget->width(), m_glWidget->height());
		setDevicePixelRatio(m_glWidget->devicePixelRatio());
	}
	initViewer();
}

OsgWidget::~OsgWidget() = default;

bool OsgWidget::hasImportedContent() const
{
	if (m_stagingGroup.valid() && m_stagingGroup->getNumChildren() > 0)
	{
		return true;
	}
	return !m_backendObjectRoots.empty();
}

void OsgWidget::applyRigidRotationAboutWorldPivot(const std::vector<std::string>& backendIds, const osg::Vec3f& pivotWorld,
	const osg::Quat& deltaRotation)
{
	for (const std::string& id : backendIds)
	{
		const auto it = m_backendObjectRoots.find(id);
		if (it == m_backendObjectRoots.end() || !it->second.valid())
		{
			continue;
		}
		osg::MatrixTransform* mt = it->second.get();
		osg::Vec3d t;
		osg::Quat q;
		osg::Vec3d s;
		osg::Quat so;
		mt->getMatrix().decompose(t, q, s, so);
		const osg::Vec3f p(static_cast<float>(t.x()), static_cast<float>(t.y()), static_cast<float>(t.z()));
		const osg::Vec3f rel = p - pivotWorld;
		const osg::Vec3f pNew = deltaRotation * rel + pivotWorld;
		const osg::Quat qNew = deltaRotation * q;
		mt->setMatrix(osg::Matrixd::translate(osg::Vec3d(pNew.x(), pNew.y(), pNew.z())) * osg::Matrixd::rotate(qNew));
	}
	if (m_viewer.valid())
	{
		m_viewer->setSceneData(m_root.get());
	}
	requestRedraw();
}

osg::Vec3f OsgWidget::averageBackendRootPositionWorld(const std::vector<std::string>& backendIds) const
{
	osg::Vec3f sum(0.0f, 0.0f, 0.0f);
	int n = 0;
	for (const std::string& id : backendIds)
	{
		const auto it = m_backendObjectRoots.find(id);
		if (it == m_backendObjectRoots.end() || !it->second.valid())
		{
			continue;
		}
		osg::NodePath path;
		for (osg::Node* n = it->second.get(); n != nullptr; n = n->getNumParents() > 0 ? n->getParent(0) : nullptr)
		{
			path.insert(path.begin(), n);
		}
		const osg::Matrixd w = osg::computeLocalToWorld(path);
		const osg::Vec3d tr = w.getTrans();
		sum += osg::Vec3f(static_cast<float>(tr.x()), static_cast<float>(tr.y()), static_cast<float>(tr.z()));
		++n;
	}
	if (n <= 0)
	{
		return osg::Vec3f(0.0f, 0.0f, 0.0f);
	}
	return sum * (1.0f / static_cast<float>(n));
}

namespace
{

osg::NodePath nodePathToSceneRoot(const osg::Node* leaf)
{
	osg::NodePath path;
	for (const osg::Node* n = leaf; n != nullptr; n = n->getNumParents() > 0 ? n->getParent(0) : nullptr)
	{
		path.insert(path.begin(), const_cast<osg::Node*>(n));
	}
	return path;
}

double matMaxAbsDiff(const osg::Matrixd& a, const osg::Matrixd& b)
{
	double m = 0.0;
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			const double d = std::abs(static_cast<double>(a(r, c)) - static_cast<double>(b(r, c)));
			if (d > m)
			{
				m = d;
			}
		}
	}
	return m;
}

osg::Group* findUrdfLinkContainer(osg::Group* sceneSubtree, const std::string& urdfLinkName)
{
	if (!sceneSubtree || urdfLinkName.empty())
	{
		return nullptr;
	}
	const std::string want = urdfLinkName + "_Container";
	struct FindLinkContainer final : osg::NodeVisitor
	{
		std::string wantName;
		osg::Group* found = nullptr;
		explicit FindLinkContainer(std::string w)
			: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
			, wantName(std::move(w))
		{
		}
		void apply(osg::Group& g) override
		{
			if (found)
			{
				return;
			}
			if (g.getName() == wantName)
			{
				found = &g;
				return;
			}
			traverse(g);
		}
	} vis(want);
	sceneSubtree->accept(vis);
	return vis.found;
}

osg::ref_ptr<osg::Geode> createReachabilityOriginGeode(bool reachable)
{
	const float r = reachable ? 6.0f : 7.0f;
	const osg::Vec4 color = reachable ? osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) : osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f);
	osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), r);
	osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(sphere.get());
	drawable->setColor(color);
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(drawable.get());
	geode->setNodeMask(OsgScene::kMaskPickOverlay);
	osg::StateSet* ss = geode->getOrCreateStateSet();
	ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	return geode;
}

osg::ref_ptr<osg::Geode> createInstructionPoseAxisGeode(
	float axisLengthMm,
	bool lineMotion,
	bool alwaysVisible = false)
{
	osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	const osg::Vec4 xColor = lineMotion ? osg::Vec4(1.0f, 0.4f, 0.4f, 1.0f) : osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f);
	const osg::Vec4 yColor = lineMotion ? osg::Vec4(0.4f, 1.0f, 0.4f, 1.0f) : osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f);
	const osg::Vec4 zColor = lineMotion ? osg::Vec4(0.4f, 0.6f, 1.0f, 1.0f) : osg::Vec4(0.1f, 0.3f, 1.0f, 1.0f);

	verts->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	verts->push_back(osg::Vec3(axisLengthMm, 0.0f, 0.0f));
	colors->push_back(xColor);
	colors->push_back(xColor);

	verts->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	verts->push_back(osg::Vec3(0.0f, axisLengthMm, 0.0f));
	colors->push_back(yColor);
	colors->push_back(yColor);

	verts->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	verts->push_back(osg::Vec3(0.0f, 0.0f, axisLengthMm));
	colors->push_back(zColor);
	colors->push_back(zColor);

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray(verts.get());
	geom->setColorArray(colors.get(), osg::Array::BIND_PER_VERTEX);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, static_cast<GLsizei>(verts->size())));

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geom.get());
	geode->setNodeMask(OsgScene::kMaskPickOverlay);

	osg::StateSet* ss = geode->getOrCreateStateSet();
	ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	if (alwaysVisible)
	{
		ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
		ss->setRenderBinDetails(100, "RenderBin");
	}
	else
	{
		ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
	}
	ss->setMode(GL_BLEND, osg::StateAttribute::ON);
	osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(lineMotion ? 3.0f : 2.5f);
	ss->setAttributeAndModes(lw.get(), osg::StateAttribute::ON);
	return geode;
}

} // namespace

void OsgWidget::setPerFrameHook(std::function<void(OsgWidget*)> fn)
{
	m_perFrameHook = std::move(fn);
}

void OsgWidget::setRobotObjectGizmoSyncHook(RobotObjectGizmoSyncFn fn)
{
	m_robotObjectGizmoSyncHook = std::move(fn);
}

void OsgWidget::setRobotObjectGizmoFkRefreshHook(RobotObjectGizmoFkRefreshFn fn)
{
	m_robotObjectGizmoFkRefreshHook = std::move(fn);
}

void OsgWidget::syncActiveBackendRootFromObjectFrame(const ObjectGizmoFrame& cur, bool dragging)
{
	if (m_robotObjectGizmoSyncHook && m_robotObjectGizmoSyncHook(cur, dragging))
	{
		OsgScene::syncActiveBackendRootFromObjectFrame(cur, true);
		if (m_robotObjectGizmoFkRefreshHook)
		{
			m_robotObjectGizmoFkRefreshHook(cur, dragging);
		}
		return;
	}
	OsgScene::syncActiveBackendRootFromObjectFrame(cur, dragging);
}

bool OsgWidget::isTransformGizmoDragging() const
{
	return m_dragging || m_rotating || m_tcpTeachDragging || m_tcpTeachRotating;
}

bool OsgWidget::syncOuterPatFromBackend(const BackendDataBase& data)
{
	if (!data.hasPoseProperty())
	{
		return false;
	}
	const std::string id = data.id();
	auto it = m_backendObjectRoots.find(id);
	if (it == m_backendObjectRoots.end() || !it->second.valid())
	{
		return false;
	}
	const auto cIt = m_backendModelCenters.find(id);
	if (cIt == m_backendModelCenters.end())
	{
		return false;
	}
	const osg::Vec3f center = cIt->second;
	const BackendVec3 p = data.pose();
	const BackendVec3 r = data.rotation();
	const osg::Vec3d centerPlusPose(
		static_cast<double>(center.x()) + p.x,
		static_cast<double>(center.y()) + p.y,
		static_cast<double>(center.z()) + p.z);
	const osg::Quat q = backendvisual_math::eulerDegToQuat(
		osg::Vec3f(static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.z)));
	// Same row-vector convention as ObjectGizmoFrame / MeshBackendVisual::buildOuterBranch.
	const osg::Matrixd targetWorld =
		ObjectGizmoFrame::outerLocalMatrix(
			osg::Vec3f(static_cast<float>(centerPlusPose.x()), static_cast<float>(centerPlusPose.y()),
				static_cast<float>(centerPlusPose.z())),
			q);
	setBackendRootWorldMatrixFromWorld(id, targetWorld);
	requestRedraw();
	return true;
}

void OsgWidget::setCameraFollowBackendId(std::string backendId)
{
	m_cameraFollowBackendId = std::move(backendId);
}

void OsgWidget::clearCameraFollowBackendId()
{
	m_cameraFollowBackendId.clear();
}

void OsgWidget::updateCameraFollowCenter()
{
	if (m_cameraFollowBackendId.empty() || !m_trackballManipulator.valid())
	{
		return;
	}
	osg::Matrixd w;
	if (!getBackendRootWorldMatrix(m_cameraFollowBackendId, w))
	{
		return;
	}
	m_trackballManipulator->setCenter(w.getTrans());
}

bool OsgWidget::getBackendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const
{
	const auto it = m_backendObjectRoots.find(backendId);
	if (it == m_backendObjectRoots.end() || !it->second.valid())
	{
		return false;
	}
	const osg::MatrixTransform* mt = it->second.get();
	outWorld = osg::computeLocalToWorld(nodePathToSceneRoot(mt));
	return true;
}

void OsgWidget::setBackendRootWorldMatrixFromWorld(const std::string& backendId, const osg::Matrixd& worldMat)
{
	const auto it = m_backendObjectRoots.find(backendId);
	if (it == m_backendObjectRoots.end() || !it->second.valid())
	{
		return;
	}
	osg::MatrixTransform* mt = it->second.get();
	osg::Matrixd parentWorld;
	parentWorld.makeIdentity();
	// Prefer backend hierarchy parent world (matches FK / RobotKinematicsDBG parentBackendId).
	// Scene-node parent paths can disagree after reparent (flat group vs per-link outer PAT chain).
	const auto parentRel = m_backendParentIds.find(backendId);
	if (parentRel != m_backendParentIds.end() && !parentRel->second.empty())
	{
		if (!getBackendRootWorldMatrix(parentRel->second, parentWorld))
		{
			parentWorld.makeIdentity();
		}
	}
	else
	{
		const osg::NodePath fullPath = nodePathToSceneRoot(mt);
		if (fullPath.size() >= 2U)
		{
			osg::NodePath parentPath;
			parentPath.reserve(fullPath.size() - 1U);
			for (unsigned i = 0; i + 1U < fullPath.size(); ++i)
			{
				parentPath.push_back(fullPath[i]);
			}
			parentWorld = osg::computeLocalToWorld(parentPath);
		}
	}
	const osg::Matrixd invPw = osg::Matrixd::inverse(parentWorld);
	// Row-vector OSG convention (see ObjectGizmoFrame): p_world = p_local * local * parentWorld.
	// Hence local = worldMat * inv(parentWorld). Column-order inv(P)*W only matches when P is identity.
	mt->setMatrix(worldMat * invPw);
	if (m_viewer.valid())
	{
		m_viewer->setSceneData(m_root.get());
	}
	requestRedraw();
}

bool OsgWidget::tryGetBackendModelCenterMm(const std::string& backendId, double& outCx, double& outCy, double& outCz) const
{
	const auto it = m_backendModelCenters.find(backendId);
	if (it == m_backendModelCenters.end())
	{
		return false;
	}
	outCx = static_cast<double>(it->second.x());
	outCy = static_cast<double>(it->second.y());
	outCz = static_cast<double>(it->second.z());
	return true;
}

void OsgWidget::syncRobotMeshBackendPoseAfterKinematics(const BackendDataBase& mesh)
{
	if (const auto* m = dynamic_cast<const MeshBackendData*>(&mesh))
	{
		(void)syncOuterPatFromBackend(*m);
	}
}

void OsgWidget::setInstructionPoseAxes(const std::vector<RobotOsgUi::InstructionPoseAxis>& axes)
{
	if (!m_trajectoryOverlayGroup.valid())
	{
		return;
	}
	for (const osg::ref_ptr<osg::MatrixTransform>& t : m_instructionPoseAxisNodes)
	{
		if (!t.valid())
		{
			continue;
		}
		while (t->getNumParents() > 0)
		{
			t->getParent(0)->removeChild(t.get());
		}
	}
	m_instructionPoseAxisNodes.clear();

	if (!m_instructionPoseAxesGroup.valid())
	{
		m_instructionPoseAxesGroup = new osg::Group;
		m_instructionPoseAxesGroup->setName("InstructionPoseAxes");
		m_instructionPoseAxesGroup->setNodeMask(OsgScene::kMaskPickOverlay);
	}
	// Waypoints are world-fixed markers on the trajectory overlay (not children of moving robot links).
	osg::Group* parentGroup = m_trajectoryOverlayGroup.get();
	const bool axesInRobotAssemblyLocal = false;
	osg::Group* robotAsmRoot = nullptr;
	while (m_instructionPoseAxesGroup->getNumParents() > 0)
	{
		osg::Group* p = m_instructionPoseAxesGroup->getParent(0);
		if (!p)
		{
			break;
		}
		p->removeChild(m_instructionPoseAxesGroup.get());
	}
	if (parentGroup)
	{
		parentGroup->addChild(m_instructionPoseAxesGroup.get());
	}
	m_instructionPoseAxesGroup->removeChildren(0, m_instructionPoseAxesGroup->getNumChildren());

	for (const RobotOsgUi::InstructionPoseAxis& a : axes)
	{
		osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
		mt->setName(a.lineMotion ? "LINE_TargetAxis" : "PTP_TargetAxis");
		const osg::Vec3f p = a.positionMm;
		osg::Matrixd m;
		const osg::Quat q = OsgScene::eulerDegToQuat(a.eulerDeg);
		m.makeRotate(q);
		m.setTrans(static_cast<double>(p.x()), static_cast<double>(p.y()), static_cast<double>(p.z()));
		(void)axesInRobotAssemblyLocal;
		(void)robotAsmRoot;
		mt->setMatrix(m);
		mt->addChild(createReachabilityOriginGeode(a.reachable).get());
		mt->addChild(createInstructionPoseAxisGeode(a.lineMotion ? 100.0f : 80.0f, a.lineMotion).get());
		m_instructionPoseAxesGroup->addChild(mt.get());
		m_instructionPoseAxisNodes.push_back(mt);
	}
	requestRedraw();
}

void OsgWidget::clearInstructionPoseAxes()
{
	for (const osg::ref_ptr<osg::MatrixTransform>& t : m_instructionPoseAxisNodes)
	{
		if (!t.valid())
		{
			continue;
		}
		while (t->getNumParents() > 0)
		{
			t->getParent(0)->removeChild(t.get());
		}
	}
	m_instructionPoseAxisNodes.clear();
	if (m_instructionPoseAxesGroup.valid())
	{
		m_instructionPoseAxesGroup->removeChildren(0, m_instructionPoseAxesGroup->getNumChildren());
	}
	requestRedraw();
}

void OsgWidget::setRawTrajectoryOverlay(const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& points)
{
	if (!m_trajectoryOverlayGroup.valid())
	{
		return;
	}
	if (!m_rawTrajectoryOverlayGeode.valid())
	{
		m_rawTrajectoryOverlayGeode = new osg::Geode;
		m_rawTrajectoryOverlayGeode->setName("RawTrajectoryOverlay");
		m_rawTrajectoryOverlayGeode->setNodeMask(OsgScene::kMaskPickOverlay);
		m_trajectoryOverlayGroup->addChild(m_rawTrajectoryOverlayGeode.get());
	}
	m_rawTrajectoryOverlayGeode->removeDrawables(0, m_rawTrajectoryOverlayGeode->getNumDrawables());
	if (points.size() < 2U)
	{
		requestRedraw();
		return;
	}
	osg::ref_ptr<osg::Vec3Array> lineVerts = new osg::Vec3Array;
	lineVerts->reserve(points.size());
	for (const RobotOsgUi::RawTrajectoryOverlayVertex& v : points)
	{
		lineVerts->push_back(v.positionMm);
	}
	osg::ref_ptr<osg::Geometry> lineGeom = new osg::Geometry;
	lineGeom->setVertexArray(lineVerts.get());
	osg::ref_ptr<osg::Vec4Array> lineColor = new osg::Vec4Array;
	lineColor->push_back(osg::Vec4(0.2f, 0.85f, 1.0f, 1.0f));
	lineGeom->setColorArray(lineColor.get(), osg::Array::BIND_OVERALL);
	lineGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, static_cast<GLsizei>(lineVerts->size())));
	osg::StateSet* ss = lineGeom->getOrCreateStateSet();
	ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	ss->setAttribute(new osg::LineWidth(2.0f));
	m_rawTrajectoryOverlayGeode->addDrawable(lineGeom.get());
	osg::ref_ptr<osg::Vec3Array> ptVerts = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> ptColors = new osg::Vec4Array;
	ptVerts->reserve(points.size());
	ptColors->reserve(points.size());
	for (const RobotOsgUi::RawTrajectoryOverlayVertex& v : points)
	{
		ptVerts->push_back(v.positionMm);
		ptColors->push_back(v.reachable ? osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) : osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
	}
	osg::ref_ptr<osg::Geometry> ptGeom = new osg::Geometry;
	ptGeom->setVertexArray(ptVerts.get());
	ptGeom->setColorArray(ptColors.get(), osg::Array::BIND_PER_VERTEX);
	ptGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, static_cast<GLsizei>(ptVerts->size())));
	osg::StateSet* ptSs = ptGeom->getOrCreateStateSet();
	ptSs->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	ptSs->setAttribute(new osg::Point(5.0f));
	m_rawTrajectoryOverlayGeode->addDrawable(ptGeom.get());
	requestRedraw();
}

void OsgWidget::clearRawTrajectoryOverlay()
{
	if (m_rawTrajectoryOverlayGeode.valid())
	{
		m_rawTrajectoryOverlayGeode->removeDrawables(0, m_rawTrajectoryOverlayGeode->getNumDrawables());
	}
	requestRedraw();
}

void OsgWidget::setRawTrajectoryOverlayFrames(const std::vector<RobotOsgUi::RawTrajectoryOverlayFrame>& frames)
{
	if (!m_trajectoryOverlayGroup.valid())
	{
		return;
	}
	if (!m_rawTrajectoryFramesGroup.valid())
	{
		m_rawTrajectoryFramesGroup = new osg::Group;
		m_rawTrajectoryFramesGroup->setName("RawTrajectoryOverlayFrames");
		m_rawTrajectoryFramesGroup->setNodeMask(OsgScene::kMaskPickOverlay);
		m_trajectoryOverlayGroup->addChild(m_rawTrajectoryFramesGroup.get());
	}
	m_rawTrajectoryFramesGroup->removeChildren(0, m_rawTrajectoryFramesGroup->getNumChildren());
	const float axisLenMm = 15.0f;
	for (const RobotOsgUi::RawTrajectoryOverlayFrame& f : frames)
	{
		osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
		const osg::Quat q = OsgScene::eulerDegToQuat(f.eulerDeg);
		osg::Matrixd m;
		m.makeRotate(q);
		m.setTrans(static_cast<double>(f.positionMm.x()), static_cast<double>(f.positionMm.y()),
			static_cast<double>(f.positionMm.z()));
		mt->setMatrix(m);
		mt->addChild(createInstructionPoseAxisGeode(axisLenMm, true).get());
		m_rawTrajectoryFramesGroup->addChild(mt.get());
	}
	requestRedraw();
}

void OsgWidget::clearRawTrajectoryOverlayFrames()
{
	if (m_rawTrajectoryFramesGroup.valid())
	{
		m_rawTrajectoryFramesGroup->removeChildren(0, m_rawTrajectoryFramesGroup->getNumChildren());
	}
	requestRedraw();
}

void OsgWidget::clearRobotFrameOverlays(const std::string& robotRootBackendId)
{
	const auto it = m_robotFrameOverlayNodes.find(robotRootBackendId);
	if (it == m_robotFrameOverlayNodes.end())
	{
		return;
	}
	auto detach = [](const osg::ref_ptr<osg::MatrixTransform>& t) {
		if (!t.valid())
		{
			return;
		}
		while (t->getNumParents() > 0)
		{
			t->getParent(0)->removeChild(t.get());
		}
	};
	for (const osg::ref_ptr<osg::MatrixTransform>& t : it->second.toolNodes)
	{
		detach(t);
	}
	for (const osg::ref_ptr<osg::MatrixTransform>& u : it->second.userNodes)
	{
		detach(u);
	}
	m_robotFrameOverlayNodes.erase(it);
	requestRedraw();
}

void OsgWidget::setRobotFrameOverlays(const RobotOsgUi::RobotFrameOverlayUpdate& update)
{
	if (update.robotRootBackendId.empty())
	{
		return;
	}
	clearRobotFrameOverlays(update.robotRootBackendId);
	RobotFrameOverlayNodes nodes;

	auto mountOnParent = [&](const std::string& mountBackendId, osg::MatrixTransform* mt) -> bool {
		if (!mt)
		{
			return false;
		}
		if (!mountBackendId.empty())
		{
			const auto it = m_backendObjectRoots.find(mountBackendId);
			if (it != m_backendObjectRoots.end() && it->second.valid())
			{
				it->second->addChild(mt);
				return true;
			}
		}
		const auto itScene = m_backendObjectRoots.find(update.robotRootBackendId);
		if (itScene != m_backendObjectRoots.end() && itScene->second.valid())
		{
			osg::MatrixTransform* outer = itScene->second.get();
			if (outer->getNumChildren() > 0)
			{
				if (osg::Group* asmRoot = dynamic_cast<osg::Group*>(outer->getChild(0)))
				{
					asmRoot->addChild(mt);
					return true;
				}
			}
			// per-link robot root 可能无 mesh 子 Group，T_base_* 直接挂 outer
			outer->addChild(mt);
			return true;
		}
		return false;
	};

	if (update.showToolFrames)
	{
		for (const RobotOsgUi::RobotFrameOverlayUpdate::ToolEntry& te : update.toolFrames)
		{
			osg::ref_ptr<osg::MatrixTransform> toolMt = new osg::MatrixTransform;
			toolMt->setName(std::string("RobotToolFrame_") + te.name);
			toolMt->setMatrix(te.localMatrix);
			const float axisLen = te.active ? 100.0f : 75.0f;
			toolMt->addChild(createInstructionPoseAxisGeode(axisLen, false, true).get());
			if (mountOnParent(te.mountBackendId, toolMt.get()))
			{
				nodes.toolNodes.push_back(toolMt);
			}
		}
	}
	if (update.showUserFrames)
	{
		for (const RobotOsgUi::RobotFrameOverlayUpdate::UserEntry& ue : update.userFrames)
		{
			osg::ref_ptr<osg::MatrixTransform> userMt = new osg::MatrixTransform;
			userMt->setName(std::string("RobotUserFrame_") + ue.name);
			userMt->setMatrix(ue.localMatrix);
			userMt->addChild(createInstructionPoseAxisGeode(110.0f, false, true).get());
			if (mountOnParent(ue.mountBackendId, userMt.get()))
			{
				nodes.userNodes.push_back(userMt);
			}
		}
	}

	if (!nodes.toolNodes.empty() || !nodes.userNodes.empty())
	{
		m_robotFrameOverlayNodes[update.robotRootBackendId] = std::move(nodes);
	}
	requestRedraw();
}

void OsgWidget::setFeatureCatalogOverlay(const std::vector<RobotOsgUi::FeatureCatalogOverlayItem>& items)
{
	std::vector<FeatureCatalogOverlayItem> converted;
	converted.reserve(items.size());
	for (const RobotOsgUi::FeatureCatalogOverlayItem& item : items)
	{
		FeatureCatalogOverlayItem row;
		row.displayIndex = item.displayIndex;
		row.anchorWorldMm = item.anchorWorldMm;
		row.labelWorldMm = item.labelWorldMm;
		row.hasEdgeSegment = item.hasEdgeSegment;
		row.edgeAWorldMm = item.edgeAWorldMm;
		row.edgeBWorldMm = item.edgeBWorldMm;
		converted.push_back(row);
	}
	OsgScene::setFeatureCatalogOverlay(converted);
}

void OsgWidget::clearFeatureCatalogOverlay()
{
	OsgScene::clearFeatureCatalogOverlay();
}

namespace {

void syncGlWidgetChrome(QWidgetViewer* glWidget, bool dark)
{
	if (!glWidget)
	{
		return;
	}
	const QColor bg = viewerChromeColor(dark);
	QPalette pal = glWidget->palette();
	pal.setColor(QPalette::Window, bg);
	glWidget->setPalette(pal);
	glWidget->setAutoFillBackground(true);
	glWidget->setStyleSheet(QStringLiteral("background-color: %1;").arg(bg.name()));
}

} // namespace

void OsgWidget::setViewerBackgroundForDarkUi(bool dark)
{
	m_darkUiTheme = dark;
	syncGlWidgetChrome(m_glWidget, dark);
	const bool viewerOk = m_viewer.valid() && m_viewer->getCamera();
	if (!viewerOk)
	{
		return;
	}
	// Dark UI: medium-light gray canvas (readable vs. ~53 dock gray). Light UI: near-white.
	const osg::Vec4 color = dark ? osg::Vec4(0.40f, 0.40f, 0.42f, 1.0f) : osg::Vec4(0.98f, 0.98f, 0.98f, 1.0f);
	m_viewer->getCamera()->setClearColor(color);
	const osg::Vec4 annotationTextColor = dark
		? osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)
		: osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f);
	for (auto& a : m_annotations)
	{
		if (a.textDrawable.valid())
		{
			a.textDrawable->setColor(annotationTextColor);
		}
	}
	requestRedraw();
}

void OsgWidget::showEvent(QShowEvent* event)
{
	QWidget::showEvent(event);
	// m_darkUiTheme 由 MainWindow（主题切换 / 新建文档）写入，Host DLL 不链 ApplicationStyle
	setViewerBackgroundForDarkUi(m_darkUiTheme);
}

void OsgWidget::resizeEvent(QResizeEvent* event)
{
	QWidget::resizeEvent(event);
	if (m_glWidget && m_viewer.valid())
	{
		const double dpr = m_glWidget->devicePixelRatio();
		const int framebufferW = static_cast<int>(std::lround(static_cast<double>(m_glWidget->width()) * dpr));
		const int framebufferH = static_cast<int>(std::lround(static_cast<double>(m_glWidget->height()) * dpr));
		syncViewportLayoutFromFramebuffer(framebufferW, framebufferH);
	}
}

void OsgWidget::syncViewportLayoutFromFramebuffer(int framebufferWidth, int framebufferHeight)
{
	const int fbW = (std::max)(1, framebufferWidth);
	const int fbH = (std::max)(1, framebufferHeight);
	const double dpr = m_glWidget ? m_glWidget->devicePixelRatio() : 1.0;
	const int logicalW = (std::max)(1, static_cast<int>(std::lround(static_cast<double>(fbW) / dpr)));
	const int logicalH = (std::max)(1, static_cast<int>(std::lround(static_cast<double>(fbH) / dpr)));
	setViewportPixels(logicalW, logicalH);
	setDevicePixelRatio(dpr);
	if (m_viewer.valid() && m_viewer->getCamera())
	{
		m_viewer->getCamera()->setViewport(0, 0, fbW, fbH);
		const double aspect = static_cast<double>(fbW) / static_cast<double>(fbH);
		m_viewer->getCamera()->setProjectionMatrixAsPerspective(30.0, aspect, 10.0, 1e8);
	}
	updateWorldAxesHudViewport(fbW, fbH);
	updateViewCubeHudViewport(fbW, fbH);
	refreshPolylinePickScreenOverlayLayout();
	requestRedraw();
}

void OsgWidget::clearStagingGeometry()
{
	if (m_stagingGroup.valid())
	{
		m_stagingGroup->removeChildren(0, m_stagingGroup->getNumChildren());
	}
}

osg::Node* OsgWidget::stagingGeometryRoot() const
{
	if (!m_stagingGroup.valid() || m_stagingGroup->getNumChildren() < 1)
	{
		return nullptr;
	}
	return m_stagingGroup->getChild(0);
}

void OsgWidget::applyVisibilityMaskForBackend(const std::string& backendId)
{
	auto it = m_backendObjectRoots.find(backendId);
	if (it == m_backendObjectRoots.end() || !it->second.valid())
	{
		return;
	}
	bool vis = true;
	auto v = m_backendVisibility.find(backendId);
	if (v != m_backendVisibility.end())
	{
		vis = v->second;
	}
	it->second->setNodeMask(vis ? 0xffffffffu : 0u);
}

void OsgWidget::setBackendObjectVisible(const std::string& backendId, bool visible)
{
	m_backendVisibility[backendId] = visible;
	applyVisibilityMaskForBackend(backendId);
	requestRedraw();
}

void OsgWidget::setBackendParent(const std::string& backendId, const std::string& parentBackendId)
{
	OsgWidgetTransformHierarchyController::setBackendParent(*this, backendId, parentBackendId);
}

void OsgWidget::setBackendLogicalParent(const std::string& backendId, const std::string& parentBackendId)
{
	OsgScene::setBackendLogicalParent(backendId, parentBackendId);
}

void OsgWidget::removeBackendObjectVisual(const std::string& backendId)
{
	OsgWidgetTransformHierarchyController::removeBackendObjectVisual(*this, backendId);
}

bool OsgWidget::hasBackendObjectBranch(const std::string& backendId) const
{
	const auto it = m_backendObjectRoots.find(backendId);
	return it != m_backendObjectRoots.end() && it->second.valid();
}

void OsgWidget::syncSelectionFromBackend(const PointCloudBackendData& data)
{
	OsgScene::syncGizmoAndPickFromBackend(data);
	syncCompassGizmoOrientation();
	OsgWidgetTransformHierarchyController::finalizeSelectionSync(*this);
}

void OsgWidget::syncSelectionFromBackend(const MeshBackendData& data)
{
	OsgScene::syncGizmoAndPickFromBackend(data);
	syncCompassGizmoOrientation();
	OsgWidgetTransformHierarchyController::finalizeSelectionSync(*this);
}

void OsgWidget::syncSelectionForBackendId(const std::string& backendId)
{
	OsgWidgetTransformHierarchyController::syncSelectionForBackendId(*this, backendId);
}

void OsgWidget::setPickVisualAlias(const std::string& logicalBackendId, const std::string& visualBackendId)
{
	OsgScene::setPickVisualAlias(logicalBackendId, visualBackendId);
}

bool OsgWidget::backendSkipsInnerModelCenterRebase(const std::string& backendId) const
{
	const auto it = m_backendSkipCenterRebase.find(backendId);
	return it != m_backendSkipCenterRebase.end() && it->second;
}

osg::ref_ptr<osg::Geode> OsgWidget::buildPointCloudGeode(const PointCloudBackendData& data, QString* errorMessage) const
{
	std::string err;
	osg::ref_ptr<osg::Geode> geode = BackendVisualRegistry::buildPointCloudGeode(data, errorMessage ? &err : nullptr);
	if (!geode && errorMessage)
	{
		*errorMessage = QString::fromStdString(err);
	}
	return geode;
}

bool OsgWidget::upsertPointCloudBranchInScene(const PointCloudBackendData& data, QString* errorMessage, bool resetViewToHome)
{
	MeshVisualOptions meshOpts{};
	BranchBuildResult built;
	std::string err;
	if (!BackendVisualRegistry::buildOuterBranch(data, meshOpts, built, errorMessage ? &err : nullptr))
	{
		if (errorMessage)
		{
			*errorMessage = QString::fromStdString(err);
		}
		return false;
	}
	osg::ref_ptr<osg::MatrixTransform> outer = built.outer;
	const std::string id = data.id();
	const osg::Vec3f center = built.modelCenter;
	const float diagonal = built.diagonal;
	auto it = m_backendObjectRoots.find(id);
	if (it != m_backendObjectRoots.end() && it->second.valid() && m_backendObjectsGroup.valid())
	{
		m_backendObjectsGroup->removeChild(it->second.get());
	}
	unbindBackendVisualRoot(id);
	m_backendObjectRoots.erase(id);
	m_backendObjectsGroup->addChild(outer.get());
	const auto inserted = m_backendObjectRoots.insert(std::make_pair(id, std::move(outer)));
	if (inserted.second && inserted.first->second.valid())
	{
		bindBackendVisualRoot(id, inserted.first->second.get(), built.brepArtifacts);
	}
	m_backendModelCenters[id] = center;
	if (m_activeBackendId == id || m_activeBackendId.empty())
	{
		m_activeModelDiagonal = diagonal;
	}
	if (m_backendVisibility.find(id) == m_backendVisibility.end())
	{
		m_backendVisibility[id] = true;
	}
	applyVisibilityMaskForBackend(id);
	if (m_viewer.valid())
	{
		m_viewer->setSceneData(m_root.get());
		if (resetViewToHome)
		{
			outer->dirtyBound();
			focusCameraOnBackend(id);
		}
	}
	return true;
}

osg::ref_ptr<osg::Node> OsgWidget::buildMeshGeode(const MeshBackendData& data, QString* errorMessage,
	bool showWireOutline, bool useSceneLighting) const
{
	MeshVisualOptions opt;
	opt.showWireOutline = showWireOutline;
	opt.useSceneLighting = useSceneLighting;
	std::string err;
	osg::ref_ptr<osg::Node> node = BackendVisualRegistry::buildMeshDisplayNode(data, opt, errorMessage ? &err : nullptr);
	if (!node && errorMessage)
	{
		*errorMessage = QString::fromStdString(err);
	}
	return node;
}

bool OsgWidget::upsertBackendBranchInScene(const BackendDataBase& data, QString* errorMessage, bool resetViewToHome,
	bool showWireOutline, bool useSceneLighting, bool skipInnerModelCenterRebase)
{
	MeshVisualOptions meshOpts;
	meshOpts.showWireOutline = showWireOutline;
	meshOpts.useSceneLighting = useSceneLighting;
	meshOpts.skipInnerModelCenterRebase = skipInnerModelCenterRebase;
	BranchBuildResult built;
	std::string err;
	if (!BackendVisualRegistry::buildOuterBranch(data, meshOpts, built, errorMessage ? &err : nullptr))
	{
		if (errorMessage)
		{
			*errorMessage = QString::fromStdString(err);
		}
		return false;
	}
	osg::ref_ptr<osg::MatrixTransform> outer = built.outer;
	const std::string id = data.id();
	if (useSceneLighting)
	{
		m_litMeshBackendIds.insert(id);
	}
	else
	{
		m_litMeshBackendIds.erase(id);
	}
	const osg::Vec3f center = built.modelCenter;
	const float diagonal = built.diagonal;
	m_backendSkipCenterRebase[id] = skipInnerModelCenterRebase;
	auto it = m_backendObjectRoots.find(id);
	if (it != m_backendObjectRoots.end() && it->second.valid() && m_backendObjectsGroup.valid())
	{
		m_backendObjectsGroup->removeChild(it->second.get());
	}
	unbindBackendVisualRoot(id);
	m_backendObjectRoots.erase(id);
	m_backendObjectsGroup->addChild(outer.get());
	const auto inserted = m_backendObjectRoots.insert(std::make_pair(id, std::move(outer)));
	if (inserted.second && inserted.first->second.valid())
	{
		bindBackendVisualRoot(id, inserted.first->second.get(), built.brepArtifacts);
	}
	m_backendModelCenters[id] = center;
	if (m_activeBackendId == id || m_activeBackendId.empty())
	{
		m_activeModelDiagonal = diagonal;
	}
	if (m_backendVisibility.find(id) == m_backendVisibility.end())
	{
		m_backendVisibility[id] = true;
	}
	applyVisibilityMaskForBackend(id);
	if (m_viewer.valid())
	{
		m_viewer->setSceneData(m_root.get());
		if (resetViewToHome)
		{
			outer->dirtyBound();
			focusCameraOnBackend(id);
		}
	}
	return true;
}

bool OsgWidget::upsertMeshBranchInScene(const MeshBackendData& data, QString* errorMessage, bool resetViewToHome,
	bool showWireOutline, bool useSceneLighting, bool skipInnerModelCenterRebase)
{
	return upsertBackendBranchInScene(data, errorMessage, resetViewToHome, showWireOutline, useSceneLighting,
		skipInnerModelCenterRebase);
}

bool OsgWidget::importModelFile(const QString& filePath, QString* errorMessage)
{
	return m_importController
		? m_importController->importModelFile(*this, filePath, errorMessage)
		: false;
}

osg::Node* OsgWidget::loadXyzPointCloud(const QString& filePath, QString* errorMessage)
{
	QFile file(filePath);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		if (errorMessage) *errorMessage = QStringLiteral("Cannot open xyz file.");
		return nullptr;
	}

	QTextStream in(&file);
	osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
	points->reserve(50000);

	while (!in.atEnd())
	{
		const QString line = in.readLine().trimmed();
		if (line.isEmpty() || line.startsWith('#'))
		{
			continue;
		}

		const QStringList parts = line.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
		if (parts.size() < 3)
		{
			continue;
		}

		bool okX = false, okY = false, okZ = false;
		const float x = parts[0].toFloat(&okX);
		const float y = parts[1].toFloat(&okY);
		const float z = parts[2].toFloat(&okZ);
		if (okX && okY && okZ)
		{
			points->push_back(osg::Vec3(x, y, z));
		}
	}

	if (points->empty())
	{
		if (errorMessage) *errorMessage = QStringLiteral("No valid points found in xyz file.");
		return nullptr;
	}

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
	geometry->setVertexArray(points.get());
	geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, static_cast<GLsizei>(points->size())));

	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(0.65f, 0.82f, 0.95f, 1.0f));
	geometry->setColorArray(colors.get(), osg::Array::BIND_OVERALL);

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geometry.get());
	geode->getOrCreateStateSet()->setAttribute(new osg::Point(2.0f));
	return geode.release();
}

osg::Node* OsgWidget::loadAsciiPlyPointCloud(const QString& filePath, QString* errorMessage)
{
	QFile file(filePath);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		if (errorMessage) *errorMessage = QStringLiteral("Cannot open ply file.");
		return nullptr;
	}

	QTextStream in(&file);
	QString line;
	bool isAscii = false;
	bool headerEnded = false;
	int vertexCount = 0;

	while (!in.atEnd())
	{
		line = in.readLine().trimmed();
		if (line.startsWith(QStringLiteral("format ascii")))
		{
			isAscii = true;
		}
		else if (line.startsWith(QStringLiteral("element vertex")))
		{
			const QStringList parts = line.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
			if (parts.size() >= 3) vertexCount = parts[2].toInt();
		}
		else if (line == QStringLiteral("end_header"))
		{
			headerEnded = true;
			break;
		}
	}

	if (!headerEnded || !isAscii || vertexCount <= 0)
	{
		if (errorMessage) *errorMessage = QStringLiteral("Only ascii ply fallback is supported.");
		return nullptr;
	}

	osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
	points->reserve(static_cast<unsigned int>(vertexCount));
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->reserve(static_cast<unsigned int>(vertexCount));

	for (int i = 0; i < vertexCount && !in.atEnd(); ++i)
	{
		line = in.readLine().trimmed();
		if (line.isEmpty()) continue;
		const QStringList parts = line.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
		if (parts.size() < 3) continue;
		bool okX = false, okY = false, okZ = false;
		const float x = parts[0].toFloat(&okX);
		const float y = parts[1].toFloat(&okY);
		const float z = parts[2].toFloat(&okZ);
		if (!(okX && okY && okZ)) continue;
		points->push_back(osg::Vec3(x, y, z));

		if (parts.size() >= 6)
		{
			bool okR = false, okG = false, okB = false;
			const float r = parts[3].toFloat(&okR) / 255.0f;
			const float g = parts[4].toFloat(&okG) / 255.0f;
			const float b = parts[5].toFloat(&okB) / 255.0f;
			if (okR && okG && okB) colors->push_back(osg::Vec4(r, g, b, 1.0f));
			else colors->push_back(osg::Vec4(0.65f, 0.82f, 0.95f, 1.0f));
		}
		else
		{
			colors->push_back(osg::Vec4(0.65f, 0.82f, 0.95f, 1.0f));
		}
	}

	if (points->empty())
	{
		if (errorMessage) *errorMessage = QStringLiteral("No valid vertex data in ply.");
		return nullptr;
	}

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
	geometry->setVertexArray(points.get());
	geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, static_cast<GLsizei>(points->size())));
	geometry->setColorArray(colors.get(), osg::Array::BIND_PER_VERTEX);

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geometry.get());
	geode->getOrCreateStateSet()->setAttribute(new osg::Point(2.0f));
	return geode.release();
}

bool OsgWidget::importPointCloudFile(const QString& filePath, QString* errorMessage)
{
	return m_importController
		? m_importController->importPointCloudFile(*this, filePath, errorMessage)
		: false;
}

void OsgWidget::initUi()
{
	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->setSpacing(0);

	QGLFormat fmt;
	fmt.setDepthBufferSize(24);
	fmt.setDoubleBuffer(true);

	m_glWidget = new QWidgetViewer(fmt, this);
	// 勿用过大 minimumSize，否则会占满主窗口垂直空间、底部运行信息 Dock 无法拉高
	m_glWidget->setMinimumSize(200, 120);
	m_glWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	layout->addWidget(m_glWidget, 1);
	m_glWidget->installEventFilter(this);
}

void OsgWidget::initViewer()
{
	m_viewer = new osgViewer::Viewer;
	m_viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);
	m_viewer->setRunFrameScheme(osgViewer::Viewer::CONTINUOUS);

	// GraphicsContext must be attached to the main camera before setSceneData, otherwise OSG can
	// touch invalid viewer/GC state during scene compile (often Debug-only access violations).
	auto* gwQt = new GraphicsWindowQt1(m_glWidget);
	m_graphicsWindow = gwQt;
	gwQt->setViewer(m_viewer.get());
	m_glWidget->setGraphicsWindow(gwQt);

	connect(m_glWidget, &QWidgetViewer::windowResized, this, [this](int w, int h) {
		if (m_graphicsWindow.valid())
		{
			static_cast<GraphicsWindowQt1*>(m_graphicsWindow.get())->updateSize(w, h);
		}
		syncViewportLayoutFromFramebuffer(w, h);
	});

	m_viewer->getCamera()->setGraphicsContext(m_graphicsWindow.get());
	m_viewer->getCamera()->setCullMask(0xffffffffu);
	// Some OSG builds crash in Debug when viewport/projection are set before the graphics context is fully valid.
	// Defer these calls unless the context reports valid; resize callback will configure them again.
	if (m_graphicsWindow.valid() && m_graphicsWindow->valid() && m_glWidget)
	{
		const double dpr = m_glWidget->devicePixelRatio();
		const int deviceW = static_cast<int>(std::lround(static_cast<double>(m_glWidget->width()) * dpr));
		const int deviceH = static_cast<int>(std::lround(static_cast<double>(m_glWidget->height()) * dpr));
		m_viewer->getCamera()->setViewport(0, 0, deviceW, deviceH);
		const double aspect = static_cast<double>((std::max)(1, deviceW))
			/ static_cast<double>((std::max)(1, deviceH));
		m_viewer->getCamera()->setProjectionMatrixAsPerspective(30.0, aspect, 10.0, 1e8);
		syncViewportLayoutFromFramebuffer(deviceW, deviceH);
	}

	applyHeadlightToViewer(m_viewer.get());
	m_viewer->setSceneData(m_root.get());
	m_viewer->addEventHandler(new osgGA::StateSetManipulator(m_viewer->getCamera()->getOrCreateStateSet()));

	m_trackballManipulator = new osgGA::TrackballManipulator;
	m_trackballManipulator->setAllowThrow(false);
	m_viewer->setCameraManipulator(m_trackballManipulator.get());
	m_viewer->getEventQueue()->syncWindowRectangleWithGraphicsContext();

	setViewerBackgroundForDarkUi(m_darkUiTheme);
	m_viewer->getCamera()->setViewMatrixAsLookAt(
		osg::Vec3(3, 3, 3),
		osg::Vec3(0, 0, 0),
		osg::Vec3(0, 0, 1));

	connect(&m_frameTimer, &QTimer::timeout, this, [this]() {
		if (m_perFrameHook)
		{
			m_perFrameHook(this);
		}
		updateCameraFollowCenter();
		updateCompassScale();
		if (m_pickAnnotationController)
		{
			m_pickAnnotationController->updateAnnotationScales(*this);
		}
		const bool timerDrivenVisuals = !cameraFollowBackendId().empty() || hasPointAnnotations();
		if (timerDrivenVisuals)
		{
			requestRedraw();
		}
	});
	m_frameTimer.start(16);

	m_idleRenderTimer.setSingleShot(true);
	m_idleRenderTimer.setInterval(500);
	connect(&m_idleRenderTimer, &QTimer::timeout, this, [this]() {
		if (m_viewer.valid())
		{
			m_viewer->setRunFrameScheme(osgViewer::Viewer::ON_DEMAND);
		}
	});
	m_idleRenderTimer.start();

	OsgScene::initWorldAxesHud();
	OsgScene::initViewCubeHud();
	applyViewCubeFaceLabelImagesFromQt();
}

void OsgWidget::applyViewCubeFaceLabelImagesFromQt()
{
	static const QString kLabels[] = {
		QStringLiteral("\u9876"),
		QStringLiteral("\u5e95"),
		QStringLiteral("\u524d"),
		QStringLiteral("\u540e"),
		QStringLiteral("\u53f3"),
		QStringLiteral("\u5de6"),
	};
	osg::ref_ptr<osg::Image> images[6];
	for (int i = 0; i < 6; ++i)
	{
		images[i] = makeViewCubeLabelImage(kLabels[i]);
	}
	applyViewCubeFaceLabelImages(images);
}

void OsgWidget::noteViewportInteraction()
{
	if (!m_viewer.valid())
	{
		return;
	}
	if (m_viewer->getRunFrameScheme() != osgViewer::Viewer::CONTINUOUS)
	{
		m_viewer->setRunFrameScheme(osgViewer::Viewer::CONTINUOUS);
	}
	m_idleRenderTimer.start();
}

osg::Node* OsgWidget::createCompassNode()
{
	return OsgScene::createCompassNode();
}

void OsgWidget::setSelectionActive(bool active)
{
	m_selectionActive = active;
	if (!active)
	{
		detachGizmoOverlay();
		m_gizmoReferenceDistance = -1.0;
		m_gizmoReferenceScale = 1.0;
		m_hasLastSelectionPose = false;
	}
	updateCompassHighlight(DragAxis::None);
	emit activeAxisChanged(QStringLiteral("None"));
	refreshCompassDrawVisibility();
}

void OsgWidget::attachCompassGraphics()
{
	OsgScene::attachCompassGraphics();
}

void OsgWidget::detachCompassGraphics()
{
	OsgScene::detachCompassGraphics();
}

void OsgWidget::syncCompassGizmoOrientation()
{
	OsgScene::syncCompassGizmoOrientation();
}

void OsgWidget::logGizmoPivotDiagnostics(const char* reasonTag) const
{
	OsgScene::logGizmoPivotDiagnostics(reasonTag);
}

void OsgWidget::refreshCompassDrawVisibility()
{
	OsgScene::refreshCompassDrawVisibility();
	syncCompassGizmoOrientation();
}

void OsgWidget::syncCameraManipulatorForModes()
{
	if (!m_viewer.valid())
	{
		return;
	}
	// 多边形裁剪：左键用于加点，须禁用轨道球避免拖动转视图
	const bool blockCameraNav = m_polylinePickMode;
	// resetPosition=false keeps the current camera when re-attaching the trackball after object/point modes
	// or ESC (OSG default true would jump to viewer "home" every time).
	m_viewer->setCameraManipulator(blockCameraNav ? nullptr : m_trackballManipulator.get(), false);
	if (!blockCameraNav)
	{
		// Point/object modes swallow mouse presses in Qt but releases still reach osgGA; the
		// accumulated button mask then disagrees with reality and TrackballManipulator stops responding.
		resetNavigationInputQueues();
		if (m_glWidget)
		{
			m_glWidget->setFocus(Qt::OtherFocusReason);
			requestRedraw();
		}
	}
}

bool OsgWidget::pickAndActivateBackendAtScreenPos(const QPoint& mousePos)
{
	if (!m_glWidget)
	{
		return false;
	}
	const bool ok = OsgScene::pickAndActivateBackendAtScreenPos(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()));
	if (ok)
	{
		refreshAnnotationTexts();
		setSelectionActive(true);
		syncCompassGizmoOrientation();
		if (!m_activeBackendId.empty())
		{
			emit backendObjectPicked(QString::fromStdString(m_activeBackendId));
		}
	}
	return ok;
}

void OsgWidget::setObjectSelectionMode(bool enabled)
{
	m_objectSelectionMode = enabled;
	if (enabled)
	{
		m_pointPickMode = false;
		m_meshLinePickMode = false;
		m_meshFacePickMode = false;
		// Drop any in-progress view navigation before Qt starts swallowing mouse for gizmo.
		resetNavigationInputQueues();
	}
	syncCameraManipulatorForModes();
	if (!enabled)
	{
		m_dragging = false;
		m_rotating = false;
		m_dragAxis = DragAxis::None;
		updateCompassHighlight(DragAxis::None);
		emit activeAxisChanged(QStringLiteral("None"));
	}
	refreshCompassDrawVisibility();
}

bool OsgWidget::objectSelectionMode() const
{
	return m_objectSelectionMode;
}

void OsgWidget::setTransformGizmoFrame(TransformGizmoFrame frame)
{
	m_transformGizmoFrame = frame;
	syncCompassGizmoOrientation();
	if (m_tcpTeachActive)
	{
		updateTcpDragTeachFromTarget(m_tcpTeachTargetInBase);
	}
}

void OsgWidget::setPointPickMode(bool enabled)
{
	m_pointPickMode = enabled;
	if (enabled)
	{
		m_objectSelectionMode = false;
		if (m_polylinePickMode)
		{
			m_polylinePickMode = false;
			clearPolylinePickOverlay();
		}
		m_meshLinePickMode = false;
		m_meshFacePickMode = false;
		m_dragging = false;
		m_rotating = false;
		m_dragAxis = DragAxis::None;
		updateCompassHighlight(DragAxis::None);
		emit activeAxisChanged(QStringLiteral("None"));
		emit pointPickFeedback(QStringLiteral("Point Pick: move mouse over point cloud."));
		resetNavigationInputQueues();
	}
	else
	{
		clearPointPickMarker();
		emit pointPickFeedback(QStringLiteral("Point Pick: off"));
	}
	syncCameraManipulatorForModes();
	refreshCompassDrawVisibility();
}

bool OsgWidget::pointPickMode() const
{
	return m_pointPickMode;
}

void OsgWidget::setPolylinePickMode(bool enabled)
{
	m_polylinePickMode = enabled;
	if (enabled)
	{
		m_objectSelectionMode = false;
		m_pointPickMode = false;
		m_meshLinePickMode = false;
		m_meshFacePickMode = false;
		m_dragging = false;
		m_rotating = false;
		m_dragAxis = DragAxis::None;
		updateCompassHighlight(DragAxis::None);
		emit activeAxisChanged(QStringLiteral("None"));
		emit polylinePickFeedback(
			QStringLiteral("Polyline crop: left-click vertices, right-click/double-click close, Esc cancel"));
		resetNavigationInputQueues();
		clearPolylinePickScreenOverlay();
	}
	else
	{
		clearPolylinePickScreenOverlay();
		emit polylinePickFeedback(QStringLiteral("Polyline crop: off"));
	}
	syncCameraManipulatorForModes();
	refreshCompassDrawVisibility();
}

bool OsgWidget::polylinePickMode() const
{
	return m_polylinePickMode;
}

void OsgWidget::updatePolylinePickOverlay(const std::vector<QPoint>& vertices, const QPoint* cursorPos)
{
	std::vector<float> screenXy;
	screenXy.reserve(vertices.size() * 2U);
	for (const QPoint& p : vertices)
	{
		screenXy.push_back(static_cast<float>(p.x()));
		screenXy.push_back(static_cast<float>(p.y()));
	}
	float cursorX = 0.0f;
	float cursorY = 0.0f;
	const float* cursorXPtr = nullptr;
	const float* cursorYPtr = nullptr;
	if (cursorPos != nullptr)
	{
		cursorX = static_cast<float>(cursorPos->x());
		cursorY = static_cast<float>(cursorPos->y());
		cursorXPtr = &cursorX;
		cursorYPtr = &cursorY;
	}
	updatePolylinePickScreenOverlay(screenXy, cursorXPtr, cursorYPtr);
}

void OsgWidget::commitPolylinePick(const std::vector<QPoint>& vertices)
{
	if (vertices.size() < 3U || !m_viewer.valid() || !m_viewer->getCamera())
	{
		emit polylinePickCanceled();
		return;
	}
	QVector<float> polylineScreenXy;
	polylineScreenXy.reserve(static_cast<int>(vertices.size() * 2U));
	for (const QPoint& p : vertices)
	{
		polylineScreenXy.push_back(static_cast<float>(p.x()));
		polylineScreenXy.push_back(static_cast<float>(p.y()));
	}
	const osg::Matrixd mvp =
		m_viewer->getCamera()->getViewMatrix() * m_viewer->getCamera()->getProjectionMatrix();
	QVector<double> mvpOut;
	mvpOut.resize(16);
	for (int col = 0; col < 4; ++col)
	{
		for (int row = 0; row < 4; ++row)
		{
			mvpOut[col * 4 + row] = mvp(row, col);
		}
	}
	const int vw = viewportWidth();
	const int vh = viewportHeight();
	setPolylinePickMode(false);
	emit polylinePickCommitted(polylineScreenXy, mvpOut, vw, vh);
}

void OsgWidget::clearPolylinePickOverlay()
{
	clearPolylinePickScreenOverlay();
}

void OsgWidget::setMeshLinePickMode(bool enabled)
{
	m_meshLinePickMode = enabled;
	if (enabled)
	{
		m_objectSelectionMode = false;
		m_pointPickMode = false;
		if (m_polylinePickMode)
		{
			m_polylinePickMode = false;
			clearPolylinePickOverlay();
		}
		m_meshFacePickMode = false;
		m_dragging = false;
		m_rotating = false;
		m_dragAxis = DragAxis::None;
		updateCompassHighlight(DragAxis::None);
		emit activeAxisChanged(QStringLiteral("None"));
		hideMeshElementHighlight();
		emit meshPickFeedback(QStringLiteral("Mesh Line Pick: move mouse over mesh edges."));
		resetNavigationInputQueues();
	}
	else
	{
		hideMeshElementHighlight();
		emit meshPickFeedback(QStringLiteral("Mesh Line Pick: off"));
	}
	syncCameraManipulatorForModes();
	refreshCompassDrawVisibility();
}

bool OsgWidget::meshLinePickMode() const
{
	return m_meshLinePickMode;
}

void OsgWidget::setMeshFacePickMode(bool enabled)
{
	m_meshFacePickMode = enabled;
	if (enabled)
	{
		m_objectSelectionMode = false;
		m_pointPickMode = false;
		if (m_polylinePickMode)
		{
			m_polylinePickMode = false;
			clearPolylinePickOverlay();
		}
		m_meshLinePickMode = false;
		m_dragging = false;
		m_rotating = false;
		m_dragAxis = DragAxis::None;
		updateCompassHighlight(DragAxis::None);
		emit activeAxisChanged(QStringLiteral("None"));
		hideMeshElementHighlight();
		emit meshPickFeedback(QStringLiteral("Mesh Face Pick: move mouse over mesh faces."));
		resetNavigationInputQueues();
	}
	else
	{
		hideMeshElementHighlight();
		emit meshPickFeedback(QStringLiteral("Mesh Face Pick: off"));
	}
	syncCameraManipulatorForModes();
	refreshCompassDrawVisibility();
}

bool OsgWidget::meshFacePickMode() const
{
	return m_meshFacePickMode;
}

PickResult OsgWidget::queryPick(const PickQuery& query)
{
	return OsgScene::queryPick(query);
}

QString OsgWidget::pointCloudPluginReport() const
{
	auto hasReader = [](const char* ext) -> bool {
		return osgDB::Registry::instance()->getReaderWriterForExtension(ext) != nullptr;
	};

	const QString ply = hasReader("ply") ? QStringLiteral("OK") : QStringLiteral("Missing");
	const QString las = hasReader("las") ? QStringLiteral("OK") : QStringLiteral("Missing");
	const QString laz = hasReader("laz") ? QStringLiteral("OK") : QStringLiteral("Missing");
	const QString xyz = QStringLiteral("OK (built-in)");
	return QStringLiteral("PointCloud plugin check | ply:%1 las:%2 laz:%3 xyz:%4")
		.arg(ply).arg(las).arg(laz).arg(xyz);
}

osg::Vec3f OsgWidget::selectedPosition() const
{
	ObjectGizmoFrame f;
	if (!readActiveObjectGizmoFrame(f))
	{
		return osg::Vec3f(0.0f, 0.0f, 0.0f);
	}
	return f.backendPoseRelativeToCenter();
}

void OsgWidget::setSelectedPosition(const osg::Vec3f& position)
{
	if (!m_activeBackendOuterPat.valid())
	{
		return;
	}
	ObjectGizmoFrame f;
	if (!readActiveObjectGizmoFrame(f))
	{
		return;
	}
	f.setFromBackend(position, f.attitude(), m_modelCenter);
	f.applyToOuter(m_activeBackendOuterPat.get());
	syncActiveBackendRootFromObjectFrame(f, false);
	refreshAnnotationTexts();
	syncCompassGizmoOrientation();
	emit selectedObjectPoseChanged(position.x(), position.y(), position.z());
}

osg::Vec3f OsgWidget::selectedRotationEulerDeg() const
{
	ObjectGizmoFrame f;
	if (!readActiveObjectGizmoFrame(f))
	{
		return osg::Vec3f(0.0f, 0.0f, 0.0f);
	}
	return OsgScene::quatToEulerDeg(f.attitude());
}

void OsgWidget::setSelectedRotationEulerDeg(const osg::Vec3f& eulerDeg)
{
	if (!m_activeBackendOuterPat.valid())
	{
		return;
	}
	ObjectGizmoFrame f;
	if (!readActiveObjectGizmoFrame(f))
	{
		return;
	}
	const osg::Quat q = OsgScene::eulerDegToQuat(eulerDeg);
	const osg::Vec3d pivotInParent = ObjectGizmoFrame::pivotInOuterParentFromOuter(
		m_activeBackendOuterPat.get(), f.modelCenter());
	f.setRotationKeepingPivotInOuterParent(pivotInParent, q);
	f.applyToOuter(m_activeBackendOuterPat.get());
	syncActiveBackendRootFromObjectFrame(f, false);
	refreshAnnotationTexts();
	syncCompassGizmoOrientation();
	emit selectedObjectRotationChanged(eulerDeg.x(), eulerDeg.y(), eulerDeg.z());
}

void OsgWidget::syncActiveBackendRootFromSelectedTransform()
{
	ObjectGizmoFrame f;
	if (!readActiveObjectGizmoFrame(f))
	{
		return;
	}
	syncActiveBackendRootFromObjectFrame(f, false);
}

bool OsgWidget::writeActiveBackendPoseFromOsg(BackendDataBase& data)
{
	if (!data.hasPoseProperty() || data.id() != m_activeBackendId || !m_activeBackendOuterPat.valid())
	{
		return false;
	}
	const osg::Vec3f pose = selectedPosition();
	const osg::Vec3f euler = selectedRotationEulerDeg();
	const BackendVec3 poseB{
		static_cast<double>(pose.x()),
		static_cast<double>(pose.y()),
		static_cast<double>(pose.z())};
	const BackendVec3 eulerB{
		static_cast<double>(euler.x()),
		static_cast<double>(euler.y()),
		static_cast<double>(euler.z())};
	if (data.supportsBackendTransform())
	{
		data.applyBackendWorldPose(poseB, eulerB);
	}
	else
	{
		data.setPose(poseB);
	}
	return true;
}

void OsgWidget::setSelectedColor(float r, float g, float b, float a)
{
	OsgWidgetColorController::applyColorToActiveBackendObject(*this, osg::Vec4(r, g, b, a));
	emit selectedObjectColorChanged(r, g, b, a);
}

OsgWidget::DragAxis OsgWidget::pickAxisAtScreenPos(const QPoint& mousePos, bool preferRing, bool* outPickedRing) const
{
	const int axis = OsgScene::pickAxisAtScreenPos(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()), preferRing, outPickedRing);
	return static_cast<DragAxis>(axis);
}

void OsgWidget::updateCompassHighlight(DragAxis axis, bool highlightRing)
{
	OsgScene::updateCompassHighlight(static_cast<int>(axis), highlightRing);
}

QString OsgWidget::axisToString(DragAxis axis) const
{
	if (axis == DragAxis::X) return QStringLiteral("X");
	if (axis == DragAxis::Y) return QStringLiteral("Y");
	if (axis == DragAxis::Z) return QStringLiteral("Z");
	return QStringLiteral("None");
}

void OsgWidget::updateCompassScale()
{
	OsgScene::updateCompassScale();
}

void OsgWidget::clearPointAnnotations()
{
	for (const AnnotationEntry& a : m_annotations)
	{
		emit annotationRemoved(QString::fromStdString(a.id));
		if (!a.transform.valid())
		{
			continue;
		}
		if (m_annotationGroup.valid())
		{
			m_annotationGroup->removeChild(a.transform.get());
		}
	}
	if (m_annotationGroup.valid())
	{
		m_annotationGroup->removeChildren(0, m_annotationGroup->getNumChildren());
	}
	m_annotations.clear();
	m_annotationCounter = 0;
}

bool OsgWidget::pickPointAtScreenPos(const QPoint& mousePos, osg::Vec3f& outPointWorld) const
{
	return OsgScene::pickPointAtScreenPos(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()), outPointWorld);
}

bool OsgWidget::pickNearestPointAtScreenPos(const QPoint& mousePos, osg::Vec3f& outPointWorld, double& outDistancePx, bool previewOnly) const
{
	return OsgScene::pickNearestPointAtScreenPos(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()), outPointWorld, outDistancePx, previewOnly);
}

bool OsgWidget::pickPointByRayIntersection(const QPoint& mousePos, osg::Vec3f& outPointWorld, double& outDistancePx) const
{
	return OsgScene::pickPointByRayIntersection(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()), outPointWorld, outDistancePx);
}

bool OsgWidget::pickMeshFaceByRayIntersection(const QPoint& mousePos,
	osg::Vec3f& outPointWorld,
	osg::Vec3f& outAWorld,
	osg::Vec3f& outBWorld,
	osg::Vec3f& outCWorld,
	osg::Vec3f& outNormalWorld,
	std::vector<osg::Vec3f>* outMergedCoplanarVertsWorld,
	const std::string* scopeBackendId) const
{
	return OsgScene::pickMeshFaceByRayIntersection(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()),
		outPointWorld, outAWorld, outBWorld, outCWorld, outNormalWorld, outMergedCoplanarVertsWorld, scopeBackendId);
}

bool OsgWidget::pickMeshEdgeByRayIntersection(const QPoint& mousePos,
	osg::Vec3f& outPointWorld,
	osg::Vec3f& outEdgeAWorld,
	osg::Vec3f& outEdgeBWorld,
	double* outEdgeDistancePx,
	const std::string* scopeBackendId) const
{
	return OsgScene::pickMeshEdgeByRayIntersection(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()),
		outPointWorld, outEdgeAWorld, outEdgeBWorld, outEdgeDistancePx, scopeBackendId);
}

void OsgWidget::addPointAnnotation(const osg::Vec3f& pointWorld)
{
	if (m_pickAnnotationController)
	{
		m_pickAnnotationController->addPointAnnotation(*this, pointWorld);
	}
}

void OsgWidget::refreshAnnotationTexts()
{
	if (m_pickAnnotationController)
	{
		m_pickAnnotationController->refreshAnnotationTexts(*this);
	}
}

bool OsgWidget::setAnnotationVisible(const QString& annotationId, bool visible)
{
	return m_pickAnnotationController
		? m_pickAnnotationController->setAnnotationVisible(*this, annotationId, visible)
		: false;
}

bool OsgWidget::removeAnnotation(const QString& annotationId)
{
	return m_pickAnnotationController
		? m_pickAnnotationController->removeAnnotation(*this, annotationId)
		: false;
}

void OsgWidget::clearAllAnnotations()
{
	if (m_pickAnnotationController)
	{
		m_pickAnnotationController->clearAllAnnotations(*this);
	}
	else
	{
		clearPointAnnotations();
	}
}

QList<OsgWidget::AnnotationSnapshot> OsgWidget::annotationSnapshots() const
{
	return m_pickAnnotationController
		? m_pickAnnotationController->annotationSnapshots(*this)
		: QList<AnnotationSnapshot>();
}

void OsgWidget::restoreAnnotations(const QList<AnnotationSnapshot>& snapshots)
{
	if (m_pickAnnotationController)
	{
		m_pickAnnotationController->restoreAnnotations(*this, snapshots);
	}
}

void OsgWidget::updatePointPickMarker(const osg::Vec3f& pointWorld, bool hit)
{
	if (m_pickAnnotationController)
	{
		m_pickAnnotationController->updatePointPickMarker(*this, pointWorld, hit);
	}
}

void OsgWidget::clearPointPickMarker()
{
	if (m_pickAnnotationController)
	{
		m_pickAnnotationController->clearPointPickMarker(*this);
	}
}

void OsgWidget::applyColorToStagingGeometry(const osg::Vec4& color)
{
	OsgWidgetColorController::applyColorToStagingGeometry(*this, color);
}

void OsgWidget::applyColorToBackendObject(const std::string& backendId, const osg::Vec4& color)
{
	OsgWidgetColorController::applyColorToBackendObject(*this, backendId, color);
}

void OsgWidget::applyColorToActiveBackendObject(const osg::Vec4& color)
{
	OsgWidgetColorController::applyColorToActiveBackendObject(*this, color);
}

bool OsgWidget::eventFilter(QObject* watched, QEvent* event)
{
	if (watched == m_glWidget)
	{
		const auto type = event->type();
		if (type == QEvent::MouseButtonPress)
		{
			const auto* mouseEvent = static_cast<QMouseEvent*>(event);
			if (mouseEvent->button() == Qt::LeftButton
				&& tryPickViewCubeAtLogicalMouse(static_cast<double>(mouseEvent->x()),
					static_cast<double>(mouseEvent->y())))
			{
				requestRedraw();
				return true;
			}
		}
		if (type == QEvent::MouseMove || type == QEvent::MouseButtonPress || type == QEvent::MouseButtonRelease
			|| type == QEvent::Wheel || type == QEvent::KeyPress || type == QEvent::KeyRelease)
		{
			noteViewportInteraction();
		}
	}

	// Escape: never deliver to QWidgetViewer/osgGA. OSG handlers (e.g. StateSetManipulator stack) and
	// unpaired press/release in the event queue have caused apparent "freezes"; Qt may also propagate
	// Esc to parent shortcuts. We only use Esc to leave edit modes; in pure view mode it is a no-op.
	if (watched == m_glWidget
		&& (event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease))
	{
		const auto* keyEvent = static_cast<QKeyEvent*>(event);
		if (keyEvent->key() == Qt::Key_Escape)
		{
			if (event->type() == QEvent::KeyPress)
			{
				if (m_tcpTeachActive)
				{
					endTcpDragTeach();
					emit tcpDragTeachEnded();
					requestRedraw();
					return true;
				}
				if (m_objectSelectionMode)
				{
					m_dragging = false;
					m_rotating = false;
					m_dragAxis = DragAxis::None;
					setSelectionActive(false);
					setObjectSelectionMode(false);
					emit selectionCanceledByEsc();
					requestRedraw();
					return true;
				}
				if (m_pointPickMode)
				{
					emit selectionCanceledByEsc();
					requestRedraw();
					return true;
				}
				if (m_polylinePickMode)
				{
					setPolylinePickMode(false);
					emit polylinePickCanceled();
					requestRedraw();
					return true;
				}
				if (m_meshLinePickMode || m_meshFacePickMode)
				{
					m_meshLinePickMode = false;
					m_meshFacePickMode = false;
					hideMeshElementHighlight();
					emit selectionCanceledByEsc();
					requestRedraw();
					return true;
				}
			}
			return true;
		}
	}

	if (m_pointPickOperation && m_pointPickOperation->handleEvent(watched, event))
	{
		return true;
	}

	if (m_polylinePickOperation && m_polylinePickOperation->handleEvent(watched, event))
	{
		return true;
	}

	if (m_meshElementPickOperation && m_meshElementPickOperation->handleEvent(watched, event))
	{
		return true;
	}

	if (m_tcpDragTeachOperation && m_tcpDragTeachOperation->handleEvent(watched, event))
	{
		return true;
	}

	if (m_objectTransformOperation && m_objectTransformOperation->handleEvent(watched, event))
	{
		return true;
	}

	return QWidget::eventFilter(watched, event);
}

void OsgWidget::clearImportedContent()
{
	endTcpDragTeach();
	clearInstructionPoseAxes();
	clearStagingGeometry();
	hideMeshElementHighlight();
	if (m_backendObjectsGroup.valid())
	{
		m_backendObjectsGroup->removeChildren(0, m_backendObjectsGroup->getNumChildren());
	}
	if (m_robotAssemblyGroup.valid())
	{
		m_robotAssemblyGroup->removeChildren(0, m_robotAssemblyGroup->getNumChildren());
	}
	if (m_trajectoryOverlayGroup.valid())
	{
		m_trajectoryOverlayGroup->removeChildren(0, m_trajectoryOverlayGroup->getNumChildren());
	}
	clearPointAnnotations();
	m_litMeshBackendIds.clear();
	m_backendObjectRoots.clear();
	clearBackendVisualBindings();
	m_backendParentIds.clear();
	m_backendModelCenters.clear();
	m_backendSkipCenterRebase.clear();
	m_backendVisibility.clear();
	m_hasLastSelectionPose = false;
	m_activeBackendId.clear();
	m_activeBackendOuterPat = nullptr;
	m_pickablePointsLocal.clear();
	m_pickablePointsPreviewLocal.clear();
	m_pickablePointsCenteredLocal.clear();
	m_kdNodes.clear();
	m_kdRoot = -1;
	if (m_viewer.valid())
	{
		m_viewer->setSceneData(m_root.get());
	}
}

bool OsgWidget::captureImportedPointCloudBackend(PointCloudBackendData& out, QString* errorMessage)
{
	return m_captureController
		? m_captureController->captureImportedPointCloudBackend(*this, out, errorMessage)
		: false;
}

bool OsgWidget::captureImportedMeshBackend(MeshBackendData& out, QString* errorMessage)
{
	return m_captureController
		? m_captureController->captureImportedMeshBackend(*this, out, errorMessage)
		: false;
}

bool OsgWidget::captureImportedMeshBackendHierarchy(std::vector<MeshCapturedPart>& outParts, QString* errorMessage)
{
	return m_captureController
		? m_captureController->captureImportedMeshBackendHierarchy(*this, outParts, errorMessage)
		: false;
}

bool OsgWidget::captureViewportPng(QByteArray& outPng, QString* errorMessage, int maxWidth, int maxHeight)
{
	outPng.clear();
	if (!m_viewer.valid() || !m_glWidget || !m_graphicsWindow.valid())
	{
		if (errorMessage)
			*errorMessage = QStringLiteral("3D 视口未就绪");
		return false;
	}
	if (maxWidth <= 0 || maxHeight <= 0)
	{
		if (errorMessage)
			*errorMessage = QStringLiteral("无效的截图尺寸");
		return false;
	}

	if (!m_graphicsWindow->makeCurrent())
	{
		if (errorMessage)
			*errorMessage = QStringLiteral("无法激活 OpenGL 上下文");
		return false;
	}

	m_viewer->frame();

	osg::Camera* cam = m_viewer->getCamera();
	if (!cam)
	{
		if (errorMessage)
			*errorMessage = QStringLiteral("相机未就绪");
		return false;
	}
	const osg::Viewport* vp = cam->getViewport();
	if (!vp || vp->width() <= 0 || vp->height() <= 0)
	{
		if (errorMessage)
			*errorMessage = QStringLiteral("视口尺寸无效");
		return false;
	}

	osg::ref_ptr<osg::Image> image = new osg::Image;
	image->readPixels(static_cast<int>(vp->x()), static_cast<int>(vp->y()), static_cast<int>(vp->width()),
		static_cast<int>(vp->height()), GL_RGBA, GL_UNSIGNED_BYTE);
	image->flipVertical();

	QImage qimg(image->data(), image->s(), image->t(), static_cast<int>(image->getRowSizeInBytes()),
		QImage::Format_RGBA8888);
	QImage copy = qimg.copy();
	if (copy.isNull())
	{
		if (errorMessage)
			*errorMessage = QStringLiteral("读取帧缓冲失败");
		return false;
	}

	if (copy.width() > maxWidth || copy.height() > maxHeight)
		copy = copy.scaled(maxWidth, maxHeight, Qt::KeepAspectRatio, Qt::SmoothTransformation);

	QBuffer buf(&outPng);
	if (!buf.open(QIODevice::WriteOnly) || !copy.save(&buf, "PNG"))
	{
		outPng.clear();
		if (errorMessage)
			*errorMessage = QStringLiteral("PNG 编码失败");
		return false;
	}
	return true;
}

bool OsgWidget::loadPointCloudFromBackendData(const PointCloudBackendData& data, QString* errorMessage, bool resetViewToHome)
{
	return m_backendLoadController
		? m_backendLoadController->loadPointCloudFromBackendData(*this, data, errorMessage, resetViewToHome)
		: false;
}

bool OsgWidget::loadMeshFromBackendData(const MeshBackendData& data, QString* errorMessage, bool resetViewToHome,
	bool showWireOutline, bool useSceneLighting, bool skipInnerModelCenterRebase)
{
	return loadBackendFromBackendData(data, errorMessage, resetViewToHome, showWireOutline, useSceneLighting,
		skipInnerModelCenterRebase);
}

bool OsgWidget::loadBackendFromBackendData(const BackendDataBase& data, QString* errorMessage, bool resetViewToHome,
	bool showWireOutline, bool useSceneLighting, bool skipInnerModelCenterRebase)
{
	return m_backendLoadController
		? m_backendLoadController->loadBackendFromBackendData(*this, data, errorMessage, resetViewToHome, showWireOutline,
			  useSceneLighting, skipInnerModelCenterRebase)
		: false;
}

bool OsgWidget::isBackendMeshLit(const std::string& backendId) const
{
	return m_litMeshBackendIds.find(backendId) != m_litMeshBackendIds.end();
}

// 添加层级化机器人场景图（动态层级法）
QString OsgWidget::addHierarchicalRobotScene(osg::Group* robotAssembly, const QString& displayName)
{
	if (!robotAssembly || !m_robotAssemblyGroup.valid())
	{
		return QString();
	}

	// 生成唯一的后端 ID
	static int s_robotSceneCounter = 0;
	const QString backendId = QStringLiteral("RobotScene_%1_%2")
		.arg(displayName.isEmpty() ? QStringLiteral("URDF") : displayName)
		.arg(++s_robotSceneCounter);
	const std::string stdId = backendId.toStdString();

	// 与其它后端一致：外层 MatrixTransform 存完整局部刚体矩阵，FK / setBackendRootWorldMatrixFromWorld 无 TRS 分解损失
	osg::ref_ptr<osg::MatrixTransform> outer = new osg::MatrixTransform;
	outer->setMatrix(osg::Matrixd::identity());
	outer->setName(displayName.isEmpty() ? "RobotHierarchy" : displayName.toStdString());
	robotAssembly->dirtyBound();
	outer->addChild(robotAssembly);
	m_robotAssemblyGroup->addChild(outer.get());
	outer->dirtyBound();
	const osg::BoundingSphere bs = outer->getBound();
	// Gizmo / pose pivot use m_modelCenter as the "file origin" in outer local space (see
	// computeGizmoPivotWorld from inner PAT). URDF assembly is rooted at
	// the base link frame under this PAT — not at the meshes' bounding-sphere center (which would
	// shift the compass off the pedestal for typical arms).
	m_backendModelCenters[stdId] = osg::Vec3f(0.0f, 0.0f, 0.0f);
	if (bs.valid() && m_activeBackendId.empty())
	{
		m_activeModelDiagonal = (std::max)(1.0f, bs.radius() * 2.0f);
	}
	if (m_backendVisibility.find(stdId) == m_backendVisibility.end())
	{
		m_backendVisibility[stdId] = true;
	}
	applyVisibilityMaskForBackend(stdId);

	// 禁止对 map 使用 operator[] 赋值 ref_ptr（MSVC + OSG 3.6.5 在 ref_ptr::assign 上 C2440）
	const auto inserted = m_backendObjectRoots.insert(std::make_pair(stdId, std::move(outer)));
	if (inserted.second && inserted.first->second.valid())
	{
		bindBackendVisualRoot(stdId, inserted.first->second.get());
	}
	m_litMeshBackendIds.insert(stdId);

	// 刷新场景并对准相机（与 upsertMeshBranchInScene 一致，否则易停留在默认视角看不到模型）
	if (m_viewer.valid())
	{
		m_viewer->setSceneData(m_root.get());
		focusCameraOnBackend(stdId);
	}
	requestRedraw();

	return backendId;
}

// 移除层级化机器人场景图
void OsgWidget::removeHierarchicalRobotScene(const QString& backendId)
{
	if (backendId.isEmpty())
	{
		return;
	}
	const std::string stdId = backendId.toStdString();

	auto it = m_backendObjectRoots.find(stdId);
	if (it != m_backendObjectRoots.end() && it->second.valid() && m_robotAssemblyGroup.valid())
	{
		m_robotAssemblyGroup->removeChild(it->second.get());
		m_backendObjectRoots.erase(it);
	}
	unbindBackendVisualRoot(stdId);
	m_litMeshBackendIds.erase(stdId);

	if (m_viewer.valid())
	{
		m_viewer->setSceneData(m_root.get());
	}
}

