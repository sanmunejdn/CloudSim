/// @file OsgWidget.cpp
/// @brief OsgWidget 实现

#include "OsgWidget.h"

#include "BackendDataBase.h"
#include "BackendFollowMath.h"
#include "BackendPoseOsg.h"
#include "BackendVisualMath.h"
#include "BackendVisualRegistry.h"
#include "GraphicsWindowQt1.h"
#include "MeshBackendData.h"
#include "MeshEdgeFacePickOperation.h"
#include "ObjectGizmoFrame.h"
#include "ObjectTransformOperation.h"
#include "OsgWidgetBackendLoadController.h"
#include "OsgWidgetCaptureController.h"
#include "OsgWidgetColorController.h"
#include "OsgWidgetImportController.h"
#include "OsgWidgetPickAnnotationController.h"
#include "OsgWidgetTransformHierarchyController.h"
#include "PointCloudBackendData.h"
#include "PointPickOperation.h"
#include "QWidgetViewer.h"
#include "RobotTcpDragTeachOperation.h"

#include <QFile>
#include <QFileInfo>
#include <QMouseEvent>
#include <QRegExp>
#include <QStringList>
#include <QTextStream>
#include <QVBoxLayout>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>

#include <BrepImportArtifacts.h>
#include <osg/Array>
#include <osg/AutoTransform>
#include <osg/BlendFunc>
#include <osg/BoundingSphere>
#include <osg/Camera>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/Matrix>
#include <osg/MatrixTransform>
#include <osg/Node>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>
#include <osg/Point>
#include <osg/PolygonOffset>
#include <osg/PositionAttitudeTransform>
#include <osg/PrimitiveSet>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/StateAttribute>
#include <osg/StateSet>
#include <osg/Transform>
#include <osg/Vec4>
#include <osgDB/Options>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osgGA/EventQueue>
#include <osgGA/GUIEventAdapter>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgText/Text>
#include <osgViewer/GraphicsWindow>
#include <osgViewer/Viewer>

OsgWidget::OsgWidget(QWidget* parent) : QWidget(parent)
{
	m_feedbackTimer.start();
	m_pointPickOperation = std::make_unique<PointPickOperation>(this);
	m_objectTransformOperation = std::make_unique<ObjectTransformOperation>(this);
	m_tcpDragTeachOperation = std::make_unique<RobotTcpDragTeachOperation>(this);
	m_meshElementPickOperation = std::make_unique<MeshEdgeFacePickOperation>(this);
	m_importController = std::make_unique<OsgWidgetImportController>();
	m_backendLoadController = std::make_unique<OsgWidgetBackendLoadController>();
	m_captureController = std::make_unique<OsgWidgetCaptureController>();
	m_pickAnnotationController = std::make_unique<OsgWidgetPickAnnotationController>();
	initScene();
	initUi();
	setRequestRedraw(
		[this]()
		{
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

void OsgWidget::applyRigidRotationAboutWorldPivot(const std::vector<std::string>& backendIds,
												  const osg::Vec3f& pivotWorld, const osg::Quat& deltaRotation)
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
			: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), wantName(std::move(w))
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
	osg::StateSet* ss = geode->getOrCreateStateSet();
	ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	return geode;
}

osg::ref_ptr<osg::Geode> createInstructionPoseAxisGeode(float axisLengthMm, bool lineMotion, bool alwaysVisible = false)
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
	const osg::Matrixd targetWorld = backend_pose_osg::osgMatrixFromBackendWorldMatrix(data.worldMatrix());
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

bool OsgWidget::tryGetBackendModelCenterMm(const std::string& backendId, double& outCx, double& outCy,
										   double& outCz) const
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

void OsgWidget::setInstructionPoseAxes(const std::vector<InstructionPoseAxis>& axes)
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

	for (const InstructionPoseAxis& a : axes)
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

void OsgWidget::clearRobotFrameOverlays(const std::string& robotRootBackendId)
{
	const auto it = m_robotFrameOverlayNodes.find(robotRootBackendId);
	if (it == m_robotFrameOverlayNodes.end())
	{
		return;
	}
	auto detach = [](const osg::ref_ptr<osg::MatrixTransform>& t)
	{
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

void OsgWidget::setRobotFrameOverlays(const RobotFrameOverlayUpdate& update)
{
	if (update.robotRootBackendId.empty())
	{
		return;
	}
	clearRobotFrameOverlays(update.robotRootBackendId);
	RobotFrameOverlayNodes nodes;

	auto mountOnParent = [&](const std::string& mountBackendId, osg::MatrixTransform* mt) -> bool
	{
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
			outer->addChild(mt);
			return true;
		}
		return false;
	};

	if (update.showToolFrames)
	{
		for (const RobotFrameOverlayUpdate::ToolEntry& te : update.toolFrames)
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
		for (const RobotFrameOverlayUpdate::UserEntry& ue : update.userFrames)
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

void OsgWidget::setViewerBackgroundForDarkUi(bool dark)
{
	m_darkUiTheme = dark;
	if (!m_viewer.valid() || !m_viewer->getCamera())
	{
		return;
	}
	// Dark UI: medium-light gray canvas (readable vs. ~53 dock gray). Light UI: near-white.
	const osg::Vec4 color = dark ? osg::Vec4(0.40f, 0.40f, 0.42f, 1.0f) : osg::Vec4(0.98f, 0.98f, 0.98f, 1.0f);
	m_viewer->getCamera()->setClearColor(color);
	const osg::Vec4 annotationTextColor = dark ? osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) : osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f);
	for (auto& a : m_annotations)
	{
		if (a.textDrawable.valid())
		{
			a.textDrawable->setColor(annotationTextColor);
		}
	}
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

bool OsgWidget::upsertPointCloudBranchInScene(const PointCloudBackendData& data, QString* errorMessage,
											  bool resetViewToHome)
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
	osg::ref_ptr<osg::Node> node =
		BackendVisualRegistry::buildMeshDisplayNode(data, opt, errorMessage ? &err : nullptr);
	if (!node && errorMessage)
	{
		*errorMessage = QString::fromStdString(err);
	}
	return node;
}

bool OsgWidget::upsertMeshBranchInScene(const MeshBackendData& data, QString* errorMessage, bool resetViewToHome,
										bool showWireOutline, bool useSceneLighting)
{
	MeshVisualOptions meshOpts;
	meshOpts.showWireOutline = showWireOutline;
	meshOpts.useSceneLighting = useSceneLighting;
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

bool OsgWidget::importModelFile(const QString& filePath, QString* errorMessage)
{
	return m_importController ? m_importController->importModelFile(*this, filePath, errorMessage) : false;
}

osg::Node* OsgWidget::loadXyzPointCloud(const QString& filePath, QString* errorMessage)
{
	QFile file(filePath);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		if (errorMessage)
			*errorMessage = QStringLiteral("Cannot open xyz file.");
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
		if (errorMessage)
			*errorMessage = QStringLiteral("No valid points found in xyz file.");
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
		if (errorMessage)
			*errorMessage = QStringLiteral("Cannot open ply file.");
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
			if (parts.size() >= 3)
				vertexCount = parts[2].toInt();
		}
		else if (line == QStringLiteral("end_header"))
		{
			headerEnded = true;
			break;
		}
	}

	if (!headerEnded || !isAscii || vertexCount <= 0)
	{
		if (errorMessage)
			*errorMessage = QStringLiteral("Only ascii ply fallback is supported.");
		return nullptr;
	}

	osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
	points->reserve(static_cast<unsigned int>(vertexCount));
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->reserve(static_cast<unsigned int>(vertexCount));

	for (int i = 0; i < vertexCount && !in.atEnd(); ++i)
	{
		line = in.readLine().trimmed();
		if (line.isEmpty())
			continue;
		const QStringList parts = line.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
		if (parts.size() < 3)
			continue;
		bool okX = false, okY = false, okZ = false;
		const float x = parts[0].toFloat(&okX);
		const float y = parts[1].toFloat(&okY);
		const float z = parts[2].toFloat(&okZ);
		if (!(okX && okY && okZ))
			continue;
		points->push_back(osg::Vec3(x, y, z));

		if (parts.size() >= 6)
		{
			bool okR = false, okG = false, okB = false;
			const float r = parts[3].toFloat(&okR) / 255.0f;
			const float g = parts[4].toFloat(&okG) / 255.0f;
			const float b = parts[5].toFloat(&okB) / 255.0f;
			if (okR && okG && okB)
				colors->push_back(osg::Vec4(r, g, b, 1.0f));
			else
				colors->push_back(osg::Vec4(0.65f, 0.82f, 0.95f, 1.0f));
		}
		else
		{
			colors->push_back(osg::Vec4(0.65f, 0.82f, 0.95f, 1.0f));
		}
	}

	if (points->empty())
	{
		if (errorMessage)
			*errorMessage = QStringLiteral("No valid vertex data in ply.");
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
	return m_importController ? m_importController->importPointCloudFile(*this, filePath, errorMessage) : false;
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
	// 勿用过大 minimumSize，否则会占满主窗口垂直空间、底部运行信息 Dock 无法拉高。
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

	connect(m_glWidget, &QWidgetViewer::windowResized, this,
			[this](int w, int h)
			{
				if (m_graphicsWindow.valid())
				{
					static_cast<GraphicsWindowQt1*>(m_graphicsWindow.get())->updateSize(w, h);
				}
				setViewportPixels(w, h);
				if (m_glWidget)
				{
					setDevicePixelRatio(m_glWidget->devicePixelRatio());
				}
				if (m_viewer.valid() && m_viewer->getCamera())
				{
					m_viewer->getCamera()->setViewport(0, 0, w, h);
					const double aspect = static_cast<double>((std::max)(1, w)) / static_cast<double>((std::max)(1, h));
					m_viewer->getCamera()->setProjectionMatrixAsPerspective(30.0, aspect, 10.0, 1e8);
				}
				updateWorldAxesHudViewport(w, h);
				requestRedraw();
			});

	m_viewer->getCamera()->setGraphicsContext(m_graphicsWindow.get());
	m_viewer->getCamera()->setCullMask(0xffffffffu);
	// Some OSG builds crash in Debug when viewport/projection are set before the graphics context is fully valid.
	// Defer these calls unless the context reports valid; resize callback will configure them again.
	if (m_graphicsWindow.valid() && m_graphicsWindow->valid())
	{
		m_viewer->getCamera()->setViewport(0, 0, m_glWidget->width(), m_glWidget->height());
		const double aspect = static_cast<double>((std::max)(1, m_glWidget->width())) /
							  static_cast<double>((std::max)(1, m_glWidget->height()));
		m_viewer->getCamera()->setProjectionMatrixAsPerspective(30.0, aspect, 10.0, 1e8);
	}

	applyHeadlightToViewer(m_viewer.get());
	m_viewer->setSceneData(m_root.get());
	m_viewer->addEventHandler(new osgGA::StateSetManipulator(m_viewer->getCamera()->getOrCreateStateSet()));

	m_trackballManipulator = new osgGA::TrackballManipulator;
	m_trackballManipulator->setAllowThrow(false);
	m_viewer->setCameraManipulator(m_trackballManipulator.get());
	m_viewer->getEventQueue()->syncWindowRectangleWithGraphicsContext();

	setViewerBackgroundForDarkUi(false);
	m_viewer->getCamera()->setViewMatrixAsLookAt(osg::Vec3(3, 3, 3), osg::Vec3(0, 0, 0), osg::Vec3(0, 0, 1));

	connect(&m_frameTimer, &QTimer::timeout, this,
			[this]()
			{
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

	OsgScene::initWorldAxesHud();
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
	// Keep camera navigation enabled in all modes. Specific edit operations
	// consume events only when they really need to (e.g. dragging gizmo axis).
	const bool blockCameraNav = false;
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
	const bool ok = OsgScene::pickAndActivateBackendAtScreenPos(static_cast<double>(mousePos.x()),
																static_cast<double>(mousePos.y()));
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

void OsgWidget::setMeshLinePickMode(bool enabled)
{
	m_meshLinePickMode = enabled;
	if (enabled)
	{
		m_objectSelectionMode = false;
		m_pointPickMode = false;
		m_meshFacePickMode = false;
		m_dragging = false;
		m_rotating = false;
		m_dragAxis = DragAxis::None;
		updateCompassHighlight(DragAxis::None);
		emit activeAxisChanged(QStringLiteral("None"));
		hideMeshElementHighlight();
		emit pointPickFeedback(QStringLiteral("Mesh Line Pick: move mouse over mesh edges."));
		resetNavigationInputQueues();
	}
	else
	{
		hideMeshElementHighlight();
		emit pointPickFeedback(QStringLiteral("Mesh Line Pick: off"));
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
		m_meshLinePickMode = false;
		m_dragging = false;
		m_rotating = false;
		m_dragAxis = DragAxis::None;
		updateCompassHighlight(DragAxis::None);
		emit activeAxisChanged(QStringLiteral("None"));
		hideMeshElementHighlight();
		emit pointPickFeedback(QStringLiteral("Mesh Face Pick: move mouse over mesh faces."));
		resetNavigationInputQueues();
	}
	else
	{
		hideMeshElementHighlight();
		emit pointPickFeedback(QStringLiteral("Mesh Face Pick: off"));
	}
	syncCameraManipulatorForModes();
	refreshCompassDrawVisibility();
}

bool OsgWidget::meshFacePickMode() const
{
	return m_meshFacePickMode;
}

QString OsgWidget::pointCloudPluginReport() const
{
	auto hasReader = [](const char* ext) -> bool
	{ return osgDB::Registry::instance()->getReaderWriterForExtension(ext) != nullptr; };

	// xyz uses built-in loader in this widget, so it is always available.
	const QString ply = hasReader("ply") ? QStringLiteral("OK") : QStringLiteral("Missing");
	const QString las = hasReader("las") ? QStringLiteral("OK") : QStringLiteral("Missing");
	const QString laz = hasReader("laz") ? QStringLiteral("OK") : QStringLiteral("Missing");
	const QString xyz = QStringLiteral("OK (built-in)");
	return QStringLiteral("PointCloud plugin check | ply:%1 las:%2 laz:%3 xyz:%4").arg(ply).arg(las).arg(laz).arg(xyz);
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
	f.setCenterPlusPoseAndAttitude(f.centerPlusPose(), q);
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
	osg::Matrixd world;
	if (!getBackendRootWorldMatrix(m_activeBackendId, world))
	{
		return false;
	}
	BackendVec3 poseB{};
	BackendVec3 eulerB{};
	backend_pose_osg::backendPoseEulerFromWorldMatrix(world, poseB, eulerB);
	if (data.supportsBackendTransform())
	{
		data.applyBackendWorldPose(poseB, eulerB);
	}
	else
	{
		data.setPose(poseB);
		if (data.hasRotationProperty())
		{
			data.setRotation(eulerB);
		}
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
	const int axis = OsgScene::pickAxisAtScreenPos(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()),
												   preferRing, outPickedRing);
	return static_cast<DragAxis>(axis);
}

void OsgWidget::updateCompassHighlight(DragAxis axis, bool highlightRing)
{
	OsgScene::updateCompassHighlight(static_cast<int>(axis), highlightRing);
}

QString OsgWidget::axisToString(DragAxis axis) const
{
	if (axis == DragAxis::X)
		return QStringLiteral("X");
	if (axis == DragAxis::Y)
		return QStringLiteral("Y");
	if (axis == DragAxis::Z)
		return QStringLiteral("Z");
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
		// Ensure legacy gizmo-attached annotations are removed too.
		m_annotationGroup->removeChildren(0, m_annotationGroup->getNumChildren());
	}
	m_annotations.clear();
	m_annotationCounter = 0;
}

bool OsgWidget::pickPointAtScreenPos(const QPoint& mousePos, osg::Vec3f& outPointWorld) const
{
	return OsgScene::pickPointAtScreenPos(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()),
										  outPointWorld);
}

bool OsgWidget::pickNearestPointAtScreenPos(const QPoint& mousePos, osg::Vec3f& outPointWorld, double& outDistancePx,
											bool previewOnly) const
{
	return OsgScene::pickNearestPointAtScreenPos(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()),
												 outPointWorld, outDistancePx, previewOnly);
}

bool OsgWidget::pickPointByRayIntersection(const QPoint& mousePos, osg::Vec3f& outPointWorld,
										   double& outDistancePx) const
{
	return OsgScene::pickPointByRayIntersection(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()),
												outPointWorld, outDistancePx);
}

bool OsgWidget::pickMeshFaceByRayIntersection(const QPoint& mousePos, osg::Vec3f& outPointWorld, osg::Vec3f& outAWorld,
											  osg::Vec3f& outBWorld, osg::Vec3f& outCWorld, osg::Vec3f& outNormalWorld,
											  std::vector<osg::Vec3f>* outMergedCoplanarVertsWorld) const
{
	return OsgScene::pickMeshFaceByRayIntersection(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()),
												   outPointWorld, outAWorld, outBWorld, outCWorld, outNormalWorld,
												   outMergedCoplanarVertsWorld);
}

bool OsgWidget::pickMeshEdgeByRayIntersection(const QPoint& mousePos, osg::Vec3f& outPointWorld,
											  osg::Vec3f& outEdgeAWorld, osg::Vec3f& outEdgeBWorld) const
{
	return OsgScene::pickMeshEdgeByRayIntersection(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()),
												   outPointWorld, outEdgeAWorld, outEdgeBWorld);
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
	return m_pickAnnotationController ? m_pickAnnotationController->setAnnotationVisible(*this, annotationId, visible)
									  : false;
}

bool OsgWidget::removeAnnotation(const QString& annotationId)
{
	return m_pickAnnotationController ? m_pickAnnotationController->removeAnnotation(*this, annotationId) : false;
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
	return m_pickAnnotationController ? m_pickAnnotationController->annotationSnapshots(*this)
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
	// Escape: never deliver to QWidgetViewer/osgGA. OSG handlers (e.g. StateSetManipulator stack) and
	// unpaired press/release in the event queue have caused apparent "freezes"; Qt may also propagate
	// Esc to parent shortcuts. We only use Esc to leave edit modes; in pure view mode it is a no-op.
	if (watched == m_glWidget && (event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease))
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
	return m_captureController ? m_captureController->captureImportedPointCloudBackend(*this, out, errorMessage)
							   : false;
}

bool OsgWidget::captureImportedMeshBackend(MeshBackendData& out, QString* errorMessage)
{
	return m_captureController ? m_captureController->captureImportedMeshBackend(*this, out, errorMessage) : false;
}

bool OsgWidget::captureImportedMeshBackendHierarchy(std::vector<MeshCapturedPart>& outParts, QString* errorMessage)
{
	return m_captureController ? m_captureController->captureImportedMeshBackendHierarchy(*this, outParts, errorMessage)
							   : false;
}

bool OsgWidget::loadPointCloudFromBackendData(const PointCloudBackendData& data, QString* errorMessage,
											  bool resetViewToHome)
{
	return m_backendLoadController
			   ? m_backendLoadController->loadPointCloudFromBackendData(*this, data, errorMessage, resetViewToHome)
			   : false;
}

bool OsgWidget::loadMeshFromBackendData(const MeshBackendData& data, QString* errorMessage, bool resetViewToHome,
										bool showWireOutline, bool useSceneLighting)
{
	return m_backendLoadController ? m_backendLoadController->loadMeshFromBackendData(
										 *this, data, errorMessage, resetViewToHome, showWireOutline, useSceneLighting)
								   : false;
}

bool OsgWidget::isBackendMeshLit(const std::string& backendId) const
{
	return m_litMeshBackendIds.find(backendId) != m_litMeshBackendIds.end();
}

QString OsgWidget::addHierarchicalRobotScene(osg::Group* robotAssembly, const QString& displayName)
{
	if (!robotAssembly || !m_robotAssemblyGroup.valid())
	{
		return QString();
	}

	static int s_robotSceneCounter = 0;
	const QString backendId = QStringLiteral("RobotScene_%1_%2")
								  .arg(displayName.isEmpty() ? QStringLiteral("URDF") : displayName)
								  .arg(++s_robotSceneCounter);
	const std::string stdId = backendId.toStdString();

	// 外层 Matrix 存完整刚体，FK 写回无 TRS 损失
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

	// map 勿 operator[] 赋 ref_ptr，MSVC+OSG3.6 C2440
	const auto inserted = m_backendObjectRoots.insert(std::make_pair(stdId, std::move(outer)));
	if (inserted.second && inserted.first->second.valid())
	{
		bindBackendVisualRoot(stdId, inserted.first->second.get());
	}
	m_litMeshBackendIds.insert(stdId);

	// 导入后须 focus，否则默认视角看不到模型
	if (m_viewer.valid())
	{
		m_viewer->setSceneData(m_root.get());
		focusCameraOnBackend(stdId);
	}
	requestRedraw();

	return backendId;
}

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
