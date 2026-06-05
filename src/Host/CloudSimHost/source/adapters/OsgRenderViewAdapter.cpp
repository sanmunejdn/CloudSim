#include "adapters/OsgRenderViewAdapter.h"

#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "IDataService.h"
#include "OsgWidget.h"

#include "RobotOsgUiTypes.h"

#include <Adapters.h>
#include <RigidTransform.h>

#include "BackendDataManager.h"

#include <osg/Matrixd>
#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Camera>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osg/AutoTransform>

namespace cloudsim::host {

OsgRenderViewAdapter::OsgRenderViewAdapter(OsgWidget& widget) : m_widget(widget) {}

OsgRenderViewAdapter::OsgRenderViewAdapter(OsgWidget& widget, DocumentHost& host)
	: m_widget(widget)
	, m_host(&host)
{
}

OsgRenderViewAdapter::OsgRenderViewAdapter(DocumentHost& host)
	: OsgRenderViewAdapter(*osgWidgetFrom(host), host)
{
}

QWidget* OsgRenderViewAdapter::widget()
{
	return &m_widget;
}

const QWidget* OsgRenderViewAdapter::widget() const
{
	return &m_widget;
}

void OsgRenderViewAdapter::setWorldMatrix(const core::ObjectId& id, const core::Mat4& columnMajor)
{
	osg::Matrixd m;
	for (int i = 0; i < 16; ++i)
	{
		m.ptr()[i] = columnMajor[static_cast<size_t>(i)];
	}
	m_widget.setBackendRootWorldMatrixFromWorld(id.toStdString(), m);
}

bool OsgRenderViewAdapter::getWorldMatrix(const core::ObjectId& id, core::Mat4& outColumnMajor) const
{
	osg::Matrixd m;
	if (!m_widget.getBackendRootWorldMatrix(id.toStdString(), m))
	{
		return false;
	}
	for (int i = 0; i < 16; ++i)
	{
		outColumnMajor[static_cast<size_t>(i)] = m.ptr()[i];
	}
	return true;
}

void OsgRenderViewAdapter::setVisible(const core::ObjectId& id, bool visible)
{
	m_widget.setBackendObjectVisible(id.toStdString(), visible);
}

void OsgRenderViewAdapter::removeVisual(const core::ObjectId& id)
{
	m_widget.removeBackendObjectVisual(id.toStdString());
}

bool OsgRenderViewAdapter::hasVisualBranch(const core::ObjectId& id) const
{
	return m_widget.hasBackendObjectBranch(id.toStdString());
}

bool OsgRenderViewAdapter::tryGetModelCenterMm(const core::ObjectId& id, double& outCx, double& outCy,
	double& outCz) const
{
	return m_widget.tryGetBackendModelCenterMm(id.toStdString(), outCx, outCy, outCz);
}

void OsgRenderViewAdapter::setPickHandler(core::PickHandler handler)
{
	m_pickHandler = std::move(handler);
}

void OsgRenderViewAdapter::clearPickHandler()
{
	m_pickHandler = nullptr;
}

void OsgRenderViewAdapter::requestRedraw()
{
	m_widget.requestRedraw();
}

void OsgRenderViewAdapter::setSelectionActive(const bool active)
{
	m_widget.setSelectionActive(active);
}

void OsgRenderViewAdapter::clearInstructionPoseAxes()
{
	m_widget.clearInstructionPoseAxes();
}

bool OsgRenderViewAdapter::hasImportedContent() const
{
	return m_widget.hasImportedContent();
}

bool OsgRenderViewAdapter::isTcpDragTeachActive() const
{
	return m_widget.isTcpDragTeachActive();
}

bool OsgRenderViewAdapter::isTransformGizmoDragging() const
{
	return m_widget.isTransformGizmoDragging();
}

void OsgRenderViewAdapter::setAnnotationVisible(const core::ObjectId& annotationId, const bool visible)
{
	m_widget.setAnnotationVisible(annotationId, visible);
}

bool OsgRenderViewAdapter::removeAnnotation(const core::ObjectId& annotationId)
{
	return m_widget.removeAnnotation(annotationId);
}

void OsgRenderViewAdapter::clearAllAnnotations()
{
	m_widget.clearAllAnnotations();
}

QVector<core::AnnotationSnapshotDto> OsgRenderViewAdapter::annotationSnapshots() const
{
	const QList<OsgWidget::AnnotationSnapshot> snaps = m_widget.annotationSnapshots();
	QVector<core::AnnotationSnapshotDto> out;
	out.reserve(snaps.size());
	for (const OsgWidget::AnnotationSnapshot& s : snaps)
	{
		core::AnnotationSnapshotDto dto;
		dto.id = s.id;
		dto.displayText = s.displayText;
		dto.visible = s.visible;
		out.push_back(dto);
	}
	return out;
}

void OsgRenderViewAdapter::focusCameraOnBackend(const core::ObjectId& id)
{
	m_widget.focusCameraOnBackend(id.toStdString());
}

void OsgRenderViewAdapter::setBackendLogicalParent(const core::ObjectId& childId, const core::ObjectId& parentId)
{
	m_widget.setBackendLogicalParent(childId.toStdString(), parentId.toStdString());
}

namespace {

QString formatMatrix(const osg::Matrixd& m)
{
	QString s;
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			if (c > 0)
			{
				s += QLatin1Char(' ');
			}
			s += QString::number(m(r, c), 'g', 6);
		}
		if (r < 3)
		{
			s += QLatin1Char('\n');
		}
	}
	return s;
}

QString localMatrixSummary(const osg::Node* node)
{
	if (!node)
	{
		return QStringLiteral("—");
	}
	if (const auto* cam = dynamic_cast<const osg::Camera*>(node))
	{
		return QStringLiteral("View:\n%1\nProj:\n%2").arg(formatMatrix(cam->getViewMatrix()))
			.arg(formatMatrix(cam->getProjectionMatrix()));
	}
	if (const auto* mt = dynamic_cast<const osg::MatrixTransform*>(node))
	{
		return formatMatrix(mt->getMatrix());
	}
	if (const auto* pat = dynamic_cast<const osg::PositionAttitudeTransform*>(node))
	{
		return formatMatrix(osg::Matrixd::translate(pat->getPosition()) * osg::Matrixd::rotate(pat->getAttitude())
			* osg::Matrixd::scale(pat->getScale()));
	}
	if (const auto* at = dynamic_cast<const osg::AutoTransform*>(node))
	{
		return formatMatrix(osg::Matrixd::translate(at->getPosition()) * osg::Matrixd::rotate(at->getRotation())
			* osg::Matrixd::scale(at->getScale()));
	}
	return QStringLiteral("—");
}

void buildSnapshotRecursive(core::IRenderView::SceneNodeInfo& info, const osg::Node* node, int depthLeft)
{
	if (!node || depthLeft <= 0)
	{
		return;
	}
	info.className = QString::fromLatin1(node->className());
	info.name = QString::fromStdString(node->getName());
	info.localMatrixSummary = localMatrixSummary(node);
	if (const auto* g = node->asGroup())
	{
		for (unsigned i = 0; i < g->getNumChildren(); ++i)
		{
			core::IRenderView::SceneNodeInfo child;
			buildSnapshotRecursive(child, g->getChild(i), depthLeft - 1);
			info.children.push_back(std::move(child));
		}
	}
}

} // namespace

core::IRenderView::SceneNodeInfo OsgRenderViewAdapter::sceneGraphSnapshot(int maxDepth) const
{
	SceneNodeInfo root;
	const osg::Node* sceneRoot = m_widget.sceneGraphRoot();
	buildSnapshotRecursive(root, sceneRoot, maxDepth);
	return root;
}

bool OsgRenderViewAdapter::selectedPosition(float& outX, float& outY, float& outZ) const
{
	const osg::Vec3f p = m_widget.selectedPosition();
	outX = p.x();
	outY = p.y();
	outZ = p.z();
	return true;
}

bool OsgRenderViewAdapter::selectedRotationEulerDeg(float& outRx, float& outRy, float& outRz) const
{
	const osg::Vec3f r = m_widget.selectedRotationEulerDeg();
	outRx = r.x();
	outRy = r.y();
	outRz = r.z();
	return true;
}

void OsgRenderViewAdapter::ensureSelectionVisualForBackend(const core::ObjectId& id, const bool urdfLinkMesh)
{
	if (m_host)
	{
		m_host->ensureSelectionVisualForBackend(id.toStdString(), urdfLinkMesh);
	}
}

bool OsgRenderViewAdapter::syncOuterPatFromBackend(const core::ObjectId& id)
{
	if (m_host)
	{
		return m_host->syncOuterPatFromBackendId(id.toStdString());
	}
	return false;
}

core::GeometryKind OsgRenderViewAdapter::geometryKindForBackend(const core::ObjectId& id) const
{
	if (m_host)
	{
		return m_host->data().geometryKind(id);
	}
	return core::GeometryKind::None;
}

bool OsgRenderViewAdapter::commitGizmoPoseToBackend(const core::ObjectId& id)
{
	if (!m_host)
	{
		return false;
	}
	const auto obj = m_host->backend().getData(id.toStdString());
	if (!obj)
	{
		return false;
	}
	return m_widget.writeActiveBackendPoseFromOsg(*obj);
}

void OsgRenderViewAdapter::setViewerBackgroundForDarkUi(const bool dark)
{
	m_widget.setViewerBackgroundForDarkUi(dark);
}

void OsgRenderViewAdapter::setPerFrameHook(std::function<void()> hook)
{
	if (!hook)
	{
		m_widget.setPerFrameHook(nullptr);
		return;
	}
	m_widget.setPerFrameHook([fn = std::move(hook)](OsgWidget*) { fn(); });
}

QString OsgRenderViewAdapter::pointCloudPluginReport() const
{
	return m_widget.pointCloudPluginReport();
}

void OsgRenderViewAdapter::setCameraFollowBackendId(const core::ObjectId& id)
{
	m_widget.setCameraFollowBackendId(id.toStdString());
}

void OsgRenderViewAdapter::clearCameraFollowBackendId()
{
	m_widget.clearCameraFollowBackendId();
}

void OsgRenderViewAdapter::setObjectSelectionMode(const bool enabled)
{
	m_widget.setObjectSelectionMode(enabled);
}

bool OsgRenderViewAdapter::objectSelectionMode() const
{
	return m_widget.objectSelectionMode();
}

void OsgRenderViewAdapter::setPointPickMode(const bool enabled)
{
	m_widget.setPointPickMode(enabled);
}

bool OsgRenderViewAdapter::pointPickMode() const
{
	return m_widget.pointPickMode();
}

void OsgRenderViewAdapter::setMeshLinePickMode(const bool enabled)
{
	m_widget.setMeshLinePickMode(enabled);
}

bool OsgRenderViewAdapter::meshLinePickMode() const
{
	return m_widget.meshLinePickMode();
}

void OsgRenderViewAdapter::setMeshFacePickMode(const bool enabled)
{
	m_widget.setMeshFacePickMode(enabled);
}

bool OsgRenderViewAdapter::meshFacePickMode() const
{
	return m_widget.meshFacePickMode();
}

void OsgRenderViewAdapter::syncSelectionForBackend(const core::ObjectId& id)
{
	m_widget.syncSelectionForBackendId(id.toStdString());
	m_widget.setSelectionActive(true);
}

bool OsgRenderViewAdapter::captureViewportPng(QByteArray& outPng, QString* outError, const int maxWidth,
	const int maxHeight)
{
	return m_widget.captureViewportPng(outPng, outError, maxWidth, maxHeight);
}

namespace {

osg::Matrixd osgMatFromCore(const core::Mat4& columnMajor)
{
	osg::Matrixd m;
	for (int i = 0; i < 16; ++i)
	{
		m.ptr()[i] = columnMajor[static_cast<size_t>(i)];
	}
	return m;
}

core::Mat4 coreMatFromOsg(const osg::Matrixd& m)
{
	core::Mat4 out;
	for (int i = 0; i < 16; ++i)
	{
		out[static_cast<size_t>(i)] = m.ptr()[i];
	}
	return out;
}

osg::Vec3f osgVecFromCore(const core::Vec3& v)
{
	return osg::Vec3f(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

RobotOsgUi::InstructionPoseAxis instructionAxisToRobotOsgUi(const core::InstructionPoseAxisDto& d)
{
	RobotOsgUi::InstructionPoseAxis o;
	o.positionMm = osgVecFromCore(d.positionMm);
	o.eulerDeg = osgVecFromCore(d.eulerDeg);
	o.lineMotion = d.lineMotion;
	o.reachable = d.reachable;
	o.robotBackendId = d.robotBackendId.toStdString();
	o.backendId = d.backendId.toStdString();
	if (o.robotBackendId.empty())
	{
		o.robotBackendId = o.backendId;
	}
	o.mountTcpOnPatRoot = d.mountTcpOnPatRoot;
	o.hasLocalMatrix = d.hasLocalMatrix;
	if (d.hasLocalMatrix)
	{
		for (int i = 0; i < 16; ++i)
		{
			o.localMatrix[i] = d.localMatrix[static_cast<size_t>(i)];
		}
	}
	o.urdfTcpAttachLinkName = d.urdfTcpAttachLinkName.toStdString();
	return o;
}

} // namespace

void OsgRenderViewAdapter::setTransformGizmoFrame(const core::TransformGizmoFrameDto frame)
{
	m_widget.setTransformGizmoFrame(frame == core::TransformGizmoFrameDto::World ? OsgWidget::TransformGizmoFrame::World
																				 : OsgWidget::TransformGizmoFrame::Local);
}

core::TransformGizmoFrameDto OsgRenderViewAdapter::transformGizmoFrame() const
{
	return m_widget.transformGizmoFrame() == OsgWidget::TransformGizmoFrame::Local ? core::TransformGizmoFrameDto::Local
																				   : core::TransformGizmoFrameDto::World;
}

void OsgRenderViewAdapter::endTcpDragTeach()
{
	m_widget.endTcpDragTeach();
}

void OsgRenderViewAdapter::beginTcpDragTeach(const core::ObjectId& mountBackendId,
	const core::Mat4& targetInBaseColumnMajor, const float modelDiagonalMm,
	core::RobotBaseWorldResolver resolveRobotBaseWorld, const core::Mat4* toolLocalOnFlangeColumnMajor)
{
	const engine::RigidTransform target = engine::rigidTransformFromOsg(osgMatFromCore(targetInBaseColumnMajor));
	osg::Matrixd toolLocalOsg;
	const osg::Matrixd* toolPtr = nullptr;
	if (toolLocalOnFlangeColumnMajor)
	{
		toolLocalOsg = osgMatFromCore(*toolLocalOnFlangeColumnMajor);
		toolPtr = &toolLocalOsg;
	}
	std::function<bool(osg::Matrixd& outRobotBaseWorld)> osgResolver;
	if (resolveRobotBaseWorld)
	{
		osgResolver = [resolveRobotBaseWorld](osg::Matrixd& outWorld) -> bool {
			core::Mat4 mat;
			if (!resolveRobotBaseWorld(mat))
			{
				return false;
			}
			outWorld = osgMatFromCore(mat);
			return true;
		};
	}
	m_widget.beginTcpDragTeach(mountBackendId.toStdString(), target, modelDiagonalMm, osgResolver, toolPtr);
}

void OsgRenderViewAdapter::updateTcpDragTeachFromTarget(const core::Mat4& targetInBaseColumnMajor,
	const bool syncTargetInBase)
{
	const engine::RigidTransform target = engine::rigidTransformFromOsg(osgMatFromCore(targetInBaseColumnMajor));
	m_widget.updateTcpDragTeachFromTarget(target, syncTargetInBase);
}

void OsgRenderViewAdapter::updateTcpDragTeachToolLocalOnFlange(const core::Mat4& toolLocalOnFlangeColumnMajor)
{
	m_widget.updateTcpDragTeachToolLocalOnFlange(osgMatFromCore(toolLocalOnFlangeColumnMajor));
}

void OsgRenderViewAdapter::setInstructionPoseAxes(const QVector<core::InstructionPoseAxisDto>& axes)
{
	std::vector<RobotOsgUi::InstructionPoseAxis> converted;
	converted.reserve(static_cast<size_t>(axes.size()));
	for (const core::InstructionPoseAxisDto& d : axes)
	{
		converted.push_back(instructionAxisToRobotOsgUi(d));
	}
	m_widget.setInstructionPoseAxes(converted);
}

void OsgRenderViewAdapter::setRawTrajectoryOverlay(const QVector<core::RawTrajectoryOverlayVertexDto>& points)
{
	std::vector<RobotOsgUi::RawTrajectoryOverlayVertex> converted;
	converted.reserve(static_cast<size_t>(points.size()));
	for (const core::RawTrajectoryOverlayVertexDto& v : points)
	{
		RobotOsgUi::RawTrajectoryOverlayVertex o;
		o.positionMm = osgVecFromCore(v.positionMm);
		o.reachable = v.reachable;
		converted.push_back(o);
	}
	m_widget.setRawTrajectoryOverlay(converted);
}

void OsgRenderViewAdapter::clearRawTrajectoryOverlay()
{
	m_widget.clearRawTrajectoryOverlay();
}

void OsgRenderViewAdapter::setRawTrajectoryOverlayFrames(const QVector<core::RawTrajectoryOverlayFrameDto>& frames)
{
	std::vector<RobotOsgUi::RawTrajectoryOverlayFrame> converted;
	converted.reserve(static_cast<size_t>(frames.size()));
	for (const core::RawTrajectoryOverlayFrameDto& f : frames)
	{
		RobotOsgUi::RawTrajectoryOverlayFrame o;
		o.positionMm = osgVecFromCore(f.positionMm);
		o.eulerDeg = osgVecFromCore(f.eulerDeg);
		o.reachable = f.reachable;
		converted.push_back(o);
	}
	m_widget.setRawTrajectoryOverlayFrames(converted);
}

void OsgRenderViewAdapter::clearRawTrajectoryOverlayFrames()
{
	m_widget.clearRawTrajectoryOverlayFrames();
}

void OsgRenderViewAdapter::setRobotFrameOverlays(const core::RobotFrameOverlayUpdateDto& update)
{
	RobotOsgUi::RobotFrameOverlayUpdate u;
	u.robotRootBackendId = update.robotRootBackendId.toStdString();
	u.showToolFrames = update.showToolFrames;
	u.showUserFrames = update.showUserFrames;
	for (const core::RobotFrameOverlayUpdateDto::ToolEntryDto& te : update.toolFrames)
	{
		RobotOsgUi::RobotFrameOverlayUpdate::ToolEntry e;
		e.name = te.name.toStdString();
		e.mountBackendId = te.mountBackendId.toStdString();
		e.localMatrix = osgMatFromCore(te.localMatrix);
		e.active = te.active;
		u.toolFrames.push_back(e);
	}
	for (const core::RobotFrameOverlayUpdateDto::UserEntryDto& ue : update.userFrames)
	{
		RobotOsgUi::RobotFrameOverlayUpdate::UserEntry e;
		e.name = ue.name.toStdString();
		e.mountBackendId = ue.mountBackendId.toStdString();
		e.localMatrix = osgMatFromCore(ue.localMatrix);
		u.userFrames.push_back(e);
	}
	m_widget.setRobotFrameOverlays(u);
}

void OsgRenderViewAdapter::clearRobotFrameOverlays(const core::ObjectId& robotRootBackendId)
{
	m_widget.clearRobotFrameOverlays(robotRootBackendId.toStdString());
}

void OsgRenderViewAdapter::setFeatureCatalogOverlay(const QVector<core::FeatureCatalogOverlayItemDto>& items)
{
	std::vector<RobotOsgUi::FeatureCatalogOverlayItem> converted;
	converted.reserve(static_cast<size_t>(items.size()));
	for (const core::FeatureCatalogOverlayItemDto& item : items)
	{
		RobotOsgUi::FeatureCatalogOverlayItem o;
		o.displayIndex = item.displayIndex;
		o.anchorWorldMm = osgVecFromCore(item.anchorWorldMm);
		o.labelWorldMm = osgVecFromCore(item.labelWorldMm);
		o.hasEdgeSegment = item.hasEdgeSegment;
		o.edgeAWorldMm = osgVecFromCore(item.edgeAWorldMm);
		o.edgeBWorldMm = osgVecFromCore(item.edgeBWorldMm);
		converted.push_back(o);
	}
	m_widget.setFeatureCatalogOverlay(converted);
}

void OsgRenderViewAdapter::clearFeatureCatalogOverlay()
{
	m_widget.clearFeatureCatalogOverlay();
}

} // namespace cloudsim::host
