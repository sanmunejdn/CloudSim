/// @file OsgRenderViewAdapter.cpp
/// @brief OSG 到 IRenderView

#include "adapters/OsgRenderViewAdapter.h"

#include "../../UI/OsgWidgetCore/inc/RobotOsgUiTypes.h"
#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFollowMath.h"
#include "BackendFollowSolve.h"
#include "BackendIdUserData.h"
#include "BackendTypeIds.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "IDataService.h"
#include "ObjectGizmoFrame.h"
#include "OsgWidget.h"
#include "io/CustomDeviceHostOps.h"

#include <Adapters.h>
#include <RigidTransform.h>
#include <QHash>
#include <osg/AutoTransform>
#include <osg/BlendFunc>
#include <osg/Camera>
#include <osg/Drawable>
#include <osg/GL>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Matrixd>
#include <osg/Node>
#include <osg/PolygonMode>
#include <osg/PositionAttitudeTransform>
#include <osg/StateAttribute>
#include <osg/StateSet>
#include <osg/Texture>

namespace cloudsim::host
{
OsgRenderViewAdapter::OsgRenderViewAdapter(OsgWidget& widget) : m_widget(widget) {}

OsgRenderViewAdapter::OsgRenderViewAdapter(OsgWidget& widget, DocumentHost& host) : m_widget(widget), m_host(&host) {}

OsgRenderViewAdapter::OsgRenderViewAdapter(DocumentHost& host) : OsgRenderViewAdapter(*host.osgWidget(), host) {}

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
	// BackendMat4 / core::Mat4：Adapters 列主序 index=c*4+r；禁止 OSG ptr() 行主序直拷
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			m(r, c) = columnMajor[static_cast<size_t>(c * 4 + r)];
		}
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
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			outColumnMajor[static_cast<size_t>(c * 4 + r)] = m(r, c);
		}
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

namespace
{
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

bool localTransformMatrix(const osg::Node* node, osg::Matrixd& outLocal)
{
	if (!node)
	{
		return false;
	}
	if (const auto* mt = dynamic_cast<const osg::MatrixTransform*>(node))
	{
		outLocal = mt->getMatrix();
		return true;
	}
	if (const auto* pat = dynamic_cast<const osg::PositionAttitudeTransform*>(node))
	{
		outLocal = osg::Matrixd::translate(pat->getPosition()) * osg::Matrixd::rotate(pat->getAttitude()) *
				   osg::Matrixd::scale(pat->getScale());
		return true;
	}
	if (const auto* at = dynamic_cast<const osg::AutoTransform*>(node))
	{
		outLocal = osg::Matrixd::translate(at->getPosition()) * osg::Matrixd::rotate(at->getRotation()) *
				   osg::Matrixd::scale(at->getScale());
		return true;
	}
	return false;
}

QString localMatrixSummary(const osg::Node* node)
{
	if (!node)
	{
		return QStringLiteral("—");
	}
	if (const auto* cam = dynamic_cast<const osg::Camera*>(node))
	{
		return QStringLiteral("View:\n%1\nProj:\n%2")
			.arg(formatMatrix(cam->getViewMatrix()))
			.arg(formatMatrix(cam->getProjectionMatrix()));
	}
	osg::Matrixd local;
	if (localTransformMatrix(node, local))
	{
		return formatMatrix(local);
	}
	return QStringLiteral("—");
}

void countGeometryStats(const osg::Node* node, int& outDrawables, int& outTriangles)
{
	outDrawables = 0;
	outTriangles = 0;
	const auto* geode = node ? node->asGeode() : nullptr;
	if (!geode)
	{
		return;
	}
	outDrawables = static_cast<int>(geode->getNumDrawables());
	for (unsigned i = 0; i < geode->getNumDrawables(); ++i)
	{
		const osg::Drawable* d = geode->getDrawable(i);
		const auto* geom = d ? d->asGeometry() : nullptr;
		if (!geom)
		{
			continue;
		}
		const osg::Geometry::PrimitiveSetList& sets = geom->getPrimitiveSetList();
		for (const auto& ps : sets)
		{
			if (!ps)
			{
				continue;
			}
			const GLenum mode = ps->getMode();
			const unsigned n = ps->getNumIndices();
			if (mode == GL_TRIANGLES)
			{
				outTriangles += static_cast<int>(n / 3);
			}
			else if (mode == GL_TRIANGLE_STRIP || mode == GL_TRIANGLE_FAN)
			{
				if (n >= 3)
				{
					outTriangles += static_cast<int>(n - 2);
				}
			}
		}
	}
}

QString buildRenderSummary(const osg::Node* node)
{
	if (!node)
	{
		return QString();
	}
	QStringList parts;
	parts << QStringLiteral("NodeMask=0x%1").arg(node->getNodeMask(), 0, 16);
	const osg::StateSet* ss = node->getStateSet();
	if (ss)
	{
		const osg::StateAttribute::GLModeValue lighting = ss->getMode(GL_LIGHTING);
		if (lighting & osg::StateAttribute::ON)
		{
			parts << QStringLiteral("Lighting=ON");
		}
		else if (lighting & osg::StateAttribute::OFF)
		{
			parts << QStringLiteral("Lighting=OFF");
		}
		if (const auto* pm = dynamic_cast<const osg::PolygonMode*>(ss->getAttribute(osg::StateAttribute::POLYGONMODE)))
		{
			const osg::PolygonMode::Mode front = pm->getMode(osg::PolygonMode::FRONT);
			parts << QStringLiteral("PolygonMode=%1")
						 .arg(front == osg::PolygonMode::LINE
								  ? QStringLiteral("LINE")
								  : (front == osg::PolygonMode::POINT ? QStringLiteral("POINT")
																	  : QStringLiteral("FILL")));
		}
		if (ss->getAttribute(osg::StateAttribute::BLENDFUNC) || (ss->getMode(GL_BLEND) & osg::StateAttribute::ON))
		{
			parts << QStringLiteral("Blend=ON");
		}
		int texCount = 0;
		for (unsigned u = 0; u < 8; ++u)
		{
			if (ss->getTextureAttribute(u, osg::StateAttribute::TEXTURE))
			{
				++texCount;
			}
		}
		if (texCount > 0)
		{
			parts << QStringLiteral("Textures=%1").arg(texCount);
		}
	}
	return parts.join(QStringLiteral("; "));
}

void buildSnapshotRecursive(core::IRenderView::SceneNodeInfo& info, const osg::Node* node, int depthLeft,
							const osg::Matrixd& parentWorld,
							const QHash<QString, core::BackendObjectDto>& backendById)
{
	if (!node || depthLeft <= 0)
	{
		return;
	}
	info.className = QString::fromLatin1(node->className());
	info.name = QString::fromStdString(node->getName());
	info.nodeMask = node->getNodeMask();
	info.visible = (info.nodeMask & 0x1u) != 0;
	info.localMatrixSummary = localMatrixSummary(node);
	info.renderSummary = buildRenderSummary(node);

	osg::Matrixd local = osg::Matrixd::identity();
	const bool hasLocalXform = localTransformMatrix(node, local);
	// OSG：子世界 = 父世界 × 本地
	const osg::Matrixd world = hasLocalXform ? (parentWorld * local) : parentWorld;
	info.worldMatrixSummary = formatMatrix(world);

	const osg::BoundingSphere bs = node->getBound();
	if (bs.valid())
	{
		info.boundSummary = QStringLiteral("c=(%1,%2,%3) r=%4")
								.arg(bs.center().x(), 0, 'g', 6)
								.arg(bs.center().y(), 0, 'g', 6)
								.arg(bs.center().z(), 0, 'g', 6)
								.arg(bs.radius(), 0, 'g', 6);
	}

	countGeometryStats(node, info.drawableCount, info.triangleCount);

	if (const auto* meta = dynamic_cast<const BackendIdUserData*>(node->getUserData()))
	{
		info.backendId = QString::fromStdString(meta->backendId());
		info.hasBackend = !info.backendId.isEmpty();
		const auto it = backendById.constFind(info.backendId);
		if (it != backendById.constEnd())
		{
			info.backendClassName = it->className;
			info.visible = it->visible;
			if (!it->name.isEmpty())
			{
				info.displayName = it->name;
			}
		}
	}

	if (info.displayName.isEmpty())
	{
		info.displayName = info.name;
	}
	if (info.displayName.isEmpty())
	{
		info.displayName = info.hasBackend ? info.backendId : info.className;
	}

	if (const auto* g = node->asGroup())
	{
		info.childCount = static_cast<int>(g->getNumChildren());
		for (unsigned i = 0; i < g->getNumChildren(); ++i)
		{
			core::IRenderView::SceneNodeInfo child;
			buildSnapshotRecursive(child, g->getChild(i), depthLeft - 1, world, backendById);
			info.children.push_back(std::move(child));
		}
	}
}

} // namespace

core::IRenderView::SceneNodeInfo OsgRenderViewAdapter::sceneGraphSnapshot(int maxDepth) const
{
	SceneNodeInfo root;
	const osg::Node* sceneRoot = m_widget.sceneGraphRoot();
	QHash<QString, core::BackendObjectDto> backendById;
	if (m_host)
	{
		const QVector<core::BackendObjectDto> snaps = m_host->data().listObjectSnapshots();
		backendById.reserve(snaps.size());
		for (const core::BackendObjectDto& dto : snaps)
		{
			backendById.insert(dto.id, dto);
		}
	}
	buildSnapshotRecursive(root, sceneRoot, maxDepth, osg::Matrixd::identity(), backendById);
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
	const auto obj = m_host->findObject(id.toStdString());
	if (!obj)
	{
		return false;
	}
	const BackendMat4 worldBefore = obj->worldMatrix();
	if (!m_widget.writeActiveBackendPoseFromOsg(*obj))
	{
		return false;
	}
	const BackendMat4 worldAfter = obj->worldMatrix();
	if (obj->className() == backend_type::kClassCustomDevice)
	{
		syncCustomDeviceKinematicsAfterRootPoseChange(*m_host, id.toStdString());
	}
	else if (!backend_mat4_nearly_equal(worldBefore, worldAfter, 1e-9))
	{
		(void)propagateCompoundAfterRootWorldChange(*m_host, id.toStdString(), worldBefore, worldAfter);
	}
	return true;
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

namespace
{
osg::Matrixd osgMatFromCore(const core::Mat4& columnMajor)
{
	osg::Matrixd m;
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			m(r, c) = columnMajor[static_cast<size_t>(c * 4 + r)];
		}
	}
	return m;
}

core::Mat4 mat4FromOsg(const osg::Matrixd& m)
{
	core::Mat4 out{};
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			out[static_cast<size_t>(c * 4 + r)] = m(r, c);
		}
	}
	return out;
}

RobotOsgUi::InstructionPoseAxis instructionAxisToRobotOsgUi(const core::InstructionPoseAxisDto& d)
{
	RobotOsgUi::InstructionPoseAxis o;
	o.positionMm = d.positionMm;
	o.eulerDeg = d.eulerDeg;
	o.lineMotion = d.lineMotion;
	o.reachable = d.reachable;
	o.instructionId = d.instructionId.toStdString();
	o.isArcVia = d.isArcVia;
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
		o.localMatrix = d.localMatrix;
	}
	o.urdfTcpAttachLinkName = d.urdfTcpAttachLinkName.toStdString();
	return o;
}

} // namespace

void OsgRenderViewAdapter::setTransformGizmoFrame(const core::TransformGizmoFrameDto frame)
{
	m_widget.setTransformGizmoFrame(frame == core::TransformGizmoFrameDto::World
										? OsgWidget::TransformGizmoFrame::World
										: OsgWidget::TransformGizmoFrame::Local);
}

core::TransformGizmoFrameDto OsgRenderViewAdapter::transformGizmoFrame() const
{
	return m_widget.transformGizmoFrame() == OsgWidget::TransformGizmoFrame::Local
			   ? core::TransformGizmoFrameDto::Local
			   : core::TransformGizmoFrameDto::World;
}

void OsgRenderViewAdapter::endTcpDragTeach()
{
	m_widget.endTcpDragTeach();
}

void OsgRenderViewAdapter::beginTcpDragTeach(const core::ObjectId& mountBackendId,
											 const core::Mat4& targetInBaseColumnMajor, const float modelDiagonalMm,
											 core::RobotBaseWorldResolver resolveRobotBaseWorld,
											 const core::Mat4* toolLocalOnFlangeColumnMajor)
{
	const engine::RigidTransform target = engine::rigidTransformFromOsg(osgMatFromCore(targetInBaseColumnMajor));
	osg::Matrixd toolLocalOsg;
	const osg::Matrixd* toolPtr = nullptr;
	if (toolLocalOnFlangeColumnMajor)
	{
		toolLocalOsg = osgMatFromCore(*toolLocalOnFlangeColumnMajor);
		toolPtr = &toolLocalOsg;
	}
	std::function<bool(osg::Matrixd & outRobotBaseWorld)> osgResolver;
	if (resolveRobotBaseWorld)
	{
		osgResolver = [resolveRobotBaseWorld](osg::Matrixd& outWorld) -> bool
		{
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

core::Mat4 OsgRenderViewAdapter::tcpDragTeachTargetInBase() const
{
	return mat4FromOsg(engine::osgMatrixFromRigidTransform(m_widget.tcpDragTeachTargetInBase()));
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
		o.positionMm = v.positionMm;
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
		o.positionMm = f.positionMm;
		o.eulerDeg = f.eulerDeg;
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
		e.localMatrix = te.localMatrix;
		e.active = te.active;
		u.toolFrames.push_back(e);
	}
	for (const core::RobotFrameOverlayUpdateDto::UserEntryDto& ue : update.userFrames)
	{
		RobotOsgUi::RobotFrameOverlayUpdate::UserEntry e;
		e.name = ue.name.toStdString();
		e.mountBackendId = ue.mountBackendId.toStdString();
		e.localMatrix = ue.localMatrix;
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
		o.anchorWorldMm = item.anchorWorldMm;
		o.labelWorldMm = item.labelWorldMm;
		o.hasEdgeSegment = item.hasEdgeSegment;
		o.edgeAWorldMm = item.edgeAWorldMm;
		o.edgeBWorldMm = item.edgeBWorldMm;
		o.edgePolylineWorldMm.reserve(static_cast<size_t>(item.edgePolylineWorldMm.size()));
		for (const core::Vec3& p : item.edgePolylineWorldMm)
			o.edgePolylineWorldMm.push_back(p);
		o.faceTrianglesWorldMm.reserve(static_cast<size_t>(item.faceTrianglesWorldMm.size()));
		for (const core::Vec3& p : item.faceTrianglesWorldMm)
			o.faceTrianglesWorldMm.push_back(p);
		converted.push_back(std::move(o));
	}
	m_widget.setFeatureCatalogOverlay(converted);
}

void OsgRenderViewAdapter::clearFeatureCatalogOverlay()
{
	m_widget.clearFeatureCatalogOverlay();
}

std::string OsgRenderViewAdapter::resolvePickScopeBackendId(const std::string& backendId) const
{
	return m_widget.resolvePickScopeBackendId(backendId);
}

bool OsgRenderViewAdapter::backendSkipsInnerModelCenterRebase(const std::string& backendId) const
{
	return m_widget.backendSkipsInnerModelCenterRebase(backendId);
}

std::string OsgRenderViewAdapter::activeBackendId() const
{
	return m_widget.activeBackendId();
}

void OsgRenderViewAdapter::setRobotObjectGizmoSyncHook(std::function<bool()> hook)
{
	m_widget.setRobotObjectGizmoSyncHook([hook](const ObjectGizmoFrame&, bool) -> bool
										 { return hook ? hook() : false; });
}

void OsgRenderViewAdapter::setRobotObjectGizmoFkRefreshHook(std::function<void()> hook)
{
	m_widget.setRobotObjectGizmoFkRefreshHook(
		[hook](const ObjectGizmoFrame&, bool)
		{
			if (hook)
				hook();
		});
}

} // namespace cloudsim::host
