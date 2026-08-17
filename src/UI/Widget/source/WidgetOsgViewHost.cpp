/// @file WidgetOsgViewHost.cpp
/// @brief WidgetOsgView 宿主

#include "WidgetOsgViewHost.h"

#include "BackendSceneDocumentFacade.h"
#include "DocumentPage.h"
#include "IRenderView.h"
#include "IRobotBackendPoseSink.h"
#include "OsgWidget.h"

#include <QString>

#include <Adapters.h>
#include <RobotOsgUiTypes.h>

namespace
{
// core::Mat4 列主序 index=c*4+r；禁止与 OSG ptr() 行主序直拷（平移会丢）
cloudsim::core::Mat4 mat4FromOsg(const osg::Matrixd& m)
{
	cloudsim::core::Mat4 out;
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			out[static_cast<size_t>(c * 4 + r)] = m(r, c);
		}
	}
	return out;
}

osg::Matrixd osgMatFromCore(const cloudsim::core::Mat4& columnMajor)
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

cloudsim::core::Mat4 mat4FromRigid(const engine::RigidTransform& t)
{
	return mat4FromOsg(engine::osgMatrixFromRigidTransform(t));
}

cloudsim::core::InstructionPoseAxisDto instructionAxisToDto(const RobotOsgUi::InstructionPoseAxis& a)
{
	cloudsim::core::InstructionPoseAxisDto d;
	d.positionMm = a.positionMm;
	d.eulerDeg = a.eulerDeg;
	d.lineMotion = a.lineMotion;
	d.reachable = a.reachable;
	d.robotBackendId = QString::fromStdString(a.robotBackendId);
	d.backendId = QString::fromStdString(a.backendId);
	d.mountTcpOnPatRoot = a.mountTcpOnPatRoot;
	d.hasLocalMatrix = a.hasLocalMatrix;
	if (a.hasLocalMatrix)
	{
		d.localMatrix = a.localMatrix;
	}
	d.urdfTcpAttachLinkName = QString::fromStdString(a.urdfTcpAttachLinkName);
	return d;
}

} // namespace

WidgetOsgViewHost::WidgetOsgViewHost(DocumentPage* page) : m_page(page) {}

cloudsim::core::IRenderView* WidgetOsgViewHost::renderView() const
{
	return m_page ? &m_page->render() : nullptr;
}

OsgWidget* WidgetOsgViewHost::osgWidget() const
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		return qobject_cast<OsgWidget*>(rv->widget());
	}
	return nullptr;
}

IRobotBackendPoseSink* WidgetOsgViewHost::poseSink()
{
	return m_page ? m_page->sceneFacade().poseSink() : nullptr;
}

void WidgetOsgViewHost::requestRedraw()
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->requestRedraw();
	}
}

bool WidgetOsgViewHost::objectSelectionMode() const
{
	cloudsim::core::IRenderView* rv = renderView();
	return rv && rv->objectSelectionMode();
}

void WidgetOsgViewHost::setObjectSelectionMode(const bool enabled)
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->setObjectSelectionMode(enabled);
	}
}

void WidgetOsgViewHost::clearBackendObjectSelection()
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->setSelectionActive(false);
	}
}

void WidgetOsgViewHost::setSelectionActive(const bool active)
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->setSelectionActive(active);
	}
}

void WidgetOsgViewHost::setTransformGizmoFrame(const int worldOrLocal)
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->setTransformGizmoFrame(worldOrLocal == 0 ? cloudsim::core::TransformGizmoFrameDto::World
													 : cloudsim::core::TransformGizmoFrameDto::Local);
	}
}

bool WidgetOsgViewHost::transformGizmoFrameIsLocal() const
{
	cloudsim::core::IRenderView* rv = renderView();
	return rv && rv->transformGizmoFrame() == cloudsim::core::TransformGizmoFrameDto::Local;
}

void WidgetOsgViewHost::setPointPickMode(const bool enabled)
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->setPointPickMode(enabled);
	}
}

bool WidgetOsgViewHost::pointPickMode() const
{
	cloudsim::core::IRenderView* rv = renderView();
	return rv && rv->pointPickMode();
}

bool WidgetOsgViewHost::hasBackendObjectBranch(const std::string& backendId) const
{
	cloudsim::core::IRenderView* rv = renderView();
	return rv && rv->hasVisualBranch(QString::fromStdString(backendId));
}

bool WidgetOsgViewHost::getBackendRootWorldMatrix(const std::string& backendId, cloudsim::core::Mat4& outWorld) const
{
	cloudsim::core::IRenderView* rv = renderView();
	return rv && rv->getWorldMatrix(QString::fromStdString(backendId), outWorld);
}

bool WidgetOsgViewHost::tryGetBackendModelCenterMm(const std::string& backendId, double& cx, double& cy,
												   double& cz) const
{
	cloudsim::core::IRenderView* rv = renderView();
	return rv && rv->tryGetModelCenterMm(QString::fromStdString(backendId), cx, cy, cz);
}

std::string WidgetOsgViewHost::resolvePickScopeBackendId(const std::string& backendId) const
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		return rv->resolvePickScopeBackendId(backendId);
	}
	return backendId;
}

bool WidgetOsgViewHost::backendSkipsInnerModelCenterRebase(const std::string& backendId) const
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		return rv->backendSkipsInnerModelCenterRebase(backendId);
	}
	return false;
}

void WidgetOsgViewHost::setInstructionPoseAxes(const std::vector<RobotOsgUi::InstructionPoseAxis>& axes)
{
	cloudsim::core::IRenderView* rv = renderView();
	if (!rv)
	{
		return;
	}
	QVector<cloudsim::core::InstructionPoseAxisDto> converted;
	converted.reserve(static_cast<int>(axes.size()));
	for (const RobotOsgUi::InstructionPoseAxis& a : axes)
	{
		converted.push_back(instructionAxisToDto(a));
	}
	rv->setInstructionPoseAxes(converted);
}

void WidgetOsgViewHost::clearInstructionPoseAxes()
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->clearInstructionPoseAxes();
	}
}

void WidgetOsgViewHost::setRawTrajectoryOverlay(const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& points,
												const std::vector<std::size_t>& segmentEndExclusive)
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->setRawTrajectoryOverlay(points, segmentEndExclusive);
		return;
	}
	cloudsim::core::IRenderView* rv = renderView();
	if (!rv)
	{
		return;
	}
	QVector<cloudsim::core::RawTrajectoryOverlayVertexDto> converted;
	converted.reserve(static_cast<int>(points.size()));
	for (const RobotOsgUi::RawTrajectoryOverlayVertex& v : points)
	{
		cloudsim::core::RawTrajectoryOverlayVertexDto d;
		d.positionMm = v.positionMm;
		d.reachable = v.reachable;
		converted.push_back(d);
	}
	rv->setRawTrajectoryOverlay(converted);
}

void WidgetOsgViewHost::clearRawTrajectoryOverlay()
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->clearRawTrajectoryOverlay();
	}
}

void WidgetOsgViewHost::setRawTrajectoryOverlayAxisComponents(bool showX, bool showY, bool showZ)
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->setRawTrajectoryOverlayAxisComponents(showX, showY, showZ);
	}
}

void WidgetOsgViewHost::setRawTrajectoryOverlayFrames(const std::vector<RobotOsgUi::RawTrajectoryOverlayFrame>& frames)
{
	cloudsim::core::IRenderView* rv = renderView();
	if (!rv)
	{
		return;
	}
	QVector<cloudsim::core::RawTrajectoryOverlayFrameDto> converted;
	converted.reserve(static_cast<int>(frames.size()));
	for (const RobotOsgUi::RawTrajectoryOverlayFrame& f : frames)
	{
		cloudsim::core::RawTrajectoryOverlayFrameDto d;
		d.positionMm = f.positionMm;
		d.eulerDeg = f.eulerDeg;
		d.reachable = f.reachable;
		converted.push_back(d);
	}
	rv->setRawTrajectoryOverlayFrames(converted);
}

void WidgetOsgViewHost::clearRawTrajectoryOverlayFrames()
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->clearRawTrajectoryOverlayFrames();
	}
}

void WidgetOsgViewHost::setCameraFollowBackendId(const std::string& backendId)
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		if (backendId.empty())
		{
			rv->clearCameraFollowBackendId();
		}
		else
		{
			rv->setCameraFollowBackendId(QString::fromStdString(backendId));
		}
	}
}

void WidgetOsgViewHost::setRobotFrameOverlays(const RobotOsgUi::RobotFrameOverlayUpdate& update)
{
	cloudsim::core::IRenderView* rv = renderView();
	if (!rv)
	{
		return;
	}
	cloudsim::core::RobotFrameOverlayUpdateDto dto;
	dto.robotRootBackendId = QString::fromStdString(update.robotRootBackendId);
	dto.showToolFrames = update.showToolFrames;
	dto.showUserFrames = update.showUserFrames;
	for (const RobotOsgUi::RobotFrameOverlayUpdate::ToolEntry& te : update.toolFrames)
	{
		cloudsim::core::RobotFrameOverlayUpdateDto::ToolEntryDto e;
		e.name = QString::fromStdString(te.name);
		e.mountBackendId = QString::fromStdString(te.mountBackendId);
		e.localMatrix = te.localMatrix;
		e.active = te.active;
		dto.toolFrames.push_back(e);
	}
	for (const RobotOsgUi::RobotFrameOverlayUpdate::UserEntry& ue : update.userFrames)
	{
		cloudsim::core::RobotFrameOverlayUpdateDto::UserEntryDto e;
		e.name = QString::fromStdString(ue.name);
		e.mountBackendId = QString::fromStdString(ue.mountBackendId);
		e.localMatrix = ue.localMatrix;
		dto.userFrames.push_back(e);
	}
	rv->setRobotFrameOverlays(dto);
}

void WidgetOsgViewHost::clearRobotFrameOverlays(const std::string& robotRootBackendId)
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->clearRobotFrameOverlays(QString::fromStdString(robotRootBackendId));
	}
}

void WidgetOsgViewHost::setFeatureCatalogOverlay(const std::vector<RobotOsgUi::FeatureCatalogOverlayItem>& items)
{
	cloudsim::core::IRenderView* rv = renderView();
	if (!rv)
	{
		return;
	}
	QVector<cloudsim::core::FeatureCatalogOverlayItemDto> converted;
	converted.reserve(static_cast<int>(items.size()));
	for (const RobotOsgUi::FeatureCatalogOverlayItem& item : items)
	{
		cloudsim::core::FeatureCatalogOverlayItemDto d;
		d.displayIndex = item.displayIndex;
		d.anchorWorldMm = item.anchorWorldMm;
		d.labelWorldMm = item.labelWorldMm;
		d.hasEdgeSegment = item.hasEdgeSegment;
		d.edgeAWorldMm = item.edgeAWorldMm;
		d.edgeBWorldMm = item.edgeBWorldMm;
		d.edgePolylineWorldMm.reserve(static_cast<int>(item.edgePolylineWorldMm.size()));
		for (const cloudsim::core::Vec3& p : item.edgePolylineWorldMm)
			d.edgePolylineWorldMm.push_back(p);
		d.faceTrianglesWorldMm.reserve(static_cast<int>(item.faceTrianglesWorldMm.size()));
		for (const cloudsim::core::Vec3& p : item.faceTrianglesWorldMm)
			d.faceTrianglesWorldMm.push_back(p);
		converted.push_back(d);
	}
	rv->setFeatureCatalogOverlay(converted);
}

void WidgetOsgViewHost::clearFeatureCatalogOverlay()
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->clearFeatureCatalogOverlay();
	}
}

void WidgetOsgViewHost::setReachableWorkspaceOverlay(const RobotOsgUi::ReachableWorkspaceOverlay& overlay)
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->setReachableWorkspaceOverlay(overlay);
	}
}

void WidgetOsgViewHost::clearReachableWorkspaceOverlay()
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->clearReachableWorkspaceOverlay();
	}
}

bool WidgetOsgViewHost::isTcpDragTeachActive() const
{
	cloudsim::core::IRenderView* rv = renderView();
	return rv && rv->isTcpDragTeachActive();
}

void WidgetOsgViewHost::endTcpDragTeach()
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->endTcpDragTeach();
	}
}

void WidgetOsgViewHost::beginTcpDragTeach(const std::string& mountBackendId,
										  const engine::RigidTransform& T_base_target, const float modelDiagonalMm,
										  std::function<bool(cloudsim::core::Mat4& outRobotBaseWorld)> resolveRobotBaseWorld,
										  const cloudsim::core::Mat4* toolLocalOnFlange)
{
	cloudsim::core::IRenderView* rv = renderView();
	if (!rv)
	{
		return;
	}
	rv->beginTcpDragTeach(QString::fromStdString(mountBackendId), mat4FromRigid(T_base_target), modelDiagonalMm,
						  resolveRobotBaseWorld, toolLocalOnFlange);
}

void WidgetOsgViewHost::updateTcpDragTeachFromTarget(const engine::RigidTransform& T_base_target,
													 const bool syncTargetInBase)
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->updateTcpDragTeachFromTarget(mat4FromRigid(T_base_target), syncTargetInBase);
	}
}

void WidgetOsgViewHost::updateTcpDragTeachToolLocalOnFlange(const cloudsim::core::Mat4& toolLocalOnFlange)
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->updateTcpDragTeachToolLocalOnFlange(toolLocalOnFlange);
	}
}

engine::RigidTransform WidgetOsgViewHost::tcpDragTeachTargetInBase() const
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		return engine::rigidTransformFromOsg(osgMatFromCore(rv->tcpDragTeachTargetInBase()));
	}
	return engine::RigidTransform::identity();
}

void WidgetOsgViewHost::setMeshLinePickMode(const bool enabled)
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->setMeshLinePickMode(enabled);
	}
}

void WidgetOsgViewHost::setMeshFacePickMode(const bool enabled)
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->setMeshFacePickMode(enabled);
	}
}

bool WidgetOsgViewHost::meshLinePickMode() const
{
	cloudsim::core::IRenderView* rv = renderView();
	return rv && rv->meshLinePickMode();
}

bool WidgetOsgViewHost::meshFacePickMode() const
{
	cloudsim::core::IRenderView* rv = renderView();
	return rv && rv->meshFacePickMode();
}

void WidgetOsgViewHost::setMeshPickScopeBackendId(const std::string& backendId)
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->syncSelectionForBackend(QString::fromStdString(backendId));
	}
}

void WidgetOsgViewHost::setMeshTrianglePickTool(const MeshTrianglePickTool tool, const float brushRadiusPx)
{
	m_meshTrianglePickTool = tool;
	OsgWidget* osg = osgWidget();
	if (!osg)
	{
		return;
	}
	switch (tool)
	{
	case MeshTrianglePickTool::Click:
		osg->setLabelingClickPickMode(true, true);
		break;
	case MeshTrianglePickTool::Brush:
		osg->setLabelingBrushPickMode(true, true, brushRadiusPx);
		break;
	case MeshTrianglePickTool::Polyline:
		osg->setPolylinePickMode(true);
		break;
	default:
		cancelMeshTrianglePick();
		break;
	}
}

void WidgetOsgViewHost::cancelMeshTrianglePick()
{
	m_meshTrianglePickTool = MeshTrianglePickTool::None;
	if (OsgWidget* osg = osgWidget())
	{
		osg->setLabelingClickPickMode(false, true);
		osg->setLabelingBrushPickMode(false, true, 12.f);
		osg->setPolylinePickMode(false);
	}
}

MeshTrianglePickTool WidgetOsgViewHost::meshTrianglePickTool() const
{
	return m_meshTrianglePickTool;
}

void WidgetOsgViewHost::setPolylinePickMode(const bool enabled)
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->setPolylinePickMode(enabled);
	}
	if (enabled)
	{
		m_meshTrianglePickTool = MeshTrianglePickTool::Polyline;
	}
	else if (m_meshTrianglePickTool == MeshTrianglePickTool::Polyline)
	{
		m_meshTrianglePickTool = MeshTrianglePickTool::None;
	}
}

bool WidgetOsgViewHost::polylinePickMode() const
{
	const OsgWidget* osg = osgWidget();
	return osg && osg->polylinePickMode();
}

void WidgetOsgViewHost::showMeshTriangleHighlight(const std::vector<cloudsim::core::Vec3>& triangleVertsWorld)
{
	if (OsgWidget* osg = osgWidget())
	{
		std::vector<osg::Vec3f> converted;
		converted.reserve(triangleVertsWorld.size());
		for (const cloudsim::core::Vec3& v : triangleVertsWorld)
		{
			converted.emplace_back(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
		}
		osg->showMeshFaceHighlight(converted);
	}
}

void WidgetOsgViewHost::clearMeshTriangleHighlight()
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->hideMeshElementHighlight();
	}
}

void WidgetOsgViewHost::showMeshFittedSurfacePreview(const std::vector<cloudsim::core::Vec3>& triangleVertsWorld)
{
	if (OsgWidget* osg = osgWidget())
	{
		std::vector<osg::Vec3f> converted;
		converted.reserve(triangleVertsWorld.size());
		for (const cloudsim::core::Vec3& v : triangleVertsWorld)
		{
			converted.emplace_back(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
		}
		osg->showMeshFittedSurfacePreview(converted);
	}
}

void WidgetOsgViewHost::clearMeshFittedSurfacePreview()
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->clearMeshFittedSurfacePreview();
	}
}

void WidgetOsgViewHost::showMeshSectionPlane(const std::string& backendIdUtf8, const double originModelMm[3],
											 const double normalModel[3])
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->showMeshSectionPlane(backendIdUtf8, originModelMm, normalModel);
	}
}

void WidgetOsgViewHost::beginMeshSectionPlaneEdit(
	const std::string& backendIdUtf8, const double originModelMm[3], const double normalModel[3],
	std::function<void(const double origin[3], const double normal[3])> onChanged)
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->beginMeshSectionPlaneEdit(backendIdUtf8, originModelMm, normalModel, std::move(onChanged));
	}
}

void WidgetOsgViewHost::updateMeshSectionPlanePose(const double originModelMm[3], const double normalModel[3])
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->updateMeshSectionPlanePose(originModelMm, normalModel);
	}
}

void WidgetOsgViewHost::endMeshSectionPlaneEdit()
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->endMeshSectionPlaneEdit();
	}
}

void WidgetOsgViewHost::hideMeshSectionPlane()
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->hideMeshSectionPlane();
	}
}

void WidgetOsgViewHost::setMeshSectionPlanePreviewVisible(const bool visible)
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->setMeshSectionPlanePreviewVisible(visible);
	}
}

bool WidgetOsgViewHost::getCameraViewDirectionInBackendModel(const std::string& backendIdUtf8,
															 double outDirModel[3]) const
{
	const OsgWidget* osg = osgWidget();
	return osg && osg->getCameraViewDirectionInBackendModel(backendIdUtf8, outDirModel);
}
