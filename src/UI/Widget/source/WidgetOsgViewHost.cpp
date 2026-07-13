#include "WidgetOsgViewHost.h"

#include "BackendSceneDocumentFacade.h"
#include "DocumentPage.h"
#include "IRenderView.h"
#include "IRobotBackendPoseSink.h"
#include "OsgWidget.h"

#include <Adapters.h>
#include <RobotOsgUiTypes.h>

#include <QString>

namespace {

cloudsim::core::Mat4 mat4FromOsg(const osg::Matrixd& m)
{
	cloudsim::core::Mat4 out;
	for (int i = 0; i < 16; ++i)
	{
		out[static_cast<size_t>(i)] = m.ptr()[i];
	}
	return out;
}

cloudsim::core::Mat4 mat4FromRigid(const engine::RigidTransform& t)
{
	return mat4FromOsg(engine::osgMatrixFromRigidTransform(t));
}

cloudsim::core::Vec3 vec3FromOsg(const osg::Vec3f& v)
{
	return {static_cast<double>(v.x()), static_cast<double>(v.y()), static_cast<double>(v.z())};
}

cloudsim::core::InstructionPoseAxisDto instructionAxisToDto(const RobotOsgUi::InstructionPoseAxis& a)
{
	cloudsim::core::InstructionPoseAxisDto d;
	d.positionMm = vec3FromOsg(a.positionMm);
	d.eulerDeg = vec3FromOsg(a.eulerDeg);
	d.lineMotion = a.lineMotion;
	d.reachable = a.reachable;
	d.robotBackendId = QString::fromStdString(a.robotBackendId);
	d.backendId = QString::fromStdString(a.backendId);
	d.mountTcpOnPatRoot = a.mountTcpOnPatRoot;
	d.hasLocalMatrix = a.hasLocalMatrix;
	if (a.hasLocalMatrix)
	{
		for (int i = 0; i < 16; ++i)
		{
			d.localMatrix[static_cast<size_t>(i)] = a.localMatrix[i];
		}
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

bool WidgetOsgViewHost::getBackendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const
{
	cloudsim::core::IRenderView* rv = renderView();
	if (!rv)
	{
		return false;
	}
	cloudsim::core::Mat4 mat;
	if (!rv->getWorldMatrix(QString::fromStdString(backendId), mat))
	{
		return false;
	}
	for (int i = 0; i < 16; ++i)
	{
		outWorld.ptr()[i] = mat[static_cast<size_t>(i)];
	}
	return true;
}

bool WidgetOsgViewHost::tryGetBackendModelCenterMm(const std::string& backendId, double& cx, double& cy, double& cz) const
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

void WidgetOsgViewHost::setRawTrajectoryOverlay(
	const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& points,
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
		d.positionMm = vec3FromOsg(v.positionMm);
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
		d.positionMm = vec3FromOsg(f.positionMm);
		d.eulerDeg = vec3FromOsg(f.eulerDeg);
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
		e.localMatrix = mat4FromOsg(te.localMatrix);
		e.active = te.active;
		dto.toolFrames.push_back(e);
	}
	for (const RobotOsgUi::RobotFrameOverlayUpdate::UserEntry& ue : update.userFrames)
	{
		cloudsim::core::RobotFrameOverlayUpdateDto::UserEntryDto e;
		e.name = QString::fromStdString(ue.name);
		e.mountBackendId = QString::fromStdString(ue.mountBackendId);
		e.localMatrix = mat4FromOsg(ue.localMatrix);
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
		d.anchorWorldMm = vec3FromOsg(item.anchorWorldMm);
		d.labelWorldMm = vec3FromOsg(item.labelWorldMm);
		d.hasEdgeSegment = item.hasEdgeSegment;
		d.edgeAWorldMm = vec3FromOsg(item.edgeAWorldMm);
		d.edgeBWorldMm = vec3FromOsg(item.edgeBWorldMm);
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

void WidgetOsgViewHost::beginTcpDragTeach(
	const std::string& mountBackendId,
	const engine::RigidTransform& T_base_target,
	const float modelDiagonalMm,
	std::function<bool(osg::Matrixd& outRobotBaseWorld)> resolveRobotBaseWorld,
	const osg::Matrixd* toolLocalOnFlange)
{
	cloudsim::core::IRenderView* rv = renderView();
	if (!rv)
	{
		return;
	}
	cloudsim::core::RobotBaseWorldResolver coreResolver;
	if (resolveRobotBaseWorld)
	{
		coreResolver = [resolveRobotBaseWorld](cloudsim::core::Mat4& outMat) -> bool {
			osg::Matrixd world;
			if (!resolveRobotBaseWorld(world))
			{
				return false;
			}
			outMat = mat4FromOsg(world);
			return true;
		};
	}
	cloudsim::core::Mat4 toolLocalMat;
	const cloudsim::core::Mat4* toolPtr = nullptr;
	if (toolLocalOnFlange)
	{
		toolLocalMat = mat4FromOsg(*toolLocalOnFlange);
		toolPtr = &toolLocalMat;
	}
	rv->beginTcpDragTeach(QString::fromStdString(mountBackendId), mat4FromRigid(T_base_target), modelDiagonalMm,
		coreResolver, toolPtr);
}

void WidgetOsgViewHost::updateTcpDragTeachFromTarget(
	const engine::RigidTransform& T_base_target,
	const bool syncTargetInBase)
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->updateTcpDragTeachFromTarget(mat4FromRigid(T_base_target), syncTargetInBase);
	}
}

void WidgetOsgViewHost::updateTcpDragTeachToolLocalOnFlange(const osg::Matrixd& toolLocalOnFlange)
{
	if (cloudsim::core::IRenderView* rv = renderView())
	{
		rv->updateTcpDragTeachToolLocalOnFlange(mat4FromOsg(toolLocalOnFlange));
	}
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

void WidgetOsgViewHost::showMeshTriangleHighlight(const std::vector<osg::Vec3f>& triangleVertsWorld)
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->showMeshFaceHighlight(triangleVertsWorld);
	}
}

void WidgetOsgViewHost::clearMeshTriangleHighlight()
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->hideMeshElementHighlight();
	}
}

void WidgetOsgViewHost::showMeshFittedSurfacePreview(const std::vector<osg::Vec3f>& triangleVertsWorld)
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->showMeshFittedSurfacePreview(triangleVertsWorld);
	}
}

void WidgetOsgViewHost::clearMeshFittedSurfacePreview()
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->clearMeshFittedSurfacePreview();
	}
}

void WidgetOsgViewHost::showMeshSectionPlane(
	const std::string& backendIdUtf8,
	const double originModelMm[3],
	const double normalModel[3])
{
	if (OsgWidget* osg = osgWidget())
	{
		osg->showMeshSectionPlane(backendIdUtf8, originModelMm, normalModel);
	}
}

void WidgetOsgViewHost::beginMeshSectionPlaneEdit(
	const std::string& backendIdUtf8,
	const double originModelMm[3],
	const double normalModel[3],
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

bool WidgetOsgViewHost::getCameraViewDirectionInBackendModel(
	const std::string& backendIdUtf8,
	double outDirModel[3]) const
{
	const OsgWidget* osg = osgWidget();
	return osg && osg->getCameraViewDirectionInBackendModel(backendIdUtf8, outDirModel);
}
