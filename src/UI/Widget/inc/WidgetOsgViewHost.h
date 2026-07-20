#ifndef WIDGET_WIDGETOSGVIEWHOST_H
#define WIDGET_WIDGETOSGVIEWHOST_H

/// @file WidgetOsgViewHost.h
/// @brief IRobotOsgViewHost：渲染与拾取委托 IRenderView；poseSink 经 sceneFacade

#include "../RobotWidget/inc/IRobotOsgViewHost.h"

#include <QPointer>

class DocumentPage;
class OsgWidget;

namespace cloudsim::core
{
class IRenderView;
}

/// IRobotOsgViewHost：渲染与拾取委托 IRenderView；poseSink 经 sceneFacade
class WidgetOsgViewHost : public IRobotOsgViewHost
{
public:
	explicit WidgetOsgViewHost(DocumentPage* page);

	DocumentPage* page() const { return m_page; }

	IRobotBackendPoseSink* poseSink() override;
	void requestRedraw() override;

	bool objectSelectionMode() const override;
	void setObjectSelectionMode(bool enabled) override;
	void clearBackendObjectSelection() override;
	void setSelectionActive(bool active) override;
	void setTransformGizmoFrame(int worldOrLocal) override;
	bool transformGizmoFrameIsLocal() const override;
	void setPointPickMode(bool enabled) override;
	bool pointPickMode() const override;

	bool hasBackendObjectBranch(const std::string& backendId) const override;
	bool getBackendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const override;
	bool tryGetBackendModelCenterMm(const std::string& backendId, double& cx, double& cy, double& cz) const override;
	std::string resolvePickScopeBackendId(const std::string& backendId) const override;
	bool backendSkipsInnerModelCenterRebase(const std::string& backendId) const override;

	void setInstructionPoseAxes(const std::vector<RobotOsgUi::InstructionPoseAxis>& axes) override;
	void clearInstructionPoseAxes() override;
	void setRawTrajectoryOverlay(const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& points,
								 const std::vector<std::size_t>& segmentEndExclusive = {}) override;
	void clearRawTrajectoryOverlay() override;
	void setRawTrajectoryOverlayFrames(const std::vector<RobotOsgUi::RawTrajectoryOverlayFrame>& frames) override;
	void setRawTrajectoryOverlayAxisComponents(bool showX, bool showY, bool showZ) override;
	void clearRawTrajectoryOverlayFrames() override;
	void setCameraFollowBackendId(const std::string& backendId) override;

	void setRobotFrameOverlays(const RobotOsgUi::RobotFrameOverlayUpdate& update) override;
	void clearRobotFrameOverlays(const std::string& robotRootBackendId) override;

	void setFeatureCatalogOverlay(const std::vector<RobotOsgUi::FeatureCatalogOverlayItem>& items) override;
	void clearFeatureCatalogOverlay() override;

	bool isTcpDragTeachActive() const override;
	void endTcpDragTeach() override;
	void beginTcpDragTeach(const std::string& mountBackendId, const engine::RigidTransform& T_base_target,
						   float modelDiagonalMm,
						   std::function<bool(osg::Matrixd& outRobotBaseWorld)> resolveRobotBaseWorld,
						   const osg::Matrixd* toolLocalOnFlange) override;
	void updateTcpDragTeachFromTarget(const engine::RigidTransform& T_base_target,
									  bool syncTargetInBase = true) override;
	void updateTcpDragTeachToolLocalOnFlange(const osg::Matrixd& toolLocalOnFlange) override;

	void setMeshLinePickMode(bool enabled) override;
	void setMeshFacePickMode(bool enabled) override;
	bool meshLinePickMode() const override;
	bool meshFacePickMode() const override;
	void setMeshPickScopeBackendId(const std::string& backendId) override;

	void setMeshTrianglePickTool(MeshTrianglePickTool tool, float brushRadiusPx = 12.f) override;
	void cancelMeshTrianglePick() override;
	MeshTrianglePickTool meshTrianglePickTool() const override;

	void setPolylinePickMode(bool enabled) override;
	bool polylinePickMode() const override;

	void showMeshTriangleHighlight(const std::vector<osg::Vec3f>& triangleVertsWorld) override;
	void clearMeshTriangleHighlight() override;

	void showMeshFittedSurfacePreview(const std::vector<osg::Vec3f>& triangleVertsWorld) override;
	void clearMeshFittedSurfacePreview() override;

	void showMeshSectionPlane(const std::string& backendIdUtf8, const double originModelMm[3],
							  const double normalModel[3]) override;
	void
	beginMeshSectionPlaneEdit(const std::string& backendIdUtf8, const double originModelMm[3],
							  const double normalModel[3],
							  std::function<void(const double origin[3], const double normal[3])> onChanged) override;
	void updateMeshSectionPlanePose(const double originModelMm[3], const double normalModel[3]) override;
	void endMeshSectionPlaneEdit() override;
	void hideMeshSectionPlane() override;
	void setMeshSectionPlanePreviewVisible(bool visible) override;
	bool getCameraViewDirectionInBackendModel(const std::string& backendIdUtf8, double outDirModel[3]) const override;

private:
	cloudsim::core::IRenderView* renderView() const;
	OsgWidget* osgWidget() const;

	MeshTrianglePickTool m_meshTrianglePickTool = MeshTrianglePickTool::None;

	QPointer<DocumentPage> m_page;
};

#endif // WIDGET_WIDGETOSGVIEWHOST_H
