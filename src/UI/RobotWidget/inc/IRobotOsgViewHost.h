#ifndef ROBOTWIDGET_IROBOTOSGVIEWHOST_H
#define ROBOTWIDGET_IROBOTOSGVIEWHOST_H

/// @file IRobotOsgViewHost.h
/// @brief 机器人 UI 的 OSG/三维操作（OsgWidget 实现）

#include "robotwidget_global.h"

#include "CoreTypes.h"
#include "RobotOsgUiTypes.h"

#include <functional>
#include <string>
#include <vector>

struct PickResult;
enum class PickKind;

enum class MeshTrianglePickTool
{
	None = 0,
	Click,
	Brush,
	Polyline
};

class IRobotBackendPoseSink;
namespace engine
{
class RigidTransform;
}

/// 机器人 UI 的三维操作（经 IRenderView / OsgWidget 实现）
class ROBOTWIDGET_EXPORT IRobotOsgViewHost
{
public:
	virtual ~IRobotOsgViewHost() = default;

	virtual IRobotBackendPoseSink* poseSink() = 0;
	virtual void requestRedraw() = 0;

	virtual bool objectSelectionMode() const = 0;
	virtual void setObjectSelectionMode(bool enabled) = 0;
	virtual void clearBackendObjectSelection() = 0;
	virtual void setSelectionActive(bool active) = 0;
	virtual void setTransformGizmoFrame(int worldOrLocal) = 0;
	virtual bool transformGizmoFrameIsLocal() const = 0;

	virtual void setPointPickMode(bool enabled) = 0;
	virtual bool pointPickMode() const = 0;

	virtual bool hasBackendObjectBranch(const std::string& backendId) const = 0;
	virtual bool getBackendRootWorldMatrix(const std::string& backendId, cloudsim::core::Mat4& outWorld) const = 0;
	virtual bool tryGetBackendModelCenterMm(const std::string& backendId, double& cx, double& cy, double& cz) const = 0;
	/// 装配子零件无 Geode 时映射到共享 visual；与 OsgScene::resolvePickScopeBackendId 一致
	virtual std::string resolvePickScopeBackendId(const std::string& backendId) const = 0;
	/// skipInnerModelCenterRebase 时不应对 STEP 文件坐标加减 modelCenter
	virtual bool backendSkipsInnerModelCenterRebase(const std::string& backendId) const = 0;

	virtual void setInstructionPoseAxes(const std::vector<RobotOsgUi::InstructionPoseAxis>& axes) = 0;
	virtual void clearInstructionPoseAxes() = 0;
	virtual void setRawTrajectoryOverlay(const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& points,
										 const std::vector<std::size_t>& segmentEndExclusive = {}) = 0;
	virtual void clearRawTrajectoryOverlay() = 0;
	virtual void setRawTrajectoryOverlayFrames(const std::vector<RobotOsgUi::RawTrajectoryOverlayFrame>& frames) = 0;
	virtual void setRawTrajectoryOverlayAxisComponents(bool showX, bool showY, bool showZ) = 0;
	virtual void clearRawTrajectoryOverlayFrames() = 0;
	virtual void setCameraFollowBackendId(const std::string& backendId) = 0;

	virtual void setRobotFrameOverlays(const RobotOsgUi::RobotFrameOverlayUpdate& update) = 0;
	virtual void clearRobotFrameOverlays(const std::string& robotRootBackendId) = 0;

	virtual bool isTcpDragTeachActive() const = 0;
	virtual void endTcpDragTeach() = 0;
	virtual void beginTcpDragTeach(const std::string& mountBackendId, const engine::RigidTransform& T_base_target,
								   float modelDiagonalMm,
								   std::function<bool(cloudsim::core::Mat4& outRobotBaseWorld)> resolveRobotBaseWorld,
								   const cloudsim::core::Mat4* toolLocalOnFlange) = 0;
	virtual void updateTcpDragTeachFromTarget(const engine::RigidTransform& T_base_target,
											  bool syncTargetInBase = true) = 0;
	virtual void updateTcpDragTeachToolLocalOnFlange(const cloudsim::core::Mat4& toolLocalOnFlange) = 0;
	virtual engine::RigidTransform tcpDragTeachTargetInBase() const = 0;

	virtual void setMeshLinePickMode(bool enabled) = 0;
	virtual void setMeshFacePickMode(bool enabled) = 0;
	virtual bool meshLinePickMode() const = 0;
	virtual bool meshFacePickMode() const = 0;
	virtual void setMeshPickScopeBackendId(const std::string& backendId) = 0;

	/// mesh 轨迹/区域三角面拾取（委托 OsgWidget 标注拾取模式）
	virtual void setMeshTrianglePickTool(MeshTrianglePickTool tool, float brushRadiusPx = 12.f) = 0;
	virtual void cancelMeshTrianglePick() = 0;
	virtual MeshTrianglePickTool meshTrianglePickTool() const = 0;

	virtual void setPolylinePickMode(bool enabled) = 0;
	virtual bool polylinePickMode() const = 0;

	virtual void showMeshTriangleHighlight(const std::vector<cloudsim::core::Vec3>& triangleVertsWorld) = 0;
	virtual void clearMeshTriangleHighlight() = 0;

	virtual void showMeshFittedSurfacePreview(const std::vector<cloudsim::core::Vec3>& triangleVertsWorld) = 0;
	virtual void clearMeshFittedSurfacePreview() = 0;

	virtual void showMeshSectionPlane(const std::string& backendIdUtf8, const double originModelMm[3],
									  const double normalModel[3]) = 0;
	virtual void
	beginMeshSectionPlaneEdit(const std::string& backendIdUtf8, const double originModelMm[3],
							  const double normalModel[3],
							  std::function<void(const double origin[3], const double normal[3])> onChanged) = 0;
	virtual void updateMeshSectionPlanePose(const double originModelMm[3], const double normalModel[3]) = 0;
	virtual void endMeshSectionPlaneEdit() = 0;
	virtual void hideMeshSectionPlane() = 0;
	virtual void setMeshSectionPlanePreviewVisible(bool visible) = 0;
	virtual bool getCameraViewDirectionInBackendModel(const std::string& backendIdUtf8,
													  double outDirModel[3]) const = 0;

	virtual void setFeatureCatalogOverlay(const std::vector<RobotOsgUi::FeatureCatalogOverlayItem>& items) = 0;
	virtual void clearFeatureCatalogOverlay() = 0;

	virtual void setReachableWorkspaceOverlay(const RobotOsgUi::ReachableWorkspaceOverlay& overlay) = 0;
	virtual void clearReachableWorkspaceOverlay() = 0;
};

#endif // ROBOTWIDGET_IROBOTOSGVIEWHOST_H
