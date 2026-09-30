#ifndef ROBOTWIDGET_IROBOTOSGOVERLAY_H
#define ROBOTWIDGET_IROBOTOSGOVERLAY_H

/// @file IRobotOsgOverlay.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 机器人 UI 三维叠加层（IRobotOsgViewHost 切面）

#include "robotwidget_global.h"

#include "CoreTypes.h"
#include "RobotOsgUiTypes.h"

#include <string>
#include <vector>

/// 轨迹、坐标系与 mesh 高亮叠加
class ROBOTWIDGET_EXPORT IRobotOsgOverlay
{
public:
	virtual ~IRobotOsgOverlay() = default;

	virtual void setInstructionPoseAxes(const std::vector<RobotOsgUi::InstructionPoseAxis>& axes) = 0;
	virtual void clearInstructionPoseAxes() = 0;
	virtual void setRawTrajectoryOverlay(const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& points,
										 const std::vector<std::size_t>& segmentEndExclusive = {}) = 0;
	virtual void clearRawTrajectoryOverlay() = 0;
	virtual void setRawTrajectoryOverlayFrames(const std::vector<RobotOsgUi::RawTrajectoryOverlayFrame>& frames) = 0;
	virtual void setRawTrajectoryOverlayAxisComponents(bool showX, bool showY, bool showZ) = 0;
	virtual void clearRawTrajectoryOverlayFrames() = 0;

	virtual void setRobotFrameOverlays(const RobotOsgUi::RobotFrameOverlayUpdate& update) = 0;
	virtual void clearRobotFrameOverlays(const std::string& robotRootBackendId) = 0;

	virtual void setFeatureCatalogOverlay(const std::vector<RobotOsgUi::FeatureCatalogOverlayItem>& items) = 0;
	virtual void clearFeatureCatalogOverlay() = 0;

	virtual void setReachableWorkspaceOverlay(const RobotOsgUi::ReachableWorkspaceOverlay& overlay) = 0;
	virtual void clearReachableWorkspaceOverlay() = 0;

	virtual void setPlaybackCursorOverlay(const RobotOsgUi::PlaybackCursorOverlay& cursor) = 0;
	virtual void clearPlaybackCursorOverlay() = 0;
	virtual void setWaypointIndexLabels(const std::vector<RobotOsgUi::WaypointIndexLabel>& labels) = 0;
	virtual void clearWaypointIndexLabels() = 0;

	virtual void showMeshTriangleHighlight(const std::vector<cloudsim::core::Vec3>& triangleVertsWorld) = 0;
	virtual void clearMeshTriangleHighlight() = 0;

	virtual void showMeshFittedSurfacePreview(const std::vector<cloudsim::core::Vec3>& triangleVertsWorld) = 0;
	virtual void clearMeshFittedSurfacePreview() = 0;
};

#endif // ROBOTWIDGET_IROBOTOSGOVERLAY_H
