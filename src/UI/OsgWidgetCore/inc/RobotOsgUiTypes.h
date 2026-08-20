#ifndef OSGWIDGETCORE_ROBOTOSGUITYPES_H
#define OSGWIDGETCORE_ROBOTOSGUITYPES_H

/// @file RobotOsgUiTypes.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 机器人视口叠加数据结构（无 osg；渲染边界再转 OSG）

#include "CoreTypes.h"

#include <string>
#include <vector>

namespace RobotOsgUi
{
struct InstructionPoseAxis
{
	cloudsim::core::Vec3 positionMm{};
	cloudsim::core::Vec3 eulerDeg{};
	bool lineMotion = false;
	bool reachable = true;
	/// 对应指令 id；3D 拾取跳树用，空则不可点
	std::string instructionId;
	std::string robotBackendId;
	bool mountTcpOnPatRoot = false;
	bool hasLocalMatrix = false;
	cloudsim::core::Mat4 localMatrix{};
	std::string urdfTcpAttachLinkName;
	cloudsim::core::Mat4 worldMatrix{};
	std::string backendId;
};

struct RawTrajectoryOverlayVertex
{
	cloudsim::core::Vec3 positionMm{};
	bool reachable = true;
};

struct RawTrajectoryOverlayFrame
{
	cloudsim::core::Vec3 positionMm{};
	cloudsim::core::Vec3 eulerDeg{};
	bool reachable = true;
};

struct RawTrajectoryPreviewOptions
{
	bool showAxes = true;
	bool showAxisX = true;
	bool showAxisY = true;
	bool showAxisZ = true;
	int axisInterval = 0;
};

struct FeatureCatalogOverlayItem
{
	int displayIndex = 0;
	cloudsim::core::Vec3 anchorWorldMm{};
	cloudsim::core::Vec3 labelWorldMm{};
	bool hasEdgeSegment = false;
	cloudsim::core::Vec3 edgeAWorldMm{};
	cloudsim::core::Vec3 edgeBWorldMm{};
	/// 完整边折线（世界 mm）；优先于 edgeA/B 短线段
	std::vector<cloudsim::core::Vec3> edgePolylineWorldMm;
	/// 面三角 soup（世界 mm，每 3 点一三角）；选中后本体高亮
	std::vector<cloudsim::core::Vec3> faceTrianglesWorldMm;
};

/// 可达域体素中心（世界 mm）+ 边长
struct ReachableWorkspaceOverlay
{
	std::vector<cloudsim::core::Vec3> voxelCentersMm;
	double cellSizeMm = 60.0;
};

/// 播放游标：当前运动指令 TCP（世界 mm）；只改 MatrixTransform，勿整批重建路点轴
struct PlaybackCursorOverlay
{
	cloudsim::core::Vec3 positionMm{};
	cloudsim::core::Vec3 eulerDeg{};
};

/// 路点序号（1-based displayIndex）；邻域/抽稀后传入，避免全量编号叠字
struct WaypointIndexLabel
{
	int displayIndex = 0;
	cloudsim::core::Vec3 positionMm{};
};

struct RobotFrameOverlayUpdate
{
	std::string robotRootBackendId;
	bool showToolFrames = false;
	struct ToolEntry
	{
		std::string name;
		std::string mountBackendId;
		cloudsim::core::Mat4 localMatrix{};
		bool active = false;
	};
	std::vector<ToolEntry> toolFrames;
	bool showUserFrames = false;
	struct UserEntry
	{
		std::string name;
		std::string mountBackendId;
		cloudsim::core::Mat4 localMatrix{};
	};
	std::vector<UserEntry> userFrames;
};

} // namespace RobotOsgUi

#endif // OSGWIDGETCORE_ROBOTOSGUITYPES_H
