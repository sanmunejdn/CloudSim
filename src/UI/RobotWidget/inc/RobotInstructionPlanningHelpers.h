#pragma once

#include "RobotInstructionModel.h"
#include "robotwidget_global.h"

#include <unordered_map>
#include <vector>
#include <QVector>
#include <QString>

class IRobotDocumentHost;
class IRobotOsgViewHost;
namespace RobotCoordinate { struct RobotCoordinateFrameSet; }

namespace RobotInstructionPlanning
{

struct MotionPoseBackup
{
	RobotInstruction::Vec3 pose{};
	RobotInstruction::Vec3 eulerDeg{};
	std::unordered_map<std::string, std::string> extensions;
};

ROBOTWIDGET_EXPORT MotionPoseBackup backupInstructionPose(const RobotInstruction::Base& ins);
ROBOTWIDGET_EXPORT void restoreInstructionPose(RobotInstruction::Base& ins, const MotionPoseBackup& backup);

ROBOTWIDGET_EXPORT std::string encodeJointAnglesRadCsv(const QVector<double>& jointAnglesRad);
ROBOTWIDGET_EXPORT QVector<double> jointAnglesRadFromInstructionContext(const RobotInstruction::Base& ins);
ROBOTWIDGET_EXPORT double motionDurationSecFromInstruction(const RobotInstruction::Base& ins, double defaultSec = 0.5);

/// Drop taught joint CSV so preview/Run replan IK after \c motion.tool.frameId / tool matrix change.
ROBOTWIDGET_EXPORT void invalidateTaughtJointsForToolFrameChange(RobotInstruction::Base& ins);

/// 从 \a fromMotionIndexInclusive 起清除示教关节（含当前点），避免下游仍用旧工具系下的 CSV。
ROBOTWIDGET_EXPORT void invalidateTaughtJointsFromMotionIndexForward(
	const std::vector<const RobotInstruction::Base*>& motions,
	int fromMotionIndexInclusive);

/// True only when taught CSV matches this point's frozen tool context.
ROBOTWIDGET_EXPORT bool shouldUseTaughtJointCsv(
	const RobotInstruction::Base& ins,
	const RobotCoordinate::RobotCoordinateFrameSet* coordinateFrames);

/// motion.tool.frameId 为空或 "active" 时视为跟随全局激活工具系
ROBOTWIDGET_EXPORT bool motionFollowsActiveToolFrame(const RobotInstruction::Base& ins);

/// 将工具系写入 instruction context。跟随 active 的路点用 frames.activeToolFrameId，否则 resolve motion.tool.frameId。
ROBOTWIDGET_EXPORT void syncInstructionToolContextFromFrames(
	RobotInstruction::Base& ins,
	const RobotCoordinate::RobotCoordinateFrameSet& frames);

/// IK/规划成功后回写示教关节与冻结工具 context
ROBOTWIDGET_EXPORT void persistTaughtJointsAndToolContext(
	RobotInstruction::Base& ins,
	const QVector<double>& jointQ,
	const RobotCoordinate::RobotCoordinateFrameSet& frames);

ROBOTWIDGET_EXPORT void prepareMotionInstructionForPlanning(
	RobotInstruction::Base& ins,
	const QVector<double>& rollingQ,
	IRobotDocumentHost* doc,
	IRobotOsgViewHost* osg,
	int instIdx,
	const QString& urdfPath,
	const std::string& defaultTcpLinkName,
	const RobotCoordinate::RobotCoordinateFrameSet* coordinateFrames);

} // namespace RobotInstructionPlanning
