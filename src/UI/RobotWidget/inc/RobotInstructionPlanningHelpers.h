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

/// True only when taught CSV matches this point's frozen \c motion.tool.frameId / tool matrix (not global "active").
ROBOTWIDGET_EXPORT bool shouldUseTaughtJointCsv(
	const RobotInstruction::Base& ins,
	const RobotCoordinate::RobotCoordinateFrameSet* coordinateFrames);

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
