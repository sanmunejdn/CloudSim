#pragma once

#include "RobotInstructionModel.h"
#include "robotwidget_global.h"

#include <unordered_map>
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
