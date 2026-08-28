/// @file RobotInstructionPlanningHelpers.cpp
/// @brief 指令规划辅助

#include "RobotInstructionPlanningHelpers.h"

#include "IRobotDocumentHost.h"
#include "IRobotOsgViewHost.h"
#include "RobotCoordinateFrames.h"
#include "RobotInstructionIkContext.h"

#include <QStringList>

namespace RobotInstructionPlanning
{
std::string encodeJointAnglesRadCsv(const QVector<double>& jointAnglesRad)
{
	QStringList parts;
	parts.reserve(jointAnglesRad.size());
	for (double v : jointAnglesRad)
	{
		parts.push_back(QString::number(v, 'g', 12));
	}
	return parts.join(QLatin1Char(',')).toStdString();
}

QVector<double> jointAnglesRadFromInstructionContext(const RobotInstruction::Base& ins)
{
	const auto& ext = ins.extensionProperties();
	const auto it = ext.find("context.currentJointRadCsv");
	if (it == ext.cend() || it->second.empty())
	{
		return {};
	}
	const QStringList parts = QString::fromStdString(it->second).split(QLatin1Char(','));
	QVector<double> out;
	out.reserve(parts.size());
	for (const QString& p : parts)
	{
		bool ok = false;
		const double v = p.trimmed().toDouble(&ok);
		if (ok)
		{
			out.push_back(v);
		}
	}
	return out;
}

double motionDurationSecFromInstruction(const RobotInstruction::Base& ins, const double defaultSec)
{
	const auto& ext = ins.extensionProperties();
	const auto it = ext.find("motion.durationSec");
	if (it != ext.cend() && !it->second.empty())
	{
		bool ok = false;
		const double v = QString::fromStdString(it->second).toDouble(&ok);
		if (ok && v > 1e-6)
		{
			return v;
		}
	}
	return defaultSec;
}

MotionPoseBackup backupInstructionPose(const RobotInstruction::Base& ins)
{
	MotionPoseBackup b;
	if (ins.hasPoseProperty())
	{
		b.pose = ins.pose();
		b.eulerDeg = ins.eulerDeg();
	}
	b.extensions = ins.extensionProperties();
	return b;
}

void restoreInstructionPose(RobotInstruction::Base& ins, const MotionPoseBackup& backup)
{
	if (ins.hasPoseProperty())
	{
		ins.setPose(backup.pose);
		ins.setEulerDeg(backup.eulerDeg);
	}
	// 整体替换：规划期新增的 extension（种子 CSV 等）不能残留
	const auto current = ins.extensionProperties();
	for (const auto& kv : current)
	{
		if (backup.extensions.find(kv.first) == backup.extensions.end())
		{
			ins.eraseExtensionProperty(kv.first);
		}
	}
	for (const auto& kv : backup.extensions)
	{
		ins.setExtensionProperty(kv.first, kv.second);
	}
}

void invalidateTaughtJointsForToolFrameChange(RobotInstruction::Base& ins)
{
	ins.eraseExtensionProperty("context.currentJointRadCsv");
	ins.eraseExtensionProperty("context.axisConfigSeeded");
}

void invalidateTaughtJointsFromMotionIndexForward(const std::vector<const RobotInstruction::Base*>& motions,
												  const int fromMotionIndexInclusive)
{
	if (fromMotionIndexInclusive < 0)
	{
		return;
	}
	for (size_t i = static_cast<size_t>(fromMotionIndexInclusive); i < motions.size(); ++i)
	{
		if (RobotInstruction::Base* ins = const_cast<RobotInstruction::Base*>(motions[i]))
		{
			invalidateTaughtJointsForToolFrameChange(*ins);
		}
	}
}

bool motionFollowsActiveToolFrame(const RobotInstruction::Base& ins)
{
	return RobotInstruction::motionUsesActiveToolFrame(ins);
}

void syncInstructionToolContextFromFrames(RobotInstruction::Base& ins,
										  const RobotCoordinate::RobotCoordinateFrameSet& frames)
{
	RobotInstruction::syncToolContextFromFrames(ins, frames);
}

void persistTaughtJointsAndToolContext(RobotInstruction::Base& ins, const QVector<double>& jointQ,
									   const RobotCoordinate::RobotCoordinateFrameSet& frames)
{
	(void)jointQ;
	// 指令只存 TCP；关节不落盘，仅同步工具系供规划读上下文
	syncInstructionToolContextFromFrames(ins, frames);
	ins.eraseExtensionProperty("context.currentJointRadCsv");
}

bool shouldUseTaughtJointCsv(const RobotInstruction::Base& ins,
							 const RobotCoordinate::RobotCoordinateFrameSet* coordinateFrames)
{
	(void)ins;
	(void)coordinateFrames;
	// 禁止示教关节短路：每点必须走求解器
	return false;
}

void prepareMotionInstructionForPlanning(RobotInstruction::Base& ins, const QVector<double>& rollingQ,
										 IRobotDocumentHost* doc, IRobotOsgViewHost* osg, int instIdx,
										 const QString& urdfPath, const std::string& defaultTcpLinkName,
										 const RobotCoordinate::RobotCoordinateFrameSet* coordinateFrames)
{
	(void)doc;
	(void)osg;
	(void)instIdx;
	std::vector<double> seed(rollingQ.begin(), rollingQ.end());
	RobotInstruction::prepareInstructionIkContext(ins, seed, urdfPath.toStdString(), defaultTcpLinkName,
												  coordinateFrames);
}

} // namespace RobotInstructionPlanning
