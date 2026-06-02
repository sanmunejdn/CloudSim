#include "RobotInstructionPlanningHelpers.h"
#include "IRobotDocumentHost.h"
#include "IRobotOsgViewHost.h"
#include "RobotCoordinateFrames.h"

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

void invalidateTaughtJointsFromMotionIndexForward(
	const std::vector<const RobotInstruction::Base*>& motions,
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
	const auto& ext = ins.extensionProperties();
	const auto itMotion = ext.find(RobotCoordinate::kExtMotionToolFrameId);
	if (itMotion == ext.end() || itMotion->second.empty() || itMotion->second == "active")
	{
		return true;
	}
	return false;
}

void syncInstructionToolContextFromToolFrame(
	RobotInstruction::Base& ins,
	const RobotCoordinate::RobotCoordinateFrameSet& frames,
	const RobotCoordinate::RobotToolFrame& tool)
{
	const BackendMat4 toolMat = RobotCoordinate::frameToMat4(tool.T_flange_tool);
	ins.setExtensionProperty(
		RobotCoordinate::kExtContextToolFrameMat4, RobotCoordinate::encodeMat4Csv(toolMat));
	ins.setExtensionProperty("context.activeToolFrameId", tool.id);
	const std::string flangeLink = RobotCoordinate::effectiveFlangeLinkName(frames, tool);
	if (!flangeLink.empty())
	{
		ins.setExtensionProperty("context.flangeLinkName", flangeLink);
	}
}

void syncInstructionToolContextFromFrames(
	RobotInstruction::Base& ins,
	const RobotCoordinate::RobotCoordinateFrameSet& frames)
{
	const RobotCoordinate::RobotToolFrame* tool = nullptr;
	if (motionFollowsActiveToolFrame(ins))
	{
		// 跟随 active 的路点必须用全局激活工具，不能走 resolve 里 stale 的 frozen id
		tool = RobotCoordinate::activeToolFrame(frames);
	}
	else if (const RobotCoordinate::RobotToolFrame* resolved =
				 RobotCoordinate::resolveToolFrameForExtension(frames, ins.extensionProperties()))
	{
		tool = resolved;
	}
	if (tool)
	{
		syncInstructionToolContextFromToolFrame(ins, frames, *tool);
	}
}

void persistTaughtJointsAndToolContext(
	RobotInstruction::Base& ins,
	const QVector<double>& jointQ,
	const RobotCoordinate::RobotCoordinateFrameSet& frames)
{
	if (jointQ.isEmpty())
	{
		return;
	}
	ins.setExtensionProperty("context.currentJointRadCsv", encodeJointAnglesRadCsv(jointQ));
	syncInstructionToolContextFromFrames(ins, frames);
}

bool shouldUseTaughtJointCsv(
	const RobotInstruction::Base& ins,
	const RobotCoordinate::RobotCoordinateFrameSet* coordinateFrames)
{
	if (jointAnglesRadFromInstructionContext(ins).isEmpty())
	{
		return false;
	}
	const auto& ext = ins.extensionProperties();
	const auto itMotion = ext.find(RobotCoordinate::kExtMotionToolFrameId);
	const std::string motionId = (itMotion != ext.end()) ? itMotion->second : std::string();
	const bool followsActive = motionId.empty() || motionId == "active";
	if (followsActive)
	{
		if (!coordinateFrames)
		{
			return false;
		}
		const auto itFrozen = ext.find("context.activeToolFrameId");
		if (itFrozen == ext.end() || itFrozen->second.empty())
		{
			return false;
		}
		if (itFrozen->second != coordinateFrames->activeToolFrameId)
		{
			return false;
		}
	}
	else
	{
		const auto itFrozen = ext.find("context.activeToolFrameId");
		if (itFrozen != ext.end() && !itFrozen->second.empty() && itFrozen->second != motionId)
		{
			return false;
		}
	}
	if (coordinateFrames)
	{
		const BackendMat4 live = RobotCoordinate::toolMat4ForExtension(*coordinateFrames, ext);
		const auto itMat = ext.find(RobotCoordinate::kExtContextToolFrameMat4);
		if (itMat != ext.end() && !itMat->second.empty())
		{
			BackendMat4 frozen{};
			if (RobotCoordinate::parseMat4Csv(itMat->second, frozen))
			{
				const std::string liveCsv = RobotCoordinate::encodeMat4Csv(live);
				const std::string frozenCsv = RobotCoordinate::encodeMat4Csv(frozen);
				if (liveCsv != frozenCsv)
				{
					return false;
				}
			}
		}
	}
	return true;
}

void prepareMotionInstructionForPlanning(
	RobotInstruction::Base& ins,
	const QVector<double>& rollingQ,
	IRobotDocumentHost* doc,
	IRobotOsgViewHost* osg,
	int instIdx,
	const QString& urdfPath,
	const std::string& defaultTcpLinkName,
	const RobotCoordinate::RobotCoordinateFrameSet* coordinateFrames)
{
	(void)osg;
	QStringList parts;
	parts.reserve(rollingQ.size());
	for (double v : rollingQ)
	{
		parts.push_back(QString::number(v, 'g', 12));
	}
	ins.setExtensionProperty("context.currentJointRadCsv", parts.join(QLatin1Char(',')).toStdString());
	ins.setExtensionProperty("context.urdfPath", urdfPath.toStdString());
	ins.setExtensionProperty("context.tcpLinkName", defaultTcpLinkName);
	if (coordinateFrames)
	{
		const BackendMat4 T_tool = RobotCoordinate::toolMat4ForExtension(*coordinateFrames, ins.extensionProperties());
		ins.setExtensionProperty("context.toolFrameMat4", RobotCoordinate::encodeMat4Csv(T_tool));
		if (const RobotCoordinate::RobotToolFrame* tool =
				RobotCoordinate::resolveToolFrameForExtension(*coordinateFrames, ins.extensionProperties()))
		{
			const std::string flangeLink = RobotCoordinate::effectiveFlangeLinkName(*coordinateFrames, *tool);
			if (!flangeLink.empty())
			{
				ins.setExtensionProperty("context.flangeLinkName", flangeLink);
			}
		}
		else if (!coordinateFrames->flangeLinkName.empty())
		{
			ins.setExtensionProperty("context.flangeLinkName", coordinateFrames->flangeLinkName);
		}
	}
	(void)doc;
	(void)instIdx;
}

} // namespace RobotInstructionPlanning
