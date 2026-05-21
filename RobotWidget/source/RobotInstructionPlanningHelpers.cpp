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
	}
	(void)doc;
	(void)instIdx;
}

} // namespace RobotInstructionPlanning
