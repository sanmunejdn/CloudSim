#include "adapters/RobotServiceAdapter.h"

#include "RobotProgramStore.h"

namespace cloudsim::host {

RobotServiceAdapter::RobotServiceAdapter(RobotProgramStore& programs) : m_programs(programs) {}

core::RobotRegistrationDto RobotServiceAdapter::registerUrdfRobot(const QString& urdfPath,
	const core::ImportOptionsDto& options)
{
	(void)urdfPath;
	(void)options;
	return {false, QStringLiteral("registerUrdfRobot: use RobotWidget import path"), {}};
}

bool RobotServiceAdapter::applyJointAnglesRad(const core::ObjectId& sceneRootBackendId,
	const QVector<double>& jointAnglesRad, QString* outError)
{
	(void)sceneRootBackendId;
	(void)jointAnglesRad;
	if (outError)
		*outError = QStringLiteral("applyJointAnglesRad: use RobotSimulationController");
	return false;
}

bool RobotServiceAdapter::planInstruction(const core::MotionInstructionDto& instruction,
	const core::PlanContextDto& context, core::PlanResultDto& out, QString* outError)
{
	(void)instruction;
	(void)context;
	out = {};
	if (outError)
		*outError = QStringLiteral("planInstruction: use RobotScene planner");
	return false;
}

QJsonArray RobotServiceAdapter::robotProgramsJson() const
{
	(void)m_programs;
	return {};
}

bool RobotServiceAdapter::setRobotProgramsJson(const QJsonArray& programs, QString* outError)
{
	(void)programs;
	if (outError)
		*outError = QStringLiteral("setRobotProgramsJson: use RobotProjectIoAdapter");
	return false;
}

} // namespace cloudsim::host
