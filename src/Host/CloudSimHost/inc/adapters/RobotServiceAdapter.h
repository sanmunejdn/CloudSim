#pragma once

#include "IRobotService.h"

#include <QJsonArray>

class RobotProgramStore;

namespace cloudsim::host {

class DocumentHost;

/// IRobotService 门面：URDF/程序/规划在 Host，仿真 UI 仍经 IRobotUrdfImportContext 回调 DocumentPage
class RobotServiceAdapter final : public core::IRobotService
{
public:
	RobotServiceAdapter(DocumentHost& host, RobotProgramStore& programs);

	core::RobotRegistrationDto registerUrdfRobot(const QString& urdfPath,
		const core::ImportOptionsDto& options) override;

	bool applyJointAnglesRad(const core::ObjectId& sceneRootBackendId, const QVector<double>& jointAnglesRad,
		QString* outError = nullptr) override;

	bool planInstruction(const core::MotionInstructionDto& instruction, const core::PlanContextDto& context,
		core::PlanResultDto& out, QString* outError = nullptr) override;

	QJsonArray robotProgramsJson() const override;
	bool setRobotProgramsJson(const QJsonArray& programs, QString* outError = nullptr) override;

private:
	DocumentHost& m_host;
	RobotProgramStore& m_programs;
};

} // namespace cloudsim::host
