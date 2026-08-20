#ifndef CLOUDSIMHOST_ROBOTSERVICEADAPTER_H
#define CLOUDSIMHOST_ROBOTSERVICEADAPTER_H

/// @file RobotServiceAdapter.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Host 机器人服务

#include "IRobotService.h"

#include <QJsonArray>

class RobotProgramStore;

namespace cloudsim::host
{
class DocumentHost;

/// Host 机器人服务
class RobotServiceAdapter final : public core::IRobotService
{
public:
	RobotServiceAdapter(DocumentHost& host, RobotProgramStore& programs);

	core::RobotRegistrationDto registerUrdfRobot(const QString& urdfPath,
												 const core::ImportOptionsDto& options) override;

	bool applyJointAnglesRad(const core::ObjectId& sceneRootBackendId, const QVector<double>& jointAnglesRad,
							 QVector<double>* outAggregated = nullptr, QString* outError = nullptr) override;

	bool planInstruction(const core::MotionInstructionDto& instruction, const core::PlanContextDto& context,
						 core::PlanResultDto& out, QString* outError = nullptr) override;

	QJsonArray robotProgramsJson() const override;
	bool setRobotProgramsJson(const QJsonArray& programs, QString* outError = nullptr) override;

	QVector<core::PropertyRowDto> instructionPropertyRows(const QString& instructionId) const override;
	bool applyInstructionPropertyChange(const QString& instructionId, const QString& key, const QString& value,
										QString* outError = nullptr) override;
	QStringList feasibleMotionAxisConfigTokens(const QString& instructionId,
											   const core::MotionInstructionDto& instruction,
											   const core::PlanContextDto& context) const override;

	core::FeasibleMotionAxisOptionsDto
	queryFeasibleMotionAxisOptions(const QString& instructionId,
								   QVector<double>* outSeedJointRad = nullptr) const override;
	core::FeasibleMotionAxisOptionsDto cachedFeasibleMotionAxisOptions() const override;

private:
	DocumentHost& m_host;
	RobotProgramStore& m_programs;
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_ROBOTSERVICEADAPTER_H
