#pragma once

#include "CoreTypes.h"
#include "cloudsim_core_global.h"

#include <QJsonArray>
#include <QJsonObject>

namespace cloudsim::core {

/// 文档机器人服务
class CLOUDSIM_CORE_EXPORT IRobotService
{
public:
	virtual ~IRobotService() = default;

	virtual RobotRegistrationDto registerUrdfRobot(const QString& urdfPath, const ImportOptionsDto& options) = 0;

	virtual bool applyJointAnglesRad(const ObjectId& sceneRootBackendId, const QVector<double>& jointAnglesRad,
		QVector<double>* outAggregated = nullptr, QString* outError = nullptr) = 0;

	virtual bool planInstruction(const MotionInstructionDto& instruction, const PlanContextDto& context,
		PlanResultDto& out, QString* outError = nullptr) = 0;

	virtual QJsonArray robotProgramsJson() const = 0;
	virtual bool setRobotProgramsJson(const QJsonArray& programs, QString* outError = nullptr) = 0;

	virtual QVector<PropertyRowDto> instructionPropertyRows(const QString& instructionId) const = 0;
	virtual bool applyInstructionPropertyChange(const QString& instructionId, const QString& key,
		const QString& value, QString* outError = nullptr) = 0;
	virtual QStringList feasibleMotionAxisConfigTokens(const QString& instructionId,
		const MotionInstructionDto& instruction, const PlanContextDto& context) const = 0;
};

} // namespace cloudsim::core
