#ifndef CLOUDSIMHOST_IROBOTINSTRUCTIONPROPERTYDELEGATE_H
#define CLOUDSIMHOST_IROBOTINSTRUCTIONPROPERTYDELEGATE_H

/// @file IRobotInstructionPropertyDelegate.h
/// @brief 仿真指令属性编辑（由 Widget/MainWindowRobotHost 实现，供 RobotServiceAdapter 转发）

#include "cloudsim_host_global.h"

#include "CoreTypes.h"

#include <QVector>

namespace cloudsim::host
{
/// 仿真指令属性编辑（由 Widget/MainWindowRobotHost 实现，供 RobotServiceAdapter 转发）
class CLOUDSIM_HOST_EXPORT IRobotInstructionPropertyDelegate
{
public:
	virtual ~IRobotInstructionPropertyDelegate() = default;

	virtual QVector<core::PropertyRowDto> instructionPropertyRows(const QString& instructionId) = 0;
	virtual bool applyInstructionPropertyChange(const QString& instructionId, const QString& key, const QString& value,
												QString* outError = nullptr) = 0;
	virtual core::FeasibleMotionAxisOptionsDto
	queryFeasibleMotionAxisOptions(const QString& instructionId, QVector<double>* outSeedJointRad = nullptr) = 0;
	virtual core::FeasibleMotionAxisOptionsDto cachedFeasibleMotionAxisOptions() = 0;
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_IROBOTINSTRUCTIONPROPERTYDELEGATE_H
