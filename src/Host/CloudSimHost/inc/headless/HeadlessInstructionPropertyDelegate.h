#ifndef CLOUDSIMHOST_HEADLESSINSTRUCTIONPROPERTYDELEGATE_H
#define CLOUDSIMHOST_HEADLESSINSTRUCTIONPROPERTYDELEGATE_H

/// @file HeadlessInstructionPropertyDelegate.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Web/Headless：从 RobotProgramStore 读写指令属性（无 Widget）

#include "cloudsim_host_global.h"

#include "IRobotInstructionPropertyDelegate.h"

namespace cloudsim::host
{
class DocumentHost;

class CLOUDSIM_HOST_EXPORT HeadlessInstructionPropertyDelegate final : public IRobotInstructionPropertyDelegate
{
public:
	explicit HeadlessInstructionPropertyDelegate(DocumentHost& host);

	QVector<core::PropertyRowDto> instructionPropertyRows(const QString& instructionId) override;
	bool applyInstructionPropertyChange(const QString& instructionId, const QString& key, const QString& value,
										QString* outError = nullptr) override;
	core::FeasibleMotionAxisOptionsDto queryFeasibleMotionAxisOptions(const QString& instructionId,
																	  QVector<double>* outSeedJointRad = nullptr) override;
	core::FeasibleMotionAxisOptionsDto cachedFeasibleMotionAxisOptions() override;

private:
	DocumentHost& m_host;
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_HEADLESSINSTRUCTIONPROPERTYDELEGATE_H
