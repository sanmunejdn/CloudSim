#ifndef ROBOTWIDGET_INSTRUCTIONPROPERTYPANEL_H
#define ROBOTWIDGET_INSTRUCTIONPROPERTYPANEL_H

/// @file InstructionPropertyPanel.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 仿真指令 Qt 属性浏览器（实现位于 RobotWidget，Widget 不 include RobotScene）

#include "robotwidget_global.h"

#include <QVariant>
#include <QVector>
#include <memory>

namespace RobotInstruction
{
class Base;
struct FeasibleMotionAxisConfigurationOptions;
} // namespace RobotInstruction

class IRobotInstructionPropertyUiHost;
class QtProperty;

/// 仿真指令 Qt 属性浏览器（实现位于 RobotWidget，Widget 不 include RobotScene）
class ROBOTWIDGET_EXPORT InstructionPropertyPanel
{
public:
	static void update(IRobotInstructionPropertyUiHost& host,
					   const std::shared_ptr<RobotInstruction::Base>& instruction, bool refreshFeasibleAxisOptions);

	static void applySuggestedAxisPresetFromSeedIfNeeded(
		IRobotInstructionPropertyUiHost& host, const std::shared_ptr<RobotInstruction::Base>& instruction,
		const QVector<double>& seedJointRad, const RobotInstruction::FeasibleMotionAxisConfigurationOptions& feasible);

	static bool handleVariantPropertyValueChanged(IRobotInstructionPropertyUiHost& host, QtProperty* property,
												  const QVariant& value);
};

#endif // ROBOTWIDGET_INSTRUCTIONPROPERTYPANEL_H
