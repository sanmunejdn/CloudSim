#pragma once

#include "RobotInstructionController.h"
#include "robotwidget_global.h"

#include <memory>

class RobotSimulationController;

/// 指令属性面板辅助（完整 Qt 属性浏览器仍在 MainWindow）
class ROBOTWIDGET_EXPORT RobotInstructionPropertyEditor
{
public:
	static RobotInstruction::FeasibleMotionAxisConfigurationOptions feasibleMotionAxisConfigurationOptions(
		RobotSimulationController& controller,
		const std::shared_ptr<RobotInstruction::Base>& instruction,
		QVector<double>* outSeedJointRad = nullptr);
};
