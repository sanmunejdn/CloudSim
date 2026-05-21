#pragma once

#include "RobotInstructionController.h"
#include "robotwidget_global.h"

#include <memory>

class RobotSimulationController;

/// Instruction property-panel helpers; full Qt property browser UI remains in Widget \ref MainWindow.
class ROBOTWIDGET_EXPORT RobotInstructionPropertyEditor
{
public:
	static RobotInstruction::FeasibleMotionAxisConfigurationOptions feasibleMotionAxisConfigurationOptions(
		RobotSimulationController& controller,
		const std::shared_ptr<RobotInstruction::Base>& instruction,
		QVector<double>* outSeedJointRad = nullptr);
};
