/// @file RobotInstructionPropertyEditor.cpp
/// @brief 指令属性编辑

#include "RobotInstructionPropertyEditor.h"

#include "RobotSimulationController.h"

RobotInstruction::FeasibleMotionAxisConfigurationOptions
RobotInstructionPropertyEditor::feasibleMotionAxisConfigurationOptions(
	RobotSimulationController& controller, const std::shared_ptr<RobotInstruction::Base>& instruction,
	QVector<double>* outSeedJointRad)
{
	return controller.feasibleMotionAxisConfigurationOptionsForInstruction(instruction, outSeedJointRad);
}
