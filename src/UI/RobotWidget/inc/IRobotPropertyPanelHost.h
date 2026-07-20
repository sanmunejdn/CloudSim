#ifndef ROBOTWIDGET_IROBOTPROPERTYPANELHOST_H
#define ROBOTWIDGET_IROBOTPROPERTYPANELHOST_H

/// @file IRobotPropertyPanelHost.h
/// @brief 指令属性面板服务（MainWindow 实现）

#include "robotwidget_global.h"

#include <memory>

namespace RobotInstruction
{
class Base;
}

/// 指令属性面板服务（MainWindow 实现）
class ROBOTWIDGET_EXPORT IRobotPropertyPanelHost
{
public:
	virtual ~IRobotPropertyPanelHost() = default;

	virtual void refreshInstructionPropertyPanel(const std::shared_ptr<RobotInstruction::Base>& instruction,
												 bool refreshFeasibleAxisOptions = true) = 0;
	virtual void clearInstructionPropertyPanel() = 0;
	virtual void invalidateInstructionPropertyCache() = 0;
};

#endif // ROBOTWIDGET_IROBOTPROPERTYPANELHOST_H
