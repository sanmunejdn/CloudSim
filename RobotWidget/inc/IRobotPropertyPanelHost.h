#pragma once

#include "robotwidget_global.h"

#include <memory>

namespace RobotInstruction { class Base; }

/// Property panel services for robot instruction editing (implemented by Widget MainWindow host).
class ROBOTWIDGET_EXPORT IRobotPropertyPanelHost
{
public:
	virtual ~IRobotPropertyPanelHost() = default;

	virtual void refreshInstructionPropertyPanel(
		const std::shared_ptr<RobotInstruction::Base>& instruction,
		bool refreshFeasibleAxisOptions = true) = 0;
	virtual void clearInstructionPropertyPanel() = 0;
	virtual void invalidateInstructionPropertyCache() = 0;
};
