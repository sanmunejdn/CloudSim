/// @file HeadlessInstructionPropertyDelegate.cpp
/// @brief Web Headless 指令属性：直接改 ProgramStore，不经 Widget

#include "HeadlessInstructionPropertyDelegate.h"

#include "DocumentHost.h"
#include "RobotInstructionModel.h"
#include "RobotInstructionPropertyDto.h"
#include "RobotProgramCatalog.h"
#include "RobotProgramStore.h"

#include <memory>

namespace cloudsim::host
{
namespace
{
std::shared_ptr<RobotInstruction::Base>
findInSteps(const std::vector<std::shared_ptr<RobotInstruction::Base>>& steps, const std::string& idUtf8)
{
	for (const auto& step : steps)
	{
		if (!step)
		{
			continue;
		}
		if (step->id() == idUtf8)
		{
			return step;
		}
		if (auto hit = findInSteps(step->nestedSteps(), idUtf8))
		{
			return hit;
		}
		if (auto hit = findInSteps(step->elseSteps(), idUtf8))
		{
			return hit;
		}
	}
	return nullptr;
}

std::shared_ptr<RobotInstruction::Base> findInstruction(RobotProgramStore& store, const QString& instructionId)
{
	const std::string idUtf8 = instructionId.toStdString();
	const QStringList keys = store.allCatalogs().keys();
	for (const QString& bid : keys)
	{
		RobotInstruction::RobotProgramCatalog& catalog = store.catalogFor(bid);
		for (RobotInstruction::RobotProgram& prog : catalog.programs())
		{
			if (auto hit = findInSteps(prog.steps, idUtf8))
			{
				return hit;
			}
		}
	}
	return nullptr;
}
} // namespace

HeadlessInstructionPropertyDelegate::HeadlessInstructionPropertyDelegate(DocumentHost& host) : m_host(host) {}

QVector<core::PropertyRowDto> HeadlessInstructionPropertyDelegate::instructionPropertyRows(const QString& instructionId)
{
	const auto ins = findInstruction(m_host.robotProgramStore(), instructionId);
	if (!ins)
	{
		return {};
	}
	return propertyRowsFromInstructionSnapshotJson(ins->snapshotPropertyRows());
}

bool HeadlessInstructionPropertyDelegate::applyInstructionPropertyChange(const QString& instructionId,
																		 const QString& key, const QString& value,
																		 QString* outError)
{
	const auto ins = findInstruction(m_host.robotProgramStore(), instructionId);
	if (!ins)
	{
		if (outError)
		{
			*outError = QStringLiteral("instruction not found: %1").arg(instructionId);
		}
		return false;
	}
	std::string err;
	if (!ins->applyPropertyChange(key.toStdString(), value.toStdString(), &err))
	{
		if (outError)
		{
			*outError = QString::fromStdString(err);
		}
		return false;
	}
	// 对齐桌面：选信号名时同步解析端口
	if (key == QStringLiteral("logic.io.signalName") && !value.isEmpty())
	{
		const int port = m_host.namedSignalTable().resolvePort(value.toStdString(), -1);
		if (port >= 0 && ins->hasIoPortProperty())
			ins->setIoPort(port);
	}
	else if (key == QStringLiteral("logic.condition.signalName") && !value.isEmpty())
	{
		const int port = m_host.namedSignalTable().resolvePort(value.toStdString(), -1);
		if (port >= 0)
		{
			RobotInstruction::Condition c = ins->condition();
			c.ioPort = port;
			c.signalName = value.toStdString();
			ins->setCondition(c);
		}
	}
	return true;
}

core::FeasibleMotionAxisOptionsDto
HeadlessInstructionPropertyDelegate::queryFeasibleMotionAxisOptions(const QString& instructionId,
																	QVector<double>* outSeedJointRad)
{
	(void)instructionId;
	(void)outSeedJointRad;
	return {};
}

core::FeasibleMotionAxisOptionsDto HeadlessInstructionPropertyDelegate::cachedFeasibleMotionAxisOptions()
{
	return {};
}

} // namespace cloudsim::host
