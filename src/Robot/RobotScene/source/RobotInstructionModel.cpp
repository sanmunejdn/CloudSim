/// @file RobotInstructionModel.cpp
/// @brief 指令模型

#include "RobotInstructionModel.h"

#include "InstructionPropertyBinding.h"

#include <atomic>

namespace RobotInstruction
{
const Condition Base::s_emptyCondition{};
const std::string Base::s_emptyString{};

bool isPathPlanType(const Type t)
{
	return t == Type::PathPlan;
}

bool isPlanningCategory(const Category c)
{
	return c == Category::Planning;
}

Category categoryForType(const Type t)
{
	switch (t)
	{
	case Type::PTP:
	case Type::LINE:
	case Type::ARC:
		return Category::Motion;
	case Type::PathPlan:
		return Category::Planning;
	default:
		return Category::Logic;
	}
}

std::string typeToString(const Type t)
{
	switch (t)
	{
	case Type::PTP:
		return "ptp";
	case Type::LINE:
		return "line";
	case Type::ARC:
		return "arc";
	case Type::WAIT:
		return "wait";
	case Type::IF:
		return "if";
	case Type::WHILE:
		return "while";
	case Type::SET_DO:
		return "set_do";
	case Type::SET_AO:
		return "set_ao";
	case Type::PathPlan:
		return "path_plan";
	case Type::DeviceAxis:
		return "device_axis";
	default:
		return "unknown";
	}
}

bool typeFromString(const std::string& s, Type& out)
{
	if (s == "ptp" || s == "PTP")
	{
		out = Type::PTP;
		return true;
	}
	if (s == "line" || s == "LINE")
	{
		out = Type::LINE;
		return true;
	}
	if (s == "arc" || s == "ARC" || s == "circ")
	{
		out = Type::ARC;
		return true;
	}
	if (s == "wait" || s == "WAIT")
	{
		out = Type::WAIT;
		return true;
	}
	if (s == "if" || s == "IF")
	{
		out = Type::IF;
		return true;
	}
	if (s == "while" || s == "WHILE")
	{
		out = Type::WHILE;
		return true;
	}
	if (s == "set_do" || s == "setDO" || s == "SET_DO")
	{
		out = Type::SET_DO;
		return true;
	}
	if (s == "set_ao" || s == "setAO" || s == "SET_AO")
	{
		out = Type::SET_AO;
		return true;
	}
	if (s == "path_plan" || s == "PathPlan" || s == "PATH_PLAN")
	{
		out = Type::PathPlan;
		return true;
	}
	if (s == "device_axis" || s == "DeviceAxis" || s == "DEVICE_AXIS")
	{
		out = Type::DeviceAxis;
		return true;
	}
	return false;
}

std::string makeInstructionId()
{
	static std::atomic<unsigned long long> sCounter{1ULL};
	const unsigned long long v = sCounter.fetch_add(1ULL);
	return std::string("INS_") + std::to_string(v);
}

const Condition& Base::condition() const
{
	return s_emptyCondition;
}

const std::string& Base::ioSignalName() const
{
	return s_emptyString;
}

const std::string& Base::deviceBackendId() const
{
	return s_emptyString;
}

static const std::vector<std::shared_ptr<Base>> s_emptySteps;

const std::vector<std::shared_ptr<Base>>& Base::nestedSteps() const
{
	return s_emptySteps;
}

const std::vector<std::shared_ptr<Base>>& Base::elseSteps() const
{
	return s_emptySteps;
}

Base::Base() : m_id(makeInstructionId()) {}

nlohmann::json Base::snapshotPropertyRows() const
{
	nlohmann::json rows = nlohmann::json::array();
	instruction_property_binding::appendBindingRows(*this, rows);
	return rows;
}

bool Base::applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg)
{
	if (instruction_property_binding::applyBindingKey(*this, key, value, errMsg))
	{
		return true;
	}
	if (errMsg)
	{
		*errMsg = "Unknown property key.";
	}
	return false;
}

void PtpInstruction::setAxisConfig(const std::string& v)
{
	m_axisConfiguration = motionAxisConfigurationFromLegacyString(v);
}

void LineInstruction::setAxisConfig(const std::string& v)
{
	m_axisConfiguration = motionAxisConfigurationFromLegacyString(v);
}

void ArcInstruction::setAxisConfig(const std::string& v)
{
	m_axisConfiguration = motionAxisConfigurationFromLegacyString(v);
}

PtpInstruction::PtpInstruction()
{
	setType(Type::PTP);
	setName("PTP");
	m_axisConfiguration.preset = "AUTO";
}

LineInstruction::LineInstruction()
{
	setType(Type::LINE);
	setName("LINE");
	m_axisConfiguration.preset = "AUTO";
}

ArcInstruction::ArcInstruction()
{
	setType(Type::ARC);
	setName("ARC");
	m_axisConfiguration.preset = "AUTO";
}

WaitInstruction::WaitInstruction()
{
	setType(Type::WAIT);
	setName("Wait");
	m_condition.kind = ConditionKind::Io;
	m_condition.ioEquals = true;
	m_durationSec = 0.0;
}

IfInstruction::IfInstruction()
{
	setType(Type::IF);
	setName("If");
}

WhileInstruction::WhileInstruction()
{
	setType(Type::WHILE);
	setName("While");
}

SetDigitalOutputInstruction::SetDigitalOutputInstruction()
{
	setType(Type::SET_DO);
	setName("Set DO");
}

PathPlanInstruction* asPathPlan(Base& ins)
{
	return ins.type() == Type::PathPlan ? dynamic_cast<PathPlanInstruction*>(&ins) : nullptr;
}

const PathPlanInstruction* asPathPlan(const Base& ins)
{
	return ins.type() == Type::PathPlan ? dynamic_cast<const PathPlanInstruction*>(&ins) : nullptr;
}

DeviceAxisInstruction* asDeviceAxis(Base& ins)
{
	return ins.type() == Type::DeviceAxis ? dynamic_cast<DeviceAxisInstruction*>(&ins) : nullptr;
}

const DeviceAxisInstruction* asDeviceAxis(const Base& ins)
{
	return ins.type() == Type::DeviceAxis ? dynamic_cast<const DeviceAxisInstruction*>(&ins) : nullptr;
}

PathPlanInstruction::PathPlanInstruction()
{
	setType(Type::PathPlan);
	setName("Path Plan");
	m_rawTrajectoryKey = id();
}

SetAnalogOutputInstruction::SetAnalogOutputInstruction()
{
	setType(Type::SET_AO);
	setName("Set AO");
}

DeviceAxisInstruction::DeviceAxisInstruction()
{
	setType(Type::DeviceAxis);
	setName("Device Axis");
}

} // namespace RobotInstruction
