#include "RobotInstructionModel.h"

#include "RobotInstructionAttribute.h"
#include "../../Data/PropertyCore/inc/PropertyAttribute.h"

#include <atomic>

namespace RobotInstruction
{
const Condition Base::s_emptyCondition{};

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
	case Type::PTP: return "ptp";
	case Type::LINE: return "line";
	case Type::WAIT: return "wait";
	case Type::IF: return "if";
	case Type::WHILE: return "while";
	case Type::SET_DO: return "set_do";
	case Type::SET_AO: return "set_ao";
	case Type::PathPlan: return "path_plan";
	default: return "unknown";
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

static const std::vector<std::shared_ptr<Base>> s_emptySteps;

const std::vector<std::shared_ptr<Base>>& Base::nestedSteps() const
{
	return s_emptySteps;
}

const std::vector<std::shared_ptr<Base>>& Base::elseSteps() const
{
	return s_emptySteps;
}

Base::Base()
	: m_id(makeInstructionId())
{
}

nlohmann::json Base::snapshotPropertyRows() const
{
	nlohmann::json rows = nlohmann::json::array();
	property_core::PropertyPipeline<Base, AttributeBase>::appendRows(m_attributes, *this, rows);
	for (const auto& kv : m_extensionProperties)
	{
		nlohmann::json row;
		row["key"] = kv.first;
		row["label"] = kv.first;
		row["editable"] = true;
		row["value"] = kv.second;
		rows.push_back(std::move(row));
	}
	return rows;
}

bool Base::applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg)
{
	if (property_core::PropertyPipeline<Base, AttributeBase>::apply(m_attributes, *this, key, value, errMsg))
	{
		return true;
	}
	m_extensionProperties[key] = value;
	return true;
}

void Base::addAttribute(const std::shared_ptr<AttributeBase>& attr)
{
	if (attr)
	{
		m_attributes.push_back(attr);
	}
}

void PtpInstruction::setAxisConfig(const std::string& v)
{
	m_axisConfiguration = motionAxisConfigurationFromLegacyString(v);
}

void LineInstruction::setAxisConfig(const std::string& v)
{
	m_axisConfiguration = motionAxisConfigurationFromLegacyString(v);
}

PtpInstruction::PtpInstruction()
{
	setType(Type::PTP);
	setName("PTP");
	m_axisConfiguration.preset = "AUTO";
	addAttribute(std::make_shared<PoseAttribute>());
	addAttribute(std::make_shared<EulerAttribute>());
	addAttribute(makeSpeedAttribute());
	addAttribute(makeAccelAttribute());
	for (const AttributePtr& attr : makeMotionAxisConfigAttributes())
	{
		addAttribute(attr);
	}
}

LineInstruction::LineInstruction()
{
	setType(Type::LINE);
	setName("LINE");
	m_axisConfiguration.preset = "AUTO";
	addAttribute(std::make_shared<PoseAttribute>());
	addAttribute(std::make_shared<EulerAttribute>());
	addAttribute(makeSpeedAttribute());
	addAttribute(makeAccelAttribute());
	addAttribute(makeBlendRadiusAttribute());
	for (const AttributePtr& attr : makeMotionAxisConfigAttributes())
	{
		addAttribute(attr);
	}
}

WaitInstruction::WaitInstruction()
{
	setType(Type::WAIT);
	setName("Wait");
	addAttribute(makeScalarDoubleAttribute(
		[](const Base& b) { return b.hasDurationProperty(); },
		[](const Base& b) { return b.durationSec(); },
		[](Base& b, const double& v) { b.setDurationSec(v); },
		"logic.wait.durationSec",
		"Duration (s)"));
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
	addAttribute(makeScalarDoubleAttribute(
		[](const Base& b) { return b.hasIoPortProperty(); },
		[](const Base& b) { return static_cast<double>(b.ioPort()); },
		[](Base& b, const double& v) { b.setIoPort(static_cast<int>(v)); },
		"logic.io.port",
		"Port"));
	addAttribute(makeScalarDoubleAttribute(
		[](const Base& b) { return b.hasIoValueProperty(); },
		[](const Base& b) { return b.ioBoolValue() ? 1.0 : 0.0; },
		[](Base& b, const double& v) { b.setIoBoolValue(v >= 0.5); },
		"logic.io.digitalValue",
		"Value (0/1)"));
}

PathPlanInstruction* asPathPlan(Base& ins)
{
	return ins.type() == Type::PathPlan ? dynamic_cast<PathPlanInstruction*>(&ins) : nullptr;
}

const PathPlanInstruction* asPathPlan(const Base& ins)
{
	return ins.type() == Type::PathPlan ? dynamic_cast<const PathPlanInstruction*>(&ins) : nullptr;
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
	addAttribute(makeScalarDoubleAttribute(
		[](const Base& b) { return b.hasIoPortProperty(); },
		[](const Base& b) { return static_cast<double>(b.ioPort()); },
		[](Base& b, const double& v) { b.setIoPort(static_cast<int>(v)); },
		"logic.io.port",
		"Port"));
	addAttribute(makeScalarDoubleAttribute(
		[](const Base& b) { return true; },
		[](const Base& b) { return b.ioAnalogValue(); },
		[](Base& b, const double& v) { b.setIoAnalogValue(v); },
		"logic.io.analogValue",
		"Analog value"));
}

} // namespace RobotInstruction
