/// @file RobotInstructionCondition.cpp
/// @brief 指令条件

#include "RobotInstructionCondition.h"

#include <algorithm>
#include <cctype>

namespace RobotInstruction
{
namespace
{
std::string toLower(std::string s)
{
	std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	return s;
}
} // namespace

Condition conditionFromJson(const nlohmann::json& j)
{
	Condition c;
	if (!j.is_object())
	{
		return c;
	}
	const std::string kind = j.value("kind", std::string("always"));
	const std::string k = toLower(kind);
	if (k == "never")
	{
		c.kind = ConditionKind::Never;
	}
	else if (k == "io")
	{
		c.kind = ConditionKind::Io;
		c.ioPort = j.value("port", 0);
		c.ioEquals = j.value("equals", false);
		c.signalName = j.value("signalName", std::string());
	}
	else if (k == "compare")
	{
		c.kind = ConditionKind::Compare;
		c.compareLeft = j.value("left", std::string());
		c.compareOp = j.value("op", std::string("eq"));
		c.compareRight = j.value("right", 0.0);
	}
	else
	{
		c.kind = ConditionKind::Always;
	}
	return c;
}

nlohmann::json conditionToJson(const Condition& c)
{
	nlohmann::json j;
	switch (c.kind)
	{
	case ConditionKind::Never:
		j["kind"] = "never";
		break;
	case ConditionKind::Io:
		j["kind"] = "io";
		j["port"] = c.ioPort;
		j["equals"] = c.ioEquals;
		if (!c.signalName.empty())
		{
			j["signalName"] = c.signalName;
		}
		break;
	case ConditionKind::Compare:
		j["kind"] = "compare";
		j["left"] = c.compareLeft;
		j["op"] = c.compareOp;
		j["right"] = c.compareRight;
		break;
	case ConditionKind::Always:
	default:
		j["kind"] = "always";
		break;
	}
	return j;
}

} // namespace RobotInstruction
