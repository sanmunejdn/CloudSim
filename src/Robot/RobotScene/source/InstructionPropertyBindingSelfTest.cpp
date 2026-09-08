/// @file InstructionPropertyBindingSelfTest.cpp
/// @brief Binding / schema / snapshot 回归

#include "InstructionPropertyBindingSelfTest.h"

#include "InstructionPropertyBinding.h"
#include "RobotInstructionModel.h"
#include "RobotInstructionPropertySchema.h"

#include "RunLogger.h"

#include <cmath>
#include <set>
#include <string>
#include <vector>

namespace
{
void fail(std::vector<std::string>* failures, const std::string& msg)
{
	if (failures)
	{
		failures->push_back(msg);
	}
	RunLogger::warn("[InstructionPropertyBindingSelfTest] " + msg);
}

bool snapshotHasKey(const nlohmann::json& rows, const std::string& key)
{
	if (!rows.is_array())
	{
		return false;
	}
	for (const auto& row : rows)
	{
		if (row.is_object() && row.value("key", std::string()) == key)
		{
			return true;
		}
	}
	return false;
}

std::set<std::string> bindingKeys(const RobotInstruction::Base& cmd)
{
	std::set<std::string> keys;
	for (const RobotInstruction::InstructionPropertyBinding& b :
		 RobotInstruction::instruction_property_binding::collectBindings(cmd))
	{
		keys.insert(b.desc.key);
	}
	return keys;
}

std::set<std::string> schemaKeys(const RobotInstruction::Type type)
{
	std::set<std::string> keys;
	for (const property_core::PropertyDescriptor& d : RobotInstruction::schemaForInstructionType(type).descriptors)
	{
		keys.insert(d.key);
	}
	return keys;
}

bool snapshotKeysSubsetOfSchema(const RobotInstruction::Base& cmd)
{
	const std::set<std::string> schema = schemaKeys(cmd.type());
	const nlohmann::json rows = cmd.snapshotPropertyRows();
	if (!rows.is_array())
	{
		return false;
	}
	for (const auto& row : rows)
	{
		if (!row.is_object())
		{
			continue;
		}
		const std::string key = row.value("key", std::string());
		if (!key.empty() && schema.count(key) == 0)
		{
			return false;
		}
	}
	return true;
}

} // namespace

bool runInstructionPropertyBindingSelfTest(std::vector<std::string>* failures)
{
	bool ok = true;
	const auto check = [&](bool cond, const std::string& msg)
	{
		if (!cond)
		{
			ok = false;
			fail(failures, msg);
		}
	};

	RobotInstruction::PtpInstruction ptp;
	RobotInstruction::LineInstruction line;
	RobotInstruction::ArcInstruction arc;
	RobotInstruction::WaitInstruction wait;
	RobotInstruction::SetDigitalOutputInstruction setDo;

	check(bindingKeys(ptp) == schemaKeys(RobotInstruction::Type::PTP), "PTP binding keys != schema");
	check(bindingKeys(line) == schemaKeys(RobotInstruction::Type::LINE), "LINE binding keys != schema");
	check(bindingKeys(arc) == schemaKeys(RobotInstruction::Type::ARC), "ARC binding keys != schema");
	check(bindingKeys(wait) == schemaKeys(RobotInstruction::Type::WAIT), "WAIT binding keys != schema");
	check(bindingKeys(setDo) == schemaKeys(RobotInstruction::Type::SET_DO), "SetDo binding keys != schema");

	check(snapshotKeysSubsetOfSchema(ptp), "PTP snapshot key not in schema");
	check(snapshotKeysSubsetOfSchema(line), "LINE snapshot key not in schema");
	check(snapshotKeysSubsetOfSchema(arc), "ARC snapshot key not in schema");
	check(snapshotKeysSubsetOfSchema(wait), "WAIT snapshot key not in schema");
	check(snapshotKeysSubsetOfSchema(setDo), "SetDo snapshot key not in schema");

	std::string err;
	check(!ptp.applyPropertyChange("no.such.key", "1", &err), "unknown key should fail");
	check(err.find("Unknown") != std::string::npos, "unknown key error message");

	check(ptp.applyPropertyChange("motion.target.pose.x", "123.456", nullptr), "PTP apply pose.x");
	check(std::abs(ptp.pose().x - 123.456) < 1e-3, "PTP pose.x value");

	if (ok)
	{
		RunLogger::info("[InstructionPropertyBindingSelfTest] PASS");
	}
	return ok;
}
