/// @file InstructionPropertyBindingSelfTest.cpp
/// @brief Binding / schema / snapshot 回归

#include "InstructionPropertyBindingSelfTest.h"

#include "InstructionPropertyBinding.h"
#include "RobotInstructionAxisConfiguration.h"
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

	// PTP 切配置：同/异配置类决定是否跳过最短角折圈；腕跳门仍供 LINE 硬拦
	{
		RobotInstruction::JointConfigurationClass a{};
		a.elbow = RobotInstruction::ElbowPosture::Up;
		a.wrist = RobotInstruction::WristPosture::NoFlip;
		a.arm = RobotInstruction::ArmPosture::Front;
		RobotInstruction::JointConfigurationClass b = a;
		check(RobotInstruction::sameJointConfigurationClass(a, b), "same class equal");
		check(!RobotInstruction::ptpShouldSkipShortestAngleFold(a, b), "same class keep fold");
		b.elbow = RobotInstruction::ElbowPosture::Down;
		check(RobotInstruction::ptpShouldSkipShortestAngleFold(a, b), "elbow switch skip fold");
		RobotInstruction::MotionAxisConfiguration want{};
		want.preset = "CUSTOM";
		want.elbow = RobotInstruction::ElbowPosture::Down;
		check(want.matchesClass(b), "matchesClass Down");
		check(!want.matchesClass(a), "matchesClass reject Up");
		const std::vector<std::string> names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
		const std::vector<double> seed(6, 0.0);
		std::vector<double> flipped = seed;
		flipped[4] = 2.0; // > π/2 腕跳
		const double jump = RobotInstruction::wristConfigurationJumpRad(flipped, seed, names);
		check(jump > RobotInstruction::kWristContinuityGateRad, "wrist jump exceeds LINE gate");
	}

	if (ok)
	{
		RunLogger::info("[InstructionPropertyBindingSelfTest] PASS");
	}
	return ok;
}
