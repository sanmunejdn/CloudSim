/// @file InstructionPropertyBindingSelfTest.cpp
/// @brief Binding / schema / snapshot 回归

#include "InstructionPropertyBindingSelfTest.h"

#include "IkGoldenSelfTest.h"
#include "IkSolveMetrics.h"
#include "IkSolveTypes.h"
#include "InstructionPropertyBinding.h"
#include "RobotInstructionAxisConfiguration.h"
#include "RobotInstructionModel.h"
#include "RobotInstructionPropertySchema.h"
#include "RobotTeachIk.h"
#include "RunLogger.h"
#include "UrdfIkMachineProfile.h"

#include <cmath>
#include <set>
#include <string>
#include <unordered_map>
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

	// IK 策略：相对种子折圈、PTP/路径 policy、腕门
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
		flipped[4] = 2.0; // > π/2 相对种子翻腕
		check(RobotInstruction::shouldSkipShortestAngleFold(seed, flipped, names), "relative wrist flip skip fold");
		check(!RobotInstruction::shouldSkipShortestAngleFold(seed, seed, names), "same q keep fold");
		const double jump = RobotInstruction::wristConfigurationJumpRad(flipped, seed, names);
		check(jump > RobotInstruction::kWristContinuityGateRad, "wrist jump exceeds LINE gate");

		RobotInstruction::MotionAxisConfiguration autoCfg{};
		const auto ptpPol = RobotInstruction::makePtpIkPolicy(autoCfg);
		check(!ptpPol.requireWristContinuity, "PTP AUTO wrist soft");
		check(!ptpPol.requireMatchesClass, "PTP AUTO no class");
		check(ptpPol.mode == RobotInstruction::IkSolveMode::Search, "PTP AUTO Search");
		check(ptpPol.budgetMs > 0, "PTP budget");
		check(ptpPol.maxSeeds > 0, "PTP maxSeeds");
		check(ptpPol.raceNumericalFallback, "PTP race numerical");
		check(!ptpPol.raceAnalyticalAndNumerical, "PTP default sequential IK");
		check(!ptpPol.allowCartDegrade, "PTP allowCartDegrade default false");
		check(ptpPol.budgetMs >= 200, "PTP budget ample");
		RobotInstruction::MotionAxisConfiguration pathCfg = want;
		const auto pathPol = RobotInstruction::makeCartesianPathIkPolicy(pathCfg);
		check(pathPol.requireWristContinuity, "path wrist hard");
		check(pathPol.requireMatchesClass, "path class hard factory");
		check(!pathPol.allowCartDegrade, "path allowCartDegrade default false");
		check(pathPol.mode == RobotInstruction::IkSolveMode::Search, "path endpoint Search");
		check(pathPol.rankMode == RobotInstruction::IkRankMode::Distance, "path rank Distance");
		const auto pathAuto = RobotInstruction::makeCartesianPathIkPolicy(autoCfg);
		check(pathAuto.requireWristContinuity, "path AUTO wrist still hard");
		check(!pathAuto.requireMatchesClass, "path AUTO no class");
		RobotInstruction::IkSelectionPolicy ov = pathAuto;
		std::unordered_map<std::string, std::string> ext;
		ext["context.ikRankMode"] = "Manip";
		ext["context.allowCartDegrade"] = "1";
		ext["context.playbackPlanLite"] = "1";
		RobotInstruction::applyIkPolicyExtensionOverrides(ext, ov);
		check(ov.rankMode == RobotInstruction::IkRankMode::Manip, "override rank Manip");
		check(ov.allowCartDegrade, "override degrade on");
		check(ov.budgetMs < pathAuto.budgetMs, "lite halves budget");
		const std::string tags =
			RobotInstruction::formatIkPolicySummaryTags(ov, true, true, true);
		check(tags.find("[rank=Manip]") != std::string::npos, "tag rank");
		check(tags.find("[cartDegraded]") != std::string::npos, "tag cartDegraded");
		check(tags.find("[denseRetry]") != std::string::npos, "tag denseRetry");

		check(RobotInstruction::classifyIkFailMessage("not spherical-wrist 6R") ==
				  RobotInstruction::IkFailKind::Topology,
			  "fail Topology");
		check(RobotInstruction::classifyIkFailMessage("IK预算耗尽") == RobotInstruction::IkFailKind::Budget,
			  "fail Budget");
		check(RobotInstruction::formatIkFailKindTag(RobotInstruction::IkFailKind::Budget).find("[fail=Budget]") !=
				  std::string::npos,
			  "fail tag Budget");
		check(!RobotInstruction::formatIkFailKindZh(RobotInstruction::IkFailKind::Residual).empty(), "fail zh");
		check(RobotInstruction::formatIkFailKindZh(RobotInstruction::IkFailKind::Budget).find("ikProfile=run") !=
				  std::string::npos,
			  "Budget zh escalate hint");
		check(RobotInstruction::classifyIkFailMessage("路径终点腕连续失败（相对起点超过门限）") ==
				  RobotInstruction::IkFailKind::Wrist,
			  "fail Wrist");
		check(RobotInstruction::classifyIkFailMessage("路径终点轴配置不匹配") ==
				  RobotInstruction::IkFailKind::AxisClass,
			  "fail AxisClass");

		check(RobotInstruction::encodeIkBranchId(1, 1) == 3, "branch encode");
		check(RobotInstruction::ikBranchElbowSign(3) == 1 && RobotInstruction::ikBranchWrist(3) == 1, "branch decode");

		RobotInstruction::IkSelectionPolicy dragPol = ptpPol;
		std::unordered_map<std::string, std::string> dragExt;
		dragExt["context.ikProfile"] = "drag";
		RobotInstruction::applyIkPolicyExtensionOverrides(dragExt, dragPol);
		check(dragPol.mode == RobotInstruction::IkSolveMode::NearSeed, "drag NearSeed");
		check(dragPol.budgetMs <= 80, "drag budget<=80");
		check(dragPol.maxSeeds <= 1, "drag maxSeeds<=1");
		check(!dragPol.allowParallelSeeds, "drag no parallel");

		RobotInstruction::IkSelectionPolicy parPol = ptpPol;
		std::unordered_map<std::string, std::string> parExt;
		parExt["context.ikParallel"] = "1";
		RobotInstruction::applyIkPolicyExtensionOverrides(parExt, parPol);
		check(parPol.allowParallelSeeds, "ikParallel on");
		check(!ptpPol.softLimitSingularityPenalty, "soft limit penalty default off");
		RobotInstruction::IkSelectionPolicy softPol = ptpPol;
		std::unordered_map<std::string, std::string> softExt;
		softExt["context.ikSoftLimitPenalty"] = "1";
		RobotInstruction::applyIkPolicyExtensionOverrides(softExt, softPol);
		check(softPol.softLimitSingularityPenalty, "ikSoftLimitPenalty on");

		// golden：支编码约定（无整臂仿真）
		check(RobotInstruction::encodeIkBranchId(0, 0) == 0, "golden branch 0");
		check(RobotInstruction::encodeIkBranchId(1, 1) == 3, "golden branch 3");
		RobotInstruction::resetIkSolveMetrics();
		RobotInstruction::recordIkSolveSuccess(12);
		RobotInstruction::recordIkSolveFailure(RobotInstruction::IkFailKind::Budget, 30);
		const auto snap = RobotInstruction::ikSolveMetricsSnapshot();
		check(snap.success == 1 && snap.failBudget == 1, "metrics snapshot");
		check(RobotInstruction::formatIkSolveMetricsSummary().find("ikStats") != std::string::npos, "metrics fmt");

		RobotInstruction::IkSelectionPolicy adaptPol = ptpPol;
		adaptPol.raceAnalyticalAndNumerical = true;
		adaptPol.seedAttractionWeight = 0.02;
		adaptPol.requireMatchesClass = true;
		// 空路径视为非球形：关竞速/贴种子，且 2A 关硬 class
		RobotInstruction::adaptIkPolicyForRobot({}, adaptPol, nullptr);
		check(!adaptPol.raceAnalyticalAndNumerical, "adapt non-spherical race off");
		check(adaptPol.seedAttractionWeight == 0.0, "adapt non-spherical attract off");
		check(!adaptPol.requireMatchesClass, "adapt 2A no hard class");

		{
			const auto w0 = RobotTeachIk::CoordinatedIkCostWeights::defaults();
			check(w0.externalDeltaPriorWeight > 0.0, "coord cost default prior");
			std::unordered_map<std::string, std::string> wExt;
			wExt["context.externalDeltaPriorWeight"] = "0.12";
			wExt["context.adaptiveExternalDamping"] = "0";
			const auto w1 = RobotTeachIk::CoordinatedIkCostWeights::fromExtension(wExt);
			check(std::abs(w1.externalDeltaPriorWeight - 0.12) < 1e-9, "coord cost ext prior");
			check(!w1.adaptiveExternalDamping, "coord cost ext adaptive off");
		}

		const auto emptyMach = UrdfRobotLoader::resolveIkMachineProfile(QString());
		check(emptyMach.elbowJointIndex < 0 && emptyMach.wristJointIndex < 0, "empty urdf profile");
		check(UrdfRobotLoader::ikMachinePackageKeyFromUrdfPath(
				  QStringLiteral("D:/robots/abb_irb1100/urdf/irb1100.urdf")) == QStringLiteral("abb_irb1100"),
			  "packageKey from urdf dir");

		std::vector<double> qElbow(6, 0.0);
		qElbow[2] = 1.0;
		qElbow[1] = -1.0;
		const auto clsHint =
			RobotInstruction::classifyJointConfiguration(qElbow, names, &seed, -1, -1);
		check(clsHint.elbow == RobotInstruction::ElbowPosture::Up, "elbow hint joint_3");
		const auto clsOv =
			RobotInstruction::classifyJointConfiguration(qElbow, names, &seed, 1, -1);
		check(clsOv.elbow == RobotInstruction::ElbowPosture::Down, "elbow override index 1");
	}

	if (ok && !RobotInstruction::runIkGoldenSelfTest(failures))
	{
		ok = false;
	}

	if (ok)
	{
		RunLogger::info("[InstructionPropertyBindingSelfTest] PASS");
	}
	return ok;
}
