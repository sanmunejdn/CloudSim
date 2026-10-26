/// @file RobotInstructionAxisConfiguration.cpp
/// @brief 指令轴配置

#include "RobotInstructionAxisConfiguration.h"

#include "SphericalWristAnalyticalIk.h"
#include "UrdfIkMachineProfile.h"

#include <QString>

#include <algorithm>
#include <cctype>
#include <cmath>

namespace RobotInstruction
{
namespace
{
std::string upperAscii(std::string s)
{
	for (char& c : s)
	{
		c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
	}
	return s;
}

int findJointIndexByHint(const std::vector<std::string>& jointNames, const char* hint, int fallbackIndex)
{
	if (hint)
	{
		const std::string needle(hint);
		for (size_t i = 0; i < jointNames.size(); ++i)
		{
			std::string lower = jointNames[i];
			std::transform(lower.begin(), lower.end(), lower.begin(),
						   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
			if (lower.find(needle) != std::string::npos)
			{
				return static_cast<int>(i);
			}
		}
	}
	if (fallbackIndex >= 0 && fallbackIndex < static_cast<int>(jointNames.size()))
	{
		return fallbackIndex;
	}
	return -1;
}
} // namespace

std::vector<std::string> motionAxisConfigPresetTokens()
{
	return {
		"AUTO",
		"ELBOW_UP",
		"ELBOW_DOWN",
		"WRIST_FLIP",
		"WRIST_NO_FLIP",
		"ELBOW_UP_WRIST_NO_FLIP",
		"ELBOW_UP_WRIST_FLIP",
		"ELBOW_DOWN_WRIST_NO_FLIP",
		"ELBOW_DOWN_WRIST_FLIP",
		"CUSTOM",
	};
}

std::vector<std::string> elbowPostureTokens()
{
	return {"AUTO", "UP", "DOWN"};
}

std::vector<std::string> wristPostureTokens()
{
	return {"AUTO", "NO_FLIP", "FLIP"};
}

std::vector<std::string> armPostureTokens()
{
	return {"AUTO", "FRONT", "BACK"};
}

std::vector<std::string> motionAxisTurnTokens()
{
	return {"AUTO", "-2", "-1", "0", "1", "2", "3"};
}

bool jointTurnFromToken(const std::string& token, int& outTurn)
{
	const std::string u = upperAscii(token);
	if (u == "AUTO")
	{
		outTurn = kMotionAxisTurnAuto;
		return true;
	}
	try
	{
		outTurn = std::stoi(u);
		return true;
	}
	catch (...)
	{
		return false;
	}
}

std::string jointTurnToToken(const int turn)
{
	if (turn == kMotionAxisTurnAuto)
	{
		return "AUTO";
	}
	return std::to_string(turn);
}

bool isValidMotionAxisTurnToken(const std::string& token)
{
	int v = 0;
	return jointTurnFromToken(token, v);
}

int classifyJointTurnRevolutions(const double qRad, const double refRad)
{
	constexpr double kTwoPi = 6.283185307179586;
	return static_cast<int>(std::llround((qRad - refRad) / kTwoPi));
}

bool elbowPostureFromToken(const std::string& token, ElbowPosture& out)
{
	const std::string u = upperAscii(token);
	if (u == "AUTO")
	{
		out = ElbowPosture::Auto;
		return true;
	}
	if (u == "UP")
	{
		out = ElbowPosture::Up;
		return true;
	}
	if (u == "DOWN")
	{
		out = ElbowPosture::Down;
		return true;
	}
	return false;
}

bool wristPostureFromToken(const std::string& token, WristPosture& out)
{
	const std::string u = upperAscii(token);
	if (u == "AUTO")
	{
		out = WristPosture::Auto;
		return true;
	}
	if (u == "NO_FLIP" || u == "NOFLIP")
	{
		out = WristPosture::NoFlip;
		return true;
	}
	if (u == "FLIP")
	{
		out = WristPosture::Flip;
		return true;
	}
	return false;
}

bool armPostureFromToken(const std::string& token, ArmPosture& out)
{
	const std::string u = upperAscii(token);
	if (u == "AUTO")
	{
		out = ArmPosture::Auto;
		return true;
	}
	if (u == "FRONT")
	{
		out = ArmPosture::Front;
		return true;
	}
	if (u == "BACK")
	{
		out = ArmPosture::Back;
		return true;
	}
	return false;
}

std::string elbowPostureToToken(const ElbowPosture v)
{
	switch (v)
	{
	case ElbowPosture::Up:
		return "UP";
	case ElbowPosture::Down:
		return "DOWN";
	default:
		return "AUTO";
	}
}

std::string wristPostureToToken(const WristPosture v)
{
	switch (v)
	{
	case WristPosture::Flip:
		return "FLIP";
	case WristPosture::NoFlip:
		return "NO_FLIP";
	default:
		return "AUTO";
	}
}

std::string armPostureToToken(const ArmPosture v)
{
	switch (v)
	{
	case ArmPosture::Front:
		return "FRONT";
	case ArmPosture::Back:
		return "BACK";
	default:
		return "AUTO";
	}
}

bool motionAxisConfigPresetFromToken(const std::string& token, std::string& outPreset)
{
	const std::string u = upperAscii(token);
	for (const std::string& p : motionAxisConfigPresetTokens())
	{
		if (p == u)
		{
			outPreset = p;
			return true;
		}
	}
	return false;
}

void applyPresetToConfiguration(const std::string& preset, MotionAxisConfiguration& cfg)
{
	cfg.preset = upperAscii(preset);
	cfg.elbow = ElbowPosture::Auto;
	cfg.wrist = WristPosture::Auto;
	cfg.arm = ArmPosture::Auto;
	if (cfg.preset == "AUTO")
	{
		return;
	}
	if (cfg.preset == "CUSTOM" || cfg.preset == "EXPLICIT")
	{
		return;
	}
	if (cfg.preset == "ELBOW_UP")
	{
		cfg.elbow = ElbowPosture::Up;
		return;
	}
	if (cfg.preset == "ELBOW_DOWN")
	{
		cfg.elbow = ElbowPosture::Down;
		return;
	}
	if (cfg.preset == "WRIST_FLIP")
	{
		cfg.wrist = WristPosture::Flip;
		return;
	}
	if (cfg.preset == "WRIST_NO_FLIP")
	{
		cfg.wrist = WristPosture::NoFlip;
		return;
	}
	if (cfg.preset == "ELBOW_UP_WRIST_NO_FLIP")
	{
		cfg.elbow = ElbowPosture::Up;
		cfg.wrist = WristPosture::NoFlip;
		return;
	}
	if (cfg.preset == "ELBOW_UP_WRIST_FLIP")
	{
		cfg.elbow = ElbowPosture::Up;
		cfg.wrist = WristPosture::Flip;
		return;
	}
	if (cfg.preset == "ELBOW_DOWN_WRIST_NO_FLIP")
	{
		cfg.elbow = ElbowPosture::Down;
		cfg.wrist = WristPosture::NoFlip;
		return;
	}
	if (cfg.preset == "ELBOW_DOWN_WRIST_FLIP")
	{
		cfg.elbow = ElbowPosture::Down;
		cfg.wrist = WristPosture::Flip;
	}
}

bool isValidMotionAxisConfigPreset(const std::string& token)
{
	std::string p;
	return motionAxisConfigPresetFromToken(token, p);
}

bool isValidElbowPostureToken(const std::string& token)
{
	ElbowPosture v{};
	return elbowPostureFromToken(token, v);
}

bool isValidWristPostureToken(const std::string& token)
{
	WristPosture v{};
	return wristPostureFromToken(token, v);
}

bool isValidArmPostureToken(const std::string& token)
{
	ArmPosture v{};
	return armPostureFromToken(token, v);
}

void MotionAxisConfiguration::resolveEffective(JointConfigurationClass& out) const
{
	MotionAxisConfiguration expanded = *this;
	if (expanded.preset != "CUSTOM" && expanded.preset != "EXPLICIT")
	{
		applyPresetToConfiguration(expanded.preset, expanded);
	}
	out.elbow = expanded.elbow;
	out.wrist = expanded.wrist;
	out.arm = expanded.arm;
	out.turnJ1 = turnJ1;
	out.turnJ4 = turnJ4;
	out.turnJ6 = turnJ6;
}

bool MotionAxisConfiguration::isFullyAuto() const
{
	if (turnJ1 != kMotionAxisTurnAuto || turnJ4 != kMotionAxisTurnAuto || turnJ6 != kMotionAxisTurnAuto)
	{
		return false;
	}
	JointConfigurationClass eff{};
	resolveEffective(eff);
	return eff.elbow == ElbowPosture::Auto && eff.wrist == WristPosture::Auto && eff.arm == ArmPosture::Auto;
}

bool MotionAxisConfiguration::matchesClass(const JointConfigurationClass& observed) const
{
	JointConfigurationClass want{};
	resolveEffective(want);
	if (want.elbow != ElbowPosture::Auto && want.elbow != observed.elbow)
	{
		return false;
	}
	if (want.wrist != WristPosture::Auto && want.wrist != observed.wrist)
	{
		return false;
	}
	if (want.arm != ArmPosture::Auto && want.arm != observed.arm)
	{
		return false;
	}
	if (want.turnJ1 != kMotionAxisTurnAuto && want.turnJ1 != observed.turnJ1)
	{
		return false;
	}
	if (want.turnJ4 != kMotionAxisTurnAuto && want.turnJ4 != observed.turnJ4)
	{
		return false;
	}
	if (want.turnJ6 != kMotionAxisTurnAuto && want.turnJ6 != observed.turnJ6)
	{
		return false;
	}
	return true;
}

MotionAxisConfiguration motionAxisConfigurationFromLegacyString(const std::string& legacy)
{
	MotionAxisConfiguration cfg;
	std::string preset;
	if (motionAxisConfigPresetFromToken(legacy, preset))
	{
		applyPresetToConfiguration(preset, cfg);
		cfg.preset = preset;
	}
	else
	{
		cfg.preset = "AUTO";
	}
	return cfg;
}

MotionAxisConfiguration motionAxisConfigurationFromJson(const nlohmann::json& j)
{
	MotionAxisConfiguration cfg;
	if (j.is_object() && j.contains("preset"))
	{
		cfg.preset = upperAscii(j.value("preset", std::string("AUTO")));
		ElbowPosture e{};
		WristPosture w{};
		ArmPosture a{};
		if (elbowPostureFromToken(j.value("elbow", std::string("AUTO")), e))
		{
			cfg.elbow = e;
		}
		if (wristPostureFromToken(j.value("wrist", std::string("AUTO")), w))
		{
			cfg.wrist = w;
		}
		if (armPostureFromToken(j.value("arm", std::string("AUTO")), a))
		{
			cfg.arm = a;
		}
		if (j.contains("turns") && j["turns"].is_object())
		{
			const auto& t = j["turns"];
			if (t.contains("j1"))
			{
				cfg.turnJ1 = t["j1"].get<int>();
			}
			if (t.contains("j4"))
			{
				cfg.turnJ4 = t["j4"].get<int>();
			}
			if (t.contains("j6"))
			{
				cfg.turnJ6 = t["j6"].get<int>();
			}
		}
		if (cfg.preset != "CUSTOM" && cfg.preset != "EXPLICIT")
		{
			applyPresetToConfiguration(cfg.preset, cfg);
		}
		return cfg;
	}
	return motionAxisConfigurationFromLegacyString(j.is_string() ? j.get<std::string>() : std::string("AUTO"));
}

void writeMotionAxisConfigurationToJson(const MotionAxisConfiguration& cfg, nlohmann::json& j)
{
	j["preset"] = cfg.preset;
	j["elbow"] = elbowPostureToToken(cfg.elbow);
	j["wrist"] = wristPostureToToken(cfg.wrist);
	j["arm"] = armPostureToToken(cfg.arm);
	if (cfg.turnJ1 != INT_MIN || cfg.turnJ4 != INT_MIN || cfg.turnJ6 != INT_MIN)
	{
		nlohmann::json turns = nlohmann::json::object();
		if (cfg.turnJ1 != INT_MIN)
		{
			turns["j1"] = cfg.turnJ1;
		}
		if (cfg.turnJ4 != INT_MIN)
		{
			turns["j4"] = cfg.turnJ4;
		}
		if (cfg.turnJ6 != INT_MIN)
		{
			turns["j6"] = cfg.turnJ6;
		}
		j["turns"] = turns;
	}
}

std::string formatMotionAxisConfigurationSummary(const MotionAxisConfiguration& cfg, const bool chinese)
{
	if (cfg.isFullyAuto())
	{
		return {};
	}
	JointConfigurationClass eff{};
	cfg.resolveEffective(eff);
	std::string out;
	auto appendPart = [&](const std::string& en, const std::string& zh)
	{
		if (!out.empty())
		{
			out += chinese ? "/" : "/";
		}
		out += chinese ? zh : en;
	};
	if (eff.elbow == ElbowPosture::Up)
	{
		appendPart("Elbow up", "肘上");
	}
	else if (eff.elbow == ElbowPosture::Down)
	{
		appendPart("Elbow down", "肘下");
	}
	if (eff.wrist == WristPosture::Flip)
	{
		appendPart("Wrist flip", "腕翻");
	}
	else if (eff.wrist == WristPosture::NoFlip)
	{
		appendPart("Wrist no-flip", "腕不翻");
	}
	if (eff.arm == ArmPosture::Front)
	{
		appendPart("Arm front", "臂前");
	}
	else if (eff.arm == ArmPosture::Back)
	{
		appendPart("Arm back", "臂后");
	}
	if (cfg.turnJ1 != kMotionAxisTurnAuto)
	{
		appendPart("J1 turn " + std::to_string(cfg.turnJ1), "J1转" + std::to_string(cfg.turnJ1));
	}
	if (cfg.turnJ4 != kMotionAxisTurnAuto)
	{
		appendPart("J4 turn " + std::to_string(cfg.turnJ4), "J4转" + std::to_string(cfg.turnJ4));
	}
	if (cfg.turnJ6 != kMotionAxisTurnAuto)
	{
		appendPart("J6 turn " + std::to_string(cfg.turnJ6), "J6转" + std::to_string(cfg.turnJ6));
	}
	return out;
}

bool motionAxisConfigurationRequiresConstraint(const MotionAxisConfiguration& cfg)
{
	return !cfg.isFullyAuto();
}

std::string suggestMotionAxisPresetToken(const JointConfigurationClass& c)
{
	if (c.elbow == ElbowPosture::Up && c.wrist == WristPosture::NoFlip)
	{
		return "ELBOW_UP_WRIST_NO_FLIP";
	}
	if (c.elbow == ElbowPosture::Up && c.wrist == WristPosture::Flip)
	{
		return "ELBOW_UP_WRIST_FLIP";
	}
	if (c.elbow == ElbowPosture::Down && c.wrist == WristPosture::NoFlip)
	{
		return "ELBOW_DOWN_WRIST_NO_FLIP";
	}
	if (c.elbow == ElbowPosture::Down && c.wrist == WristPosture::Flip)
	{
		return "ELBOW_DOWN_WRIST_FLIP";
	}
	if (c.elbow == ElbowPosture::Up)
	{
		return "ELBOW_UP";
	}
	if (c.elbow == ElbowPosture::Down)
	{
		return "ELBOW_DOWN";
	}
	if (c.wrist == WristPosture::Flip)
	{
		return "WRIST_FLIP";
	}
	if (c.wrist == WristPosture::NoFlip)
	{
		return "WRIST_NO_FLIP";
	}
	return "AUTO";
}

JointConfigurationClass classifyJointConfiguration(const std::vector<double>& qRad,
												   const std::vector<std::string>& jointNames,
												   const std::vector<double>* referenceQRad,
												   const int elbowJointIndexOverride,
												   const int wristJointIndexOverride)
{
	JointConfigurationClass c;
	if (qRad.empty())
	{
		return c;
	}
	const int n = static_cast<int>(qRad.size());
	int elbowIdx = elbowJointIndexOverride;
	if (elbowIdx < 0 || elbowIdx >= n)
	{
		// 先名字再 joint_3，与工业 6 轴肘索引惯例对齐
		elbowIdx = findJointIndexByHint(jointNames, "elbow", -1);
		if (elbowIdx < 0)
		{
			elbowIdx = findJointIndexByHint(jointNames, "joint_3", 2);
		}
	}
	int wristIdx = wristJointIndexOverride;
	if (wristIdx < 0 || wristIdx >= n)
	{
		wristIdx = findJointIndexByHint(jointNames, "wrist", -1);
		if (wristIdx < 0)
		{
			wristIdx = findJointIndexByHint(jointNames, "joint_5", 4);
		}
	}
	const int j1Idx = findJointIndexByHint(jointNames, nullptr, 0);
	const int j4Idx = findJointIndexByHint(jointNames, "joint_4", 3);
	const int j6Idx = findJointIndexByHint(jointNames, "joint_6", 5);
	const double refJ1 = (referenceQRad && j1Idx >= 0 && j1Idx < static_cast<int>(referenceQRad->size()))
							 ? (*referenceQRad)[static_cast<size_t>(j1Idx)]
							 : 0.0;
	const double refJ4 = (referenceQRad && j4Idx >= 0 && j4Idx < static_cast<int>(referenceQRad->size()))
							 ? (*referenceQRad)[static_cast<size_t>(j4Idx)]
							 : 0.0;
	const double refJ6 = (referenceQRad && j6Idx >= 0 && j6Idx < static_cast<int>(referenceQRad->size()))
							 ? (*referenceQRad)[static_cast<size_t>(j6Idx)]
							 : 0.0;

	if (elbowIdx >= 0 && elbowIdx < static_cast<int>(qRad.size()))
	{
		c.elbow = qRad[static_cast<size_t>(elbowIdx)] >= 0.0 ? ElbowPosture::Up : ElbowPosture::Down;
	}
	if (wristIdx >= 0 && wristIdx < static_cast<int>(qRad.size()))
	{
		if (referenceQRad && referenceQRad->size() == qRad.size())
		{
			double delta = qRad[static_cast<size_t>(wristIdx)] - (*referenceQRad)[static_cast<size_t>(wristIdx)];
			while (delta > 3.14159265358979323846)
			{
				delta -= 6.283185307179586;
			}
			while (delta < -3.14159265358979323846)
			{
				delta += 6.283185307179586;
			}
			c.wrist = (std::abs(delta) > 1.5707963267948966) ? WristPosture::Flip : WristPosture::NoFlip;
		}
		else
		{
			const double w = qRad[static_cast<size_t>(wristIdx)];
			c.wrist = (std::abs(w) > 1.5707963267948966) ? WristPosture::Flip : WristPosture::NoFlip;
		}
	}
	if (j1Idx >= 0 && j1Idx < static_cast<int>(qRad.size()))
	{
		const double j1 = qRad[static_cast<size_t>(j1Idx)];
		c.arm = (std::cos(j1) >= 0.0) ? ArmPosture::Front : ArmPosture::Back;
		c.turnJ1 = classifyJointTurnRevolutions(j1, refJ1);
	}
	if (j4Idx >= 0 && j4Idx < static_cast<int>(qRad.size()))
	{
		c.turnJ4 = classifyJointTurnRevolutions(qRad[static_cast<size_t>(j4Idx)], refJ4);
	}
	if (j6Idx >= 0 && j6Idx < static_cast<int>(qRad.size()))
	{
		c.turnJ6 = classifyJointTurnRevolutions(qRad[static_cast<size_t>(j6Idx)], refJ6);
	}
	return c;
}

bool sameJointConfigurationClass(const JointConfigurationClass& a, const JointConfigurationClass& b)
{
	return a.elbow == b.elbow && a.wrist == b.wrist && a.arm == b.arm && a.turnJ1 == b.turnJ1 && a.turnJ4 == b.turnJ4 &&
		   a.turnJ6 == b.turnJ6;
}

IkSelectionPolicy makePtpIkPolicy(const MotionAxisConfiguration& cfg)
{
	IkSelectionPolicy p;
	p.requireWristContinuity = false;
	const bool constrain = motionAxisConfigurationRequiresConstraint(cfg);
	p.requireMatchesClass = constrain;
	p.mode = IkSolveMode::Search;
	p.rankMode = IkRankMode::Distance;
	p.maxSeeds = constrain ? 32 : 24;
	// 非球形腕每种子走完整 DLS，预算过紧会误失败
	p.budgetMs = constrain ? 250 : 200;
	p.raceNumericalFallback = true;
	// 默认串行：解析失败再数值；竞速可选扩展打开
	p.raceAnalyticalAndNumerical = false;
	p.allowCartDegrade = true;
	p.manipOnDistanceTie = true;
	p.seedAttractionWeight = 0.0;
	return p;
}

IkSelectionPolicy makeCartesianPathIkPolicy(const MotionAxisConfiguration& cfg)
{
	IkSelectionPolicy p;
	p.requireWristContinuity = true;
	p.requireMatchesClass = motionAxisConfigurationRequiresConstraint(cfg);
	p.mode = IkSolveMode::Search;
	p.rankMode = IkRankMode::Distance;
	p.maxSeeds = 32;
	p.budgetMs = 200;
	p.raceNumericalFallback = true;
	p.raceAnalyticalAndNumerical = false;
	p.allowCartDegrade = true;
	p.manipOnDistanceTie = true;
	p.seedAttractionWeight = 0.0;
	return p;
}

IkFailKind classifyIkFailMessage(const std::string& message)
{
	if (message.empty())
	{
		return IkFailKind::Other;
	}
	std::string lower = message;
	std::transform(lower.begin(), lower.end(), lower.begin(),
				   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	auto has = [&](const char* needle) { return lower.find(needle) != std::string::npos; };
	if (has("spherical-wrist") || has("topology") || has("拓扑") || (has("解析") && has("失败")))
	{
		return IkFailKind::Topology;
	}
	if (has("budget") || has("预算") || has("耗尽"))
	{
		return IkFailKind::Budget;
	}
	if (has("collision") || has("碰撞"))
	{
		return IkFailKind::Collision;
	}
	if (has("轴配置") || has("肘/腕") || has("不匹配"))
	{
		return IkFailKind::AxisClass;
	}
	if (has("腕部跳变") || has("腕连续") || (has("wrist") && (has("连续") || has("跳"))))
	{
		return IkFailKind::Wrist;
	}
	if (has("缺少ik种子") || has("无可用ik种子") || (has("seed") && (has("missing") || has("empty"))))
	{
		return IkFailKind::NoSeed;
	}
	if (has("residual") || has("残差") || has("未收敛") || has("ik未收敛"))
	{
		return IkFailKind::Residual;
	}
	return IkFailKind::Other;
}

std::string formatIkFailKindTag(const IkFailKind kind)
{
	switch (kind)
	{
	case IkFailKind::None:
		return {};
	case IkFailKind::Topology:
		return " [fail=Topology]";
	case IkFailKind::Residual:
		return " [fail=Residual]";
	case IkFailKind::Budget:
		return " [fail=Budget]";
	case IkFailKind::Wrist:
		return " [fail=Wrist]";
	case IkFailKind::AxisClass:
		return " [fail=AxisClass]";
	case IkFailKind::Collision:
		return " [fail=Collision]";
	case IkFailKind::NoSeed:
		return " [fail=NoSeed]";
	case IkFailKind::Other:
	default:
		return " [fail=Other]";
	}
}

std::string formatIkFailKindZh(const IkFailKind kind)
{
	switch (kind)
	{
	case IkFailKind::None:
		return {};
	case IkFailKind::Topology:
		return "机型拓扑不匹配";
	case IkFailKind::Residual:
		return "IK残差未收敛";
	case IkFailKind::Budget:
		return "IK预算耗尽；可设 context.ikProfile=run 或加大 ikBudgetMs";
	case IkFailKind::Wrist:
		return "腕构型跳变过大";
	case IkFailKind::AxisClass:
		return "轴配置无匹配解";
	case IkFailKind::Collision:
		return "候选碰撞";
	case IkFailKind::NoSeed:
		return "无可用IK种子";
	case IkFailKind::Other:
	default:
		return "IK失败";
	}
}

void parseIkJointIndexOverrides(const std::unordered_map<std::string, std::string>& ext, int& elbowJointIndex,
								int& wristJointIndex)
{
	elbowJointIndex = -1;
	wristJointIndex = -1;
	const auto itElbow = ext.find("context.elbowJointIndex");
	if (itElbow != ext.end())
	{
		try
		{
			const int v = std::stoi(itElbow->second);
			if (v >= 0)
			{
				elbowJointIndex = v;
			}
		}
		catch (...)
		{
		}
	}
	const auto itWrist = ext.find("context.wristJointIndex");
	if (itWrist != ext.end())
	{
		try
		{
			const int v = std::stoi(itWrist->second);
			if (v >= 0)
			{
				wristJointIndex = v;
			}
		}
		catch (...)
		{
		}
	}
}

void applyIkPolicyExtensionOverrides(const std::unordered_map<std::string, std::string>& ext, IkSelectionPolicy& policy)
{
	const auto itProfile = ext.find("context.ikProfile");
	if (itProfile != ext.end())
	{
		const std::string& p = itProfile->second;
		if (p == "drag")
		{
			policy.budgetMs = 80;
			policy.maxSeeds = 8;
		}
		else if (p == "preview")
		{
			policy.budgetMs = 120;
			policy.maxSeeds = 16;
		}
		else if (p == "run")
		{
			policy.budgetMs = 250;
			policy.maxSeeds = 32;
		}
	}
	const auto itRank = ext.find("context.ikRankMode");
	if (itRank != ext.end())
	{
		if (itRank->second == "Manip" || itRank->second == "manip")
		{
			policy.rankMode = IkRankMode::Manip;
		}
		else if (itRank->second == "Speed" || itRank->second == "speed")
		{
			policy.rankMode = IkRankMode::Speed;
		}
		else if (itRank->second == "Distance" || itRank->second == "distance")
		{
			policy.rankMode = IkRankMode::Distance;
		}
	}
	const auto itBudget = ext.find("context.ikBudgetMs");
	if (itBudget != ext.end())
	{
		try
		{
			const int ms = std::stoi(itBudget->second);
			if (ms >= 0)
			{
				policy.budgetMs = ms;
			}
		}
		catch (...)
		{
		}
	}
	const auto itDegrade = ext.find("context.allowCartDegrade");
	if (itDegrade != ext.end())
	{
		policy.allowCartDegrade = !(itDegrade->second == "0" || itDegrade->second == "false");
	}
	const auto itLite = ext.find("context.playbackPlanLite");
	if (itLite != ext.end() && itLite->second == "1")
	{
		policy.budgetMs = std::max(20, policy.budgetMs / 2);
		policy.maxSeeds = std::max(4, policy.maxSeeds / 2);
	}
}

namespace
{
thread_local std::string t_ikEscalateSummaryTag;
}

void clearIkEscalateSummaryTag()
{
	t_ikEscalateSummaryTag.clear();
}

void setIkEscalateSummaryTag(const std::string& tag)
{
	t_ikEscalateSummaryTag = tag;
}

std::string consumeIkEscalateSummaryTag()
{
	std::string out = t_ikEscalateSummaryTag;
	t_ikEscalateSummaryTag.clear();
	return out;
}

void adaptIkPolicyForRobot(const std::string& urdfPath, IkSelectionPolicy& policy,
						   const std::unordered_map<std::string, std::string>* ext)
{
	bool spherical = false;
	if (!urdfPath.empty())
	{
		const UrdfRobotLoader::UrdfIkMachineProfile mach =
			UrdfRobotLoader::resolveIkMachineProfile(QString::fromStdString(urdfPath));
		if (mach.forceSphericalWrist == UrdfRobotLoader::SphericalWristForce::No)
		{
			spherical = false;
		}
		else if (mach.forceSphericalWrist == UrdfRobotLoader::SphericalWristForce::Yes)
		{
			spherical = true;
		}
		else
		{
			spherical = UrdfRobotLoader::SphericalWristAnalyticalIk::isEligible(QString::fromStdString(urdfPath));
		}
	}
	if (!spherical)
	{
		policy.raceAnalyticalAndNumerical = false;
		policy.seedAttractionWeight = 0.0;
		return;
	}
	if (!ext)
	{
		return;
	}
	const auto itRace = ext->find("context.ikRace");
	if (itRace != ext->end() && (itRace->second == "1" || itRace->second == "true"))
	{
		policy.raceAnalyticalAndNumerical = true;
	}
	const auto itAttract = ext->find("context.ikSeedAttract");
	if (itAttract != ext->end())
	{
		try
		{
			const double w = std::stod(itAttract->second);
			if (w >= 0.0)
			{
				policy.seedAttractionWeight = w;
			}
		}
		catch (...)
		{
		}
	}
}

std::string formatIkPolicySummaryTags(const IkSelectionPolicy& policy, const bool denseRetry, const bool numRace,
									  const bool cartDegraded)
{
	std::string t;
	t += (policy.mode == IkSolveMode::Search) ? " [ikMode=Search]" : " [ikMode=NearSeed]";
	if (policy.rankMode == IkRankMode::Manip)
	{
		t += " [rank=Manip]";
	}
	else if (policy.rankMode == IkRankMode::Speed)
	{
		t += " [rank=Speed]";
	}
	else
	{
		t += " [rank=Distance]";
	}
	if (denseRetry)
	{
		t += " [denseRetry]";
	}
	if (numRace)
	{
		t += " [numRace]";
	}
	if (cartDegraded)
	{
		t += " [cartDegraded]";
	}
	return t;
}

bool shouldSkipShortestAngleFold(const std::vector<double>& seedQ, const std::vector<double>& targetQ,
								 const std::vector<std::string>& jointNames)
{
	if (seedQ.empty() || seedQ.size() != targetQ.size() || jointNames.size() != seedQ.size())
	{
		return false;
	}
	const JointConfigurationClass seedCls = classifyJointConfiguration(seedQ, jointNames, &seedQ);
	const JointConfigurationClass tgtCls = classifyJointConfiguration(targetQ, jointNames, &seedQ);
	return !sameJointConfigurationClass(seedCls, tgtCls);
}

double wristConfigurationJumpRad(const std::vector<double>& qRad, const std::vector<double>& seedRad,
								 const std::vector<std::string>& jointNames)
{
	if (qRad.empty() || qRad.size() != seedRad.size())
	{
		return 1e30;
	}
	const int j4Idx = findJointIndexByHint(jointNames, "joint_4", 3);
	const int wristIdx = findJointIndexByHint(jointNames, "wrist", 4);
	const int j6Idx = findJointIndexByHint(jointNames, "joint_6", 5);
	constexpr double kPi = 3.14159265358979323846;
	constexpr double kTwoPi = 6.283185307179586;
	double maxAbs = 0.0;
	auto accum = [&](const int idx)
	{
		if (idx < 0 || idx >= static_cast<int>(qRad.size()))
		{
			return;
		}
		// 最短角差：fold/转数同构不当作翻腕
		double d = qRad[static_cast<size_t>(idx)] - seedRad[static_cast<size_t>(idx)];
		while (d > kPi)
		{
			d -= kTwoPi;
		}
		while (d < -kPi)
		{
			d += kTwoPi;
		}
		maxAbs = std::max(maxAbs, std::abs(d));
	};
	accum(j4Idx);
	accum(wristIdx);
	accum(j6Idx);
	return maxAbs;
}

} // namespace RobotInstruction
