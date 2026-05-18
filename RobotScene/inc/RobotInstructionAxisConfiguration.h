#pragma once

#include "robot_scene_global.h"

#include <json.hpp>

#include <climits>
#include <string>
#include <vector>

namespace RobotInstruction
{

enum class ROBOT_SCENE_API ElbowPosture
{
	Auto = 0,
	Up,
	Down
};

enum class ROBOT_SCENE_API WristPosture
{
	Auto = 0,
	NoFlip,
	Flip
};

enum class ROBOT_SCENE_API ArmPosture
{
	Auto = 0,
	Front,
	Back
};

/// Signed full-revolution count relative to planning seed (INT_MIN = do not constrain).
constexpr int kMotionAxisTurnAuto = INT_MIN;

struct ROBOT_SCENE_API JointConfigurationClass
{
	ElbowPosture elbow = ElbowPosture::Auto;
	WristPosture wrist = WristPosture::Auto;
	ArmPosture arm = ArmPosture::Auto;
	int turnJ1 = kMotionAxisTurnAuto;
	int turnJ4 = kMotionAxisTurnAuto;
	int turnJ6 = kMotionAxisTurnAuto;
};

struct ROBOT_SCENE_API MotionAxisConfiguration
{
	std::string preset = "AUTO";
	ElbowPosture elbow = ElbowPosture::Auto;
	WristPosture wrist = WristPosture::Auto;
	ArmPosture arm = ArmPosture::Auto;
	int turnJ1 = kMotionAxisTurnAuto;
	int turnJ4 = kMotionAxisTurnAuto;
	int turnJ6 = kMotionAxisTurnAuto;

	/// Effective constraints after expanding preset (not CUSTOM/EXPLICIT).
	void resolveEffective(JointConfigurationClass& out) const;

	bool isFullyAuto() const;
	bool matchesClass(const JointConfigurationClass& observed) const;
};

ROBOT_SCENE_API std::vector<std::string> motionAxisConfigPresetTokens();
ROBOT_SCENE_API std::vector<std::string> elbowPostureTokens();
ROBOT_SCENE_API std::vector<std::string> wristPostureTokens();
ROBOT_SCENE_API std::vector<std::string> armPostureTokens();
ROBOT_SCENE_API std::vector<std::string> motionAxisTurnTokens();

ROBOT_SCENE_API bool jointTurnFromToken(const std::string& token, int& outTurn);
ROBOT_SCENE_API std::string jointTurnToToken(int turn);
ROBOT_SCENE_API bool isValidMotionAxisTurnToken(const std::string& token);

/// Integer revolutions of joint angle relative to reference: round((q - ref) / 2pi).
ROBOT_SCENE_API int classifyJointTurnRevolutions(double qRad, double refRad);

ROBOT_SCENE_API bool elbowPostureFromToken(const std::string& token, ElbowPosture& out);
ROBOT_SCENE_API bool wristPostureFromToken(const std::string& token, WristPosture& out);
ROBOT_SCENE_API bool armPostureFromToken(const std::string& token, ArmPosture& out);
ROBOT_SCENE_API std::string elbowPostureToToken(ElbowPosture v);
ROBOT_SCENE_API std::string wristPostureToToken(WristPosture v);
ROBOT_SCENE_API std::string armPostureToToken(ArmPosture v);

ROBOT_SCENE_API bool motionAxisConfigPresetFromToken(const std::string& token, std::string& outPreset);
ROBOT_SCENE_API void applyPresetToConfiguration(const std::string& preset, MotionAxisConfiguration& cfg);

ROBOT_SCENE_API bool isValidMotionAxisConfigPreset(const std::string& token);
ROBOT_SCENE_API bool isValidElbowPostureToken(const std::string& token);
ROBOT_SCENE_API bool isValidWristPostureToken(const std::string& token);
ROBOT_SCENE_API bool isValidArmPostureToken(const std::string& token);

ROBOT_SCENE_API MotionAxisConfiguration motionAxisConfigurationFromJson(const nlohmann::json& j);
ROBOT_SCENE_API void writeMotionAxisConfigurationToJson(const MotionAxisConfiguration& cfg, nlohmann::json& j);

/// Legacy single-string axisConfig ("AUTO", preset tokens).
ROBOT_SCENE_API MotionAxisConfiguration motionAxisConfigurationFromLegacyString(const std::string& legacy);

/// Short summary for tree UI (empty when AUTO).
ROBOT_SCENE_API std::string formatMotionAxisConfigurationSummary(const MotionAxisConfiguration& cfg, bool chinese);

ROBOT_SCENE_API bool motionAxisConfigurationRequiresConstraint(const MotionAxisConfiguration& cfg);

ROBOT_SCENE_API JointConfigurationClass classifyJointConfiguration(
	const std::vector<double>& qRad,
	const std::vector<std::string>& jointNames,
	const std::vector<double>* referenceQRad = nullptr);

/// Pick the most specific preset token matching an observed posture (may return "AUTO").
ROBOT_SCENE_API std::string suggestMotionAxisPresetToken(const JointConfigurationClass& observed);

} // namespace RobotInstruction
