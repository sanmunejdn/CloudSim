#ifndef ROBOTSCENE_ROBOTINSTRUCTIONAXISCONFIGURATION_H
#define ROBOTSCENE_ROBOTINSTRUCTIONAXISCONFIGURATION_H

/// @file RobotInstructionAxisConfiguration.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 相对规划种子的整圈数；INT_MIN 表示不约束

#include "robot_scene_global.h"

#include <climits>
#include <string>
#include <unordered_map>
#include <vector>

#include <json.hpp>

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

/// 相对规划种子的整圈数；INT_MIN 表示不约束
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

	/// 展开 preset 后的有效约束（非 CUSTOM/EXPLICIT）
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

/// round((q - ref) / 2π) 整圈分类
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

/// 旧单字符串 axisConfig（"AUTO" 等 preset）
ROBOT_SCENE_API MotionAxisConfiguration motionAxisConfigurationFromLegacyString(const std::string& legacy);

/// 树 UI 短摘要（AUTO 时为空）
ROBOT_SCENE_API std::string formatMotionAxisConfigurationSummary(const MotionAxisConfiguration& cfg, bool chinese);

ROBOT_SCENE_API bool motionAxisConfigurationRequiresConstraint(const MotionAxisConfiguration& cfg);

/// elbow/wristJointIndexOverride≥0 时覆盖名字 hint（context.elbowJointIndex / wristJointIndex）
ROBOT_SCENE_API JointConfigurationClass classifyJointConfiguration(const std::vector<double>& qRad,
																   const std::vector<std::string>& jointNames,
																   const std::vector<double>* referenceQRad = nullptr,
																   int elbowJointIndexOverride = -1,
																   int wristJointIndexOverride = -1);

/// 肘/腕/臂/转数类是否一致（用于是否跳过最短角折圈）
ROBOT_SCENE_API bool sameJointConfigurationClass(const JointConfigurationClass& a, const JointConfigurationClass& b);

/// NearSeed：贴滚动种子；Search：多种子搜支（MoveIt/TRAC-IK 式）
enum class ROBOT_SCENE_API IkSolveMode
{
	NearSeed = 0,
	Search
};

/// Distance=贴种子；Speed=先收敛先取；Manip=可操作度优先（远离奇异）
enum class ROBOT_SCENE_API IkRankMode
{
	Distance = 0,
	Speed,
	Manip
};

/// PTP 腕软 / LINE 腕硬 等由工厂生成，避免调用点散落 bool
struct ROBOT_SCENE_API IkSelectionPolicy
{
	IkSolveMode mode = IkSolveMode::NearSeed;
	bool requireWristContinuity = true;
	bool requireMatchesClass = false;
	IkRankMode rankMode = IkRankMode::Distance;
	/// 0=不截断种子数
	int maxSeeds = 0;
	/// 0=不限时（ms）；Search 默认有界防 Preview 卡顿
	int budgetMs = 0;
	/// 首轮失败后再跑纯数值 DLS
	bool raceNumericalFallback = true;
	/// 同种子上解析与数值并行择近（默认关；球形腕可扩展打开）
	bool raceAnalyticalAndNumerical = false;
	/// LINE 笛卡尔失败是否允许退关节插值（默认 false；需扩展键显式打开）
	bool allowCartDegrade = false;
	/// Distance 平局时再算可操作度（省雅可比）
	bool manipOnDistanceTie = true;
	/// DLS 贴种子弹簧权重（0=关闭；默认关防非球形腕误用）
	double seedAttractionWeight = 0.0;
	/// Search 多种子有界并行（扩展 context.ikParallel=1；默认关；外轴激活时强制串行）
	bool allowParallelSeeds = false;
	/// 距离项叠加 URDF 限位端软罚（扩展 context.ikSoftLimitPenalty=1；默认关）
	bool softLimitSingularityPenalty = false;
};

/// IK 失败分类（summary [fail=…] / UI 短句）
enum class ROBOT_SCENE_API IkFailKind
{
	None = 0,
	Topology,
	Residual,
	Budget,
	Wrist,
	AxisClass,
	Collision,
	NoSeed,
	Other
};

/// 兼容旧摘要；主路径应直出 IkFailKind
ROBOT_SCENE_API IkFailKind classifyIkFailMessage(const std::string& message);
ROBOT_SCENE_API std::string formatIkFailKindTag(IkFailKind kind);
ROBOT_SCENE_API std::string formatIkFailKindZh(IkFailKind kind);

ROBOT_SCENE_API IkSelectionPolicy makePtpIkPolicy(const MotionAxisConfiguration& cfg);
/// 笛卡尔路径终点用；样点可将 mode 改为 NearSeed
ROBOT_SCENE_API IkSelectionPolicy makeCartesianPathIkPolicy(const MotionAxisConfiguration& cfg);

/// 读指令扩展覆盖：ikProfile / ikRankMode / ikBudgetMs / allowCartDegrade / playbackPlanLite
ROBOT_SCENE_API void applyIkPolicyExtensionOverrides(const std::unordered_map<std::string, std::string>& ext,
													  IkSelectionPolicy& policy);

/// 按机型收紧：非球形腕关竞速/贴种子；球形腕仅扩展显式打开
ROBOT_SCENE_API void adaptIkPolicyForRobot(const std::string& urdfPath, IkSelectionPolicy& policy,
											const std::unordered_map<std::string, std::string>* ext = nullptr);

/// 规划 summary 短标记（排障）
ROBOT_SCENE_API std::string formatIkPolicySummaryTags(const IkSelectionPolicy& policy, bool denseRetry, bool numRace,
													  bool cartDegraded);

/// 解析 context.elbowJointIndex / wristJointIndex（十进制，失败保持 -1）
ROBOT_SCENE_API void parseIkJointIndexOverrides(const std::unordered_map<std::string, std::string>& ext,
												 int& elbowJointIndex, int& wristJointIndex);

/// 相对种子分类后构型变化则跳过 ±2π 最短折圈（保留 IK 转数）
ROBOT_SCENE_API bool shouldSkipShortestAngleFold(const std::vector<double>& seedQ,
												 const std::vector<double>& targetQ,
												 const std::vector<std::string>& jointNames);

/// 兼容旧名：比较已分类结果
inline bool ptpShouldSkipShortestAngleFold(const JointConfigurationClass& seedCls,
										   const JointConfigurationClass& targetCls)
{
	return !sameJointConfigurationClass(seedCls, targetCls);
}

/// 相对种子的腕区跳变（J4/J5/J6 最短角差的最大者，rad；已对 2π 折同构）
/// 邻点连续：小于约 π/2 视为未翻腕；球腕 J4/J6 对调通常 ≥π
ROBOT_SCENE_API double wristConfigurationJumpRad(const std::vector<double>& qRad, const std::vector<double>& seedRad,
												 const std::vector<std::string>& jointNames);

/// 腕连续门限（rad）：超过则视为跳腕支
inline constexpr double kWristContinuityGateRad = 1.5707963267948966; // π/2

/// 从观测姿态反推最具体 preset（可能为 AUTO）
ROBOT_SCENE_API std::string suggestMotionAxisPresetToken(const JointConfigurationClass& observed);

} // namespace RobotInstruction

#endif // ROBOTSCENE_ROBOTINSTRUCTIONAXISCONFIGURATION_H
