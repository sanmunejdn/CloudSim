#ifndef ROBOTSCENE_ROBOTINSTRUCTIONCONTROLLER_H
#define ROBOTSCENE_ROBOTINSTRUCTIONCONTROLLER_H

/// @file RobotInstructionController.h
/// @brief 当前指令上下文中通过 IK+姿态约束的轴配置枚举

#include "robot_scene_global.h"

#include "RobotExternalAxes.h"
#include "RobotInstructionModel.h"
#include "SerialLinkKinematics.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace RobotInstruction
{
struct ROBOT_SCENE_API PlanResult
{
	bool ok = false;
	std::string plannerName;
	std::string summary;
	double durationSec = 0.0;
	std::vector<double> jointTargetsRad;
	std::vector<std::vector<double>> jointTrajectoryRad;
	bool hasExternalAxisQ = false;
	/// 首轴兼容（首个启用/RobotBase）
	double externalAxisQ = 0.0;
	/// 完整配置下标对齐；写计划时与 externalAxisQ 同时填充
	std::vector<double> externalAxisQs;
};

/// 当前指令上下文中通过 IK+姿态约束的轴配置枚举
struct ROBOT_SCENE_API FeasibleMotionAxisConfigurationOptions
{
	std::vector<std::string> presetTokens;
	std::vector<std::string> elbowTokens;
	std::vector<std::string> wristTokens;
	std::vector<std::string> armTokens;
	std::vector<std::string> turnJ1Tokens;
	std::vector<std::string> turnJ4Tokens;
	std::vector<std::string> turnJ6Tokens;
};

class ROBOT_SCENE_API PlannerBase
{
public:
	virtual ~PlannerBase() = default;
	virtual bool canHandle(Type type) const = 0;
	virtual bool validate(const Base& cmd, std::string* errMsg) const = 0;
	virtual bool plan(const Base& cmd, PlanResult& out, std::string* errMsg) const = 0;
};

class ROBOT_SCENE_API Controller
{
public:
	/// REP：规划时将工件工作架变换到机器人 P0；未设置则跳过工件采样
	struct WorkpieceIkFrameContext
	{
		bool valid = false;
		std::array<double, 16> p0World{};
		std::string boundBackendId;
		std::array<double, 16> w0World{};
		std::array<double, 16> offsetW0Local{};
	};

	void setDhRows(const std::vector<robot_kinematics::DhRow>& rows);
	void clearDhRows();
	bool hasDhRows() const;

	void setExternalAxes(const RobotExternal::RobotExternalAxisConfigSet& axes);
	void clearExternalAxes();
	bool hasEnabledExternalAxes() const;

	void setWorkpieceIkFrameContext(const WorkpieceIkFrameContext& ctx);
	void clearWorkpieceIkFrameContext();
	const WorkpieceIkFrameContext& workpieceIkFrameContext() const { return m_workpieceIkFrame; }

	void registerPlanner(const std::shared_ptr<PlannerBase>& planner);
	void clearPlanners();
	void buildDefaultPlanners();

	bool validate(const Base& cmd, std::string* errMsg) const;
	bool plan(const Base& cmd, PlanResult& out, std::string* errMsg) const;

	/// 可规划/启动的轴配置选项（cmd 须带规划上下文）
	FeasibleMotionAxisConfigurationOptions queryFeasibleMotionAxisConfigurationOptions(const Base& cmd) const;

private:
	const PlannerBase* findPlanner(Type t) const;

private:
	std::vector<robot_kinematics::DhRow> m_dhRows;
	RobotExternal::RobotExternalAxisConfigSet m_externalAxes;
	WorkpieceIkFrameContext m_workpieceIkFrame{};
	std::vector<std::shared_ptr<PlannerBase>> m_planners;
};
} // namespace RobotInstruction

#endif // ROBOTSCENE_ROBOTINSTRUCTIONCONTROLLER_H
