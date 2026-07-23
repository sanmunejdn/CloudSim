#ifndef ROBOTSCENE_ROBOTINSTRUCTIONCONTROLLER_H
#define ROBOTSCENE_ROBOTINSTRUCTIONCONTROLLER_H

/// @file RobotInstructionController.h
/// @brief 当前指令上下文中通过 IK+姿态约束的轴配置枚举

#include "robot_scene_global.h"

#include "RobotExternalAxes.h"
#include "RobotInstructionModel.h"
#include "SerialLinkKinematics.h"

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
	double externalAxisQ = 0.0;
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
	void setDhRows(const std::vector<robot_kinematics::DhRow>& rows);
	void clearDhRows();
	bool hasDhRows() const;

	void setExternalAxes(const RobotExternal::RobotExternalAxisConfigSet& axes);
	void clearExternalAxes();
	bool hasEnabledExternalAxes() const;

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
	std::vector<std::shared_ptr<PlannerBase>> m_planners;
};
} // namespace RobotInstruction

#endif // ROBOTSCENE_ROBOTINSTRUCTIONCONTROLLER_H
