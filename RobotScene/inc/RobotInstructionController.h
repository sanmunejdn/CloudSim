#pragma once

#include "RobotInstructionModel.h"
#include "robot_scene_global.h"

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
	double durationSec = 0.0; // Planned execution time for this segment.
	std::vector<double> jointTargetsRad; // Final joint target for this instruction segment.
	std::vector<std::vector<double>> jointTrajectoryRad; // Optional absolute trajectory waypoints.
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

	void registerPlanner(const std::shared_ptr<PlannerBase>& planner);
	void clearPlanners();
	void buildDefaultPlanners();

	bool validate(const Base& cmd, std::string* errMsg) const;
	bool plan(const Base& cmd, PlanResult& out, std::string* errMsg) const;

private:
	const PlannerBase* findPlanner(Type t) const;

private:
	std::vector<robot_kinematics::DhRow> m_dhRows;
	std::vector<std::shared_ptr<PlannerBase>> m_planners;
};
} // namespace RobotInstruction
