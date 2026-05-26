#pragma once

#include "RobotInstructionModel.h"
#include "TrajectoryPipelineTypes.h"
#include "robot_scene_global.h"

#include <json.hpp>

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace RobotInstruction
{

inline constexpr const char* kDefaultMainProgramId = "main";

/// 路点分组元数据，不插入程序树
struct ROBOT_SCENE_API InstructionGroup
{
	std::string id;
	std::string name;
	std::vector<std::string> memberInstructionIds;
};

struct ROBOT_SCENE_API RobotProgram
{
	std::string id;
	std::string name;
	bool isMain = false;
	std::vector<std::shared_ptr<Base>> steps;
	std::vector<InstructionGroup> groups;
};

class ROBOT_SCENE_API RobotProgramCatalog
{
public:
	static RobotProgramCatalog withDefaultMain();

	std::string activeProgramId() const { return m_activeProgramId; }
	void setActiveProgramId(const std::string& programId);

	RobotProgram* findProgram(const std::string& programId);
	const RobotProgram* findProgram(const std::string& programId) const;
	RobotProgram* mainProgram();
	const RobotProgram* mainProgram() const;

	InstructionGroup* findGroup(const std::string& groupId);
	const InstructionGroup* findGroup(const std::string& groupId) const;
	InstructionGroup* findGroupInProgram(const std::string& programId, const std::string& groupId);

	std::vector<const Base*> resolveGroupMembers(const InstructionGroup& group, const RobotProgram& prog) const;
	std::vector<std::string> resolveOpScopeInstructionIds(const OpScope& scope, const RobotProgram& prog) const;
	std::vector<std::string> expandToMotionWaypointIds(
		const RobotProgram& prog,
		const std::vector<std::string>& instructionIds) const;

	bool addProgram(RobotProgram program, std::string* errMsg = nullptr);
	bool removeProgram(const std::string& programId, std::string* errMsg = nullptr);
	bool renameProgram(const std::string& programId, const std::string& newName, std::string* errMsg = nullptr);

	std::vector<RobotProgram>& programs() { return m_programs; }
	const std::vector<RobotProgram>& programs() const { return m_programs; }

	std::vector<std::shared_ptr<Base>>& activeSteps();
	const std::vector<std::shared_ptr<Base>>& activeSteps() const;

	void pruneGroupMembers(const std::string& programId, const std::unordered_set<std::string>& removedIds);

	nlohmann::json toJson() const;
	static bool fromJson(const nlohmann::json& j, RobotProgramCatalog& out, std::string* errMsg = nullptr);

private:
	RobotProgram* findProgramMutable(const std::string& programId);

	std::string m_activeProgramId = kDefaultMainProgramId;
	std::vector<RobotProgram> m_programs;
};

ROBOT_SCENE_API std::string makeGroupId();
ROBOT_SCENE_API std::string makeProgramId();

} // namespace RobotInstruction
