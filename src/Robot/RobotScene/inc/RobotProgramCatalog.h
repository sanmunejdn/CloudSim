#ifndef ROBOTSCENE_ROBOTPROGRAMCATALOG_H
#define ROBOTSCENE_ROBOTPROGRAMCATALOG_H

/// @file RobotProgramCatalog.h
/// @brief 路点分组元数据，不插入程序树

#include "robot_scene_global.h"

#include "RawTrajectory.h"
#include "RobotInstructionModel.h"
#include "TrajectoryPipelineTypes.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <json.hpp>

namespace RobotInstruction
{
inline constexpr const char* kDefaultMainProgramId = "main";
inline constexpr int kRobotProgramCatalogSchemaVersion = 1;

enum class ROBOT_SCENE_API InstructionGroupRole
{
	Generic = 0,
	PathPlanOutput
};

/// 路点分组元数据，不插入程序树
struct ROBOT_SCENE_API InstructionGroup
{
	std::string id;
	std::string name;
	InstructionGroupRole role = InstructionGroupRole::Generic;
	std::string pathPlanInstructionId;
	std::vector<std::string> memberInstructionIds;
};

/// 按 PathPlan 指令 id 存储离散 raw（与 session 槽位解耦）
struct ROBOT_SCENE_API PathPlanRawStore
{
	bool save(const std::string& pathPlanId, const RawTrajectory& raw);
	bool load(const std::string& pathPlanId, RawTrajectory& out) const;
	bool remove(const std::string& pathPlanId);
	bool contains(const std::string& pathPlanId) const;
	void clear();

	const std::unordered_map<std::string, RawTrajectory>& entries() const { return m_entries; }

	nlohmann::json toJson() const;
	static bool fromJson(const nlohmann::json& j, PathPlanRawStore& out, std::string* errMsg = nullptr);

private:
	std::unordered_map<std::string, RawTrajectory> m_entries;
};

struct ROBOT_SCENE_API RobotProgram
{
	std::string id;
	std::string name;
	bool isMain = false;
	std::vector<std::shared_ptr<Base>> steps;
	std::vector<InstructionGroup> groups;
};

ROBOT_SCENE_API bool emitRawTrajectoryToProgram(const RawTrajectory& trajectory, RobotProgram& program,
												std::string* errMsg = nullptr, std::string* outGroupId = nullptr,
												const std::string* pathPlanInstructionId = nullptr);

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
	std::vector<std::string> expandToMotionWaypointIds(const RobotProgram& prog,
													   const std::vector<std::string>& instructionIds) const;

	bool addProgram(RobotProgram program, std::string* errMsg = nullptr);
	bool removeProgram(const std::string& programId, std::string* errMsg = nullptr);
	bool renameProgram(const std::string& programId, const std::string& newName, std::string* errMsg = nullptr);

	std::vector<RobotProgram>& programs() { return m_programs; }
	const std::vector<RobotProgram>& programs() const { return m_programs; }

	std::vector<std::shared_ptr<Base>>& activeSteps();
	const std::vector<std::shared_ptr<Base>>& activeSteps() const;

	void pruneGroupMembers(const std::string& programId, const std::unordered_set<std::string>& removedIds);
	void prunePathPlanReferences(const std::string& programId,
								 const std::unordered_set<std::string>& removedPathPlanIds);
	PathPlanInstruction* findPathPlan(const std::string& programId, const std::string& pathPlanId);
	const PathPlanInstruction* findPathPlan(const std::string& programId, const std::string& pathPlanId) const;
	std::vector<PathPlanInstruction*> listPathPlans(const std::string& programId);
	InstructionGroup* findPathPlanOutputGroup(RobotProgram& prog, const std::string& pathPlanId);

	PathPlanRawStore& pathPlanRaws() { return m_pathPlanRaws; }
	const PathPlanRawStore& pathPlanRaws() const { return m_pathPlanRaws; }

	nlohmann::json toJson() const;
	static bool fromJson(const nlohmann::json& j, RobotProgramCatalog& out, std::string* errMsg = nullptr);

private:
	RobotProgram* findProgramMutable(const std::string& programId);

	std::string m_activeProgramId = kDefaultMainProgramId;
	std::vector<RobotProgram> m_programs;
	PathPlanRawStore m_pathPlanRaws;
};

ROBOT_SCENE_API std::string makeGroupId();
ROBOT_SCENE_API std::string makeProgramId();

/// 旧工程无 PathPlan 时补一条默认规划指令并关联首个有成员的分组
ROBOT_SCENE_API void migrateLegacyPathPlans(RobotProgramCatalog& catalog);

} // namespace RobotInstruction

#endif // ROBOTSCENE_ROBOTPROGRAMCATALOG_H
