#include "RobotProgramCatalog.h"

#include "InstructionProgramDocument.h"
#include "RobotInstructionFactory.h"
#include "RobotInstructionModel.h"
#include "RobotInstructionProgram.h"

#include <algorithm>
#include <chrono>
#include <random>
#include <unordered_map>
#include <unordered_set>

namespace RobotInstruction
{
namespace
{
std::string makeUniqueToken()
{
	const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
	static std::mt19937 rng(static_cast<unsigned>(now));
	std::uniform_int_distribution<int> dist(100000, 999999);
	return std::to_string(now) + "_" + std::to_string(dist(rng));
}
} // namespace

std::string makeGroupId()
{
	return "grp_" + makeUniqueToken();
}

std::string makeProgramId()
{
	return "prog_" + makeUniqueToken();
}

RobotProgramCatalog RobotProgramCatalog::withDefaultMain()
{
	RobotProgramCatalog catalog;
	RobotProgram mainProg;
	mainProg.id = kDefaultMainProgramId;
	mainProg.name = "Main";
	mainProg.isMain = true;
	catalog.m_programs.push_back(std::move(mainProg));
	catalog.m_activeProgramId = kDefaultMainProgramId;
	return catalog;
}

void RobotProgramCatalog::setActiveProgramId(const std::string& programId)
{
	if (findProgram(programId))
	{
		m_activeProgramId = programId;
	}
}

RobotProgram* RobotProgramCatalog::findProgram(const std::string& programId)
{
	return findProgramMutable(programId);
}

const RobotProgram* RobotProgramCatalog::findProgram(const std::string& programId) const
{
	for (const auto& prog : m_programs)
	{
		if (prog.id == programId)
		{
			return &prog;
		}
	}
	return nullptr;
}

RobotProgram* RobotProgramCatalog::mainProgram()
{
	if (RobotProgram* p = findProgramMutable(kDefaultMainProgramId))
	{
		return p;
	}
	for (auto& prog : m_programs)
	{
		if (prog.isMain)
		{
			return &prog;
		}
	}
	return m_programs.empty() ? nullptr : &m_programs.front();
}

const RobotProgram* RobotProgramCatalog::mainProgram() const
{
	return const_cast<RobotProgramCatalog*>(this)->mainProgram();
}

InstructionGroup* RobotProgramCatalog::findGroup(const std::string& groupId)
{
	return findGroupInProgram(m_activeProgramId, groupId);
}

const InstructionGroup* RobotProgramCatalog::findGroup(const std::string& groupId) const
{
	return const_cast<RobotProgramCatalog*>(this)->findGroup(groupId);
}

InstructionGroup* RobotProgramCatalog::findGroupInProgram(
	const std::string& programId,
	const std::string& groupId)
{
	RobotProgram* prog = findProgramMutable(programId);
	if (!prog)
	{
		return nullptr;
	}
	for (auto& group : prog->groups)
	{
		if (group.id == groupId)
		{
			return &group;
		}
	}
	return nullptr;
}

std::vector<const Base*> RobotProgramCatalog::resolveGroupMembers(
	const InstructionGroup& group,
	const RobotProgram& prog) const
{
	std::unordered_map<std::string, std::shared_ptr<Base>> idMap;
	InstructionProgramDocument::collectIdMapRecursive(prog.steps, idMap);
	std::vector<const Base*> out;
	out.reserve(group.memberInstructionIds.size());
	for (const std::string& memberId : group.memberInstructionIds)
	{
		const auto it = idMap.find(memberId);
		if (it != idMap.end() && it->second && isMotionWaypointType(it->second->type()))
		{
			out.push_back(it->second.get());
		}
	}
	return out;
}

std::vector<std::string> RobotProgramCatalog::resolveOpScopeInstructionIds(
	const OpScope& scope,
	const RobotProgram& prog) const
{
	std::vector<std::string> out;
	if (scope.kind == OpScope::Kind::EntireProgram)
	{
		for (const Base* motion : collectMotionInstructions(prog.steps))
		{
			if (motion)
			{
				out.push_back(motion->id());
			}
		}
		return out;
	}
	if (scope.kind == OpScope::Kind::Group)
	{
		for (const auto& group : prog.groups)
		{
			if (group.id == scope.groupId)
			{
				out = group.memberInstructionIds;
				break;
			}
		}
		return out;
	}
	if (scope.kind == OpScope::Kind::InstructionIds)
	{
		return scope.instructionIds;
	}
	if (scope.kind == OpScope::Kind::PointIndexRange)
	{
		const int from = std::max(1, scope.pointFrom);
		const int to = std::max(from, scope.pointTo);
		for (const Base* motion : collectMotionInstructions(prog.steps))
		{
			if (!motion)
			{
				continue;
			}
			const int idx = motionPointIndex(*motion);
			if (idx >= from && idx <= to)
			{
				out.push_back(motion->id());
			}
		}
	}
	return out;
}

namespace
{
void collectMotionIdsFromSteps(
	const std::vector<std::shared_ptr<Base>>& steps,
	std::vector<std::string>& out,
	std::unordered_set<std::string>& seen)
{
	for (const std::shared_ptr<Base>& ins : steps)
	{
		if (!ins)
		{
			continue;
		}
		if (isMotionWaypointType(ins->type()))
		{
			if (seen.insert(ins->id()).second)
			{
				out.push_back(ins->id());
			}
			continue;
		}
		if (ins->type() == Type::IF)
		{
			const auto* ifIns = dynamic_cast<const IfInstruction*>(ins.get());
			if (ifIns)
			{
				collectMotionIdsFromSteps(ifIns->nestedSteps(), out, seen);
				collectMotionIdsFromSteps(ifIns->elseSteps(), out, seen);
			}
		}
		else if (ins->type() == Type::WHILE)
		{
			collectMotionIdsFromSteps(ins->nestedSteps(), out, seen);
		}
	}
}

void collectMotionIdsFromInstructionTree(
	const Base& ins,
	std::vector<std::string>& out,
	std::unordered_set<std::string>& seen)
{
	if (isMotionWaypointType(ins.type()))
	{
		if (seen.insert(ins.id()).second)
		{
			out.push_back(ins.id());
		}
		return;
	}
	if (ins.type() == Type::IF)
	{
		const auto& ifIns = static_cast<const IfInstruction&>(ins);
		collectMotionIdsFromSteps(ifIns.nestedSteps(), out, seen);
		collectMotionIdsFromSteps(ifIns.elseSteps(), out, seen);
	}
	else if (ins.type() == Type::WHILE)
	{
		collectMotionIdsFromSteps(ins.nestedSteps(), out, seen);
	}
}
} // namespace

std::vector<std::string> RobotProgramCatalog::expandToMotionWaypointIds(
	const RobotProgram& prog,
	const std::vector<std::string>& instructionIds) const
{
	std::unordered_map<std::string, std::shared_ptr<Base>> idMap;
	InstructionProgramDocument::collectIdMapRecursive(prog.steps, idMap);
	std::vector<std::string> out;
	std::unordered_set<std::string> seen;
	out.reserve(instructionIds.size());
	for (const std::string& id : instructionIds)
	{
		const auto it = idMap.find(id);
		if (it == idMap.end() || !it->second)
		{
			continue;
		}
		collectMotionIdsFromInstructionTree(*it->second, out, seen);
	}
	return out;
}

bool RobotProgramCatalog::addProgram(RobotProgram program, std::string* errMsg)
{
	if (program.id.empty())
	{
		program.id = makeProgramId();
	}
	if (findProgram(program.id))
	{
		if (errMsg)
		{
			*errMsg = "program id already exists";
		}
		return false;
	}
	if (program.name.empty())
	{
		program.name = program.id;
	}
	m_programs.push_back(std::move(program));
	return true;
}

bool RobotProgramCatalog::removeProgram(const std::string& programId, std::string* errMsg)
{
	if (programId == kDefaultMainProgramId)
	{
		if (errMsg)
		{
			*errMsg = "cannot remove main program";
		}
		return false;
	}
	const auto it = std::remove_if(
		m_programs.begin(),
		m_programs.end(),
		[&programId](const RobotProgram& p) { return p.id == programId; });
	if (it == m_programs.end())
	{
		if (errMsg)
		{
			*errMsg = "program not found";
		}
		return false;
	}
	m_programs.erase(it, m_programs.end());
	if (m_activeProgramId == programId)
	{
		m_activeProgramId = kDefaultMainProgramId;
	}
	return true;
}

bool RobotProgramCatalog::renameProgram(
	const std::string& programId,
	const std::string& newName,
	std::string* errMsg)
{
	RobotProgram* prog = findProgramMutable(programId);
	if (!prog)
	{
		if (errMsg)
		{
			*errMsg = "program not found";
		}
		return false;
	}
	if (newName.empty())
	{
		if (errMsg)
		{
			*errMsg = "empty program name";
		}
		return false;
	}
	prog->name = newName;
	return true;
}

std::vector<std::shared_ptr<Base>>& RobotProgramCatalog::activeSteps()
{
	RobotProgram* prog = findProgramMutable(m_activeProgramId);
	if (!prog)
	{
		prog = mainProgram();
	}
	static std::vector<std::shared_ptr<Base>> s_empty;
	return prog ? prog->steps : s_empty;
}

const std::vector<std::shared_ptr<Base>>& RobotProgramCatalog::activeSteps() const
{
	return const_cast<RobotProgramCatalog*>(this)->activeSteps();
}

void RobotProgramCatalog::pruneGroupMembers(
	const std::string& programId,
	const std::unordered_set<std::string>& removedIds)
{
	RobotProgram* prog = findProgramMutable(programId);
	if (!prog || removedIds.empty())
	{
		return;
	}
	for (auto& group : prog->groups)
	{
		group.memberInstructionIds.erase(
			std::remove_if(
				group.memberInstructionIds.begin(),
				group.memberInstructionIds.end(),
				[&removedIds](const std::string& id) { return removedIds.count(id) != 0; }),
			group.memberInstructionIds.end());
	}
}

RobotProgram* RobotProgramCatalog::findProgramMutable(const std::string& programId)
{
	for (auto& prog : m_programs)
	{
		if (prog.id == programId)
		{
			return &prog;
		}
	}
	return nullptr;
}

nlohmann::json RobotProgramCatalog::toJson() const
{
	nlohmann::json j;
	j["activeProgramId"] = m_activeProgramId;
	nlohmann::json programsArr = nlohmann::json::array();
	for (const RobotProgram& prog : m_programs)
	{
		nlohmann::json pj;
		pj["id"] = prog.id;
		pj["name"] = prog.name;
		pj["isMain"] = prog.isMain;
		nlohmann::json insArr = nlohmann::json::array();
		for (const auto& ins : prog.steps)
		{
			if (ins)
			{
				insArr.push_back(RobotInstruction::toJson(*ins));
			}
		}
		pj["instructions"] = insArr;
		nlohmann::json groupsArr = nlohmann::json::array();
		for (const InstructionGroup& group : prog.groups)
		{
			nlohmann::json gj;
			gj["id"] = group.id;
			gj["name"] = group.name;
			gj["memberIds"] = group.memberInstructionIds;
			groupsArr.push_back(gj);
		}
		pj["groups"] = groupsArr;
		programsArr.push_back(pj);
	}
	j["programs"] = programsArr;
	return j;
}

bool RobotProgramCatalog::fromJson(const nlohmann::json& j, RobotProgramCatalog& out, std::string* errMsg)
{
	out = RobotProgramCatalog{};
	if (j.contains("activeProgramId") && j["activeProgramId"].is_string())
	{
		out.m_activeProgramId = j["activeProgramId"].get<std::string>();
	}
	if (j.contains("programs") && j["programs"].is_array())
	{
		for (const auto& pj : j["programs"])
		{
			if (!pj.is_object())
			{
				continue;
			}
			RobotProgram prog;
			prog.id = pj.value("id", kDefaultMainProgramId);
			prog.name = pj.value("name", prog.id);
			prog.isMain = pj.value("isMain", prog.id == kDefaultMainProgramId);
			if (pj.contains("instructions") && pj["instructions"].is_array())
			{
				prog.steps = createListFromJson(pj["instructions"], errMsg);
				if (errMsg && !errMsg->empty())
				{
					return false;
				}
			}
			if (pj.contains("groups") && pj["groups"].is_array())
			{
				for (const auto& gj : pj["groups"])
				{
					if (!gj.is_object())
					{
						continue;
					}
					InstructionGroup group;
					group.id = gj.value("id", makeGroupId());
					group.name = gj.value("name", group.id);
					if (gj.contains("memberIds") && gj["memberIds"].is_array())
					{
						for (const auto& mid : gj["memberIds"])
						{
							if (mid.is_string())
							{
								group.memberInstructionIds.push_back(mid.get<std::string>());
							}
						}
					}
					prog.groups.push_back(std::move(group));
				}
			}
			out.m_programs.push_back(std::move(prog));
		}
	}
	if (out.m_programs.empty())
	{
		out = withDefaultMain();
	}
	if (!out.findProgram(out.m_activeProgramId))
	{
		out.m_activeProgramId = kDefaultMainProgramId;
	}
	return true;
}

} // namespace RobotInstruction
