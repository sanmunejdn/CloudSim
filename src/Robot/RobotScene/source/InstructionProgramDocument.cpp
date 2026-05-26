#include "InstructionProgramDocument.h"

#include "RobotInstructionProgram.h"

namespace RobotInstruction
{
InstructionProgramDocument::InstructionProgramDocument(std::vector<std::shared_ptr<Base>>* rootSteps)
	: m_root(rootSteps)
{
}

Base* InstructionProgramDocument::findById(const std::string& instructionId) const
{
	if (!m_root)
	{
		return nullptr;
	}
	return findRecursive(*m_root, instructionId);
}

bool InstructionProgramDocument::removeById(const std::string& instructionId)
{
	if (!m_root)
	{
		return false;
	}
	return removeRecursive(*m_root, instructionId);
}

bool InstructionProgramDocument::insertAtRoot(const size_t index, std::shared_ptr<Base> instruction)
{
	if (!m_root || !instruction)
	{
		return false;
	}
	const size_t idx = std::min(index, m_root->size());
	m_root->insert(m_root->begin() + static_cast<std::ptrdiff_t>(idx), std::move(instruction));
	return true;
}

bool InstructionProgramDocument::appendToRoot(std::shared_ptr<Base> instruction)
{
	if (!m_root || !instruction)
	{
		return false;
	}
	m_root->push_back(std::move(instruction));
	return true;
}

void InstructionProgramDocument::collectIdMap(std::unordered_map<std::string, std::shared_ptr<Base>>& out) const
{
	out.clear();
	if (!m_root)
	{
		return;
	}
	collectIdMapRecursive(*m_root, out);
}

void InstructionProgramDocument::renumberAndNotify()
{
	if (m_root)
	{
		renumberMotionPointIndices(*m_root);
	}
}

void InstructionProgramDocument::collectIdMapRecursive(
	const std::vector<std::shared_ptr<Base>>& steps,
	std::unordered_map<std::string, std::shared_ptr<Base>>& out)
{
	for (const auto& ins : steps)
	{
		if (!ins)
		{
			continue;
		}
		out[ins->id()] = ins;
		if (ins->type() == Type::IF)
		{
			collectIdMapRecursive(ins->nestedSteps(), out);
			collectIdMapRecursive(ins->elseSteps(), out);
		}
		else if (ins->type() == Type::WHILE)
		{
			collectIdMapRecursive(ins->nestedSteps(), out);
		}
	}
}

bool InstructionProgramDocument::removeRecursive(std::vector<std::shared_ptr<Base>>& steps, const std::string& id)
{
	for (auto it = steps.begin(); it != steps.end(); ++it)
	{
		if (*it && (*it)->id() == id)
		{
			steps.erase(it);
			return true;
		}
	}
	for (const auto& ins : steps)
	{
		if (!ins)
		{
			continue;
		}
		if (ins->type() == Type::IF)
		{
			if (auto* ifIns = dynamic_cast<IfInstruction*>(ins.get()))
			{
				if (removeRecursive(ifIns->thenSteps(), id) || removeRecursive(ifIns->elseStepsMut(), id))
				{
					return true;
				}
			}
		}
		else if (ins->type() == Type::WHILE)
		{
			if (auto* whileIns = dynamic_cast<WhileInstruction*>(ins.get()))
			{
				if (removeRecursive(whileIns->bodySteps(), id))
				{
					return true;
				}
			}
		}
	}
	return false;
}

Base* InstructionProgramDocument::findRecursive(
	const std::vector<std::shared_ptr<Base>>& steps,
	const std::string& id)
{
	for (const auto& ins : steps)
	{
		if (!ins)
		{
			continue;
		}
		if (ins->id() == id)
		{
			return ins.get();
		}
		if (ins->type() == Type::IF)
		{
			if (Base* found = findRecursive(ins->nestedSteps(), id))
			{
				return found;
			}
			if (Base* found = findRecursive(ins->elseSteps(), id))
			{
				return found;
			}
		}
		else if (ins->type() == Type::WHILE)
		{
			if (Base* found = findRecursive(ins->nestedSteps(), id))
			{
				return found;
			}
		}
	}
	return nullptr;
}

} // namespace RobotInstruction
